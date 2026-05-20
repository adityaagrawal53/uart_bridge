#!/usr/bin/env python3
"""
latency_pdr_node.py
===================
Benchmarks robot-to-robot communication under two transport conditions:

  transport:=cyclonedds  (baseline)
    Robot subscribes to real /amcl_pose and /scan, publishes on /<robot_id>/ping.
    Peer robot subscribes to that topic and the measurement is one-way latency.

  transport:=mixer  (Mixer/DPP condition)
    Robot subscribes to real /amcl_pose and /scan, writes packed bytes to the # TODO:
    DPP board over USB serial.  The DPP board transmits via Mixer (802.15.4)
    to the peer DPP, which forwards bytes back to the peer RPi4.  The peer
    RPi4 teammate node echoes those bytes back over DDS so that THIS node can
    receive them and measure round-trip latency.

Serial framing contract (agreed with DPP teammate)
---------------------------------------------------
  - Fixed-size packets only (no length header needed)
  - This node writes the packed payload bytes to serial
  - DPP echoes the EXACT same bytes back unchanged
  - Sequence number and timestamps are owned by this node

Metrics
-------
  Latency     one-way (CycloneDDS) or RTT/2 (Mixer)            ms
  Jitter      running std-dev of latency samples                ms
  Packet loss received / transmitted                            %
  CPU         per-sample RPi4 CPU % via psutil background thread
  Bandwidth   cumulative bytes/s on WiFi interface

Parameters
----------
  robot_id          str       'robot1'
  other_robot_ids   list[str] ['robot2']
  ping_rate_hz      float     10.0 # TODO: change to 5 for Mixer purposes?
  transport         str       'cyclonedds'  |  'mixer'
  payload_type      str       'pose'        |  'lidar'
  serial_port       str       '/dev/ttyUSB1'   # changed
  serial_baud       int       460800          # changed
  wifi_iface        str       'wlan0'
  log_dir           str       '/home/ubuntu/measurements'
"""

import math
import os
import csv
import struct # COMMENT: converts the data to C structs that should (in theory) be easy to read on the DPP side since it's coded in C (and lots of vibes)
import threading
import time
from datetime import datetime

# ROS2 dependencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

try:
    import psutil
    _PSUTIL = True
except ImportError:
    _PSUTIL = False

try:
    import serial
    _SERIAL = True
except ImportError:
    _SERIAL = False

# ---------------------------------------------------------------------------
# Payload formats
#
#   POSE:  tag(B) seq(Q) ts_ns(Q) x y z qx qy qz qw (7d)  ->   89 bytes
#   LIDAR: tag(B) seq(Q) ts_ns(Q) ranges*360 (360f)        -> 1457 bytes
#
# Big-endian so byte order is unambiguous on both RPi4 and DPP board.
# TODO: Is this payload format specific to the Mixer/DPP, or is it also what is used to format the
# the ROS2 data? I was under the impression that the ROS2 data was sent as std. topics over cyclone_dds...
# ---------------------------------------------------------------------------

POSE_TAG  = 0x01
LIDAR_TAG = 0x02

POSE_FMT  = '>BQQ7d'
LIDAR_FMT = '>BQQ360f'

POSE_SIZE  = struct.calcsize(POSE_FMT)   # 89  bytes # TODO: chatgpt says this is actually 73 bytes, unless padding is involved. conspiracy?
LIDAR_SIZE = struct.calcsize(LIDAR_FMT)  # 1457 bytes


def _read_iface_bytes(iface: str) -> int:
    """Read cumulative RX bytes for a network interface from /proc/net/dev."""
    try:
        with open('/proc/net/dev') as f:
            for line in f:
                if iface in line:
                    return int(line.split()[1])
    except Exception:
        pass
    return 0


# ---------------------------------------------------------------------------

class LatencyPDRNode(Node):

    def __init__(self):
        super().__init__('latency_pdr_node')

        # ------------------------------------------------------------------ #
        # Parameters                                                           #
        # ------------------------------------------------------------------ #
        self.declare_parameter('robot_id',        'robot1')
        self.declare_parameter('other_robot_ids', ['robot2'])
        self.declare_parameter('ping_rate_hz',    10.0) # TODO: Change later?
        self.declare_parameter('transport',       'cyclonedds')
        self.declare_parameter('payload_type',    'pose')
        self.declare_parameter('serial_port',     '/dev/ttyUSB1') # TODO: changed
        self.declare_parameter('serial_baud',     460800) # TODO: changed
        self.declare_parameter('wifi_iface',      'wlan0')
        self.declare_parameter('log_dir',         '/home/ubuntu/measurements')

        self.robot_id     = self.get_parameter('robot_id').value
        self.other_ids    = self.get_parameter('other_robot_ids').value
        self.transport    = self.get_parameter('transport').value
        self.payload_type = self.get_parameter('payload_type').value
        self.serial_port  = self.get_parameter('serial_port').value
        self.serial_baud  = self.get_parameter('serial_baud').value
        self.wifi_iface   = self.get_parameter('wifi_iface').value
        self.log_dir      = self.get_parameter('log_dir').value

        self._validate_params()

        # ------------------------------------------------------------------ #
        # Latest sensor data                                                   #
        # ------------------------------------------------------------------ #
        self._latest_pose: tuple | None   = None   # (x,y,z,qx,qy,qz,qw)
        self._latest_ranges: tuple | None = None   # 360 floats

        # ------------------------------------------------------------------ #
        # Per-sender measurement state                                         #
        # ------------------------------------------------------------------ #
        self.seq = 0

        # CycloneDDS: one sender per peer robot
        # Mixer:      single logical sender 'dpp' (round-trip echo)
        self._senders = self.other_ids if self.transport == 'cyclonedds' else ['dpp']

        self.received_seqs: dict[str, set]   = {s: set() for s in self._senders}
        self.expected_seqs: dict[str, int]   = {s: -1    for s in self._senders}
        self._lat_n:        dict[str, int]   = {s: 0     for s in self._senders}
        self._lat_mean:     dict[str, float] = {s: 0.0   for s in self._senders}
        self._lat_M2:       dict[str, float] = {s: 0.0   for s in self._senders}

        # Mixer only: seq -> send_time_ns for RTT measurement
        self._send_times: dict[int, int] = {}

        # ------------------------------------------------------------------ #
        # CPU monitoring (background thread)                                  #
        # ------------------------------------------------------------------ #
        self._cpu_samples: list[float] = []
        self._cpu_lock = threading.Lock()
        if _PSUTIL:
            threading.Thread(target=self._cpu_monitor, daemon=True).start()
        else:
            self.get_logger().warn("psutil not installed — CPU will read 0.0")

        # ------------------------------------------------------------------ #
        # Bandwidth baseline                                                   #
        # ------------------------------------------------------------------ #
        self._bw_start_bytes = _read_iface_bytes(self.wifi_iface)
        self._bw_start_time  = time.monotonic()

        # ------------------------------------------------------------------ #
        # Transport setup                                                      #
        # ------------------------------------------------------------------ #
        self._ser = None

        if self.transport == 'mixer':
            self._setup_serial()
        else:
            self._setup_cyclonedds()

        # ------------------------------------------------------------------ #
        # Sensor subscribers (both transports)                                #
        # ------------------------------------------------------------------ #
        self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{self.robot_id}/amcl_pose',
            self._amcl_callback,
            10 # TODO: is this hertz? if so, change
        )
        self.create_subscription(
            LaserScan,
            f'/{self.robot_id}/scan',
            self._scan_callback,
            10  # TODO: is this hertz? if so, change
        )

        # ------------------------------------------------------------------ #
        # CSV                                                                  #
        # ------------------------------------------------------------------ #
        os.makedirs(self.log_dir, exist_ok=True)
        ts    = datetime.now().strftime('%Y%m%d_%H%M%S')
        fname = (
            f"{self.log_dir}/"
            f"{self.transport}_{self.payload_type}_{self.robot_id}_{ts}.csv"
        )
        self.csv_file   = open(fname, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'recv_time_ns',
            'send_time_ns',
            'latency_ms',       # one-way for DDS; RTT/2 for Mixer
            # TODO: what does RTT means? round trip as in it recieves its own message back at some point
            # and this can be used to calculated the latency between two nodes on average??
            'jitter_ms',
            'sender_id',
            'seq',
            'payload_type',
            'payload_bytes',
            'transport',
            'cpu_pct',
            'bw_bytes_per_s',
        ])
        self.get_logger().info(
            f"[{self.robot_id}] transport={self.transport} "
            f"payload={self.payload_type}  ->  {fname}"
        )

        # ------------------------------------------------------------------ #
        # Timers                                                               #
        # ------------------------------------------------------------------ #
        period = 1.0 / self.get_parameter('ping_rate_hz').value
        self.create_timer(period, self._publish_ping)
        self.create_timer(10.0,   self._log_summary)

    # ====================================================================== #
    # Validation                                                               #
    # ====================================================================== #

    def _validate_params(self):
        if self.transport not in ('cyclonedds', 'mixer'):
            raise ValueError(
                f"transport must be 'cyclonedds' or 'mixer', got '{self.transport}'"
            )
        if self.payload_type not in ('pose', 'lidar'):
            raise ValueError(
                f"payload_type must be 'pose' or 'lidar', got '{self.payload_type}'"
            )
        if self.transport == 'mixer' and not _SERIAL:
            raise RuntimeError(
                "pyserial not installed — "
                "run: pip install pyserial --break-system-packages"
            )

    # ====================================================================== #
    # Transport setup                                                          #
    # ====================================================================== #

    def _setup_cyclonedds(self):
        """Outbound publisher + one subscriber per peer robot."""
        self.ping_pub = self.create_publisher(
                String, f'/{self.robot_id}/ping', 10 # TODO: hertz?
        )
        for other in self.other_ids:
            self.create_subscription(
                String,
                f'/{other}/ping',
                lambda msg, sid=other: self._dds_ping_callback(msg, sid),
                10 # TODO: hertz?
            )

    def _setup_serial(self):
        """
        Open USB serial port to the DPP board.
        serial_port and serial_baud are placeholder parameters —
        update once confirmed with teammate. 

        A background thread reads echo bytes back from the DPP so that
        the main ROS spin thread is never blocked.
        """
        try:
            self._ser = serial.Serial(
                port     = self.serial_port,
                baudrate = self.serial_baud,
                timeout  = 0.1,
            )
            self.get_logger().info(
                f"Serial opened: {self.serial_port} @ {self.serial_baud} baud"
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Cannot open serial port: {e}")
            self._ser = None
            return

        payload_size = POSE_SIZE if self.payload_type == 'pose' else LIDAR_SIZE
        threading.Thread(
            target=self._serial_read_loop,
            args=(payload_size,),
            daemon=True,
        ).start()

    # ====================================================================== #
    # Sensor callbacks                                                         #
    # ====================================================================== #

    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        self._latest_pose = (
            p.position.x,    p.position.y,    p.position.z,
            p.orientation.x, p.orientation.y,
            p.orientation.z, p.orientation.w,
        )

    def _scan_callback(self, msg: LaserScan):
        # Replace inf/nan with 0.0 (struct.pack cannot handle them)
        ranges = [r if math.isfinite(r) else 0.0 for r in msg.ranges[:360]]
        while len(ranges) < 360:   # pad if scan has fewer than 360 points
            ranges.append(0.0)
        self._latest_ranges = tuple(ranges)

    # ====================================================================== #
    # Payload builders / parsers                                              #
    # ====================================================================== #

    def _build_payload(self, seq: int, now_ns: int) -> bytes:
        if self.payload_type == 'pose':
            pose = self._latest_pose or (0.0,) * 7
            return struct.pack(POSE_FMT, POSE_TAG, seq, now_ns, *pose)
        else:
            ranges = self._latest_ranges or (0.0,) * 360
            return struct.pack(LIDAR_FMT, LIDAR_TAG, seq, now_ns, *ranges)

    def _parse_payload(self, raw: bytes):
        """Returns (tag, seq, send_time_ns) or raises ValueError."""
        if not raw:
            raise ValueError("empty payload")
        tag = raw[0]
        if tag == POSE_TAG:
            if len(raw) < POSE_SIZE:
                raise ValueError(f"pose payload too short ({len(raw)} bytes)")
            fields = struct.unpack_from(POSE_FMT, raw)
            return fields[0], fields[1], fields[2]   # tag, seq, ts
        elif tag == LIDAR_TAG:
            if len(raw) < LIDAR_SIZE:
                raise ValueError(f"lidar payload too short ({len(raw)} bytes)")
            fields = struct.unpack_from(LIDAR_FMT, raw)
            return fields[0], fields[1], fields[2]   # tag, seq, ts
        else:
            raise ValueError(f"unknown tag 0x{tag:02x}")

    # ====================================================================== #
    # Publish / send (timer callback)                                         #
    # ====================================================================== #

    def _publish_ping(self):
        now_ns = self.get_clock().now().nanoseconds # TODO: why is this in ns? ms would be more practical for the purposes of our needs?

        # Block until real sensor data is available
        if self.payload_type == 'pose' and self._latest_pose is None:
            self.get_logger().warn(
                "Waiting for /amcl_pose ...", throttle_duration_sec=5.0
            )
            return
        if self.payload_type == 'lidar' and self._latest_ranges is None:
            self.get_logger().warn(
                "Waiting for /scan ...", throttle_duration_sec=5.0
            )
            return

        raw = self._build_payload(self.seq, now_ns)

        # TODO: this looks like the meat of the entire script
        if self.transport == 'cyclonedds':
            # Hex-encode into String to avoid custom message type
            msg = String()
            msg.data = raw.hex()
            self.ping_pub.publish(msg)

        else:  # mixer
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write(raw)
                    self._send_times[self.seq] = now_ns
                    # Trim to avoid unbounded growth
                    if len(self._send_times) > 500:
                        del self._send_times[min(self._send_times)]
                except serial.SerialException as e:
                    self.get_logger().error(f"Serial write error: {e}")
            else:
                self.get_logger().warn(
                    "Serial port not available", throttle_duration_sec=5.0
                )

        self.seq += 1

    # ====================================================================== #
    # CycloneDDS receive callback                                             #
    # ====================================================================== #

    def _dds_ping_callback(self, msg: String, sender_id: str):
        recv_time_ns = self.get_clock().now().nanoseconds
        try:
            raw = bytes.fromhex(msg.data)
            tag, seq, send_time_ns = self._parse_payload(raw)
        except (ValueError, struct.error) as e:
            self.get_logger().warn(f"Bad DDS payload from {sender_id}: {e}")
            return

        latency_ms = (recv_time_ns - send_time_ns) / 1e6

        if latency_ms < 0:
            self.get_logger().warn(
                f"Negative latency {latency_ms:.2f} ms from {sender_id} "
                f"seq={seq} — check NTP sync"
            )

        self._record_measurement(
            recv_time_ns, send_time_ns, latency_ms,
            sender_id, seq, tag, len(raw)
        )

    # ====================================================================== #
    # Mixer serial receive loop (daemon thread)                               #
    # ====================================================================== #

    def _serial_read_loop(self, payload_size: int):
        """
        Reads exactly payload_size bytes per echo from the DPP board.
        Uses RTT/2 as the one-way latency estimate when the send timestamp
        is available; falls back to the embedded timestamp otherwise.
        """
        self.get_logger().info(
            f"Serial read loop started — expecting {payload_size} bytes per echo"
        )
        buf = b''
        while True:
            try:
                chunk = self._ser.read(payload_size - len(buf))
                if not chunk:
                    continue
                buf += chunk
                if len(buf) < payload_size:
                    continue   # wait for rest of frame

                raw = buf[:payload_size]
                buf = buf[payload_size:]   # keep any overflow

                recv_time_ns = self.get_clock().now().nanoseconds

                try:
                    tag, seq, send_time_ns = self._parse_payload(raw)
                except (ValueError, struct.error) as e:
                    self.get_logger().warn(f"Bad serial echo: {e}")
                    buf = b''   # reset framing
                    continue

                if seq in self._send_times:
                    rtt_ms     = (recv_time_ns - self._send_times.pop(seq)) / 1e6
                    latency_ms = rtt_ms / 2.0
                    # TODO: why are we still dividing by two? clarify? chat-gpt kun help me
                else:
                    # Fallback: embedded timestamp (requires NTP sync)
                    latency_ms = (recv_time_ns - send_time_ns) / 1e6
                    # TODO: this is correct (theoretically?) why is it in the else? aren't all the robots synced by definition?

                if latency_ms < 0:
                    self.get_logger().warn(
                        f"Negative latency {latency_ms:.2f} ms seq={seq} — check NTP"
                    )

                self._record_measurement(
                    recv_time_ns, send_time_ns, latency_ms,
                    'dpp', seq, tag, len(raw)
                )

            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.1)

    # ====================================================================== #
    # Shared measurement recorder                                             #
    # ====================================================================== #

    def _record_measurement(
        self,
        recv_time_ns:  int,
        send_time_ns:  int,
        latency_ms:    float,
        sender_id:     str,
        seq:           int,
        tag:           int,
        payload_bytes: int,
    ):
        jitter_ms     = self._update_jitter(sender_id, latency_ms)
        cpu_pct       = self._latest_cpu()
        bw            = self._current_bw()
        payload_label = 'pose' if tag == POSE_TAG else 'lidar'

        self.csv_writer.writerow([
            recv_time_ns,
            send_time_ns,
            round(latency_ms,  3),
            round(jitter_ms,   3),
            sender_id,
            seq,
            payload_label,
            payload_bytes,
            self.transport,
            round(cpu_pct, 1),
            round(bw, 1),
        ])
        self.csv_file.flush()

        self.received_seqs[sender_id].add(seq)
        if seq > self.expected_seqs[sender_id]:
            self.expected_seqs[sender_id] = seq

    # ====================================================================== #
    # CPU                                                                      #
    # ====================================================================== #

    def _cpu_monitor(self):
        while True:
            sample = psutil.cpu_percent(interval=0.5)
            with self._cpu_lock:
                self._cpu_samples.append(sample)
                if len(self._cpu_samples) > 120:
                    self._cpu_samples.pop(0)

    def _latest_cpu(self) -> float:
        with self._cpu_lock:
            return self._cpu_samples[-1] if self._cpu_samples else 0.0

    def _mean_cpu(self) -> float:
        with self._cpu_lock:
            return (
                sum(self._cpu_samples) / len(self._cpu_samples)
                if self._cpu_samples else 0.0
            )

    # ====================================================================== #
    # Bandwidth                                                                #
    # ====================================================================== #

    def _current_bw(self) -> float:
        now_bytes = _read_iface_bytes(self.wifi_iface)
        elapsed   = time.monotonic() - self._bw_start_time
        return (now_bytes - self._bw_start_bytes) / elapsed if elapsed > 0.001 else 0.0

    # ====================================================================== #
    # Welford online jitter                                                    #
    # ====================================================================== #

    def _update_jitter(self, sender_id: str, latency_ms: float) -> float:
        self._lat_n[sender_id] += 1
        n    = self._lat_n[sender_id]
        mean = self._lat_mean[sender_id]
        M2   = self._lat_M2[sender_id]
        delta  = latency_ms - mean
        mean  += delta / n
        M2    += delta * (latency_ms - mean)
        self._lat_mean[sender_id] = mean
        self._lat_M2[sender_id]   = M2
        return math.sqrt(M2 / (n - 1)) if n >= 2 else 0.0

    # ====================================================================== #
    # Periodic summary log                                                     #
    # ====================================================================== #

    def _log_summary(self):
        mean_cpu = self._mean_cpu()
        bw       = self._current_bw()
        for sid in self._senders:
            expected = self.expected_seqs[sid]
            if expected < 0:
                self.get_logger().info(f"[{sid}]: no messages received yet")
                continue
            expected_count = expected + 1
            received_count = len(self.received_seqs[sid])
            pdr    = (received_count / expected_count) * 100
            mean   = self._lat_mean[sid]
            jitter = (
                math.sqrt(self._lat_M2[sid] / (self._lat_n[sid] - 1))
                if self._lat_n[sid] >= 2 else 0.0
            )
            self.get_logger().info(
                f"[{self.robot_id} <- {sid}]  "
                f"PDR={pdr:.1f}% ({received_count}/{expected_count})  "
                f"lat={mean:.2f}ms  jitter={jitter:.2f}ms  "
                f"cpu={mean_cpu:.1f}%  bw={bw/1024:.1f}KB/s  "
                f"transport={self.transport}  payload={self.payload_type}"
            )

    # ====================================================================== #
    # Cleanup                                                                #
    # ====================================================================== #

    def destroy_node(self):
        self._log_summary()
        if self._ser and self._ser.is_open:
            self._ser.close()
        self.csv_file.close()
        super().destroy_node()


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = LatencyPDRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
