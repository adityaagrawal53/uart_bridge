#!/usr/bin/env python3
"""
latency_pdr_node.py

DDS mode:
    Uses ROS 2 topics for baseline measurement.

Mixer mode:
    Uses JSON-over-UART to communicate with AP/DPP.
    This uses the same UART style as odom_uart.py:
        - one JSON object per line
        - newline terminated
        - UTF-8 text
        - buffer until newline
        - ignore garbage lines

Mixer pose source:
    /<robot_id>/amcl_pose

Mixer lidar source:
    /<robot_id>/scan

Important:
    The AP/DPP path must return JSON that still contains:
        seq
        ts_ns
        tag

    Without seq and ts_ns, this node cannot calculate latency or PDR.
"""

import math
import os
import csv
import struct
import json
import threading
import time
from datetime import datetime

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
# DDS binary payload formats
#
# These are kept for the CycloneDDS baseline.
# Mixer mode uses JSON-over-UART instead.
# ---------------------------------------------------------------------------

POSE_TAG = 0x01
LIDAR_TAG = 0x02

POSE_FMT = ">BQQ7d"
LIDAR_FMT = ">BQQ360f"

POSE_SIZE = struct.calcsize(POSE_FMT)      # 73 bytes
LIDAR_SIZE = struct.calcsize(LIDAR_FMT)    # 1457 bytes


def _read_iface_bytes(iface: str) -> int:
    """Read cumulative RX bytes for a network interface from /proc/net/dev."""
    try:
        with open("/proc/net/dev") as f:
            for line in f:
                if iface in line:
                    return int(line.split()[1])
    except Exception:
        pass
    return 0


class LatencyPDRNode(Node):
    def __init__(self):
        super().__init__("latency_pdr_node")

        # ------------------------------------------------------------------ #
        # Parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter("robot_id", "robot1")
        self.declare_parameter("other_robot_ids", ["robot2"])
        self.declare_parameter("ping_rate_hz", 5.0)
        self.declare_parameter("transport", "cyclonedds")
        self.declare_parameter("payload_type", "pose")
        self.declare_parameter("serial_port", "/dev/ttyUSB1")
        self.declare_parameter("serial_baud", 460800)
        self.declare_parameter("wifi_iface", "wlan0")
        self.declare_parameter("log_dir", "/home/ubuntu/measurements")

        self.robot_id = self.get_parameter("robot_id").value
        self.other_ids = self.get_parameter("other_robot_ids").value
        self.ping_rate_hz = float(self.get_parameter("ping_rate_hz").value)
        self.transport = self.get_parameter("transport").value
        self.payload_type = self.get_parameter("payload_type").value
        self.serial_port = self.get_parameter("serial_port").value
        self.serial_baud = int(self.get_parameter("serial_baud").value)
        self.wifi_iface = self.get_parameter("wifi_iface").value
        self.log_dir = self.get_parameter("log_dir").value

        self._validate_params()

        # ------------------------------------------------------------------ #
        # Latest sensor data
        # ------------------------------------------------------------------ #
        self._latest_pose: tuple | None = None
        # pose = (x, y, z, qx, qy, qz, qw)

        self._latest_ranges: tuple | None = None
        # ranges = 360 floats

        # ------------------------------------------------------------------ #
        # Measurement state
        # ------------------------------------------------------------------ #
        self.seq = 0

        self._senders = self.other_ids if self.transport == "cyclonedds" else ["dpp"]

        self.received_seqs: dict[str, set] = {s: set() for s in self._senders}
        self.expected_seqs: dict[str, int] = {s: -1 for s in self._senders}

        self._lat_n: dict[str, int] = {s: 0 for s in self._senders}
        self._lat_mean: dict[str, float] = {s: 0.0 for s in self._senders}
        self._lat_M2: dict[str, float] = {s: 0.0 for s in self._senders}

        # Mixer only: seq -> send_time_ns
        self._send_times: dict[int, int] = {}

        # Mixer UART receive buffer
        self.rx_buffer = ""

        # ------------------------------------------------------------------ #
        # CPU monitoring
        # ------------------------------------------------------------------ #
        self._cpu_samples: list[float] = []
        self._cpu_lock = threading.Lock()

        if _PSUTIL:
            threading.Thread(target=self._cpu_monitor, daemon=True).start()
        else:
            self.get_logger().warn("psutil not installed — CPU will read 0.0")

        # ------------------------------------------------------------------ #
        # Bandwidth baseline
        # ------------------------------------------------------------------ #
        self._bw_start_bytes = _read_iface_bytes(self.wifi_iface)
        self._bw_start_time = time.monotonic()

        # ------------------------------------------------------------------ #
        # Transport setup
        # ------------------------------------------------------------------ #
        self._ser = None

        if self.transport == "mixer":
            self._setup_serial()
        else:
            self._setup_cyclonedds()

        # ------------------------------------------------------------------ #
        # ROS subscribers
        # ------------------------------------------------------------------ #
        self.create_subscription(
            PoseWithCovarianceStamped,
            f"/{self.robot_id}/amcl_pose",
            self._amcl_callback,
            10,
        )

        self.create_subscription(
            LaserScan,
            f"/{self.robot_id}/scan",
            self._scan_callback,
            10,
        )

        # ------------------------------------------------------------------ #
        # CSV
        # ------------------------------------------------------------------ #
        os.makedirs(self.log_dir, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = (
            f"{self.log_dir}/"
            f"{self.transport}_{self.payload_type}_{self.robot_id}_{ts}.csv"
        )

        self.csv_file = open(fname, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow(
            [
                "recv_time_ns",
                "send_time_ns",
                "latency_ms",
                "jitter_ms",
                "sender_id",
                "seq",
                "payload_type",
                "payload_bytes",
                "transport",
                "cpu_pct",
                "bw_bytes_per_s",
            ]
        )

        self.get_logger().info(
            f"[{self.robot_id}] transport={self.transport} "
            f"payload={self.payload_type} -> {fname}"
        )

        # ------------------------------------------------------------------ #
        # Timers
        # ------------------------------------------------------------------ #
        period = 1.0 / self.ping_rate_hz
        self.create_timer(period, self._publish_ping)
        self.create_timer(10.0, self._log_summary)

    # ====================================================================== #
    # Validation
    # ====================================================================== #

    def _validate_params(self):
        if self.transport not in ("cyclonedds", "mixer"):
            raise ValueError(
                f"transport must be 'cyclonedds' or 'mixer', got '{self.transport}'"
            )

        if self.payload_type not in ("pose", "lidar"):
            raise ValueError(
                f"payload_type must be 'pose' or 'lidar', got '{self.payload_type}'"
            )

        if self.transport == "mixer" and not _SERIAL:
            raise RuntimeError(
                "pyserial not installed — run: "
                "pip install pyserial --break-system-packages"
            )

    # ====================================================================== #
    # Transport setup
    # ====================================================================== #

    def _setup_cyclonedds(self):
        """DDS baseline publisher/subscribers."""
        self.ping_pub = self.create_publisher(
            String,
            f"/{self.robot_id}/ping",
            10,
        )

        for other in self.other_ids:
            self.create_subscription(
                String,
                f"/{other}/ping",
                lambda msg, sid=other: self._dds_ping_callback(msg, sid),
                10,
            )

    def _setup_serial(self):
        """Open UART to AP/DPP and start JSON receive thread."""
        try:
            self._ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baud,
                timeout=0.01,
            )
            self.get_logger().info(
                f"Serial opened: {self.serial_port} @ {self.serial_baud} baud"
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Cannot open serial port: {e}")
            self._ser = None
            return

        threading.Thread(
            target=self._serial_read_loop,
            daemon=True,
        ).start()

    # ====================================================================== #
    # ROS callbacks
    # ====================================================================== #

    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose

        self._latest_pose = (
            p.position.x,
            p.position.y,
            p.position.z,
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w,
        )

    def _scan_callback(self, msg: LaserScan):
        ranges = [r if math.isfinite(r) else 0.0 for r in msg.ranges[:360]]

        while len(ranges) < 360:
            ranges.append(0.0)

        self._latest_ranges = tuple(ranges)

    # ====================================================================== #
    # Payload builders / parsers
    # ====================================================================== #

    def _build_mixer_json_payload(self, seq: int, now_ns: int) -> str:
        """
        Build newline-delimited JSON payload for AP/DPP UART.

        Pose uses /amcl_pose.
        Lidar uses /scan.
        """
        if self.payload_type == "pose":
            x, y, z, qx, qy, qz, qw = self._latest_pose or (
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
            )

            payload = {
                "tag": "pose",
                "seq": int(seq),
                "ts_ns": int(now_ns),
                "x": round(float(x), 3),
                "y": round(float(y), 3),
                "z": round(float(z), 3),
                "qx": round(float(qx), 6),
                "qy": round(float(qy), 6),
                "qz": round(float(qz), 6),
                "qw": round(float(qw), 6),
            }

        else:
            ranges = self._latest_ranges or (0.0,) * 360

            payload = {
                "tag": "lidar",
                "seq": int(seq),
                "ts_ns": int(now_ns),
                "ranges": [round(float(r), 3) for r in ranges],
            }

        return json.dumps(payload, separators=(",", ":")) + "\n"

    def _build_dds_payload(self, seq: int, now_ns: int) -> bytes:
        """Build original binary DDS comparison payload."""
        if self.payload_type == "pose":
            pose = self._latest_pose or (0.0,) * 7
            return struct.pack(POSE_FMT, POSE_TAG, seq, now_ns, *pose)

        ranges = self._latest_ranges or (0.0,) * 360
        return struct.pack(LIDAR_FMT, LIDAR_TAG, seq, now_ns, *ranges)

    def _parse_dds_payload(self, raw: bytes):
        """Returns (tag, seq, send_time_ns) for DDS binary payload."""
        if not raw:
            raise ValueError("empty payload")

        tag = raw[0]

        if tag == POSE_TAG:
            if len(raw) < POSE_SIZE:
                raise ValueError(f"pose payload too short ({len(raw)} bytes)")
            fields = struct.unpack_from(POSE_FMT, raw)
            return fields[0], fields[1], fields[2]

        if tag == LIDAR_TAG:
            if len(raw) < LIDAR_SIZE:
                raise ValueError(f"lidar payload too short ({len(raw)} bytes)")
            fields = struct.unpack_from(LIDAR_FMT, raw)
            return fields[0], fields[1], fields[2]

        raise ValueError(f"unknown tag 0x{tag:02x}")

    # ====================================================================== #
    # Publish / send timer
    # ====================================================================== #

    def _publish_ping(self):
        now_ns = self.get_clock().now().nanoseconds

        if self.transport == "mixer":
            if self.payload_type == "pose" and self._latest_pose is None:
                self.get_logger().warn(
                    "Waiting for /amcl_pose ...",
                    throttle_duration_sec=5.0,
                )
                return

            if self.payload_type == "lidar" and self._latest_ranges is None:
                self.get_logger().warn(
                    "Waiting for /scan ...",
                    throttle_duration_sec=5.0,
                )
                return

            if self._ser and self._ser.is_open:
                try:
                    line = self._build_mixer_json_payload(self.seq, now_ns)
                    self._ser.write(line.encode("utf-8"))

                    self._send_times[self.seq] = now_ns

                    if len(self._send_times) > 500:
                        del self._send_times[min(self._send_times)]

                except serial.SerialException as e:
                    self.get_logger().error(f"Serial write error: {e}")
            else:
                self.get_logger().warn(
                    "Serial port not available",
                    throttle_duration_sec=5.0,
                )

            self.seq += 1
            return

        # DDS baseline
        if self.payload_type == "pose" and self._latest_pose is None:
            self.get_logger().warn(
                "Waiting for /amcl_pose ...",
                throttle_duration_sec=5.0,
            )
            return

        if self.payload_type == "lidar" and self._latest_ranges is None:
            self.get_logger().warn(
                "Waiting for /scan ...",
                throttle_duration_sec=5.0,
            )
            return

        raw = self._build_dds_payload(self.seq, now_ns)

        msg = String()
        msg.data = raw.hex()
        self.ping_pub.publish(msg)

        self.seq += 1

    # ====================================================================== #
    # DDS receive callback
    # ====================================================================== #

    def _dds_ping_callback(self, msg: String, sender_id: str):
        recv_time_ns = self.get_clock().now().nanoseconds

        try:
            raw = bytes.fromhex(msg.data)
            tag, seq, send_time_ns = self._parse_dds_payload(raw)
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
            recv_time_ns=recv_time_ns,
            send_time_ns=send_time_ns,
            latency_ms=latency_ms,
            sender_id=sender_id,
            seq=seq,
            tag=tag,
            payload_bytes=len(raw),
        )

    # ====================================================================== #
    # Mixer/DPP UART JSON receive loop
    # ====================================================================== #

    def _serial_read_loop(self):
        """
        Read newline-delimited JSON from AP/DPP UART.

        This follows odom_uart.py style:
            read bytes
            decode UTF-8
            accumulate in rx_buffer
            split complete lines on newline
            ignore garbage
            json.loads()
        """
        self.get_logger().info("Serial JSON read loop started")

        while True:
            try:
                data = self._ser.read(self._ser.in_waiting or 1).decode(
                    "utf-8",
                    errors="ignore",
                )

                if not data:
                    continue

                self.rx_buffer += data

                while "\n" in self.rx_buffer:
                    line, self.rx_buffer = self.rx_buffer.split("\n", 1)
                    line = line.strip()

                    if not line:
                        continue

                    if "{" not in line or "}" not in line:
                        continue

                    if len(line) < 10 or len(line) > 10000:
                        continue

                    try:
                        payload = json.loads(line)
                    except json.JSONDecodeError:
                        continue

                    if not isinstance(payload, dict):
                        continue

                    tag_value = payload.get("tag", None)
                    seq_value = payload.get("seq", None)
                    ts_value = payload.get("ts_ns", None)

                    if tag_value is None or seq_value is None or ts_value is None:
                        self.get_logger().warn(
                            f"JSON echo missing tag/seq/ts_ns: {payload}",
                            throttle_duration_sec=2.0,
                        )
                        continue

                    try:
                        seq = int(seq_value)
                        send_time_ns = int(ts_value)
                    except (TypeError, ValueError):
                        self.get_logger().warn(
                            f"Bad seq/ts_ns in JSON echo: {payload}",
                            throttle_duration_sec=2.0,
                        )
                        continue

                    recv_time_ns = self.get_clock().now().nanoseconds

                    if seq in self._send_times:
                        # Round trip through DPP/Mixer and back.
                        rtt_ms = (recv_time_ns - self._send_times.pop(seq)) / 1e6
                        latency_ms = rtt_ms / 2.0
                    else:
                        # Fallback if local send time is missing.
                        # Requires synchronized clocks.
                        latency_ms = (recv_time_ns - send_time_ns) / 1e6

                    if latency_ms < 0:
                        self.get_logger().warn(
                            f"Negative latency {latency_ms:.2f} ms seq={seq} "
                            f"— check clock sync"
                        )

                    if tag_value == "pose":
                        tag_id = POSE_TAG
                    elif tag_value == "lidar":
                        tag_id = LIDAR_TAG
                    else:
                        self.get_logger().warn(
                            f"Unknown JSON tag: {tag_value}",
                            throttle_duration_sec=2.0,
                        )
                        continue

                    self._record_measurement(
                        recv_time_ns=recv_time_ns,
                        send_time_ns=send_time_ns,
                        latency_ms=latency_ms,
                        sender_id="dpp",
                        seq=seq,
                        tag=tag_id,
                        payload_bytes=len(line.encode("utf-8")),
                    )

            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.1)

    # ====================================================================== #
    # Measurement recorder
    # ====================================================================== #

    def _record_measurement(
        self,
        recv_time_ns: int,
        send_time_ns: int,
        latency_ms: float,
        sender_id: str,
        seq: int,
        tag: int,
        payload_bytes: int,
    ):
        jitter_ms = self._update_jitter(sender_id, latency_ms)
        cpu_pct = self._latest_cpu()
        bw = self._current_bw()

        payload_label = "pose" if tag == POSE_TAG else "lidar"

        self.csv_writer.writerow(
            [
                recv_time_ns,
                send_time_ns,
                round(latency_ms, 3),
                round(jitter_ms, 3),
                sender_id,
                seq,
                payload_label,
                payload_bytes,
                self.transport,
                round(cpu_pct, 1),
                round(bw, 1),
            ]
        )
        self.csv_file.flush()

        self.received_seqs[sender_id].add(seq)

        if seq > self.expected_seqs[sender_id]:
            self.expected_seqs[sender_id] = seq

    # ====================================================================== #
    # CPU
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
            if not self._cpu_samples:
                return 0.0
            return sum(self._cpu_samples) / len(self._cpu_samples)

    # ====================================================================== #
    # Bandwidth
    # ====================================================================== #

    def _current_bw(self) -> float:
        now_bytes = _read_iface_bytes(self.wifi_iface)
        elapsed = time.monotonic() - self._bw_start_time

        if elapsed <= 0.001:
            return 0.0

        return (now_bytes - self._bw_start_bytes) / elapsed

    # ====================================================================== #
    # Jitter
    # ====================================================================== #

    def _update_jitter(self, sender_id: str, latency_ms: float) -> float:
        self._lat_n[sender_id] += 1

        n = self._lat_n[sender_id]
        mean = self._lat_mean[sender_id]
        m2 = self._lat_M2[sender_id]

        delta = latency_ms - mean
        mean += delta / n
        m2 += delta * (latency_ms - mean)

        self._lat_mean[sender_id] = mean
        self._lat_M2[sender_id] = m2

        if n < 2:
            return 0.0

        return math.sqrt(m2 / (n - 1))

    # ====================================================================== #
    # Summary
    # ====================================================================== #

    def _log_summary(self):
        mean_cpu = self._mean_cpu()
        bw = self._current_bw()

        for sid in self._senders:
            expected = self.expected_seqs[sid]

            if expected < 0:
                self.get_logger().info(f"[{sid}]: no messages received yet")
                continue

            expected_count = expected + 1
            received_count = len(self.received_seqs[sid])

            pdr = (received_count / expected_count) * 100.0
            mean = self._lat_mean[sid]

            if self._lat_n[sid] >= 2:
                jitter = math.sqrt(self._lat_M2[sid] / (self._lat_n[sid] - 1))
            else:
                jitter = 0.0

            self.get_logger().info(
                f"[{self.robot_id} <- {sid}] "
                f"PDR={pdr:.1f}% ({received_count}/{expected_count}) "
                f"lat={mean:.2f}ms "
                f"jitter={jitter:.2f}ms "
                f"cpu={mean_cpu:.1f}% "
                f"bw={bw / 1024:.1f}KB/s "
                f"transport={self.transport} "
                f"payload={self.payload_type}"
            )

    # ====================================================================== #
    # Cleanup
    # ====================================================================== #

    def destroy_node(self):
        self._log_summary()

        if self._ser and self._ser.is_open:
            self._ser.close()

        self.csv_file.close()
        super().destroy_node()


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


if __name__ == "__main__":
    main()