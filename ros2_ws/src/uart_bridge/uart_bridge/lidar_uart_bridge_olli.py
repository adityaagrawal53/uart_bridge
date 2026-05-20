"""
lidar_uart_bridge.py
====================
ROS 2 node that multiplexes LiDAR scan data and robot pose over a single
UART link to the MSP432 application processor (AP).

Frame format (identical to the original lidar bridge):
    SYNC(2)  ENC_LEN(2LE)  COBS_PAYLOAD(N)  CRC16(2LE)

Inside the COBS payload the first byte is now a packet-type tag:
    0x01  –  LiDAR scan   (original content, prepended with type byte)
    0x02  –  Pose TX       Pi  → MSP432  (own pose, to be broadcast via BOLT)
    0x03  –  Pose RX       MSP432 → Pi  (peer pose received from BOLT/Mixer)

Pose binary layout (all little-endian, 25 bytes after the type byte):
    uint8   robot_id
    int32   x_fp         (Q16.16 fixed-point metres)
    int32   y_fp
    int32   theta_fp     (Q16.16 fixed-point radians)
    int32   v_fp         (Q16.16 fixed-point m/s)
    uint32  timestamp_ms

Subscriptions
    /scan               sensor_msgs/LaserScan      → UART TX  (type 0x01)
    /pose               nav_msgs/Odometry          → UART TX  (type 0x02)
        OR
    /pose_cov           geometry_msgs/PoseWithCovarianceStamped

Publishers
    /scan_uart          sensor_msgs/LaserScan      ← UART RX  (type 0x01)
    /peer_poses         geometry_msgs/PoseArray    ← UART RX  (type 0x03)

Parameters
    port                str     /dev/ttyUSB0
    baudrate            int     460800
    scan_topic_in       str     /scan
    scan_topic_out      str     /scan_uart
    pose_topic_in       str     /odom          (nav_msgs/Odometry)
    peer_poses_topic    str     /peer_poses
    rate_hz             float   10.0           max scan TX rate
    pose_rate_hz        float   20.0           max pose TX rate
    downsample_step     int     2
    robot_id            int     1
"""

import binascii
import math
import struct
import time
from typing import cast

import rclpy
import serial
from cobs import cobs
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# ─── Wire constants ────────────────────────────────────────────────────────────
SYNC = b"\xaa\x55"

PKT_TYPE_SCAN     = 0x01
PKT_TYPE_POSE_TX  = 0x02   # Pi  → MSP432  (own pose)
PKT_TYPE_POSE_RX  = 0x03   # MSP432 → Pi  (peer pose from BOLT)
PKT_TYPE_SCAN_SUMMARY = 0x04  # compact scan summary through Mixer

# Pose payload: uint8 robot_id, int32×4 (x,y,theta,v), uint32 timestamp_ms
POSE_FMT  = "<BiiiiI"      # 1 + 4+4+4+4 + 4 = 21 bytes
POSE_SIZE = struct.calcsize(POSE_FMT)

SCAN_BINS = 20
SCAN_SUMMARY_FMT = "<BHBhhI" + "H" * SCAN_BINS
SCAN_SUMMARY_SIZE = struct.calcsize(SCAN_SUMMARY_FMT)

# Scan header (unchanged from original)
HEADER_FMT  = "<III6f3H"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

MAX_POINTS = 20_000

# Q16.16 fixed-point helpers
Q = 65536.0


def _to_fp(v: float) -> int:
    """Float → Q16.16 signed fixed-point int32."""
    return int(round(v * Q))


def _from_fp(v: int) -> float:
    """Q16.16 signed fixed-point int32 → float."""
    return v / Q


def crc16(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF)


def _yaw_from_quaternion(q) -> float:
    """Extract yaw (Z-rotation) from a geometry_msgs Quaternion."""
    # q is nav_msgs/Odometry pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _build_frame(type_byte: int, payload: bytes) -> bytes:
    """Wrap payload in:  type_byte | payload  →  COBS  →  SYNC+len+COBS+CRC."""
    inner   = bytes([type_byte]) + payload
    encoded = cobs.encode(inner)
    crc     = struct.pack("<H", crc16(encoded))
    return SYNC + struct.pack("<H", len(encoded)) + encoded + crc


# ─── Node ──────────────────────────────────────────────────────────────────────

class LidarUARTBridge(Node):

    def __init__(self):
        super().__init__("lidar_uart_bridge")

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter("port",             "/dev/ttyUSB0")
        self.declare_parameter("baudrate",         460800)
        self.declare_parameter("scan_topic_in",    "/scan")
        self.declare_parameter("scan_topic_out",   "/scan_uart")
        self.declare_parameter("pose_topic_in",    "/odom")
        self.declare_parameter("peer_poses_topic", "/peer_poses")
        self.declare_parameter("peer_scan_topic",  "/peer_scan")
        self.declare_parameter("rate_hz",          10.0)
        self.declare_parameter("pose_rate_hz",     20.0)
        self.declare_parameter("downsample_step",  2)
        self.declare_parameter("robot_id",         1)

        port      = cast(str,   self.get_parameter("port").value)
        baudrate  = cast(int,   self.get_parameter("baudrate").value)
        self.rate_hz      = cast(float, self.get_parameter("rate_hz").value)
        self.pose_rate_hz = cast(float, self.get_parameter("pose_rate_hz").value)
        self.step         = max(1, cast(int, self.get_parameter("downsample_step").value))
        self.robot_id     = cast(int,   self.get_parameter("robot_id").value)

        # ── serial port ───────────────────────────────────────────────────────
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.get_logger().info(f"UART opened {port} @ {baudrate}")

        # ── TX state ──────────────────────────────────────────────────────────
        self.rx_buffer  = b""
        self.last_scan_tx  = 0.0
        self.last_pose_tx  = 0.0
        self.seq = 0

        # ── ROS subscriptions ─────────────────────────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan,
            cast(str, self.get_parameter("scan_topic_in").value),
            self.scan_tx_cb,
            qos_profile_sensor_data,
        )

        self.pose_sub = self.create_subscription(
            Odometry,
            cast(str, self.get_parameter("pose_topic_in").value),
            self.pose_tx_cb,
            qos_profile_sensor_data,
        )

        # ── ROS publishers ────────────────────────────────────────────────────
        self.scan_pub = self.create_publisher(
            LaserScan,
            cast(str, self.get_parameter("scan_topic_out").value),
            qos_profile_sensor_data,
        )

        self.peer_pub = self.create_publisher(
            PoseArray,
            cast(str, self.get_parameter("peer_poses_topic").value),
            10,
        )

        self.peer_scan_pub = self.create_publisher(
            LaserScan,
            cast(str, self.get_parameter("peer_scan_topic").value),
            qos_profile_sensor_data,
        )

        # Debug raw publishers
        self.tx_dbg = self.create_publisher(String, "/uart_tx_raw", 10)
        self.rx_dbg = self.create_publisher(String, "/uart_rx_raw", 10)

        # ── timers ────────────────────────────────────────────────────────────
        self.timer       = self.create_timer(0.002, self.rx_loop)
        self.dbg_timer   = self.create_timer(1.0,   self.debug_heartbeat)

    # ── helpers ────────────────────────────────────────────────────────────────

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _send(self, type_byte: int, payload: bytes):
        frame = _build_frame(type_byte, payload)
        self.ser.write(frame)

    def debug_heartbeat(self):
        msg      = String()
        msg.data = f"RX buffer size={len(self.rx_buffer)}"
        self.rx_dbg.publish(msg)

    # ── TX: LiDAR scan (type 0x01) ─────────────────────────────────────────────

    def scan_tx_cb(self, msg: LaserScan):
        """Transmit a compact 20-bin scan summary through AP/CP/Mixer.

        Full LaserScan frames are too large for BOLT/Mixer, so each bin stores
        the minimum valid range in millimetres over one angular sector.
        """
        now = self._now_sec()
        if now - self.last_scan_tx < 1.0 / max(0.1, self.rate_hz):
            return
        self.last_scan_tx = now

        ranges_in = list(msg.ranges)
        if not ranges_in:
            return

        bins = [0] * SCAN_BINS
        n = len(ranges_in)
        for b in range(SCAN_BINS):
            start_i = int(b * n / SCAN_BINS)
            end_i = int((b + 1) * n / SCAN_BINS)
            vals = []
            for r in ranges_in[start_i:max(start_i + 1, end_i)]:
                if not (math.isnan(r) or math.isinf(r)) and r >= 0:
                    vals.append(min(max(int(r * 1000), 0), 0xFFFF))
            bins[b] = min(vals) if vals else 0

        try:
            ts_ms = int(msg.header.stamp.sec) * 1000 + int(msg.header.stamp.nanosec) // 1_000_000
        except Exception:
            ts_ms = int(time.time() * 1000)

        angle_min_cdeg = int(round(math.degrees(float(msg.angle_min)) * 100.0))
        total_span = float(msg.angle_increment) * max(1, n - 1)
        angle_inc_cdeg = int(round(math.degrees(total_span / SCAN_BINS) * 100.0))

        payload = struct.pack(
            SCAN_SUMMARY_FMT,
            self.robot_id & 0xFF,
            self.seq & 0xFFFF,
            SCAN_BINS,
            max(min(angle_min_cdeg, 32767), -32768),
            max(min(angle_inc_cdeg, 32767), -32768),
            ts_ms & 0xFFFFFFFF,
            *bins,
        )

        self._send(PKT_TYPE_SCAN_SUMMARY, payload)
        self.seq += 1

    # ── TX: own pose (type 0x02) ───────────────────────────────────────────────

    def pose_tx_cb(self, msg: Odometry):
        now = self._now_sec()
        if now - self.last_pose_tx < 1.0 / max(0.1, self.pose_rate_hz):
            return
        self.last_pose_tx = now

        p     = msg.pose.pose.position
        yaw   = _yaw_from_quaternion(msg.pose.pose.orientation)

        # Linear speed from twist (magnitude of x/y)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v  = math.sqrt(vx * vx + vy * vy)

        # ROS stamp → milliseconds
        try:
            ts_ms = (int(msg.header.stamp.sec) * 1000
                     + int(msg.header.stamp.nanosec) // 1_000_000)
        except Exception:
            ts_ms = int(time.time() * 1000)

        payload = struct.pack(
            POSE_FMT,
            self.robot_id & 0xFF,
            _to_fp(p.x),
            _to_fp(p.y),
            _to_fp(yaw),
            _to_fp(v),
            ts_ms & 0xFFFFFFFF,
        )

        self._send(PKT_TYPE_POSE_TX, payload)

        self.get_logger().debug(
            f"POSE TX robot={self.robot_id} "
            f"x={p.x:.3f} y={p.y:.3f} yaw={math.degrees(yaw):.1f}° v={v:.3f}"
        )

    # ── RX loop ────────────────────────────────────────────────────────────────

    def rx_loop(self):
        data = self.ser.read(self.ser.in_waiting or 1)
        if data:
            self.rx_buffer += data

        while True:
            buf = self.rx_buffer
            idx = buf.find(SYNC)
            if idx < 0:
                self.rx_buffer = b""
                return
            if idx > 0:
                self.rx_buffer = buf[idx:]
                buf = self.rx_buffer

            if len(buf) < 4:
                return

            enc_len = struct.unpack("<H", buf[2:4])[0]
            total   = 4 + enc_len + 2

            if len(buf) < total:
                return

            packet   = buf[:total]
            encoded  = packet[4 : 4 + enc_len]
            crc_recv = struct.unpack("<H", packet[4 + enc_len : 4 + enc_len + 2])[0]

            self.rx_buffer = buf[total:]

            if crc16(encoded) != crc_recv:
                self.get_logger().warning("RX CRC mismatch – dropping frame")
                continue

            try:
                decoded = cobs.decode(encoded)
            except Exception as e:
                self.get_logger().warning(f"COBS decode error: {e}")
                continue

            if len(decoded) < 1:
                continue

            pkt_type = decoded[0]
            body     = decoded[1:]

            if pkt_type == PKT_TYPE_SCAN:
                self._handle_rx_scan(body)
            elif pkt_type == PKT_TYPE_POSE_RX:
                self._handle_rx_pose(body)
            elif pkt_type == PKT_TYPE_SCAN_SUMMARY:
                self._handle_rx_scan_summary(body)
            else:
                self.get_logger().warning(f"RX unknown packet type 0x{pkt_type:02x}")

    # ── RX: scan (type 0x01) ───────────────────────────────────────────────────

    def _handle_rx_scan(self, body: bytes):
        if len(body) < HEADER_SIZE:
            self.get_logger().warning("RX scan: body too short for header")
            return

        (
            seq,
            stamp_sec,
            stamp_nsec,
            angle_min,
            angle_inc,
            time_increment,
            scan_time,
            range_min,
            range_max,
            original_count,
            step,
            count,
        ) = struct.unpack(HEADER_FMT, body[:HEADER_SIZE])

        expected = HEADER_SIZE + 2 * count + 4 * count
        if len(body) != expected:
            self.get_logger().warning(
                f"RX scan SIZE MISMATCH: expected={expected} got={len(body)} count={count}"
            )
            return

        offset = HEADER_SIZE
        ranges = struct.unpack("<" + "H" * count, body[offset : offset + 2 * count])
        offset += 2 * count
        intensities = struct.unpack("<" + "f" * count, body[offset : offset + 4 * count])

        msg = LaserScan()
        msg.header.stamp.sec    = stamp_sec
        msg.header.stamp.nanosec = stamp_nsec
        msg.header.frame_id     = "lidar_uart"

        ds = max(1, int(step))
        msg.angle_min       = angle_min
        msg.angle_increment = angle_inc * ds
        msg.time_increment  = time_increment * ds
        msg.scan_time       = scan_time
        msg.range_min       = range_min
        msg.range_max       = range_max
        msg.ranges          = [r / 1000.0 if r > 0 else float("inf") for r in ranges]
        msg.intensities     = list(intensities)

        self.scan_pub.publish(msg)

    # ── RX: peer pose (type 0x03) ──────────────────────────────────────────────

    def _handle_rx_pose(self, body: bytes):
        if len(body) < POSE_SIZE:
            self.get_logger().warning(
                f"RX pose: body too short ({len(body)} < {POSE_SIZE})"
            )
            return

        robot_id, x_fp, y_fp, theta_fp, v_fp, ts_ms = struct.unpack(
            POSE_FMT, body[:POSE_SIZE]
        )

        x     = _from_fp(x_fp)
        y     = _from_fp(y_fp)
        theta = _from_fp(theta_fp)

        self.get_logger().debug(
            f"POSE RX robot={robot_id} x={x:.3f} y={y:.3f} "
            f"yaw={math.degrees(theta):.1f}° ts={ts_ms} ms"
        )

        # Publish as a single-element PoseArray so callers can correlate robot_id
        # via the frame_id field: "robot_<id>"
        pa = PoseArray()
        pa.header.frame_id = f"robot_{robot_id}"
        try:
            pa.header.stamp = self.get_clock().now().to_msg()
        except Exception:
            pass

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0

        # Convert yaw back to quaternion
        half = theta / 2.0
        pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(half),
            w=math.cos(half),
        )

        pa.poses.append(pose)
        self.peer_pub.publish(pa)

    # ── RX: compact peer scan summary (type 0x04) ─────────────────────────────

    def _handle_rx_scan_summary(self, body: bytes):
        if len(body) < SCAN_SUMMARY_SIZE:
            self.get_logger().warning(
                f"RX scan summary: body too short ({len(body)} < {SCAN_SUMMARY_SIZE})"
            )
            return

        unpacked = struct.unpack(SCAN_SUMMARY_FMT, body[:SCAN_SUMMARY_SIZE])
        robot_id, seq, count, angle_min_cdeg, angle_inc_cdeg, ts_ms, *ranges_mm = unpacked
        count = min(int(count), SCAN_BINS)

        msg = LaserScan()
        msg.header.frame_id = f"robot_{robot_id}_scan"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = math.radians(angle_min_cdeg / 100.0)
        msg.angle_increment = math.radians(angle_inc_cdeg / 100.0)
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = 0.0
        msg.range_max = 65.535
        msg.ranges = [r / 1000.0 if r > 0 else float("inf") for r in ranges_mm[:count]]
        msg.intensities = [float(robot_id)] * count
        self.peer_scan_pub.publish(msg)

    # ── cleanup ────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = LidarUARTBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
