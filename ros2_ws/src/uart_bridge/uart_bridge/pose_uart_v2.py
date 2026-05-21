#!/usr/bin/env python3
"""
pose_uart.py
============

ROS 2 UART bridge for pose-only testing.

This node:
  - subscribes to /amcl_pose (geometry_msgs/PoseWithCovarianceStamped)
  - packs the pose into the same fixed binary format used by
    latency_pdr_node.py mixer pose mode
  - wraps the packet with sync/length/CRC framing
  - writes that framed packet to a serial port
  - reads the echoed packet back
  - decodes it into a new ROS topic

The receiver expects the framed packet format only. If a frame cannot be
decoded, it is dropped and logged for debugging.

Binary pose payload (big-endian):
  POSE_FMT = '>BQQ7d'
  fields: tag, seq, ts_ns, x, y, z, qx, qy, qz, qw
  size: 73 bytes

Framing:
  SYNC(2) + LEN(2LE) + PAYLOAD(73) + CRC16(2LE)

This file is intended for straight pose testing only; it does not involve lidar.
"""

import binascii
import math
import struct
import time
from typing import cast

import rclpy
import serial
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

SYNC = b"\xaa\x55"
POSE_TAG = 0x01
POSE_FMT = ">BQQ7d"
POSE_SIZE = struct.calcsize(POSE_FMT)
FRAME_SIZE = 2 + 2 + POSE_SIZE + 2


def crc16_ccitt(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF)


class PoseUARTBridge(Node):
    def __init__(self):
        super().__init__("pose_uart_bridge")

        # Parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 460800)
        self.declare_parameter("input_topic", "/amcl_pose")
        self.declare_parameter("output_topic", "/amcl_pose_uart")
        self.declare_parameter("output_frame_id", "")

        self.port = cast(str, self.get_parameter("port").value)
        self.baudrate = cast(int, self.get_parameter("baudrate").value)
        self.input_topic = cast(str, self.get_parameter("input_topic").value)
        self.output_topic = cast(str, self.get_parameter("output_topic").value)
        self.output_frame_id = cast(str, self.get_parameter("output_frame_id").value)

        # Serial
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.01)
        self.get_logger().info(f"Opened {self.port} @ {self.baudrate}")
        self.get_logger().info(
            f"Pose payload format={POSE_FMT} payload_size={POSE_SIZE} frame_size={FRAME_SIZE} bytes"
        )

        # State
        self.rx_buffer = b""
        self.seq = 0
        self.last_sent_time = 0.0
        self.latest_frame_id = self.output_frame_id or "map"

        # ROS interfaces
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.input_topic,
            self.pose_callback,
            qos_profile_sensor_data,
        )
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.output_topic,
            10,
        )

        # Serial polling
        self.timer = self.create_timer(0.01, self.read_serial)
        self.stats_timer = self.create_timer(5.0, self._log_debug_summary)

        # Debug counters
        self.rx_frames = 0
        self.rx_crc_fail = 0
        self.rx_len_fail = 0
        self.rx_unpack_fail = 0
        self.rx_bad_tag = 0
        self.rx_bad_ts = 0
        self.rx_drop_bytes = 0
        self.rx_bytes = 0

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _now_ns() -> int:
        return time.time_ns()

    def _pack_pose(self, msg: PoseWithCovarianceStamped, seq: int, ts_ns: int) -> bytes:
        p = msg.pose.pose
        return struct.pack(
            POSE_FMT,
            POSE_TAG,
            seq & 0xFFFFFFFFFFFFFFFF,
            ts_ns & 0xFFFFFFFFFFFFFFFF,
            float(p.position.x),
            float(p.position.y),
            float(p.position.z),
            float(p.orientation.x),
            float(p.orientation.y),
            float(p.orientation.z),
            float(p.orientation.w),
        )

    def _build_frame(self, payload: bytes) -> bytes:
        crc = crc16_ccitt(payload)
        return SYNC + struct.pack("<H", len(payload)) + payload + struct.pack("<H", crc)

    def _timestamp_plausible(self, ts_ns: int) -> bool:
        if ts_ns <= 0:
            return False
        now_ns = time.time_ns()
        return abs(ts_ns - now_ns) < 6 * 3600 * 1_000_000_000

    def _publish_pose(self, pose_fields):
        _, _, ts_ns, x, y, z, qx, qy, qz, qw = pose_fields

        if not all(math.isfinite(v) for v in (x, y, z, qx, qy, qz, qw)):
            raise ValueError("pose contains non-finite values")

        msg = PoseWithCovarianceStamped()
        msg.header.stamp.sec = int(ts_ns // 1_000_000_000)
        msg.header.stamp.nanosec = int(ts_ns % 1_000_000_000)
        msg.header.frame_id = self.latest_frame_id or "map"

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        self.pub.publish(msg)

    # ------------------------------------------------------------------
    # ROS -> UART
    # ------------------------------------------------------------------

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.latest_frame_id = msg.header.frame_id or self.latest_frame_id or "map"

        now = time.monotonic()
        if now - self.last_sent_time < 0.0:
            return
        self.last_sent_time = now

        ts_ns = self._now_ns()
        payload = self._pack_pose(msg, self.seq, ts_ns)
        frame = self._build_frame(payload)

        try:
            self.ser.write(frame)
        except Exception as e:
            self.get_logger().error(f"UART write failed: {e}")
            return

        self.seq += 1

    # ------------------------------------------------------------------
    # UART -> ROS
    # ------------------------------------------------------------------

    def _drop_bytes(self, n: int, reason: str):
        if n <= 0:
            return
        self.rx_drop_bytes += n
        self.get_logger().debug(f"Dropping {n} byte(s): {reason}")
        self.rx_buffer = self.rx_buffer[n:]

    def _log_debug_summary(self):
        self.get_logger().info(
            f"PoseUART stats: bytes={self.rx_bytes} frames={self.rx_frames} "
            f"crc_fail={self.rx_crc_fail} len_fail={self.rx_len_fail} "
            f"unpack_fail={self.rx_unpack_fail} bad_tag={self.rx_bad_tag} "
            f"bad_ts={self.rx_bad_ts} dropped_bytes={self.rx_drop_bytes} "
            f"buffer={len(self.rx_buffer)}"
        )

    def read_serial(self):
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                self.rx_buffer += data
                self.rx_bytes += len(data)
                self.get_logger().debug(
                    f"UART read {len(data)} byte(s), buffer={len(self.rx_buffer)}"
                )

            while True:
                sync_idx = self.rx_buffer.find(SYNC)
                if sync_idx < 0:
                    # Keep the tail so a SYNC byte split across reads can still match.
                    tail_keep = len(SYNC) - 1
                    if len(self.rx_buffer) > tail_keep:
                        self._drop_bytes(
                            len(self.rx_buffer) - tail_keep,
                            "no SYNC found (preserving tail for resync)",
                        )
                    return

                if sync_idx > 0:
                    self._drop_bytes(sync_idx, "leading junk before SYNC")

                if len(self.rx_buffer) < 4:
                    self.get_logger().debug("SYNC found but waiting for length bytes")
                    return

                payload_len = struct.unpack("<H", self.rx_buffer[2:4])[0]
                if payload_len != POSE_SIZE:
                    self.rx_len_fail += 1
                    self.get_logger().debug(
                        f"Bad payload length {payload_len}; expected {POSE_SIZE}. "
                        f"Resyncing without legacy fallback"
                    )
                    self._drop_bytes(1, "payload length mismatch")
                    continue

                total_len = 2 + 2 + payload_len + 2
                if len(self.rx_buffer) < total_len:
                    self.get_logger().debug(
                        f"Partial frame: have={len(self.rx_buffer)} need={total_len}"
                    )
                    return

                payload = self.rx_buffer[4 : 4 + payload_len]
                crc_recv = struct.unpack(
                    "<H", self.rx_buffer[4 + payload_len : total_len]
                )[0]
                crc_calc = crc16_ccitt(payload)

                self._drop_bytes(total_len, "consumed frame")

                if crc_recv != crc_calc:
                    self.rx_crc_fail += 1
                    self.get_logger().debug(
                        f"CRC mismatch: recv=0x{crc_recv:04x} calc=0x{crc_calc:04x}"
                    )
                    continue

                try:
                    fields = struct.unpack(POSE_FMT, payload)
                except struct.error:
                    self.rx_unpack_fail += 1
                    self.get_logger().debug(
                        f"Payload unpack failed for {len(payload)} bytes"
                    )
                    continue

                tag, _, ts_ns = fields[0], fields[1], fields[2]
                if tag != POSE_TAG:
                    self.rx_bad_tag += 1
                    self.get_logger().debug(f"Rejected frame: tag=0x{tag:02x}")
                    continue

                if not self._timestamp_plausible(ts_ns):
                    self.rx_bad_ts += 1
                    self.get_logger().debug(f"Rejected frame: ts_ns={ts_ns}")
                    continue

                try:
                    self._publish_pose(fields)
                    self.rx_frames += 1
                except Exception as e:
                    self.get_logger().debug(f"Publish failed: {e}")
                    continue

        except Exception as e:
            self.get_logger().error(f"UART read error: {e}")

    def destroy_node(self):
        try:
            self._log_debug_summary()
            self.ser.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseUARTBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
