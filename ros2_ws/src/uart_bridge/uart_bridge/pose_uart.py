#!/usr/bin/env python3
"""
pose_uart.py
============

ROS 2 UART bridge for pose-only testing.

This node:
  - subscribes to /amcl_pose (geometry_msgs/PoseWithCovarianceStamped)
  - packs the pose into the same fixed binary format used by
    latency_pdr_node.py mixer pose mode
  - writes that raw packet to a serial port
  - reads the echoed packet back
  - decodes it into a new ROS topic

Binary pose format (big-endian):
  POSE_FMT = '>BQQ7d'
  fields: tag, seq, ts_ns, x, y, z, qx, qy, qz, qw
  size: 73 bytes

This file is intended for straight pose testing only; it does not involve lidar.
"""

import struct
import time
from typing import cast

import rclpy
import serial
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

POSE_TAG = 0x01
POSE_FMT = ">BQQ7d"
POSE_SIZE = struct.calcsize(POSE_FMT)


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
        self.get_logger().info(f"Pose packet format={POSE_FMT} size={POSE_SIZE} bytes")

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

    def _publish_pose(self, seq: int, ts_ns: int, pose_fields):
        _, _, _, x, y, z, qx, qy, qz, qw = pose_fields

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

        # Keep covariance zeroed; the packet only carries pose, not covariance.
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

        try:
            self.ser.write(payload)
        except Exception as e:
            self.get_logger().error(f"UART write failed: {e}")
            return

        self.seq += 1

    # ------------------------------------------------------------------
    # UART -> ROS
    # ------------------------------------------------------------------

    def read_serial(self):
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                self.rx_buffer += data

            while len(self.rx_buffer) >= POSE_SIZE:
                raw = self.rx_buffer[:POSE_SIZE]
                self.rx_buffer = self.rx_buffer[POSE_SIZE:]

                try:
                    fields = struct.unpack(POSE_FMT, raw)
                except struct.error:
                    # If the stream gets out of sync, drop one byte and retry.
                    self.rx_buffer = raw[1:] + self.rx_buffer
                    continue

                tag, seq, ts_ns = fields[0], fields[1], fields[2]
                if tag != POSE_TAG:
                    # Keep scanning for a valid packet boundary.
                    self.rx_buffer = raw[1:] + self.rx_buffer
                    continue

                self._publish_pose(seq, ts_ns, fields)

        except Exception as e:
            self.get_logger().error(f"UART read error: {e}")

    def destroy_node(self):
        try:
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
