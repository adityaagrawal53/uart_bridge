#!/usr/bin/env python3
"""
amcl_pose_generator.py
======================

Synthetic PoseWithCovarianceStamped publisher for testing pose pipelines.

This node publishes sample AMCL-like pose data on a configurable topic
(default: /amcl_pose). It is intended for testing downstream nodes such as
pose_uart.py without needing a real localization stack.

Modes:
  - static: fixed pose
  - circle:  moves in a circle and rotates tangentially
  - line:    moves along +x at a constant speed
"""

import math
import time
from typing import cast

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class AmclPoseGenerator(Node):
    def __init__(self):
        super().__init__("amcl_pose_generator")

        # Parameters
        self.declare_parameter("output_topic", "/amcl_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("mode", "circle")  # static | circle | line

        self.declare_parameter("x0", 0.0)
        self.declare_parameter("y0", 0.0)
        self.declare_parameter("yaw0", 0.0)

        self.declare_parameter("radius", 1.0)
        self.declare_parameter("angular_speed", 0.25)  # rad/s
        self.declare_parameter("speed", 0.25)  # m/s for line mode

        self.declare_parameter("cov_xy", 0.05)
        self.declare_parameter("cov_yaw", 0.10)

        self.output_topic = cast(str, self.get_parameter("output_topic").value)
        self.frame_id = cast(str, self.get_parameter("frame_id").value)
        self.rate_hz = cast(float, self.get_parameter("rate_hz").value)
        self.mode = cast(str, self.get_parameter("mode").value).lower().strip()

        self.x0 = cast(float, self.get_parameter("x0").value)
        self.y0 = cast(float, self.get_parameter("y0").value)
        self.yaw0 = cast(float, self.get_parameter("yaw0").value)

        self.radius = cast(float, self.get_parameter("radius").value)
        self.angular_speed = cast(float, self.get_parameter("angular_speed").value)
        self.speed = cast(float, self.get_parameter("speed").value)

        self.cov_xy = cast(float, self.get_parameter("cov_xy").value)
        self.cov_yaw = cast(float, self.get_parameter("cov_yaw").value)

        if self.rate_hz <= 0.0:
            raise ValueError("rate_hz must be > 0")
        if self.mode not in ("static", "circle", "line"):
            raise ValueError("mode must be one of: static, circle, line")

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, self.output_topic, 10
        )
        self.timer = self.create_timer(1.0 / self.rate_hz, self._publish)

        self.start_time = time.monotonic()

        self.get_logger().info(
            f"Publishing synthetic AMCL pose on {self.output_topic} "
            f"mode={self.mode} rate={self.rate_hz}Hz"
        )

    @staticmethod
    def _yaw_to_quaternion(yaw: float):
        half = yaw * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def _pose_at_time(self, t: float):
        if self.mode == "static":
            x = self.x0
            y = self.y0
            yaw = self.yaw0
        elif self.mode == "line":
            x = self.x0 + self.speed * t
            y = self.y0
            yaw = self.yaw0
        else:  # circle
            theta = self.angular_speed * t
            x = self.x0 + self.radius * math.cos(theta)
            y = self.y0 + self.radius * math.sin(theta)
            yaw = self.yaw0 + theta + (math.pi / 2.0)

        return x, y, yaw

    def _publish(self):
        t = time.monotonic() - self.start_time
        x, y, yaw = self._pose_at_time(t)
        qx, qy, qz, qw = self._yaw_to_quaternion(yaw)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Simple covariance: x/y/yaw moderately confident, the rest small.
        cov = [0.0] * 36
        cov[0] = self.cov_xy * self.cov_xy
        cov[7] = self.cov_xy * self.cov_xy
        cov[14] = 1e-6
        cov[21] = 1e-6
        cov[28] = 1e-6
        cov[35] = self.cov_yaw * self.cov_yaw
        msg.pose.covariance = cov

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AmclPoseGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
