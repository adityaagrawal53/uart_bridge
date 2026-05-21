#!/usr/bin/env python3
"""
pose_repeater_node.py
=====================
AMCL only publishes PoseWithCovarianceStamped when the robot's estimated
position changes.  This node subscribes to /robot{id}/amcl_pose, caches
the latest pose, and republishes it at a fixed rate on
/robot{id}/amcl_pose_stamped so that latency_pdr_node always has a
fresh message to forward.

Parameters
----------
  robot_id        str     'robot1'
  publish_rate_hz float   10.0      match ping_rate_hz in latency_pdr_node
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class PoseRepeaterNode(Node):

    def __init__(self):
        super().__init__('pose_repeater_node')

        self.declare_parameter('robot_id',        'robot1')
        self.declare_parameter('publish_rate_hz', 10.0)

        self.robot_id = self.get_parameter('robot_id').value
        rate_hz       = self.get_parameter('publish_rate_hz').value

        self._latest_pose: PoseWithCovarianceStamped | None = None

        # Subscribe to AMCL (event-driven, only updates on pose change)
        self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{self.robot_id}/amcl_pose',
            self._amcl_callback,
            10
        )

        # Publish at fixed rate
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped,
            f'/{self.robot_id}/amcl_pose_repeated',
            10
        )

        self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info(
            f"[{self.robot_id}] Repeating amcl_pose at {rate_hz} Hz "
            f"on /{self.robot_id}/amcl_pose_repeated"
        )

    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        """Cache the latest pose whenever AMCL publishes."""
        self._latest_pose = msg

    def _publish(self):
        if self._latest_pose is None:
            self.get_logger().warn(
                "No pose received from AMCL yet — waiting ...",
                throttle_duration_sec=5.0
            )
            return

        # Update the header stamp to now so receivers get a fresh timestamp
        self._latest_pose.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._latest_pose)


def main(args=None):
    rclpy.init(args=args)
    node = PoseRepeaterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
