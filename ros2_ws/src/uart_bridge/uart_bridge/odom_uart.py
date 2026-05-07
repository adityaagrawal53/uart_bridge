import json
import math

import rclpy
import serial
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class OdomUARTBridge(Node):
    def __init__(self):
        super().__init__("odom_uart_bridge")
        self.rx_buffer = ""

        # Parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("rate_limit_hz", 50.0)

        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.rate_limit = (
            self.get_parameter("rate_limit_hz").get_parameter_value().double_value
        )

        # Serial
        self.ser = serial.Serial(port, baudrate, timeout=0.01)
        self.get_logger().info(f"Opened {port} @ {baudrate}")

        # Rate limiting
        self.last_sent_time = 0.0

        # ROS interfaces
        self.sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, qos_profile_sensor_data
        )

        self.pub = self.create_publisher(Odometry, "/odom_uart", 10)

        # Timer for reading UART
        self.timer = self.create_timer(0.01, self.read_serial)

    # -------------------------
    # Helpers
    # -------------------------

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    # -------------------------
    # ROS → UART
    # -------------------------

    def odom_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_sent_time < 1.0 / self.rate_limit:
            return
        self.last_sent_time = now

        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        payload = {
            "x": round(msg.pose.pose.position.x, 3),
            "y": round(msg.pose.pose.position.y, 3),
            "yaw": round(yaw, 3),
            "vx": round(msg.twist.twist.linear.x, 3),
            "wz": round(msg.twist.twist.angular.z, 3),
        }

        try:
            line = json.dumps(payload) + "\n"
            self.ser.write(line.encode("utf-8"))
        except Exception as e:
            self.get_logger().error(f"UART write failed: {e}")

    # -------------------------
    # UART → ROS
    # -------------------------

    def read_serial(self):
        try:
            data = self.ser.read(self.ser.in_waiting or 1).decode(
                "utf-8", errors="ignore"
            )
            if not data:
                return

            self.rx_buffer += data

            while "\n" in self.rx_buffer:
                line, self.rx_buffer = self.rx_buffer.split("\n", 1)
                line = line.strip()

                # ---- FILTER STAGE ----
                if not line:
                    continue

                # quick sanity check (reject obvious garbage early)
                if "{" not in line or "}" not in line:
                    continue

                # optional: reject lines that are too small/large
                if len(line) < 10 or len(line) > 500:
                    continue

                # ---- PARSE STAGE ----
                try:
                    data = json.loads(line)
                except json.JSONDecodeError:
                    continue  # silently ignore garbage

                # ---- VALIDATE CONTENT ----
                if not isinstance(data, dict):
                    continue

                # ---- BUILD MESSAGE ----
                msg = Odometry()

                msg.pose.pose.position.x = data.get("x", 0.0)
                msg.pose.pose.position.y = data.get("y", 0.0)

                yaw = data.get("yaw", 0.0)
                msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
                msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

                msg.twist.twist.linear.x = data.get("vx", 0.0)
                msg.twist.twist.angular.z = data.get("wz", 0.0)

                self.pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"UART read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = OdomUARTBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
