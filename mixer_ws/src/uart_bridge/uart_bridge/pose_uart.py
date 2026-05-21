import json

import rclpy
import serial
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time


class AmclPoseUARTBridge(Node):
    def __init__(self):
        super().__init__("amcl_pose_uart_bridge")
        self.rx_buffer = ""
        self.tx_seq = 0

        # Parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 460800)
        self.declare_parameter("rate_limit_hz", 5.0)
        self.declare_parameter("tag", 1)
        self.declare_parameter("input_topic", "/amcl_pose")
        self.declare_parameter("output_topic", "/amcl_pose_uart")
        self.declare_parameter("frame_id", "map")

        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.rate_limit = (
            self.get_parameter("rate_limit_hz").get_parameter_value().double_value
        )
        self.tag = self.get_parameter("tag").get_parameter_value().integer_value
        self.input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        self.output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )

        if self.rate_limit <= 0.0:
            raise ValueError("rate_limit_hz must be > 0")

        # Serial
        self.ser = serial.Serial(port, baudrate, timeout=0.01)
        self.get_logger().info(f"Opened {port} @ {baudrate}")

        # Rate limiting
        self.last_sent_time = 0.0

        # ROS interfaces
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.input_topic,
            self.pose_callback,
            qos_profile_sensor_data,
        )
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, self.output_topic, 10
        )

        # Timer for reading UART
        self.timer = self.create_timer(0.01, self.read_serial)

    # -------------------------
    # Helpers
    # -------------------------

    @staticmethod
    def _stamp_from_ns(t_ns: int):
        stamp = Time(nanoseconds=t_ns).to_msg()
        return stamp

    def _build_payload(self, msg: PoseWithCovarianceStamped):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        return {
            "tag": self.tag,
            "seq": self.tx_seq,
            "t_ns": int(self.get_clock().now().nanoseconds),
            "x": position.x,
            "y": position.y,
            "z": position.z,
            "qx": orientation.x,
            "qy": orientation.y,
            "qz": orientation.z,
            "qw": orientation.w,
        }

    # -------------------------
    # ROS → UART
    # -------------------------

    def pose_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_sent_time < 1.0 / self.rate_limit:
            return
        self.last_sent_time = now

        payload = self._build_payload(msg)

        try:
            line = json.dumps(payload) + "\n"
            self.ser.write(line.encode("utf-8"))
            self.tx_seq += 1
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

                if int(data.get("tag", -1)) != self.tag:
                    continue

                # ---- BUILD MESSAGE ----
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = self.frame_id

                t_ns = int(data.get("t_ns", 0))
                msg.header.stamp = self._stamp_from_ns(t_ns)

                msg.pose.pose.position.x = float(data.get("x", 0.0))
                msg.pose.pose.position.y = float(data.get("y", 0.0))
                msg.pose.pose.position.z = float(data.get("z", 0.0))
                msg.pose.pose.orientation.x = float(data.get("qx", 0.0))
                msg.pose.pose.orientation.y = float(data.get("qy", 0.0))
                msg.pose.pose.orientation.z = float(data.get("qz", 0.0))
                msg.pose.pose.orientation.w = float(data.get("qw", 1.0))

                self.pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"UART read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AmclPoseUARTBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
