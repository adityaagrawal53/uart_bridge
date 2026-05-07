import math
import struct

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LidarUARTBridge(Node):
    def __init__(self):
        super().__init__("lidar_uart_bridge")

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("scan_topic_in", "/scan")
        self.declare_parameter("scan_topic_out", "/scan_uart")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("downsample_step", 3)

        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value

        self.rate_hz = self.get_parameter("rate_hz").value
        self.downsample_step = self.get_parameter("downsample_step").value

        # -------------------------------------------------
        # Serial
        # -------------------------------------------------
        self.ser = serial.Serial(port, baudrate, timeout=0.01)

        self.get_logger().info(f"Opened UART {port} @ {baudrate}")

        self.rx_buffer = b""
        self.last_sent_time = 0.0

        # -------------------------------------------------
        # ROS
        # -------------------------------------------------
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic_in").value,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.scan_pub = self.create_publisher(
            LaserScan, self.get_parameter("scan_topic_out").value, 10
        )

        self.tx_debug_pub = self.create_publisher(String, "/uart_tx_raw", 10)
        self.rx_debug_pub = self.create_publisher(String, "/uart_rx_raw", 10)

        self.timer = self.create_timer(0.005, self.read_serial)

    # =====================================================
    # Debug helper
    # =====================================================
    def publish_debug(self, pub, tag, data):
        msg = String()
        msg.data = f"{tag} len={len(data)} hex={data.hex()}"
        pub.publish(msg)

    # =====================================================
    # ROS -> UART
    # =====================================================
    def scan_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_sent_time < 1.0 / self.rate_hz:
            return
        self.last_sent_time = now

        ranges = msg.ranges[:: self.downsample_step]

        compressed = []
        for r in ranges:
            if math.isinf(r) or math.isnan(r) or r < 0:
                compressed.append(0)
            elif r > 65.0:
                compressed.append(65535)
            else:
                compressed.append(int(r * 1000.0))

        try:
            header = struct.pack(
                "<Hfff",
                len(compressed),
                msg.angle_min,
                msg.angle_max,
                msg.angle_increment * self.downsample_step,
            )

            payload = struct.pack("<" + "H" * len(compressed), *compressed)

            body = header + payload

            packet = struct.pack("<HH", 0xAA55, len(body)) + body

            self.ser.write(packet)

            self.publish_debug(self.tx_debug_pub, "TX", packet)

        except Exception as e:
            self.get_logger().error(f"TX error: {e}")

    # =====================================================
    # UART -> ROS (ROBUST STATEFUL PARSER)
    # =====================================================
    def read_serial(self):
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                self.rx_buffer += data

            while True:
                # Need at least sync + length
                if len(self.rx_buffer) < 4:
                    return

                # -------------------------------------------------
                # STRICT SYNC CHECK (NO BYTE GUESSING)
                # -------------------------------------------------
                sync = struct.unpack("<H", self.rx_buffer[:2])[0]

                if sync != 0xAA55:
                    self.rx_buffer = self.rx_buffer[1:]
                    continue

                length = struct.unpack("<H", self.rx_buffer[2:4])[0]

                # -------------------------------------------------
                # VALIDATE LENGTH EARLY
                # -------------------------------------------------
                if length < 16 or length > 20000:
                    self.rx_buffer = self.rx_buffer[2:]
                    continue

                total_size = 4 + length

                if len(self.rx_buffer) < total_size:
                    return

                packet = self.rx_buffer[4:total_size]
                self.rx_buffer = self.rx_buffer[total_size:]

                # -------------------------------------------------
                # BASIC HEADER VALIDATION
                # -------------------------------------------------
                if len(packet) < 14:
                    continue

                try:
                    count = struct.unpack("<H", packet[:2])[0]

                    if count == 0 or count > 4000:
                        continue

                    expected = 14 + (count * 2)

                    if len(packet) != expected:
                        continue

                    count, angle_min, angle_max, angle_inc = struct.unpack(
                        "<Hfff", packet[:14]
                    )

                    ranges_raw = struct.unpack("<" + ("H" * count), packet[14:])

                    msg = LaserScan()

                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "lidar_uart"

                    msg.angle_min = angle_min
                    msg.angle_max = angle_max
                    msg.angle_increment = angle_inc

                    msg.scan_time = 0.0
                    msg.time_increment = 0.0

                    msg.range_min = 0.15
                    msg.range_max = 65.0

                    msg.ranges = []

                    for r in ranges_raw:
                        v = r / 1000.0
                        msg.ranges.append(
                            v if 0.0 < v < msg.range_max else float("inf")
                        )

                    self.scan_pub.publish(msg)

                    self.publish_debug(self.rx_debug_pub, "RX_OK", packet)

                except Exception:
                    # resync aggressively if parsing fails
                    self.rx_buffer = self.rx_buffer[1:]
                    continue

        except Exception as e:
            self.get_logger().error(f"RX error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LidarUARTBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
