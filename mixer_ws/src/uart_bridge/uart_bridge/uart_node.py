import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class UARTBridge(Node):

    def __init__(self):
        super().__init__('uart_bridge')

        # Parameters (optional but useful)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Open serial port
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.get_logger().info(f"Opened {port} at {baudrate}")

        # Publisher (UART → ROS)
        self.publisher_ = self.create_publisher(String, 'uart_rx', 10)

        # Subscriber (ROS → UART)
        self.subscription = self.create_subscription(
            String,
            'uart_tx',
            self.tx_callback,
            10
        )

        # Timer to poll serial
        self.timer = self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode('utf-8', errors='ignore').strip()
            msg = String()
            msg.data = data
            self.publisher_.publish(msg)

    def tx_callback(self, msg):
        data = msg.data + '\n'
        self.ser.write(data.encode('utf-8'))


def main(args=None):
    rclpy.init(args=args)
    node = UARTBridge()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
