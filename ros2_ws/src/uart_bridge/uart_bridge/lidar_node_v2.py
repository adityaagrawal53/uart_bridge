import binascii
import math
import struct
import time
from typing import cast

import rclpy
import serial
from cobs import cobs
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

SYNC = b"\xaa\x55"

HEADER_FMT = "<III6f3H"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

MAX_PACKET_SIZE = 65536
MAX_BUFFER_SIZE = 131072
MAX_POINTS = 20000


def crc16(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF)


class LidarUARTBridge(Node):
    def __init__(self):
        super().__init__("lidar_uart_bridge")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 460800)
        self.declare_parameter("scan_topic_in", "/scan")
        self.declare_parameter("scan_topic_out", "/scan_uart")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("downsample_step", 2)

        self.port = cast(str, self.get_parameter("port").value)
        self.baudrate = cast(int, self.get_parameter("baudrate").value)

        self.rate_hz = cast(float, self.get_parameter("rate_hz").value)
        self.step = cast(int, self.get_parameter("downsample_step").value) or 1
        if self.step < 1:
            self.step = 1

        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.get_logger().info(f"UART opened {self.port} @ {self.baudrate}")

        self.rx_buffer = b""
        self.last_tx = 0.0
        self.seq = 0

        self.sub = self.create_subscription(
            LaserScan,
            cast(str, self.get_parameter("scan_topic_in").value),
            self.tx_cb,
            qos_profile_sensor_data,
        )

        self.pub = self.create_publisher(
            LaserScan,
            cast(str, self.get_parameter("scan_topic_out").value),
            qos_profile_sensor_data,
        )

        self.tx_dbg = self.create_publisher(String, "/uart_tx_raw", 10)
        self.rx_dbg = self.create_publisher(String, "/uart_rx_raw", 10)

        self.timer = self.create_timer(0.002, self.rx_loop)
        self.debug_timer = self.create_timer(1.0, self.debug_heartbeat)

    def dbg(self, pub, tag, data):
        msg = String()
        msg.data = f"{tag} len={len(data)} hex={data[:48].hex()}"
        pub.publish(msg)

    def debug_heartbeat(self):
        msg = String()
        msg.data = f"RX buffer size={len(self.rx_buffer)}"
        self.rx_dbg.publish(msg)

    # =========================================================
    # TX
    # =========================================================
    def tx_cb(self, msg: LaserScan):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_tx < 1.0 / max(0.1, self.rate_hz):
            return
        self.last_tx = now

        original_count = len(msg.ranges)
        ranges = list(msg.ranges)[:: self.step]
        intensities = list(getattr(msg, "intensities", []))[:: self.step]

        count = len(ranges)
        if count == 0 or count > MAX_POINTS:
            return

        if len(intensities) < count:
            intensities += [0.0] * (count - len(intensities))
        else:
            intensities = intensities[:count]

        packed_ranges = []
        for r in ranges:
            if math.isnan(r) or math.isinf(r) or r < 0:
                packed_ranges.append(0)
            else:
                packed_ranges.append(min(max(int(r * 1000), 0), 0xFFFF))

        # -----------------------------
        # FIX: float32 intensities
        # -----------------------------
        packed_ints = [float(i) for i in intensities]

        try:
            stamp_sec = int(msg.header.stamp.sec)
            stamp_nsec = int(msg.header.stamp.nanosec)
        except Exception:
            t = time.time()
            stamp_sec = int(t)
            stamp_nsec = int((t - stamp_sec) * 1e9)

        header = struct.pack(
            HEADER_FMT,
            self.seq & 0xFFFFFFFF,
            stamp_sec,
            stamp_nsec,
            float(msg.angle_min),
            float(msg.angle_increment),
            float(getattr(msg, "time_increment", 0.0)),
            float(getattr(msg, "scan_time", 0.0)),
            float(getattr(msg, "range_min", 0.0)),
            float(getattr(msg, "range_max", 0.0)),
            original_count & 0xFFFF,
            self.step & 0xFFFF,
            count,
        )

        body = (
            header
            + struct.pack("<" + "H" * count, *packed_ranges)
            + struct.pack("<" + "f" * count, *packed_ints)  # <-- KEY FIX
        )

        encoded = cobs.encode(body)
        crc = struct.pack("<H", crc16(encoded))

        packet = SYNC + struct.pack("<H", len(encoded)) + encoded + crc

        self.ser.write(packet)
        self.seq += 1

    # =========================================================
    # RX
    # =========================================================
    def rx_loop(self):
        data = self.ser.read(self.ser.in_waiting or 1)
        if data:
            self.rx_buffer += data

        while True:
            buf = self.rx_buffer
            idx = buf.find(SYNC)
            if idx < 0:
                return

            if len(buf) < idx + 4:
                return

            enc_len = struct.unpack("<H", buf[idx + 2 : idx + 4])[0]
            total = 4 + enc_len + 2

            if len(buf) < idx + total:
                return

            packet = buf[idx : idx + total]
            encoded = packet[4 : 4 + enc_len]
            crc_recv = struct.unpack("<H", packet[4 + enc_len : 4 + enc_len + 2])[0]

            self.rx_buffer = buf[idx + total :]

            if crc16(encoded) != crc_recv:
                continue

            decoded = cobs.decode(encoded)

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
            ) = struct.unpack(HEADER_FMT, decoded[:HEADER_SIZE])

            expected_len = HEADER_SIZE + (2 * count) + (4 * count)

            if len(decoded) != expected_len:
                self.get_logger().warning(
                    f"SIZE MISMATCH: expected={expected_len} got={len(decoded)} count={count}"
                )
                return

            offset = HEADER_SIZE

            ranges = struct.unpack(
                "<" + "H" * count,
                decoded[offset : offset + 2 * count],
            )
            offset += 2 * count

            # -----------------------------
            # FIX: float32 intensities
            # -----------------------------
            intensities = struct.unpack(
                "<" + "f" * count,
                decoded[offset : offset + 4 * count],
            )

            msg = LaserScan()
            msg.header.stamp.sec = stamp_sec
            msg.header.stamp.nanosec = stamp_nsec
            msg.header.frame_id = "lidar_uart"

            downsample_step = max(1, int(step))

            msg.angle_min = angle_min
            msg.angle_increment = angle_inc * downsample_step
            msg.time_increment = time_increment * downsample_step
            msg.scan_time = scan_time
            msg.range_min = range_min
            msg.range_max = range_max

            msg.ranges = [r / 1000.0 if r > 0 else float("inf") for r in ranges]
            msg.intensities = list(intensities)

            self.pub.publish(msg)

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
