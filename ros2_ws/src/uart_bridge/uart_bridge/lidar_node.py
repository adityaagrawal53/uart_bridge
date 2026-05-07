import binascii
import math
import struct
import time

import rclpy
import serial
from cobs import cobs
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

SYNC = b"\xaa\x55"
# header: seq (I), stamp_sec (I), stamp_nsec (I), angle_min (f), angle_increment (f),
# time_increment (f), scan_time (f), range_min (f), range_max (f),
# original_count (H), step (H), count (H)
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

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 460800)
        self.declare_parameter("scan_topic_in", "/scan")
        self.declare_parameter("scan_topic_out", "/scan_uart")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("downsample_step", 2)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.step = int(self.get_parameter("downsample_step").value) or 1
        if self.step < 1:
            self.step = 1

        # -------------------------
        # Serial
        # -------------------------
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.get_logger().info(f"UART opened {self.port} @ {self.baudrate}")

        self.rx_buffer = b""
        self.last_tx = 0.0
        self.seq = 0

        # -------------------------
        # ROS
        # -------------------------
        self.sub = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic_in").value,
            self.tx_cb,
            qos_profile_sensor_data,
        )

        self.pub = self.create_publisher(
            LaserScan,
            self.get_parameter("scan_topic_out").value,
            qos_profile_sensor_data,
        )

        # raw debug publishers (TX publishes the framed packet; RX will publish framed packet too)
        self.tx_dbg = self.create_publisher(String, "/uart_tx_raw", 10)
        self.rx_dbg = self.create_publisher(String, "/uart_rx_raw", 10)

        # parsing timer
        self.timer = self.create_timer(0.002, self.rx_loop)

        # heartbeat so you always see rx activity
        self.debug_timer = self.create_timer(1.0, self.debug_heartbeat)

    # -------------------------
    # Debug helper
    # -------------------------
    def dbg(self, pub, tag, data):
        try:
            msg = String()
            msg.data = f"{tag} len={len(data)} hex={data[:48].hex()}"
            pub.publish(msg)
        except Exception:
            pass

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

        # original full count
        orig_count = len(msg.ranges)

        # downsample ranges
        ranges_list = list(msg.ranges)[:: self.step]

        packed = []
        for r in ranges_list:
            if math.isnan(r) or math.isinf(r) or r < 0:
                packed.append(0)
            else:
                v = int(r * 1000)
                packed.append(min(max(v, 0), 0xFFFF))

        count = len(packed)
        if count > MAX_POINTS:
            self.get_logger().warning(f"Too many points to send: {count}")
            return

        # send original angle_increment so receiver can reconstruct angles
        angle_min = float(msg.angle_min)
        orig_angle_inc = float(msg.angle_increment)

        time_increment = float(getattr(msg, "time_increment", 0.0))
        scan_time = float(getattr(msg, "scan_time", 0.0))
        range_min = float(getattr(msg, "range_min", 0.0))
        range_max = float(getattr(msg, "range_max", 0.0))

        # pack header with original_count and step so RX can expand back to original size
        # include original timestamp (sec, nsec)
        try:
            stamp_sec = int(msg.header.stamp.sec)
            stamp_nsec = int(msg.header.stamp.nanosec)
        except Exception:
            t = time.time()
            stamp_sec = int(t)
            stamp_nsec = int((t - stamp_sec) * 1e9)
        header = struct.pack(
            HEADER_FMT,
            int(self.seq) & 0xFFFFFFFF,
            int(stamp_sec) & 0xFFFFFFFF,
            int(stamp_nsec) & 0xFFFFFFFF,
            angle_min,
            orig_angle_inc,
            time_increment,
            scan_time,
            range_min,
            range_max,
            int(orig_count) & 0xFFFF,
            int(self.step) & 0xFFFF,
            int(count) & 0xFFFF,
        )

        if count:
            body = header + struct.pack("<" + "H" * count, *packed)
        else:
            body = header

        encoded = cobs.encode(body)

        if len(encoded) == 0 or len(encoded) > MAX_PACKET_SIZE:
            self.get_logger().warning(
                f"Encoded packet size out of range: {len(encoded)}"
            )
            return

        crc = struct.pack("<H", crc16(encoded))

        packet = SYNC + struct.pack("<H", len(encoded)) + encoded + crc

        try:
            self.ser.write(packet)
            self.dbg(self.tx_dbg, "TX", packet)
            self.seq = (self.seq + 1) & 0xFFFFFFFF
        except Exception as e:
            self.get_logger().warning(f"TX error: {e}")

    # =========================================================
    # RX
    # =========================================================
    def rx_loop(self):
        try:
            available = (
                self.ser.in_waiting if getattr(self.ser, "in_waiting", 0) > 0 else 1
            )
            data = self.ser.read(available)
        except Exception as e:
            self.get_logger().warning(f"Serial read error: {e}")
            return

        if data:
            self.rx_buffer += data

        if len(self.rx_buffer) > MAX_BUFFER_SIZE:
            self.get_logger().warning("rx_buffer exceeded max size, trimming")
            self.rx_buffer = self.rx_buffer[-MAX_BUFFER_SIZE:]

        while True:
            buf = self.rx_buffer
            idx = buf.find(SYNC)

            if idx < 0:
                if len(buf) > 1:
                    self.rx_buffer = buf[-1:]
                return

            if len(buf) < idx + 4:
                if idx > 0:
                    self.rx_buffer = buf[idx:]
                return

            enc_len = struct.unpack("<H", buf[idx + 2 : idx + 4])[0]

            if enc_len == 0 or enc_len > MAX_PACKET_SIZE:
                self.get_logger().warning(
                    f"Invalid encoded length: {enc_len}; resyncing"
                )
                self.rx_buffer = buf[idx + 1 :]
                continue

            total_packet_len = 2 + 2 + enc_len + 2

            if len(buf) < idx + total_packet_len:
                return

            packet = buf[idx : idx + total_packet_len]
            encoded = packet[4 : 4 + enc_len]
            crc_recv = struct.unpack("<H", packet[4 + enc_len : 4 + enc_len + 2])[0]

            # advance buffer past this packet
            self.rx_buffer = buf[idx + total_packet_len :]

            # publish the framed packet (so /uart_rx_raw matches /uart_tx_raw)
            try:
                self.dbg(self.rx_dbg, "RX", packet)
            except Exception:
                pass

            if crc16(encoded) != crc_recv:
                self.get_logger().warning("CRC mismatch, dropping packet")
                continue

            try:
                decoded = cobs.decode(encoded)
            except Exception as e:
                self.get_logger().warning(f"COBS decode failed: {e}")
                continue

            if len(decoded) < HEADER_SIZE:
                self.get_logger().warning("Decoded packet too short for header")
                continue

            try:
                (
                    seq,
                    stamp_sec,
                    stamp_nsec,
                    angle_min,
                    angle_inc_orig,
                    time_increment,
                    scan_time,
                    range_min,
                    range_max,
                    original_count,
                    step,
                    count,
                ) = struct.unpack(HEADER_FMT, decoded[:HEADER_SIZE])
            except Exception as e:
                self.get_logger().warning(f"Header unpack failed: {e}")
                continue

            if count > MAX_POINTS:
                self.get_logger().warning(f"Point count too large: {count}")
                continue

            expected = HEADER_SIZE + int(count) * 2
            if len(decoded) < expected:
                self.get_logger().warning(
                    f"Decoded packet too short for {count} points (len={len(decoded)})"
                )
                continue

            try:
                if count:
                    packed = struct.unpack(
                        "<" + "H" * int(count), decoded[HEADER_SIZE:expected]
                    )
                else:
                    packed = []

                # reconstruct full-length packed array using original_count and step
                orig_count = int(original_count) if original_count > 0 else 0
                step = int(step) if step > 0 else 1

                if orig_count <= 0:
                    # if original count not provided/sane, fall back to packed-array interpretation
                    full_packed = list(packed) if packed else []
                else:
                    full_packed = [0] * orig_count
                    for i, v in enumerate(packed):
                        idx2 = i * step
                        if idx2 < orig_count:
                            full_packed[idx2] = v

                msg = LaserScan()
                # reuse original timestamp from the transmitted scan
                try:
                    msg.header.stamp.sec = int(stamp_sec)
                    msg.header.stamp.nanosec = int(stamp_nsec)
                except Exception:
                    msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "lidar_uart"

                msg.angle_min = float(angle_min)
                msg.angle_increment = float(angle_inc_orig)
                if orig_count > 0:
                    msg.angle_max = msg.angle_min + msg.angle_increment * (
                        orig_count - 1
                    )
                else:
                    msg.angle_max = msg.angle_min

                msg.time_increment = float(time_increment)
                msg.scan_time = float(scan_time)

                msg.range_min = float(range_min)
                msg.range_max = float(range_max)

                msg.ranges = [
                    r / 1000.0 if r > 0 else float("inf") for r in full_packed
                ]
                msg.intensities = [0.0] * len(msg.ranges)

                self.pub.publish(msg)
                try:
                    self.dbg(self.rx_dbg, "RX_OK", decoded)
                except Exception:
                    pass

            except Exception as e:
                self.get_logger().warning(f"Failed to unpack/publish scan: {e}")
                continue

    def destroy_node(self):
        try:
            if self.ser and getattr(self.ser, "is_open", False):
                try:
                    self.ser.close()
                except Exception:
                    pass
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = LidarUARTBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
