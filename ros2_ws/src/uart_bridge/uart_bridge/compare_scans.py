#!/usr/bin/env python3

import argparse
import math
import statistics
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanComparator(Node):
    def __init__(self, orig_topic="/scan", rx_topic="/scan_uart", window_sec=5.0):
        super().__init__("scan_comparator")

        self.window_sec = float(window_sec)

        self.orig_msgs = {}
        self.rx_msgs = {}

        self.sub_orig = self.create_subscription(
            LaserScan, orig_topic, self.cb_orig, qos_profile_sensor_data
        )
        self.sub_rx = self.create_subscription(
            LaserScan, rx_topic, self.cb_rx, qos_profile_sensor_data
        )

        self.create_timer(1.0, self._cleanup)

        self.pair_count = 0

        self.get_logger().info(f"Comparing {orig_topic} ↔ {rx_topic} (angle-aligned)")

    # ------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------
    @staticmethod
    def stamp_key(msg: LaserScan):
        return (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))

    @staticmethod
    def angle(msg, i):
        return msg.angle_min + i * msg.angle_increment

    @staticmethod
    def find_nearest(rx: LaserScan, angle: float):
        best_i = -1
        best_d = float("inf")

        for i in range(len(rx.ranges)):
            a = rx.angle_min + i * rx.angle_increment
            d = abs(a - angle)
            if d < best_d:
                best_d = d
                best_i = i

        return best_i

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------
    def cb_orig(self, msg):
        k = self.stamp_key(msg)
        self.orig_msgs[k] = (msg, self.now())

        if k in self.rx_msgs:
            self._compare(self.orig_msgs.pop(k)[0], self.rx_msgs.pop(k)[0])

    def cb_rx(self, msg):
        k = self.stamp_key(msg)
        self.rx_msgs[k] = (msg, self.now())

        if k in self.orig_msgs:
            self._compare(self.orig_msgs.pop(k)[0], self.rx_msgs.pop(k)[0])

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------
    # Core comparison
    # ------------------------------------------------------------
    def _compare(self, orig: LaserScan, rx: LaserScan):
        range_diffs = []
        intensity_diffs = []

        inf_inf = 0
        inf_mismatch = 0

        # iterate over ORIGINAL scan only
        for i in range(len(orig.ranges)):
            a = orig.ranges[i]
            angle = self.angle(orig, i)

            j = self.find_nearest(rx, angle)
            if j < 0:
                continue

            b = rx.ranges[j]

            ai = orig.intensities[i] if i < len(orig.intensities) else 0.0
            bi = rx.intensities[j] if j < len(rx.intensities) else 0.0

            # ---------------- RANGE ----------------
            if math.isinf(a) and math.isinf(b):
                inf_inf += 1
            elif math.isinf(a) != math.isinf(b):
                inf_mismatch += 1
                range_diffs.append(float("inf"))
            else:
                range_diffs.append(abs(a - b))

            # ---------------- INTENSITY ----------------
            intensity_diffs.append(abs(float(ai) - float(bi)))

        finite_r = [d for d in range_diffs if math.isfinite(d)]
        finite_i = [d for d in intensity_diffs if math.isfinite(d)]

        def safe_mean(x):
            return statistics.mean(x) if x else float("nan")

        def safe_max(x):
            return max(x) if x else float("nan")

        mean_r = safe_mean(finite_r)
        max_r = safe_max(finite_r)

        mean_i = safe_mean(finite_i)
        max_i = safe_max(finite_i)

        int_match = (
            sum(1 for d in finite_i if d < 1e-3) / len(finite_i) * 100
            if finite_i
            else 0.0
        )

        self.pair_count += 1
        stamp = orig.header.stamp

        self.get_logger().info(
            f"Pair#{self.pair_count} "
            f"stamp={stamp.sec}.{stamp.nanosec} "
            f"orig_len={len(orig.ranges)} rx_len={len(rx.ranges)} "
            f"range_mean={mean_r:.6f}m range_max={max_r:.6f}m "
            f"int_mean_err={mean_i:.6f} int_max_err={max_i:.6f} "
            f"int_match={int_match:.2f}% "
            f"inf_inf={inf_inf} inf_mismatch={inf_mismatch}"
        )

    # ------------------------------------------------------------
    # cleanup
    # ------------------------------------------------------------
    def _cleanup(self):
        now = self.now()
        expire = now - self.window_sec

        for d in (self.orig_msgs, self.rx_msgs):
            for k in list(d.keys()):
                if d[k][1] < expire:
                    del d[k]


def main(argv=None):
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument("--orig", default="/scan")
    parser.add_argument("--rx", default="/scan_uart")
    parser.add_argument("--window", type=float, default=5.0)
    args = parser.parse_args(argv[1:] if argv else sys.argv[1:])

    node = ScanComparator(
        orig_topic=args.orig,
        rx_topic=args.rx,
        window_sec=args.window,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
