#!/usr/bin/env python3
"""
Compare /scan and /scan_uart by matching messages using header.stamp.
Subscribes with qos_profile_sensor_data (BEST_EFFORT) to avoid incompatibility.

Usage:
  python3 compare_scans.py [--orig /scan] [--rx /scan_uart] [--window 5.0]

The node matches messages that have identical header.stamp (sec,nanosec) and
computes mean/max absolute differences, counts of infinite/invalid samples, and
percentage within thresholds (1cm,5cm,10cm).
"""

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
        self.orig_topic = orig_topic
        self.rx_topic = rx_topic
        self.window_sec = float(window_sec)

        # stores: key -> (msg, recv_time_seconds)
        self.orig_msgs = {}
        self.rx_msgs = {}

        self.sub_orig = self.create_subscription(
            LaserScan, orig_topic, self.cb_orig, qos_profile_sensor_data
        )
        self.sub_rx = self.create_subscription(
            LaserScan, rx_topic, self.cb_rx, qos_profile_sensor_data
        )

        # periodic cleanup of unmatched messages
        self.create_timer(1.0, self._cleanup)

        self.pair_count = 0

        self.get_logger().info(
            f"Listening for {orig_topic} and {rx_topic}; matching window={self.window_sec}s"
        )

    @staticmethod
    def _stamp_key(msg: LaserScan):
        try:
            s = int(msg.header.stamp.sec)
            ns = int(msg.header.stamp.nanosec)
        except Exception:
            # fallback to now when stamp missing
            now = rclpy.clock.Clock().now().to_msg()
            s = int(now.sec)
            ns = int(now.nanosec)
        return (s, ns)

    def cb_orig(self, msg: LaserScan):
        key = self._stamp_key(msg)
        self.orig_msgs[key] = (msg, self.get_clock().now().nanoseconds / 1e9)
        if key in self.rx_msgs:
            rx_msg, _ = self.rx_msgs.pop(key)
            orig_msg, _ = self.orig_msgs.pop(key)
            self._compare_pair(orig_msg, rx_msg)

    def cb_rx(self, msg: LaserScan):
        key = self._stamp_key(msg)
        self.rx_msgs[key] = (msg, self.get_clock().now().nanoseconds / 1e9)
        if key in self.orig_msgs:
            orig_msg, _ = self.orig_msgs.pop(key)
            rx_msg, _ = self.rx_msgs.pop(key)
            self._compare_pair(orig_msg, rx_msg)

    def _compare_pair(self, orig: LaserScan, rx: LaserScan):
        # Compare ranges element-wise up to the minimum length
        n_orig = len(orig.ranges)
        n_rx = len(rx.ranges)
        n = min(n_orig, n_rx)

        diffs = []
        inf_inf = 0
        inf_mismatch = 0
        count_valid = 0

        for i in range(n):
            a = orig.ranges[i]
            b = rx.ranges[i]
            if math.isinf(a) and math.isinf(b):
                inf_inf += 1
                continue
            if math.isinf(a) != math.isinf(b):
                inf_mismatch += 1
                # treat as large difference but include in stats as NaN/infinite
                count_valid += 1
                diffs.append(float("inf") if math.isinf(a) else abs(a - b))
                continue
            # both finite
            diff = abs(a - b)
            diffs.append(diff)
            count_valid += 1

        finite = [d for d in diffs if math.isfinite(d)]
        mean = statistics.mean(finite) if finite else float("nan")
        mx = max(finite) if finite else float("nan")
        within_1cm = (
            (sum(1 for d in finite if d <= 0.01) / len(finite) * 100) if finite else 0.0
        )
        within_5cm = (
            (sum(1 for d in finite if d <= 0.05) / len(finite) * 100) if finite else 0.0
        )
        within_10cm = (
            (sum(1 for d in finite if d <= 0.1) / len(finite) * 100) if finite else 0.0
        )

        self.pair_count += 1
        stamp = orig.header.stamp
        self.get_logger().info(
            f"Pair#{self.pair_count} stamp={stamp.sec}.{stamp.nanosec} orig_len={n_orig} rx_len={n_rx} "
            f"compared={len(finite)} mean_abs_diff={mean:.6f} m max_abs_diff={mx:.6f} m "
            f"inf_inf={inf_inf} inf_mismatch={inf_mismatch} within1cm={within_1cm:.1f}% within5cm={within_5cm:.1f}% within10cm={within_10cm:.1f}%"
        )

        # report some large diffs (first up to 10)
        large = []
        for i in range(n):
            if i >= n:
                break
            a = orig.ranges[i]
            b = rx.ranges[i]
            if math.isfinite(a) and math.isfinite(b):
                d = abs(a - b)
                if d > 0.1:  # threshold for 'large'
                    large.append((i, d))
        if large:
            s = ",".join(f"{i}:{d:.3f}" for i, d in large[:10])
            self.get_logger().info(f"Large diffs (index:diff m) first10: {s}")

    def _cleanup(self):
        now = self.get_clock().now().nanoseconds / 1e9
        expire = now - self.window_sec
        for d in (self.orig_msgs, self.rx_msgs):
            for k in list(d.keys()):
                if d[k][1] < expire:
                    del d[k]


def main(argv=None):
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument("--orig", default="/scan", help="original scan topic")
    parser.add_argument("--rx", default="/scan_uart", help="reconstructed scan topic")
    parser.add_argument(
        "--window", type=float, default=5.0, help="matching window seconds"
    )
    args = parser.parse_args(argv[1:] if argv else sys.argv[1:])

    node = ScanComparator(
        orig_topic=args.orig, rx_topic=args.rx, window_sec=args.window
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
