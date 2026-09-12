#!/usr/bin/env python3
"""
edge_logger_node.py

Offline Trajectory & Edge Logger:
- Subscribes to /edge_distance (Float32), /edge_tilt (Float32), and /imu (Imu).
- Computes Euler yaw angle (-180° to +180°).
- Writes synchronized timestamped records into a CSV file.
- Periodic terminal status updates for quick field verification.
"""

import csv
from datetime import datetime
import math
import os
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu


class EdgeLoggerNode(Node):
    def __init__(self):
        super().__init__('edge_logger_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('log_dir', os.path.expanduser('~'))
        self.declare_parameter('sample_rate_hz', 20.0)

        log_dir = str(self.get_parameter('log_dir').value)
        sample_hz = float(self.get_parameter('sample_rate_hz').value)

        # ---------------- CSV File Initialization ----------------
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        os.makedirs(log_dir, exist_ok=True)
        self.filepath = os.path.join(log_dir, f'solarbot_edge_log_{timestamp_str}.csv')

        self.csv_file = open(self.filepath, mode='w', newline='', buffering=1)
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp_sec',
            'edge_dist_m',
            'edge_dist_cm',
            'edge_tilt_deg',
            'imu_yaw_deg',
            'imu_yaw_rad'
        ])

        # ---------------- Cached Values ----------------
        self.latest_dist = None
        self.latest_tilt = None
        self.latest_yaw = None
        self.row_count = 0

        # ---------------- Subscriptions ----------------
        self.create_subscription(
            Float32, '/edge_distance', self._dist_cb, 10
        )
        self.create_subscription(
            Float32, '/edge_tilt', self._tilt_cb, 10
        )
        self.create_subscription(
            Imu, '/imu', self._imu_cb, qos_profile=qos_profile_sensor_data
        )

        # Periodic logging timer
        timer_period = 1.0 / max(1.0, sample_hz)
        self.create_timer(timer_period, self._write_row)
        self.create_timer(2.0, self._print_status)

        self.get_logger().info(f'📁 Edge Logger Node Started. Saving to:\n -> {self.filepath}')

    def _dist_cb(self, msg: Float32):
        self.latest_dist = float(msg.data)

    def _tilt_cb(self, msg: Float32):
        self.latest_tilt = float(msg.data)

    def _imu_cb(self, msg: Imu):
        q = msg.quaternion if hasattr(msg, 'quaternion') else msg.orientation
        if not (math.isfinite(q.x) and math.isfinite(q.y) and math.isfinite(q.z) and math.isfinite(q.w)):
            return

        # 4-quadrant Euler yaw conversion
        yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        self.latest_yaw = yaw

    def _write_row(self):
        # Ensure initial samples have arrived before writing
        if self.latest_dist is None or self.latest_yaw is None:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        dist_m = self.latest_dist
        dist_cm = dist_m * 100.0
        tilt_deg = self.latest_tilt if self.latest_tilt is not None else float('nan')
        yaw_rad = self.latest_yaw
        yaw_deg = math.degrees(yaw_rad)

        self.csv_writer.writerow([
            f'{now_sec:.3f}',
            f'{dist_m:.4f}',
            f'{dist_cm:.2f}',
            f'{tilt_deg:.2f}',
            f'{yaw_deg:.2f}',
            f'{yaw_rad:.4f}'
        ])
        self.row_count += 1

    def _print_status(self):
        if self.latest_dist is not None and self.latest_yaw is not None:
            tilt_str = f'{self.latest_tilt:+.2f}°' if self.latest_tilt is not None else 'N/A'
            self.get_logger().info(
                f'📊 Logged {self.row_count} rows | Dist: {self.latest_dist * 100.0:5.1f} cm | '
                f'Tilt: {tilt_str:>7} | Yaw: {math.degrees(self.latest_yaw):+6.1f}°'
            )

    def close(self):
        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
            self.get_logger().info(f'💾 File successfully saved ({self.row_count} records).')


def main(args=None):
    rclpy.init(args=args)
    node = EdgeLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()