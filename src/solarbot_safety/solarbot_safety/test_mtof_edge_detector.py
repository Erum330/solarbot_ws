#!/usr/bin/env python3
"""
calibrate_mtof_edge_node.py (Calibrated Version)
"""

import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class CalibrateMtofSingleShotNode(Node):
    def __init__(self):
        super().__init__('calibrate_mtof_edge_node')

        self.declare_parameter('surface_threshold_m', 0.18)
        self.declare_parameter('transpose_grid',      True)
        self.declare_parameter('flip_vertical',       True)

        self.thresh = float(self.get_parameter('surface_threshold_m').value)
        self.transpose = bool(self.get_parameter('transpose_grid').value)
        self.flip_vertical = bool(self.get_parameter('flip_vertical').value)

        # Calibrated model constants
        self.slope_m_per_col = 0.0325   # 3.25 cm / column
        self.base_offset_m   = 0.0940   # 9.40 cm base offset

        self.latest_msg = None
        self.reading_idx = 0

        self.create_subscription(
            PointCloud2,
            '/right_mid_tof/points',
            self._cloud_cb,
            qos_profile=qos_profile_sensor_data
        )

        self.get_logger().info('🔬 Calibrated Single-Shot MToF Node Ready.')

    def _cloud_cb(self, msg: PointCloud2):
        self.latest_msg = msg

    def _subpixel_edge(self, row):
        for c in range(7):
            d1 = row[c]
            d2 = row[c + 1]
            if d1 <= self.thresh and d2 > self.thresh:
                fraction = (self.thresh - d1) / max(1e-4, (d2 - d1))
                return float(c) + float(fraction)
        if row[0] > self.thresh:
            return 0.0
        return None

    def capture_and_display(self):
        if self.latest_msg is None:
            print("\n⏳ Waiting for PointCloud2 message...")
            return

        pts = list(point_cloud2.read_points(self.latest_msg, field_names=('x', 'y', 'z'), skip_nans=False))
        if len(pts) < 64:
            print("⚠️ Incomplete point cloud (< 64 points).")
            return

        self.reading_idx += 1

        raw_grid = np.zeros((8, 8), dtype=np.float32)
        for r in range(8):
            for c in range(8):
                x, y, z = pts[r * 8 + c]
                dist = math.sqrt(x*x + y*y + z*z) if (math.isfinite(x) and x > 0) else 9.99
                raw_grid[r, c] = dist

        grid = raw_grid.T if self.transpose else raw_grid
        if self.flip_vertical:
            grid = np.flipud(grid)

        edge_pts_2d = []
        for r in range(8):
            col_cross = self._subpixel_edge(grid[r, :])
            if col_cross is not None:
                edge_pts_2d.append([col_cross, float(r)])

        if len(edge_pts_2d) >= 3:
            pts_array = np.array(edge_pts_2d, dtype=np.float32)
            [vx, vy, x0, y0] = cv2.fitLine(pts_array, cv2.DIST_L2, 0, 0.01, 0.01)
            vx, vy, x0, y0 = float(vx[0]), float(vy[0]), float(x0[0]), float(y0[0])
            if vy < 0:
                vx = -vx
                vy = -vy

            avg_col = x0
            tilt_deg = math.degrees(math.atan2(vx, vy))

            # Calibrated physical distance
            measured_dist_m = (self.slope_m_per_col * avg_col) + self.base_offset_m
            measured_dist_cm = measured_dist_m * 100.0

            print(f"\n{'='*55}")
            print(f" 📸 READING #{self.reading_idx}")
            print(f" -> Subpixel Edge Column   : {avg_col:5.2f} (Cols 0 to 7)")
            print(f" -> CALIBRATED DISTANCE    : {measured_dist_cm:5.1f} cm ({measured_dist_m:6.3f} m)")
            print(f" -> Edge Tilt Relative     : {tilt_deg:+5.1f}°")
            print(f"{'='*55}\n")
        else:
            print("\n⚠️ No valid edge found across sensor grid.")


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateMtofSingleShotNode()

    try:
        while rclpy.ok():
            input("👉 Press [ENTER] to read distance...")
            rclpy.spin_once(node, timeout_sec=0.1)
            rclpy.spin_once(node, timeout_sec=0.1)
            node.capture_and_display()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()