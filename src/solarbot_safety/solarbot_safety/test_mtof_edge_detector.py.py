#!/usr/bin/env python3
"""
test_mtof_edge_detector.py

Subscribes to /right_mid_tof/points (PointCloud2) from tof_bridge,
reconstructs the 8x8 depth grid, displays a terminal heatmap,
and detects the table edge transition across each row.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class MtofEdgeDetector(Node):
    def __init__(self):
        super().__init__('mtof_edge_detector')

        self.declare_parameter('surface_threshold_m', 0.08)  # Values > 8cm are open air (off table)
        self.declare_parameter('grid_size', 8)

        self.thresh = float(self.get_parameter('surface_threshold_m').value)
        self.grid_size = int(self.get_parameter('grid_size').value)

        self.create_subscription(
            PointCloud2,
            '/right_mid_tof/points',
            self._cloud_cb,
            qos_profile=qos_profile_sensor_data
        )

        self.get_logger().info('👁️ 8x8 Multi-ToF Edge Detector running...')

    def _cloud_cb(self, msg: PointCloud2):
        # Extract (x, y, z) points from the PointCloud2 stream
        pts = [p for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)]
        if len(pts) < self.grid_size * self.grid_size:
            return

        # Reconstruct 8x8 matrix of euclidean distances
        grid = np.zeros((self.grid_size, self.grid_size))
        for r in range(self.grid_size):
            for c in range(self.grid_size):
                idx = r * self.grid_size + c
                x, y, z = pts[idx]
                grid[r, c] = math.sqrt(x*x + y*y + z*z) if math.isfinite(x) else 9.99

        edge_cols = []
        # Find the transition index (Table -> Air) per row
        for r in range(self.grid_size):
            row = grid[r, :]
            # Locate first column where distance exceeds the surface threshold
            edge_idx = np.where(row > self.thresh)[0]
            if len(edge_idx) > 0:
                edge_cols.append((r, edge_idx[0]))
            else:
                edge_cols.append((r, -1))  # Fully on table

        self._render_terminal(grid, edge_cols)

    def _render_terminal(self, grid, edge_cols):
        # Clear screen escape sequence
        out = "\033[H\033[J=== 8x8 ToF Edge Heatmap (Row: Fwd -> Back, Col: Inner -> Outer) ===\n"
        edge_dict = dict(edge_cols)

        for r in range(self.grid_size):
            row_str = f"Row {r} | "
            for c in range(self.grid_size):
                val = grid[r, c]
                if c == edge_dict.get(r, -1):
                    # Edge marker
                    row_str += f" \033[91m[|{val:.2f}|]\033[0m"
                elif val <= self.thresh:
                    # Solid surface (green)
                    row_str += f" \033[92m {val:.2f} \033[0m"
                else:
                    # Open air (blue)
                    row_str += f" \033[94m {val:.2f} \033[0m"
            out += row_str + "\n"

        # Fit line through detected edge column indices
        valid_pts = [(r, c) for r, c in edge_cols if c != -1]
        if len(valid_pts) >= 3:
            rows, cols = zip(*valid_pts)
            # Linear regression: col = slope * row + intercept
            slope, intercept = np.polyfit(rows, cols, 1)
            lateral_offset_col = np.mean(cols)
            heading_tilt_deg = math.degrees(math.atan(slope))

            out += f"\n--- Edge Alignment Metrics ---"
            out += f"\nAverage Edge Position: Column {lateral_offset_col:.2f} / 7.0"
            out += f"\nEdge Tilt vs Robot Body: {heading_tilt_deg:+.2f}°"
            if abs(heading_tilt_deg) < 3.0:
                out += " \033[92m(ALIGNED PARALLEL)\033[0m"
            elif heading_tilt_deg > 0:
                out += " \033[93m(DRIFTING OUTWARD)\033[0m"
            else:
                out += " \033[93m(DRIFTING INWARD)\033[0m"

        print(out)


def main(args=None):
    rclpy.init(args=args)
    node = MtofEdgeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()