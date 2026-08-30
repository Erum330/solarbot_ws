#!/usr/bin/env python3
"""
view_mtof_gui.py

Live OpenCV Heatmap & Edge Visualizer for 8x8 Multi-Zone ToF.
Displays a 480x480 graphical window showing:
  - Color depth map (JET colormap)
  - Distance values in cm inside each zone
  - Red line overlay showing the detected table edge
"""

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class MtofGuiViewer(Node):
    def __init__(self):
        super().__init__('mtof_gui_viewer')

        self.declare_parameter('surface_threshold_m', 0.08)  # > 8cm = off table
        self.declare_parameter('min_display_range_m', 0.02)
        self.declare_parameter('max_display_range_m', 0.30)

        self.thresh = float(self.get_parameter('surface_threshold_m').value)
        self.min_rng = float(self.get_parameter('min_display_range_m').value)
        self.max_rng = float(self.get_parameter('max_display_range_m').value)

        self.create_subscription(
            PointCloud2,
            '/right_mid_tof/points',
            self._cloud_cb,
            qos_profile=qos_profile_sensor_data
        )

        self.get_logger().info('🖼️ OpenCV 8x8 ToF Heatmap Window Active. Press "q" in window to close.')

    def _cloud_cb(self, msg: PointCloud2):
        pts = [p for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)]
        if len(pts) < 64:
            return

        grid = np.zeros((8, 8), dtype=np.float32)
        for r in range(8):
            for c in range(8):
                x, y, z = pts[r * 8 + c]
                grid[r, c] = math.sqrt(x*x + y*y + z*z) if (math.isfinite(x) and x > 0) else self.max_rng

        # Normalize 8x8 matrix to 0-255 uint8 for colormap rendering
        clipped = np.clip(grid, self.min_rng, self.max_rng)
        norm = ((clipped - self.min_rng) / (self.max_rng - self.min_rng) * 255.0).astype(np.uint8)

        # Apply JET colormap (Blue = Close/Table, Red = Far/Drop-off)
        heatmap = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

        # Upscale 8x8 to 480x480 using nearest neighbor to preserve distinct grid zones
        display_img = cv2.resize(heatmap, (480, 480), interpolation=cv2.INTER_NEAREST)

        # Draw grid lines and per-zone distance text (in cm)
        cell_size = 480 // 8
        edge_pts_px = []

        for r in range(8):
            # Locate first column crossing table edge threshold
            edge_cols = np.where(grid[r, :] > self.thresh)[0]
            if len(edge_cols) > 0:
                ec = edge_cols[0]
                edge_pts_px.append((int((ec + 0.5) * cell_size), int((r + 0.5) * cell_size)))

            for c in range(8):
                x_px = c * cell_size
                y_px = r * cell_size

                # Grid cell borders
                cv2.rectangle(display_img, (x_px, y_px), (x_px + cell_size, y_px + cell_size), (40, 40, 40), 1)

                # Distance readout in cm
                val_cm = grid[r, c] * 100.0
                text = f"{val_cm:.0f}" if val_cm < 99 else ">99"
                cv2.putText(
                    display_img, text,
                    (x_px + 12, y_px + 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA
                )

        # Draw fitted edge line in bright red
        if len(edge_pts_px) >= 3:
            for pt in edge_pts_px:
                cv2.circle(display_img, pt, 5, (0, 0, 255), -1)

            xs, ys = zip(*edge_pts_px)
            # Fit line: x = slope * y + intercept
            slope, intercept = np.polyfit(ys, xs, 1)
            pt1 = (int(intercept), 0)
            pt2 = (int(slope * 480 + intercept), 480)
            cv2.line(display_img, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)

        # Axis Orientation Labels
        cv2.putText(display_img, "FRONT (Row 0)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(display_img, "REAR (Row 7)", (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(display_img, "INNER (Col 0)", (150, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        cv2.putText(display_img, "OUTER (Col 7)", (340, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        cv2.imshow("8x8 Multi-ToF Edge Map", display_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MtofGuiViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()