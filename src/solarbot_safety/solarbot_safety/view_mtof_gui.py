#!/usr/bin/env python3
"""
view_mtof_gui.py

Live OpenCV Heatmap & Edge Visualizer for 8x8 Multi-Zone ToF.
Stabilized version matching mtof_edge_follower:
- Sub-pixel linear interpolation for continuous edge transition detection.
- Exponential Moving Average (EMA) filtering on edge line end-points to eliminate flicker.
- Corrected hardware orientation (transpose + flip_vertical).
- Visual readout of smoothed average edge column and tilt angle.
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

        # Thresholds and rendering options
        self.declare_parameter('surface_threshold_m', 0.18)   # > 18 cm is open air / drop-off
        self.declare_parameter('min_display_range_m', 0.05)   # 5 cm (deep blue)
        self.declare_parameter('max_display_range_m', 0.60)   # 60 cm (dark red)
        self.declare_parameter('transpose_grid', True)        # Optical coordinate rotation
        self.declare_parameter('flip_vertical', True)         # Align Row 0 to physical Front
        self.declare_parameter('ema_alpha', 0.25)             # Smoothing factor (0.1=slow/smooth, 1.0=raw)

        self.thresh = float(self.get_parameter('surface_threshold_m').value)
        self.min_rng = float(self.get_parameter('min_display_range_m').value)
        self.max_rng = float(self.get_parameter('max_display_range_m').value)
        self.transpose = bool(self.get_parameter('transpose_grid').value)
        self.flip_vertical = bool(self.get_parameter('flip_vertical').value)
        self.alpha = float(self.get_parameter('ema_alpha').value)

        # Persistent filter states for the rendered edge line end-points (X coordinates)
        self.filtered_x_top = None
        self.filtered_x_bottom = None
        self.filtered_avg_col = 3.5
        self.filtered_tilt_deg = 0.0

        self.create_subscription(
            PointCloud2,
            '/right_mid_tof/points',
            self._cloud_cb,
            qos_profile=qos_profile_sensor_data
        )

        self.get_logger().info(f'🖼️ Stabilized OpenCV 8x8 ToF Heatmap Active (EMA Alpha: {self.alpha}).')

    def _subpixel_edge(self, row):
        """Finds continuous sub-column edge boundary via linear interpolation."""
        for c in range(7):
            d1 = row[c]
            d2 = row[c + 1]
            if d1 <= self.thresh and d2 > self.thresh:
                fraction = (self.thresh - d1) / max(1e-4, (d2 - d1))
                return float(c) + float(fraction)
        if row[0] > self.thresh:
            return 0.0
        return None

    def _cloud_cb(self, msg: PointCloud2):
        pts = [p for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)]
        if len(pts) < 64:
            return

        raw_grid = np.zeros((8, 8), dtype=np.float32)
        for r in range(8):
            for c in range(8):
                x, y, z = pts[r * 8 + c]
                raw_grid[r, c] = math.sqrt(x*x + y*y + z*z) if (math.isfinite(x) and x > 0) else self.max_rng

        # 1. Transpose optical coordinate alignment
        grid = raw_grid.T if self.transpose else raw_grid

        # 2. Invert vertical rows to align Row 0 with physical Front
        if self.flip_vertical:
            grid = np.flipud(grid)

        # Normalize 0-255 uint8 for colormap rendering
        clipped = np.clip(grid, self.min_rng, self.max_rng)
        norm = ((clipped - self.min_rng) / (self.max_rng - self.min_rng) * 255.0).astype(np.uint8)

        # Apply JET colormap (Blue = Panel Surface, Red = Air Drop-off)
        heatmap = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
        display_img = cv2.resize(heatmap, (480, 480), interpolation=cv2.INTER_NEAREST)

        cell_size = 480 // 8
        edge_pts_px = []
        raw_cols = []
        raw_rows = []

        # Find continuous sub-pixel edge points across each row
        for r in range(8):
            col_float = self._subpixel_edge(grid[r, :])
            if col_float is not None:
                raw_cols.append(col_float)
                raw_rows.append(r)
                # Map floating point column position to pixel coordinate
                px_x = int(col_float * cell_size)
                px_y = int((r + 0.5) * cell_size)
                edge_pts_px.append((px_x, px_y))

            # Draw cell borders and text labels
            for c in range(8):
                x_px = c * cell_size
                y_px = r * cell_size

                cv2.rectangle(display_img, (x_px, y_px), (x_px + cell_size, y_px + cell_size), (40, 40, 40), 1)

                val_cm = grid[r, c] * 100.0
                text = f"{val_cm:.0f}" if val_cm < 99 else ">99"
                cv2.putText(
                    display_img, text,
                    (x_px + 12, y_px + 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA
                )

        # Robust Line Fitting with Temporal Smoothing
        if len(edge_pts_px) >= 4:
            # Draw sub-pixel detected edge dots (yellow)
            for pt in edge_pts_px:
                cv2.circle(display_img, pt, 4, (0, 255, 255), -1)

            # Fit line: col = slope * row + intercept
            slope, intercept = np.polyfit(raw_rows, raw_cols, 1)

            # Convert to image pixel domain: y_px ranges from 0 to 480
            # row_float = y_px / cell_size -> x_px = (slope * (y_px / cell_size) + intercept) * cell_size
            raw_x_top = float(intercept * cell_size)
            raw_x_bottom = float((slope * 7.5 + intercept) * cell_size)

            raw_avg_col = float(np.mean(raw_cols))
            raw_tilt_deg = math.degrees(math.atan(slope))

            # Apply Exponential Moving Average (EMA) to kill flicker
            if self.filtered_x_top is None:
                self.filtered_x_top = raw_x_top
                self.filtered_x_bottom = raw_x_bottom
                self.filtered_avg_col = raw_avg_col
                self.filtered_tilt_deg = raw_tilt_deg
            else:
                self.filtered_x_top = (self.alpha * raw_x_top) + ((1.0 - self.alpha) * self.filtered_x_top)
                self.filtered_x_bottom = (self.alpha * raw_x_bottom) + ((1.0 - self.alpha) * self.filtered_x_bottom)
                self.filtered_avg_col = (self.alpha * raw_avg_col) + ((1.0 - self.alpha) * self.filtered_avg_col)
                self.filtered_tilt_deg = (self.alpha * raw_tilt_deg) + ((1.0 - self.alpha) * self.filtered_tilt_deg)

            pt1 = (int(self.filtered_x_top), 0)
            pt2 = (int(self.filtered_x_bottom), 480)

            # Draw stabilized edge line in bright red
            cv2.line(display_img, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)

            # Status Overlay Box
            status_text = f"Col: {self.filtered_avg_col:.2f} | Tilt: {self.filtered_tilt_deg:+.1f} deg"
            cv2.rectangle(display_img, (5, 425), (280, 450), (0, 0, 0), -1)
            cv2.putText(
                display_img, status_text,
                (10, 443), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA
            )

        # Body Frame Axis Labels
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