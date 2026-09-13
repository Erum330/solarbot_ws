#!/usr/bin/env python3
"""
map_finalizer_node.py

Generates a closed, obstacle-free rectangular navigation map
directly from the mapped perimeter boundary points saved in panel_edge_map.csv.
"""

import os
import math
import yaml
import numpy as np
import cv2
from PIL import Image

import rclpy
from rclpy.node import Node


class MapFinalizerNode(Node):
    def __init__(self):
        super().__init__('map_finalizer_node')

        default_dir = os.path.expanduser('~/solarbot_ws/src/solarbot_localization/maps')
        self.declare_parameter('maps_dir', default_dir)
        self.declare_parameter('csv_filename', 'panel_edge_map.csv')
        self.declare_parameter('output_map_name', 'solar_panel_map')
        self.declare_parameter('resolution', 0.01)       # 1 cm/pixel
        self.declare_parameter('grid_size_m', 3.0)       # 3.0m x 3.0m map space
        self.declare_parameter('wall_thickness_px', 3)   # 3 px solid boundary

        self.maps_dir = os.path.expanduser(str(self.get_parameter('maps_dir').value))
        self.csv_file = os.path.join(self.maps_dir, str(self.get_parameter('csv_filename').value))
        self.out_name = str(self.get_parameter('output_map_name').value)
        self.res = float(self.get_parameter('resolution').value)
        self.grid_size = float(self.get_parameter('grid_size_m').value)
        self.wall_t = int(self.get_parameter('wall_thickness_px').value)

        self.generate_navigation_map()

    def generate_navigation_map(self):
        if not os.path.exists(self.csv_file):
            self.get_logger().error(f"Cannot find CSV at: {self.csv_file}. Complete perimeter mapping first.")
            return

        # 1. Load points from perimeter scan
        try:
            pts = np.loadtxt(self.csv_file, delimiter=',', skiprows=1, dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f"Failed reading CSV: {e}")
            return

        if len(pts) < 4:
            self.get_logger().error("Not enough edge points found in CSV to build boundary.")
            return

        # 2. Compute the minimum-area oriented bounding box
        # cv2.minAreaRect expects shape (N, 1, 2)
        pts_cv = pts.reshape((-1, 1, 2)).astype(np.float32)
        rect = cv2.minAreaRect(pts_cv)
        box_m = cv2.boxPoints(rect)  # (4, 2) corner points in meters (odom frame)

        (cx, cy), (width_m, length_m), angle = rect
        self.get_logger().info(f"📐 Extracted Bounding Box: {width_m:.3f}m x {length_m:.3f}m, Tilt: {angle:.2f}°")

        # 3. Setup grid parameters
        cells = int(self.grid_size / self.res)
        origin_m = -self.grid_size / 2.0  # [-1.5, -1.5]

        # Convert corner coordinates from meters to grid pixels
        box_px = np.int32(np.round((box_m - origin_m) / self.res))

        # 4. Rasterize: 205 = unknown outside, 254 = free glass inside, 0 = perimeter wall
        img = np.full((cells, cells), 205, dtype=np.uint8)
        cv2.fillPoly(img, [box_px], 254)
        cv2.polylines(img, [box_px], isClosed=True, color=0, thickness=self.wall_t)

        # Flip along Y to follow standard ROS OccupancyGrid frame conventions
        im = Image.fromarray(np.flipud(img))
        pgm_path = os.path.join(self.maps_dir, f'{self.out_name}.pgm')
        im.save(pgm_path)

        # 5. Write YAML metadata
        yaml_path = os.path.join(self.maps_dir, f'{self.out_name}.yaml')
        meta = {
            'image': f'{self.out_name}.pgm',
            'mode': 'trinary',
            'resolution': self.res,
            'origin': [float(origin_m), float(origin_m), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }

        with open(yaml_path, 'w') as f:
            yaml.dump(meta, f, default_flow_style=False)

        self.get_logger().info(f"✅ Closed navigation map generated successfully!")
        self.get_logger().info(f"📄 Output YAML: {yaml_path}")
        self.get_logger().info(f"🖼️ Output PGM : {pgm_path}")


def main(args=None):
    rclpy.init(args=args)
    node = MapFinalizerNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()