#!/usr/bin/env python3
"""
save_nav2_map.py
Converts mapped occupancy data to Nav2 map.yaml and map.pgm.
"""

import os
import cv2
import numpy as np

MAPS_DIR = os.path.expanduser('~/solarbot_ws/src/solarbot_localization/maps')
CSV_PATH = os.path.join(MAPS_DIR, 'panel_edge_map.csv')
PGM_PATH = os.path.join(MAPS_DIR, 'panel_map.pgm')
YAML_PATH = os.path.join(MAPS_DIR, 'panel_map.yaml')

RESOLUTION = 0.01  # 1 cm per cell
MAP_SIZE_M = 3.0   # 3.0 m x 3.0 m
GRID_CELLS = int(MAP_SIZE_M / RESOLUTION)
ORIGIN_OFFSET = MAP_SIZE_M / 2.0

def export():
    if not os.path.exists(CSV_PATH):
        print(f"Error: {CSV_PATH} not found.")
        return

    pts = np.loadtxt(CSV_PATH, delimiter=',', skiprows=1, dtype=np.float32)
    # Nav2 PGM standard: 254 = free (white), 0 = occupied (black), 205 = unknown (grey)
    grid = np.full((GRID_CELLS, GRID_CELLS), 205, dtype=np.uint8)

    # Mark interior as free space based on extents
    min_x, min_y = np.min(pts, axis=0)
    max_x, max_y = np.max(pts, axis=0)
    
    r_min = int((min_y + ORIGIN_OFFSET) / RESOLUTION)
    r_max = int((max_y + ORIGIN_OFFSET) / RESOLUTION)
    c_min = int((min_x + ORIGIN_OFFSET) / RESOLUTION)
    c_max = int((max_x + ORIGIN_OFFSET) / RESOLUTION)
    grid[r_min:r_max, c_min:c_max] = 254

    # Mark physical edge perimeter as occupied
    for x, y in pts:
        c = int((x + ORIGIN_OFFSET) / RESOLUTION)
        r = int((y + ORIGIN_OFFSET) / RESOLUTION)
        if 0 <= r < GRID_CELLS and 0 <= c < GRID_CELLS:
            grid[r, c] = 0

    # Thicken boundary wall slightly for Nav2 collision checking
    kernel = np.ones((3, 3), np.uint8)
    edges = (grid == 0).astype(np.uint8) * 255
    edges = cv2.dilate(edges, kernel, iterations=2)
    grid[edges == 255] = 0

    # Flip vertically for ROS coordinate frame convention
    grid = cv2.flip(grid, 0)
    cv2.imwrite(PGM_PATH, grid)

    yaml_content = f"""image: panel_map.pgm
mode: trinary
resolution: {RESOLUTION}
origin: [{-ORIGIN_OFFSET}, {-ORIGIN_OFFSET}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    with open(YAML_PATH, 'w') as f:
        f.write(yaml_content)

    print(f"✅ Nav2 Map generated: {PGM_PATH} and {YAML_PATH}")

if __name__ == '__main__':
    export()