#!/usr/bin/env python3
"""
measure_map_dimensions.py

Measures the fitted rectangle directly from map.pgm + map.yaml (odom_mapper's
actual output - it does not save a separate .npy file) and compares it
against the panel's real physical dimensions.

Usage:
  python3 measure_map_dimensions.py /path/to/map.pgm [real_w] [real_h] [current_px_per_meter]

Reads resolution from the matching map.yaml (same directory, same
basename) if present, otherwise defaults to 0.05.
Defaults: real_w=4.0, real_h=1.8 (panel outer frame in warehouse_rooftop.sdf)
"""

import sys
import os
import numpy as np
import cv2
import yaml


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 measure_map_dimensions.py <map.pgm> [real_w] [real_h]')
        sys.exit(1)

    pgm_path = sys.argv[1]
    real_w = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
    real_h = float(sys.argv[3]) if len(sys.argv) > 3 else 1.8

    yaml_path = os.path.join(os.path.dirname(pgm_path), 'map.yaml')
    resolution = 0.05
    if os.path.exists(yaml_path):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
            resolution = float(meta.get('resolution', 0.05))
        print(f'Resolution read from map.yaml: {resolution}')
    else:
        print(f'No map.yaml found next to {pgm_path}, defaulting resolution=0.05')

    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f'Could not read {pgm_path} - check the path.')
        sys.exit(1)

    # 205 = unknown (the only place that exact value appears); anything
    # else (254 = free interior, 0 = border) is part of the fitted rectangle.
    known_mask = (img != 205)
    if not np.any(known_mask):
        print('Map is entirely unknown - nothing to measure.')
        sys.exit(1)

    ys, xs = np.where(known_mask)
    points = np.column_stack((xs, ys)).astype(np.float32)

    rect = cv2.minAreaRect(points)
    (cx, cy), (w_px, h_px), angle = rect

    measured_long = max(w_px, h_px) * resolution
    measured_short = min(w_px, h_px) * resolution

    real_long = max(real_w, real_h)
    real_short = min(real_w, real_h)

    err_long_pct = (measured_long - real_long) / real_long * 100
    err_short_pct = (measured_short - real_short) / real_short * 100

    print(f'Measured: long={measured_long:.3f}m  short={measured_short:.3f}m  '
          f'aspect={measured_long/measured_short:.2f}:1  angle={angle:.1f}deg')
    print(f'Real:     long={real_long:.3f}m  short={real_short:.3f}m  '
          f'aspect={real_long/real_short:.2f}:1')
    print()
    print(f'Error (long axis):  {err_long_pct:+.1f}%')
    print(f'Error (short axis): {err_short_pct:+.1f}%')


if __name__ == '__main__':
    main()