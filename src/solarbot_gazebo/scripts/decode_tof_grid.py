#!/usr/bin/env python3
"""
Decode a `ros2 topic echo /right_mid_tof/points` PointCloud2 dump into an
8x8 range grid, so you can see the reach-per-row pattern at a glance
instead of eyeballing the Gazebo lidar visual.

Usage:
  ros2 topic echo /right_mid_tof/points --once > /tmp/scan.yaml
  python3 decode_tof_grid.py /tmp/scan.yaml
"""
import sys
import struct
import re

def parse_data_block(text):
    # pulls the flat list of ints under "data:" (handles the '- N' per-line
    # style ros2 topic echo produces)
    m = re.search(r"^data:\s*\n((?:- .+\n?)+)", text, re.MULTILINE)
    if not m:
        raise ValueError("Couldn't find a 'data:' block - paste the full echo output.")
    vals = []
    for line in m.group(1).splitlines():
        line = line.strip().lstrip("- ").strip()
        if line == "'...'" or line == "...":
            continue
        vals.append(int(line))
    return bytes(vals)

def main(path):
    text = open(path).read()
    height = int(re.search(r"^height:\s*(\d+)", text, re.MULTILINE).group(1))
    width = int(re.search(r"^width:\s*(\d+)", text, re.MULTILINE).group(1))
    point_step = int(re.search(r"^point_step:\s*(\d+)", text, re.MULTILINE).group(1))
    data = parse_data_block(text)

    expected = height * width * point_step
    if len(data) < expected:
        print(f"WARNING: only {len(data)} bytes present, expected {expected} "
              f"(echo output was probably truncated with '...' - paste the full thing)")

    print(f"{'row':>3} {'col':>3} {'x':>8} {'y':>8} {'z':>8} {'range':>8}")
    for row in range(height):
        for col in range(width):
            off = (row * width + col) * point_step
            if off + 12 > len(data):
                continue
            x, y, z = struct.unpack('<fff', data[off:off+12])
            r = (x*x + y*y + z*z) ** 0.5
            flag = "  <-- near max range (likely a miss)" if r > 3.9 else ""
            print(f"{row:>3} {col:>3} {x:8.4f} {y:8.4f} {z:8.4f} {r:8.4f}{flag}")

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/scan.yaml")