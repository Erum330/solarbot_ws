#!/usr/bin/env python3
"""
generate_cleaning_path.py

Boustrophedon (Lawnmower) Coverage Path Planner:
- Computes the 4 exact RANSAC orthogonal corners from panel_edge_map.csv.
- Offsets the perimeter inward by a physical safety margin.
- Sweeps back and forth parallel to the longer axis to minimize turns.
- Exports path waypoints to CSV and generates a visual inspection plot.
"""

import os
import numpy as np
import matplotlib.pyplot as plt

# ---------------- Configuration Parameters ----------------
MAPS_DIR = os.path.expanduser('~/solarbot_ws/src/solarbot_localization/maps')
CSV_PATH = os.path.join(MAPS_DIR, 'panel_edge_map.csv')
WAYPOINTS_CSV = os.path.join(MAPS_DIR, 'cleaning_waypoints.csv')
OUTPUT_IMG = os.path.join(MAPS_DIR, 'solar_panel_cleaning_plan.png')

SAFETY_MARGIN_M = 0.12   # 12 cm buffer from glass edge to track center
TOOL_SWATH_M    = 0.22   # 22 cm effective cleaning stride width


# ---------------- Geometry & Corner Solver ----------------
def fit_line_ransac(points, max_trials=120, residual_threshold=0.03):
    best_inliers = []
    best_line = None
    n_points = len(points)
    if n_points < 2:
        return None

    for _ in range(max_trials):
        idx = np.random.choice(n_points, 2, replace=False)
        p1, p2 = points[idx[0]], points[idx[1]]
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        norm = np.hypot(dx, dy)
        if norm < 1e-6:
            continue
        a, b = -dy / norm, dx / norm
        c = -(a * p1[0] + b * p1[1])
        distances = np.abs(a * points[:, 0] + b * points[:, 1] + c)
        inliers = np.where(distances < residual_threshold)[0]

        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_line = (a, b, c)

    if len(best_inliers) >= 2:
        inlier_pts = points[best_inliers]
        mean = np.mean(inlier_pts, axis=0)
        _, _, vh = np.linalg.svd(inlier_pts - mean)
        direction = vh[0]
        a, b = -direction[1], direction[0]
        c = -(a * mean[0] + b * mean[1])
        return a, b, c
    return best_line


def intersect_lines(line1, line2):
    a1, b1, c1 = line1
    a2, b2, c2 = line2
    det = a1 * b2 - a2 * b1
    if abs(det) < 1e-6:
        return None
    x = (b1 * c2 - b2 * c1) / det
    y = (c1 * a2 - c2 * a1) / det
    return np.array([x, y], dtype=np.float32)


def get_ordered_corners(csv_path):
    pts = np.loadtxt(csv_path, delimiter=',', skiprows=1, dtype=np.float32)
    deltas = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    corner_indices = np.argsort(deltas)[-3:]
    split_indices = np.sort(corner_indices) + 1
    segments = np.split(pts, split_indices)

    raw_lines = [fit_line_ransac(seg) for seg in segments]
    a1, b1, _ = raw_lines[0]
    base_theta = np.arctan2(-a1, b1)

    orthogonal_lines = []
    for i, seg in enumerate(segments):
        side_theta = base_theta + i * (np.pi / 2.0)
        nx, ny = -np.sin(side_theta), np.cos(side_theta)
        center = np.median(seg, axis=0)
        c = -(nx * center[0] + ny * center[1])
        orthogonal_lines.append((nx, ny, c))

    corners = [intersect_lines(orthogonal_lines[i], orthogonal_lines[(i + 1) % 4]) for i in range(4)]
    return np.array(corners, dtype=np.float32)


# ---------------- Path Planning ----------------
def generate_boustrophedon_plan(corners, margin, swath):
    """
    Transforms the bounding box into a local coordinate frame,
    computes the inward safety margin, sweeps parallel to the long edge,
    and maps the generated waypoints back to odom space.
    """
    origin = corners[0]
    edge_01 = corners[1] - corners[0]
    edge_12 = corners[2] - corners[1]

    len_01 = np.linalg.norm(edge_01)
    len_12 = np.linalg.norm(edge_12)

    # Align primary local axis (u) with the longer side
    if len_01 >= len_12:
        u_axis = edge_01 / len_01
        v_axis = np.array([-u_axis[1], u_axis[0]])  # Perpendicular unit vector
        length, width = len_01, len_12
    else:
        u_axis = edge_12 / len_12
        v_axis = np.array([-u_axis[1], u_axis[0]])
        length, width = len_12, len_01

    # Ensure v_axis points inward toward the centroid
    centroid = np.mean(corners, axis=0)
    if np.dot(v_axis, centroid - origin) < 0:
        v_axis = -v_axis

    # Bounded cleaning domain after applying safety margin
    u_min, u_max = margin, length - margin
    v_min, v_max = margin, width - margin

    num_passes = int(np.ceil((v_max - v_min) / swath)) + 1
    v_coords = np.linspace(v_min, v_max, num_passes)

    local_waypoints = []
    forward = True

    for v in v_coords:
        if forward:
            local_waypoints.append((u_min, v))
            local_waypoints.append((u_max, v))
        else:
            local_waypoints.append((u_max, v))
            local_waypoints.append((u_min, v))
        forward = not forward

    # Map local (u, v) waypoints back to world odom (x, y)
    world_waypoints = []
    for u, v in local_waypoints:
        pt = origin + u * u_axis + v * v_axis
        world_waypoints.append(pt)

    # Inward offset boundary polygon for visualization
    inset_box_local = [
        (u_min, v_min),
        (u_max, v_min),
        (u_max, v_max),
        (u_min, v_max)
    ]
    inset_box_world = np.array([origin + u * u_axis + v * v_axis for u, v in inset_box_local])

    return np.array(world_waypoints), inset_box_world, length, width


# ---------------- Visualization & Export ----------------
def run():
    if not os.path.exists(CSV_PATH):
        print(f"Error: Could not locate '{CSV_PATH}'.")
        return

    corners = get_ordered_corners(CSV_PATH)
    waypoints, inset_box, length, width = generate_boustrophedon_plan(
        corners, SAFETY_MARGIN_M, TOOL_SWATH_M
    )

    # Save Waypoint Coordinates
    np.savetxt(WAYPOINTS_CSV, waypoints, delimiter=',', header='x_m,y_m', comments='')
    print(f"✅ Generated {len(waypoints)} cleaning waypoints -> {WAYPOINTS_CSV}")

    # Build Visualization
    fig, ax = plt.subplots(figsize=(10, 8), dpi=300)
    fig.patch.set_facecolor('#ffffff')
    ax.set_facecolor('#fafafa')

    # Draw Physical Panel Edge
    c_loop = np.vstack([corners, corners[0]])
    ax.plot(c_loop[:, 0], c_loop[:, 1], 'r--', linewidth=2.0, label='Physical Panel Perimeter')
    for i, c in enumerate(corners):
        ax.scatter(c[0], c[1], color='#d62728', s=60, zorder=5)
        ax.annotate(f"C{i+1}", xy=(c[0], c[1]), xytext=(7, 7), textcoords='offset points', fontweight='bold')

    # Draw Inward Safety Boundary
    in_loop = np.vstack([inset_box, inset_box[0]])
    ax.plot(in_loop[:, 0], in_loop[:, 1], color='#ff7f0e', linestyle=':', linewidth=1.8, label=f'Safety Inset ({SAFETY_MARGIN_M*100:.0f}cm)')

    # Draw Sweeping Cleaning Path
    ax.plot(waypoints[:, 0], waypoints[:, 1], color='#1f77b4', linewidth=2.2, label='Lawnmower Path', zorder=4)
    ax.scatter(waypoints[:, 0], waypoints[:, 1], color='#1f77b4', s=30, zorder=5)

    # Highlight Start and Finish Waypoints
    ax.scatter(waypoints[0, 0], waypoints[0, 1], color='#2ca02c', s=140, marker='o', label='Start Waypoint', zorder=6)
    ax.scatter(waypoints[-1, 0], waypoints[-1, 1], color='#9467bd', s=140, marker='X', label='End Waypoint', zorder=6)

    # Path Direction Arrows
    for i in range(len(waypoints) - 1):
        p1 = waypoints[i]
        p2 = waypoints[i + 1]
        mid = (p1 + p2) / 2.0
        dp = (p2 - p1)
        norm = np.hypot(dp[0], dp[1])
        if norm > 0.05:
            ax.annotate(
                "", xy=mid + (dp / norm) * 0.02, xytext=mid,
                arrowprops=dict(arrowstyle="->", color="#1f77b4", lw=1.5),
                zorder=5
            )

    # Annotation Box
    info_text = (
        f"AUTONOMOUS COVERAGE PLANNER\n"
        f"───────────────────────────────\n"
        f"Panel Envelope : {length:.3f} m × {width:.3f} m\n"
        f"Safety Margin  : {SAFETY_MARGIN_M*100:.1f} cm inward\n"
        f"Tool Swath     : {TOOL_SWATH_M*100:.1f} cm stride\n"
        f"Total Sweeps   : {int(len(waypoints) / 2)} linear passes\n"
        f"Total Waypoints: {len(waypoints)}"
    )
    ax.text(
        0.03, 0.97, info_text, transform=ax.transAxes, verticalalignment='top',
        fontsize=8.5, fontfamily='monospace',
        bbox=dict(boxstyle='round,pad=0.5', facecolor='#ffffff', edgecolor='#bbbbbb', alpha=0.95),
        zorder=10
    )

    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X Position (m) [odom frame]', fontsize=11, fontweight='bold', labelpad=8)
    ax.set_ylabel('Y Position (m) [odom frame]', fontsize=11, fontweight='bold', labelpad=8)
    ax.set_title('Solar Panel Boustrophedon Cleaning Path Plan', fontsize=12, fontweight='bold', pad=12)
    ax.grid(True, linestyle=':', alpha=0.6, color='#888888')
    ax.legend(loc='lower right', framealpha=0.9, fontsize=9)

    margin = 0.30
    ax.set_xlim(np.min(corners[:, 0]) - margin, np.max(corners[:, 0]) + margin)
    ax.set_ylim(np.min(corners[:, 1]) - margin, np.max(corners[:, 1]) + margin + 0.20)

    plt.tight_layout()
    plt.savefig(OUTPUT_IMG, dpi=300)
    plt.close()
    print(f"✅ Coverage plot successfully saved -> {OUTPUT_IMG}")


if __name__ == '__main__':
    run()