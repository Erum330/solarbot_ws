#!/usr/bin/env python3
"""
plot_panel_map.py

Diagnostic Solar Panel Perimeter Plotter with RANSAC Orthogonal Fitting:
- Reads CSV coordinates from src/solarbot_localization/maps/panel_edge_map.csv
- Splits data chronologically into 4 edges by detecting turn gaps
- Fits lines using RANSAC and enforces exact 90-degree geometric orthogonality
- Computes sharp intersecting corner vertices (C1..C4) without turn chamfering
- Saves publication-quality report to solar_panel_perimeter_report.png
"""

import os
import numpy as np
import matplotlib.pyplot as plt

# ---------------- File Paths & Panel Truth ----------------
MAPS_DIR = os.path.expanduser('~/solarbot_ws/src/solarbot_localization/maps')
CSV_PATH = os.path.join(MAPS_DIR, 'panel_edge_map.csv')
OUTPUT_IMG = os.path.join(MAPS_DIR, 'solar_panel_perimeter_report.png')

TARGET_LENGTH = 1.400  # meters
TARGET_WIDTH  = 1.200  # meters


def fit_line_ransac(points, max_trials=120, residual_threshold=0.03):
    """
    Fits a robust 2D line (a*x + b*y + c = 0, a^2 + b^2 = 1) using RANSAC.
    """
    best_inliers = []
    best_line = None
    n_points = len(points)

    if n_points < 2:
        return None

    for _ in range(max_trials):
        idx = np.random.choice(n_points, 2, replace=False)
        p1, p2 = points[idx[0]], points[idx[1]]

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        norm = np.hypot(dx, dy)
        if norm < 1e-6:
            continue

        a = -dy / norm
        b = dx / norm
        c = -(a * p1[0] + b * p1[1])

        distances = np.abs(a * points[:, 0] + b * points[:, 1] + c)
        inliers = np.where(distances < residual_threshold)[0]

        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_line = (a, b, c)

    # Refine using PCA/SVD over inliers
    if len(best_inliers) >= 2:
        inlier_pts = points[best_inliers]
        mean = np.mean(inlier_pts, axis=0)
        _, _, vh = np.linalg.svd(inlier_pts - mean)
        direction = vh[0]
        a = -direction[1]
        b = direction[0]
        c = -(a * mean[0] + b * mean[1])
        return a, b, c

    return best_line


def intersect_lines(line1, line2):
    """Computes the 2D intersection point (x, y) between two lines."""
    a1, b1, c1 = line1
    a2, b2, c2 = line2

    det = a1 * b2 - a2 * b1
    if abs(det) < 1e-6:
        return None

    x = (b1 * c2 - b2 * c1) / det
    y = (c1 * a2 - c2 * a1) / det
    return np.array([x, y], dtype=np.float32)


def compute_orthogonal_corners_ransac(pts):
    """
    Splits points into 4 sides, fits RANSAC lines, enforces strict 90-deg angles,
    and returns exact corners and dimensions.
    """
    deltas = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    corner_indices = np.argsort(deltas)[-3:]
    split_indices = np.sort(corner_indices) + 1

    segments = np.split(pts, split_indices)

    raw_lines = []
    for seg in segments:
        line = fit_line_ransac(seg)
        if line is None:
            # Fallback if segment is small
            line = (1.0, 0.0, -seg[0][0])
        raw_lines.append(line)

    # Primary orientation angle from Side 1
    a1, b1, _ = raw_lines[0]
    base_theta = np.arctan2(-a1, b1)

    orthogonal_lines = []
    for i, (seg, _) in enumerate(zip(segments, raw_lines)):
        side_theta = base_theta + i * (np.pi / 2.0)
        nx = -np.sin(side_theta)
        ny = np.cos(side_theta)
        center = np.median(seg, axis=0)
        c = -(nx * center[0] + ny * center[1])
        orthogonal_lines.append((nx, ny, c))

    corners = []
    for i in range(4):
        pt = intersect_lines(orthogonal_lines[i], orthogonal_lines[(i + 1) % 4])
        corners.append(pt)
    corners = np.array(corners)

    d1 = np.linalg.norm(corners[0] - corners[1])
    d2 = np.linalg.norm(corners[1] - corners[2])
    length = max(d1, d2)
    width = min(d1, d2)

    return corners, float(length), float(width), segments


def generate_panel_plot():
    if not os.path.exists(CSV_PATH):
        print(f"Error: Could not find '{CSV_PATH}'. Run a mapping trial first.")
        return

    try:
        pts = np.loadtxt(CSV_PATH, delimiter=',', skiprows=1, dtype=np.float32)
    except Exception as e:
        print(f"Failed to read CSV: {e}")
        return

    if len(pts) < 10:
        print("Error: Not enough points in CSV.")
        return

    x_pts = pts[:, 0]
    y_pts = pts[:, 1]

    # Compute RANSAC orthogonal geometric corners
    corners, measured_len, measured_wid, segments = compute_orthogonal_corners_ransac(pts)

    err_len_m = measured_len - TARGET_LENGTH
    err_len_pct = (err_len_m / TARGET_LENGTH) * 100.0

    err_wid_m = measured_wid - TARGET_WIDTH
    err_wid_pct = (err_wid_m / TARGET_WIDTH) * 100.0

    # Setup Plot
    fig, ax = plt.subplots(figsize=(10, 8), dpi=300)
    fig.patch.set_facecolor('#ffffff')
    ax.set_facecolor('#fafafa')

    # Plot raw MToF projected points
    ax.scatter(
        x_pts, y_pts,
        c='#1f77b4', s=24, alpha=0.85, edgecolors='none',
        label=f'MToF Edge Points (N={len(pts)})', zorder=4
    )

    # Plot chronological perimeter path
    ax.plot(x_pts, y_pts, color='#2ca02c', linewidth=1.0, alpha=0.45, label='Robot Travel Polygon', zorder=3)

    # Plot Synthesized 90-degree RANSAC Box
    box_loop = np.vstack([corners, corners[0]])
    ax.plot(
        box_loop[:, 0], box_loop[:, 1],
        color='#d62728', linestyle='--', linewidth=2.0,
        label='RANSAC Orthogonal Boundary (Exact 90°)', zorder=5
    )

    # Mark Corner Vertices (C1..C4)
    for i, corner in enumerate(corners):
        ax.scatter(corner[0], corner[1], color='#d62728', s=80, zorder=6)
        ax.annotate(
            f"C{i+1}\n({corner[0]:.2f}, {corner[1]:.2f})",
            xy=(corner[0], corner[1]),
            xytext=(10, 10), textcoords='offset points',
            fontsize=8, fontweight='bold', color='#222222',
            bbox=dict(boxstyle='round,pad=0.25', facecolor='white', alpha=0.9, edgecolor='#cccccc')
        )

    # Metrics Summary Card (Positioned neatly without overlapping points)
    metrics_text = (
        f"PERIMETER GEOMETRY EVALUATION\n"
        f"─────────────────────────────────────\n"
        f"Target Dimensions : {TARGET_LENGTH:.3f} m × {TARGET_WIDTH:.3f} m\n"
        f"RANSAC Orthogonal : {measured_len:.3f} m × {measured_wid:.3f} m\n"
        f"Length Error (ΔL) : {err_len_m:+.3f} m ({err_len_pct:+.2f}%)\n"
        f"Width Error  (ΔW) : {err_wid_m:+.3f} m ({err_wid_pct:+.2f}%)\n"
        f"Total Samples     : {len(pts)} points"
    )

    ax.text(
        0.03, 0.97, metrics_text,
        transform=ax.transAxes, verticalalignment='top',
        fontsize=8.5, fontfamily='monospace',
        bbox=dict(boxstyle='round,pad=0.5', facecolor='#ffffff', edgecolor='#bbbbbb', alpha=0.95),
        zorder=10
    )

    # Styling and Margins
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X Position (m) [odom frame]', fontsize=11, fontweight='bold', labelpad=8)
    ax.set_ylabel('Y Position (m) [odom frame]', fontsize=11, fontweight='bold', labelpad=8)
    ax.set_title('SolarBot Autonomous Perimeter Mapping (RANSAC Orthogonal)', fontsize=12, fontweight='bold', pad=12)

    ax.grid(True, linestyle=':', alpha=0.6, color='#888888')
    ax.legend(loc='lower right', framealpha=0.9, fontsize=9)

    margin = 0.30
    ax.set_xlim(np.min(x_pts) - margin, np.max(x_pts) + margin)
    # Give extra top margin so the summary card stays well above the data
    ax.set_ylim(np.min(y_pts) - margin, np.max(y_pts) + margin + 0.25)

    plt.tight_layout()
    plt.savefig(OUTPUT_IMG, dpi=300)
    plt.close()

    print(f"✅ RANSAC orthogonal plot successfully saved to: {OUTPUT_IMG}")


if __name__ == '__main__':
    generate_panel_plot()