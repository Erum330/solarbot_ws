#!/usr/bin/env python3
"""
analyze_validation_log.py

Reads odom_validation_log.csv (written by odom_validation_logger) and
prints a summary: error statistics (matching the paper's Table I/II
style), and detection of "stuck" periods where /odom_cam stopped
updating (consecutive identical cam_x/cam_y rows) - these should be
EXCLUDED from calibration/error analysis, since they reflect a frozen
odometry, not tracking error.

Usage:
  python3 analyze_validation_log.py /path/to/odom_validation_log.csv
"""

import csv
import sys
import statistics as stats


def load_rows(path):
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        return [
            {k: float(v) for k, v in row.items()}
            for row in reader
        ]


def find_stuck_runs(rows, min_run=3):
    """Consecutive rows with identical cam_x AND cam_y = frozen odometry.
    Also checks whether ground truth kept moving during that window -
    if gt was ALSO flat, the robot legitimately stopped (not a tracking
    failure); if gt kept moving while cam froze, that's a real stuck
    tracking period."""
    runs = []
    i = 0
    n = len(rows)
    while i < n - 1:
        j = i
        while (j + 1 < n
               and rows[j + 1]['cam_x'] == rows[i]['cam_x']
               and rows[j + 1]['cam_y'] == rows[i]['cam_y']):
            j += 1
        run_len = j - i + 1
        if run_len >= min_run:
            gt_x_start, gt_y_start = rows[i]['gz_x'], rows[i]['gz_y']
            gt_x_end, gt_y_end = rows[j]['gz_x'], rows[j]['gz_y']
            gt_moved = ((gt_x_end - gt_x_start) ** 2
                        + (gt_y_end - gt_y_start) ** 2) ** 0.5
            runs.append((i, run_len, gt_moved))
        i = j + 1
    return runs


def variance(values):
    m = stats.mean(values)
    return sum((v - m) ** 2 for v in values) / len(values)


def print_paper_style_tables(clean_rows):
    """Reproduces the paper's Table I (raw error stats) and Table II
    (distance-normalized deviation ratio) format, computed from the
    clean (non-stuck) portion of a run."""
    if len(clean_rows) < 2:
        print('Too few clean rows for distance-normalized tables.')
        return

    print('=== TABLE I: Error Statistics (paper format) ===')
    print(f'{"Metric":<10}{"Mean":>10}{"Max":>10}{"Variance":>12}{"Std Dev":>10}')
    for key, label, unit in [
        ('dx_error', 'dx (m)', 'm'),
        ('dy_error', 'dy (m)', 'm'),
        ('dtheta_error_deg', 'dtheta (deg)', 'deg'),
        ('dxy_error', 'dxy (m)', 'm'),
    ]:
        vals = [abs(r[key]) for r in clean_rows]
        print(f'{label:<10}{stats.mean(vals):>10.4f}{max(vals):>10.4f}'
              f'{variance(vals):>12.6f}{stats.pstdev(vals):>10.4f}')
    print()

    # Distance-normalized: how far did ground truth actually travel in
    # each axis over the clean portion, and what fraction of that
    # distance does the mean error represent (paper's "ratio" column).
    gt_x_total = sum(abs(clean_rows[i]['gz_x'] - clean_rows[i - 1]['gz_x'])
                      for i in range(1, len(clean_rows)))
    gt_y_total = sum(abs(clean_rows[i]['gz_y'] - clean_rows[i - 1]['gz_y'])
                      for i in range(1, len(clean_rows)))
    gt_theta_total = sum(abs(clean_rows[i]['gz_theta_deg'] - clean_rows[i - 1]['gz_theta_deg'])
                          for i in range(1, len(clean_rows)))
    gt_xy_total = sum(
        ((clean_rows[i]['gz_x'] - clean_rows[i - 1]['gz_x']) ** 2
         + (clean_rows[i]['gz_y'] - clean_rows[i - 1]['gz_y']) ** 2) ** 0.5
        for i in range(1, len(clean_rows))
    )

    mean_dx = stats.mean(abs(r['dx_error']) for r in clean_rows)
    mean_dy = stats.mean(abs(r['dy_error']) for r in clean_rows)
    mean_dtheta = stats.mean(abs(r['dtheta_error_deg']) for r in clean_rows)
    mean_dxy = stats.mean(abs(r['dxy_error']) for r in clean_rows)

    print('=== TABLE II: Distance-Normalized Deviations (paper format) ===')
    print(f'{"Axis":<14}{"Mean Deviation":>16}{"Distance Traveled":>20}{"Ratio":>10}')
    rows_out = [
        ('X (m)', mean_dx, gt_x_total),
        ('Y (m)', mean_dy, gt_y_total),
        ('Rotation (deg)', mean_dtheta, gt_theta_total),
        ('XY Euclidean (m)', mean_dxy, gt_xy_total),
    ]
    for label, mean_dev, dist in rows_out:
        ratio = mean_dev / dist if dist > 0 else float('nan')
        print(f'{label:<14}{mean_dev:>16.4f}{dist:>20.4f}{ratio:>10.5f}')
    print()
    print(f'For comparison, the paper reported (Table II): '
          f'X ratio=0.00413, Y ratio=0.00168, Rotation ratio=0.00112, XY ratio=0.00302')


def summarize(values, label):
    if not values:
        print(f'  {label}: no data')
        return
    print(f'  {label}: mean={stats.mean(values):.4f}  '
          f'max={max(values):.4f}  '
          f'std={stats.pstdev(values):.4f}  n={len(values)}')


def find_step_jumps(rows, threshold_m=0.3):
    """Detect a single row-to-row jump in cam position larger than
    threshold - this is the signature of one bad ECC frame producing a
    permanent offset, which a coarse first-half/second-half trend check
    can completely miss (it just looks like 'flat' on both sides of an
    arbitrary split point)."""
    jumps = []
    for i in range(1, len(rows)):
        prev = rows[i - 1]
        curr = rows[i]
        d = ((curr['cam_x'] - prev['cam_x']) ** 2
             + (curr['cam_y'] - prev['cam_y']) ** 2) ** 0.5
        if d > threshold_m:
            jumps.append((i, d, prev['cam_x'], prev['cam_y'], curr['cam_x'], curr['cam_y']))
    return jumps


def suggest_calibration(clean_rows, clean_indices, current_px_per_meter=None, current_theta_scale=None):
    """Compares total cam-reported path length/rotation against ground
    truth over the clean run - but ONLY across row-pairs that were
    genuinely consecutive in the original data (clean_indices[i] ==
    clean_indices[i-1] + 1). A pair that straddles an excluded stuck
    period is skipped entirely from BOTH cam and gt sums, not just from
    cam. Skipping stuck ROWS but still taking the raw before/after
    difference across the gap silently double-counts against cam: it
    lost that odometry (transform_detector's own resync logic drops it
    deliberately after long stuck periods), but ground truth never had
    a gap, so the naive before/after diff still includes everything
    that happened during the gap - which showed up as a rotation ratio
    that swung from 2.007 to 0.247 between two runs with different
    stuck patterns, not because the true scale changed."""
    cam_dist = 0.0
    gt_dist = 0.0
    cam_rot = 0.0
    gt_rot = 0.0
    skipped_gaps = 0
    for k in range(1, len(clean_rows)):
        if clean_indices[k] != clean_indices[k - 1] + 1:
            skipped_gaps += 1
            continue
        a, b = clean_rows[k - 1], clean_rows[k]
        cam_dist += ((b['cam_x'] - a['cam_x']) ** 2 + (b['cam_y'] - a['cam_y']) ** 2) ** 0.5
        gt_dist += ((b['gz_x'] - a['gz_x']) ** 2 + (b['gz_y'] - a['gz_y']) ** 2) ** 0.5
        cam_rot += abs(b['cam_theta_deg'] - a['cam_theta_deg'])
        gt_rot += abs(b['gz_theta_deg'] - a['gz_theta_deg'])

    print(f'=== Calibration check (consecutive-row-only, {skipped_gaps} stuck-gap(s) excluded from both sides) ===')
    print(f'  cam total path length:  {cam_dist:.3f} m')
    print(f'  gt total path length:   {gt_dist:.3f} m')
    if gt_dist <= 0:
        print('  Ground truth shows no movement - cannot compute a ratio.')
        return
    ratio = cam_dist / gt_dist
    print(f'  ratio (cam/gt): {ratio:.3f} '
          f'({"cam overestimates distance" if ratio > 1 else "cam underestimates distance"})')
    if current_px_per_meter is not None:
        suggested = current_px_per_meter * ratio
        print(f'  current px_per_meter: {current_px_per_meter:.0f}')
        print(f'  suggested px_per_meter: {suggested:.0f}')
    else:
        print('  Pass current px_per_meter as a 2nd argument to get a suggested corrected value.')
    print()

    print('=== Rotation calibration check (consecutive-row-only) ===')
    print(f'  cam total rotation:  {cam_rot:.1f} deg')
    print(f'  gt total rotation:   {gt_rot:.1f} deg')
    if gt_rot > 0:
        rot_ratio = cam_rot / gt_rot
        print(f'  ratio (cam/gt): {rot_ratio:.3f}')
        if current_theta_scale is not None:
            suggested_theta = current_theta_scale / rot_ratio
            print(f'  current theta_scale: {current_theta_scale:.3f}')
            print(f'  suggested theta_scale: {suggested_theta:.3f}')
        else:
            print('  Pass current theta_scale as a 3rd argument to get a suggested value.')
    print()


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 analyze_odom_log.py <path_to_csv> [current_px_per_meter] [current_theta_scale]')
        sys.exit(1)

    current_ppm = None
    if len(sys.argv) > 2:
        try:
            current_ppm = float(sys.argv[2])
        except ValueError:
            pass

    current_theta = None
    if len(sys.argv) > 3:
        try:
            current_theta = float(sys.argv[3])
        except ValueError:
            pass

    path = sys.argv[1]
    rows = load_rows(path)
    if not rows:
        print('No rows found in CSV.')
        sys.exit(1)

    print(f'Loaded {len(rows)} rows from {path}\n')

    jumps = find_step_jumps(rows)
    print('=== Step-jump check (single bad frame causing a permanent offset) ===')
    if jumps:
        for row_idx, dist, px, py, cx, cy in jumps:
            print(f'  JUMP at row {row_idx}: {dist:.3f}m in one step '
                  f'({px:.3f},{py:.3f}) -> ({cx:.3f},{cy:.3f})')
        print(f'  {len(jumps)} jump(s) found - any "flat trend" result below should be '
              f'treated with suspicion if jumps landed near the midpoint split.')
    else:
        print('  None found. Good - the flat/growing verdict below can be trusted.')
    print()

    stuck_runs = find_stuck_runs(rows)
    real_stuck = [(i, length) for i, length, gt_moved in stuck_runs if gt_moved > 0.05]
    legit_stops = [(i, length) for i, length, gt_moved in stuck_runs if gt_moved <= 0.05]
    stuck_row_count = sum(length for _, length in real_stuck)
    stuck_pct = 100.0 * stuck_row_count / len(rows)

    print('=== Stuck/frozen periods (odom_cam not updating) ===')
    if real_stuck:
        for start_idx, length in real_stuck:
            print(f'  REAL STUCK: rows {start_idx}-{start_idx + length - 1} '
                  f'({length} samples, ~{length}s) frozen at '
                  f"cam=({rows[start_idx]['cam_x']:.3f}, {rows[start_idx]['cam_y']:.3f}) "
                  f'while ground truth kept moving')
    if legit_stops:
        for start_idx, length in legit_stops:
            print(f'  (robot legitimately stopped: rows {start_idx}-{start_idx + length - 1}, '
                  f'{length}s, ground truth also flat here - not a tracking failure)')
    if real_stuck:
        print(f'  TOTAL REAL STUCK: {len(real_stuck)} period(s), '
              f'{stuck_row_count} rows ({stuck_pct:.1f}% of the run)')
    elif not stuck_runs:
        print('  None detected. Good.')
    else:
        print('  No real stuck periods - all frozen stretches match the robot legitimately stopping.')
    print()

    # Exclude stuck rows from error stats - they don't reflect real
    # tracking error, just a frozen reading vs a moving ground truth.
    stuck_indices = set()
    for start_idx, length in real_stuck:
        for k in range(start_idx, start_idx + length):
            stuck_indices.add(k)

    clean_rows = [r for i, r in enumerate(rows) if i not in stuck_indices]
    clean_indices = [i for i in range(len(rows)) if i not in stuck_indices]

    suggest_calibration(clean_rows, clean_indices, current_ppm, current_theta)
    print(f'=== Error statistics (excluding {len(stuck_indices)} stuck rows, '
          f'{len(clean_rows)} clean rows remain) ===')
    summarize([r['dx_error'] for r in clean_rows], 'dx_error (m)')
    summarize([r['dy_error'] for r in clean_rows], 'dy_error (m)')
    summarize([r['dxy_error'] for r in clean_rows], 'dxy_error (m)')
    summarize([r['dtheta_error_deg'] for r in clean_rows], 'dtheta_error (deg)')
    print()

    print_paper_style_tables(clean_rows)

    if len(clean_rows) < 5:
        print('Too few clean rows for a trend - run longer or check for stuck periods.')
        return

    # Trend check: is error growing over the clean portion, or roughly flat?
    first_half = clean_rows[:len(clean_rows) // 2]
    second_half = clean_rows[len(clean_rows) // 2:]
    mean_first = stats.mean(r['dxy_error'] for r in first_half)
    mean_second = stats.mean(r['dxy_error'] for r in second_half)

    print('=== Trend (first half vs second half of clean data) ===')
    print(f'  mean dxy_error, first half:  {mean_first:.3f} m')
    print(f'  mean dxy_error, second half: {mean_second:.3f} m')
    if mean_second > mean_first * 1.3:
        print('  -> GROWING: error is trending upward. Likely still drifting.')
    elif mean_second < mean_first * 0.7:
        print('  -> SHRINKING: error is trending downward. Good sign.')
    else:
        print('  -> ROUGHLY FLAT: error is stable, not accumulating. This is '
              'what "good" looks like once calibration is right.')


if __name__ == '__main__':
    main()