#!/usr/bin/env python3
"""
mtof_edge_follower.py

Stabilized Closed-Loop Edge Follower with Pulsed Alignment and Coordinated Cruise.
- Stage 1: Pulsed breakout alignment on the spot (0.16 rad/s).
- Stage 2: Coordinated lateral recovery that allows controlled diagonal heading
  approach angles without heading/lateral cancellation.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class MtofEdgeFollower(Node):
    def __init__(self):
        super().__init__('mtof_edge_follower')

        # ---------------- Control Parameters ----------------
        self.declare_parameter('forward_speed',        0.08)     # Cruise speed (m/s)
        self.declare_parameter('target_col',           3.5)      # Target edge column (0.0 to 7.0)
        self.declare_parameter('surface_threshold_m',  0.18)     # > 18 cm = air / drop-off
        self.declare_parameter('kp_lateral',           0.12)     # Lateral recovery gain
        self.declare_parameter('kp_heading',           0.015)    # Parallel dampening gain
        self.declare_parameter('align_turn_speed',     0.16)     # Pulsed alignment breakout rate
        self.declare_parameter('min_wz_breakout',      0.085)    # Min angular velocity while cruising
        self.declare_parameter('max_wz',               0.20)     # Max steering authority
        self.declare_parameter('gap_delta_m',          0.05)     # Front drop-off trigger threshold
        self.declare_parameter('ema_alpha',            0.30)     # Filter factor
        self.declare_parameter('deadband_col',         0.15)     # Column deadband
        self.declare_parameter('deadband_deg',         1.4)      # Heading deadband (deg)
        self.declare_parameter('align_patience_frames', 3)       # Consecutive parallel frames needed to lock

        self.fwd_spd          = float(self.get_parameter('forward_speed').value)
        self.target_col       = float(self.get_parameter('target_col').value)
        self.thresh           = float(self.get_parameter('surface_threshold_m').value)
        self.kp_lat           = float(self.get_parameter('kp_lateral').value)
        self.kp_head          = float(self.get_parameter('kp_heading').value)
        self.align_turn_spd   = float(self.get_parameter('align_turn_speed').value)
        self.min_wz           = float(self.get_parameter('min_wz_breakout').value)
        self.max_wz           = float(self.get_parameter('max_wz').value)
        self.gap_delta        = float(self.get_parameter('gap_delta_m').value)
        self.alpha            = float(self.get_parameter('ema_alpha').value)
        self.deadband_col     = float(self.get_parameter('deadband_col').value)
        self.deadband_deg     = float(self.get_parameter('deadband_deg').value)
        self.align_frames_req = int(self.get_parameter('align_patience_frames').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PointCloud2, '/right_mid_tof/points', self._cloud_cb, qos_profile=qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/front_mid_tof', self._front_cb, qos_profile=qos_profile_sensor_data)

        self.front_baseline = None
        self.front_dist = None
        self.stopped = False

        # States: 'ALIGNING' -> 'FOLLOWING'
        self.state = 'ALIGNING'
        self.align_consecutive_good = 0
        self.pulse_active = False
        self.pulse_start_time = 0.0
        self.settle_start_time = 0.0

        self.filtered_col = self.target_col
        self.filtered_tilt = 0.0
        self.last_log_time = 0.0

        self.get_logger().info('🧭 MToF Edge Follower: Coordinated Guidance Initialized.')

    def _front_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        if valid:
            val = min(valid)
            self.front_dist = val
            if self.front_baseline is None:
                self.front_baseline = val
                self.get_logger().info(f'📍 Front ToF baseline set: {self.front_baseline:.3f}m')

    def _subpixel_edge(self, row):
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
        if self.stopped:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # Corner arrival check
        if self.front_dist is not None and self.front_baseline is not None:
            if (self.front_dist - self.front_baseline) > self.gap_delta:
                self.cmd_pub.publish(Twist())
                self.stopped = True
                self.get_logger().warn(
                    f'🛑 Corner Reached! Front drop: {self.front_dist:.3f}m. Stopping.'
                )
                return

        pts = [p for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)]
        if len(pts) < 64:
            return

        raw_grid = np.zeros((8, 8), dtype=np.float32)
        for r in range(8):
            for c in range(8):
                x, y, z = pts[r * 8 + c]
                raw_grid[r, c] = math.sqrt(x*x + y*y + z*z) if (math.isfinite(x) and x > 0) else 0.60

        grid = np.flipud(raw_grid.T)

        edge_pts = []
        for r in range(8):
            col_cross = self._subpixel_edge(grid[r, :])
            if col_cross is not None:
                edge_pts.append((r, col_cross))

        if len(edge_pts) < 4:
            cmd = Twist()
            cmd.linear.x = self.fwd_spd if self.state == 'FOLLOWING' else 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        rows, cols = zip(*edge_pts)
        slope, _ = np.polyfit(rows, cols, 1)

        raw_col = float(np.mean(cols))
        raw_tilt = -math.degrees(math.atan(slope))

        self.filtered_col = (self.alpha * raw_col) + ((1.0 - self.alpha) * self.filtered_col)
        self.filtered_tilt = (self.alpha * raw_tilt) + ((1.0 - self.alpha) * self.filtered_tilt)

        # ---------------- STAGE 1: PULSED IN-PLACE ALIGNMENT ----------------
        if self.state == 'ALIGNING':
            if abs(self.filtered_tilt) <= self.deadband_deg:
                self.align_consecutive_good += 1
                self.cmd_pub.publish(Twist())
                self.pulse_active = False

                if self.align_consecutive_good >= self.align_frames_req:
                    self.state = 'FOLLOWING'
                    self.get_logger().info(
                        f"\n"
                        f"╔═══════════════════════════════════════════════════════════════╗\n"
                        f"║ 🎯 PARALLEL ALIGNMENT ACHIEVED — STARTING CRUISE              ║\n"
                        f"╠═══════════════════════════════════════════════════════════════╣\n"
                        f"║  Resting Tilt Error      : {self.filtered_tilt:+5.2f}°                     ║\n"
                        f"║  Starting Column         : {self.filtered_col:5.2f} (Target: {self.target_col:.1f})         ║\n"
                        f"╚═══════════════════════════════════════════════════════════════╝"
                    )
                return
            else:
                self.align_consecutive_good = 0

            # 90ms micro-burst at 0.16 rad/s, followed by 180ms settle
            if not self.pulse_active:
                if (now_sec - self.settle_start_time) > 0.18:
                    self.pulse_active = True
                    self.pulse_start_time = now_sec
            else:
                if (now_sec - self.pulse_start_time) > 0.09:
                    self.pulse_active = False
                    self.settle_start_time = now_sec
                    self.cmd_pub.publish(Twist())

            cmd = Twist()
            cmd.linear.x = 0.0
            if self.pulse_active:
                cmd.angular.z = -math.copysign(self.align_turn_spd, self.filtered_tilt)
            else:
                cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            if now_sec - self.last_log_time > 0.20:
                direction = "RIGHT" if self.filtered_tilt > 0 else "LEFT"
                pulse_str = "BURST" if self.pulse_active else "SETTLE"
                self.get_logger().info(
                    f'🧭 [{pulse_str:^6}] Tilt: {self.filtered_tilt:+5.1f}° (Goal: 0.0°) | '
                    f'Target Turn: {direction} @ {self.align_turn_spd:.2f} rad/s'
                )
                self.last_log_time = now_sec
            return

        # ---------------- STAGE 2: CLOSED-LOOP CRUISE FOLLOWING ----------------
        lat_err = self.target_col - self.filtered_col
        ctrl_lat = 0.0 if abs(lat_err) < self.deadband_col else lat_err

        # Line-of-sight target approach angle: 
        # When shifted laterally, the chassis SHOULD point inward/outward to re-converge.
        target_tilt_deg = float(np.clip(ctrl_lat * 4.0, -8.0, 8.0))
        tilt_deviation = target_tilt_deg - self.filtered_tilt
        ctrl_head = 0.0 if abs(tilt_deviation) < self.deadband_deg else tilt_deviation

        raw_wz = (self.kp_lat * ctrl_lat) + (self.kp_head * ctrl_head)

        if abs(raw_wz) > 0.010:
            wz = math.copysign(max(self.min_wz, abs(raw_wz)), raw_wz)
        else:
            wz = 0.0

        wz = float(np.clip(wz, -self.max_wz, self.max_wz))

        cmd = Twist()
        cmd.linear.x = self.fwd_spd
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

        if now_sec - self.last_log_time > 0.20:
            if abs(self.filtered_tilt) <= self.deadband_deg:
                align_status = '✅ PARALLEL'
            elif self.filtered_tilt > 0:
                align_status = '↗️ NOSE-IN'
            else:
                align_status = '↘️ NOSE-OUT'

            if abs(self.filtered_col - self.target_col) <= self.deadband_col:
                pos_status = 'LOCKED'
            elif self.filtered_col < self.target_col:
                pos_status = 'DRIFT-OUT'
            else:
                pos_status = 'DRIFT-IN'

            self.get_logger().info(
                f'[{align_status:^12}] Tilt: {self.filtered_tilt:+5.1f}° | '
                f'[{pos_status:^9}] Col: {self.filtered_col:4.2f}/{self.target_col:.1f} | '
                f'Cmd wz: {wz:+5.3f} rad/s | Rows: {len(edge_pts)}/8'
            )
            self.last_log_time = now_sec


def main(args=None):
    rclpy.init(args=args)
    node = MtofEdgeFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok():
                node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()