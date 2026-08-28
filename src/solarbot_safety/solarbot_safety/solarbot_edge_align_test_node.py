#!/usr/bin/env python3
"""
solarbot_perimeter_openloop_node.py

Odom-free variant of solarbot_perimeter_node.py, for testing the
perimeter FSM before transform_detector / any odometry source is
running (or trusted).

What changed vs. the odom-based version:
  - No Odometry subscription, no self.x/self.y, no have_odom gate.
  - Heading still comes from the IMU only (the odom-based node already
    preferred IMU yaw over odom yaw whenever IMU was available - this
    node just drops the odom fallback, since it's the position half of
    odom that's actually unavailable here, not the yaw half).
  - Every stage that used _dist_from_snap() (INIT_BACKUP, CORNER_BACKUP,
    PANEL_ADJUST, ALIGN_BACKUP, ALIGN_TO_EDGE) now uses an elapsed-time
    budget instead, computed once from the same *_dist_m parameters
    divided by the relevant speed. This is open-loop dead reckoning:
    it assumes cmd_vel_bridge's cmd_scale is calibrated close to
    correct. It will drift more than real odom and is NOT a
    replacement for it long-term - use this only for a first bench/
    table test without transform_detector in the loop.
  - Every timed stage still checks the ToF trip condition FIRST, same
    as the original - the time budget is a fallback/cap, not the
    primary stop condition, wherever a trip is physically expected.
  - Added a hard per-stage safety cap (time_safety_multiplier) and a
    FAULT stage: if a timed motion runs past its capped duration
    without the expected sensor trip, the robot stops and LATCHES
    stopped rather than guessing and continuing - on a table-edge
    task, continuing blind past an expected trip is the wrong default.
  - No /panel_corner or /panel_edge_points publishing - there's no
    real position to put in them without odom. Trip events are logged
    instead.

Still requires: /imu (for heading), /front_mid_tof, /rear_mid_tof,
/left_mid_tof, /right_mid_tof/points (same as the odom-based node).
Does NOT require transform_detector / any Odometry topic at all.
"""

import math
from collections import deque
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from sensor_msgs_py import point_cloud2


class Stage(Enum):
    CALIBRATE     = auto()
    INIT_BACKUP   = auto()
    FOLLOW_SIDE   = auto()
    CORNER_BACKUP = auto()
    TURN_CORNER   = auto()
    PANEL_ADJUST  = auto()
    ALIGN_BACKUP  = auto()
    ALIGN_TO_EDGE = auto()
    SETTLE        = auto()
    FAULT         = auto()
    DONE          = auto()


class SolarbotPerimeterOpenLoopNode(Node):
    def __init__(self):
        super().__init__('solarbot_perimeter_openloop_node')

        # ---------------- Configurable Parameters ----------------
        self.declare_parameter('forward_speed',         0.12)
        self.declare_parameter('backup_speed',        -0.08)
        self.declare_parameter('turn_speed',           0.40)
        self.declare_parameter('turn_angle_deg',       90.0)
        self.declare_parameter('turn_tolerance_deg',   1.2)

        # Distances - same meaning as the odom-based node, but now used
        # only to DERIVE time budgets (distance / |speed|), not measured
        # directly.
        self.declare_parameter('init_backup_dist_m',    0.06)
        self.declare_parameter('corner_backup_dist_m',  0.01)
        self.declare_parameter('align_backup_dist_m',   0.01)
        self.declare_parameter('edge_find_cap_m',       0.15)
        self.declare_parameter('adjust_dist_m',         0.04)

        # Safety multiplier applied on top of the nominal distance/speed
        # time, and a hard floor/ceiling so a zero or tiny speed doesn't
        # produce a zero or infinite budget.
        self.declare_parameter('time_safety_multiplier', 1.6)
        self.declare_parameter('min_stage_time_sec',     0.15)
        self.declare_parameter('max_stage_time_sec',     4.0)

        self.declare_parameter('settle_sec',            0.40)
        self.declare_parameter('straight_kp',           2.5)
        self.declare_parameter('straight_max_wz',       0.30)
        self.declare_parameter('num_sides',             4)

        # Fast-response filter parameters (unchanged from original)
        self.declare_parameter('filter_window',        5)
        self.declare_parameter('min_filter_samples',    3)
        self.declare_parameter('calibration_sec',       1.0)
        self.declare_parameter('gap_delta_m',           0.005)

        self.declare_parameter('cmd_vel_topic',  '/cmd_vel')

        p = self.get_parameter
        self.fwd_spd          = float(p('forward_speed').value)
        self.bkp_spd          = float(p('backup_speed').value)
        self.turn_spd         = float(p('turn_speed').value)
        self.turn_angle       = math.radians(float(p('turn_angle_deg').value))
        self.turn_tol         = math.radians(float(p('turn_tolerance_deg').value))

        self.init_backup_dist   = float(p('init_backup_dist_m').value)
        self.corner_backup_dist = float(p('corner_backup_dist_m').value)
        self.align_backup_dist  = float(p('align_backup_dist_m').value)
        self.edge_find_cap      = float(p('edge_find_cap_m').value)
        self.adjust_dist        = float(p('adjust_dist_m').value)

        self.time_safety_mult = float(p('time_safety_multiplier').value)
        self.min_stage_time   = float(p('min_stage_time_sec').value)
        self.max_stage_time   = float(p('max_stage_time_sec').value)

        self.settle_sec       = float(p('settle_sec').value)
        self.straight_kp      = float(p('straight_kp').value)
        self.straight_max_wz  = float(p('straight_max_wz').value)
        self.num_sides        = int(p('num_sides').value)

        self.filter_window      = int(p('filter_window').value)
        self.min_filter_samples = int(p('min_filter_samples').value)
        self.calibration_sec    = float(p('calibration_sec').value)
        self.gap_delta          = float(p('gap_delta_m').value)

        self.cmd_vel_topic = str(p('cmd_vel_topic').value)

        # ---- Precompute time budgets from distance / |speed| ----
        def time_budget(dist_m, speed_mps):
            spd = abs(speed_mps)
            if spd < 1e-6:
                self.get_logger().warn(
                    f'time_budget: speed is ~0 for a {dist_m} m move - using max_stage_time_sec')
                return self.max_stage_time
            t = (dist_m / spd) * self.time_safety_mult
            return max(self.min_stage_time, min(self.max_stage_time, t))

        self.init_backup_time   = time_budget(self.init_backup_dist, self.bkp_spd)
        self.corner_backup_time = time_budget(self.corner_backup_dist, self.bkp_spd)
        self.align_backup_time  = time_budget(self.align_backup_dist, self.bkp_spd)
        self.edge_find_time     = time_budget(self.edge_find_cap, self.fwd_spd * 0.5)
        self.adjust_time        = time_budget(self.adjust_dist, self.fwd_spd)

        self.get_logger().info(
            f'Open-loop time budgets (dist/speed * {self.time_safety_mult}, '
            f'clamped [{self.min_stage_time}, {self.max_stage_time}]s): '
            f'init_backup={self.init_backup_time:.2f}s corner_backup={self.corner_backup_time:.2f}s '
            f'align_backup={self.align_backup_time:.2f}s edge_find={self.edge_find_time:.2f}s '
            f'adjust={self.adjust_time:.2f}s'
        )

        # Buffers
        self.front_buf = deque(maxlen=self.filter_window)
        self.rear_buf  = deque(maxlen=self.filter_window)
        self.left_buf  = deque(maxlen=self.filter_window)
        self.right_buf = deque(maxlen=self.filter_window)

        self.baseline = {'front': None, 'rear': None, 'left': None, 'right': None}
        self.calib_start = None

        # State tracking - IMU only, no odom
        self.have_imu = False
        self.imu_yaw  = 0.0

        self.stage           = Stage.CALIBRATE
        self.completed_sides = 0

        self.turn_target_yaw  = 0.0
        self.side_heading_yaw = 0.0

        self.settle_end         = self.get_clock().now()
        self.after_settle_stage = Stage.FOLLOW_SIDE
        self.init_snap_set      = False

        self.adjust_spd = self.fwd_spd
        self.fault_reason = ''

        # Per-stage timer bookkeeping (replaces snap_x/snap_y)
        self.stage_start_time = self.get_clock().now()
        self.stage_time_budget = 0.0

        # ROS 2 Communications
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(LaserScan, '/front_mid_tof', self._front_cb, 10)
        self.create_subscription(LaserScan, '/rear_mid_tof',  self._rear_cb, 10)
        self.create_subscription(LaserScan, '/left_mid_tof',  self._left_cb, 10)
        self.create_subscription(PointCloud2, '/right_mid_tof/points', self._right_cb, 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, 10)

        self.create_timer(0.05, self._loop)
        self.get_logger().info(
            '🚀 SolarBot Open-Loop (no odom) Perimeter Controller Active - '
            'TIME-BASED dead reckoning for backup/adjust moves, verify on blocks first'
        )

    # ---------------- Sensor callbacks (unchanged) ----------------
    def _scan_min(self, msg):
        vals = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        return min(vals) if vals else math.inf

    def _front_cb(self, msg):
        v = self._scan_min(msg)
        if math.isfinite(v): self.front_buf.append(v)

    def _rear_cb(self, msg):
        v = self._scan_min(msg)
        if math.isfinite(v): self.rear_buf.append(v)

    def _left_cb(self, msg):
        v = self._scan_min(msg)
        if math.isfinite(v): self.left_buf.append(v)

    def _right_cb(self, msg):
        min_dist = math.inf
        for x, y, z in point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True):
            d = math.sqrt(x * x + y * y + z * z)
            if d < min_dist:
                min_dist = d
        if math.isfinite(min_dist):
            self.right_buf.append(min_dist)

    def _imu_cb(self, msg):
        q = msg.orientation
        if not (math.isfinite(q.x) and math.isfinite(q.y) and math.isfinite(q.z) and math.isfinite(q.w)):
            return
        self.imu_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.have_imu = True

    def _mean(self, buf):
        return (sum(buf) / len(buf)) if len(buf) >= self.min_filter_samples else None

    def _filtered_readings(self):
        return {
            'front': self._mean(self.front_buf),
            'rear':  self._mean(self.rear_buf),
            'left':  self._mean(self.left_buf),
            'right': self._mean(self.right_buf),
        }

    def _filters_ready(self):
        return all(v is not None for v in self._filtered_readings().values())

    def _get_tripped_sensor(self, keys):
        readings = self._filtered_readings()
        for key in keys:
            r = readings[key]
            b = self.baseline[key]
            if r is not None and b is not None:
                if (r - b) > self.gap_delta:
                    return key
        return None

    def _heading(self):
        return self.imu_yaw

    def _pub(self, vx=0.0, wz=0.0):
        cmd = Twist()
        cmd.linear.x  = float(vx) if math.isfinite(vx) else 0.0
        cmd.angular.z = float(wz) if math.isfinite(wz) else 0.0
        self.cmd_pub.publish(cmd)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _norm_angle(self, a):
        while a > math.pi:  a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    def _yaw_err(self, target):
        return self._norm_angle(target - self._heading())

    # ---------------- Time-budget helpers (replace snap_xy / dist_from_snap) ----------------
    def _start_timed_stage(self, budget_sec):
        self.stage_start_time = self.get_clock().now()
        self.stage_time_budget = budget_sec

    def _stage_elapsed_sec(self):
        return (self.get_clock().now() - self.stage_start_time).nanoseconds * 1e-9

    def _stage_time_expired(self):
        return self._stage_elapsed_sec() >= self.stage_time_budget

    def _enter_settle(self, next_stage):
        self.stage = Stage.SETTLE
        self.after_settle_stage = next_stage
        self.settle_end = self.get_clock().now() + Duration(seconds=self.settle_sec)

    def _enter_follow_side(self):
        self.side_heading_yaw = self._heading()
        self.stage = Stage.FOLLOW_SIDE
        self.get_logger().info(
            f'▶️ SIDE {self.completed_sides + 1} (Heading: {math.degrees(self.side_heading_yaw):.1f}°)')

    def _enter_fault(self, reason):
        self.fault_reason = reason
        self.stage = Stage.FAULT
        self._stop()
        self.get_logger().error(
            f'🛑 FAULT: {reason} - stopping and latching. Restart the node once checked.')

    # ---------------- Main loop ----------------
    def _loop(self):
        if not self.have_imu:
            self._stop()
            return
        if not self._filters_ready():
            self._stop()
            return

        # ---- CALIBRATE ----
        if self.stage == Stage.CALIBRATE:
            self._stop()
            if self.calib_start is None:
                self.calib_start = self.get_clock().now()
                self.get_logger().info('🔧 Calibrating baselines on panel...')
                return
            elapsed = (self.get_clock().now() - self.calib_start).nanoseconds * 1e-9
            if elapsed >= self.calibration_sec:
                readings = self._filtered_readings()
                for k in self.baseline:
                    self.baseline[k] = readings[k]
                self.get_logger().info(f"✅ Baselines set: {self.baseline}")
                self.stage = Stage.INIT_BACKUP
            return

        # ---- INIT_BACKUP ----
        if self.stage == Stage.INIT_BACKUP:
            if not self.init_snap_set:
                self._start_timed_stage(self.init_backup_time)
                self.init_snap_set = True
            tripped = self._get_tripped_sensor(('rear',))
            if tripped:
                self.get_logger().info(f'📍 Edge trip during INIT_BACKUP via [{tripped}]')
                self._stop()
                self._enter_settle(Stage.FOLLOW_SIDE)
                return
            if self._stage_time_expired():
                self.get_logger().warn(
                    'INIT_BACKUP time budget reached without a rear-sensor trip - '
                    'continuing (this is the expected common case on an open table edge).')
                self._stop()
                self._enter_settle(Stage.FOLLOW_SIDE)
                return
            self._pub(self.bkp_spd, 0.0)
            return

        # ---- FOLLOW_SIDE ----
        if self.stage == Stage.FOLLOW_SIDE:
            tripped = self._get_tripped_sensor(('front',))
            if tripped:
                self.get_logger().info(
                    f'⚠️ Edge reached on Side {self.completed_sides + 1} via [{tripped}] — backing up for turn...')
                self._stop()
                self._start_timed_stage(self.corner_backup_time)
                self.stage = Stage.CORNER_BACKUP
                return

            err = self._yaw_err(self.side_heading_yaw)
            wz = max(-self.straight_max_wz, min(self.straight_max_wz, self.straight_kp * err))
            self._pub(self.fwd_spd, wz)
            return

        # ---- CORNER_BACKUP ----
        if self.stage == Stage.CORNER_BACKUP:
            rear_tripped = self._get_tripped_sensor(('rear',))

            if rear_tripped or self._stage_time_expired():
                if rear_tripped:
                    self.get_logger().info(f'📍 Corner backup trip via [{rear_tripped}]')
                self._stop()
                current_heading = self._heading()

                grid_cardinal = round(current_heading / (math.pi / 2.0)) * (math.pi / 2.0)
                self.turn_target_yaw = self._norm_angle(grid_cardinal + self.turn_angle)

                self.get_logger().info(
                    f'🔄 Corner Backup complete. Turning 90° '
                    f'(Current: {math.degrees(current_heading):.1f}°, Target: {math.degrees(self.turn_target_yaw):.1f}°)...'
                )
                self.stage = Stage.TURN_CORNER
                return

            self._pub(self.bkp_spd, 0.0)
            return

        # ---- TURN_CORNER (IMU-only, unchanged in spirit) ----
        if self.stage == Stage.TURN_CORNER:
            sensors_to_check = ('rear',) if self.completed_sides == 0 else ('front',)
            tripped_key = self._get_tripped_sensor(sensors_to_check)

            if tripped_key is not None:
                self._stop()
                if tripped_key == 'rear':
                    self.adjust_spd = self.fwd_spd
                    self.get_logger().warn(f'⚠️ Rear sensor [{tripped_key}] hit edge mid-turn! Creeping FORWARD...')
                else:
                    self.adjust_spd = self.bkp_spd
                    self.get_logger().warn(f'⚠️ Front sensor [{tripped_key}] hit edge mid-turn! Creeping BACKWARD...')

                self._start_timed_stage(self.adjust_time)
                self.stage = Stage.PANEL_ADJUST
                return

            err = self._yaw_err(self.turn_target_yaw)
            if abs(err) <= self.turn_tol:
                self._stop()
                self.side_heading_yaw = self.turn_target_yaw
                self.get_logger().info('✅ Turn complete. Aligning frame to cardinal axis...')
                self._start_timed_stage(self.align_backup_time)
                self.stage = Stage.ALIGN_BACKUP
                return

            p_turn_spd = max(0.18, min(self.turn_spd, 1.2 * abs(err)))
            self._pub(0.0, math.copysign(p_turn_spd, err))
            return

        # ---- PANEL_ADJUST ----
        if self.stage == Stage.PANEL_ADJUST:
            if self._stage_time_expired():
                self._stop()
                self.get_logger().info('↩️ Nudge complete. Resuming turn...')
                self.stage = Stage.TURN_CORNER
                return
            self._pub(self.adjust_spd, 0.0)
            return

        # ---- ALIGN_BACKUP ----
        if self.stage == Stage.ALIGN_BACKUP:
            tripped = self._get_tripped_sensor(('rear',))
            if tripped or self._stage_time_expired():
                if tripped:
                    self.get_logger().info(f'📍 Align backup trip via [{tripped}]')
                self._stop()
                self.completed_sides += 1
                if self.completed_sides >= self.num_sides:
                    self.stage = Stage.DONE
                    self.get_logger().info('✅ FULL PERIMETER COMPLETED!')
                    return

                self.get_logger().info(f'🔍 Nudging outward to locate edge for Side {self.completed_sides + 1}...')
                self._start_timed_stage(self.edge_find_time)
                self.stage = Stage.ALIGN_TO_EDGE
                return
            self._pub(self.bkp_spd, 0.0)
            return

        # ---- ALIGN_TO_EDGE ----
        if self.stage == Stage.ALIGN_TO_EDGE:
            tripped = self._get_tripped_sensor(('front',))

            if tripped:
                self.get_logger().info(f'📍 Edge located for Side {self.completed_sides + 1} via [{tripped}]!')
                self._stop()
                self._enter_settle(Stage.FOLLOW_SIDE)
                return

            if self._stage_time_expired():
                # Unlike the odom version (which had a real distance cap and could
                # just declare "edge found here"), an open-loop timeout here means
                # we don't actually know where the edge is - safer to fault than to
                # guess and start following a side from an unknown offset.
                self._enter_fault(
                    f'ALIGN_TO_EDGE timed out ({self.edge_find_time:.2f}s) with no front trip - '
                    f'unknown position without odom, refusing to guess.')
                return

            self._pub(self.fwd_spd * 0.5, 0.0)
            return

        # ---- SETTLE ----
        if self.stage == Stage.SETTLE:
            self._stop()
            if self.get_clock().now() >= self.settle_end:
                if self.after_settle_stage == Stage.FOLLOW_SIDE:
                    self._enter_follow_side()
                else:
                    self.stage = self.after_settle_stage
            return

        # ---- FAULT ----
        if self.stage == Stage.FAULT:
            self._stop()
            return

        # ---- DONE ----
        if self.stage == Stage.DONE:
            self._stop()
            return

    def destroy_cleanly(self):
        try:
            self._stop()
            self.destroy_node()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SolarbotPerimeterOpenLoopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_cleanly()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()