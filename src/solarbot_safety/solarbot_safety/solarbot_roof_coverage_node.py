#!/usr/bin/env python3
"""
solarbot_perimeter_node.py

Perimeter-following FSM updated for 4-ToF middle-of-side layout.
Synchronized with properties.xacro (right_mid_tof_y = -0.105, right_mid_tof_z = -0.005)
and reading /right_mid_tof/points (PointCloud2).
"""

import math
from collections import deque
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from sensor_msgs_py import point_cloud2


class Stage(Enum):
    CALIBRATE     = auto()
    RECALIBRATE   = auto()  # Fresh baselines after every gap crossing -
                             # confirmed on hardware that a stale
                             # baseline (calibrated once on panel 1,
                             # never refreshed) let open roof deck
                             # false-positive as "solid panel" once the
                             # robot crossed off the actual panel grid.
    INIT_BACKUP   = auto()
    FOLLOW_SIDE   = auto()
    CORNER_BACKUP = auto()
    TURN_CORNER   = auto()
    PANEL_ADJUST  = auto()
    ALIGN_BACKUP  = auto()
    ALIGN_TO_EDGE = auto()
    CROSS_GAP     = auto()  # Drive toward a DELIBERATELY chosen next-
                             # panel heading (see _plan_next_crossing) -
                             # confirmed crossed once BOTH front AND
                             # rear read "on panel" (not tripped), since
                             # that means the whole chassis has cleared
                             # the gap, not just the front tip.
    SETTLE        = auto()
    DONE          = auto()


class SolarbotPerimeterNode(Node):
    def __init__(self):
        super().__init__('solarbot_perimeter_node')

        # ---------------- Configurable Parameters ----------------
        self.declare_parameter('forward_speed',         0.12)
        self.declare_parameter('backup_speed',        -0.08)
        self.declare_parameter('turn_speed',           0.40)
        self.declare_parameter('turn_angle_deg',       90.0)
        self.declare_parameter('turn_tolerance_deg',   1.2)

        # Distances
        self.declare_parameter('init_backup_dist_m',    0.06)
        self.declare_parameter('corner_backup_dist_m',  0.01)
        self.declare_parameter('align_backup_dist_m',   0.01)
        self.declare_parameter('edge_find_cap_m',       0.15)
        self.declare_parameter('adjust_dist_m',         0.04)
        # Safety-only backstop for a crossing that doesn't find a panel
        # within a reasonable distance - see _plan_next_crossing for the
        # actual (deliberate) mission-end detection.
        self.declare_parameter('gap_cross_max_dist_m',  1.00)

        # Grid topology - confirmed from warehouse_rooftop.sdf: 2
        # columns (0.5m column gap between them), 5 panels per column
        # (0.3m row gap within a column). A panel's own 4-sided
        # perimeter loop always ends facing the OPPOSITE of its entry
        # heading, so "just keep going straight after side 4" only
        # reaches the next panel by coincidence - this plan explicitly
        # turns toward the real next-panel direction instead.
        self.declare_parameter('panels_per_column',     5)
        self.declare_parameter('num_columns',           2)

        self.declare_parameter('settle_sec',            0.40)
        self.declare_parameter('straight_kp',           2.5)
        self.declare_parameter('straight_max_wz',       0.30)
        self.declare_parameter('num_sides',             4)

        # Fast-response filter parameters
        self.declare_parameter('filter_window',        5)
        self.declare_parameter('min_filter_samples',    3)
        self.declare_parameter('calibration_sec',       1.0)
        self.declare_parameter('gap_delta_m',           0.005)

        # Topics
        self.declare_parameter('odom_topic',     '/diff_drive_controller/odom')
        self.declare_parameter('cmd_vel_topic',  '/cmd_vel')

        # Mounting offsets matching common/properties.xacro exactly
        self.declare_parameter('front_mid_tof_x',   0.200)
        self.declare_parameter('front_mid_tof_y',   0.000)
        self.declare_parameter('rear_mid_tof_x',   -0.200)
        self.declare_parameter('rear_mid_tof_y',    0.000)
        self.declare_parameter('left_mid_tof_x',    0.000)
        self.declare_parameter('left_mid_tof_y',    0.115)
        self.declare_parameter('right_mid_tof_x',   0.000)
        self.declare_parameter('right_mid_tof_y',  -0.105)

        # Continuous lateral edge-following
        self.declare_parameter('lateral_kp',           1.5)
        self.declare_parameter('lateral_target_offset', 0.0)
        self.declare_parameter('lateral_max_wz',       0.15)

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
        self.gap_cross_max_dist = float(p('gap_cross_max_dist_m').value)
        self.panels_per_column  = int(p('panels_per_column').value)
        self.num_columns        = int(p('num_columns').value)

        self.settle_sec       = float(p('settle_sec').value)
        self.straight_kp      = float(p('straight_kp').value)
        self.straight_max_wz  = float(p('straight_max_wz').value)
        self.num_sides        = int(p('num_sides').value)

        self.filter_window      = int(p('filter_window').value)
        self.min_filter_samples = int(p('min_filter_samples').value)
        self.calibration_sec    = float(p('calibration_sec').value)
        self.gap_delta          = float(p('gap_delta_m').value)

        self.odom_topic          = str(p('odom_topic').value)
        self.cmd_vel_topic       = str(p('cmd_vel_topic').value)

        self.sensor_offsets = {
            'front': (float(p('front_mid_tof_x').value), float(p('front_mid_tof_y').value)),
            'rear':  (float(p('rear_mid_tof_x').value),  float(p('rear_mid_tof_y').value)),
            'left':  (float(p('left_mid_tof_x').value),  float(p('left_mid_tof_y').value)),
            'right': (float(p('right_mid_tof_x').value), float(p('right_mid_tof_y').value)),
        }

        self.lateral_kp            = float(p('lateral_kp').value)
        self.lateral_target_offset = float(p('lateral_target_offset').value)
        self.lateral_max_wz        = float(p('lateral_max_wz').value)

        # Buffers
        self.front_buf = deque(maxlen=self.filter_window)
        self.rear_buf  = deque(maxlen=self.filter_window)
        self.left_buf  = deque(maxlen=self.filter_window)
        self.right_buf = deque(maxlen=self.filter_window)

        self.baseline = {'front': None, 'rear': None, 'left': None, 'right': None}
        self.calib_start = None

        # State Tracking
        self.have_odom = False
        self.have_imu  = False
        self.x = self.y = 0.0
        self.odom_yaw = 0.0
        self.imu_yaw  = 0.0

        self.stage           = Stage.CALIBRATE
        self.completed_sides = 0
        self.snap_x = self.snap_y = 0.0

        self.turn_target_yaw  = 0.0
        self.side_heading_yaw = 0.0

        self.settle_end         = self.get_clock().now()
        self.after_settle_stage = Stage.FOLLOW_SIDE
        self.init_snap_set      = False

        self.adjust_spd = self.fwd_spd

        # Grid-plan state. row_gap_heading/col_gap_heading are captured
        # ONCE, from panel_1's own entry/Side2 headings, then every
        # later crossing direction is derived from them rather than
        # re-guessed - see _plan_next_crossing.
        self.row_gap_heading = None
        self.col_gap_heading = None
        self.row_in_col = 0
        self.col_index = 0
        self.turning_for_cross = False
        self.cross_target_yaw = 0.0
        self.after_calibrate_stage = Stage.INIT_BACKUP

        # ROS 2 Communications
        self.cmd_pub          = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.panel_corner_pub = self.create_publisher(Point, '/panel_corner', 10)
        self.edge_point_pub   = self.create_publisher(Point, '/panel_edge_points', 10)

        self.create_subscription(LaserScan, '/front_mid_tof', self._front_cb, 10)
        self.create_subscription(LaserScan, '/rear_mid_tof',  self._rear_cb, 10)
        self.create_subscription(LaserScan, '/left_mid_tof',  self._left_cb, 10)
        self.create_subscription(PointCloud2, '/right_mid_tof/points', self._right_cb, 10)
        self.create_subscription(Odometry,  self.odom_topic,    self._odom_cb, 10)
        self.create_subscription(Imu,       '/imu',             self._imu_cb, 10)

        self.create_timer(0.05, self._loop)
        self.get_logger().info('🚀 SolarBot Uniform Perimeter Controller Active')

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

    def _odom_cb(self, msg):
        px, py = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(q.w) and math.isfinite(q.z)):
            return
        self.x, self.y = px, py
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.have_odom = True

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
        return self.imu_yaw if self.have_imu else self.odom_yaw

    def _sensor_world_xy(self, key):
        lx, ly = self.sensor_offsets[key]
        h = self._heading()
        wx = self.x + lx * math.cos(h) - ly * math.sin(h)
        wy = self.y + lx * math.sin(h) + ly * math.cos(h)
        return wx, wy

    def _publish_edge_point(self, key):
        wx, wy = self._sensor_world_xy(key)
        pt = Point()
        pt.x = wx
        pt.y = wy
        pt.z = 0.0
        self.edge_point_pub.publish(pt)
        self.get_logger().info(f'📍 Edge point marked via [{key}]: ({wx:.3f}, {wy:.3f})')

    def _pub(self, vx=0.0, wz=0.0):
        cmd = Twist()
        cmd.linear.x  = float(vx) if math.isfinite(vx) else 0.0
        cmd.angular.z = float(wz) if math.isfinite(wz) else 0.0
        self.cmd_pub.publish(cmd)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _snap_xy(self):
        self.snap_x = self.x
        self.snap_y = self.y

    def _dist_from_snap(self):
        return math.hypot(self.x - self.snap_x, self.y - self.snap_y)

    def _norm_angle(self, a):
        while a > math.pi:  a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    def _yaw_err(self, target):
        return self._norm_angle(target - self._heading())

    def _plan_next_crossing(self):
        """Decide what happens after the current panel's last side.
        Returns True if there's a next panel to cross to (and sets
        self.cross_target_yaw, self.row_in_col, self.col_index for it),
        or False if this was the last panel in the grid.

        Snake pattern: go straight up column 0 (row_gap_heading), cross
        to column 1 at the top (col_gap_heading), then straight down
        column 1 (row_gap_heading + 180deg) back to the start row.
        """
        is_last_in_column = self.row_in_col >= (self.panels_per_column - 1)
        is_last_column = self.col_index >= (self.num_columns - 1)

        if not is_last_in_column:
            # Stay in this column, cross a row gap. Direction alternates
            # per column so the overall path snakes instead of always
            # trying to go the same absolute direction (which would be
            # wrong for every other column).
            if self.col_index % 2 == 0:
                target = self.row_gap_heading
            else:
                target = self._norm_angle(self.row_gap_heading + math.pi)
            self.row_in_col += 1
            self.cross_target_yaw = target
            return True

        if not is_last_column:
            # Last panel in this column - cross the column gap instead.
            self.cross_target_yaw = self.col_gap_heading
            self.col_index += 1
            self.row_in_col = 0
            return True

        return False

    def _enter_settle(self, next_stage):
        self.stage = Stage.SETTLE
        self.after_settle_stage = next_stage
        self.settle_end = self.get_clock().now() + Duration(seconds=self.settle_sec)

    def _enter_follow_side(self):
        self.side_heading_yaw = self._heading()
        if self.row_gap_heading is None:
            # First time ever - this defines the whole grid's reference
            # frame. row_gap_heading is the direction that stays within
            # the current column (matches panel_1's own Side2 heading,
            # i.e. entry+90deg); col_gap_heading is the direction that
            # switches columns (matches panel_1's own entry heading).
            self.row_gap_heading = self._norm_angle(self.side_heading_yaw + math.pi / 2.0)
            self.col_gap_heading = self.side_heading_yaw
            self.get_logger().info(
                f'📐 Grid reference captured: row_gap_heading={math.degrees(self.row_gap_heading):.1f}°, '
                f'col_gap_heading={math.degrees(self.col_gap_heading):.1f}°')
        self.stage = Stage.FOLLOW_SIDE
        self._snap_xy()
        self.get_logger().info(
            f'▶️ Column {self.col_index + 1}, Panel {self.row_in_col + 1}/{self.panels_per_column}, '
            f'SIDE {self.completed_sides + 1} (Heading: {math.degrees(self.side_heading_yaw):.1f}°)')

    def _loop(self):
        if not (self.have_odom and self.have_imu):
            self._stop()
            return
        if not self._filters_ready():
            self._stop()
            return

        # ---- CALIBRATE / RECALIBRATE ----
        if self.stage in (Stage.CALIBRATE, Stage.RECALIBRATE):
            self._stop()
            if self.calib_start is None:
                self.calib_start = self.get_clock().now()
                self.get_logger().info('🔧 Calibrating baselines...')
                return
            elapsed = (self.get_clock().now() - self.calib_start).nanoseconds * 1e-9
            if elapsed >= self.calibration_sec:
                readings = self._filtered_readings()
                for k in self.baseline:
                    self.baseline[k] = readings[k]
                self.get_logger().info(f"✅ Baselines set: {self.baseline}")
                self.calib_start = None
                if self.after_calibrate_stage == Stage.FOLLOW_SIDE:
                    self._enter_follow_side()
                else:
                    self.stage = self.after_calibrate_stage
            return

        # ---- INIT_BACKUP ----
        if self.stage == Stage.INIT_BACKUP:
            if not self.init_snap_set:
                self._snap_xy()
                self.init_snap_set = True
            tripped = self._get_tripped_sensor(('rear',))
            if tripped or self._dist_from_snap() >= self.init_backup_dist:
                if tripped:
                    self._publish_edge_point(tripped)
                self._stop()
                self._enter_settle(Stage.FOLLOW_SIDE)
                return
            self._pub(self.bkp_spd, 0.0)
            return

        # ---- FOLLOW_SIDE ----
        if self.stage == Stage.FOLLOW_SIDE:
            tripped = self._get_tripped_sensor(('front',))
            if tripped:
                self._publish_edge_point(tripped)
                self._stop()
                self._snap_xy()

                if self.completed_sides == (self.num_sides - 1):
                    has_next = self._plan_next_crossing()
                    if not has_next:
                        self.get_logger().info('✅ ALL PANELS COVERED - MISSION COMPLETE!')
                        self.stage = Stage.DONE
                        return
                    self.get_logger().info(
                        f'✅ Side {self.completed_sides + 1} (last side) done! Turning to face '
                        f'next panel (target {math.degrees(self.cross_target_yaw):.1f}°)...')
                    self.turning_for_cross = True
                    self.stage = Stage.CORNER_BACKUP
                    return

                self.get_logger().info(f'⚠️ Edge reached on Side {self.completed_sides + 1} — backing up for turn...')
                self.stage = Stage.CORNER_BACKUP
                return

            err = self._yaw_err(self.side_heading_yaw)
            wz = max(-self.straight_max_wz, min(self.straight_max_wz, self.straight_kp * err))
            self._pub(self.fwd_spd, wz)
            return

        # ---- CROSS_GAP ----
        if self.stage == Stage.CROSS_GAP:
            front_tripped = self._get_tripped_sensor(('front',))
            rear_tripped = self._get_tripped_sensor(('rear',))

            if not front_tripped and not rear_tripped:
                # Whole chassis clear - both ends read solid panel, not
                # just the front tip.
                self._stop()
                self._snap_xy()
                self.completed_sides = 0
                self.get_logger().info(
                    '📍 Front and rear both confirm next panel. Recalibrating baselines '
                    '(now on a different physical panel - a stale baseline previously let '
                    'open roof false-positive as solid panel here)...')
                self.after_calibrate_stage = Stage.FOLLOW_SIDE
                self.calib_start = None
                self._enter_settle(Stage.RECALIBRATE)
                return

            if self._dist_from_snap() >= self.gap_cross_max_dist:
                self._stop()
                self.get_logger().warn(
                    f'⚠️ No next panel confirmed within {self.gap_cross_max_dist}m - the grid plan '
                    f'expected one here. Stopping rather than continuing blind.')
                self.stage = Stage.DONE
                return

            err = self._yaw_err(self.side_heading_yaw)
            wz = max(-self.straight_max_wz, min(self.straight_max_wz, self.straight_kp * err))
            self._pub(self.fwd_spd, wz)
            return

        # ---- CORNER_BACKUP ----
        if self.stage == Stage.CORNER_BACKUP:
            rear_tripped = self._get_tripped_sensor(('rear',))
            backup_done = self._dist_from_snap() >= self.corner_backup_dist

            if rear_tripped or backup_done:
                if rear_tripped:
                    self._publish_edge_point(rear_tripped)
                self._stop()
                current_heading = self._heading()

                if self.turning_for_cross:
                    # Deliberate grid-plan heading, not a fixed +90deg
                    # corner turn - could be +90, +180, or -90 depending
                    # on where the actual next panel is.
                    self.turn_target_yaw = self.cross_target_yaw
                else:
                    grid_cardinal = round(current_heading / (math.pi / 2.0)) * (math.pi / 2.0)
                    self.turn_target_yaw = self._norm_angle(grid_cardinal + self.turn_angle)

                self.get_logger().info(
                    f'🔄 Corner Backup complete. Turning '
                    f'(Current: {math.degrees(current_heading):.1f}°, Target: {math.degrees(self.turn_target_yaw):.1f}°)...'
                )
                self.stage = Stage.TURN_CORNER
                return

            self._pub(self.bkp_spd, 0.0)
            return

        # ---- TURN_CORNER ----
        if self.stage == Stage.TURN_CORNER:
            sensors_to_check = ('rear',) if self.completed_sides == 0 else ('front',)
            tripped_key = self._get_tripped_sensor(sensors_to_check)

            if tripped_key is not None:
                self._stop()
                self._snap_xy()
                if tripped_key == 'rear':
                    self.adjust_spd = self.fwd_spd
                    self.get_logger().warn(f'⚠️ Rear sensor [{tripped_key}] hit edge mid-turn! Creeping FORWARD...')
                else:
                    self.adjust_spd = self.bkp_spd
                    self.get_logger().warn(f'⚠️ Front sensor [{tripped_key}] hit edge mid-turn! Creeping BACKWARD...')

                self.stage = Stage.PANEL_ADJUST
                return

            err = self._yaw_err(self.turn_target_yaw)
            if abs(err) <= self.turn_tol:
                self._stop()
                self._snap_xy()

                self.side_heading_yaw = self.turn_target_yaw

                if self.turning_for_cross:
                    self.turning_for_cross = False
                    self.get_logger().info('✅ Facing next panel. Crossing gap...')
                    self.stage = Stage.CROSS_GAP
                    return

                corner_pt = Point()
                corner_pt.x, corner_pt.y, corner_pt.z = self.x, self.y, 0.0
                self.panel_corner_pub.publish(corner_pt)
                self.get_logger().info('✅ Turn complete. Aligning frame to cardinal axis...')
                self.stage = Stage.ALIGN_BACKUP
                return

            p_turn_spd = max(0.18, min(self.turn_spd, 1.2 * abs(err)))
            self._pub(0.0, math.copysign(p_turn_spd, err))
            return

        # ---- PANEL_ADJUST ----
        if self.stage == Stage.PANEL_ADJUST:
            if self._dist_from_snap() >= self.adjust_dist:
                self._stop()
                self.get_logger().info('↩️ Nudge complete. Resuming turn...')
                self.stage = Stage.TURN_CORNER
                return
            self._pub(self.adjust_spd, 0.0)
            return

        # ---- ALIGN_BACKUP ----
        if self.stage == Stage.ALIGN_BACKUP:
            tripped = self._get_tripped_sensor(('rear',))
            if tripped or self._dist_from_snap() >= self.align_backup_dist:
                if tripped:
                    self._publish_edge_point(tripped)
                self._stop()
                self.completed_sides += 1
                if self.completed_sides >= self.num_sides:
                    self.stage = Stage.DONE
                    self.get_logger().info('✅ FULL PERIMETER COMPLETED!')
                    return

                self._snap_xy()
                self.get_logger().info(f'🔍 Nudging outward to locate edge for Side {self.completed_sides + 1}...')
                self.stage = Stage.ALIGN_TO_EDGE
                return
            self._pub(self.bkp_spd, 0.0)
            return

        # ---- ALIGN_TO_EDGE ----
        if self.stage == Stage.ALIGN_TO_EDGE:
            tripped = self._get_tripped_sensor(('front',))

            if tripped or self._dist_from_snap() >= self.edge_find_cap:
                if tripped:
                    self._publish_edge_point(tripped)
                self._stop()
                self._snap_xy()
                self.get_logger().info(f'📍 Edge located for Side {self.completed_sides + 1}! Starting side follow...')
                self._enter_settle(Stage.FOLLOW_SIDE)
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
    node = SolarbotPerimeterNode()
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