#!/usr/bin/env python3
"""
solarbot_roof_coverage_node.py

Multi-panel roof coverage: traces each panel's perimeter (grid-snapped
targets, dynamic turn deceleration, edge-realignment after every turn),
then drives forward over the gap onto the next panel in the same
column and repeats, instead of stopping after one panel.

Separate node from solarbot_perimeter_node.py (single-panel only,
left untouched) - use this one for full-column roof coverage.
"""

import math
from collections import deque
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu


class Stage(Enum):
    CALIBRATE     = auto()
    INIT_BACKUP   = auto()
    FOLLOW_SIDE   = auto()
    CORNER_BACKUP = auto()
    TURN_CORNER   = auto()
    PANEL_ADJUST  = auto()
    ALIGN_BACKUP  = auto()
    ALIGN_TO_EDGE = auto()  # Edge-finding routine for Sides 2, 3, 4
    CROSS_GAP     = auto()  # Drive over the gap onto the next panel
    SETTLE        = auto()
    DONE          = auto()


class SolarbotRoofCoverageNode(Node):
    def __init__(self):
        super().__init__('solarbot_roof_coverage_node')

        # ---------------- Configurable Parameters ----------------
        self.declare_parameter('forward_speed',         0.12)   # Reduced for higher ToF sampling density
        self.declare_parameter('backup_speed',        -0.08)
        self.declare_parameter('turn_speed',           0.40)   # Controlled turn speed
        self.declare_parameter('turn_angle_deg',       90.0)
        self.declare_parameter('turn_tolerance_deg',   1.2)   # Tighter tolerance

        # Distances
        self.declare_parameter('init_backup_dist_m',    0.06)
        self.declare_parameter('corner_backup_dist_m',  0.04)   # Reduced so pivot stays closer to edge
        self.declare_parameter('align_backup_dist_m',   0.04)   # Reduced
        self.declare_parameter('edge_find_cap_m',       0.15)   # Safety search distance
        self.declare_parameter('adjust_dist_m',         0.04)

        self.declare_parameter('settle_sec',            0.40)   # IMU stabilization pause
        self.declare_parameter('straight_kp',           2.5)
        self.declare_parameter('straight_max_wz',       0.30)
        self.declare_parameter('num_sides',             4)

        # Multi-panel traversal: after finishing one panel's perimeter,
        # drive forward across the gap onto the next panel instead of
        # stopping. Panel spacing in warehouse_rooftop.sdf is 2.1m
        # center-to-center, panel Y-size 1.8m -> ~0.3m physical gap.
        # total_panels=5 matches one full column (panel_1..panel_5).
        self.declare_parameter('total_panels',          5)
        self.declare_parameter('max_gap_cross_dist_m',  1.0)  # safety cap

        # Sensor filtering / calibration
        self.declare_parameter('filter_window',        15)
        self.declare_parameter('min_filter_samples',    10)
        self.declare_parameter('calibration_sec',       1.0)
        self.declare_parameter('gap_delta_m',           0.008)

        # Topics
        self.declare_parameter('odom_topic',     '/diff_drive_controller/odom')
        self.declare_parameter('cmd_vel_topic',  '/cmd_vel')

        # ToF sensor mounting offsets relative to base_link origin - MUST
        # match common/properties.xacro. Used to compute the TRUE physical
        # edge location when a sensor trips (the sensor is not at the
        # robot's center, so the edge is at base_link position + this
        # offset rotated into the world frame, not at base_link itself).
        self.declare_parameter('front_left_tof_x',   0.200)
        self.declare_parameter('front_left_tof_y',   0.075)
        self.declare_parameter('front_right_tof_x',  0.200)
        self.declare_parameter('front_right_tof_y', -0.075)
        self.declare_parameter('rear_left_tof_x',   -0.200)
        self.declare_parameter('rear_left_tof_y',    0.075)
        self.declare_parameter('rear_right_tof_x',  -0.200)
        self.declare_parameter('rear_right_tof_y',  -0.075)

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

        self.settle_sec       = float(p('settle_sec').value)
        self.straight_kp      = float(p('straight_kp').value)
        self.straight_max_wz  = float(p('straight_max_wz').value)
        self.num_sides        = int(p('num_sides').value)
        self.total_panels        = int(p('total_panels').value)
        self.max_gap_cross_dist  = float(p('max_gap_cross_dist_m').value)

        self.filter_window      = int(p('filter_window').value)
        self.min_filter_samples = int(p('min_filter_samples').value)
        self.calibration_sec    = float(p('calibration_sec').value)
        self.gap_delta          = float(p('gap_delta_m').value)

        self.odom_topic          = str(p('odom_topic').value)
        self.cmd_vel_topic       = str(p('cmd_vel_topic').value)

        self.sensor_offsets = {
            'fl': (float(p('front_left_tof_x').value), float(p('front_left_tof_y').value)),
            'fr': (float(p('front_right_tof_x').value), float(p('front_right_tof_y').value)),
            'rl': (float(p('rear_left_tof_x').value), float(p('rear_left_tof_y').value)),
            'rr': (float(p('rear_right_tof_x').value), float(p('rear_right_tof_y').value)),
        }

        # Buffers
        self.fl_buf = deque(maxlen=self.filter_window)
        self.fr_buf = deque(maxlen=self.filter_window)
        self.rl_buf = deque(maxlen=self.filter_window)
        self.rr_buf = deque(maxlen=self.filter_window)

        self.baseline = {'fl': None, 'fr': None, 'rl': None, 'rr': None}
        self.calib_start = None

        # State Tracking
        self.have_odom = False
        self.have_imu  = False
        self.x = self.y = 0.0
        self.odom_yaw = 0.0
        self.imu_yaw  = 0.0

        self.stage           = Stage.CALIBRATE
        self.completed_sides = 0
        self.panels_completed = 0
        self.gap_phase = None  # 'waiting_for_gap' -> 'waiting_for_landing'
        self.snap_x = self.snap_y = 0.0

        self.turn_target_yaw  = 0.0
        self.side_heading_yaw = 0.0

        self.settle_end         = self.get_clock().now()
        self.after_settle_stage = Stage.FOLLOW_SIDE
        self.init_snap_set      = False

        self.adjust_spd = self.fwd_spd

        # ROS 2 Communications
        self.cmd_pub          = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.panel_corner_pub = self.create_publisher(Point, '/panel_corner', 10)
        # Every individual ToF edge-trip event (not just the 4 corners) -
        # builds a full physical-sensor-based boundary trace, independent
        # of camera odometry drift.
        self.edge_point_pub   = self.create_publisher(Point, '/panel_edge_points', 10)

        self.create_subscription(LaserScan, '/front_left_tof',  self._fl_cb, 10)
        self.create_subscription(LaserScan, '/front_right_tof', self._fr_cb, 10)
        self.create_subscription(LaserScan, '/rear_left_tof',   self._rl_cb, 10)
        self.create_subscription(LaserScan, '/rear_right_tof',  self._rr_cb, 10)
        self.create_subscription(Odometry,  self.odom_topic,    self._odom_cb, 10)
        self.create_subscription(Imu,       '/imu',             self._imu_cb, 10)

        self.create_timer(0.05, self._loop)
        self.get_logger().info('🚀 SolarBot Uniform Perimeter Controller Active')

    def _scan_min(self, msg):
        vals = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        return min(vals) if vals else math.inf

    def _fl_cb(self, msg):
        v = self._scan_min(msg)
        if math.isfinite(v): self.fl_buf.append(v)

    def _fr_cb(self, msg):
        v = self._scan_min(msg)
        if math.isfinite(v): self.fr_buf.append(v)

    def _rl_cb(self, msg):
        v = self._scan_min(msg)
        if math.isfinite(v): self.rl_buf.append(v)

    def _rr_cb(self, msg):
        v = self._scan_min(msg)
        if math.isfinite(v): self.rr_buf.append(v)

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
            'fl': self._mean(self.fl_buf),
            'fr': self._mean(self.fr_buf),
            'rl': self._mean(self.rl_buf),
            'rr': self._mean(self.rr_buf),
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
        """Transform a tripped sensor's local mounting offset into the
        world frame using the robot's current pose - this is the TRUE
        physical location of the detected edge, not the robot's center."""
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

    def _enter_settle(self, next_stage):
        self.stage = Stage.SETTLE
        self.after_settle_stage = next_stage
        self.settle_end = self.get_clock().now() + Duration(seconds=self.settle_sec)

    def _enter_follow_side(self):
        self.side_heading_yaw = self._heading()
        self.stage = Stage.FOLLOW_SIDE
        self._snap_xy()
        self.get_logger().info(f'▶️ SIDE {self.completed_sides + 1} (Heading: {math.degrees(self.side_heading_yaw):.1f}°)')

    def _loop(self):
        if not (self.have_odom and self.have_imu):
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
                self._snap_xy()
                self.init_snap_set = True
            tripped = self._get_tripped_sensor(('rl', 'rr'))
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
            tripped = self._get_tripped_sensor(('fl', 'fr'))
            if tripped:
                self._publish_edge_point(tripped)
                is_last_side = (self.completed_sides == self.num_sides - 1)
                more_panels_remain = (self.panels_completed + 1 < self.total_panels)
                if is_last_side and more_panels_remain:
                    # Don't stop, don't do the closing turn - that would
                    # face the robot back toward its ORIGINAL start
                    # heading, not toward the next panel. Keep driving
                    # straight through in the current heading, directly
                    # into gap-crossing.
                    self.completed_sides += 1
                    self.panels_completed += 1
                    self.get_logger().info(
                        f'✅ Panel {self.panels_completed}/{self.total_panels} perimeter complete! '
                        f'Skipping closing turn - continuing straight onto next panel...')
                    self._snap_xy()
                    # Already tripped right now - CROSS_GAP's own check
                    # will see this immediately and move to
                    # waiting_for_landing on its very next iteration,
                    # continuing the same forward motion with no stop.
                    self.gap_phase = 'waiting_for_gap'
                    self.stage = Stage.CROSS_GAP
                    return
                self._stop()
                self._snap_xy()
                self.get_logger().info(f'⚠️ Edge reached on Side {self.completed_sides + 1} — backing up for turn...')
                self.stage = Stage.CORNER_BACKUP
                return

            err = self._yaw_err(self.side_heading_yaw)
            wz = max(-self.straight_max_wz, min(self.straight_max_wz, self.straight_kp * err))
            self._pub(self.fwd_spd, wz)
            return

        # ---- CORNER_BACKUP ----
        if self.stage == Stage.CORNER_BACKUP:
            rear_tripped = self._get_tripped_sensor(('rl', 'rr'))
            backup_done = self._dist_from_snap() >= self.corner_backup_dist

            if rear_tripped or backup_done:
                if rear_tripped:
                    self._publish_edge_point(rear_tripped)
                self._stop()
                current_heading = self._heading()

                # Snap heading to nearest 90° cardinal grid
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

        # ---- TURN_CORNER ----
        if self.stage == Stage.TURN_CORNER:
            sensors_to_check = ('rl', 'rr') if self.completed_sides == 0 else ('fl', 'fr')
            tripped_key = self._get_tripped_sensor(sensors_to_check)

            if tripped_key is not None:
                # NOTE: deliberately NOT calling _publish_edge_point here.
                # The robot is pivoting in place during this stage, so a
                # trip means the sensor swept OVER the edge mid-rotation -
                # that's a safety-correction trigger for PANEL_ADJUST, not
                # a genuine "found the boundary by driving into it" event
                # like the ones in FOLLOW_SIDE/ALIGN_TO_EDGE. Including
                # these polluted the edge trace with pivot-arc artifacts.
                self._stop()
                self._snap_xy()
                if tripped_key in ('rl', 'rr'):
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
                # This is a true 90-degree corner of the panel - mark it
                # distinctly on /panel_corner (separate from the continuous
                # /panel_edge_points trace).
                corner_pt = Point()
                corner_pt.x, corner_pt.y, corner_pt.z = self.x, self.y, 0.0
                self.panel_corner_pub.publish(corner_pt)
                self.get_logger().info('✅ Turn complete. Aligning frame...')
                self.stage = Stage.ALIGN_BACKUP
                return

            # Dynamic deceleration to prevent turn overshoot
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
            tripped = self._get_tripped_sensor(('rl', 'rr'))
            if tripped or self._dist_from_snap() >= self.align_backup_dist:
                if tripped:
                    self._publish_edge_point(tripped)
                self._stop()
                self.completed_sides += 1
                if self.completed_sides >= self.num_sides:
                    self.panels_completed += 1
                    self.get_logger().info(
                        f'✅ Panel {self.panels_completed}/{self.total_panels} perimeter complete!')
                    if self.panels_completed >= self.total_panels:
                        self.stage = Stage.DONE
                        self.get_logger().info('🏁 ALL PANELS COMPLETE!')
                        return
                    # More panels remain - drive forward over the gap
                    # onto the next one instead of stopping.
                    self._snap_xy()
                    self.gap_phase = 'waiting_for_gap'
                    self.get_logger().info(
                        f'➡️ Crossing gap to panel {self.panels_completed + 1}...')
                    self.stage = Stage.CROSS_GAP
                    return

                # Transition to Edge Finding
                self._snap_xy()
                self.get_logger().info(f'🔍 Nudging outward to locate edge for Side {self.completed_sides + 1}...')
                self.stage = Stage.ALIGN_TO_EDGE
                return
            self._pub(self.bkp_spd, 0.0)
            return

        # ---- CROSS_GAP ----
        # Two-phase sensor watch, not a blind fixed-distance drive:
        # first wait for the front sensors to trip (confirms we've
        # actually started crossing the gap, not still sitting on the
        # panel we just finished), THEN wait for them to un-trip again
        # (confirms we've landed on solid panel on the far side). Using
        # "not tripped" alone as the exit condition would falsely fire
        # immediately, since that's also true before crossing starts.
        if self.stage == Stage.CROSS_GAP:
            tripped = self._get_tripped_sensor(('fl', 'fr'))
            dist = self._dist_from_snap()

            if dist >= self.max_gap_cross_dist:
                # Never found solid ground again within the safety cap -
                # either genuinely no more panels, or something's wrong.
                # Treat as finished rather than driving forever.
                self._stop()
                self.get_logger().warn(
                    f'⚠️ No panel found after crossing {dist:.2f}m - '
                    f'stopping as if all panels are complete.')
                self.stage = Stage.DONE
                return

            if self.gap_phase == 'waiting_for_gap':
                if tripped:
                    self.gap_phase = 'waiting_for_landing'
                self._pub(self.fwd_spd, 0.0)
                return

            if self.gap_phase == 'waiting_for_landing':
                if not tripped:
                    self._stop()
                    self._snap_xy()
                    self.completed_sides = 0
                    self.gap_phase = None
                    self.get_logger().info(
                        f'📍 Landed on panel {self.panels_completed + 1}. Recalibrating...')
                    self.after_calibrate_stage = Stage.INIT_BACKUP
                    self.stage = Stage.CALIBRATE
                    return
                self._pub(self.fwd_spd, 0.0)
                return
            return

        # ---- ALIGN_TO_EDGE ----
        if self.stage == Stage.ALIGN_TO_EDGE:
            tripped = self._get_tripped_sensor(('fl', 'fr'))

            # Creep forward until front sensor finds the edge or safety limit is reached
            if tripped or self._dist_from_snap() >= self.edge_find_cap:
                if tripped:
                    self._publish_edge_point(tripped)
                self._stop()
                self._snap_xy()
                self.get_logger().info(f'📍 Edge located for Side {self.completed_sides + 1}! Starting side follow...')

                # Enter settle before beginning straight flight
                self._enter_settle(Stage.FOLLOW_SIDE)
                return

            # Slow forward creep
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
    node = SolarbotRoofCoverageNode()
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