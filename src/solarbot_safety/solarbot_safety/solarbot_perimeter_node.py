#!/usr/bin/env python3
"""
solarbot_perimeter_node.py

Perimeter-following FSM with grid-snapped target math and dynamic deceleration
to eliminate turn overshoot.
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
    SETTLE        = auto()
    DONE          = auto()


class SolarbotPerimeterNode(Node):
    def __init__(self):
        super().__init__('solarbot_perimeter_node')

        # ---------------- Configurable Parameters ----------------
        self.declare_parameter('forward_speed',        0.12)
        self.declare_parameter('backup_speed',        -0.10)
        self.declare_parameter('turn_speed',           0.45)   # Reduced from 0.75 for fine stopping
        self.declare_parameter('turn_angle_deg',       90.0)
        self.declare_parameter('turn_tolerance_deg',   1.5)   # Tighter tolerance

        self.declare_parameter('init_backup_dist_m',    0.10)
        self.declare_parameter('corner_backup_dist_m',  0.12)
        self.declare_parameter('align_backup_dist_m',   0.08)
        self.declare_parameter('adjust_dist_m',         0.05)

        self.declare_parameter('settle_sec',            0.40)   # Increased settle time for IMU stabilization
        self.declare_parameter('straight_kp',           2.5)
        self.declare_parameter('straight_max_wz',       0.35)
        self.declare_parameter('num_sides',             4)

        # Sensor filtering / calibration
        self.declare_parameter('filter_window',        15)
        self.declare_parameter('min_filter_samples',    10)
        self.declare_parameter('calibration_sec',       1.0)
        self.declare_parameter('gap_delta_m',           0.008)

        # Topics
        self.declare_parameter('odom_topic',     '/diff_drive_controller/odom')
        self.declare_parameter('cmd_vel_topic',  '/cmd_vel')

        p = self.get_parameter
        self.fwd_spd          = float(p('forward_speed').value)
        self.bkp_spd          = float(p('backup_speed').value)
        self.turn_spd         = float(p('turn_speed').value)
        self.turn_angle       = math.radians(float(p('turn_angle_deg').value))
        self.turn_tol         = math.radians(float(p('turn_tolerance_deg').value))

        self.init_backup_dist    = float(p('init_backup_dist_m').value)
        self.corner_backup_dist  = float(p('corner_backup_dist_m').value)
        self.align_backup_dist   = float(p('align_backup_dist_m').value)
        self.adjust_dist         = float(p('adjust_dist_m').value)

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

        self.create_subscription(LaserScan, '/front_left_tof',  self._fl_cb, 10)
        self.create_subscription(LaserScan, '/front_right_tof', self._fr_cb, 10)
        self.create_subscription(LaserScan, '/rear_left_tof',   self._rl_cb, 10)
        self.create_subscription(LaserScan, '/rear_right_tof',  self._rr_cb, 10)
        self.create_subscription(Odometry,  self.odom_topic,    self._odom_cb, 10)
        self.create_subscription(Imu,       '/imu',             self._imu_cb, 10)

        self.create_timer(0.05, self._loop)
        self.get_logger().info('🚀 Perimeter Controller Active (Grid-Snapped Precision Turning)')

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
            if self._get_tripped_sensor(('rl', 'rr')) or self._dist_from_snap() >= self.init_backup_dist:
                self._stop()
                self._enter_settle(Stage.FOLLOW_SIDE)
                return
            self._pub(self.bkp_spd, 0.0)
            return

        # ---- FOLLOW_SIDE ----
        if self.stage == Stage.FOLLOW_SIDE:
            if self._get_tripped_sensor(('fl', 'fr')):
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
                self._stop()
                current_heading = self._heading()
                
                # GRID-SNAPPED TARGET: Snap current heading to nearest 90° increment, then add 90°
                grid_cardinal = round(current_heading / (math.pi / 2.0)) * (math.pi / 2.0)
                self.turn_target_yaw = self._norm_angle(grid_cardinal + self.turn_angle)
                
                self.get_logger().info(
                    f'🔄 Corner Backup complete. Turning 90° '
                    f'(Current: {math.degrees(current_heading):.1f}°, Snapped Target: {math.degrees(self.turn_target_yaw):.1f}°)...'
                )
                self.stage = Stage.TURN_CORNER
                return

            self._pub(self.bkp_spd, 0.0)
            return

        # ---- TURN_CORNER ----
        if self.stage == Stage.TURN_CORNER:
            # Check REAR sensors during Turn 1, FRONT sensors during Turn 2+
            sensors_to_check = ('rl', 'rr') if self.completed_sides == 0 else ('fl', 'fr')
            tripped_key = self._get_tripped_sensor(sensors_to_check)

            if tripped_key is not None:
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
                self.get_logger().info('✅ Turn complete. Aligning for next side...')
                self.stage = Stage.ALIGN_BACKUP
                return

            # Proportional angular speed deceleration to prevent overshoot
            p_turn_spd = max(0.20, min(self.turn_spd, 1.2 * abs(err)))
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
            if self._get_tripped_sensor(('rl', 'rr')) or self._dist_from_snap() >= self.align_backup_dist:
                self._stop()
                self.completed_sides += 1
                if self.completed_sides >= self.num_sides:
                    self.stage = Stage.DONE
                    self.get_logger().info('✅ FULL PERIMETER COMPLETED!')
                    return
                self._enter_settle(Stage.FOLLOW_SIDE)
                return
            self._pub(self.bkp_spd, 0.0)
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