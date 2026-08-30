#!/usr/bin/env python3
"""
solarbot_perimeter_openloop_node.py

Odom-free perimeter-following FSM for table/panel edge testing.
- Uses IMU yaw for straight-line tracking and 90-degree corner spins.
- Uses timed open-loop displacements for micro-backups and nudges.
- Uses qos_profile_sensor_data (BEST_EFFORT) for hardware sensor compatibility.
"""

import math
from collections import deque
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

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
    DONE          = auto()


class SolarbotPerimeterOpenLoopNode(Node):
    def __init__(self):
        super().__init__('solarbot_perimeter_openloop_node')

        # ---------------- Configurable Parameters ----------------
        self.declare_parameter('forward_speed',         0.12)
        self.declare_parameter('backup_speed',        -0.08)
        self.declare_parameter('turn_speed',           0.40)
        self.declare_parameter('turn_angle_deg',       90.0)
        self.declare_parameter('turn_tolerance_deg',   1.5)

        # Distances (Converted to duration: t = dist / |speed|)
        self.declare_parameter('init_backup_dist_m',    0.06)
        self.declare_parameter('corner_backup_dist_m',  0.02)
        self.declare_parameter('align_backup_dist_m',   0.02)
        self.declare_parameter('edge_find_cap_m',       0.15)
        self.declare_parameter('adjust_dist_m',         0.04)

        self.declare_parameter('settle_sec',            0.40)
        self.declare_parameter('straight_kp',           2.5)
        self.declare_parameter('straight_max_wz',       0.30)
        self.declare_parameter('num_sides',             4)

        # Sensor Filtering
        self.declare_parameter('filter_window',        5)
        self.declare_parameter('min_filter_samples',    3)
        self.declare_parameter('calibration_sec',       1.0)
        self.declare_parameter('gap_delta_m',           0.008)

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

        self.settle_sec       = float(p('settle_sec').value)
        self.straight_kp      = float(p('straight_kp').value)
        self.straight_max_wz  = float(p('straight_max_wz').value)
        self.num_sides        = int(p('num_sides').value)

        self.filter_window      = int(p('filter_window').value)
        self.min_filter_samples = int(p('min_filter_samples').value)
        self.calibration_sec    = float(p('calibration_sec').value)
        self.gap_delta          = float(p('gap_delta_m').value)

        self.cmd_vel_topic = str(p('cmd_vel_topic').value)

        # Buffers
        self.front_buf = deque(maxlen=self.filter_window)
        self.rear_buf  = deque(maxlen=self.filter_window)
        self.left_buf  = deque(maxlen=self.filter_window)
        self.right_buf = deque(maxlen=self.filter_window)

        self.baseline = {'front': None, 'rear': None, 'left': None, 'right': None}
        self.calib_start = None

        # State Tracking
        self.have_imu  = False
        self.imu_yaw   = 0.0

        self.stage           = Stage.CALIBRATE
        self.completed_sides = 0

        self.turn_target_yaw  = 0.0
        self.side_heading_yaw = 0.0

        self.settle_end         = self.get_clock().now()
        self.action_timeout_end = self.get_clock().now()
        self.after_settle_stage = Stage.FOLLOW_SIDE

        self.adjust_spd = self.fwd_spd

        # ROS 2 Communications
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # BEST_EFFORT QoS subscriptions
        self.create_subscription(LaserScan,  '/front_mid_tof',        self._front_cb, qos_profile=qos_profile_sensor_data)
        self.create_subscription(LaserScan,  '/rear_mid_tof',         self._rear_cb,  qos_profile=qos_profile_sensor_data)
        self.create_subscription(LaserScan,  '/left_mid_tof',         self._left_cb,  qos_profile=qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/right_mid_tof/points', self._right_cb, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Imu,        '/imu',                  self._imu_cb,   qos_profile=qos_profile_sensor_data)

        self.create_timer(0.05, self._loop)
        self.get_logger().info('🚀 SolarBot Odom-Free Perimeter Controller Initialized')

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
        for x, y, z in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
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

    def _norm_angle(self, a):
        while a > math.pi:  a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    def _yaw_err(self, target):
        return self._norm_angle(target - self._heading())

    def _pub(self, vx=0.0, wz=0.0):
        cmd = Twist()
        cmd.linear.x  = float(vx) if math.isfinite(vx) else 0.0
        cmd.angular.z = float(wz) if math.isfinite(wz) else 0.0
        self.cmd_pub.publish(cmd)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _set_action_duration(self, dist_m, speed):
        duration_sec = abs(dist_m / speed) if abs(speed) > 1e-5 else 0.5
        self.action_timeout_end = self.get_clock().now() + Duration(seconds=duration_sec)

    def _action_timed_out(self):
        return self.get_clock().now() >= self.action_timeout_end

    def _enter_settle(self, next_stage):
        self.stage = Stage.SETTLE
        self.after_settle_stage = next_stage
        self.settle_end = self.get_clock().now() + Duration(seconds=self.settle_sec)

    def _enter_follow_side(self):
        self.side_heading_yaw = self._heading()
        self.stage = Stage.FOLLOW_SIDE
        self.get_logger().info(f'▶️ SIDE {self.completed_sides + 1} (Heading: {math.degrees(self.side_heading_yaw):.1f}°)')

    def _loop(self):
        if not self.have_imu:
            self._stop()
            return
        if not self._filters_ready():
            self._stop()
            return

        # ---- 1. CALIBRATE ----
        if self.stage == Stage.CALIBRATE:
            self._stop()
            if self.calib_start is None:
                self.calib_start = self.get_clock().now()
                self.get_logger().info('🔧 Calibrating ToF baselines on table surface...')
                return
            elapsed = (self.get_clock().now() - self.calib_start).nanoseconds * 1e-9
            if elapsed >= self.calibration_sec:
                readings = self._filtered_readings()
                for k in self.baseline:
                    self.baseline[k] = readings[k]
                self.get_logger().info(f"✅ Baselines set: {self.baseline}")
                self._set_action_duration(self.init_backup_dist, self.bkp_spd)
                self.stage = Stage.INIT_BACKUP
            return

        # ---- 2. INIT_BACKUP ----
        if self.stage == Stage.INIT_BACKUP:
            tripped = self._get_tripped_sensor(('rear',))
            if tripped or self._action_timed_out():
                self._stop()
                self._enter_settle(Stage.FOLLOW_SIDE)
                return
            self._pub(self.bkp_spd, 0.0)
            return

        # ---- 3. FOLLOW_SIDE ----
        if self.stage == Stage.FOLLOW_SIDE:
            tripped = self._get_tripped_sensor(('front',))
            if tripped:
                self._stop()
                self.get_logger().info(f'⚠️ Edge reached on Side {self.completed_sides + 1} via [{tripped}] — backing up...')
                self._set_action_duration(self.corner_backup_dist, self.bkp_spd)
                self.stage = Stage.CORNER_BACKUP
                return

            err = self._yaw_err(self.side_heading_yaw)
            wz = max(-self.straight_max_wz, min(self.straight_max_wz, self.straight_kp * err))
            self._pub(self.fwd_spd, wz)
            return

        # ---- 4. CORNER_BACKUP ----
        if self.stage == Stage.CORNER_BACKUP:
            rear_tripped = self._get_tripped_sensor(('rear',))
            if rear_tripped or self._action_timed_out():
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

        # ---- 5. TURN_CORNER ----
        if self.stage == Stage.TURN_CORNER:
            sensors_to_check = ('rear',) if self.completed_sides == 0 else ('front',)
            tripped_key = self._get_tripped_sensor(sensors_to_check)

            if tripped_key is not None:
                self._stop()
                self.adjust_spd = self.fwd_spd if tripped_key == 'rear' else self.bkp_spd
                self.get_logger().warn(f'⚠️ Sensor [{tripped_key}] hit edge mid-turn! Creeping...')
                self._set_action_duration(self.adjust_dist, self.adjust_spd)
                self.stage = Stage.PANEL_ADJUST
                return

            err = self._yaw_err(self.turn_target_yaw)
            if abs(err) <= self.turn_tol:
                self._stop()
                self.side_heading_yaw = self.turn_target_yaw
                self.get_logger().info('✅ Turn complete. Aligning frame to edge...')
                self._set_action_duration(self.align_backup_dist, self.bkp_spd)
                self.stage = Stage.ALIGN_BACKUP
                return

            p_turn_spd = max(0.18, min(self.turn_speed, 1.2 * abs(err)))
            self._pub(0.0, math.copysign(p_turn_spd, err))
            return

        # ---- 6. PANEL_ADJUST ----
        if self.stage == Stage.PANEL_ADJUST:
            if self._action_timed_out():
                self._stop()
                self.get_logger().info('↩️ Nudge complete. Resuming turn...')
                self.stage = Stage.TURN_CORNER
                return
            self._pub(self.adjust_spd, 0.0)
            return

        # ---- 7. ALIGN_BACKUP ----
        if self.stage == Stage.ALIGN_BACKUP:
            tripped = self._get_tripped_sensor(('rear',))
            if tripped or self._action_timed_out():
                self._stop()
                self.completed_sides += 1
                if self.completed_sides >= self.num_sides:
                    self.stage = Stage.DONE
                    self.get_logger().info('🎉 FULL PERIMETER COMPLETED!')
                    return

                self.get_logger().info(f'🔍 Nudging outward for Side {self.completed_sides + 1}...')
                self._set_action_duration(self.edge_find_cap, self.fwd_spd * 0.5)
                self.stage = Stage.ALIGN_TO_EDGE
                return
            self._pub(self.bkp_spd, 0.0)
            return

        # ---- 8. ALIGN_TO_EDGE ----
        if self.stage == Stage.ALIGN_TO_EDGE:
            tripped = self._get_tripped_sensor(('front',))
            if tripped or self._action_timed_out():
                self._stop()
                self.get_logger().info(f'📍 Edge located for Side {self.completed_sides + 1}! Starting side follow...')
                self._enter_settle(Stage.FOLLOW_SIDE)
                return

            self._pub(self.fwd_spd * 0.5, 0.0)
            return

        # ---- 9. SETTLE ----
        if self.stage == Stage.SETTLE:
            self._stop()
            if self.get_clock().now() >= self.settle_end:
                if self.after_settle_stage == Stage.FOLLOW_SIDE:
                    self._enter_follow_side()
                else:
                    self.stage = self.after_settle_stage
            return

        # ---- 10. DONE ----
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