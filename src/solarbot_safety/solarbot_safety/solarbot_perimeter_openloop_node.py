#!/usr/bin/env python3
"""
solarbot_lawnmower_mtof_node.py

Boustrophedon (Lawnmower) Coverage Path Node:
- SWEEP_LANE: Active IMU heading hold with anti-oscillation deadband.
- DIRECTION-AWARE SPACE CHECK: Checks the Right MToF ONLY when facing 
  180° opposite to start heading (facing the far boundary), avoiding false 
  stops on Lane 1 when starting against the reference edge.
- DUAL 90° PIVOTS: Direct-drive breakaway pivots without Nav2.
"""

import math
from enum import Enum, auto
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from sensor_msgs_py import point_cloud2

try:
    from mros_interfaces.msg import MotorCmd
    HAVE_MOTOR_CMD = True
except ImportError:
    HAVE_MOTOR_CMD = False


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        if dt <= 1e-4:
            return self.kp * error
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class Stage(Enum):
    CALIBRATE            = auto()
    SWEEP_LANE           = auto()
    CHECK_SPACE          = auto()
    MOVE_TO_CORNER_APEX  = auto()
    CORNER_PAUSE_1       = auto()
    TURN_1               = auto()
    COUNTER_BRAKE_1      = auto()
    SETTLE_1             = auto()
    TRIM_1               = auto()
    ADVANCE_STRIPE       = auto()
    CORNER_PAUSE_2       = auto()
    TURN_2               = auto()
    COUNTER_BRAKE_2      = auto()
    SETTLE_2             = auto()
    TRIM_2               = auto()
    ADVANCE_ENTRY        = auto()
    DONE                 = auto()


class SolarbotLawnmowerMtofNode(Node):
    def __init__(self):
        super().__init__('solarbot_lawnmower_mtof_node')

        # ---------------- Movement Speeds ----------------
        self.declare_parameter('base_speed',             0.22)
        self.declare_parameter('shift_speed',            0.22)
        self.declare_parameter('track_width',            0.20)

        # ---------------- Heading PID Parameters ----------------
        self.declare_parameter('pid_kp',                 0.90)
        self.declare_parameter('pid_ki',                 0.00)
        self.declare_parameter('pid_kd',                 0.10)
        self.declare_parameter('max_correction',         0.035)
        self.declare_parameter('yaw_deadband_deg',       0.80)

        # ---------------- IMU Turn Parameters ----------------
        self.declare_parameter('turn_speed',             3.50)
        self.declare_parameter('brake_lead_deg',         9.50)
        self.declare_parameter('counter_brake_spd',      2.40)
        self.declare_parameter('counter_brake_sec',      0.080)
        self.declare_parameter('trim_tolerance_deg',     1.20)
        self.declare_parameter('settle_time_sec',        0.30)

        # ---------------- MToF Parameters ----------------
        self.declare_parameter('surface_threshold_m',    0.18)
        self.declare_parameter('mtof_slope_m_per_col',   0.0325)
        self.declare_parameter('mtof_base_offset_m',     0.0940)
        self.declare_parameter('transpose_grid',         True)
        self.declare_parameter('flip_vertical',           True)
        self.declare_parameter('min_space_next_lane_m',  0.22)

        # ---------------- Sequential Timings ----------------
        self.declare_parameter('pre_turn_forward_sec',   0.45)
        self.declare_parameter('pause_duration_sec',     0.30)
        self.declare_parameter('stripe_shift_sec',       2.20)
        self.declare_parameter('advance_entry_sec',      0.70)
        self.declare_parameter('shift_debounce_sec',     0.40)
        self.declare_parameter('max_passes',             20)
        self.declare_parameter('gap_delta_m',            0.05)
        self.declare_parameter('corner_debounce_count',  2)
        self.declare_parameter('cmd_vel_topic',          '/cmd_vel')

        p = self.get_parameter
        self.base_spd        = float(p('base_speed').value)
        self.shift_spd       = float(p('shift_speed').value)
        self.track_width     = float(p('track_width').value)

        kp = float(p('pid_kp').value)
        ki = float(p('pid_ki').value)
        kd = float(p('pid_kd').value)
        self.pid_heading     = PID(kp, ki, kd)
        self.max_corr        = float(p('max_correction').value)
        self.yaw_deadband    = math.radians(float(p('yaw_deadband_deg').value))

        self.turn_angle      = math.radians(90.0)
        self.turn_spd        = float(p('turn_speed').value)
        self.brake_lead_rad  = math.radians(float(p('brake_lead_deg').value))
        self.counter_spd     = float(p('counter_brake_spd').value)
        self.counter_sec     = float(p('counter_brake_sec').value)
        self.trim_tol_rad    = math.radians(float(p('trim_tolerance_deg').value))
        self.settle_time     = float(p('settle_time_sec').value)

        self.thresh          = float(p('surface_threshold_m').value)
        self.mtof_slope      = float(p('mtof_slope_m_per_col').value)
        self.mtof_offset     = float(p('mtof_base_offset_m').value)
        self.transpose       = bool(p('transpose_grid').value)
        self.flip_vertical   = bool(p('flip_vertical').value)
        self.min_space_m     = float(p('min_space_next_lane_m').value)

        self.pre_turn_dur    = float(p('pre_turn_forward_sec').value)
        self.pause_dur       = float(p('pause_duration_sec').value)
        self.stripe_shift_dur= float(p('stripe_shift_sec').value)
        self.advance_dur     = float(p('advance_entry_sec').value)
        self.shift_blank_dur = float(p('shift_debounce_sec').value)
        self.max_passes      = int(p('max_passes').value)
        self.gap_delta       = float(p('gap_delta_m').value)
        self.debounce_req    = int(p('corner_debounce_count').value)

        # State tracking
        self.stage = Stage.CALIBRATE
        self.completed_lanes = 0
        self.initial_yaw = 0.0
        self.target_yaw = 0.0
        self.lane_heading = 0.0
        self.turn_start_yaw = 0.0
        self.active_turn_dir = 1.0
        self.turn_left = True

        self.last_loop_time = self.get_clock().now()
        self.brake_start = self.get_clock().now()
        self.settle_start = self.get_clock().now()
        self.stage_timer_start = self.get_clock().now()
        self.stage_timer_end = self.get_clock().now()

        self.has_moved = False
        self.trim_done = False
        self.edge_trip_streak = 0

        self.have_imu = False
        self.imu_yaw = 0.0

        # Sensor variables
        self.front_dist = None
        self.rear_dist = None
        self.baseline = {'front': None, 'rear': None}
        self.calib_samples = []
        self.latest_grid = None

        self.latest_edge_dist = None
        self.latest_edge_tilt = None

        # Publishers
        cmd_topic = str(p('cmd_vel_topic').value)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.status_pub = self.create_publisher(String, '/coverage_status', 10)
        self.edge_dist_pub = self.create_publisher(Float32, '/edge_distance', 10)
        self.edge_tilt_pub = self.create_publisher(Float32, '/edge_tilt', 10)

        if HAVE_MOTOR_CMD:
            self.motor_pub = self.create_publisher(MotorCmd, '/motorCmd', 10)
        else:
            self.motor_pub = None

        # Subscriptions
        self.create_subscription(LaserScan,   '/front_mid_tof',        self._front_cb, qos_profile=qos_profile_sensor_data)
        self.create_subscription(LaserScan,   '/rear_mid_tof',         self._rear_cb,  qos_profile=qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/right_mid_tof/points', self._cloud_cb, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Imu,         '/imu',                  self._imu_cb,   qos_profile=qos_profile_sensor_data)

        self.create_timer(0.02, self._loop)
        self.get_logger().info('🌾 SolarBot Lawnmower Node Active (180° Directional Space Gating).')

    def _front_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        if valid:
            self.front_dist = min(valid)

    def _rear_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        if valid:
            self.rear_dist = min(valid)

    def _imu_cb(self, msg: Imu):
        q = msg.quaternion if hasattr(msg, 'quaternion') else msg.orientation
        if not (math.isfinite(q.x) and math.isfinite(q.y) and math.isfinite(q.z) and math.isfinite(q.w)):
            return

        yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        self.imu_yaw = yaw
        self.have_imu = True

        if self.stage == Stage.CALIBRATE:
            self.calib_samples.append(yaw)

    def _cloud_cb(self, msg: PointCloud2):
        pts = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False))
        if len(pts) < 64:
            return

        raw_grid = np.zeros((8, 8), dtype=np.float32)
        for r in range(8):
            for c in range(8):
                x, y, z = pts[r * 8 + c]
                raw_grid[r, c] = math.sqrt(x*x + y*y + z*z) if (math.isfinite(x) and x > 0) else 9.99

        grid = raw_grid.T if self.transpose else raw_grid
        if self.flip_vertical:
            grid = np.flipud(grid)

        self.latest_grid = grid
        self._compute_and_publish_edge_mapping()

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

    def _compute_and_publish_edge_mapping(self):
        if self.latest_grid is None:
            return

        edge_pts_2d = []
        for r in range(8):
            col_cross = self._subpixel_edge(self.latest_grid[r, :])
            if col_cross is not None:
                edge_pts_2d.append([col_cross, float(r)])

        if len(edge_pts_2d) < 3:
            return

        pts_array = np.array(edge_pts_2d, dtype=np.float32)
        [vx, vy, x0, y0] = cv2.fitLine(pts_array, cv2.DIST_L2, 0, 0.01, 0.01)
        vx, vy, x0, y0 = float(vx[0]), float(vy[0]), float(x0[0]), float(y0[0])

        if vy < 0:
            vx = -vx
            vy = -vy

        tilt_deg = math.degrees(math.atan2(vx, vy))
        avg_col = x0

        total_edge_distance = (self.mtof_slope * avg_col) + self.mtof_offset
        self.latest_edge_dist = total_edge_distance
        self.latest_edge_tilt = tilt_deg

        msg_dist = Float32()
        msg_dist.data = float(total_edge_distance)
        self.edge_dist_pub.publish(msg_dist)

        msg_tilt = Float32()
        msg_tilt.data = float(tilt_deg)
        self.edge_tilt_pub.publish(msg_tilt)

    def _is_tripped(self, key):
        dist = self.front_dist if key == 'front' else self.rear_dist
        base = self.baseline[key]
        if dist is not None and base is not None:
            return (dist - base) > self.gap_delta
        return False

    def _norm_angle(self, a):
        while a > math.pi:  a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    def _publish_wheels(self, left: float, right: float):
        if self.motor_pub is not None:
            mc = MotorCmd()
            mc.left_lin = float(left)
            mc.right_lin = float(right)
            self.motor_pub.publish(mc)

        tw = Twist()
        tw.linear.x = float((left + right) / 2.0)
        tw.angular.z = float((right - left) / self.track_width)
        self.cmd_pub.publish(tw)

    def _stop(self):
        self._publish_wheels(0.0, 0.0)

    def _compute_steer(self, target_heading, cruise_speed, dt):
        yaw_error = self._norm_angle(target_heading - self.imu_yaw)
        if abs(yaw_error) < self.yaw_deadband:
            correction = 0.0
        else:
            raw_corr = self.pid_heading.compute(yaw_error, dt)
            correction = float(np.clip(raw_corr, -self.max_corr, self.max_corr))

        return cruise_speed - correction, cruise_speed + correction

    def _loop(self):
        if not self.have_imu or self.front_dist is None or self.rear_dist is None:
            self._stop()
            return

        now = self.get_clock().now()
        dt = (now - self.last_loop_time).nanoseconds * 1e-9
        self.last_loop_time = now

        # ---- 1. CALIBRATE ----
        if self.stage == Stage.CALIBRATE:
            self._stop()
            if len(self.calib_samples) >= 15:
                self.baseline['front'] = self.front_dist
                self.baseline['rear']  = self.rear_dist
                avg_yaw = float(np.mean(self.calib_samples))
                self.initial_yaw = round(avg_yaw / (math.pi / 2.0)) * (math.pi / 2.0)
                self.lane_heading = self.initial_yaw
                self.target_yaw = self.lane_heading
                self.pid_heading.reset()

                self.get_logger().info(f"✅ Baselines set: Front={self.baseline['front']:.3f}m, Rear={self.baseline['rear']:.3f}m")
                self.get_logger().info(f"🏁 Starting Lane 1 at heading: {math.degrees(self.lane_heading):.2f}°")
                self.stage = Stage.SWEEP_LANE
            return

        # ---- 2. SWEEP_LANE ----
        if self.stage == Stage.SWEEP_LANE:
            if self._is_tripped('front'):
                self.edge_trip_streak += 1
            else:
                self.edge_trip_streak = 0

            if self.edge_trip_streak >= self.debounce_req:
                self.edge_trip_streak = 0
                self.completed_lanes += 1

                if self.completed_lanes >= self.max_passes:
                    self._stop()
                    self.stage = Stage.DONE
                    self.get_logger().info('🎉 Max passes reached! Coverage finished.')
                    status_msg = String()
                    status_msg.data = "done"
                    self.status_pub.publish(status_msg)
                    return

                self._stop()
                self.stage = Stage.CHECK_SPACE
                return

            left, right = self._compute_steer(self.target_yaw, self.base_spd, dt)
            self._publish_wheels(left, right)
            return

        # ---- 3. CHECK_SPACE ----
        if self.stage == Stage.CHECK_SPACE:
            self._stop()
            # Calculate heading offset from initial start
            rel_yaw = abs(self._norm_angle(self.lane_heading - self.initial_yaw))
            is_facing_opposite = rel_yaw > math.radians(135.0)

            if is_facing_opposite:
                if self.latest_edge_dist is not None:
                    self.get_logger().info(f"🔍 Far edge space: {self.latest_edge_dist*100.0:.1f} cm available")
                    if self.latest_edge_dist < self.min_space_m:
                        self.get_logger().warn(
                            f"🛑 Boundary reached! Only {self.latest_edge_dist*100.0:.1f} cm left (< {self.min_space_m*100.0:.1f} cm). Stopping coverage."
                        )
                        self.stage = Stage.DONE
                        status_msg = String()
                        status_msg.data = "done"
                        self.status_pub.publish(status_msg)
                        return
            else:
                self.get_logger().info("ℹ️ Facing start direction: bypassing right MToF gate (facing cleaned panel).")

            self.get_logger().warn(f'🛑 Edge hit on Lane {self.completed_lanes}! Rolling forward to apex...')
            self.stage_timer_end = now + Duration(seconds=self.pre_turn_dur)
            self.stage = Stage.MOVE_TO_CORNER_APEX
            return

        # ---- 4. MOVE_TO_CORNER_APEX ----
        if self.stage == Stage.MOVE_TO_CORNER_APEX:
            if now >= self.stage_timer_end:
                self._stop()
                self.stage_timer_end = now + Duration(seconds=self.pause_dur)
                self.stage = Stage.CORNER_PAUSE_1
                return

            left, right = self._compute_steer(self.target_yaw, self.base_spd, dt)
            self._publish_wheels(left, right)
            return

        # ---- 5. CORNER_PAUSE_1 ----
        if self.stage == Stage.CORNER_PAUSE_1:
            self._stop()
            if now >= self.stage_timer_end:
                turn_dir = 1.0 if self.turn_left else -1.0
                self.target_yaw = self._norm_angle(self.lane_heading + (turn_dir * self.turn_angle))
                self.turn_start_yaw = self.imu_yaw
                self.has_moved = False
                self.trim_done = False
                self.get_logger().info(
                    f'🔄 TURN 1: Pivoting 90° toward adjacent stripe. Target: {math.degrees(self.target_yaw):.2f}°'
                )
                self.stage = Stage.TURN_1
            return

        # ---- 6. TURN_1 ----
        if self.stage == Stage.TURN_1:
            err = self._norm_angle(self.target_yaw - self.imu_yaw)
            abs_err = abs(err)

            if abs(self._norm_angle(self.imu_yaw - self.turn_start_yaw)) > math.radians(2.0):
                self.has_moved = True

            if self.has_moved and abs_err <= self.brake_lead_rad:
                self.brake_start = now
                self.stage = Stage.COUNTER_BRAKE_1
                return

            wz_dir = math.copysign(1.0, err)
            self.active_turn_dir = wz_dir
            turn_wheel_spd = (self.turn_spd * self.track_width) / 2.0
            self._publish_wheels(-wz_dir * turn_wheel_spd, wz_dir * turn_wheel_spd)
            return

        # ---- 7. COUNTER_BRAKE_1 ----
        if self.stage == Stage.COUNTER_BRAKE_1:
            dt_brake = (now - self.brake_start).nanoseconds * 1e-9
            if dt_brake < self.counter_sec:
                plug_dir = -self.active_turn_dir
                plug_wheel_spd = (self.counter_spd * self.track_width) / 2.0
                self._publish_wheels(-plug_dir * plug_wheel_spd, plug_dir * plug_wheel_spd)
            else:
                self._stop()
                self.settle_start = now
                self.stage = Stage.SETTLE_1
            return

        # ---- 8. SETTLE_1 ----
        if self.stage == Stage.SETTLE_1:
            self._stop()
            dt_settle = (now - self.settle_start).nanoseconds * 1e-9
            if dt_settle >= self.settle_time:
                err = self._norm_angle(self.target_yaw - self.imu_yaw)
                if abs(err) > self.trim_tol_rad and not self.trim_done:
                    self.trim_done = True
                    self.brake_start = now
                    self.stage = Stage.TRIM_1
                    return

                self.stage_timer_start = now
                self.stage_timer_end = now + Duration(seconds=self.stripe_shift_dur)
                self.pid_heading.reset()
                self.get_logger().info(f'➡️ Shifting forward along edge ({self.stripe_shift_dur:.1f}s)...')
                self.stage = Stage.ADVANCE_STRIPE
            return

        # ---- 9. TRIM_1 ----
        if self.stage == Stage.TRIM_1:
            dt_trim = (now - self.brake_start).nanoseconds * 1e-9
            err = self._norm_angle(self.target_yaw - self.imu_yaw)
            if dt_trim < 0.040:
                wz_dir = math.copysign(1.0, err)
                trim_wheel_spd = (self.turn_spd * self.track_width) / 2.0
                self._publish_wheels(-wz_dir * trim_wheel_spd, wz_dir * trim_wheel_spd)
            else:
                self._stop()
                self.settle_start = now
                self.stage = Stage.SETTLE_1
            return

        # ---- 10. ADVANCE_STRIPE ----
        if self.stage == Stage.ADVANCE_STRIPE:
            elapsed_shift = (now - self.stage_timer_start).nanoseconds * 1e-9
            if elapsed_shift > self.shift_blank_dur and self._is_tripped('front'):
                self.get_logger().warn('⚠️ End of panel hit during stripe shift - stopping shift')
                self._stop()
                self.stage = Stage.DONE
                status_msg = String()
                status_msg.data = "done"
                self.status_pub.publish(status_msg)
                return

            if now >= self.stage_timer_end:
                self._stop()
                self.stage_timer_end = now + Duration(seconds=self.pause_dur)
                self.stage = Stage.CORNER_PAUSE_2
                return

            left, right = self._compute_steer(self.target_yaw, self.shift_spd, dt)
            self._publish_wheels(left, right)
            return

        # ---- 11. CORNER_PAUSE_2 ----
        if self.stage == Stage.CORNER_PAUSE_2:
            self._stop()
            if now >= self.stage_timer_end:
                turn_dir = 1.0 if self.turn_left else -1.0
                self.target_yaw = self._norm_angle(self.target_yaw + (turn_dir * self.turn_angle))
                self.lane_heading = self.target_yaw
                self.turn_start_yaw = self.imu_yaw
                self.has_moved = False
                self.trim_done = False
                self.get_logger().info(
                    f'🔄 TURN 2: Pivoting 90° into reversed lane. Target: {math.degrees(self.target_yaw):.2f}°'
                )
                self.stage = Stage.TURN_2
            return

        # ---- 12. TURN_2 ----
        if self.stage == Stage.TURN_2:
            err = self._norm_angle(self.target_yaw - self.imu_yaw)
            abs_err = abs(err)

            if abs(self._norm_angle(self.imu_yaw - self.turn_start_yaw)) > math.radians(2.0):
                self.has_moved = True

            if self.has_moved and abs_err <= self.brake_lead_rad:
                self.brake_start = now
                self.stage = Stage.COUNTER_BRAKE_2
                return

            wz_dir = math.copysign(1.0, err)
            self.active_turn_dir = wz_dir
            turn_wheel_spd = (self.turn_spd * self.track_width) / 2.0
            self._publish_wheels(-wz_dir * turn_wheel_spd, wz_dir * turn_wheel_spd)
            return

        # ---- 13. COUNTER_BRAKE_2 ----
        if self.stage == Stage.COUNTER_BRAKE_2:
            dt_brake = (now - self.brake_start).nanoseconds * 1e-9
            if dt_brake < self.counter_sec:
                plug_dir = -self.active_turn_dir
                plug_wheel_spd = (self.counter_spd * self.track_width) / 2.0
                self._publish_wheels(-plug_dir * plug_wheel_spd, plug_dir * plug_wheel_spd)
            else:
                self._stop()
                self.settle_start = now
                self.stage = Stage.SETTLE_2
            return

        # ---- 14. SETTLE_2 ----
        if self.stage == Stage.SETTLE_2:
            self._stop()
            dt_settle = (now - self.settle_start).nanoseconds * 1e-9
            if dt_settle >= self.settle_time:
                err = self._norm_angle(self.target_yaw - self.imu_yaw)
                if abs(err) > self.trim_tol_rad and not self.trim_done:
                    self.trim_done = True
                    self.brake_start = now
                    self.stage = Stage.TRIM_2
                    return

                self.turn_left = not self.turn_left
                self.stage_timer_end = now + Duration(seconds=self.advance_dur)
                self.pid_heading.reset()
                self.stage = Stage.ADVANCE_ENTRY
            return

        # ---- 15. TRIM_2 ----
        if self.stage == Stage.TRIM_2:
            dt_trim = (now - self.brake_start).nanoseconds * 1e-9
            err = self._norm_angle(self.target_yaw - self.imu_yaw)
            if dt_trim < 0.040:
                wz_dir = math.copysign(1.0, err)
                trim_wheel_spd = (self.turn_spd * self.track_width) / 2.0
                self._publish_wheels(-wz_dir * trim_wheel_spd, wz_dir * trim_wheel_spd)
            else:
                self._stop()
                self.settle_start = now
                self.stage = Stage.SETTLE_2
            return

        # ---- 16. ADVANCE_ENTRY ----
        if self.stage == Stage.ADVANCE_ENTRY:
            if now >= self.stage_timer_end:
                self._stop()
                self.pid_heading.reset()
                self.get_logger().info(f'▶️ Lane {self.completed_lanes + 1} sweeping.')
                self.stage = Stage.SWEEP_LANE
                return

            left, right = self._compute_steer(self.target_yaw, self.base_spd, dt)
            self._publish_wheels(left, right)
            return

        # ---- 17. DONE ----
        if self.stage == Stage.DONE:
            self._stop()


def main(args=None):
    rclpy.init(args=args)
    node = SolarbotLawnmowerMtofNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok():
                node._stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()