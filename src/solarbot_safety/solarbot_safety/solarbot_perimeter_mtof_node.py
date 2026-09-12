#!/usr/bin/env python3
"""
solarbot_perimeter_mtof_node.py

Perimeter Follower with 4-Side Calibrated Telemetry:
- FOLLOW_SIDE: Pure dual-wheel differential drive (SquareDriver architecture).
- MOVE_TO_CORNER_APEX: Drives forward at cruise speed before turning to bring
  pivot center and right MToF directly to the corner apex.
- 4-SIDE LOGGING: Telemetry prints edge distance for Side 1, Side 2, Side 3, and Side 4.
- MToF CALIBRATION: Uses ground-truth linear model (d = 0.0325 * col + 0.0940 m).
- TURN_CORNER: Fast 3.50 rad/s pivot with active counter-torque reverse plug.
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


# -----------------------------
# PID Controller
# -----------------------------
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
    FOLLOW_SIDE          = auto()
    MOVE_TO_CORNER_APEX  = auto()
    CORNER_PAUSE         = auto()
    TURN_CORNER          = auto()
    COUNTER_BRAKE        = auto()
    SETTLE               = auto()
    TRIM                 = auto()
    ADVANCE_STRAIGHT     = auto()
    DONE                 = auto()


class SolarbotPerimeterMtofNode(Node):
    def __init__(self):
        super().__init__('solarbot_perimeter_mtof_node')

        # ---------------- Movement Speeds ----------------
        self.declare_parameter('base_speed',             0.25)     # Cruise speed (m/s)
        self.declare_parameter('track_width',            0.20)     # Track wheelbase (m)

        # ---------------- Heading PID Parameters ----------------
        self.declare_parameter('pid_kp',                 1.20)     # Heading hold gain
        self.declare_parameter('pid_ki',                 0.00)
        self.declare_parameter('pid_kd',                 0.15)
        self.declare_parameter('max_correction',         0.06)     # Steering authority clamp (m/s)

        # ---------------- IMU Turn Parameters ----------------
        self.declare_parameter('turn_speed',             3.50)     # Rad/s
        self.declare_parameter('brake_lead_deg',         9.20)     # Lead angle (deg)
        self.declare_parameter('counter_brake_spd',      2.20)     # Counter-torque speed (rad/s)
        self.declare_parameter('counter_brake_sec',      0.085)    # 85 ms plug
        self.declare_parameter('trim_tolerance_deg',     0.80)     # Trim window
        self.declare_parameter('settle_time_sec',        0.35)     # Settle window

        # ---------------- MToF Calibrated Model Parameters ----------------
        self.declare_parameter('surface_threshold_m',    0.18)
        self.declare_parameter('mtof_slope_m_per_col',   0.0325)   # 3.25 cm / column
        self.declare_parameter('mtof_base_offset_m',     0.0940)   # 9.40 cm base offset
        self.declare_parameter('transpose_grid',         True)
        self.declare_parameter('flip_vertical',          True)

        # ---------------- Sequential Timings ----------------
        self.declare_parameter('pre_turn_forward_sec',   0.45)     # Roll forward to corner apex
        self.declare_parameter('pause_duration_sec',     0.35)     # Dwell at apex before pivot
        self.declare_parameter('advance_duration_sec',   0.80)     # Clear onto new side
        self.declare_parameter('num_sides',              4)
        self.declare_parameter('gap_delta_m',            0.05)     # 5 cm drop to trigger edge detection
        self.declare_parameter('corner_debounce_count',  2)
        self.declare_parameter('cmd_vel_topic',          '/cmd_vel')

        p = self.get_parameter
        self.base_spd        = float(p('base_speed').value)
        self.track_width     = float(p('track_width').value)

        kp = float(p('pid_kp').value)
        ki = float(p('pid_ki').value)
        kd = float(p('pid_kd').value)
        self.pid_heading     = PID(kp, ki, kd)
        self.max_corr        = float(p('max_correction').value)

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

        self.pre_turn_dur    = float(p('pre_turn_forward_sec').value)
        self.pause_dur       = float(p('pause_duration_sec').value)
        self.advance_dur     = float(p('advance_duration_sec').value)
        self.num_sides       = int(p('num_sides').value)
        self.gap_delta       = float(p('gap_delta_m').value)
        self.debounce_req    = int(p('corner_debounce_count').value)

        # State tracking
        self.stage = Stage.CALIBRATE
        self.completed_sides = 0
        self.initial_yaw = 0.0
        self.target_yaw = 0.0
        self.turn_start_yaw = 0.0
        self.last_loop_time = self.get_clock().now()
        self.brake_start = self.get_clock().now()
        self.settle_start = self.get_clock().now()
        self.stage_timer_end = self.get_clock().now()

        self.has_moved = False
        self.trim_done = False
        self.corner_trip_streak = 0

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
        self.status_pub = self.create_publisher(String, '/mapping_status', 10)
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
        self.get_logger().info('🚀 SolarBot Active (All 4 Sides Calibrated Edge Telemetry Enabled).')

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

        # Calibrated model: d = m * col + c
        total_edge_distance = (self.mtof_slope * avg_col) + self.mtof_offset

        self.latest_edge_dist = total_edge_distance
        self.latest_edge_tilt = tilt_deg

        msg_dist = Float32()
        msg_dist.data = float(total_edge_distance)
        self.edge_dist_pub.publish(msg_dist)

        msg_tilt = Float32()
        msg_tilt.data = float(tilt_deg)
        self.edge_tilt_pub.publish(msg_tilt)

    def _log_turn_telemetry(self):
        current_yaw_deg = math.degrees(self.imu_yaw)
        target_yaw_deg = math.degrees(self.target_yaw)
        actual_delta = math.degrees(self._norm_angle(self.imu_yaw - self.turn_start_yaw))
        imu_err_deg = math.degrees(self._norm_angle(self.target_yaw - self.imu_yaw))

        edge_dist_str = f"{self.latest_edge_dist * 100.0:.1f} cm" if self.latest_edge_dist is not None else "N/A"

        self.get_logger().info(
            f"\n"
            f"╔═══════════════════════════════════════════════════════════════╗\n"
            f"║ 🏁 PURE IMU TURN COMPLETED - Side {self.completed_sides} -> {self.completed_sides + 1}\n"
            f"╠═══════════════════════════════════════════════════════════════╣\n"
            f"║  Target Cardinal Heading : {target_yaw_deg:6.2f}°                     ║\n"
            f"║  Final Resting Yaw       : {current_yaw_deg:6.2f}°                     ║\n"
            f"║  Net Angle Turned        : {actual_delta:6.2f}° (Goal: 90.0°)          ║\n"
            f"║  True Residual Error     : {imu_err_deg:+6.2f}°                     ║\n"
            f"║  Calibrated Edge Distance: {edge_dist_str:>8}                     ║\n"
            f"╚═══════════════════════════════════════════════════════════════╝"
        )

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
                self.target_yaw = self.initial_yaw
                self.pid_heading.reset()

                # Measure and report Side 1 starting edge distance
                edge_dist_str = f"{self.latest_edge_dist * 100.0:.1f} cm" if self.latest_edge_dist is not None else "N/A"

                self.get_logger().info(f"✅ Baselines set: Front={self.baseline['front']:.3f}m, Rear={self.baseline['rear']:.3f}m")
                self.get_logger().info(
                    f"\n"
                    f"╔═══════════════════════════════════════════════════════════════╗\n"
                    f"║ 📍 SIDE 1 STARTING POSITION                                   ║\n"
                    f"╠═══════════════════════════════════════════════════════════════╣\n"
                    f"║  Target Cardinal Heading : {math.degrees(self.target_yaw):6.2f}°                     ║\n"
                    f"║  Initial Resting Yaw     : {math.degrees(self.imu_yaw):6.2f}°                     ║\n"
                    f"║  Calibrated Edge Distance: {edge_dist_str:>8}                     ║\n"
                    f"╚═══════════════════════════════════════════════════════════════╝"
                )
                self.stage = Stage.FOLLOW_SIDE
            return

        # ---- 2. FOLLOW_SIDE ----
        if self.stage == Stage.FOLLOW_SIDE:
            if self._is_tripped('front'):
                self.corner_trip_streak += 1
            else:
                self.corner_trip_streak = 0

            if self.corner_trip_streak >= self.debounce_req:
                self.corner_trip_streak = 0
                self.completed_sides += 1

                if self.completed_sides >= self.num_sides:
                    self._stop()
                    self.stage = Stage.DONE
                    
                    # Print full summary for Side 4 completion
                    edge_dist_str = f"{self.latest_edge_dist * 100.0:.1f} cm" if self.latest_edge_dist is not None else "N/A"
                    self.get_logger().info(
                        f"\n"
                        f"╔═══════════════════════════════════════════════════════════════╗\n"
                        f"║ 🏁 SIDE 4 COMPLETED - FULL PERIMETER CLOSED                   ║\n"
                        f"╠═══════════════════════════════════════════════════════════════╣\n"
                        f"║  Final Cardinal Heading  : {math.degrees(self.target_yaw):6.2f}°                     ║\n"
                        f"║  Final Resting Yaw       : {math.degrees(self.imu_yaw):6.2f}°                     ║\n"
                        f"║  Calibrated Edge Distance: {edge_dist_str:>8}                     ║\n"
                        f"╚═══════════════════════════════════════════════════════════════╝"
                    )
                    self.get_logger().info('🎉 FULL SINGLE-PANEL PERIMETER COMPLETED!')
                    status_msg = String()
                    status_msg.data = "done"
                    self.status_pub.publish(status_msg)
                    return

                self.get_logger().warn(f'🛑 Corner Detected on Side {self.completed_sides}! Rolling forward to corner apex...')
                # Drive forward to the corner apex before turning
                self.stage_timer_end = now + Duration(seconds=self.pre_turn_dur)
                self.stage = Stage.MOVE_TO_CORNER_APEX
                return

            # Pure cardinal heading lock
            yaw_error = self._norm_angle(self.target_yaw - self.imu_yaw)
            raw_correction = self.pid_heading.compute(yaw_error, dt)
            correction = float(np.clip(raw_correction, -self.max_corr, self.max_corr))

            left = self.base_spd - correction
            right = self.base_spd + correction
            self._publish_wheels(left, right)
            return

        # ---- 3. MOVE_TO_CORNER_APEX (Active forward roll: ~15 cm) ----
        if self.stage == Stage.MOVE_TO_CORNER_APEX:
            if now >= self.stage_timer_end:
                self._stop()
                self.stage_timer_end = now + Duration(seconds=self.pause_dur)
                self.stage = Stage.CORNER_PAUSE
                return

            yaw_error = self._norm_angle(self.target_yaw - self.imu_yaw)
            raw_correction = self.pid_heading.compute(yaw_error, dt)
            correction = float(np.clip(raw_correction, -self.max_corr, self.max_corr))

            left = self.base_spd - correction
            right = self.base_spd + correction
            self._publish_wheels(left, right)
            return

        # ---- 4. CORNER_PAUSE (Short dwell before 90-deg turn) ----
        if self.stage == Stage.CORNER_PAUSE:
            self._stop()
            if now >= self.stage_timer_end:
                self.target_yaw = self._norm_angle(self.initial_yaw + self.completed_sides * (math.pi / 2.0))
                self.turn_start_yaw = self.imu_yaw
                self.has_moved = False
                self.trim_done = False
                self.get_logger().info(
                    f'🔄 Starting 90° Turn. Current: {math.degrees(self.turn_start_yaw):.2f}° '
                    f'-> Target: {math.degrees(self.target_yaw):.2f}°'
                )
                self.stage = Stage.TURN_CORNER
            return

        # ---- 5. TURN_CORNER (Fast 3.50 rad/s Pivot) ----
        if self.stage == Stage.TURN_CORNER:
            err = self._norm_angle(self.target_yaw - self.imu_yaw)
            abs_err = abs(err)

            if abs(self._norm_angle(self.imu_yaw - self.turn_start_yaw)) > math.radians(2.0):
                self.has_moved = True

            # Trigger reverse plug at 9.20 deg lead
            if self.has_moved and abs_err <= self.brake_lead_rad:
                self.brake_start = now
                self.stage = Stage.COUNTER_BRAKE
                return

            wz_dir = math.copysign(1.0, err)
            turn_wheel_spd = (self.turn_spd * self.track_width) / 2.0
            self._publish_wheels(-wz_dir * turn_wheel_spd, wz_dir * turn_wheel_spd)
            return

        # ---- 6. COUNTER_BRAKE (85 ms reverse pulse) ----
        if self.stage == Stage.COUNTER_BRAKE:
            dt_brake = (now - self.brake_start).nanoseconds * 1e-9
            if dt_brake < self.counter_sec:
                wz_dir = -math.copysign(1.0, self.turn_angle)
                plug_wheel_spd = (self.counter_spd * self.track_width) / 2.0
                self._publish_wheels(-wz_dir * plug_wheel_spd, wz_dir * plug_wheel_spd)
            else:
                self._stop()
                self.settle_start = now
                self.stage = Stage.SETTLE
            return

        # ---- 7. SETTLE (Confirmation Window) ----
        if self.stage == Stage.SETTLE:
            self._stop()
            dt_settle = (now - self.settle_start).nanoseconds * 1e-9

            if dt_settle >= self.settle_time:
                err = self._norm_angle(self.target_yaw - self.imu_yaw)
                if abs(err) > self.trim_tol_rad and not self.trim_done:
                    self.trim_done = True
                    self.brake_start = now
                    self.stage = Stage.TRIM
                    return

                self._log_turn_telemetry()
                self.stage_timer_end = now + Duration(seconds=self.advance_dur)
                self.stage = Stage.ADVANCE_STRAIGHT
            return

        # ---- 8. TRIM (40 ms Fine Correction Pulse) ----
        if self.stage == Stage.TRIM:
            dt_trim = (now - self.brake_start).nanoseconds * 1e-9
            err = self._norm_angle(self.target_yaw - self.imu_yaw)

            if dt_trim < 0.040:
                wz_dir = math.copysign(1.0, err)
                trim_wheel_spd = (self.turn_spd * self.track_width) / 2.0
                self._publish_wheels(-wz_dir * trim_wheel_spd, wz_dir * trim_wheel_spd)
            else:
                self._stop()
                self.settle_start = now
                self.stage = Stage.SETTLE
            return

        # ---- 9. ADVANCE_STRAIGHT (Drive forward onto new side) ----
        if self.stage == Stage.ADVANCE_STRAIGHT:
            if now >= self.stage_timer_end:
                self._stop()
                self.pid_heading.reset()
                self.get_logger().info(f'▶️ Side {self.completed_sides + 1} cruising.')
                self.stage = Stage.FOLLOW_SIDE
                return

            yaw_error = self._norm_angle(self.target_yaw - self.imu_yaw)
            raw_correction = self.pid_heading.compute(yaw_error, dt)
            correction = float(np.clip(raw_correction, -self.max_corr, self.max_corr))

            left = self.base_spd - correction
            right = self.base_spd + correction
            self._publish_wheels(left, right)
            return

        # ---- 10. DONE ----
        if self.stage == Stage.DONE:
            self._stop()


def main(args=None):
    rclpy.init(args=args)
    node = SolarbotPerimeterMtofNode()
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