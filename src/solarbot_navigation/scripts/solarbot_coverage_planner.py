#!/usr/bin/env python3
"""
solarbot_coverage_fsm_node.py

Direct-drive boustrophedon (lawnmower/Roomba-style) panel coverage -
NO Nav2. Confirmed on hardware that NavigateThroughPoses gets stuck
spot-rotating forever at pass-reversal waypoints (distance_remaining
stays 0.00m indefinitely) - its controller tries to satisfy heading at
intermediate waypoints, and a near-zero position delta combined with a
180deg heading reversal between passes has no smooth path to converge
on. A discrete stop/turn/shift/turn state machine sidesteps that
entirely by never asking a continuous-path controller to do something
it isn't designed for.

State sequence, adapted from edge_guard_node.py's proven
backup/turn/shift/turn/cooldown pattern:

  SWEEP -> (front edge found) -> BACKUP_TO_STANDOFF -> TURN_1
         -> SHIFT_LINE -> TURN_2 -> COOLDOWN -> SWEEP (reversed) -> ...

Trip detection reuses solarbot_perimeter_node.py's calibrated
baseline + gap_delta approach (filtered mean, compare against a
per-run baseline) rather than edge_guard_node's raw absolute-threshold
check - that's the version already hardened through extensive testing
on this specific sensor set, and it's what's actually running on this
robot's front/rear/left ToF sensors.

Sensors:
  front_mid_tof, rear_mid_tof, left_mid_tof - LaserScan point sensors
  right_mid_tof/points - VL53L5CX grid, PointCloud2. Only the near/
    steep rows are used (see _right_cb) - the shallow rows are tuned
    to reach across a panel gap to the NEXT panel, which is not what
    edge/cliff detection during a coverage sweep wants to see.
"""

import math
from collections import deque
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from sensor_msgs_py import point_cloud2


class State(Enum):
    CALIBRATE          = auto()
    SWEEP              = auto()
    BACKUP_TO_STANDOFF = auto()
    TURN_1             = auto()
    SHIFT_LINE         = auto()
    TURN_2             = auto()
    COOLDOWN           = auto()
    DONE               = auto()


class SolarbotCoverageFsmNode(Node):
    def __init__(self):
        super().__init__('solarbot_coverage_fsm_node')

        # ---------------- Configurable Parameters ----------------
        self.declare_parameter('forward_speed',        0.12)
        self.declare_parameter('backup_speed',        -0.10)
        self.declare_parameter('turn_speed',           0.40)
        self.declare_parameter('turn_angle_deg',       90.0)
        self.declare_parameter('turn_tolerance_deg',    1.5)

        self.declare_parameter('standoff_backup_dist_m', 0.10)
        self.declare_parameter('stripe_width_m',         0.28)   # lateral shift per pass - keep < right sensor's real gap-crossing FOV if you want overlap with edge-following coverage
        self.declare_parameter('shift_speed',            0.10)
        self.declare_parameter('shift_heading_kp',       2.2)
        self.declare_parameter('shift_max_wz',           0.35)

        self.declare_parameter('max_passes',            20)      # safety cap - stop after this many passes even if no boundary ends the sweep
        self.declare_parameter('cooldown_sec',           0.4)
        self.declare_parameter('straight_kp',            2.5)
        self.declare_parameter('straight_max_wz',        0.30)

        # Sensor filtering / calibration - same pattern as
        # solarbot_perimeter_node.py, deliberately kept identical so
        # tuning knowledge transfers directly between the two nodes.
        self.declare_parameter('filter_window',         15)
        self.declare_parameter('min_filter_samples',    10)
        self.declare_parameter('calibration_sec',        1.0)
        self.declare_parameter('gap_delta_m',            0.008)
        self.declare_parameter('right_near_row_count',   3)

        self.declare_parameter('odom_topic',    '/diff_drive_controller/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        p = self.get_parameter
        self.fwd_spd   = float(p('forward_speed').value)
        self.bkp_spd   = float(p('backup_speed').value)
        self.turn_spd  = float(p('turn_speed').value)
        self.turn_angle = math.radians(float(p('turn_angle_deg').value))
        self.turn_tol  = math.radians(float(p('turn_tolerance_deg').value))

        self.standoff       = float(p('standoff_backup_dist_m').value)
        self.stripe_w        = float(p('stripe_width_m').value)
        self.shift_spd        = float(p('shift_speed').value)
        self.shift_heading_kp = float(p('shift_heading_kp').value)
        self.shift_max_wz     = float(p('shift_max_wz').value)

        self.max_passes    = int(p('max_passes').value)
        self.cooldown_sec  = float(p('cooldown_sec').value)
        self.straight_kp     = float(p('straight_kp').value)
        self.straight_max_wz = float(p('straight_max_wz').value)

        self.filter_window      = int(p('filter_window').value)
        self.min_filter_samples = int(p('min_filter_samples').value)
        self.calibration_sec    = float(p('calibration_sec').value)
        self.gap_delta          = float(p('gap_delta_m').value)
        self.right_near_row_count = int(p('right_near_row_count').value)

        self.odom_topic    = str(p('odom_topic').value)
        self.cmd_vel_topic = str(p('cmd_vel_topic').value)

        # Buffers
        self.front_buf = deque(maxlen=self.filter_window)
        self.rear_buf  = deque(maxlen=self.filter_window)
        self.left_buf  = deque(maxlen=self.filter_window)
        self.right_buf = deque(maxlen=self.filter_window)

        self.baseline = {'front': None, 'rear': None, 'left': None, 'right': None}
        self.calib_start = None

        # State tracking
        self.have_odom = False
        self.have_imu  = False
        self.x = self.y = 0.0
        self.odom_yaw = 0.0
        self.imu_yaw  = 0.0

        self.state = State.CALIBRATE
        self.pass_count = 0
        self.turn_left = True   # alternates each pass, like edge_guard_node

        self.snap_x = self.snap_y = 0.0
        self.end_time = self.get_clock().now()

        self.sweep_entry_yaw  = 0.0
        self.turn1_target_yaw = 0.0
        self.turn2_target_yaw = 0.0
        self.shift_target_yaw = 0.0

        # ROS I/O
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(LaserScan, '/front_mid_tof', self._front_cb, 10)
        self.create_subscription(LaserScan, '/rear_mid_tof',  self._rear_cb, 10)
        self.create_subscription(LaserScan, '/left_mid_tof',  self._left_cb, 10)
        self.create_subscription(PointCloud2, '/right_mid_tof/points', self._right_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, 10)

        self.create_timer(0.05, self._loop)
        self.get_logger().info('🌾 SolarBot Coverage FSM (direct-drive, no Nav2) started')

    # ---------------- sensor callbacks ----------------
    def _scan_min(self, msg):
        vals = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        return min(vals) if vals else math.inf

    def _front_cb(self, msg):
        v = self._scan_min(msg)
        if math.isfinite(v):
            self.front_buf.append(v)

    def _rear_cb(self, msg):
        v = self._scan_min(msg)
        if math.isfinite(v):
            self.rear_buf.append(v)

    def _left_cb(self, msg):
        v = self._scan_min(msg)
        if math.isfinite(v):
            self.left_buf.append(v)

    def _right_cb(self, msg):
        # Only the near/steep rows - see module docstring. Mixing in the
        # shallow rows (tuned to reach across a panel gap to the NEXT
        # panel) would make this reading meaningless for "is there an
        # edge right here" during a coverage sweep.
        min_dist = math.inf
        width = msg.width if msg.width > 0 else 8
        near_row_limit = self.right_near_row_count * width
        for idx, (x, y, z) in enumerate(point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=False)):
            if idx >= near_row_limit:
                break
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
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

    # ---------------- helpers ----------------
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
            if r is not None and b is not None and (r - b) > self.gap_delta:
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
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _yaw_err(self, target):
        return self._norm_angle(target - self._heading())

    def _shift_progress(self):
        """Progress along the intended shift direction, projected into
        the local frame of shift_target_yaw - same technique as
        edge_guard_node.py's _shift_progress()."""
        dx = self.x - self.snap_x
        dy = self.y - self.snap_y
        return dx * math.cos(self.shift_target_yaw) + dy * math.sin(self.shift_target_yaw)

    def _enter(self, s: State):
        self.state = s
        self._snap_xy()
        self.get_logger().info(f'➡️  {s.name}')

    def _enter_timed(self, s: State, dur: float):
        self.state = s
        self.end_time = self.get_clock().now() + Duration(seconds=dur)
        self._snap_xy()
        self.get_logger().info(f'➡️  {s.name} ({dur:.1f}s)')

    # ---------------- main loop ----------------
    def _loop(self):
        if not (self.have_odom and self.have_imu):
            self._stop()
            return

        # ---- CALIBRATE ----
        if self.state == State.CALIBRATE:
            self._stop()
            if self.calib_start is None:
                self.calib_start = self.get_clock().now()
                self.get_logger().info('🔧 Calibrating baselines...')
                return
            if not self._filters_ready():
                return
            elapsed = (self.get_clock().now() - self.calib_start).nanoseconds * 1e-9
            if elapsed >= self.calibration_sec:
                readings = self._filtered_readings()
                for k in self.baseline:
                    self.baseline[k] = readings[k]
                self.get_logger().info(f'✅ Baselines set: {self.baseline}')
                self.sweep_entry_yaw = self._heading()
                self._enter(State.SWEEP)
            return

        if not self._filters_ready():
            self._stop()
            return

        # ---- SWEEP ----
        if self.state == State.SWEEP:
            front_tripped = self._get_tripped_sensor(('front',))
            if front_tripped:
                self._stop()
                self.get_logger().info(f'⚠️ Front edge found - pass {self.pass_count + 1} complete, recovering...')

                if self.turn_left:
                    self.turn1_target_yaw = self._norm_angle(self.sweep_entry_yaw + self.turn_angle)
                else:
                    self.turn1_target_yaw = self._norm_angle(self.sweep_entry_yaw - self.turn_angle)

                self.shift_target_yaw = self.turn1_target_yaw
                self.turn2_target_yaw = self._norm_angle(self.sweep_entry_yaw + math.pi)

                self._enter(State.BACKUP_TO_STANDOFF)
                return

            if self.pass_count >= self.max_passes:
                self._stop()
                self.state = State.DONE
                self.get_logger().info(f'✅ COVERAGE DONE (hit max_passes={self.max_passes} safety cap)')
                return

            err = self._yaw_err(self.sweep_entry_yaw)
            wz = max(-self.straight_max_wz, min(self.straight_max_wz, self.straight_kp * err))
            self._pub(self.fwd_spd, wz)
            return

        # ---- BACKUP_TO_STANDOFF ----
        if self.state == State.BACKUP_TO_STANDOFF:
            rear_tripped = self._get_tripped_sensor(('rear',))
            if rear_tripped:
                self.get_logger().warn('⚠️ Rear edge during backup - proceeding to turn anyway')
                self._stop()
                self._enter(State.TURN_1)
                return

            if self._dist_from_snap() >= self.standoff:
                self._stop()
                self._enter(State.TURN_1)
                return

            self._pub(self.bkp_spd, 0.0)
            return

        # ---- TURN_1 ----
        if self.state == State.TURN_1:
            err = self._yaw_err(self.turn1_target_yaw)
            if abs(err) <= self.turn_tol:
                self._stop()
                self._enter(State.SHIFT_LINE)
                return
            wz = math.copysign(self.turn_spd, err)
            self._pub(0.0, wz)
            return

        # ---- SHIFT_LINE ----
        if self.state == State.SHIFT_LINE:
            # After TURN_1, the front sensor now faces the shift
            # direction - a trip here means the panel edge is closer
            # than a full stripe_width away in that direction.
            front_tripped = self._get_tripped_sensor(('front',))
            if front_tripped:
                self.get_logger().warn('⚠️ Side edge during shift - cutting shift short')
                self._stop()
                self._enter(State.TURN_2)
                return

            if self._shift_progress() >= self.stripe_w:
                self._stop()
                self._enter(State.TURN_2)
                return

            err = self._yaw_err(self.shift_target_yaw)
            wz = max(-self.shift_max_wz, min(self.shift_max_wz, self.shift_heading_kp * err))
            self._pub(self.shift_spd, wz)
            return

        # ---- TURN_2 ----
        if self.state == State.TURN_2:
            err = self._yaw_err(self.turn2_target_yaw)
            if abs(err) <= self.turn_tol:
                self._stop()
                self.turn_left = not self.turn_left
                self.pass_count += 1
                self._enter_timed(State.COOLDOWN, self.cooldown_sec)
                return
            wz = math.copysign(self.turn_spd, err)
            self._pub(0.0, wz)
            return

        # ---- COOLDOWN ----
        if self.state == State.COOLDOWN:
            self._stop()
            if self.get_clock().now() >= self.end_time:
                self.sweep_entry_yaw = self.turn2_target_yaw
                self._enter(State.SWEEP)
            return

        # ---- DONE ----
        if self.state == State.DONE:
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
    node = SolarbotCoverageFsmNode()
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