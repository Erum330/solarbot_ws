#!/usr/bin/env python3
"""
solarbot_column1_perimeter_node.py

Tracked Chassis Multi-Panel Column Perimeter Follower:
- Step & Sense Gap Probing: Creeps forward 22 cm over panel frames/seams.
- Uses sub-pixel cv2.fitLine to prevent false corner detection and eliminate heading noise.
- Capped angular velocity prevents front bumper swing-out.
- Confirms true array corner, backs up 22 cm, and pivots 90°.
- Publishes /mapping_status: "done" upon mission completion.
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
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from sensor_msgs_py import point_cloud2


class Stage(Enum):
    CALIBRATE   = auto()
    POST_TURN   = auto()
    FOLLOW_SIDE = auto()
    PROBE_GAP   = auto()
    RECOVER_REV = auto()
    TURN_CORNER = auto()
    AVOID_EDGE  = auto()
    SETTLE      = auto()
    DONE        = auto()


class SolarbotColumn1PerimeterNode(Node):
    def __init__(self):
        super().__init__('solarbot_column1_perimeter_node')

        # ---------------- Movement Speeds ----------------
        self.declare_parameter('forward_speed',        0.08)       # Cruise speed (m/s)
        self.declare_parameter('probe_speed',          0.08)       # Gap step speed (m/s)
        self.declare_parameter('creep_speed',          0.04)       # Reverse recovery speed (m/s)
        self.declare_parameter('turn_spd',             0.22)       # Controlled turn velocity (rad/s)
        self.declare_parameter('turn_tolerance_deg',  2.0)

        # ---------------- Step & Sense Parameters ----------------
        self.declare_parameter('probe_distance_m',     0.22)       # Step 22 cm forward to clear frame borders
        self.declare_parameter('gap_delta_m',          0.05)       # 5 cm drop triggers gap check
        self.declare_parameter('surface_threshold_m',  0.18)       # > 18 cm = air drop-off

        # ---------------- Controllers ----------------
        self.declare_parameter('target_col',           3.5)        # Desired edge column
        self.declare_parameter('kp_imu_heading',       2.2)        # Cardinal heading lock gain
        self.declare_parameter('kp_mtof_lateral',      0.020)      # Lateral trim gain
        self.declare_parameter('max_wz',               0.08)       # Clamped max angular rate

        # ---------------- Timings & Structure ----------------
        self.declare_parameter('post_turn_sec',        1.00)
        self.declare_parameter('settle_sec',           0.40)
        self.declare_parameter('num_sides',            4)
        self.declare_parameter('cmd_vel_topic',        '/cmd_vel')

        p = self.get_parameter
        self.fwd_spd       = float(p('forward_speed').value)
        self.probe_spd     = float(p('probe_speed').value)
        self.creep_spd     = float(p('creep_speed').value)
        self.turn_spd      = float(p('turn_spd').value)
        self.turn_tol      = math.radians(float(p('turn_tolerance_deg').value))

        self.probe_dist    = float(p('probe_distance_m').value)
        self.gap_delta     = float(p('gap_delta_m').value)
        self.thresh        = float(p('surface_threshold_m').value)

        self.target_col    = float(p('target_col').value)
        self.kp_imu        = float(p('kp_imu_heading').value)
        self.kp_lat        = float(p('kp_mtof_lateral').value)
        self.max_wz        = float(p('max_wz').value)

        self.post_turn_sec = float(p('post_turn_sec').value)
        self.settle_sec    = float(p('settle_sec').value)
        self.num_sides     = int(p('num_sides').value)

        # State tracking
        self.stage = Stage.CALIBRATE
        self.completed_sides = 0
        self.initial_yaw = 0.0
        self.target_yaw = 0.0
        self.recovery_vx = 0.0
        self.settle_end = self.get_clock().now()
        self.post_turn_end = self.get_clock().now()
        self.probe_end = self.get_clock().now()
        self.recover_end = self.get_clock().now()
        self.panel_detect_streak = 0

        self.have_imu = False
        self.imu_yaw = 0.0

        # Sensor readings & Baselines
        self.front_dist = None
        self.rear_dist = None
        self.baseline = {'front': None, 'rear': None}
        self.calib_start = None
        self.latest_grid = None

        # Communications
        self.cmd_pub = self.create_publisher(Twist, str(p('cmd_vel_topic').value), 10)
        self.status_pub = self.create_publisher(String, '/mapping_status', 10)
        self.create_subscription(LaserScan,  '/front_mid_tof',        self._front_cb, qos_profile=qos_profile_sensor_data)
        self.create_subscription(LaserScan,  '/rear_mid_tof',         self._rear_cb,  qos_profile=qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/right_mid_tof/points', self._cloud_cb, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Imu,        '/imu',                  self._imu_cb,   qos_profile=qos_profile_sensor_data)

        self.create_timer(0.05, self._loop)
        self.get_logger().info('🚀 SolarBot Column 1 Follower Active (Hardware Mode)')

    def _front_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        if valid:
            self.front_dist = min(valid)

    def _rear_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        if valid:
            self.rear_dist = min(valid)

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        if not (math.isfinite(q.x) and math.isfinite(q.y) and math.isfinite(q.z) and math.isfinite(q.w)):
            return
        self.imu_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.have_imu = True

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
        pts = [p for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)]
        if len(pts) < 64:
            return

        raw_grid = np.zeros((8, 8), dtype=np.float32)
        for r in range(8):
            for c in range(8):
                x, y, z = pts[r * 8 + c]
                raw_grid[r, c] = math.sqrt(x*x + y*y + z*z) if (math.isfinite(x) and x > 0) else 9.99

        # Invert rows for hardware
        self.latest_grid = np.flipud(raw_grid.T)

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

    def _pub(self, vx=0.0, wz=0.0):
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(wz)
        self.cmd_pub.publish(cmd)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _enter_settle(self):
        self._stop()
        self.stage = Stage.SETTLE
        self.settle_end = self.get_clock().now() + Duration(seconds=self.settle_sec)

    def _loop(self):
        if not self.have_imu or self.front_dist is None or self.rear_dist is None:
            self._stop()
            return

        # ---- 1. CALIBRATE ----
        if self.stage == Stage.CALIBRATE:
            self._stop()
            if self.calib_start is None:
                self.calib_start = self.get_clock().now()
                self.get_logger().info('🔧 Calibrating ToF baselines on panel surface...')
                return

            elapsed = (self.get_clock().now() - self.calib_start).nanoseconds * 1e-9
            if elapsed >= 1.0:
                self.baseline['front'] = self.front_dist
                self.baseline['rear']  = self.rear_dist
                self.initial_yaw = round(self.imu_yaw / (math.pi / 2.0)) * (math.pi / 2.0)
                self.target_yaw = self.initial_yaw
                self.get_logger().info(f"✅ Baselines: Front={self.baseline['front']:.3f}m, Rear={self.baseline['rear']:.3f}m")
                self.get_logger().info(f"📍 Side 1 (Heading {math.degrees(self.target_yaw):.1f}°) Started.")
                self.stage = Stage.FOLLOW_SIDE
            return

        # ---- 2. POST_TURN ----
        if self.stage == Stage.POST_TURN:
            if self.get_clock().now() >= self.post_turn_end:
                self.get_logger().info(f'▶️ Corner cleared. Following Side {self.completed_sides + 1}...')
                self.stage = Stage.FOLLOW_SIDE
                return

            err = self._norm_angle(self.target_yaw - self.imu_yaw)
            wz = max(-self.max_wz, min(self.max_wz, self.kp_imu * err))
            self._pub(self.fwd_spd, wz)
            return

        # ---- 3. FOLLOW_SIDE ----
        if self.stage == Stage.FOLLOW_SIDE:
            if self._is_tripped('front'):
                probe_sec = self.probe_dist / self.probe_spd
                self.probe_end = self.get_clock().now() + Duration(seconds=probe_sec)
                self.panel_detect_streak = 0
                self.get_logger().info(f'🔍 Drop detected! Stepping forward {self.probe_dist*100:.0f}cm to sense next panel...')
                self.stage = Stage.PROBE_GAP
                return

            imu_err = self._norm_angle(self.target_yaw - self.imu_yaw)
            wz_base = self.kp_imu * imu_err

            lat_trim = 0.0
            if self.latest_grid is not None:
                edge_pts_2d = []
                for r in range(8):
                    col_cross = self._subpixel_edge(self.latest_grid[r, :])
                    if col_cross is not None:
                        edge_pts_2d.append([col_cross, float(r)])

                if len(edge_pts_2d) >= 4:
                    pts_array = np.array(edge_pts_2d, dtype=np.float32)
                    [vx, vy, x0, y0] = cv2.fitLine(pts_array, cv2.DIST_L2, 0, 0.01, 0.01)
                    lat_err = float(x0[0]) - self.target_col
                    lat_trim = -self.kp_lat * lat_err

            wz = wz_base + lat_trim
            wz = max(-self.max_wz, min(self.max_wz, wz))
            self._pub(self.fwd_spd, wz)
            return

        # ---- 4. PROBE_GAP ----
        if self.stage == Stage.PROBE_GAP:
            if not self._is_tripped('front'):
                self.panel_detect_streak += 1
                if self.panel_detect_streak >= 2:
                    self.get_logger().info('✅ Next panel detected! Seam bridged successfully. Continuing side...')
                    self.stage = Stage.FOLLOW_SIDE
                    return
            else:
                self.panel_detect_streak = 0

            if self.get_clock().now() >= self.probe_end:
                self._stop()
                self.completed_sides += 1
                self.get_logger().warn(f'🛑 No panel found after {self.probe_dist*100:.0f}cm step! True Array Corner on Side {self.completed_sides}.')

                if self.completed_sides >= self.num_sides:
                    self.stage = Stage.DONE
                    self.get_logger().info('🎉 FULL 5-PANEL COLUMN PERIMETER COMPLETED!')
                    status_msg = String()
                    status_msg.data = "done"
                    self.status_pub.publish(status_msg)
                    return

                rev_sec = self.probe_dist / self.creep_spd
                self.recover_end = self.get_clock().now() + Duration(seconds=rev_sec)
                self.stage = Stage.RECOVER_REV
                return

            err = self._norm_angle(self.target_yaw - self.imu_yaw)
            wz = max(-self.max_wz, min(self.max_wz, self.kp_imu * err))
            self._pub(self.probe_spd, wz)
            return

        # ---- 5. RECOVER_REV ----
        if self.stage == Stage.RECOVER_REV:
            if self.get_clock().now() >= self.recover_end:
                self._stop()
                self.target_yaw = self._norm_angle(self.initial_yaw + self.completed_sides * (math.pi / 2.0))
                self.get_logger().info(f'🔄 Safe on corner. Pivoting 90° to Target: {math.degrees(self.target_yaw):.1f}°')
                self.stage = Stage.TURN_CORNER
                return
            self._pub(-self.creep_spd, 0.0)
            return

        # ---- 6. TURN_CORNER ----
        if self.stage == Stage.TURN_CORNER:
            if self._is_tripped('front'):
                self._stop()
                self.recovery_vx = -self.creep_spd
                self.get_logger().warn('⚠️ Front ToF overhang mid-turn! Creeping BACKWARD...')
                self.stage = Stage.AVOID_EDGE
                return

            if self._is_tripped('rear'):
                self._stop()
                self.recovery_vx = self.creep_spd
                self.get_logger().warn('⚠️ Rear ToF overhang mid-turn! Creeping FORWARD...')
                self.stage = Stage.AVOID_EDGE
                return

            err = self._norm_angle(self.target_yaw - self.imu_yaw)
            if abs(err) <= self.turn_tol:
                self.get_logger().info(f'✅ Turn complete to {math.degrees(self.target_yaw):.1f}°. Settling...')
                self._enter_settle()
                return

            p_spd = max(0.14, min(self.turn_spd, 1.2 * abs(err)))
            self._pub(0.0, math.copysign(p_spd, err))
            return

        # ---- 7. AVOID_EDGE ----
        if self.stage == Stage.AVOID_EDGE:
            if not self._is_tripped('front') and not self._is_tripped('rear'):
                self._stop()
                self.get_logger().info('↩️ Overhang clear. Resuming turn...')
                self.stage = Stage.TURN_CORNER
                return
            self._pub(self.recovery_vx, 0.0)
            return

        # ---- 8. SETTLE ----
        if self.stage == Stage.SETTLE:
            self._stop()
            if self.get_clock().now() >= self.settle_end:
                self.post_turn_end = self.get_clock().now() + Duration(seconds=self.post_turn_sec)
                self.stage = Stage.POST_TURN
            return

        # ---- 9. DONE ----
        if self.stage == Stage.DONE:
            self._stop()


def main(args=None):
    rclpy.init(args=args)
    node = SolarbotColumn1PerimeterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()