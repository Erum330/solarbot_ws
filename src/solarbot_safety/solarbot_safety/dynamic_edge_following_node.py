#!/usr/bin/env python3

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu


class Stage(Enum):
    INIT_BACKUP     = auto()
    FOLLOW_SIDE     = auto()
    CORNER_BACKUP   = auto()
    TURN_CORNER     = auto()
    RECOVER_FORWARD = auto()  # Active mid-turn rear edge recovery
    ALIGN_BACKUP    = auto()
    SETTLE          = auto()
    DONE            = auto()


class DynamicEdgeFollowingNode(Node):
    def __init__(self):
        super().__init__('dynamic_edge_following_node')

        # ---------------- Configurable Parameters ----------------
        self.declare_parameter('forward_speed',         0.12)
        self.declare_parameter('backup_speed',         -0.10)
        self.declare_parameter('turn_speed',            0.85)
        self.declare_parameter('turn_angle_deg',        90.0)
        self.declare_parameter('turn_tolerance_deg',    1.5)
        
        # Safe distance threshold for 1.0cm panel step drop
        self.declare_parameter('finite_safe_max',        0.075)
        
        self.declare_parameter('init_backup_dist_m',    0.10)
        self.declare_parameter('corner_backup_dist_m',  0.12)
        self.declare_parameter('align_backup_dist_m',   0.06)
        self.declare_parameter('recover_fwd_dist_m',    0.06)  # Distance to creep forward on rear trip
        self.declare_parameter('settle_sec',            0.30)
        self.declare_parameter('straight_heading_kp',   2.5)
        self.declare_parameter('straight_max_wz',       0.35)
        self.declare_parameter('turn_left',             True)
        self.declare_parameter('num_sides',             4)

        # Topics
        self.declare_parameter('fl_tof_topic',   '/front_left_tof')
        self.declare_parameter('fr_tof_topic',   '/front_right_tof')
        self.declare_parameter('rl_tof_topic',   '/rear_left_tof')
        self.declare_parameter('rr_tof_topic',   '/rear_right_tof')
        self.declare_parameter('odom_topic',     '/diff_drive_controller/odom')
        self.declare_parameter('imu_topic',      '/imu')
        self.declare_parameter('cmd_vel_topic',  '/cmd_vel')

        p = self.get_parameter
        self.fwd_spd             = float(p('forward_speed').value)
        self.bkp_spd             = float(p('backup_speed').value)
        self.turn_spd            = float(p('turn_speed').value)
        self.turn_angle          = math.radians(float(p('turn_angle_deg').value))
        self.turn_tol            = math.radians(float(p('turn_tolerance_deg').value))
        self.safe_max            = float(p('finite_safe_max').value)
        self.init_backup_dist    = float(p('init_backup_dist_m').value)
        self.corner_backup_dist  = float(p('corner_backup_dist_m').value)
        self.align_backup_dist   = float(p('align_backup_dist_m').value)
        self.recover_fwd_dist    = float(p('recover_fwd_dist_m').value)
        self.settle_sec          = float(p('settle_sec').value)
        self.straight_heading_kp = float(p('straight_heading_kp').value)
        self.straight_max_wz     = float(p('straight_max_wz').value)
        self.turn_left           = bool(p('turn_left').value)
        self.num_sides           = int(p('num_sides').value)

        # ---------------- Sensor state ----------------
        self.lc = self.rc = self.rlc = self.rrc = None

        # ---------------- Pose state ----------------
        self.have_odom = False
        self.have_imu  = False
        self.x = self.y = 0.0
        self.odom_yaw  = 0.0
        self.imu_yaw   = 0.0

        # ---------------- FSM state ----------------
        self.stage           = Stage.INIT_BACKUP
        self.completed_sides = 0
        self.snap_x = self.snap_y = 0.0

        self.turn_target_yaw  = 0.0
        self.side_heading_yaw = 0.0

        self.settle_end         = self.get_clock().now()
        self.after_settle_stage = Stage.FOLLOW_SIDE
        self.init_snap_set      = False

        # ---------------- ROS I/O ----------------
        cmd_topic  = str(p('cmd_vel_topic').value)
        odom_topic = str(p('odom_topic').value)
        imu_topic  = str(p('imu_topic').value)

        self.cmd_pub          = self.create_publisher(Twist, cmd_topic, 10)
        self.panel_corner_pub = self.create_publisher(Point, '/panel_corner', 10)

        self.create_subscription(LaserScan, str(p('fl_tof_topic').value), self._lc_cb,  10)
        self.create_subscription(LaserScan, str(p('fr_tof_topic').value), self._rc_cb,  10)
        self.create_subscription(LaserScan, str(p('rl_tof_topic').value), self._rlc_cb, 10)
        self.create_subscription(LaserScan, str(p('rr_tof_topic').value), self._rrc_cb, 10)
        self.create_subscription(Odometry,  odom_topic,                    self._odom_cb, 10)
        self.create_subscription(Imu,       imu_topic,                     self._imu_cb,  10)

        self.create_timer(0.05, self._loop)
        self.get_logger().info('🚀 Hybrid Controller Active (Mid-Turn Rear Safety Enabled)')

    # ---------------- Sensor callbacks ----------------
    def _scan_min(self, msg):
        vals = [r for r in msg.ranges
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        return min(vals) if vals else math.inf

    def _lc_cb(self,  msg): self.lc  = self._scan_min(msg)
    def _rc_cb(self,  msg): self.rc  = self._scan_min(msg)
    def _rlc_cb(self, msg): self.rlc = self._scan_min(msg)
    def _rrc_cb(self, msg): self.rrc = self._scan_min(msg)

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny, cosy)
        self.have_odom = True

    def _imu_cb(self, msg):
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.imu_yaw = math.atan2(siny, cosy)
        self.have_imu = True

    # ---------------- Helpers ----------------
    def _heading(self):
        return self.imu_yaw if self.have_imu else self.odom_yaw

    def _pub(self, vx=0.0, wz=0.0):
        cmd = Twist()
        cmd.linear.x  = float(vx)
        cmd.angular.z = float(wz)
        self.cmd_pub.publish(cmd)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _safe(self, v):
        return math.isfinite(v) and v <= self.safe_max

    def _front_edge(self):
        return not (self._safe(self.lc) and self._safe(self.rc))

    def _rear_edge(self):
        return not (self._safe(self.rlc) and self._safe(self.rrc))

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

    def _clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def _publish_side_start(self):
        msg = Point()
        msg.x, msg.y, msg.z = float(self.x), float(self.y), 0.0
        self.panel_corner_pub.publish(msg)

    def _publish_side_end(self):
        msg = Point()
        msg.x, msg.y, msg.z = float(self.x), float(self.y), 1.0
        self.panel_corner_pub.publish(msg)

    def _enter_settle(self, next_stage):
        self.stage               = Stage.SETTLE
        self.after_settle_stage = next_stage
        self.settle_end         = self.get_clock().now() + Duration(seconds=self.settle_sec)

    def _enter_follow_side(self):
        self.side_heading_yaw = self._heading()
        self.stage = Stage.FOLLOW_SIDE
        self._snap_xy()
        self._publish_side_start()
        self.get_logger().info(f'▶️ SIDE {self.completed_sides + 1}')

    # ---------------- Main loop ----------------
    def _loop(self):
        if not self.have_odom:
            self._stop()
            return
        if any(v is None for v in [self.lc, self.rc, self.rlc, self.rrc]):
            self._stop()
            return

        # ---- INIT_BACKUP ----
        if self.stage == Stage.INIT_BACKUP:
            if not self.init_snap_set:
                self._snap_xy()
                self.init_snap_set = True
            if self._rear_edge() or self._dist_from_snap() >= self.init_backup_dist:
                self._stop()
                self._enter_settle(Stage.FOLLOW_SIDE)
                return
            self._pub(self.bkp_spd, 0.0)
            return

        # ---- FOLLOW_SIDE ----
        if self.stage == Stage.FOLLOW_SIDE:
            if self._front_edge():
                self._stop()
                self._publish_side_end()
                self._snap_xy()
                self.stage = Stage.CORNER_BACKUP
                return

            err = self._yaw_err(self.side_heading_yaw)
            wz  = self._clamp(
                self.straight_heading_kp * err,
                -self.straight_max_wz, self.straight_max_wz
            )
            self._pub(self.fwd_spd, wz)
            return

        # ---- CORNER_BACKUP ----
        if self.stage == Stage.CORNER_BACKUP:
            if self._rear_edge() or self._dist_from_snap() >= self.corner_backup_dist:
                self._stop()
                turn_sign = 1.0 if self.turn_left else -1.0
                self.turn_target_yaw = self._norm_angle(
                    self._heading() + turn_sign * self.turn_angle
                )
                self.get_logger().info(
                    f'Turn target: {math.degrees(self.turn_target_yaw):.1f}°'
                )
                self.stage = Stage.TURN_CORNER
                return
            self._pub(self.bkp_spd, 0.0)
            return

        # ---- TURN_CORNER ----
        if self.stage == Stage.TURN_CORNER:
            # SAFETY CHECK: If rear wheel swings over edge during pivot, interrupt and creep forward!
            if self._rear_edge():
                self._stop()
                self.get_logger().warn('⚠️ Rear sensor over edge during TURN! Creeping FORWARD to safety...')
                self._snap_xy()
                self.stage = Stage.RECOVER_FORWARD
                return

            err = self._yaw_err(self.turn_target_yaw)
            if abs(err) <= self.turn_tol:
                self._stop()
                self._snap_xy()
                self.stage = Stage.ALIGN_BACKUP
                return
            self._pub(0.0, math.copysign(self.turn_spd, err))
            return

        # ---- RECOVER_FORWARD ----
        if self.stage == Stage.RECOVER_FORWARD:
            # Creep forward slightly until rear wheels are safely back on the panel
            if not self._rear_edge() and self._dist_from_snap() >= self.recover_fwd_dist:
                self._stop()
                self.get_logger().info('↩️ Cleared edge! Resuming turn target.')
                # Re-derive turn target from current orientation
                turn_sign = 1.0 if self.turn_left else -1.0
                self.turn_target_yaw = self._norm_angle(
                    self._heading() + turn_sign * self.turn_angle
                )
                self.stage = Stage.TURN_CORNER
                return
            self._pub(self.fwd_spd, 0.0)
            return

        # ---- ALIGN_BACKUP ----
        if self.stage == Stage.ALIGN_BACKUP:
            if self._rear_edge() or self._dist_from_snap() >= self.align_backup_dist:
                self._stop()
                self.completed_sides += 1
                if self.completed_sides >= self.num_sides:
                    self.stage = Stage.DONE
                    self.get_logger().info('✅ PERIMETER COMPLETE')
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
        try: self._stop()
        except Exception: pass
        try: self.destroy_node()
        except Exception: pass


def main(args=None):
    rclpy.init(args=args)
    node = DynamicEdgeFollowingNode()
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