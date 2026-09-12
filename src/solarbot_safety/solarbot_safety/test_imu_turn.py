#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class TestImuTurnNode(Node):
    def __init__(self):
        super().__init__('test_imu_turn_node')

        # ---------------- Calibrated Parameters ----------------
        self.declare_parameter('turn_angle_deg',        90.0)
        self.declare_parameter('turn_speed',            3.50)   # Cruise speed
        self.declare_parameter('brake_lead_deg',        9.20)   # Shifted from 11.40 to close the 2.18 deg gap
        self.declare_parameter('counter_brake_spd',     2.20)   # Reverse counter-torque
        self.declare_parameter('counter_brake_sec',     0.085)  # 85ms reverse plug
        self.declare_parameter('trim_tolerance_deg',    0.80)   # Maximum acceptable error before micro-trim
        self.declare_parameter('settle_time_sec',       0.35)   # Physical rest confirmation
        self.declare_parameter('cmd_vel_topic',         '/cmd_vel')

        self.turn_angle         = math.radians(self.get_parameter('turn_angle_deg').value)
        self.turn_spd           = float(self.get_parameter('turn_speed').value)
        self.brake_lead_rad     = math.radians(self.get_parameter('brake_lead_deg').value)
        self.counter_brake_spd  = float(self.get_parameter('counter_brake_spd').value)
        self.counter_brake_sec  = float(self.get_parameter('counter_brake_sec').value)
        self.trim_tol_rad       = math.radians(self.get_parameter('trim_tolerance_deg').value)
        self.settle_time        = float(self.get_parameter('settle_time_sec').value)

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, 10)

        self.have_imu = False
        self.imu_yaw = 0.0
        self.start_yaw = None
        self.target_yaw = None
        
        # State Flow: WARMUP -> RUNNING -> COUNTER_BRAKE -> SETTLING -> TRIM -> DONE
        self.state = 'WARMUP'
        self.warmup_samples = []
        self.brake_start = None
        self.settle_start = None
        self.has_moved = False
        self.trim_done = False

        self.create_timer(0.02, self._loop)
        self.get_logger().info('🧭 IMU Turn Node Active (Calibrated 9.20° Lead + Micro-Trim).')

    def _norm_angle(self, a):
        while a > math.pi:  a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    def _imu_cb(self, msg: Imu):
        q = msg.quaternion if hasattr(msg, 'quaternion') else msg.orientation
        if not (math.isfinite(q.x) and math.isfinite(q.y) and math.isfinite(q.z) and math.isfinite(q.w)):
            return

        yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        self.imu_yaw = yaw
        self.have_imu = True

        if self.state == 'WARMUP':
            self.warmup_samples.append(yaw)
            if len(self.warmup_samples) >= 15:
                self.start_yaw = float(np.mean(self.warmup_samples))
                self.target_yaw = self._norm_angle(self.start_yaw + self.turn_angle)
                self.state = 'RUNNING'
                self.get_logger().info(
                    f'📍 Calibrated Start: {math.degrees(self.start_yaw):.2f}° -> '
                    f'Target: {math.degrees(self.target_yaw):.2f}°'
                )

    def _loop(self):
        if not self.have_imu or self.state in ('WARMUP', 'DONE'):
            return

        now = self.get_clock().now()
        err = self._norm_angle(self.target_yaw - self.imu_yaw)
        abs_err = abs(err)

        # ---------------- 1. Active Counter-Torque Plug ----------------
        if self.state == 'COUNTER_BRAKE':
            dt_brake = (now - self.brake_start).nanoseconds * 1e-9
            if dt_brake < self.counter_brake_sec:
                cmd = Twist()
                cmd.angular.z = -math.copysign(self.counter_brake_spd, self.turn_angle)
                self.cmd_pub.publish(cmd)
            else:
                self.cmd_pub.publish(Twist())
                self.settle_start = now
                self.state = 'SETTLING'
            return

        # ---------------- 2. Settle & Adaptive Micro-Trim ----------------
        if self.state == 'SETTLING':
            self.cmd_pub.publish(Twist())
            dt_settle = (now - self.settle_start).nanoseconds * 1e-9

            if dt_settle >= self.settle_time:
                # If error is still outside tolerance and we haven't trimmed yet, nudge once
                if abs_err > self.trim_tol_rad and not self.trim_done:
                    self.trim_done = True
                    self.brake_start = now
                    self.state = 'TRIM'
                    return

                self.state = 'DONE'
                actual_delta = self._norm_angle(self.imu_yaw - self.start_yaw)
                self.get_logger().info(
                    f"\n"
                    f"╔═══════════════════════════════════════════════════════════════╗\n"
                    f"║ 🏁 TRUE RESTING HEADING CONFIRMED                             ║\n"
                    f"╠═══════════════════════════════════════════════════════════════╣\n"
                    f"║  Start Yaw               : {math.degrees(self.start_yaw):6.2f}°                     ║\n"
                    f"║  Final Resting Yaw       : {math.degrees(self.imu_yaw):6.2f}°                     ║\n"
                    f"║  Net Angle Turned        : {math.degrees(actual_delta):6.2f}° (Goal: {math.degrees(self.turn_angle):.1f}°)       ║\n"
                    f"║  True Residual Error     : {math.degrees(err):+6.2f}°                     ║\n"
                    f"╚═══════════════════════════════════════════════════════════════╝"
                )
            return

        # ---------------- 3. Single-Tick Micro-Trim Pulse ----------------
        if self.state == 'TRIM':
            dt_trim = (now - self.brake_start).nanoseconds * 1e-9
            if dt_trim < 0.040:  # 40ms micro-burst at 3.5 rad/s breaks static friction for ~1.5-2.0 deg
                cmd = Twist()
                cmd.angular.z = math.copysign(self.turn_spd, err)
                self.cmd_pub.publish(cmd)
            else:
                self.cmd_pub.publish(Twist())
                self.settle_start = now
                self.state = 'SETTLING'
            return

        # ---------------- 4. Primary Turn Drive (3.5 rad/s) ----------------
        if abs(self._norm_angle(self.imu_yaw - self.start_yaw)) > math.radians(2.0):
            self.has_moved = True

        if self.has_moved and abs_err <= self.brake_lead_rad:
            self.brake_start = now
            self.state = 'COUNTER_BRAKE'
            return

        cmd = Twist()
        cmd.angular.z = math.copysign(self.turn_spd, err)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TestImuTurnNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok():
                node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()