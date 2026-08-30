#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class TestImuTurnNode(Node):
    def __init__(self):
        super().__init__('test_imu_turn_node')

        self.declare_parameter('turn_angle_deg', 90.0)
        self.declare_parameter('turn_speed', 0.40)
        self.declare_parameter('turn_tol_deg', 1.5)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.turn_angle = math.radians(self.get_parameter('turn_angle_deg').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.turn_tol   = math.radians(self.get_parameter('turn_tol_deg').value)

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, 10)

        self.have_imu = False
        self.imu_yaw = 0.0
        self.start_yaw = None
        self.target_yaw = None
        self.done = False

        self.create_timer(0.05, self._loop)
        self.get_logger().info('🧭 IMU 90-Degree Turn Test Initialized. Waiting for IMU...')

    def _norm_angle(self, a):
        while a > math.pi:  a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    def _imu_cb(self, msg):
        q = msg.orientation
        if not (math.isfinite(q.x) and math.isfinite(q.y) and math.isfinite(q.z) and math.isfinite(q.w)):
            return
        self.imu_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.have_imu = True

    def _loop(self):
        if not self.have_imu or self.done:
            return

        if self.start_yaw is None:
            self.start_yaw = self.imu_yaw
            self.target_yaw = self._norm_angle(self.start_yaw + self.turn_angle)
            self.get_logger().info(f'Start Yaw: {math.degrees(self.start_yaw):.1f}° | Target: {math.degrees(self.target_yaw):.1f}°')
            return

        err = self._norm_angle(self.target_yaw - self.imu_yaw)

        if abs(err) <= self.turn_tol:
            self.cmd_pub.publish(Twist())
            self.done = True
            self.get_logger().info(f'✅ Turn Complete! Final Heading: {math.degrees(self.imu_yaw):.1f}° (Err: {math.degrees(err):.2f}°)')
            return

        wz = max(0.18, min(self.turn_speed, 1.2 * abs(err)))
        cmd = Twist()
        cmd.angular.z = math.copysign(wz, err)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TestImuTurnNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()