#!/usr/bin/env python3
"""
test_timed_backup.py

Executes a precise timed displacement using open-loop /cmd_vel commands.
Starts the timer on the first active execution loop.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist


class TestTimedBackupNode(Node):
    def __init__(self):
        super().__init__('test_timed_backup_node')

        # ---------------- Configurable Parameters ----------------
        self.declare_parameter('backup_speed', -0.08)       # Linear velocity in m/s
        self.declare_parameter('target_dist_m', 0.04)       # Displacement in meters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.spd = float(self.get_parameter('backup_speed').value)
        self.dist = float(self.get_parameter('target_dist_m').value)
        cmd_topic = str(self.get_parameter('cmd_vel_topic').value)

        # Calculate time budget: t = d / |v|
        self.duration_sec = abs(self.dist / self.spd) if abs(self.spd) > 1e-5 else 0.0

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        # Defer start_time initialization until the loop starts running
        self.start_time = None
        self.end_time = None
        self.done = False

        # 50 Hz control loop
        self.create_timer(0.02, self._loop)
        self.get_logger().info(
            f'⏱️ Timed Backup Configured: Speed={self.spd:.2f} m/s, Dist={self.dist*100:.1f} cm, '
            f'Budget={self.duration_sec:.2f}s. Waiting for loop start...'
        )

    def _stop_robot(self):
        stop_cmd = Twist()
        for _ in range(3):
            self.cmd_pub.publish(stop_cmd)

    def _loop(self):
        if self.done:
            return

        now = self.get_clock().now()

        # Start timer on the first actual loop tick
        if self.start_time is None:
            self.start_time = now
            self.end_time = self.start_time + Duration(seconds=self.duration_sec)
            self.get_logger().info(f'▶️ Motion Started: Executing for {self.duration_sec:.2f} seconds...')

        # Check if duration expired
        if now >= self.end_time:
            self._stop_robot()
            self.done = True
            self.get_logger().info('✅ Timed backup complete. Robot halted.')
            raise SystemExit  # Triggers clean exit from rclpy.spin

        cmd = Twist()
        cmd.linear.x = self.spd
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TestTimedBackupNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node._stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()