#!/usr/bin/env python3
"""
cmd_vel_nav_bridge.py

Dedicated bridge for Nav2 operation.
Translates Nav2 /cmd_vel to /motorCmd while applying in-place pivot boosts
specifically tuned for trajectory tracking on solar glass.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mros_interfaces.msg import MotorCmd


class CmdVelNavBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_nav_bridge')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/motorCmd')
        self.declare_parameter('wheel_separation_m', 0.156)
        self.declare_parameter('min_pivot_mps', 0.23)  # Clears ConverterNode 0.20 m/s threshold

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.wheel_sep = float(self.get_parameter('wheel_separation_m').value)
        self.min_pivot_mps = float(self.get_parameter('min_pivot_mps').value)

        self.pub = self.create_publisher(MotorCmd, output_topic, 10)
        self.sub = self.create_subscription(Twist, input_topic, self.cb, 10)

        self.get_logger().info("Dedicated Nav2 Motor Bridge active (Pivot Boost: 0.23 m/s)")

    def cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive kinematic decomposition
        left_mps = v - (w * self.wheel_sep / 2.0)
        right_mps = v + (w * self.wheel_sep / 2.0)

        # In-place turn boost (only triggered during Nav2 heading alignment)
        if abs(v) < 0.05 and abs(w) > 0.05:
            sign_l = -1.0 if w > 0 else 1.0
            sign_r = 1.0 if w > 0 else -1.0
            left_mps = sign_l * max(abs(left_mps), self.min_pivot_mps)
            right_mps = sign_r * max(abs(right_mps), self.min_pivot_mps)

        out = MotorCmd()
        out.left_lin = float(left_mps)
        out.right_lin = float(right_mps)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()