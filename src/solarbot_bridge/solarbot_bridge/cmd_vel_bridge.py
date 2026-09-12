#!/usr/bin/env python3
"""
cmd_vel_bridge.py

Converts geometry_msgs/Twist on '/cmd_vel' into mros_interfaces/MotorCmd
on '/motorCmd' with 1:1 direct pass-through in m/s.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mros_interfaces.msg import MotorCmd


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/motorCmd')
        self.declare_parameter('wheel_separation_m', 0.156)
        self.declare_parameter('cmd_scale', 1.0)
        # Default True: passes exact m/s (0.1 -> 0.1)
        self.declare_parameter('publish_raw_mps', True)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.wheel_sep = float(self.get_parameter('wheel_separation_m').value)
        self.scale = float(self.get_parameter('cmd_scale').value)
        self.raw_mps = bool(self.get_parameter('publish_raw_mps').value)

        self.pub = self.create_publisher(MotorCmd, output_topic, 10)
        self.sub = self.create_subscription(Twist, input_topic, self.cb, 10)

        self.get_logger().info(
            f'cmd_vel_bridge: {input_topic} -> {output_topic} '
            f'(publish_raw_mps={self.raw_mps}, scale={self.scale})'
        )

    def cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive kinematic decomposition
        left_mps = v - (w * self.wheel_sep / 2.0)
        right_mps = v + (w * self.wheel_sep / 2.0)

        out = MotorCmd()
        if self.raw_mps:
            out.left_lin = float(left_mps)
            out.right_lin = float(right_mps)
        else:
            out.left_lin = float(left_mps * self.scale)
            out.right_lin = float(right_mps * self.scale)

        self.pub.publish(out)
        self.get_logger().info(
            f'IN: v={v:.3f} w={w:.3f} -> OUT: left={out.left_lin:.3f} right={out.right_lin:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
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