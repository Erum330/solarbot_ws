#!/usr/bin/env python3
"""
cmd_vel_bridge.py

Converts geometry_msgs/Twist on '/cmd_vel' into mros_interfaces/MotorCmd
on '/motorCmd'.

Two output modes, controlled by the 'publish_raw_mps' param:
  - publish_raw_mps=True  (default): left_lin/right_lin carry the actual
    per-wheel ground speed in m/s, unscaled. Use this for bench/no-hardware
    testing so /motorCmd values are directly interpretable without doing
    cmd_scale math by hand.
  - publish_raw_mps=False: left_lin/right_lin are multiplied by cmd_scale
    into the raw firmware units converter_node.mCmd_callback() expects
    (clamped +/-400, dead zone 10). Use this once actually driving real
    hardware -- cmd_scale is still an UNCALIBRATED GUESS, see TODO below.

TODO(calibrate) before driving on hardware with publish_raw_mps=False:
  converter_node.mCmd_callback() clamps MotorCmd.left_lin/right_lin to
  +/-400 with a dead zone of 10 and passes the value straight through
  as a raw command to the firmware - it does NOT know about m/s. That
  means `cmd_scale` below (m/s -> raw units) is a GUESS. To calibrate:
  put the robot up on blocks, publish a known /cmd_vel (e.g. linear.x
  0.2 m/s), and tune cmd_scale until the firmware's actual commanded
  speed matches what you asked for.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from mros_interfaces.msg import MotorCmd


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/motorCmd')
        self.declare_parameter('wheel_separation_m', 0.156)
        # Raw MotorCmd units per (m/s) of wheel-ground speed. PLACEHOLDER.
        # Only used when publish_raw_mps is False.
        self.declare_parameter('cmd_scale', 350.0)
        # True = publish left_lin/right_lin directly in m/s (bench testing,
        # no hardware needed). False = apply cmd_scale for real firmware.
        self.declare_parameter('publish_raw_mps', True)
        # Safety: stop the motors if no fresh /cmd_vel arrives in time -
        # mros_converter/firmware have no watchdog of their own.
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.wheel_sep = float(self.get_parameter('wheel_separation_m').value)
        self.scale = float(self.get_parameter('cmd_scale').value)
        self.raw_mps = bool(self.get_parameter('publish_raw_mps').value)
        self.timeout = Duration(seconds=float(self.get_parameter('cmd_vel_timeout_sec').value))

        self.pub = self.create_publisher(MotorCmd, output_topic, 10)
        self.sub = self.create_subscription(Twist, input_topic, self.cb, 10)

        self.last_rx = self.get_clock().now()
        self.create_timer(0.1, self.watchdog_cb)

        mode = 'RAW m/s (bench mode, no hardware needed)' if self.raw_mps \
            else f'SCALED raw units (cmd_scale={self.scale}, UNCALIBRATED - tune on the bench)'
        self.get_logger().info(
            f'cmd_vel_bridge: {input_topic} (Twist) -> {output_topic} (MotorCmd), '
            f'wheel_separation={self.wheel_sep} m, mode: {mode}'
        )

    def cb(self, msg: Twist):
        self.last_rx = self.get_clock().now()
        self._publish(msg.linear.x, msg.angular.z)

    def watchdog_cb(self):
        if (self.get_clock().now() - self.last_rx) > self.timeout:
            # No recent command - hold the last-known-stop state by
            # continuously publishing zero rather than latching silently.
            self._publish(0.0, 0.0)

    def _publish(self, v, w):
        left_mps = v - (w * self.wheel_sep / 2.0)
        right_mps = v + (w * self.wheel_sep / 2.0)

        out = MotorCmd()
        if self.raw_mps:
            out.left_lin = left_mps
            out.right_lin = right_mps
        else:
            out.left_lin = left_mps * self.scale
            out.right_lin = right_mps * self.scale
        self.pub.publish(out)


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
