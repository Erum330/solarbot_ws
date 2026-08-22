#!/usr/bin/env python3
"""
cmd_vel_bridge.py

Converts geometry_msgs/Twist on '/cmd_vel' (published by whichever
solarbot_navigation/solarbot_safety FSM node is driving, or nav2's
velocity_smoother) into mros_interfaces/MotorCmd on '/motorCmd',
which panelbot2_ws's mros_converter already knows how to turn into
raw/cmd_vel for the ESP32 firmware.

Differential-drive kinematics use wheel_separation from
solarbot_gazebo/config/controllers.yaml (0.156 m) so this stays
consistent with the simulated robot's turning behaviour.

TODO(calibrate) before driving on hardware:
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
        self.declare_parameter('cmd_scale', 200.0)
        # Safety: stop the motors if no fresh /cmd_vel arrives in time -
        # mros_converter/firmware have no watchdog of their own.
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.wheel_sep = float(self.get_parameter('wheel_separation_m').value)
        self.scale = float(self.get_parameter('cmd_scale').value)
        self.timeout = Duration(seconds=float(self.get_parameter('cmd_vel_timeout_sec').value))

        self.pub = self.create_publisher(MotorCmd, output_topic, 10)
        self.sub = self.create_subscription(Twist, input_topic, self.cb, 10)

        self.last_rx = self.get_clock().now()
        self.create_timer(0.1, self.watchdog_cb)

        self.get_logger().info(
            f'cmd_vel_bridge: {input_topic} (Twist) -> {output_topic} (MotorCmd), '
            f'wheel_separation={self.wheel_sep} m, cmd_scale={self.scale} '
            f'(UNCALIBRATED - tune on the bench)'
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
