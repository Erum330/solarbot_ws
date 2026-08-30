#!/usr/bin/env python3
"""
motor_teleop.py

Direct bare-metal keyboard teleop configured with physical SI units:
- Linear speed: m/s (meters per second)
- Angular speed: rad/s (radians per second)
- Differential kinematics using track width (wheel separation base)
"""

import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from mros_interfaces.msg import MotorCmd

INSTRUCTIONS = (
    "Reading from keyboard and publishing direct MotorCmd to /motorCmd!\n"
    "---------------------------\n"
    "Units: Linear (m/s), Angular (rad/s)\n\n"
    "Moving around:\n"
    "   u    i    o\n"
    "   j    k    l\n"
    "   m    ,    .\n\n"
    "Controls:\n"
    "   i : Forward\n"
    "   , : Backward\n"
    "   j : Spin Left (in place)\n"
    "   l : Spin Right (in place)\n"
    "   u : Forward Left\n"
    "   o : Forward Right\n"
    "   m : Backward Left\n"
    "   . : Backward Right\n"
    "   k / SPACE : STOP (zero both motors)\n\n"
    "Speed Control:\n"
    "   q/z : increase/decrease max speeds by 10%\n"
    "   w/x : increase/decrease only linear speed by 10%\n"
    "   e/c : increase/decrease only angular speed by 10%\n\n"
    "CTRL-C to quit\n"
)

MOVE_BINDINGS = {
    'i': (1.0, 0.0),
    'o': (1.0, -1.0),
    'j': (0.0, 1.0),
    'l': (0.0, -1.0),
    'u': (1.0, 1.0),
    ',': (-1.0, 0.0),
    '.': (-1.0, 1.0),
    'm': (-1.0, -1.0),
}

SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


class MotorTeleop(Node):
    def __init__(self):
        super().__init__('motor_teleop')

        # Linear speed in m/s, Angular speed in rad/s, Track width in meters
        self.declare_parameter('linear_speed_mps', 0.20)
        self.declare_parameter('angular_speed_radps', 0.50)
        self.declare_parameter('track_width_m', 0.22)          # Distance between left and right wheels
        self.declare_parameter('speed_to_cmd_scale', 1.0)     # Scale factor if firmware expects raw counts (e.g. 1000.0)
        self.declare_parameter('max_cmd_val', 400.0)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.lin_speed = float(self.get_parameter('linear_speed_mps').value)
        self.ang_speed = float(self.get_parameter('angular_speed_radps').value)
        self.track_width = float(self.get_parameter('track_width_m').value)
        self.scale = float(self.get_parameter('speed_to_cmd_scale').value)
        self.max_cmd = float(self.get_parameter('max_cmd_val').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self.pub = self.create_publisher(MotorCmd, '/motorCmd', 10)
        
        # Internal velocities in m/s
        self.left_mps = 0.0
        self.right_mps = 0.0

        self.create_timer(1.0 / rate, self.publish_cb)
        self.get_logger().info('motor_teleop started - publishing in m/s to /motorCmd')

    def publish_cb(self):
        msg = MotorCmd()
        # Scale m/s to motor command units
        cmd_left = self.left_mps * self.scale
        cmd_right = self.right_mps * self.scale

        msg.left_lin = self.clamp(cmd_left)
        msg.right_lin = self.clamp(cmd_right)
        self.pub.publish(msg)

    def clamp(self, v):
        return max(-self.max_cmd, min(self.max_cmd, float(v)))

    def stop(self):
        self.left_mps = 0.0
        self.right_mps = 0.0
        self.publish_cb()

    def set_motion(self, x, th):
        # target linear (m/s) and angular (rad/s)
        v = x * self.lin_speed
        w = th * self.ang_speed

        # Standard differential drive kinematics:
        # v_left  = v - (w * L / 2)
        # v_right = v + (w * L / 2)
        half_width = self.track_width / 2.0
        self.left_mps = v - (w * half_width)
        self.right_mps = v + (w * half_width)


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = MotorTeleop()
    print(INSTRUCTIONS)
    print(f'currently:\tlinear: {node.lin_speed:.2f} m/s\tangular: {node.ang_speed:.2f} rad/s\ttrack_width: {node.track_width:.3f} m')

    settings = termios.tcgetattr(sys.stdin)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key(settings)

            if key in MOVE_BINDINGS.keys():
                x, th = MOVE_BINDINGS[key]
                node.set_motion(x, th)
                print(f'left={node.left_mps:.3f} m/s\tright={node.right_mps:.3f} m/s\t(lin={node.lin_speed:.2f} m/s, ang={node.ang_speed:.2f} rad/s)')

            elif key in SPEED_BINDINGS.keys():
                node.lin_speed = max(0.01, node.lin_speed * SPEED_BINDINGS[key][0])
                node.ang_speed = max(0.01, node.ang_speed * SPEED_BINDINGS[key][1])
                print(f'currently:\tlinear: {node.lin_speed:.2f} m/s\tangular: {node.ang_speed:.2f} rad/s')

            elif key in ('k', ' '):
                node.stop()
                print(f'STOPPED: left=0.0 m/s\tright=0.0 m/s')

            elif key == '\x03':  # Ctrl+C
                break

    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('\nStopped - both motors zeroed.')


if __name__ == '__main__':
    main()