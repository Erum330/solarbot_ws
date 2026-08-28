#!/usr/bin/env python3
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
    "   e/c : increase/decrease only turn speed by 10%\n\n"
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

        self.declare_parameter('speed',220.0)
        self.declare_parameter('turn', 30.0)
        self.declare_parameter('max_val', 400.0)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.speed = float(self.get_parameter('speed').value)
        self.turn = float(self.get_parameter('turn').value)
        self.max_val = float(self.get_parameter('max_val').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self.pub = self.create_publisher(MotorCmd, '/motorCmd', 10)
        self.left = 0.0
        self.right = 0.0

        self.create_timer(1.0 / rate, self.publish_cb)
        self.get_logger().info('motor_teleop started - publishing to /motorCmd')

    def publish_cb(self):
        msg = MotorCmd()
        msg.left_lin = self.clamp(self.left)
        msg.right_lin = self.clamp(self.right)
        self.pub.publish(msg)

    def clamp(self, v):
        return max(-self.max_val, min(self.max_val, float(v)))

    def stop(self):
        self.left = 0.0
        self.right = 0.0
        self.publish_cb()

    def set_motion(self, x, th):
        target_lin = x * self.speed
        target_ang = th * self.turn
        self.left = target_lin - target_ang
        self.right = target_lin + target_ang


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
    print(f'currently:\tspeed {node.speed:.1f}\tturn {node.turn:.1f}')

    settings = termios.tcgetattr(sys.stdin)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key(settings)

            if key in MOVE_BINDINGS.keys():
                x, th = MOVE_BINDINGS[key]
                node.set_motion(x, th)
                print(f'left={node.left:.1f}\tright={node.right:.1f}\t(speed={node.speed:.1f}, turn={node.turn:.1f})')

            elif key in SPEED_BINDINGS.keys():
                node.speed = node.clamp(node.speed * SPEED_BINDINGS[key][0])
                node.turn = node.clamp(node.turn * SPEED_BINDINGS[key][1])
                print(f'currently:\tspeed {node.speed:.1f}\tturn {node.turn:.1f}')

            elif key in ('k', ' '):
                node.stop()
                print(f'STOPPED: left={node.left:.1f}\tright={node.right:.1f}')

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