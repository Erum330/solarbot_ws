#!/usr/bin/env python3

import math
import statistics
from collections import deque
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class TestForwardNode(Node):
    def __init__(self):
        super().__init__('test_forward_node')
        
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Snappier 3-sample window for instant edge response
        self.fl_buf = deque(maxlen=3)
        self.fr_buf = deque(maxlen=3)
        self.rl_buf = deque(maxlen=3)
        self.rr_buf = deque(maxlen=3)
        self.have_odom = False
        
        # Exact outer frame edge threshold (11.6 mm)
        self.PANEL_FRAME_THRESHOLD = 0.0116
        self.consecutive_hits = 0
        
        self.create_subscription(LaserScan, '/front_left_tof',  self._fl_cb,  10)
        self.create_subscription(LaserScan, '/front_right_tof', self._fr_cb,  10)
        self.create_subscription(LaserScan, '/rear_left_tof',   self._rl_cb,  10)
        self.create_subscription(LaserScan, '/rear_right_tof',  self._rr_cb,  10)
        self.create_subscription(Odometry,  '/diff_drive_controller/odom', self._odom_cb, 10)
        
        self.timer = self.create_timer(0.05, self._loop) # 20 Hz
        self.get_logger().info('--- INSTANT OUTER EDGE DETECTION STARTED ---')

    def _scan_min(self, msg):
        vals = [r for r in msg.ranges if math.isfinite(r)]
        return min(vals) if vals else math.inf

    def _fl_cb(self, msg): 
        v = self._scan_min(msg)
        if v != math.inf: self.fl_buf.append(v)

    def _fr_cb(self, msg): 
        v = self._scan_min(msg)
        if v != math.inf: self.fr_buf.append(v)

    def _rl_cb(self, msg): 
        v = self._scan_min(msg)
        if v != math.inf: self.rl_buf.append(v)

    def _rr_cb(self, msg): 
        v = self._scan_min(msg)
        if v != math.inf: self.rr_buf.append(v)

    def _odom_cb(self, msg):
        self.have_odom = True

    def _med(self, buf):
        return statistics.median(buf) if len(buf) >= 2 else 0.0

    def _loop(self):
        cmd = Twist()
        
        if not self.have_odom or len(self.fl_buf) < 2 or len(self.fr_buf) < 2:
            self.pub.publish(cmd)
            return

        fl = self._med(self.fl_buf)
        fr = self._med(self.fr_buf)

        self.get_logger().info(
            f'FL: {fl:.4f}m | FR: {fr:.4f}m | Target: >= {self.PANEL_FRAME_THRESHOLD:.4f}m',
            throttle_duration_sec=0.2
        )

        # Triggers the instant both front sensors touch the outer white frame border (>= 0.0116m)
        if fl >= self.PANEL_FRAME_THRESHOLD and fr >= self.PANEL_FRAME_THRESHOLD:
            self.consecutive_hits += 1
        else:
            self.consecutive_hits = 0

        # Stop instantly after 2 hits on the outer frame
        if self.consecutive_hits >= 2:
            self.get_logger().info('🎯 OUTER PANEL FRAME DETECTED! Stopping right at the edge.')
            self.pub.publish(Twist()) # Stop
            self.timer.cancel()
            return

        # Drive forward steadily at 0.08 m/s for precise stopping
        cmd.linear.x = 0.08
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TestForwardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()