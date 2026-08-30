#!/usr/bin/env python3
"""
test_straight_tof_stop.py

Direct hardware test node: Drives forward using /cmd_vel until the front ToF
sensor exceeds the hardcoded edge threshold, matching the BEST_EFFORT QoS profile.
"""

import math
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class TestStraightTofStopNode(Node):
    def __init__(self):
        super().__init__('test_straight_tof_stop_node')

        # ---------------- Configurable Parameters ----------------
        self.declare_parameter('forward_speed', 0.10)       # Linear velocity in m/s
        self.declare_parameter('edge_threshold_m', 0.180)   # Edge trigger distance in meters
        self.declare_parameter('filter_window', 3)          # Samples for moving average smoothing

        self.fwd_spd = float(self.get_parameter('forward_speed').value)
        self.edge_threshold = float(self.get_parameter('edge_threshold_m').value)
        self.window_size = int(self.get_parameter('filter_window').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Uses qos_profile_sensor_data (BEST_EFFORT) to match tof_bridge publisher
        self.create_subscription(
            LaserScan,
            '/front_mid_tof',
            self._front_tof_cb,
            qos_profile=qos_profile_sensor_data
        )

        self.buf = deque(maxlen=self.window_size)
        self.stopped = False

        self.create_timer(0.05, self._loop)
        self.get_logger().info(
            f'🚀 Direct ToF Stop Active | Speed: {self.fwd_spd:.2f} m/s | Threshold: {self.edge_threshold:.3f} m | QoS: BEST_EFFORT'
        )

    def _front_tof_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        if valid:
            self.buf.append(min(valid))

    def _loop(self):
        if self.stopped or not self.buf:
            return

        current_val = sum(self.buf) / len(self.buf)

        # Trigger stop if distance exceeds the edge threshold
        if current_val > self.edge_threshold:
            self.cmd_pub.publish(Twist())
            self.stopped = True
            self.get_logger().warn(
                f'🛑 EDGE DETECTED! Reading: {current_val:.4f} m (Threshold: {self.edge_threshold:.4f} m). Stopping immediately.'
            )
            return

        cmd = Twist()
        cmd.linear.x = self.fwd_spd
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TestStraightTofStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()