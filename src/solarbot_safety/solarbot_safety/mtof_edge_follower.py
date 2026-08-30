#!/usr/bin/env python3
"""
mtof_edge_follower.py

Closed-loop edge follower using the 8x8 Multi-ToF:
- Drives forward continuously.
- Dynamically steers (angular.z) using PD control to maintain a fixed lateral
  distance and parallel alignment relative to the right table edge.
- Halts if /front_mid_tof detects the end corner.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class MtofEdgeFollower(Node):
    def __init__(self):
        super().__init__('mtof_edge_follower')

        # ---------------- Control Parameters ----------------
        self.declare_parameter('forward_speed',        0.10)       # Forward crawl (m/s)
        self.declare_parameter('target_col',           3.5)        # Desired edge column (0 to 7)
        self.declare_parameter('surface_threshold_m',  0.08)       # Drop-off threshold
        self.declare_parameter('kp_lateral',           0.08)       # Steering gain for distance error
        self.declare_parameter('kp_heading',           0.015)      # Steering gain for tilt angle
        self.declare_parameter('max_wz',               0.25)       # Max angular velocity cap
        self.declare_parameter('front_stop_m',         0.16)       # Front corner stop threshold

        self.fwd_spd    = float(self.get_parameter('forward_speed').value)
        self.target_col = float(self.get_parameter('target_col').value)
        self.thresh     = float(self.get_parameter('surface_threshold_m').value)
        self.kp_lat     = float(self.get_parameter('kp_lateral').value)
        self.kp_head    = float(self.get_parameter('kp_heading').value)
        self.max_wz     = float(self.get_parameter('max_wz').value)
        self.front_stop = float(self.get_parameter('front_stop_m').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PointCloud2, '/right_mid_tof/points', self._cloud_cb, qos_profile=qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/front_mid_tof', self._front_cb, qos_profile=qos_profile_sensor_data)

        self.front_dist = 0.0
        self.stopped = False

        self.get_logger().info('🚀 Multi-ToF Edge Following Controller Active.')

    def _front_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        if valid:
            self.front_dist = min(valid)

    def _cloud_cb(self, msg: PointCloud2):
        if self.stopped:
            return

        # Front edge check (Corner arrival)
        if self.front_dist > self.front_stop:
            self.cmd_pub.publish(Twist())
            self.stopped = True
            self.get_logger().warn(f'🛑 Corner Reached! Front ToF: {self.front_dist:.3f}m. Halting.')
            return

        pts = [p for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)]
        if len(pts) < 64:
            return

        grid = np.zeros((8, 8))
        for r in range(8):
            for c in range(8):
                x, y, z = pts[r * 8 + c]
                grid[r, c] = math.sqrt(x*x + y*y + z*z) if math.isfinite(x) else 9.99

        # Detect edge columns
        edge_pts = []
        for r in range(8):
            edge_idx = np.where(grid[r, :] > self.thresh)[0]
            if len(edge_idx) > 0:
                edge_pts.append((r, edge_idx[0]))

        if len(edge_pts) < 3:
            # Fallback: Drive straight forward if edge is momentarily lost
            cmd = Twist()
            cmd.linear.x = self.fwd_spd
            self.cmd_pub.publish(cmd)
            return

        rows, cols = zip(*edge_pts)
        slope, _ = np.polyfit(rows, cols, 1)

        current_col = np.mean(cols)
        lat_err = current_col - self.target_col
        tilt_err_deg = math.degrees(math.atan(slope))

        # Steering correction:
        # If lat_err > 0 -> Robot is too far right (drifting off) -> Turn LEFT (+wz)
        # If tilt_err > 0 -> Robot is pointed away from table -> Turn LEFT (+wz)
        wz = (self.kp_lat * lat_err) + (self.kp_head * tilt_err_deg)
        wz = max(-self.max_wz, min(self.max_wz, wz))

        cmd = Twist()
        cmd.linear.x = self.fwd_spd
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MtofEdgeFollower()
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