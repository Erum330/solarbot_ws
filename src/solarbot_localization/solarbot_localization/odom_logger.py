#!/usr/bin/env python3
"""
odom_logger.py

Logs /odom_cam vs /gazebo/odom (already origin-corrected by real_odo.py)
to CSV, with error columns computed inline (matching odom_validation_logger.py
in solarbot_localization) so the companion analyze_odom_log.py script can
detect stuck/frozen periods, step-jumps, and produce paper-style Table I/II
reports without needing a separate origin-correction pass.

Sample period changed from the original 5.0s to 1.0s: several stuck
periods observed during testing lasted only 4-15 seconds - a 5s sample
rate would only catch 1-3 samples per stuck period, not enough for
reliable detection. 1Hz matches solarbot_localization's proven cadence.
"""

import csv
import math
import os
from time import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')

        self.declare_parameter('cam_topic', '/odom_cam')
        self.declare_parameter('gz_topic', '/gazebo/odom')
        self.declare_parameter('log_period_sec', 1.0)
        self.declare_parameter('log_path', '')  # empty = auto

        cam_topic = str(self.get_parameter('cam_topic').value)
        gz_topic = str(self.get_parameter('gz_topic').value)
        period = float(self.get_parameter('log_period_sec').value)
        log_path_param = str(self.get_parameter('log_path').value)

        self.create_subscription(Odometry, cam_topic, self.cam_cb, 10)
        self.create_subscription(Odometry, gz_topic, self.gz_cb, 10)

        self.cam_pose = None
        self.gz_pose = None

        if log_path_param:
            self.log_path = log_path_param
        else:
            script_dir = os.path.dirname(os.path.realpath(__file__))
            data_dir = os.path.join(os.path.dirname(script_dir), "Data")
            os.makedirs(data_dir, exist_ok=True)
            self.log_path = os.path.join(data_dir, "log.csv")

        with open(self.log_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',
                'cam_x', 'cam_y', 'cam_theta_deg',
                'gz_x', 'gz_y', 'gz_theta_deg',
                'dx_error', 'dy_error', 'dxy_error', 'dtheta_error_deg',
            ])

        self.create_timer(period, self.timer_cb)
        self.get_logger().info(
            f'Logging {cam_topic} vs {gz_topic} to {self.log_path} every {period}s')

    def cam_cb(self, msg: Odometry):
        self.cam_pose = self.extract_pose(msg)

    def gz_cb(self, msg: Odometry):
        self.gz_pose = self.extract_pose(msg)

    def extract_pose(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        return (x, y, yaw)

    def timer_cb(self):
        if self.cam_pose is None or self.gz_pose is None:
            self.get_logger().info(
                f'Waiting for data... cam={self.cam_pose is not None} '
                f'gz={self.gz_pose is not None}', throttle_duration_sec=5.0)
            return

        cx, cy, cth = self.cam_pose
        gx, gy, gth = self.gz_pose

        dx = cx - gx
        dy = cy - gy
        dxy = math.hypot(dx, dy)
        dth = math.degrees(math.atan2(math.sin(cth - gth), math.cos(cth - gth)))

        with open(self.log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                time(), cx, cy, math.degrees(cth), gx, gy, math.degrees(gth),
                dx, dy, dxy, dth,
            ])

        self.get_logger().info(
            f'cam=({cx:.3f},{cy:.3f},{math.degrees(cth):.1f}deg) '
            f'gz=({gx:.3f},{gy:.3f},{math.degrees(gth):.1f}deg) '
            f'err_xy={dxy:.3f}m err_th={dth:.1f}deg')


def main():
    rclpy.init()
    node = OdomLogger()
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
