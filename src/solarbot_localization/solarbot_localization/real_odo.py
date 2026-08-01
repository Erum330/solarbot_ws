#!/usr/bin/env python3
"""
real_odo.py (processing)

Republishes ground-truth odometry as /gazebo/odom, origin-corrected so
it shares the same "starts at 0,0,0" convention as /odom_cam.

"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class GazeboOdomRepublisher(Node):
    def __init__(self):
        super().__init__('gazebo_odom_republisher')

        self.declare_parameter('input_topic', '/ground_truth/odom')
        self.declare_parameter('output_topic', '/gazebo/odom')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self.create_subscription(Odometry, input_topic, self.odom_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)

        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None

        self.get_logger().info(
            f'Gazebo ground-truth odom republisher: {input_topic} -> {output_topic}')

    def odom_cb(self, msg: Odometry):
        pose = msg.pose.pose
        x, y, q = pose.position.x, pose.position.y, pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.initial_x is None:
            self.initial_x, self.initial_y, self.initial_yaw = x, y, yaw
            self.get_logger().info(
                f'Origin set at: x={x:.3f}, y={y:.3f}, theta={math.degrees(yaw):.2f}deg')

        dx, dy = x - self.initial_x, y - self.initial_y
        cos_init, sin_init = math.cos(-self.initial_yaw), math.sin(-self.initial_yaw)

        rel_x = cos_init * dx - sin_init * dy
        rel_y = sin_init * dx + cos_init * dy
        rel_yaw = math.atan2(math.sin(yaw - self.initial_yaw), math.cos(yaw - self.initial_yaw))

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link_ideal"

        odom.pose.pose.position.x = rel_x
        odom.pose.pose.position.y = rel_y
        odom.pose.pose.position.z = 0.0

        half_yaw = rel_yaw * 0.5
        odom.pose.pose.orientation.z = math.sin(half_yaw)
        odom.pose.pose.orientation.w = math.cos(half_yaw)

        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = GazeboOdomRepublisher()
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
