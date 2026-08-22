#!/usr/bin/env python3
"""
imu_bridge.py

Converts the ESP32's IMU reading — decoded by panelbot2_ws's
mros_converter into mros_interfaces/Imu on 'imu/data' — into the
standard sensor_msgs/Imu on '/imu' that solarbot_localization's
transform_detector (and solarbot_navigation/solarbot_safety) expect.

This is a pure field copy; no unit conversion is needed because
mros_converter already produced physical units (quaternion, rad/s
gyro... CONFIRM gyro units against your firmware, see note below).
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu as RosImu
from mros_interfaces.msg import Imu as MrosImu


class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')

        self.declare_parameter('input_topic', 'imu/data')
        self.declare_parameter('output_topic', '/imu')
        self.declare_parameter('frame_id', 'imu_link')

        # TODO(calibrate): mros_converter passes the firmware's raw gyro
        # values straight through with no scaling. If your firmware's
        # IMU driver reports gyro in deg/s rather than rad/s, set this
        # to True and this node will convert on the way through.
        self.declare_parameter('gyro_is_deg_per_sec', False)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.gyro_deg = bool(self.get_parameter('gyro_is_deg_per_sec').value)

        self.pub = self.create_publisher(RosImu, output_topic, 10)
        self.sub = self.create_subscription(MrosImu, input_topic, self.cb, 50)

        self.get_logger().info(
            f'imu_bridge: {input_topic} (mros_interfaces/Imu) -> '
            f'{output_topic} (sensor_msgs/Imu), frame_id={self.frame_id}'
        )

    def cb(self, msg: MrosImu):
        out = RosImu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id

        out.orientation.w = msg.quaternion.w
        out.orientation.x = msg.quaternion.x
        out.orientation.y = msg.quaternion.y
        out.orientation.z = msg.quaternion.z

        gx, gy, gz = msg.gyro.x, msg.gyro.y, msg.gyro.z
        if self.gyro_deg:
            import math
            gx, gy, gz = (v * math.pi / 180.0 for v in (gx, gy, gz))
        out.angular_velocity.x = gx
        out.angular_velocity.y = gy
        out.angular_velocity.z = gz

        out.linear_acceleration.x = msg.accel.x
        out.linear_acceleration.y = msg.accel.y
        out.linear_acceleration.z = msg.accel.z

        # Covariances unknown until characterized on the bench.
        # First element -1 means "do not use this field" per REP-145.
        out.orientation_covariance[0] = -1.0
        out.angular_velocity_covariance[0] = -1.0
        out.linear_acceleration_covariance[0] = -1.0

        # If the firmware quaternion isn't actually calibrated/valid yet
        # (msg.calibration.data low), orientation should be treated as
        # unreliable by downstream consumers.
        if msg.calibration.data < 1.0:
            out.orientation_covariance[0] = -1.0

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuBridge()
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
