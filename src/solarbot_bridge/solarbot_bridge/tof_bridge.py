#!/usr/bin/env python3
"""
tof_bridge.py

Splits the single mros_interfaces/Tof message (produced by
panelbot2_ws's mros_converter on 'tof/data') into the four topics
solarbot_navigation / solarbot_safety actually subscribe to, matching
solarbot_description/urdf/sensors/sensors.xacro exactly:

  tof_front  (scalar, VL53L0X)  -> sensor_msgs/LaserScan  /front_mid_tof
  tof_left   (scalar, VL53L0X)  -> sensor_msgs/LaserScan  /left_mid_tof
  tof_back   (scalar, VL53L0X)  -> sensor_msgs/LaserScan  /rear_mid_tof
  mtof_right (8x8 grid, VL53L5CX) -> sensor_msgs/PointCloud2 /right_mid_tof/points

TODO(calibrate) before trusting this on hardware:
  - Confirm the unit mros_converter's Tof values are actually in.
    converter_node.py does `raw[64] - 60` etc with no further scaling;
    if that's raw millimeters, set tof_unit_is_mm=True (default) so
    this node converts to metres for ROS. If it's already metres,
    flip the parameter to False.
  - The grid FOV (grid_fov_deg) is a placeholder for a VL53L5CX-class
    sensor (~45 deg). Replace with the real datasheet value once
    confirmed, or your PointCloud2 zone spacing will be off.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from mros_interfaces.msg import Tof


class TofBridge(Node):
    def __init__(self):
        super().__init__('tof_bridge')

        self.declare_parameter('input_topic', 'tof/data')
        self.declare_parameter('tof_unit_is_mm', True)
        self.declare_parameter('range_min_m', 0.02)
        self.declare_parameter('range_max_m', 2.0)
        self.declare_parameter('grid_fov_deg', 45.0)
        self.declare_parameter('grid_size', 8)

        self.unit_mm = bool(self.get_parameter('tof_unit_is_mm').value)
        self.range_min = float(self.get_parameter('range_min_m').value)
        self.range_max = float(self.get_parameter('range_max_m').value)
        self.grid_fov = math.radians(float(self.get_parameter('grid_fov_deg').value))
        self.grid_size = int(self.get_parameter('grid_size').value)

        input_topic = self.get_parameter('input_topic').value

        self.pub_front = self.create_publisher(LaserScan, '/front_mid_tof', qos_profile_sensor_data)
        self.pub_left = self.create_publisher(LaserScan, '/left_mid_tof', qos_profile_sensor_data)
        self.pub_rear = self.create_publisher(LaserScan, '/rear_mid_tof', qos_profile_sensor_data)
        self.pub_right = self.create_publisher(PointCloud2, '/right_mid_tof/points', qos_profile_sensor_data)

        self.sub = self.create_subscription(Tof, input_topic, self.cb, qos_profile_sensor_data)

        # Precompute the 8x8 grid's per-zone angular offsets once.
        n = self.grid_size
        half_fov = self.grid_fov / 2.0
        if n > 1:
            self._angles = [-half_fov + i * (self.grid_fov / (n - 1)) for i in range(n)]
        else:
            self._angles = [0.0]

        self.get_logger().info(
            f'tof_bridge: {input_topic} (mros_interfaces/Tof) -> '
            f'/front_mid_tof, /left_mid_tof, /rear_mid_tof (LaserScan) + '
            f'/right_mid_tof/points (PointCloud2)'
        )

    def _to_m(self, v):
        return (v / 1000.0) if self.unit_mm else v

    def _single_point_scan(self, frame_id, raw_value, stamp):
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = frame_id
        scan.angle_min = 0.0
        scan.angle_max = 0.0
        scan.angle_increment = 0.0
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        r = self._to_m(raw_value)
        # A negative/zero-ish reading from a bad or offline sensor should
        # read as "no return" (inf), not as a bogus near-zero obstacle.
        if r < self.range_min:
            r = float('inf')
        scan.ranges = [r]
        return scan

    def cb(self, msg: Tof):
        stamp = self.get_clock().now().to_msg()

        self.pub_front.publish(self._single_point_scan('front_mid_tof_link', msg.tof_front, stamp))
        self.pub_left.publish(self._single_point_scan('left_mid_tof_link', msg.tof_left, stamp))
        self.pub_rear.publish(self._single_point_scan('rear_mid_tof_link', msg.tof_back, stamp))

        # ---- 8x8 grid -> PointCloud2 ----
        n = self.grid_size
        if len(msg.mtof_right) < n * n:
            self.get_logger().warn(
                f'mtof_right has {len(msg.mtof_right)} points, expected {n * n} - skipping this frame')
            return

        points = []
        for row in range(n):
            for col in range(n):
                d = self._to_m(msg.mtof_right[row * n + col])
                if d < self.range_min or d > self.range_max:
                    continue
                # Sensor optical convention: X forward (range), Y/Z from
                # the per-zone horizontal/vertical angle. Adjust the axis
                # mapping here once the sensor's real mounting/rotation
                # (see right_mid_tof_joint in sensors.xacro) is confirmed.
                ax = self._angles[col]
                ay = self._angles[row]
                x = d * math.cos(ax) * math.cos(ay)
                y = d * math.sin(ax)
                z = d * math.sin(ay)
                points.append((x, y, z))

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = point_cloud2.create_cloud(
            header=self._header('right_mid_tof_link', stamp),
            fields=fields,
            points=points,
        )
        self.pub_right.publish(cloud)

    def _header(self, frame_id, stamp):
        from std_msgs.msg import Header
        h = Header()
        h.stamp = stamp
        h.frame_id = frame_id
        return h


def main(args=None):
    rclpy.init(args=args)
    node = TofBridge()
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
