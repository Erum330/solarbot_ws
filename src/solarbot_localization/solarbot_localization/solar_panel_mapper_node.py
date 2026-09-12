#!/usr/bin/env python3
"""
solar_panel_mapper_node.py

Solar Panel Perimeter Mapper:
- Motion Tracking: Integrates linear speed from /motorCmd or /cmd_vel with
  calibrated slip scale (slip_scale=0.768).
- TF Broadcaster: Broadcasts active 'odom -> base_footprint' so RViz Map display accepts frames.
- Edge Projection: Projects calibrated /edge_distance onto the map frame.
- Visualization: Publishes /panel_perimeter_map (OccupancyGrid) and
  /panel_edge_polygon (PolygonStamped).
- Automatic Export: Saves panel_edge_map.csv and panel_perimeter_map.png
  directly into src/solarbot_localization/maps.
"""

import math
import os
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data

from std_msgs.msg import Float32, String
from sensor_msgs.msg import Imu
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PolygonStamped, Point32, Twist, TransformStamped
from tf2_ros import TransformBroadcaster

try:
    from mros_interfaces.msg import MotorCmd
    HAVE_MOTOR_CMD = True
except ImportError:
    HAVE_MOTOR_CMD = False


class SolarPanelMapperNode(Node):
    def __init__(self):
        super().__init__('solar_panel_mapper_node')

        # ---------------- Default Directory Setup ----------------
        default_maps_dir = os.path.expanduser('~/solarbot_ws/src/solarbot_localization/maps')
        os.makedirs(default_maps_dir, exist_ok=True)

        # ---------------- Parameters ----------------
        self.declare_parameter('resolution',          0.01)       # 1 cm per cell
        self.declare_parameter('map_size_m',          3.0)        # 3.0m x 3.0m grid bounding area
        self.declare_parameter('slip_scale',          0.768)      # Calibrated scale factor
        self.declare_parameter('cmd_vel_topic',       '/cmd_vel')
        self.declare_parameter('motor_cmd_topic',     '/motorCmd')
        self.declare_parameter('use_motor_cmd',       True)
        self.declare_parameter('save_dir',            default_maps_dir)

        self.res = float(self.get_parameter('resolution').value)
        self.map_size = float(self.get_parameter('map_size_m').value)
        self.slip_scale = float(self.get_parameter('slip_scale').value)
        self.use_motor_cmd = bool(self.get_parameter('use_motor_cmd').value) and HAVE_MOTOR_CMD
        self.save_dir = str(self.get_parameter('save_dir').value)

        self.grid_cells = int(self.map_size / self.res)
        self.origin_offset = self.map_size / 2.0

        # Occupancy Grid: -1 = unknown, 0 = free glass, 100 = perimeter edge
        self.grid = np.full((self.grid_cells, self.grid_cells), -1, dtype=np.int8)

        # ---------------- State Variables ----------------
        self.xr = 0.0
        self.yr = 0.0
        self.imu_yaw = 0.0
        self.current_vx = 0.0
        self.latest_edge_dist = None

        self.last_time = self.get_clock().now()
        self.edge_points = []
        self.is_active = True

        # ---------------- QoS Profile (Matches RViz Map Display) ----------------
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # ---------------- Publishers & Broadcaster ----------------
        self.map_pub = self.create_publisher(OccupancyGrid, '/panel_perimeter_map', map_qos)
        self.poly_pub = self.create_publisher(PolygonStamped, '/panel_edge_polygon', 1)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- Subscriptions ----------------
        self.create_subscription(Float32, '/edge_distance', self._dist_cb, 10)
        self.create_subscription(Imu,     '/imu',           self._imu_cb,  qos_profile=qos_profile_sensor_data)
        self.create_subscription(String,  '/mapping_status',self._status_cb, 10)

        if self.use_motor_cmd:
            motor_topic = str(self.get_parameter('motor_cmd_topic').value)
            self.create_subscription(MotorCmd, motor_topic, self._motor_cb, 10)
            self.get_logger().info(f'Listening to {motor_topic} for velocity feedback.')
        else:
            cmd_topic = str(self.get_parameter('cmd_vel_topic').value)
            self.create_subscription(Twist, cmd_topic, self._cmd_cb, 10)
            self.get_logger().info(f'Listening to {cmd_topic} for velocity feedback.')

        # Timers: 50 Hz motion integration & TF broadcast, 1 Hz Map broadcast
        self.create_timer(0.02, self._integration_loop)
        self.create_timer(1.0,  self._publish_occupancy_grid)

        self.get_logger().info(f'🗺️ Mapper Initialized with slip_scale={self.slip_scale:.3f}')
        self.get_logger().info(f'📁 Maps will be saved to: {self.save_dir}')

    def _motor_cb(self, msg: MotorCmd):
        raw_vx = float((msg.left_lin + msg.right_lin) / 2.0)
        self.current_vx = raw_vx if raw_vx > 0.08 else 0.0

    def _cmd_cb(self, msg: Twist):
        raw_vx = float(msg.linear.x)
        self.current_vx = raw_vx if raw_vx > 0.08 else 0.0

    def _imu_cb(self, msg: Imu):
        q = msg.quaternion if hasattr(msg, 'quaternion') else msg.orientation
        if not (math.isfinite(q.x) and math.isfinite(q.y) and math.isfinite(q.z) and math.isfinite(q.w)):
            return
        self.imu_yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def _dist_cb(self, msg: Float32):
        d = float(msg.data)
        if 0.10 <= d <= 0.32:
            self.latest_edge_dist = d
            if self.current_vx > 0.08 and self.is_active:
                xe = self.xr + d * math.sin(self.imu_yaw)
                ye = self.yr - d * math.cos(self.imu_yaw)

                self.edge_points.append((xe, ye))
                self._mark_cell(xe, ye, 100)          # Perimeter boundary
                self._mark_cell(self.xr, self.yr, 0)      # Robot track

    def _integration_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if self.current_vx > 0.0 and self.is_active:
            ds = (self.current_vx * self.slip_scale) * dt
            self.xr += ds * math.cos(self.imu_yaw)
            self.yr += ds * math.sin(self.imu_yaw)

        # Broadcast live TF so RViz message filter accepts frames
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.xr
        t.transform.translation.y = self.yr
        t.transform.translation.z = 0.0
        
        half_yaw = self.imu_yaw * 0.5
        t.transform.rotation.z = math.sin(half_yaw)
        t.transform.rotation.w = math.cos(half_yaw)
        self.tf_broadcaster.sendTransform(t)

    def _mark_cell(self, x, y, value):
        col = int((x + self.origin_offset) / self.res)
        row = int((y + self.origin_offset) / self.res)

        if 0 <= row < self.grid_cells and 0 <= col < self.grid_cells:
            self.grid[row, col] = value

    def _publish_occupancy_grid(self):
        now_msg = self.get_clock().now().to_msg()

        grid_msg = OccupancyGrid()
        # Zeroing stamp allows RViz to render without waiting on TF synchronization
        grid_msg.header.stamp.sec = 0
        grid_msg.header.stamp.nanosec = 0
        grid_msg.header.frame_id = 'odom'

        meta = MapMetaData()
        meta.resolution = self.res
        meta.width = self.grid_cells
        meta.height = self.grid_cells
        meta.origin.position.x = -self.origin_offset
        meta.origin.position.y = -self.origin_offset
        meta.origin.position.z = 0.0
        meta.origin.orientation.w = 1.0

        grid_msg.info = meta
        grid_msg.data = self.grid.flatten().tolist()
        self.map_pub.publish(grid_msg)

        if self.edge_points:
            poly_msg = PolygonStamped()
            poly_msg.header.stamp = now_msg
            poly_msg.header.frame_id = 'odom'
            for pt in self.edge_points[::2]:
                p = Point32()
                p.x = float(pt[0])
                p.y = float(pt[1])
                p.z = 0.0
                poly_msg.polygon.points.append(p)
            self.poly_pub.publish(poly_msg)

    def _status_cb(self, msg: String):
        if msg.data == "done" and self.is_active:
            self.is_active = False
            self.get_logger().info('💾 Perimeter completed. Processing final map...')
            self._save_map()

    def _save_map(self):
        if len(self.edge_points) < 10:
            self.get_logger().error('Insufficient edge points collected.')
            return

        os.makedirs(self.save_dir, exist_ok=True)
        pts = np.array(self.edge_points, dtype=np.float32)

        min_x, min_y = np.min(pts, axis=0)
        max_x, max_y = np.max(pts, axis=0)
        measured_length = max_x - min_x
        measured_width = max_y - min_y

        self.get_logger().info(
            f"\n"
            f"╔═══════════════════════════════════════════════════════════════╗\n"
            f"║ 📐 MAPPED SOLAR PANEL BOUNDARIES                              ║\n"
            f"╠═══════════════════════════════════════════════════════════════╣\n"
            f"║  True Physical Target    :  1.400 m x 1.200 m                 ║\n"
            f"║  Mapped Length (X-span)  : {measured_length:6.3f} m                        ║\n"
            f"║  Mapped Width  (Y-span)  : {measured_width:6.3f} m                        ║\n"
            f"║  Total Edge Points Plotted: {len(self.edge_points):6d}                        ║\n"
            f"╚═══════════════════════════════════════════════════════════════╝"
        )

        csv_path = os.path.join(self.save_dir, 'panel_edge_map.csv')
        np.savetxt(csv_path, pts, delimiter=',', header='x_m,y_m', comments='')
        self.get_logger().info(f'✅ Point coordinates saved to: {csv_path}')

        img = np.zeros((self.grid_cells, self.grid_cells), dtype=np.uint8)
        for x, y in self.edge_points:
            c = int((x + self.origin_offset) / self.res)
            r = int((y + self.origin_offset) / self.res)
            if 0 <= r < self.grid_cells and 0 <= c < self.grid_cells:
                img[r, c] = 255

        kernel = np.ones((3, 3), np.uint8)
        img = cv2.dilate(img, kernel, iterations=1)
        img = cv2.flip(img, 0)

        img_path = os.path.join(self.save_dir, 'panel_perimeter_map.png')
        cv2.imwrite(img_path, img)
        self.get_logger().info(f'✅ Perimeter visual map saved to: {img_path}')


def main(args=None):
    rclpy.init(args=args)
    node = SolarPanelMapperNode()
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