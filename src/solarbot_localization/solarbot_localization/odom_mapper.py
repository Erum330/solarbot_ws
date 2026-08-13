import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
import numpy as np
import os
import cv2
import yaml
import math

class OdomGridMapNode(Node):
    def __init__(self):
        super().__init__('odom_grid_map_node')

        self.instant_convert = False    # convert npy to pgm on run
        self.write_npy = True          # overwrite npy

        # Parameters
        self.resolution = 0.05  # meters per cell
        self.width = 250       # number of cells
        self.height = 250
        self.origin_x = -2.5  # meters
        self.origin_y = -2.5

        # Initialize grid
        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)

        # Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, '/odom_grid_map', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom_cam', self.odom_callback, 10)
        self.status_sub = self.create_subscription(String, '/mapping_status', self.status_callback, 10)

        self.get_logger().info("OdomGridMapNode started")
        if self.instant_convert:
            self.convert_pgm()

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract yaw from orientation quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Apply offset: 0.17 m to the right of base link
        offset = 0.17
        x_offset = x + offset * math.cos(yaw - math.pi/2)
        y_offset = y + offset * math.sin(yaw - math.pi/2)

        # Convert to grid coordinates
        gx = int((x_offset - self.origin_x) / self.resolution)
        gy = int((y_offset - self.origin_y) / self.resolution)

        if 0 <= gx < self.width and 0 <= gy < self.height:
            self.grid[gy, gx] = 0  # mark free space
        else:
            self.get_logger().warn(f"Pose ({x_offset:.2f}, {y_offset:.2f}) outside grid bounds")

        occ_grid = OccupancyGrid()
        occ_grid.header.stamp = self.get_clock().now().to_msg()
        occ_grid.header.frame_id = "odom"
        occ_grid.info.resolution = self.resolution
        occ_grid.info.width = self.width
        occ_grid.info.height = self.height
        occ_grid.info.origin.position.x = self.origin_x
        occ_grid.info.origin.position.y = self.origin_y
        occ_grid.info.origin.orientation.w = 1.0
        occ_grid.data = self.grid.flatten().tolist()

        self.map_pub.publish(occ_grid)

    def status_callback(self, msg: String):
        if msg.data.strip().lower() == "done":
            self.get_logger().info("Mapping complete. Processing rectangle fill...")

            if self.write_npy:
                self.save_npy(msg)
            self.convert_pgm()

            self.get_logger().info("Shutting down node")
            rclpy.shutdown()

    def save_npy(self, msg: String):
        if msg.data.strip().lower() == "done":
            self.get_logger().info("Mapping status is 'done'. Saving map and shutting down...")

            script_dir = os.path.dirname(os.path.realpath(__file__))
            parent_dir = os.path.dirname(script_dir)
            map_dir = os.path.join(parent_dir, "maps")
            os.makedirs(map_dir, exist_ok=True)
            map_path = os.path.join(map_dir, "odom_grid_map.npy")
            np.save(map_path, self.grid)
            self.get_logger().info(f"Map saved to {map_path}")
            self.get_logger().info("Saved pgm")

    def convert_pgm(self):
        self.get_logger().info("Converting saved .npy map to ROTATED RECT PGM + YAML...")

        script_dir = os.path.dirname(os.path.realpath(__file__))
        parent_dir = os.path.dirname(script_dir)
        map_dir = os.path.join(parent_dir, "maps")
        npy_path = os.path.join(map_dir, "odom_grid_map.npy")

        if not os.path.exists(npy_path):
            self.get_logger().error("No .npy map found to convert")
            return

        grid = np.load(npy_path)

        # --------------------------------------------------
        # STEP 1: Extract known points
        # --------------------------------------------------
        known_mask = (grid != -1)

        if not np.any(known_mask):
            self.get_logger().error("Map is entirely unknown")
            return

        ys, xs = np.where(known_mask)
        points = np.column_stack((xs, ys)).astype(np.float32)

        # --------------------------------------------------
        # STEP 2: Compute minimum area rectangle
        # --------------------------------------------------
        rect = cv2.minAreaRect(points)
        box = cv2.boxPoints(rect)   # 4 corners
        box = np.int32(box)

        # --------------------------------------------------
        # STEP 3: Create masks
        # --------------------------------------------------
        h, w = grid.shape
        rect_mask = np.zeros((h, w), dtype=np.uint8)

        # Fill rectangle interior
        cv2.fillPoly(rect_mask, [box], 255)

        # Border mask (1-pixel thick)
        border_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.polylines(border_mask, [box], isClosed=True, color=255, thickness=1)

        # --------------------------------------------------
        # STEP 4: Construct processed grid
        # --------------------------------------------------
        processed_grid = np.full_like(grid, -1)

        # Interior → FREE
        processed_grid[rect_mask == 255] = 0

        # Border → OCCUPIED (overwrite interior edges)
        processed_grid[border_mask == 255] = 100

        # --------------------------------------------------
        # STEP 5: Convert to PGM
        # --------------------------------------------------
        pgm_img = np.zeros_like(processed_grid, dtype=np.uint8)

        pgm_img[processed_grid == 0] = 254   # free
        pgm_img[processed_grid == 100] = 0   # occupied
        pgm_img[processed_grid == -1] = 205  # unknown

        pgm_path = os.path.join(map_dir, "map.pgm")
        yaml_path = os.path.join(map_dir, "map.yaml")

        cv2.imwrite(pgm_path, cv2.flip(pgm_img, 0))

        # --------------------------------------------------
        # STEP 6: YAML
        # --------------------------------------------------
        yaml_dict = {
            "image": "map.pgm",
            "resolution": float(self.resolution),
            "origin": [float(self.origin_x), float(self.origin_y), 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
            "mode": "trinary"
        }

        with open(yaml_path, "w") as f:
            yaml.dump(yaml_dict, f, default_flow_style=False)

        self.get_logger().info(f"Saved ROTATED RECT map to {pgm_path}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomGridMapNode()
    rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()