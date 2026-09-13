#!/usr/bin/env python3
"""
solarbot_nav2_lawnmower_node.py

10-Point Boustrophedon Coverage Plan anchored to (0.0, 0.0):
- 5 linear passes with a tighter 24 cm stride to eliminate uncovered gaps.
- Sequential goal dispatch via Nav2 Simple Commander.
- Live Right MToF (/edge_distance) space gating before each shift.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Format: (x_m, y_m, target_heading_deg, is_shift_check)
WAYPOINTS_CONFIG = [
    # Pass 1: (0, 0) across +X
    (0.00, 0.00,   0.0, False),  # WP 1: Start
    (1.05, 0.00,  90.0, False),  # WP 2: End sweep 1 -> Pivot to +Y

    # Shift 1 -> Pass 2
    (1.05, 0.24, 180.0, True),   # WP 3: Shift up, pivot to -X
    (0.00, 0.24,  90.0, False),  # WP 4: End sweep 2 -> Pivot to +Y

    # Shift 2 -> Pass 3
    (0.00, 0.48,   0.0, True),   # WP 5: Shift up, pivot to +X
    (1.05, 0.48,  90.0, False),  # WP 6: End sweep 3 -> Pivot to +Y

    # Shift 3 -> Pass 4
    (1.05, 0.72, 180.0, True),   # WP 7: Shift up, pivot to -X
    (0.00, 0.72,  90.0, False),  # WP 8: End sweep 4 -> Pivot to +Y

    # Shift 4 -> Pass 5
    (0.00, 0.96,   0.0, True),   # WP 9: Shift up, pivot to +X
    (1.05, 0.96,   0.0, False),  # WP 10: End sweep 5 (Finish)
]

MIN_REQUIRED_SPACE_M = 0.20  # Minimum clearance required for next pass


class LawnmowerCommander(Node):
    def __init__(self):
        super().__init__('solarbot_nav2_lawnmower_commander')
        self.latest_edge_dist = None
        self.create_subscription(Float32, '/edge_distance', self._edge_cb, 10)

    def _edge_cb(self, msg: Float32):
        if math.isfinite(msg.data) and msg.data > 0.0:
            self.latest_edge_dist = msg.data


def create_pose(nav: BasicNavigator, x: float, y: float, heading_deg: float) -> PoseStamped:
    yaw_rad = math.radians(heading_deg)
    pose = PoseStamped()
    pose.header.frame_id = 'odom'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose


def main():
    rclpy.init()
    listener_node = LawnmowerCommander()
    nav = BasicNavigator()

    nav.get_logger().info("⏳ Waiting for NavigateToPose action server...")
    if not nav.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
        nav.get_logger().error("❌ NavigateToPose action server unavailable! Ensure Nav2 launch is active.")
        return

    nav.get_logger().info("🚀 Nav2 ready! Starting 10-point coverage plan from (0, 0)...")

    total_wps = len(WAYPOINTS_CONFIG)

    for i, (curr_x, curr_y, heading_deg, is_shift) in enumerate(WAYPOINTS_CONFIG):
        rclpy.spin_once(listener_node, timeout_sec=0.05)

        # Gate lane entries using side clearance
        if is_shift and listener_node.latest_edge_dist is not None:
            avail_cm = listener_node.latest_edge_dist * 100.0
            nav.get_logger().info(f"🔍 Checking side space before WP {i+1}: {avail_cm:.1f} cm available")
            if listener_node.latest_edge_dist < MIN_REQUIRED_SPACE_M:
                nav.get_logger().warn(
                    f"🛑 Boundary reached! Only {avail_cm:.1f} cm left (< {MIN_REQUIRED_SPACE_M*100.0:.1f} cm). Stopping."
                )
                break

        target_pose = create_pose(nav, curr_x, curr_y, heading_deg)

        nav.get_logger().info(
            f"📍 [WP {i+1}/{total_wps}] Target: ({curr_x:.3f}, {curr_y:.3f}) | Heading: {heading_deg:.1f}°"
        )
        nav.goToPose(target_pose)

        while not nav.isTaskComplete():
            rclpy.spin_once(listener_node, timeout_sec=0.05)

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            nav.get_logger().info(f"✅ WP {i+1} reached!")
        elif result == TaskResult.CANCELED:
            nav.get_logger().warn(f"⚠️ WP {i+1} canceled.")
            break
        elif result == TaskResult.FAILED:
            nav.get_logger().error(f"❌ WP {i+1} failed! Aborting sequence.")
            break

    nav.get_logger().info("🎉 10-point coverage plan finished!")
    listener_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()