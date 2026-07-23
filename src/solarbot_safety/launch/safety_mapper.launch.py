from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='solarbot_safety',
            executable='dynamic_edge_following_node',
            name='dynamic_edge_following_node',
            output='screen',
            parameters=[{
                'forward_speed': 0.10,
                'backup_speed': -0.08,
                'turn_speed': 0.50,
                'turn_angle_deg': 90.0,
                'turn_tolerance_deg': 1.5,
                'cliff_threshold_min': 0.0105,
                'corner_backup_dist_m': 0.12,
                'align_backup_dist_m': 0.05,
                'settle_sec': 0.50,
                'straight_heading_kp': 4.0,
                'straight_max_wz': 0.35,
                'turn_left': True,
                'num_sides': 4,
            }],
        ),
    ])