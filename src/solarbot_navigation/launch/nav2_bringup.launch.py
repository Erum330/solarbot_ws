import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('solarbot_navigation')
    nav2_bt_share = get_package_share_directory('nav2_bt_navigator')
    default_params = os.path.join(nav_share, 'config', 'nav2_params.yaml')

    default_bt_xml = os.path.join(
        nav2_bt_share, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'
    )
    default_through_bt_xml = os.path.join(
        nav2_bt_share, 'behavior_trees', 'navigate_through_poses_w_replanning_and_recovery.xml'
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='',
        description='Full path to the saved map.yaml from odom_mapper'
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the Nav2 parameters file'
    )

    map_yaml = LaunchConfiguration('map_yaml')
    params_file = LaunchConfiguration('params_file')

    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'planner_server',
        'smoother_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    return LaunchDescription([
        map_yaml_arg,
        params_arg,

        # Static transform publisher bridging map -> odom identity
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_bridge',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom',
            ],
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_yaml, 'use_sim_time': True}],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                params_file,
                {
                    'default_nav_to_pose_bt_xml': default_bt_xml,
                    'default_nav_through_poses_bt_xml': default_through_bt_xml,
                }
            ],
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', '/cmd_vel'),
            ],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),
    ])