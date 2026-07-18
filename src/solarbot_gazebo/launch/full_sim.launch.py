import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # --------------------------
    # Configuration & Naming
    # --------------------------
    package_name = 'solarbot_gazebo'
    description_package = 'solarbot_description'

    # --------------------------
    # Robot State Publisher
    # --------------------------
    # Points to solarbot_gazebo since rsp.launch.py is housed there
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    # --------------------------
    # Gazebo World Setup
    # --------------------------
    world_file = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'warehouse_rooftop.sdf'
    )

    # Force Gazebo to find the shared libraries by explicitly overriding environment variables
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', world_file],
        output='screen',
        additional_env={
            'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib:/opt/ros/jazzy/opt/gz_sim_vendor/lib'
        }
    )

    # --------------------------
    # Spawn Solarbot
    # --------------------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'solarbot',
            '-x', '-3.10',
            '-y', '4.20',
            '-z', '0.05'
        ],
        output='screen'
    )

    # --------------------------
    # ROS-GZ Bridge Configuration
    # --------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )

    # --------------------------
    # Custom Twist Adapter Node 
    # --------------------------
    # Located inside solarbot_description/scripts/twistToStamped.py
    twist_to_stamped_script = os.path.join(
        get_package_share_directory(description_package),
        'scripts',
        'twistToStamped.py'
    )

    # Executed as a system process to bypass CMake libexec constraints smoothly
    twist_to_stamped = ExecuteProcess(
        cmd=['python3', twist_to_stamped_script],
        output='screen'
    )

    # --------------------------
    # ros2_control Hardware Loop Manager Spawners
    # --------------------------
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # --------------------------
    # Final Launch Tree Processing
    # --------------------------
    return LaunchDescription([
        rsp,
        gz_sim,
        spawn_entity,
        bridge,
        twist_to_stamped,
        diff_drive_spawner,
        joint_broad_spawner
    ])