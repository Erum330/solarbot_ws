"""
gazebo.launch.py
────────────────────────────────────────────────────────────────────
Launches the SolarBot Gazebo simulation (flat-panel warehouse world):

  1. gz sim          — Gazebo Harmonic, warehouse_rooftop.sdf
  2. robot_state_publisher  — /robot_description + static TFs
  3. gz_ros2_control — ros2_control ↔ Gazebo joint interface
  4. ros_gz_bridge   — Gazebo topics → ROS 2
  5. Controller spawners (joint_state_broadcaster + diff_drive_controller)

Robot spawns at the docking station (west end), facing east (+x) toward panels.

Usage:
  ros2 launch solarbot_gazebo gazebo.launch.py

Args:
  gz_gui:=false     headless Gazebo
  rviz:=true        open RViz2 alongside
  world:=<path>     override world file
────────────────────────────────────────────────────────────────────
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_gazebo = FindPackageShare('solarbot_gazebo')
    pkg_desc   = FindPackageShare('solarbot_description')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    # ── Paths ──────────────────────────────────────────────────
    default_world = PathJoinSubstitution(
        [pkg_gazebo, 'worlds', 'warehouse_rooftop.sdf'])
    default_xacro = PathJoinSubstitution(
        [pkg_desc,   'urdf',   'solarbot.urdf.xacro'])
    default_ctrl  = PathJoinSubstitution(
        [pkg_gazebo, 'config', 'controllers.yaml'])
    default_rviz  = PathJoinSubstitution(
        [pkg_desc,   'rviz',   'display.rviz'])

    # ── Launch arguments ───────────────────────────────────────
    args = [
        DeclareLaunchArgument('world',
            default_value=default_world,
            description='Path to Gazebo world SDF'),
        DeclareLaunchArgument('gz_gui',
            default_value='true',
            description='Start Gazebo with GUI (false = headless)'),
        DeclareLaunchArgument('rviz',
            default_value='false',
            description='Open RViz2 alongside Gazebo'),
        # ── Robot spawn pose ──
        # Spawns inside docking station (x=-3.20, y=4.20),
        # 5 cm off the roof surface, facing east (+x) toward panels.
        DeclareLaunchArgument('spawn_x',   default_value='-3.20'),
        DeclareLaunchArgument('spawn_y',   default_value='4.20'),
        DeclareLaunchArgument('spawn_z',   default_value='0.05'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
    ]

    world     = LaunchConfiguration('world')
    show_rviz = LaunchConfiguration('rviz')
    spawn_x   = LaunchConfiguration('spawn_x')
    spawn_y   = LaunchConfiguration('spawn_y')
    spawn_z   = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    # ── Robot description ──────────────────────────────────────
    # use_real_hardware=false → mock_components in ros2_control block
    robot_description = ParameterValue(
        Command(['xacro ', default_xacro, ' use_real_hardware:=false']),
        value_type=str,
    )

    # ── 1. Gazebo ──────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── 2. Robot state publisher ───────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ── 3. Spawn robot ─────────────────────────────────────────
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name',  'solarbot',
            '-x',  spawn_x,
            '-y',  spawn_y,
            '-z',  spawn_z,
            '-Y',  spawn_yaw,
        ],
    )

    # ── 4. ROS–Gazebo topic bridge ─────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/front_tof@sensor_msgs/msg/Range[gz.msgs.LaserScan',
            '/front_left_tof@sensor_msgs/msg/Range[gz.msgs.LaserScan',
            '/front_right_tof@sensor_msgs/msg/Range[gz.msgs.LaserScan',
            '/rear_left_tof@sensor_msgs/msg/Range[gz.msgs.LaserScan',
            '/rear_right_tof@sensor_msgs/msg/Range[gz.msgs.LaserScan',
            '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
    )

    # ── 5. Controller spawners (delayed 4 s) ──────────────────
    spawners = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller', '--param-file', default_ctrl],
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ],
    )

    # ── 6. RViz (optional) ────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(show_rviz),
    )

    return LaunchDescription(args + [gz_sim, rsp, spawn, bridge, spawners, rviz])
