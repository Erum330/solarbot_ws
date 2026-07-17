"""
gazebo.launch.py  —  updated for 4-joint DiffDrive (no gz_ros2_control).

The Gazebo DiffDrive plugin drives all 4 track joints directly.
No controller_manager spawners needed — the plugin handles everything.

cmd_vel bridge direction: ROS→Gazebo (@ = bidirectional in ros_gz_bridge)
odometry / tf bridge:     Gazebo→ROS
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_gazebo = FindPackageShare('solarbot_gazebo')
    pkg_desc   = FindPackageShare('solarbot_description')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    default_world = PathJoinSubstitution([pkg_gazebo, 'worlds', 'warehouse_rooftop.sdf'])
    default_xacro = PathJoinSubstitution([pkg_desc,   'urdf',   'solarbot.urdf.xacro'])
    default_rviz  = PathJoinSubstitution([pkg_desc,   'rviz',   'display.rviz'])

    args = [
        DeclareLaunchArgument('world',     default_value=default_world),
        DeclareLaunchArgument('gz_gui',    default_value='true'),
        DeclareLaunchArgument('rviz',      default_value='false'),
        DeclareLaunchArgument('spawn_x',   default_value='-3.20'),
        DeclareLaunchArgument('spawn_y',   default_value='4.20'),
        DeclareLaunchArgument('spawn_z',   default_value='0.07'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
    ]

    world     = LaunchConfiguration('world')
    show_rviz = LaunchConfiguration('rviz')
    spawn_x   = LaunchConfiguration('spawn_x')
    spawn_y   = LaunchConfiguration('spawn_y')
    spawn_z   = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    robot_description = ParameterValue(
        Command(['xacro ', default_xacro, ' use_real_hardware:=false']),
        value_type=str,
    )

    # ── Gazebo ────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Robot state publisher ─────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ── Spawn robot ───────────────────────────────────────────
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name',  'solarbot',
            '-x', spawn_x, '-y', spawn_y,
            '-z', spawn_z, '-Y', spawn_yaw,
        ],
    )

    # ── ROS–Gazebo topic bridge ───────────────────────────────
    # cmd_vel: @ = bidirectional so teleop (ROS) can drive Gazebo DiffDrive
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # cmd_vel: ROS → Gazebo (teleop publishes here, DiffDrive reads it)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Sensors: Gazebo → ROS
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
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
    )

    # ── RViz (optional) ───────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(show_rviz),
    )

    return LaunchDescription(args + [gz_sim, rsp, spawn, bridge, rviz])
