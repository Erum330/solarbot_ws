#!/usr/bin/env python3
"""
bridges.launch.py

Launches the three interface bridges:
  1. tof_bridge:     mros_interfaces/Tof -> 3x LaserScan + 1x PointCloud2
  2. imu_bridge:     mros_interfaces/Imu -> sensor_msgs/Imu (/imu)
  3. cmd_vel_bridge: geometry_msgs/Twist (/cmd_vel) -> mros_interfaces/MotorCmd (/motorCmd)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---------------- Launch Configurations / Arguments ----------------
    # cmd_vel_bridge arguments
    publish_raw_mps_arg = DeclareLaunchArgument(
        'publish_raw_mps',
        default_value='false',
        description='True = publish m/s directly (bench test). False = scale by cmd_scale for firmware.'
    )
    cmd_scale_arg = DeclareLaunchArgument(
        'cmd_scale',
        default_value='350.0',
        description='Scale factor mapping (m/s) to raw firmware PWM/counts.'
    )
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation_m',
        default_value='0.156',
        description='Distance between left and right drive wheels in meters.'
    )
    cmd_vel_timeout_arg = DeclareLaunchArgument(
        'cmd_vel_timeout_sec',
        default_value='0.5',
        description='Watchdog timeout to zero motors if /cmd_vel stops streaming.'
    )

    # imu_bridge arguments
    gyro_is_deg_per_sec_arg = DeclareLaunchArgument(
        'gyro_is_deg_per_sec',
        default_value='false',
        description='Set True if firmware reports gyro in deg/s instead of rad/s.'
    )
    imu_frame_id_arg = DeclareLaunchArgument(
        'imu_frame_id',
        default_value='imu_link',
        description='Frame ID attached to output sensor_msgs/Imu.'
    )

    # tof_bridge arguments
    tof_unit_is_mm_arg = DeclareLaunchArgument(
        'tof_unit_is_mm',
        default_value='true',
        description='True if raw Tof integer readings are in millimeters (converts to meters).'
    )
    grid_fov_deg_arg = DeclareLaunchArgument(
        'grid_fov_deg',
        default_value='45.0',
        description='Field of view for the 8x8 multizone ToF in degrees.'
    )

    # ---------------- Node Definitions ----------------
    tof_bridge_node = Node(
        package='solarbot_bridge',
        executable='tof_bridge',
        name='tof_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'input_topic': 'tof/data',
            'tof_unit_is_mm': LaunchConfiguration('tof_unit_is_mm'),
            'range_min_m': 0.02,
            'range_max_m': 2.0,
            'grid_fov_deg': LaunchConfiguration('grid_fov_deg'),
            'grid_size': 8,
        }]
    )

    imu_bridge_node = Node(
        package='solarbot_bridge',
        executable='imu_bridge',
        name='imu_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'input_topic': 'imu/data',
            'output_topic': '/imu',
            'frame_id': LaunchConfiguration('imu_frame_id'),
            'gyro_is_deg_per_sec': LaunchConfiguration('gyro_is_deg_per_sec'),
        }]
    )

    cmd_vel_bridge_node = Node(
        package='solarbot_bridge',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/motorCmd',
            'wheel_separation_m': LaunchConfiguration('wheel_separation_m'),
            'cmd_scale': LaunchConfiguration('cmd_scale'),
            'publish_raw_mps': LaunchConfiguration('publish_raw_mps'),
            'cmd_vel_timeout_sec': LaunchConfiguration('cmd_vel_timeout_sec'),
        }]
    )

    return LaunchDescription([
        publish_raw_mps_arg,
        cmd_scale_arg,
        wheel_separation_arg,
        cmd_vel_timeout_arg,
        gyro_is_deg_per_sec_arg,
        imu_frame_id_arg,
        tof_unit_is_mm_arg,
        grid_fov_deg_arg,
        tof_bridge_node,
        imu_bridge_node,
        cmd_vel_bridge_node,
    ])