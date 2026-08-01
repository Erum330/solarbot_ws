import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    headless_arg = DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo headless')

    # This repo's solarbot_gazebo/gazebo.launch.py already declares its
    # own 'headless' arg and handles the gz sim GUI/server split
    # internally - just pass headless through directly.
    gazebo_share = get_package_share_directory('solarbot_gazebo')
    gazebo_launch = os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        headless_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
            }.items()
        ),

        # ECC visual odometry, delayed 10s to let the sim settle before
        # transform_detector starts processing camera frames.
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='solarbot_localization',
                    executable='transform_detector',
                    name='transform_detector',
                    output='screen'
                )
            ]
        )
    ])
