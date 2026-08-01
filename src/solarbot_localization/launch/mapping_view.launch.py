import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    proc_share = get_package_share_directory('solarbot_localization')

    headless_arg = DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo headless')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2')

    rviz_config = os.path.join(proc_share, 'config', 'map_2odom.rviz')
    odom_vis_launch = os.path.join(proc_share, 'launch', 'odom_vis.launch.py')

    return LaunchDescription([
        headless_arg,
        use_rviz_arg,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odom_vis_launch),
            launch_arguments={
                'headless': LaunchConfiguration('headless')
            }.items()
        ),
    ])
