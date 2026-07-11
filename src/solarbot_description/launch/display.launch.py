from launch import LaunchDescription

from launch_ros.actions import Node

from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution

from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_path = get_package_share_directory("solarbot_description")

    xacro_file = os.path.join(
        pkg_path,
        "urdf",
        "solarbot.urdf.xacro"
    )

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description
            }
        ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_path, "rviz", "display.rviz"),
        ],
        output="screen",
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        rviz,
    ])
