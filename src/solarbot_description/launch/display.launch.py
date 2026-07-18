"""
display.launch.py
─────────────────────────────────────────────────────────────────────
Launches a full URDF visualisation stack for SolarBot:

  robot_state_publisher   — publishes /robot_description + static TFs
  joint_state_publisher_gui — GUI sliders for manually moving joints
  rviz2                   — 3D visualiser, loads display.rviz config

Usage:
  ros2 launch solarbot_description display.launch.py

Optional arguments:
  use_joint_state_gui:=false   headless, no GUI sliders
  use_rviz:=false              skip RViz (useful for CI / scripted tests)
  use_real_hardware:=true      load real SkidSteerHardware plugin
  serial_port:=/dev/ttyACM0   Arduino USB port (only when above = true)
  xacro_path:=<full_path>      override the default xacro file location
  rviz_config:=<full_path>     override the default RViz config
─────────────────────────────────────────────────────────────────────
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Package share directory ────────────────────────────────
    pkg = FindPackageShare('solarbot_description')

    # ── Default file paths ─────────────────────────────────────
    default_xacro   = PathJoinSubstitution([pkg, 'urdf',  'solarbot.urdf.xacro'])
    default_rviz    = PathJoinSubstitution([pkg, 'rviz',  'display.rviz'])

    # ── Launch arguments ───────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'use_joint_state_gui',
            default_value='true',
            description='Start joint_state_publisher_gui (joint sliders)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2'
        ),
        DeclareLaunchArgument(
            'use_real_hardware',
            default_value='false',
            description='Load SkidSteerHardware plugin (requires Arduino connected)'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='USB serial port for the Arduino motor bridge'
        ),
        DeclareLaunchArgument(
            'xacro_path',
            default_value=default_xacro,
            description='Absolute path to the top-level xacro file'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='Absolute path to the RViz2 config file'
        ),
    ]

    # ── Convenience references ─────────────────────────────────
    use_gui          = LaunchConfiguration('use_joint_state_gui')
    use_rviz         = LaunchConfiguration('use_rviz')
    use_real_hw      = LaunchConfiguration('use_real_hardware')
    serial_port      = LaunchConfiguration('serial_port')
    xacro_path       = LaunchConfiguration('xacro_path')
    rviz_config      = LaunchConfiguration('rviz_config')

    # ── Process xacro → URDF string ───────────────────────────
    #
    # ParameterValue(..., value_type=str) is required in ROS 2 Jazzy+
    # to prevent the launch system trying to parse the multi-line URDF
    # XML string as YAML (which fails with "Unable to parse as yaml").
    #
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_path,
            ' use_real_hardware:=', use_real_hw,
            ' serial_port:=',       serial_port,
        ]),
        value_type=str,
    )

    # ── Nodes ──────────────────────────────────────────────────

    # Publishes /robot_description and all fixed TF transforms
    # derived from the URDF joint tree.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # GUI sliders — lets you manually command each continuous joint
    # (left_wheel_joint, right_wheel_joint) to verify the URDF moves
    # correctly before connecting any controllers.
    # Disabled with use_joint_state_gui:=false.
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui),
    )

    # RViz2 — loads display.rviz with RobotModel, TF and Grid pre-configured.
    # TF marker scale is set to 0.08 in the config so the axis arrows
    # don't overwhelm the model (the old "can't see the body" problem).
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    # ── Assemble launch description ────────────────────────────
    return LaunchDescription(
        args + [
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz2,
        ]
    )
