import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port', default_value='/dev/ttyACM0',
        description='Serial port the ESP32 (micro-ROS) is on')
    gps_port_arg = DeclareLaunchArgument(
        'gps_port', default_value='/dev/ttyUSB0',
        description='Serial port the GPS module is on')
    camera_device_arg = DeclareLaunchArgument(
        'camera_device', default_value='/dev/video0',
        description='V4L2 device for the downward camera')
    use_gps_arg = DeclareLaunchArgument(
        'use_gps', default_value='true',
        description='Start the GPS driver')

    esp32_port = LaunchConfiguration('esp32_port')
    gps_port = LaunchConfiguration('gps_port')
    camera_device = LaunchConfiguration('camera_device')

    # ------------------------------------------------------------
    # 1. Robot description / TF tree - everything else assumes the
    #    URDF frames (imu_link, front_mid_tof_link, ...) already exist.
    # ------------------------------------------------------------
    description_share = get_package_share_directory('solarbot_description')
    xacro_file = os.path.join(description_share, 'urdf', 'solarbot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config.toxml(),
            'use_sim_time': False,
        }],
    )

    # ------------------------------------------------------------
    # 2. ESP32 link: micro-ROS agent -> mros_converter
    #    (from panelbot2_ws; must be sourced/overlaid alongside this ws)
    # ------------------------------------------------------------
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_serial',
        output='screen',
        arguments=['serial', '--dev', esp32_port, '-b', '1000000'],
    )

    converter_node = Node(
        package='mros_converter',
        executable='converter_node',
        name='converter_node',
        output='screen',
    )

    safety_monitor = Node(
        package='mros_converter',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
        parameters=[{'enable_diagnostics': True}],
    )

    # ------------------------------------------------------------
    # 3. Camera + GPS, running directly on the Pi
    # ------------------------------------------------------------
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': camera_device,
            'image_size': [1280, 720],
            'pixel_format': 'YUYV',
            'camera_frame_id': 'downward_camera_optical_frame',
        }],
    )

    gps_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='gps_driver',
        output='screen',
        parameters=[{
            'port': gps_port,
            'baud': 115200,
            'frame_id': 'gps_link',
            'useRMC': True,
            'time_ref_source': 'gps',
        }],
        condition=IfCondition(LaunchConfiguration('use_gps')),
    )

    # ------------------------------------------------------------
    # 4. solarbot_bridge: mros_interfaces <-> standard ROS msgs
    #    Started after the converter has had a moment to come up so
    #    the bridges aren't spinning on topics that don't exist yet.
    # ------------------------------------------------------------
    bridge_nodes = TimerAction(
        period=4.0,
        actions=[
            Node(package='solarbot_bridge', executable='imu_bridge',
                 name='imu_bridge', output='screen'),
            Node(package='solarbot_bridge', executable='tof_bridge',
                 name='tof_bridge', output='screen'),
            Node(package='solarbot_bridge', executable='cmd_vel_bridge',
                 name='cmd_vel_bridge', output='screen'),
        ],
    )

    # ------------------------------------------------------------
    # 5. Vision odometry - needs real camera + /imu frames flowing,
    #    hence the longer delay (mirrors odom_vis.launch.py's 10s).
    # ------------------------------------------------------------
    transform_detector = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='solarbot_localization',
                executable='transform_detector',
                name='transform_detector',
                output='screen',
            ),
        ],
    )

    # ------------------------------------------------------------
    # 6. Behavior: direct-drive coverage FSM (not Nav2 - see prior
    #    notes on NavigateThroughPoses getting stuck on real hardware).
    #    NOT auto-started - uncomment once you've bench-verified every
    #    topic above with `ros2 topic echo`, and calibrated cmd_scale.
    # ------------------------------------------------------------
    # coverage_fsm = TimerAction(
    #     period=12.0,
    #     actions=[
    #         Node(
    #             package='solarbot_safety',
    #             executable='solarbot_roof_coverage_node',
    #             name='solarbot_roof_coverage_node',
    #             output='screen',
    #             parameters=[{
    #                 'odom_topic': '/odom_cam',
    #                 'cmd_vel_topic': '/cmd_vel',
    #             }],
    #         ),
    #     ],
    # )

    return LaunchDescription([
        esp32_port_arg,
        gps_port_arg,
        camera_device_arg,
        use_gps_arg,

        robot_state_publisher,
        micro_ros_agent,
        converter_node,
        safety_monitor,
        camera_node,
        gps_node,
        bridge_nodes,
        transform_detector,
        # coverage_fsm,
    ])
