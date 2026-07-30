import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ----------------------------------------------------------
    # Launch Configurations & Arguments
    # ----------------------------------------------------------
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo Sim headless (-s server only)'
    )

    headless_config = LaunchConfiguration('headless')

    # ----------------------------------------------------------
    # Packages
    # ----------------------------------------------------------
    package_name = "solarbot_gazebo"
    description_package = "solarbot_description"

    # ----------------------------------------------------------
    # Robot State Publisher
    # ----------------------------------------------------------
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "rsp.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "True"}.items(),
    )

    # ----------------------------------------------------------
    # Gazebo World
    # ----------------------------------------------------------
    world_file = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        "warehouse_rooftop.sdf",
    )

    # 1. Headless = True (Server only: gz sim -v 4 -s -r world.sdf)
    gz_sim_headless = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", "-s", "-r", world_file],
        output="screen",
        condition=IfCondition(headless_config),
        additional_env={
            "GZ_SIM_SYSTEM_PLUGIN_PATH":
                "/opt/ros/jazzy/lib:/opt/ros/jazzy/opt/gz_sim_vendor/lib"
        },
    )

    # 2. Headless = False (GUI + Server: gz sim -v 4 -r world.sdf)
    gz_sim_gui = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", "-r", world_file],
        output="screen",
        condition=UnlessCondition(headless_config),
        additional_env={
            "GZ_SIM_SYSTEM_PLUGIN_PATH":
                "/opt/ros/jazzy/lib:/opt/ros/jazzy/opt/gz_sim_vendor/lib"
        },
    )

    # ----------------------------------------------------------
    # Spawn Robot
    # ----------------------------------------------------------
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-entity", "solarbot",
            "-x", "-1.75",
            "-y", "3.45",
            "-z", "0.05",
        ],
        output="screen",
    )

    # ----------------------------------------------------------
    # ROS <-> Gazebo Bridge
    # ----------------------------------------------------------
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            # Simulation clock
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

            # Odometry
            "/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",

            # Ground-truth odometry
            "/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",

            # IMU
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",

            # Camera image & metadata
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",

            "/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
            # Front obstacle ToF
            "/front_tof@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",

            # Cliff sensors
            "/front_left_tof@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/front_right_tof@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/rear_left_tof@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/rear_right_tof@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",

            # Joint states
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.JointState",
        ],
    )

    # ----------------------------------------------------------
    # Twist -> TwistStamped
    # ----------------------------------------------------------
    twist_to_stamped_script = os.path.join(
        get_package_share_directory(description_package),
        "scripts",
        "twistToStamped.py",
    )

    twist_to_stamped = ExecuteProcess(
        cmd=["python3", twist_to_stamped_script],
        output="screen",
    )

    # ----------------------------------------------------------
    # ros2_control
    # ----------------------------------------------------------
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # ----------------------------------------------------------
    # Launch
    # ----------------------------------------------------------
    return LaunchDescription([
        headless_arg,
        rsp,
        gz_sim_headless,
        gz_sim_gui,
        spawn_entity,
        bridge,
        twist_to_stamped,
        joint_broad_spawner,
        diff_drive_spawner,
    ])