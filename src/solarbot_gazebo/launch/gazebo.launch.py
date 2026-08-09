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

            # ToF sensors - 4 middle-of-side layout (replaces the old
            # front+4-corner arrangement). front_mid/rear_mid/left_mid
            # are point sensors (VL53L0X-class); right_mid is the
            # VL53L5CX-class grid sensor, simulated as a multi-sample
            # horizontal scan (see solarbot_description/urdf/gazebo/
            # gazebo.xacro for the sensor definition itself).
            "/front_mid_tof@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/rear_mid_tof@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/left_mid_tof@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # right_mid_tof: the real VL53L5CX is a genuine 8x8 grid, and
            # sensor_msgs/LaserScan (bridged above for the other 3 point
            # sensors) is strictly 1D - confirmed empirically that
            # ros_gz_bridge's LaserScan converter silently truncates a
            # true 2D gz.msgs.LaserScan down to a single row (8 of 64
            # values came through, not an error, just quietly wrong).
            # gz-sim's gpu_lidar sensor also auto-publishes a proper
            # point cloud on "<topic>/points" - bridging THAT instead
            # preserves the full 2D structure natively.
            "/right_mid_tof/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",

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