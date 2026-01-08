import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("pgrsa_2d_slam")
    default_params = os.path.join(pkg_share, "config", "rtabmap_params.yaml")

    # Launch args
    use_sim_time = LaunchConfiguration("use_sim_time")
    frame_id = LaunchConfiguration("frame_id")
    lidar_topic = LaunchConfiguration("lidar_topic")
    initial_pose = LaunchConfiguration("initial_pose")
    params_file = LaunchConfiguration("rtabmap_params_file")

    # Typed parameter to ensure bool (not string)
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    # ---- ICP Odometry (LiDAR-only) publishes /icp_odom (remapped) ----
    icp_odometry = Node(
        package="rtabmap_odom",
        executable="icp_odometry",
        name="icp_odometry",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time_param,
                "frame_id": frame_id,
                "initial_pose": initial_pose,
            },
        ],
        remappings=[
            ("scan_cloud", lidar_topic),
            ("odom", "/icp_odom"),
            ("imu", "/imu"),
        ],
    )

    # ---- RTAB-Map SLAM uses EKF odom (/odometry/filtered) ----
    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time_param,
                "frame_id": frame_id,
                "initial_pose": initial_pose,
            },
        ],
        remappings=[
            # LiDAR
            ("scan_cloud", lidar_topic),

            # Odom / IMU
            ("odom", "/odometry/filtered"),
            ("imu", "/imu"),

            # RGB-D (adjust to your bridge topics)
            ("rgb/image", "/camera/image"),
            ("depth/image", "/camera/depth_image"),
            ("rgb/camera_info", "/camera/camera_info"),
        ],
        arguments=["-d"],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulated clock (/clock).",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="base_link",
                description="Robot base frame.",
            ),
            DeclareLaunchArgument(
                "lidar_topic",
                default_value="/lidar_3d/points",
                description="3D LiDAR PointCloud2 topic.",
            ),
            DeclareLaunchArgument(
                "initial_pose",
                default_value="0 7 0.84 0 0 0",
                description="Initial pose: x y z roll pitch yaw",
            ),
            DeclareLaunchArgument(
                "rtabmap_params_file",
                default_value=default_params,
                description="YAML with RTAB-Map + ICP odometry parameters.",
            ),
            icp_odometry,
            rtabmap,
        ]
    )