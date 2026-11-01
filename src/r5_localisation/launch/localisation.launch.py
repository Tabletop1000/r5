from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("r5_localisation"), "params", "ekf.yaml"])
    navsat_params = LaunchConfiguration("navsat_params")

    declared_arguments = [
        DeclareLaunchArgument(
            "navsat_params",
            default_value=params,
            description="Parameter file applied to navsat_transform after ekf.yaml.",
        )
    ]

    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="both",
        parameters=[params],
        remappings=[("odometry/filtered", "odometry/local")],
    )
    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="both",
        parameters=[params],
        remappings=[("odometry/filtered", "odometry/global")],
    )
    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="both",
        parameters=[params, navsat_params],
        remappings=[
            ("imu", "/bno055/imu"),
            ("gps/fix", "/fix"),
            ("odometry/filtered", "odometry/global"),
        ],
    )

    return LaunchDescription(
        declared_arguments + [ekf_local_node, ekf_global_node, navsat_transform_node]
    )
