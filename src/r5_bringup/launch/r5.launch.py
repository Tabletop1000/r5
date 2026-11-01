from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def include_launch(package, launch_file):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        )
    )


def generate_launch_description():
    return LaunchDescription(
        [
            include_launch("r5", "diffbot.launch.py"),
            include_launch("r5_sensors", "bno055_imu_node-launch.py"),
            include_launch("r5_sensors", "ublox_gps_node-launch.py"),
            include_launch("r5_localisation", "localisation.launch.py"),
            include_launch("r5_navigation", "navigation.launch.py"),
        ]
    )
