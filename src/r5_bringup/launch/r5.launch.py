from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def include_launch(package, launch_file, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        **kwargs,
    )


# The real robot: motor hardware and sensor drivers, then everything above them (stack.launch.py).
def generate_launch_description():
    stack_arguments = ["dashboard", "joystick", "fix_requirement", "sprayer_stub"]
    declared_arguments = [
        DeclareLaunchArgument(
            "dashboard",
            default_value="true",
            description="Start the dashboard server and its robot-side nodes.",
        ),
        DeclareLaunchArgument(
            "joystick",
            default_value="true",
            description="Start the Xbox controller nodes.",
        ),
        DeclareLaunchArgument(
            "fix_requirement",
            default_value="any",
            description="Worst GPS fix the dashboard accepts: rtk_fixed, rtk_float or any.",
        ),
        DeclareLaunchArgument(
            "sprayer_stub",
            default_value="true",
            description="Log sprayer commands, until the sprayer exists.",
        ),
    ]

    return LaunchDescription(
        declared_arguments
        + [
            include_launch("r5", "diffbot.launch.py"),
            include_launch("r5_sensors", "bno055_imu_node-launch.py"),
            include_launch("r5_sensors", "ublox_gps_node-launch.py"),
            include_launch(
                "r5_bringup",
                "stack.launch.py",
                launch_arguments={
                    name: LaunchConfiguration(name) for name in stack_arguments
                }.items(),
            ),
        ]
    )
