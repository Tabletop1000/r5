from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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


def generate_launch_description():
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
        # The M9N can't give an RTK fix, so recording and spray jobs accept any fix until the
        # ZED-F9 is fitted. Change to rtk_fixed then.
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
            include_launch("r5_localisation", "localisation.launch.py"),
            include_launch("r5_navigation", "navigation.launch.py"),
            include_launch(
                "r5_bringup",
                "command.launch.py",
                launch_arguments={"joystick": LaunchConfiguration("joystick")}.items(),
            ),
            include_launch("r5_bringup", "monitor.launch.py"),
            include_launch(
                "r5_dashboard_server",
                "dashboard.launch.py",
                launch_arguments={
                    "fix_requirement": LaunchConfiguration("fix_requirement"),
                    "sprayer_stub": LaunchConfiguration("sprayer_stub"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("dashboard")),
            ),
        ]
    )
