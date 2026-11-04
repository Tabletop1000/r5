from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def include_launch(package, launch_file, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        **kwargs,
    )


# Everything that runs above the hardware: localisation, Nav2, velocity commands, monitoring and
# the dashboard. r5.launch.py runs it with the real robot, and r5_simulation with Gazebo, so the
# two can't drift apart.
def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use Gazebo's clock.",
        ),
        DeclareLaunchArgument(
            "navsat_params",
            default_value=PathJoinSubstitution(
                [FindPackageShare("r5_localisation"), "params", "ekf.yaml"]
            ),
            description="Parameter file applied to navsat_transform after ekf.yaml.",
        ),
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

    robot_nodes = GroupAction(
        [
            SetParameter(name="use_sim_time", value=use_sim_time),
            include_launch(
                "r5_localisation",
                "localisation.launch.py",
                launch_arguments={"navsat_params": LaunchConfiguration("navsat_params")}.items(),
            ),
            include_launch("r5_navigation", "navigation.launch.py"),
            include_launch(
                "r5_bringup",
                "command.launch.py",
                launch_arguments={"joystick": LaunchConfiguration("joystick")}.items(),
            ),
            include_launch("r5_bringup", "monitor.launch.py"),
        ]
    )
    # Outside the group, because the dashboard chooses per node which clock to use.
    dashboard = include_launch(
        "r5_dashboard_server",
        "dashboard.launch.py",
        launch_arguments={
            "use_sim_time": use_sim_time,
            "fix_requirement": LaunchConfiguration("fix_requirement"),
            "sprayer_stub": LaunchConfiguration("sprayer_stub"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("dashboard")),
    )

    return LaunchDescription(declared_arguments + [robot_nodes, dashboard])
