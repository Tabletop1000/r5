import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from r5_limits import fill


# Velocity commands go through twist_mux, so Nav2, the dashboard and the Xbox controller can't
# fight over /cmd_vel. See config/twist_mux.yaml for the inputs and their priorities.
def generate_launch_description():
    joystick = LaunchConfiguration("joystick")
    config = PathJoinSubstitution([FindPackageShare("r5_bringup"), "config"])
    # Turbo speeds are the top speeds from r5's config/limits.yaml.
    joystick_params = fill(
        os.path.join(get_package_share_directory("r5_bringup"), "config", "joystick.yaml")
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "joystick",
            default_value="true",
            description="Start the Xbox controller nodes.",
        )
    ]

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        output="both",
        parameters=[PathJoinSubstitution([config, "twist_mux.yaml"])],
        remappings=[("cmd_vel_out", "/cmd_vel")],
    )
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="both",
        parameters=[joystick_params],
        condition=IfCondition(joystick),
    )
    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="both",
        parameters=[joystick_params],
        remappings=[("cmd_vel", "/mux/joy")],
        condition=IfCondition(joystick),
    )

    return LaunchDescription(
        declared_arguments + [twist_mux_node, joy_node, teleop_twist_joy_node]
    )
