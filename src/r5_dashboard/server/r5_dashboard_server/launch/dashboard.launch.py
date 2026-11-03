"""Start rosbridge (localhost only), the dashboard server in front of it (spec D7), and the
robot-side nodes the dashboard uses: robot_status (usage, service schedule, alerts) and motor_test.

On the simulator, pass use_sim_time:=true.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    https_port = LaunchConfiguration("https_port")
    http_port = LaunchConfiguration("http_port")
    data_dir = LaunchConfiguration("data_dir")

    arguments = [
        DeclareLaunchArgument("https_port", default_value="8443"),
        DeclareLaunchArgument(
            "http_port", default_value="8080", description="Redirects to HTTPS. 0 turns it off."
        ),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        DeclareLaunchArgument("data_dir", default_value="~/.r5"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Only reachable from the robot itself. Browsers connect through the dashboard server.
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rosbridge_server"), "launch", "rosbridge_websocket_launch.xml"]
            )
        ),
        launch_arguments={"address": "127.0.0.1", "port": rosbridge_port}.items(),
    )

    server = Node(
        package="r5_dashboard_server",
        executable="server",
        output="both",
        arguments=[
            "--https-port", https_port,
            "--http-port", http_port,
            "--data-dir", data_dir,
            "--rosbridge-url", ["ws://127.0.0.1:", rosbridge_port],
        ],
    )

    robot_status = Node(
        package="r5_dashboard_server",
        executable="robot_status",
        output="both",
        # Operating time and alert times use the wall clock even in simulation.
        parameters=[{"data_dir": data_dir}],
    )

    # Wheel geometry and joint names match r5's diffbot_controllers.yaml.
    motor_test = Node(
        package="r5_dashboard_server",
        executable="motor_test",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(arguments + [rosbridge, server, robot_status, motor_test])
