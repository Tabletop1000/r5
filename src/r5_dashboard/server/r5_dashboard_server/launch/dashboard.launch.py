"""Start rosbridge (localhost only), the dashboard server in front of it (spec D7), and the
robot-side nodes the dashboard uses: robot_status (usage, service schedule, alerts), motor_test,
field_manager (fields, home point, boundary recording) and job_manager (spray jobs).

On the simulator, pass use_sim_time:=true fix_requirement:=any sprayer_stub:=true.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

# Lower-rate copies of frequent topics, for rosbridge and the dashboard's Python nodes. Every
# message a Python subscriber receives costs CPU, even if it's then thrown away, and rosbridge's
# throttle_rate only drops messages after converting them. The copies are made in C++ without
# decoding the messages. Each entry is (topic, copy, messages per second).
THROTTLED = [
    ("/bno055/imu", "/r5/dashboard/imu", 5.0),
    ("/odometry/local", "/r5/dashboard/odometry/local", 5.0),
    # job_manager checks the robot's position along a row at 10 Hz.
    ("/odometry/global", "/r5/dashboard/odometry/global", 10.0),
]
ODOMETRY_REMAPS = [(topic, copy) for topic, copy, _ in THROTTLED if topic.startswith("/odometry/")]


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
        DeclareLaunchArgument(
            "fix_requirement",
            default_value="rtk_fixed",
            description="Worst GPS fix accepted for recording boundaries and running jobs: rtk_fixed, rtk_float or any.",
        ),
        DeclareLaunchArgument(
            "sprayer_stub",
            default_value="false",
            description="Start a stand-in that logs sprayer commands, until the sprayer exists.",
        ),
    ]
    fix_requirement = LaunchConfiguration("fix_requirement")
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

    throttle = ComposableNodeContainer(
        name="dashboard_throttle",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="both",
        composable_node_descriptions=[
            ComposableNode(
                package="topic_tools",
                plugin="topic_tools::ThrottleNode",
                name="throttle" + copy.replace("/", "_"),
                parameters=[
                    {
                        "input_topic": topic,
                        "output_topic": copy,
                        "throttle_type": "messages",
                        "msgs_per_sec": rate,
                        # Only subscribes while something uses the copy.
                        "lazy": True,
                        "use_wall_clock": True,
                    }
                ],
            )
            for topic, copy, rate in THROTTLED
        ],
    )

    robot_status = Node(
        package="r5_dashboard_server",
        executable="robot_status",
        output="both",
        # Operating time and alert times use the wall clock even in simulation.
        parameters=[{"data_dir": data_dir}],
        remappings=ODOMETRY_REMAPS,
    )

    # Wheel geometry and joint names match r5's diffbot_controllers.yaml. Commands go through
    # twist_mux (r5_bringup/config/twist_mux.yaml).
    motor_test = Node(
        package="r5_dashboard_server",
        executable="motor_test",
        output="both",
        parameters=[{"use_sim_time": use_sim_time, "cmd_vel_topic": "/mux/test"}],
        remappings=ODOMETRY_REMAPS,
    )

    field_manager = Node(
        package="r5_dashboard_server",
        executable="field_manager",
        output="both",
        parameters=[
            {"data_dir": data_dir, "record_fix_requirement": fix_requirement}
        ],
    )

    job_manager = Node(
        package="r5_dashboard_server",
        executable="job_manager",
        output="both",
        # Wheel and sprayer values are placeholders until the sprayer is built (spec 5.5).
        parameters=[{"data_dir": data_dir, "job_fix_requirement": fix_requirement, "use_sim_time": use_sim_time}],
        remappings=ODOMETRY_REMAPS,
    )

    sprayer_stub = Node(
        package="r5_dashboard_server",
        executable="sprayer_stub",
        output="both",
        condition=IfCondition(LaunchConfiguration("sprayer_stub")),
    )

    return LaunchDescription(
        arguments
        + [rosbridge, server, throttle, robot_status, motor_test, field_manager, job_manager, sprayer_stub]
    )
