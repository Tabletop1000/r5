"""Start rosbridge (localhost only), the dashboard server in front of it (spec D7), and the
robot-side nodes the dashboard uses: robot_status (usage, service schedule, alerts), motor_test,
field_manager (fields, home point, boundary recording), job_manager (spray jobs), and a container
of C++ components (topic throttles and the /r5/ping service).

On the simulator, pass use_sim_time:=true fix_requirement:=any sprayer_stub:=true.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

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
# The dashboard nodes read the copies. job_manager reads the IMU for the slope limit.
REMAPS = [(topic, copy) for topic, copy, _ in THROTTLED]


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
    # Started directly rather than through rosbridge's launch file, which also starts rosapi (about
    # 50 MB); the web app uses /r5/ping instead. The two values that differ from rosbridge's own
    # defaults are the ones its launch file sets.
    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="both",
        parameters=[
            {
                "address": "127.0.0.1",
                "port": ParameterValue(rosbridge_port, value_type=int),
                "max_message_size": 10000000,
                "retry_startup_delay": 5.0,
            }
        ],
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

    # The dashboard's C++ nodes, in one process.
    components = ComposableNodeContainer(
        name="dashboard_nodes",
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
        ]
        + [
            # Answers at once; the web app uses it for the round-trip time and the link check.
            ComposableNode(package="r5_dashboard_nodes", plugin="r5_dashboard::Ping", name="ping"),
        ],
    )

    robot_status = Node(
        package="r5_dashboard_server",
        executable="robot_status",
        output="both",
        # Operating time and alert times use the wall clock even in simulation.
        parameters=[{"data_dir": data_dir}],
        remappings=REMAPS,
    )

    # Wheel geometry and joint names match r5's diffbot_controllers.yaml. Commands go through
    # twist_mux (r5_bringup/config/twist_mux.yaml).
    motor_test = Node(
        package="r5_dashboard_server",
        executable="motor_test",
        output="both",
        parameters=[{"use_sim_time": use_sim_time, "cmd_vel_topic": "/mux/test"}],
        remappings=REMAPS,
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
        remappings=REMAPS,
    )

    sprayer_stub = Node(
        package="r5_dashboard_server",
        executable="sprayer_stub",
        output="both",
        condition=IfCondition(LaunchConfiguration("sprayer_stub")),
    )

    return LaunchDescription(
        arguments
        + [rosbridge, server, components, robot_status, motor_test, field_manager, job_manager, sprayer_stub]
    )
