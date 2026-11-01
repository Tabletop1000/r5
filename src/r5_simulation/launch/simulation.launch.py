from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# ros2_control_node isn't started because Gazebo runs the controller manager itself.
def generate_launch_description():
    headless = LaunchConfiguration("headless")

    declared_arguments = [
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo without its window.",
        )
    ]

    world = PathJoinSubstitution([FindPackageShare("r5_simulation"), "worlds", "flat.sdf"])
    bridge_config = PathJoinSubstitution(
        [FindPackageShare("r5_simulation"), "config", "gz_bridge.yaml"]
    )
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("r5"), "config", "diffbot_controllers.yaml"]
    )
    navsat_params = PathJoinSubstitution(
        [FindPackageShare("r5_simulation"), "config", "navsat_sim.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("r5"), "urdf", "r5.urdf.xacro"]),
            " ",
            "use_gazebo:=true",
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [
                PythonExpression(["'-s ' if '", headless, "' == 'true' else ''"]),
                "-r -v 2 ",
                world,
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="both",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
    )
    sensor_covariance_node = Node(
        package="r5_simulation",
        executable="sensor_covariance.py",
        output="both",
        parameters=[{"use_sim_time": True}],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": ParameterValue(robot_description_content, value_type=str),
                "use_sim_time": True,
            }
        ],
    )
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="both",
        arguments=["-world", "flat", "-topic", "robot_description", "-name", "r5", "-z", "0.02"],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diffbot_base_controller",
            "--param-file",
            robot_controllers,
            "--controller-ros-args",
            "-r /diffbot_base_controller/cmd_vel:=/cmd_vel",
        ],
    )

    # The controller manager only exists once the robot is in Gazebo.
    delay_controllers_after_spawn_robot = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[joint_state_broadcaster_spawner, robot_controller_spawner],
        )
    )

    # The robot's own launch files, unchanged except for simulated time.
    robot_stack = GroupAction(
        [
            SetParameter(name="use_sim_time", value=True),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("r5_localisation"), "launch", "localisation.launch.py"]
                    )
                ),
                launch_arguments={"navsat_params": navsat_params}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("r5_navigation"), "launch", "navigation.launch.py"]
                    )
                )
            ),
        ]
    )

    return LaunchDescription(
        declared_arguments
        + [
            gazebo,
            bridge_node,
            sensor_covariance_node,
            robot_state_pub_node,
            spawn_robot_node,
            delay_controllers_after_spawn_robot,
            robot_stack,
        ]
    )
