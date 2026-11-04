from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


# Nav2's navigation_launch.py isn't used because it routes /cmd_vel through collision_monitor,
# which needs a scan source, and it starts the route and docking servers that R5 doesn't use.
# Commands go to twist_mux (r5_bringup/command.launch.py), which passes them to /cmd_vel.
def generate_launch_description():
    params = ParameterFile(
        PathJoinSubstitution([FindPackageShare("r5_navigation"), "params", "nav2_params.yaml"]),
        allow_substs=True,
    )

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="both",
        parameters=[params],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )
    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="both",
        parameters=[params],
    )
    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="both",
        parameters=[params],
    )
    waypoint_follower_node = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="both",
        parameters=[params],
    )
    velocity_smoother_node = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="both",
        parameters=[params],
        remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "/mux/nav")],
    )
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="both",
        parameters=[
            {
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "velocity_smoother",
                ],
            }
        ],
    )

    return LaunchDescription(
        [
            controller_server_node,
            planner_server_node,
            bt_navigator_node,
            waypoint_follower_node,
            velocity_smoother_node,
            lifecycle_manager_node,
        ]
    )
