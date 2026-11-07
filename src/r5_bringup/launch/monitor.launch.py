from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Robot health for the dashboard: computer temperature, WiFi signal and versions, plus
# /diagnostics grouped into /diagnostics_agg.
def generate_launch_description():
    system_monitor_node = Node(
        package="r5_monitor",
        executable="system_monitor",
        output="both",
    )
    aggregator_node = Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        output="both",
        parameters=[
            PathJoinSubstitution([FindPackageShare("r5_bringup"), "config", "diagnostics.yaml"])
        ],
    )

    return LaunchDescription([system_monitor_node, aggregator_node])
