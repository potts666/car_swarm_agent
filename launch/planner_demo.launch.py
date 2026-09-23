import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share_directory = get_package_share_directory("car_swarm_agent")

    parameters_file = os.path.join(
        package_share_directory,
        "config",
        "planner_params.yaml",
    )

    return LaunchDescription([
        Node(
            package="car_swarm_agent",
            executable="agent_node",
            name="car_swarm_agent",
            output="screen",
            parameters=[parameters_file],
        )
    ])