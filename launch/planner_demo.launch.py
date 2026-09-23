# launch 文件：节点名称、可执行程序、参数
# C++ 节点：创建规划器、生成路径
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="car_swarm_agent",
            executable="agent_node",
            name="car_swarm_agent",
            output="screen",
            parameters=[
                {
                    "start_x": 1.0,
                    "start_y": -2.0,
                    "goal_x": 7.0,
                    "goal_y": 2.0,
                    "num_segments": 4,
                }
            ],
        )
    ])