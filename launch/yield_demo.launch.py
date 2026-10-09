import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("car_swarm_agent")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("playback_rate", default_value="2.0"),
        DeclareLaunchArgument("show_before", default_value="true"),
        DeclareLaunchArgument("loop", default_value="true"),
        DeclareLaunchArgument("planner_mode", default_value="st"),
        DeclareLaunchArgument("static_map_enabled", default_value="true"),
        DeclareLaunchArgument("execution_mode", default_value="playback"),
        DeclareLaunchArgument("model_time_step", default_value="0.01"),
        DeclareLaunchArgument("model_lateral_offset", default_value="0.25"),
        DeclareLaunchArgument("model_heading_offset", default_value="0.04"),
        DeclareLaunchArgument("model_speed_scale", default_value="0.9"),
        DeclareLaunchArgument("initial_speed", default_value="1.0"),
        DeclareLaunchArgument("max_speed", default_value="1.0"),
        DeclareLaunchArgument("max_acceleration", default_value="1.0"),
        Node(
            package="car_swarm_agent",
            executable="yield_demo_node",
            name="yield_demo",
            output="screen",
            parameters=[{
                "playback_rate": ParameterValue(
                    LaunchConfiguration("playback_rate"), value_type=float),
                "show_before": ParameterValue(
                    LaunchConfiguration("show_before"), value_type=bool),
                "loop": ParameterValue(LaunchConfiguration("loop"), value_type=bool),
                "planner_mode": LaunchConfiguration("planner_mode"),
                "static_map_enabled": ParameterValue(
                    LaunchConfiguration("static_map_enabled"), value_type=bool),
                "execution_mode": LaunchConfiguration("execution_mode"),
                "model_time_step": ParameterValue(
                    LaunchConfiguration("model_time_step"), value_type=float),
                "model_lateral_offset": ParameterValue(
                    LaunchConfiguration("model_lateral_offset"), value_type=float),
                "model_heading_offset": ParameterValue(
                    LaunchConfiguration("model_heading_offset"), value_type=float),
                "model_speed_scale": ParameterValue(
                    LaunchConfiguration("model_speed_scale"), value_type=float),
                "initial_speed": ParameterValue(
                    LaunchConfiguration("initial_speed"), value_type=float),
                "max_speed": ParameterValue(
                    LaunchConfiguration("max_speed"), value_type=float),
                "max_acceleration": ParameterValue(
                    LaunchConfiguration("max_acceleration"), value_type=float),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="yield_demo_rviz",
            arguments=["-d", os.path.join(share, "rviz", "yield_demo.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
