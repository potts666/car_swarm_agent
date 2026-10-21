"""Replay an installed head-on case, independent of the original workspace."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('head_on_mpc'))
    return LaunchDescription([
        DeclareLaunchArgument('study_dir',default_value=str(share/'results/run_001')),
        DeclareLaunchArgument('case',default_value='head_on_pause_bay',choices=[
            'head_on_inline_stop','head_on_pause_bay','head_on_reverse','head_on_deadlock']),
        DeclareLaunchArgument('playback_rate',default_value='1.0'),
        DeclareLaunchArgument('loop',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),
        Node(package='head_on_mpc',executable='rviz_replay.py',output='screen',parameters=[{
            'log_dir':ParameterValue(PathJoinSubstitution([LaunchConfiguration('study_dir'),
                LaunchConfiguration('case')]),value_type=str),
            'playback_rate':ParameterValue(LaunchConfiguration('playback_rate'),value_type=float),
            'loop':ParameterValue(LaunchConfiguration('loop'),value_type=bool)}]),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/replay.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')),output='screen'),
    ])
