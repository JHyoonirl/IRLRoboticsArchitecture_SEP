"""Publish filtered object tracks and the aggregated World State."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("irl_world_state")) / "config" / "world_state.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=str(config)),
        Node(package="irl_world_state", executable="world_state", name="world_state",
             parameters=[LaunchConfiguration("config_file")], output="screen"),
    ])
