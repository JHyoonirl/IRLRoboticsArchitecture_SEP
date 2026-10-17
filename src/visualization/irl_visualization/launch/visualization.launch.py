"""Launch the World State visualizer and an optional Foxglove Bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("world_state_topic", default_value="/state/world"),
        DeclareLaunchArgument("enable_foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_address", default_value="0.0.0.0"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        Node(
            package="irl_visualization",
            executable="world_state_visualizer",
            name="world_state_visualizer",
            parameters=[{
                "world_state_topic": LaunchConfiguration("world_state_topic"),
            }],
            output="screen",
        ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            parameters=[{
                "address": LaunchConfiguration("foxglove_address"),
                "port": LaunchConfiguration("foxglove_port"),
            }],
            condition=IfCondition(LaunchConfiguration("enable_foxglove")),
            output="screen",
        ),
    ])
