"""Run the local camera boundary and sensing preprocessor only."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    camera_launch = os.path.join(
        get_package_share_directory("irl_sensor"), "launch", "camera.launch.py"
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_realsense", default_value="true"),
        DeclareLaunchArgument("camera_mode", default_value="rgbd"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            launch_arguments={
                "use_realsense": LaunchConfiguration("use_realsense"),
                "camera_mode": LaunchConfiguration("camera_mode"),
            }.items(),
        ),
        Node(
            package="irl_sensor_prepro",
            executable="sensor_preprocessor",
            name="sensor_preprocessor",
            output="screen",
        ),
    ])
