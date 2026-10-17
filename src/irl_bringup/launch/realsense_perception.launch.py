"""Compatibility launch for the local sensing boundary and perception."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("irl_bringup")
    sensing_launch = os.path.join(package_share, "launch", "sensing.launch.py")
    perception_launch = os.path.join(package_share, "launch", "perception.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("use_realsense", default_value="true"),
        DeclareLaunchArgument("camera_mode", default_value="rgbd"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensing_launch),
            launch_arguments={
                "use_realsense": LaunchConfiguration("use_realsense"),
                "camera_mode": LaunchConfiguration("camera_mode"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            launch_arguments={
                "camera_mode": LaunchConfiguration("camera_mode"),
            }.items(),
        ),
    ])
