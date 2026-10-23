"""Run the complete architecture by composing the sensing and perception launches."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("irl_bringup")
    visualization_share = get_package_share_directory("irl_visualization")
    sensing_launch = os.path.join(package_share, "launch", "sensing.launch.py")
    perception_launch = os.path.join(package_share, "launch", "perception.launch.py")
    visualization_launch = os.path.join(
        visualization_share, "launch", "visualization.launch.py"
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_realsense", default_value="true"),
        DeclareLaunchArgument("camera_mode", default_value="rgbd"),
        DeclareLaunchArgument("enable_visualization", default_value="false"),
        DeclareLaunchArgument("publish_camera_tf", default_value="true"),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(visualization_launch),
            launch_arguments={
                "enable_foxglove": "true",
            }.items(),
            condition=IfCondition(LaunchConfiguration("enable_visualization")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_share, "launch", "estimation.launch.py")),
            launch_arguments={
                "publish_camera_tf": LaunchConfiguration("publish_camera_tf"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_share, "launch", "world_state.launch.py")),
        ),
    ])
