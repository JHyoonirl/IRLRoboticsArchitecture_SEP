"""Run object and active perception using processed sensing topics."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    perception_config = os.path.join(
        get_package_share_directory("irl_object_perception"), "config", "perception.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("camera_mode", default_value="rgbd"),
        Node(
            package="irl_object_perception",
            executable="object_perception",
            name="perception_node",
            namespace="perception",
            parameters=[perception_config, {
                "color_topic": "/sensing/camera/color/image_raw",
                "depth_topic": "/sensing/camera/aligned_depth_to_color/image_raw",
                "camera_info_topic": "/sensing/camera/color/camera_info",
                "camera_mode": LaunchConfiguration("camera_mode"),
            }],
            output="screen",
        ),
        Node(
            package="irl_active_perception",
            executable="active_perception",
            name="active_perception",
            output="screen",
        ),
    ])
