"""Run the existing RGB-D perception pipeline under the perception namespace."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = os.path.join(
        get_package_share_directory("irl_object_perception"), "config", "perception.yaml"
    )
    return LaunchDescription([
        Node(
            package="irl_object_perception",
            executable="object_perception",
            name="perception_node",
            namespace="perception",
            parameters=[config, {
                "color_topic": "/sensing/camera/color/image_raw",
                "depth_topic": "/sensing/camera/aligned_depth_to_color/image_raw",
                "camera_info_topic": "/sensing/camera/color/camera_info",
            }],
            output="screen",
        ),
    ])
