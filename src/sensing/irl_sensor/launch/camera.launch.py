"""Own the local camera contract while delegating hardware access to RealSense."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include_realsense(context):
    use_realsense = context.perform_substitution(LaunchConfiguration("use_realsense"))
    if use_realsense.lower() not in {"true", "1", "yes"}:
        return []

    camera_mode = context.perform_substitution(LaunchConfiguration("camera_mode")).lower()
    if camera_mode not in {"rgb", "rgbd"}:
        raise RuntimeError("camera_mode must be either 'rgb' or 'rgbd'")

    realsense_launch = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )
    rgbd = camera_mode == "rgbd"
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                "camera_namespace": LaunchConfiguration("camera_namespace"),
                "camera_name": LaunchConfiguration("camera_name"),
                "enable_color": "true",
                "enable_depth": str(rgbd).lower(),
                "align_depth.enable": str(rgbd).lower(),
                "enable_sync": str(rgbd).lower(),
                "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
                "depth_module.depth_profile": LaunchConfiguration("depth_profile"),
                "pointcloud.enable": "false",
                "publish_tf": "true",
            }.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_realsense",
            default_value="true",
            choices=["true", "false"],
            description="Start the RealSense driver.",
        ),
        DeclareLaunchArgument(
            "camera_mode",
            default_value="rgbd",
            choices=["rgb", "rgbd"],
            description="Publish RGB only or aligned RGB-D streams.",
        ),
        DeclareLaunchArgument("camera_namespace", default_value=""),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("color_profile", default_value="640x480x30"),
        DeclareLaunchArgument("depth_profile", default_value="640x480x30"),
        OpaqueFunction(function=_include_realsense),
    ])
