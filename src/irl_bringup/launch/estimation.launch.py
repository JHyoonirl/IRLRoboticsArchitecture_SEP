"""Publish robot state and the calibrated fixed camera mounting transform."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    extrinsics = Path(get_package_share_directory("irl_camera_calibration")) / "config" / "camera_extrinsics.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("camera_extrinsics_file", default_value=str(extrinsics)),
        DeclareLaunchArgument("publish_camera_tf", default_value="true"),
        DeclareLaunchArgument("use_mock_robot_state", default_value="true"),
        Node(
            package="irl_robot_estimation", executable="camera_tf", output="screen",
            parameters=[{"camera_extrinsics_file": LaunchConfiguration("camera_extrinsics_file")}],
            condition=IfCondition(LaunchConfiguration("publish_camera_tf")),
        ),
        Node(
            package="irl_robot_estimation", executable="mock_state_estimation", output="screen",
            condition=IfCondition(LaunchConfiguration("use_mock_robot_state")),
        ),
    ])
