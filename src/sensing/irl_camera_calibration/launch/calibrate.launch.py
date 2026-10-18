import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("irl_camera_calibration"),
        "config",
        "calibration.yaml",
    )
    rviz_config = os.path.join(
        get_package_share_directory("irl_camera_calibration"),
        "config",
        "calibration.rviz",
    )
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        Node(
            package="irl_camera_calibration",
            executable="charuco_extrinsic_calibration",
            name="charuco_extrinsic_calibration",
            output="screen",
            parameters=[{"config_file": LaunchConfiguration("config_file")}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="calibration_rviz",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_rviz")),
        ),
    ])
