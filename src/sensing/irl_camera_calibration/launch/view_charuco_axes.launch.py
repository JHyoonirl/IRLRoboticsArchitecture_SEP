import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("irl_camera_calibration")
    default_config = os.path.join(share, "config", "calibration.yaml")
    rviz_config = os.path.join(share, "config", "charuco_axes.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("axes_length_m", default_value="0.04"),
        DeclareLaunchArgument(
            "annotated_image_topic",
            default_value="/camera/calibration/charuco_axes/image",
        ),
        Node(
            package="irl_camera_calibration",
            executable="charuco_axes_viewer",
            name="charuco_axes_viewer",
            output="screen",
            parameters=[{
                "config_file": LaunchConfiguration("config_file"),
                "axes_length_m": LaunchConfiguration("axes_length_m"),
                "annotated_image_topic": LaunchConfiguration("annotated_image_topic"),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="charuco_axes_rviz",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_rviz")),
        ),
    ])
