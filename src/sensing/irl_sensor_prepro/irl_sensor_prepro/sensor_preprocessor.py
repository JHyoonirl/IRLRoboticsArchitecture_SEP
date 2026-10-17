"""Normalize the sensor boundary without changing image encodings or calibration."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan

from .preprocessing import copy_sensor_message, sanitize_joint_state, sanitize_scan


class SensorPreprocessor(Node):
    def __init__(self) -> None:
        super().__init__("sensor_preprocessor")
        # shortcut: calibration and filtering stay in the driver layer until sensor specs are fixed.
        self.declare_parameters("", [
            ("color_input", "/camera/color/image_raw"),
            ("color_output", "/sensing/camera/color/image_raw"),
            ("depth_input", "/camera/aligned_depth_to_color/image_raw"),
            ("depth_output", "/sensing/camera/aligned_depth_to_color/image_raw"),
            ("camera_info_input", "/camera/color/camera_info"),
            ("camera_info_output", "/sensing/camera/color/camera_info"),
            ("joint_state_input", "/joint_states"),
            ("joint_state_output", "/sensing/joint_states"),
            ("imu_input", "/imu/data_raw"),
            ("imu_output", "/sensing/imu"),
            ("scan_input", "/scan"),
            ("scan_output", "/sensing/scan"),
            ("default_frame_id", ""),
        ])
        get = lambda name: str(self.get_parameter(name).value)
        self._frame_id = get("default_frame_id")

        self._color_pub = self.create_publisher(Image, get("color_output"), 10)
        self._depth_pub = self.create_publisher(Image, get("depth_output"), 10)
        self._camera_info_pub = self.create_publisher(CameraInfo, get("camera_info_output"), 10)
        self._joint_pub = self.create_publisher(JointState, get("joint_state_output"), 10)
        self._imu_pub = self.create_publisher(Imu, get("imu_output"), 10)
        self._scan_pub = self.create_publisher(LaserScan, get("scan_output"), 10)

        self.create_subscription(Image, get("color_input"), self._on_color, 10)
        self.create_subscription(Image, get("depth_input"), self._on_depth, 10)
        self.create_subscription(CameraInfo, get("camera_info_input"), self._on_camera_info, 10)
        self.create_subscription(JointState, get("joint_state_input"), self._on_joint_state, 10)
        self.create_subscription(Imu, get("imu_input"), self._on_imu, 10)
        self.create_subscription(LaserScan, get("scan_input"), self._on_scan, 10)

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _on_color(self, message: Image) -> None:
        self._color_pub.publish(copy_sensor_message(message, self._frame_id, self._stamp()))

    def _on_depth(self, message: Image) -> None:
        self._depth_pub.publish(copy_sensor_message(message, self._frame_id, self._stamp()))

    def _on_camera_info(self, message: CameraInfo) -> None:
        self._camera_info_pub.publish(copy_sensor_message(message, self._frame_id, self._stamp()))

    def _on_joint_state(self, message: JointState) -> None:
        self._joint_pub.publish(sanitize_joint_state(message, self._frame_id, self._stamp()))

    def _on_imu(self, message: Imu) -> None:
        self._imu_pub.publish(copy_sensor_message(message, self._frame_id, self._stamp()))

    def _on_scan(self, message: LaserScan) -> None:
        self._scan_pub.publish(sanitize_scan(message, self._frame_id, self._stamp()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorPreprocessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
