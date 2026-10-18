"""Publish the detected ChArUco board axes drawn on the camera image."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CharucoAxesViewer(Node):
    def __init__(self) -> None:
        super().__init__("charuco_axes_viewer")
        default_config = (
            Path(get_package_share_directory("irl_camera_calibration"))
            / "config/calibration.yaml"
        )
        self.declare_parameter("config_file", str(default_config))
        self.declare_parameter(
            "annotated_image_topic", "/camera/calibration/charuco_axes/image"
        )
        self.config_path = Path(self.get_parameter("config_file").value).expanduser().resolve()
        with self.config_path.open(encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)

        camera = self.config["camera"]
        board_path = Path(self.config["board_spec"])
        if not board_path.is_absolute():
            board_path = self.config_path.parent / board_path
        with board_path.open(encoding="utf-8") as stream:
            self.board_config = yaml.safe_load(stream)["board"]

        if getattr(cv2, "aruco", None) is None:
            raise RuntimeError("OpenCV was built without the aruco module")
        dictionary_id = getattr(cv2.aruco, self.board_config["dictionary"])
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        board_size = (
            int(self.board_config["squares_x"]),
            int(self.board_config["squares_y"]),
        )
        square_length = float(self.board_config["square_length_m"])
        marker_length = float(self.board_config["marker_length_m"])
        if hasattr(cv2.aruco, "CharucoBoard_create"):
            self.board = cv2.aruco.CharucoBoard_create(
                *board_size, square_length, marker_length, dictionary
            )
        else:
            self.board = cv2.aruco.CharucoBoard(
                board_size, square_length, marker_length, dictionary
            )

        self.declare_parameter("axes_length_m", 2.0 * square_length)
        self.axes_length = float(self.get_parameter("axes_length_m").value)
        if self.axes_length <= 0.0:
            raise ValueError("axes_length_m must be positive")

        self.bridge = CvBridge()
        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None
        self.annotated_topic = str(self.get_parameter("annotated_image_topic").value)
        self.image_publisher = self.create_publisher(
            # Reliable publishers can serve both RViz's default reliable
            # subscription and best-effort image subscribers.
            Image, self.annotated_topic, QoSProfile(depth=10)
        )
        self.create_subscription(
            CameraInfo,
            camera["camera_info_topic"],
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            camera["image_topic"],
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Drawing ChArUco axes from {camera['image_topic']} to "
            f"{self.annotated_topic}; axis length {self.axes_length:.3f} m"
        )

    def _on_camera_info(self, message: CameraInfo) -> None:
        matrix = np.asarray(message.k, dtype=float).reshape(3, 3)
        if not np.isfinite(matrix).all() or matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            return
        self.camera_matrix = matrix
        self.dist_coeffs = np.asarray(message.d, dtype=float)

    def _detect_board(self, gray: np.ndarray):
        aruco = cv2.aruco
        if hasattr(aruco, "CharucoDetector"):
            return aruco.CharucoDetector(self.board).detectBoard(gray)

        params = aruco.DetectorParameters_create()
        marker_corners, marker_ids, _ = aruco.detectMarkers(
            gray, self.board.getDictionary(), parameters=params
        )
        if marker_ids is None or len(marker_ids) == 0:
            return None, None, marker_corners, marker_ids
        count, corners, ids = aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, self.board
        )
        return (
            corners if count else None,
            ids if count else None,
            marker_corners,
            marker_ids,
        )

    def _centered_object_points(self, ids: np.ndarray) -> np.ndarray:
        get_corners = getattr(self.board, "getChessboardCorners", None)
        all_points = (
            np.asarray(get_corners(), dtype=np.float64).reshape(-1, 3)
            if callable(get_corners)
            else np.asarray(self.board.chessboardCorners, dtype=np.float64).reshape(-1, 3)
        )
        points = all_points[np.asarray(ids, dtype=int).reshape(-1)].copy()
        points[:, 0] -= int(self.board_config["squares_x"]) * float(
            self.board_config["square_length_m"]
        ) / 2.0
        points[:, 1] -= int(self.board_config["squares_y"]) * float(
            self.board_config["square_length_m"]
        ) / 2.0
        return points

    @staticmethod
    def _draw_label(image: np.ndarray, text: str, position: tuple[int, int], color) -> None:
        cv2.putText(
            image, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.65,
            (0, 0, 0), 4, cv2.LINE_AA,
        )
        cv2.putText(
            image, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.65,
            color, 2, cv2.LINE_AA,
        )

    def _on_image(self, message: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids, marker_corners, marker_ids = self._detect_board(gray)

            if marker_ids is not None and len(marker_ids) > 0:
                for marker in marker_corners:
                    polygon = np.rint(np.asarray(marker).reshape(-1, 2)).astype(np.int32)
                    cv2.polylines(image, [polygon], True, (0, 220, 255), 1, cv2.LINE_AA)

            status = "Waiting for CameraInfo" if self.camera_matrix is None else "No ChArUco board"
            if ids is not None and len(ids) >= 4:
                image_points = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
                for point in image_points:
                    center = tuple(np.rint(point).astype(int))
                    cv2.circle(image, center, 3, (255, 0, 255), -1, cv2.LINE_AA)
                if self.camera_matrix is not None and self.dist_coeffs is not None:
                    object_points = self._centered_object_points(ids)
                    try:
                        # ChArUco corners lie on a plane. IPPE supports a
                        # planar target with four or more non-collinear points;
                        # ITERATIVE's DLT initialization requires six points.
                        solved, rvec, tvec = cv2.solvePnP(
                            object_points,
                            image_points,
                            self.camera_matrix,
                            self.dist_coeffs,
                            flags=cv2.SOLVEPNP_IPPE,
                        )
                        if solved and float(tvec[2, 0]) > 0.0:
                            cv2.drawFrameAxes(
                                image,
                                self.camera_matrix,
                                self.dist_coeffs,
                                rvec,
                                tvec,
                                self.axes_length,
                                3,
                            )
                            status = (
                                f"Board axes | corners: {len(ids)} | "
                                f"length: {self.axes_length:.3f} m"
                            )
                        else:
                            status = "Could not solve board pose"
                    except cv2.error as exc:
                        # Keep publishing the camera frame and status overlay
                        # even if a particular detection cannot produce a pose.
                        status = f"Pose solve failed ({len(ids)} corners)"
                        self.get_logger().warning(
                            f"Could not solve ChArUco pose: {exc}"
                        )
                else:
                    status = f"Detected corners: {len(ids)} | waiting for CameraInfo"
            elif ids is not None:
                status = f"Only {len(ids)} ChArUco corners; need at least 4"

            cv2.rectangle(image, (0, 0), (image.shape[1], 32), (25, 25, 25), -1)
            self._draw_label(image, status, (10, 22), (255, 255, 255))
            legend_y = image.shape[0] - 14
            self._draw_label(image, "+X", (12, legend_y), (0, 0, 255))
            self._draw_label(image, "+Y", (70, legend_y), (0, 255, 0))
            self._draw_label(image, "+Z", (128, legend_y), (255, 0, 0))

            output = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            output.header = message.header
            self.image_publisher.publish(output)
        except (cv2.error, ValueError, IndexError) as exc:
            self.get_logger().warning(f"Could not draw ChArUco axes: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CharucoAxesViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
