"""Estimate base_link to camera_link from a stationary ChArUco board."""

from __future__ import annotations

import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener


def _quaternion_from_matrix(rotation: np.ndarray) -> np.ndarray:
    """Return a normalized quaternion in ROS x, y, z, w order."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        q = np.array([
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            q = np.array([
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            ])
        elif axis == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            q = np.array([
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            ])
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            q = np.array([
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ])
    return q / np.linalg.norm(q)


def _rotation_from_quaternion(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    """Markley quaternion average for rotations close to the same pose."""
    quaternions = [_quaternion_from_matrix(r) for r in rotations]
    reference = quaternions[0]
    aligned = [q if np.dot(q, reference) >= 0.0 else -q for q in quaternions]
    accumulator = sum(np.outer(q, q) for q in aligned)
    _, vectors = np.linalg.eigh(accumulator)
    q = vectors[:, -1]
    return _rotation_from_quaternion(q)


def _transform_from_message(message: TransformStamped) -> np.ndarray:
    t = message.transform.translation
    q = message.transform.rotation
    matrix = np.eye(4)
    matrix[:3, :3] = _rotation_from_quaternion(np.array([q.x, q.y, q.z, q.w]))
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def _rotation_from_xyz_degrees(angles: list[float]) -> np.ndarray:
    x, y, z = np.radians(np.asarray(angles, dtype=float))
    rx = np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])
    ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
    rz = np.array([[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]])
    return rz @ ry @ rx


class CharucoExtrinsicCalibration(Node):
    def __init__(self) -> None:
        super().__init__("charuco_extrinsic_calibration")
        default_config = Path(get_package_share_directory("irl_camera_calibration")) / "config/calibration.yaml"
        self.declare_parameter("config_file", str(default_config))
        self.config_path = Path(self.get_parameter("config_file").value).expanduser().resolve()
        with self.config_path.open(encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)

        self.board_config = self._load_board_spec()
        self.board = self._make_board(self.board_config)
        self.dictionary = self.board_config["dictionary"]
        self.bridge = CvBridge()
        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None
        self.camera_optical_frame = ""
        self.last_sample_time = 0.0
        self.samples: list[tuple[np.ndarray, np.ndarray, float]] = []
        self.finished = False
        self.last_status_time = 0.0
        self.last_stability_status_time = 0.0
        self.last_tf_status_time = 0.0

        self.parent_frame = self.config["board_pose"]["parent_frame"]
        self.board_frame = self.config["board_pose"]["child_frame"]
        self.camera_link_frame = self.config["camera"]["link_frame"]
        collection = self.config["collection"]
        self.target_samples = int(collection["sample_count"])
        self.min_corners = int(collection["min_charuco_corners"])
        self.min_inliers = int(collection["min_inlier_samples"])
        self.sample_period = float(collection["sample_period_sec"])
        self.max_reprojection_error = float(collection["max_reprojection_error_px"])
        self.translation_tolerance = float(collection["translation_inlier_tolerance_m"])
        self.rotation_tolerance = math.radians(float(collection["rotation_inlier_tolerance_deg"]))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        camera = self.config["camera"]
        self.create_subscription(Image, camera["image_topic"], self._on_image, 10)
        self.create_subscription(CameraInfo, camera["camera_info_topic"], self._on_camera_info, 10)
        board_pose = self.config["board_pose"]
        xyz = np.asarray(board_pose["translation_mm"], dtype=float) / 1000.0
        if xyz.shape != (3,):
            raise ValueError("board_pose.translation_mm must contain exactly three values")
        self.base_from_board = np.eye(4)
        self.base_from_board[:3, :3] = _rotation_from_xyz_degrees(board_pose["rotation_xyz_deg"])
        self.base_from_board[:3, 3] = xyz
        self.get_logger().info(
            f"Collecting {self.target_samples} ChArUco poses from "
            f"{camera['image_topic']}; board center in {self.parent_frame}: "
            f"({xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}) m"
        )

    def _load_board_spec(self) -> dict:
        spec_path = Path(self.config["board_spec"])
        if not spec_path.is_absolute():
            spec_path = self.config_path.parent / spec_path
        with spec_path.open(encoding="utf-8") as stream:
            board = yaml.safe_load(stream)["board"]
        if board.get("origin", "center") != "center":
            raise ValueError("The ChArUco board origin must be configured at its center")
        if int(board["squares_x"]) < 2 or int(board["squares_y"]) < 2:
            raise ValueError("A ChArUco board needs at least two squares in each direction")
        if not 0.0 < float(board["marker_length_m"]) < float(board["square_length_m"]):
            raise ValueError("marker_length_m must be smaller than square_length_m")
        if getattr(cv2, "aruco", None) is None:
            raise RuntimeError("OpenCV was built without the aruco module; install opencv-contrib-python")
        if not hasattr(cv2.aruco, board["dictionary"]):
            raise ValueError(f"Unknown OpenCV ArUco dictionary: {board['dictionary']}")
        return board

    @staticmethod
    def _make_board(board_config):
        aruco = cv2.aruco
        dictionary_id = getattr(aruco, board_config["dictionary"])
        dictionary = aruco.getPredefinedDictionary(dictionary_id)
        size = (int(board_config["squares_x"]), int(board_config["squares_y"]))
        square = float(board_config["square_length_m"])
        marker = float(board_config["marker_length_m"])
        if hasattr(aruco, "CharucoBoard_create"):
            return aruco.CharucoBoard_create(*size, square, marker, dictionary)
        return aruco.CharucoBoard(size, square, marker, dictionary)

    def _on_camera_info(self, message: CameraInfo) -> None:
        matrix = np.asarray(message.k, dtype=float).reshape(3, 3)
        if not np.isfinite(matrix).all() or matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            self.get_logger().warning("Ignoring invalid CameraInfo intrinsics")
            return
        self.camera_matrix = matrix
        self.dist_coeffs = np.asarray(message.d, dtype=float)
        self.camera_optical_frame = message.header.frame_id

    def _detect_charuco(self, gray: np.ndarray):
        aruco = cv2.aruco
        if hasattr(aruco, "CharucoDetector"):
            detector = aruco.CharucoDetector(self.board)
            corners, ids, _, _ = detector.detectBoard(gray)
            return corners, ids

        detector_params = aruco.DetectorParameters_create()
        marker_corners, marker_ids, _ = aruco.detectMarkers(
            gray, self.board.getDictionary(), parameters=detector_params
        )
        if marker_ids is None or len(marker_ids) == 0:
            return None, None
        count, corners, ids = aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, self.board
        )
        return (corners, ids) if count else (None, None)

    def _centered_object_points(self, ids: np.ndarray) -> np.ndarray:
        corners = getattr(self.board, "getChessboardCorners", None)
        if callable(corners):
            all_points = np.asarray(corners(), dtype=np.float64).reshape(-1, 3)
        else:
            all_points = np.asarray(self.board.chessboardCorners, dtype=np.float64).reshape(-1, 3)
        indices = np.asarray(ids, dtype=int).reshape(-1)
        points = all_points[indices].copy()
        points[:, 0] -= int(self.board_config["squares_x"]) * float(self.board_config["square_length_m"]) / 2.0
        points[:, 1] -= int(self.board_config["squares_y"]) * float(self.board_config["square_length_m"]) / 2.0
        return points

    def _on_image(self, message: Image) -> None:
        if self.finished:
            return
        if self.camera_matrix is None or self.dist_coeffs is None:
            self._periodic_status("Waiting for CameraInfo before estimating camera pose")
            return
        if not self.camera_optical_frame:
            return
        now = time.monotonic()
        if now - self.last_sample_time < self.sample_period:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
            corners, ids = self._detect_charuco(image)
            if ids is None or len(ids) < self.min_corners:
                self._periodic_status("Waiting for more visible ChArUco corners")
                return
            object_points = self._centered_object_points(ids)
            image_points = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
            solved, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not solved:
                return
            projected, _ = cv2.projectPoints(
                object_points, rvec, tvec, self.camera_matrix, self.dist_coeffs
            )
            reprojection_error = float(np.sqrt(np.mean(np.sum(
                (projected.reshape(-1, 2) - image_points) ** 2, axis=1
            ))))
            if reprojection_error > self.max_reprojection_error:
                self._periodic_status(
                    f"Rejecting board pose: reprojection error {reprojection_error:.2f}px"
                )
                return

            camera_from_board = np.eye(4)
            camera_from_board[:3, :3] = cv2.Rodrigues(rvec)[0]
            camera_from_board[:3, 3] = np.asarray(tvec).reshape(3)
            if camera_from_board[2, 3] <= 0.0:
                return
            base_from_camera_optical = self.base_from_board @ np.linalg.inv(camera_from_board)
            self.samples.append((
                base_from_camera_optical[:3, 3].copy(),
                base_from_camera_optical[:3, :3].copy(),
                reprojection_error,
            ))
            self.last_sample_time = now
            if len(self.samples) > self.target_samples:
                self.samples.pop(0)
            self._periodic_status(
                f"Accepted ChArUco sample {len(self.samples)}/{self.target_samples} "
                f"({len(ids)} corners, reprojection {reprojection_error:.2f}px)"
            )
            if len(self.samples) == self.target_samples:
                self._finish_if_stable()
        except (cv2.error, ValueError, IndexError) as exc:
            self._periodic_status(f"Could not estimate this frame: {exc}")

    def _finish_if_stable(self) -> None:
        translations = np.asarray([sample[0] for sample in self.samples])
        rotations = [sample[1] for sample in self.samples]
        median_translation = np.median(translations, axis=0)
        median_rotation = _mean_rotation(rotations)
        translation_errors = np.linalg.norm(translations - median_translation, axis=1)
        rotation_errors = np.array([
            math.acos(float(np.clip((np.trace(median_rotation.T @ rotation) - 1.0) / 2.0, -1.0, 1.0)))
            for rotation in rotations
        ])
        inliers = np.flatnonzero(
            (translation_errors <= self.translation_tolerance)
            & (rotation_errors <= self.rotation_tolerance)
        )
        if len(inliers) < self.min_inliers:
            now = time.monotonic()
            if now - self.last_stability_status_time >= 5.0:
                self.get_logger().warning(
                    f"Pose is not stable yet: {len(inliers)}/{self.min_inliers} "
                    f"inliers; translation error median/max "
                    f"{np.median(translation_errors) * 1000.0:.1f}/"
                    f"{np.max(translation_errors) * 1000.0:.1f} mm, rotation error "
                    f"median/max {np.degrees(np.median(rotation_errors)):.2f}/"
                    f"{np.degrees(np.max(rotation_errors)):.2f} deg. Keep the "
                    "camera and board still and verify the board dimensions."
                )
                self.last_stability_status_time = now
            self.samples.pop(0)
            return

        translation = np.mean(translations[inliers], axis=0)
        rotation = _mean_rotation([rotations[i] for i in inliers])
        try:
            optical_to_link = self.tf_buffer.lookup_transform(
                self.camera_optical_frame,
                self.camera_link_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except Exception as exc:  # TF may arrive slightly after the image stream.
            now = time.monotonic()
            if now - self.last_tf_status_time >= 5.0:
                self.get_logger().warning(
                    f"Waiting for TF {self.camera_link_frame} -> "
                    f"{self.camera_optical_frame}: {exc}"
                )
                self.last_tf_status_time = now
            return

        base_from_optical = np.eye(4)
        base_from_optical[:3, :3] = rotation
        base_from_optical[:3, 3] = translation
        base_from_link = base_from_optical @ _transform_from_message(optical_to_link)
        inlier_errors = [self.samples[i][2] for i in inliers]
        self._save_and_publish(
            base_from_link, base_from_optical, len(inliers), inlier_errors
        )

    def _save_and_publish(
        self,
        base_from_link: np.ndarray,
        base_from_optical: np.ndarray,
        inlier_count: int,
        inlier_errors: list[float],
    ) -> None:
        output = Path(self.config["output_file"]).expanduser()
        if not output.is_absolute():
            output = self.config_path.parent / output
        output.parent.mkdir(parents=True, exist_ok=True)
        t = base_from_link[:3, 3]
        q = _quaternion_from_matrix(base_from_link[:3, :3])
        optical_t = base_from_optical[:3, 3]
        optical_q = _quaternion_from_matrix(base_from_optical[:3, :3])
        result = {
            "parent_frame": self.parent_frame,
            "child_frame": self.camera_link_frame,
            "translation_m": [float(v) for v in t],
            "rotation_xyzw": [float(v) for v in q],
            "optical_frame": self.camera_optical_frame,
            "base_to_optical": {
                "translation_m": [float(v) for v in optical_t],
                "rotation_xyzw": [float(v) for v in optical_q],
            },
            "board_spec": self.board_config,
            "board_center_pose": self.config["board_pose"],
            "accepted_samples": inlier_count,
            "mean_reprojection_error_px": float(np.mean(inlier_errors)),
        }
        temporary_output = output.with_suffix(output.suffix + ".tmp")
        temporary_output.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
        temporary_output.replace(output)
        self.get_logger().info(f"Saved camera extrinsic calibration to {output}")

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.camera_link_frame
        transform.transform.translation.x = float(t[0])
        transform.transform.translation.y = float(t[1])
        transform.transform.translation.z = float(t[2])
        transform.transform.rotation.x = float(q[0])
        transform.transform.rotation.y = float(q[1])
        transform.transform.rotation.z = float(q[2])
        transform.transform.rotation.w = float(q[3])
        if bool(self.config.get("publish_static_transform", True)):
            board_transform = TransformStamped()
            board_transform.header.stamp = transform.header.stamp
            board_transform.header.frame_id = self.parent_frame
            board_transform.child_frame_id = self.board_frame
            board_translation = self.base_from_board[:3, 3]
            board_rotation = _quaternion_from_matrix(self.base_from_board[:3, :3])
            board_transform.transform.translation.x = float(board_translation[0])
            board_transform.transform.translation.y = float(board_translation[1])
            board_transform.transform.translation.z = float(board_translation[2])
            board_transform.transform.rotation.x = float(board_rotation[0])
            board_transform.transform.rotation.y = float(board_rotation[1])
            board_transform.transform.rotation.z = float(board_rotation[2])
            board_transform.transform.rotation.w = float(board_rotation[3])
            self.tf_broadcaster.sendTransform([board_transform, transform])
            self.get_logger().info(
                f"Published static TF {self.parent_frame} -> {self.board_frame} and "
                f"{self.parent_frame} -> {self.camera_link_frame}"
            )
        self.finished = True

    def _periodic_status(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_status_time >= 5.0:
            self.get_logger().info(message)
            self.last_status_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CharucoExtrinsicCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
