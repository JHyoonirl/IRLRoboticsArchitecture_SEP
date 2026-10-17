"""YOLOE masks to depth-based object frames and rule-based grasp poses."""

from __future__ import annotations

import re

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from .foundationpose_adapter import FoundationPoseAdapter
from .grasp_pipeline import GraspPipelineConfig, RuleBasedGraspPipeline
from .geometry import quaternion_from_matrix
from .pose_estimator import ObjectPoseEstimator
from .segmenter import Segment, YoloESegmenter
from .visualization import draw_overlay


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_node")
        self.declare_parameters("", [
            ("color_topic", "/camera/color/image_raw"),
            ("depth_topic", "/camera/aligned_depth_to_color/image_raw"),
            ("camera_info_topic", "/camera/color/camera_info"),
            ("camera_mode", "rgbd"),
            ("weights", "/workspace/jhyoon_external/models/yoloe-26m-seg-pf.pt"),
            ("segmentation_prompt", ""),
            ("confidence", 0.25),
            ("depth_scale", 0.001),
            ("max_depth", 3.0),
            ("up_axis", [0.0, -1.0, 0.0]),
            ("sync_slop", 0.05),
            ("pose_backend", "auto"),
            ("foundationpose_root", "/workspace/jhyoon_external/FoundationPose"),
            ("foundationpose_meshes", ""),
            ("foundationpose_mesh_scales", ""),
            ("foundationpose_register_iterations", 5),
            ("foundationpose_track_iterations", 2),
            ("foundationpose_debug", 0),
            ("foundationpose_debug_dir", "/workspace/test/foundationpose_debug"),
            ("show_visualization", False),
            ("visualization_target", "can"),
            ("enable_rule_grasp", True),
            ("grasp_target", "can"),
            ("mask_erosion_pixels", 3),
            ("min_depth_points", 30),
            ("table_ransac_enabled", True),
            ("table_distance_threshold", 0.008),
            ("table_ransac_iterations", 100),
            ("table_min_inliers", 100),
            ("table_normal_tolerance_degrees", 30.0),
            ("gripper_min_width", 0.0),
            ("gripper_max_width", 0.085),
            ("grasp_width_clearance", 0.005),
            ("grasp_offset", 0.0),
            ("pregrasp_offset", 0.10),
            ("table_clearance", 0.005),
        ])
        get = lambda name: self.get_parameter(name).value
        self.bridge = CvBridge()
        self.segmenter = YoloESegmenter(
            str(get("weights")),
            float(get("confidence")),
            text_prompt=str(get("segmentation_prompt")),
        )
        self.depth_scale = float(get("depth_scale"))
        self.max_depth = float(get("max_depth"))
        self.camera_mode = str(get("camera_mode")).strip().lower()
        if self.camera_mode not in {"rgb", "rgbd"}:
            raise ValueError("camera_mode must be either 'rgb' or 'rgbd'")
        self.up_axis = tuple(float(value) for value in get("up_axis"))
        self.show_visualization = bool(get("show_visualization"))
        self.visualization_target = str(get("visualization_target")).strip().lower()
        self.enable_rule_grasp = bool(get("enable_rule_grasp"))
        self.grasp_target = str(get("grasp_target")).strip().lower()
        self.window_name = "YOLOE + RGB-D rule grasp"
        if self.show_visualization:
            try:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, 960, 720)
            except cv2.error as exc:
                self.show_visualization = False
                self.get_logger().error(f"Cannot open visualization window: {exc}")
        self.intrinsics: tuple[float, float, float, float] | None = None
        self.foundationpose = FoundationPoseAdapter(
            backend=str(get("pose_backend")),
            root=str(get("foundationpose_root")),
            mesh_specs=str(get("foundationpose_meshes")),
            mesh_scales=str(get("foundationpose_mesh_scales")),
            register_iterations=int(get("foundationpose_register_iterations")),
            track_iterations=int(get("foundationpose_track_iterations")),
            debug=int(get("foundationpose_debug")),
            debug_dir=str(get("foundationpose_debug_dir")),
            logger=self.get_logger(),
        )
        self.pose_estimator = ObjectPoseEstimator(
            self.foundationpose,
            depth_scale=self.depth_scale,
            max_depth=self.max_depth,
            up_axis=self.up_axis,
        )
        self.grasp_pipeline = RuleBasedGraspPipeline(GraspPipelineConfig(
            depth_scale=self.depth_scale,
            max_depth=self.max_depth,
            mask_erosion_pixels=int(get("mask_erosion_pixels")),
            min_points=int(get("min_depth_points")),
            table_ransac_enabled=bool(get("table_ransac_enabled")),
            table_distance_threshold=float(get("table_distance_threshold")),
            table_ransac_iterations=int(get("table_ransac_iterations")),
            table_min_inliers=int(get("table_min_inliers")),
            table_normal_tolerance_degrees=float(get("table_normal_tolerance_degrees")),
            gripper_min_width=float(get("gripper_min_width")),
            gripper_max_width=float(get("gripper_max_width")),
            grasp_width_clearance=float(get("grasp_width_clearance")),
            grasp_offset=float(get("grasp_offset")),
            pregrasp_offset=float(get("pregrasp_offset")),
            table_clearance=float(get("table_clearance")),
        ))

        self.detection_pub = self.create_publisher(Detection2DArray, "detections", 10)
        self.mask_pub = self.create_publisher(Image, "instance_mask", 10)
        self.pose_pub = self.create_publisher(PoseArray, "object_poses", 10)
        self.grasp_candidates_pub = self.create_publisher(PoseArray, "grasp_candidates", 10)
        self.best_grasp_pub = self.create_publisher(PoseStamped, "best_grasp", 10)
        self.debug_image_pub = self.create_publisher(Image, "debug_image", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(CameraInfo, str(get("camera_info_topic")), self.camera_info_callback, 10)

        color_topic = str(get("color_topic"))
        self._sync = None
        if self.camera_mode == "rgbd":
            color_sub = message_filters.Subscriber(self, Image, color_topic)
            depth_sub = message_filters.Subscriber(self, Image, str(get("depth_topic")))
            sync = message_filters.ApproximateTimeSynchronizer(
                [color_sub, depth_sub], queue_size=5, slop=float(get("sync_slop"))
            )
            sync.registerCallback(self.image_callback)
            self._sync = sync
        else:
            self._color_sub = self.create_subscription(
                Image, color_topic, self.color_only_callback, 10
            )
        self._camera_info_warned = False

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def image_callback(self, color_msg: Image, depth_msg: Image) -> None:
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        segments = self.segmenter.segment(color)
        self.publish_2d_outputs(color_msg, segments)
        if self.intrinsics is None:
            if not self._camera_info_warned:
                self.get_logger().warning("Waiting for CameraInfo; masks are available but 3D poses are not")
                self._camera_info_warned = True
            self.update_visualization(color_msg.header, color, segments, {})
            return
        self._camera_info_warned = False
        if depth.shape != color.shape[:2]:
            self.get_logger().error("Depth must be aligned to the color image")
            self.update_visualization(color_msg.header, color, segments, {})
            return
        pose_visuals = self.publish_3d_outputs(color_msg, color, depth, segments)
        self.update_visualization(color_msg.header, color, segments, pose_visuals)

    def color_only_callback(self, color_msg: Image) -> None:
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        segments = self.segmenter.segment(color)
        self.publish_2d_outputs(color_msg, segments)
        self.update_visualization(color_msg.header, color, segments, {})

    def update_visualization(
        self,
        header,
        color: np.ndarray,
        segments: list[Segment],
        poses: dict[int, tuple[np.ndarray, np.ndarray | None, str]],
    ) -> None:
        camera_matrix = None
        if self.intrinsics is not None:
            fx, fy, cx, cy = self.intrinsics
            camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
        try:
            frame = draw_overlay(color, segments, poses, camera_matrix, self.visualization_target)
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
            if not self.show_visualization:
                return
            cv2.imshow(self.window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyWindow(self.window_name)
                self.show_visualization = False
        except cv2.error as exc:
            self.show_visualization = False
            self.get_logger().error(f"Visualization stopped: {exc}")

    def publish_2d_outputs(self, msg: Image, segments: list[Segment]) -> None:
        detections = Detection2DArray(header=msg.header)
        instance_mask = np.zeros((msg.height, msg.width), dtype=np.uint16)
        for index, item in enumerate(segments, start=1):
            x1, y1, x2, y2 = item.bbox
            detection = Detection2D()
            detection.bbox.center.position.x = (x1 + x2) / 2.0
            detection.bbox.center.position.y = (y1 + y2) / 2.0
            detection.bbox.size_x, detection.bbox.size_y = x2 - x1, y2 - y1
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = item.label
            hypothesis.hypothesis.score = item.score
            detection.results.append(hypothesis)
            detections.detections.append(detection)
            instance_mask[item.mask] = index
        self.detection_pub.publish(detections)
        mask_msg = self.bridge.cv2_to_imgmsg(instance_mask, encoding="mono16")
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

    def publish_3d_outputs(
        self, msg: Image, color: np.ndarray, depth: np.ndarray, segments: list[Segment]
    ) -> dict[int, tuple[np.ndarray, np.ndarray | None, str]]:
        poses = PoseArray(header=msg.header)
        grasp_candidates = PoseArray(header=msg.header)
        best_grasp: PoseStamped | None = None
        best_grasp_score = -np.inf
        transforms: list[TransformStamped] = []
        pose_visuals: dict[int, tuple[np.ndarray, np.ndarray | None, str]] = {}
        fx, fy, cx, cy = self.intrinsics
        camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
        rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        depth_m = depth.astype(np.float32) * self.depth_scale
        # Estimate the table normal from the aligned RGB-D point cloud.  The
        # configured up_axis is only a fallback for scenes where RANSAC fails.
        # A full-scene plane fit is relatively expensive.  Do not block the
        # image/debug stream when YOLOE has no object to estimate.
        table_plane = None
        if segments:
            table_plane = self.grasp_pipeline.estimate_table_plane(
                depth, self.intrinsics
            )
        pose_up_axis = self.up_axis
        if table_plane is not None:
            pose_up_axis = tuple(float(value) for value in table_plane.normal)
        for index, item in enumerate(segments):
            center = None
            quaternion = None
            rotation = None
            source = ""
            pose_estimate = self.pose_estimator.estimate(
                item.label,
                rgb,
                depth_m,
                camera_matrix,
                depth,
                self.intrinsics,
                item.mask,
                up_axis=pose_up_axis,
            )
            if pose_estimate is not None:
                center = pose_estimate.center
                quaternion = pose_estimate.quaternion
                rotation = pose_estimate.rotation
                source = pose_estimate.source
            if self.enable_rule_grasp and (
                not self.grasp_target or item.label.casefold() == self.grasp_target
            ):
                plan = self.grasp_pipeline.process(
                    item.mask,
                    depth,
                    self.intrinsics,
                    pose_up_axis,
                    table_plane=table_plane,
                )
                for candidate in plan.candidates:
                    if candidate.valid:
                        grasp_pose = Pose()
                        grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z = (
                            candidate.position.tolist()
                        )
                        grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w = (
                            quaternion_from_matrix(candidate.rotation).tolist()
                        )
                        grasp_candidates.poses.append(grasp_pose)
                if plan.selected is not None and plan.selected.score > best_grasp_score:
                    best_grasp_score = plan.selected.score
                    best_grasp = PoseStamped(header=msg.header)
                    best_grasp.pose.position.x, best_grasp.pose.position.y, best_grasp.pose.position.z = (
                        plan.selected.position.tolist()
                    )
                    best_grasp.pose.orientation.x, best_grasp.pose.orientation.y, best_grasp.pose.orientation.z, best_grasp.pose.orientation.w = (
                        quaternion_from_matrix(plan.selected.rotation).tolist()
                    )
            if center is None or quaternion is None:
                continue
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = center.tolist()
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion.tolist()
            poses.poses.append(pose)
            pose_visuals[index] = (center.copy(), rotation, source)

            transform = TransformStamped(header=msg.header)
            label = re.sub(r"[^A-Za-z0-9_]", "_", item.label)
            transform.child_frame_id = f"object_{label}_{index}"
            transform.transform.translation.x = pose.position.x
            transform.transform.translation.y = pose.position.y
            transform.transform.translation.z = pose.position.z
            transform.transform.rotation = pose.orientation
            transforms.append(transform)
        self.pose_pub.publish(poses)
        self.grasp_candidates_pub.publish(grasp_candidates)
        if best_grasp is not None:
            self.best_grasp_pub.publish(best_grasp)
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)
        return pose_visuals

    def destroy_node(self) -> bool:
        if self.show_visualization:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
