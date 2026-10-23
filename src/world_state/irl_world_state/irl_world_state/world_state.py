from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray

from irl_interfaces.msg import GripperState, RobotState, WorldObject, WorldObjectArray, WorldState
from .object_tracker import ObjectTracker, Observation, TrackerConfig


class WorldStateManager(Node):
    def __init__(self) -> None:
        super().__init__("world_state")
        defaults = TrackerConfig()
        self.declare_parameters("", [
            ("robot_state_topic", "/state/robot"),
            ("detections_topic", "/perception/detections"),
            ("object_poses_topic", "/perception/object_poses"),
            ("gripper_state_topic", "/state/gripper"),
            ("world_state_topic", "/state/world"),
            ("objects_topic", "/state/objects"),
            ("perception_objects_topic", "/perception/objects"),
            ("tracking_frame", "base_link"),
            ("max_measurement_age_sec", 0.75),
            ("publish_rate_hz", 10.0),
            ("state_timeout_sec", 1.0),
            *[("tracker_" + name, value) for name, value in vars(defaults).items()],
        ])
        get = lambda name: self.get_parameter(name).value
        self._state_timeout_ns = int(float(get("state_timeout_sec")) * 1e9)
        self._tracking_frame = str(get("tracking_frame")).strip()
        if not self._tracking_frame:
            raise ValueError("tracking_frame must name a fixed tracking coordinate frame")
        self._max_measurement_age = float(get("max_measurement_age_sec"))
        if not math.isfinite(self._max_measurement_age) or self._max_measurement_age <= 0:
            raise ValueError("max_measurement_age_sec must be finite and positive")
        self._tracker = ObjectTracker(TrackerConfig(**{
            name: get("tracker_" + name) for name in vars(defaults)
        }))
        self._last_message_stamp = None
        self._last_measurement_clock = None
        self._robot = RobotState()
        self._detections = Detection2DArray()
        self._object_poses = PoseArray()
        self._gripper = GripperState()
        self._robot_received_at = None
        self._detections_received_at = None
        self._poses_received_at = None
        self._gripper_received_at = None
        self._objects_received_at = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._publisher = self.create_publisher(WorldState, str(get("world_state_topic")), 10)
        self._objects_publisher = self.create_publisher(WorldObjectArray, str(get("objects_topic")), 10)
        self.create_subscription(RobotState, str(get("robot_state_topic")), self._on_robot_state, 10)
        self.create_subscription(Detection2DArray, str(get("detections_topic")), self._on_detections, 10)
        self.create_subscription(PoseArray, str(get("object_poses_topic")), self._on_object_poses, 10)
        self.create_subscription(GripperState, str(get("gripper_state_topic")), self._on_gripper_state, 10)
        self.create_subscription(WorldObjectArray, str(get("perception_objects_topic")), self._on_objects, 10)
        self.create_timer(1.0 / max(0.1, float(get("publish_rate_hz"))), self._publish_world_state)

    def _on_robot_state(self, message):
        self._robot, self._robot_received_at = message, self.get_clock().now()

    def _on_detections(self, message):
        self._detections, self._detections_received_at = message, self.get_clock().now()

    def _on_object_poses(self, message):
        self._object_poses, self._poses_received_at = message, self.get_clock().now()

    def _on_gripper_state(self, message):
        self._gripper, self._gripper_received_at = message, self.get_clock().now()

    def _on_objects(self, message):
        received = self.get_clock().now()
        now = received.nanoseconds / 1e9
        if self._last_measurement_clock is not None and now < self._last_measurement_clock:
            self._last_message_stamp = None
        self._last_measurement_clock = now
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if stamp > 0:
            if (now - stamp > self._max_measurement_age or stamp - now > 0.5
                    or (self._last_message_stamp is not None and stamp <= self._last_message_stamp)):
                return
            self._last_message_stamp = stamp
        observations = []
        for item in message.objects:
            source_frame = item.header.frame_id or message.header.frame_id
            position = quaternion = None
            status = item.pose_status
            if item.pose_valid and source_frame:
                p, q = item.pose.position, item.pose.orientation
                position = np.array([p.x, p.y, p.z])
                quaternion = np.array([q.x, q.y, q.z, q.w])
                if source_frame != self._tracking_frame:
                    try:
                        tf = self._tf_buffer.lookup_transform(
                            self._tracking_frame, source_frame, Time.from_msg(message.header.stamp)
                        ).transform
                        rotation = Rotation.from_quat([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])
                        position = rotation.apply(position) + [tf.translation.x, tf.translation.y, tf.translation.z]
                        quaternion = (rotation * Rotation.from_quat(quaternion)).as_quat()
                    except (TransformException, ValueError) as exc:
                        position = quaternion = None
                        status = "missing_transform"
                        self.get_logger().warning(f"Object tracking TF unavailable: {exc}", throttle_duration_sec=5.)
            elif not source_frame:
                status = "missing_frame"
            box = item.bbox
            cx, cy = box.center.position.x, box.center.position.y
            bbox = np.array([cx - box.size_x / 2, cy - box.size_y / 2,
                             cx + box.size_x / 2, cy + box.size_y / 2])
            observations.append(Observation(item.class_name, item.detection_confidence,
                                            position, quaternion, bbox, item.pose_source, status,
                                            max(0., now - stamp) if stamp > 0 else 0.))
        self._tracker.update(observations, now, self._tracking_frame)
        self._objects_received_at = received

    def _fresh(self, received_at):
        return received_at is not None and 0 <= (self.get_clock().now() - received_at).nanoseconds <= self._state_timeout_ns

    def _tracked_objects(self, now):
        result = WorldObjectArray()
        result.header.stamp = now.to_msg()
        result.header.frame_id = self._tracking_frame
        for snapshot in self._tracker.snapshot(now.nanoseconds / 1e9):
            track = snapshot.track
            item = WorldObject(header=result.header)
            item.object_id, item.class_name = track.object_id, track.label
            item.confidence = track.confidence
            item.detection_confidence = track.detection_confidence
            item.visible, item.pose_valid = snapshot.visible, snapshot.pose_valid
            item.pose_source = track.source
            item.pose_status = track.status
            if not snapshot.visible and track.state is not None:
                item.pose_status = "predicted" if snapshot.pose_valid else "stale_pose"
            item.track_confirmed, item.actionable = track.confirmed, track.actionable
            item.tracking_status = snapshot.tracking_status
            item.observation_age_sec = snapshot.age
            item.observation_count = track.hits
            if track.bbox is not None:
                x1, y1, x2, y2 = track.bbox
                item.bbox.center.position.x, item.bbox.center.position.y = (x1 + x2) / 2, (y1 + y2) / 2
                item.bbox.size_x, item.bbox.size_y = x2 - x1, y2 - y1
            if snapshot.state is not None:
                item.pose.position.x, item.pose.position.y, item.pose.position.z = snapshot.state[:3].tolist()
                item.velocity.x, item.velocity.y, item.velocity.z = snapshot.state[3:].tolist()
                q = item.pose.orientation
                q.x, q.y, q.z, q.w = track.quaternion.tolist()
                item.position_covariance = snapshot.covariance[:3, :3].reshape(-1).tolist()
            else:
                item.position_covariance = [math.inf if i % 4 == 0 else 0. for i in range(9)]
            result.objects.append(item)
        return result

    def _publish_world_state(self):
        now = self.get_clock().now()
        objects = self._tracked_objects(now)
        world = WorldState()
        world.header.stamp = now.to_msg()
        world.header.frame_id = self._tracking_frame
        robot_fresh = self._fresh(self._robot_received_at)
        detections_fresh = self._fresh(self._detections_received_at)
        poses_fresh = self._fresh(self._poses_received_at)
        gripper_fresh = self._fresh(self._gripper_received_at)
        if robot_fresh:
            world.robot = self._robot
        if detections_fresh:
            world.detections = self._detections
        if poses_fresh:
            world.object_poses = self._object_poses
        if gripper_fresh:
            world.gripper = self._gripper
        world.objects = objects.objects
        world.valid = any((robot_fresh, detections_fresh, poses_fresh, gripper_fresh,
                           self._fresh(self._objects_received_at)))
        self._publisher.publish(world)
        self._objects_publisher.publish(objects)
        transforms = []
        for item in objects.objects:
            if not (item.track_confirmed and item.pose_valid):
                continue
            transform = TransformStamped(header=item.header)
            transform.child_frame_id = "tracked_" + item.object_id
            transform.transform.translation.x = item.pose.position.x
            transform.transform.translation.y = item.pose.position.y
            transform.transform.translation.z = item.pose.position.z
            transform.transform.rotation = item.pose.orientation
            transforms.append(transform)
        if transforms:
            self._tf_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = WorldStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
