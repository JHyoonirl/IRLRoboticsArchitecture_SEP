from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

from irl_interfaces.msg import GripperState, RobotState, WorldObject, WorldState


def build_world_objects(
    detections: Detection2DArray,
    object_poses: PoseArray,
    detections_fresh: bool,
    poses_fresh: bool,
) -> list[WorldObject]:
    """Build the skill-facing object view without adding task-specific facts."""
    objects: list[WorldObject] = []
    detection_items = detections.detections if detections_fresh else []
    pose_items = object_poses.poses if poses_fresh else []
    count = max(len(detection_items), len(pose_items))
    for index in range(count):
        item = WorldObject()
        item.header = detections.header if detections_fresh else object_poses.header
        item.object_id = f"object_{index}"
        if index < len(detection_items) and detection_items[index].results:
            result = detection_items[index].results[0]
            item.class_name = result.hypothesis.class_id
            item.confidence = float(result.hypothesis.score)
            item.visible = True
            item.object_id = f"{item.class_name or 'object'}_{index}"
        if index < len(pose_items):
            item.pose = pose_items[index]
            item.pose_valid = True
        objects.append(item)
    return objects


class WorldStateManager(Node):
    def __init__(self) -> None:
        super().__init__("world_state")
        self.declare_parameters("", [
            ("robot_state_topic", "/state/robot"),
            ("detections_topic", "/perception/detections"),
            ("object_poses_topic", "/perception/object_poses"),
            ("gripper_state_topic", "/state/gripper"),
            ("world_state_topic", "/state/world"),
            ("publish_rate_hz", 10.0),
            ("state_timeout_sec", 1.0),
        ])
        get = lambda name: self.get_parameter(name).value
        self._state_timeout_ns = int(float(get("state_timeout_sec")) * 1e9)
        self._robot = RobotState()
        self._detections = Detection2DArray()
        self._object_poses = PoseArray()
        self._gripper = GripperState()
        self._robot_received = False
        self._detections_received = False
        self._poses_received = False
        self._gripper_received = False
        self._robot_received_at = None
        self._detections_received_at = None
        self._poses_received_at = None
        self._gripper_received_at = None

        self._publisher = self.create_publisher(WorldState, str(get("world_state_topic")), 10)
        self.create_subscription(
            RobotState, str(get("robot_state_topic")), self._on_robot_state, 10
        )
        self.create_subscription(
            Detection2DArray, str(get("detections_topic")), self._on_detections, 10
        )
        self.create_subscription(
            PoseArray, str(get("object_poses_topic")), self._on_object_poses, 10
        )
        self.create_subscription(
            GripperState, str(get("gripper_state_topic")), self._on_gripper_state, 10
        )
        rate = max(0.1, float(get("publish_rate_hz")))
        self.create_timer(1.0 / rate, self._publish_world_state)

    def _on_robot_state(self, message: RobotState) -> None:
        self._robot = message
        self._robot_received = True
        self._robot_received_at = self.get_clock().now()

    def _on_detections(self, message: Detection2DArray) -> None:
        self._detections = message
        self._detections_received = True
        self._detections_received_at = self.get_clock().now()

    def _on_object_poses(self, message: PoseArray) -> None:
        self._object_poses = message
        self._poses_received = True
        self._poses_received_at = self.get_clock().now()

    def _on_gripper_state(self, message: GripperState) -> None:
        self._gripper = message
        self._gripper_received = True
        self._gripper_received_at = self.get_clock().now()

    def _fresh(self, received: bool, received_at) -> bool:
        if not received or received_at is None:
            return False
        age_ns = (self.get_clock().now() - received_at).nanoseconds
        return age_ns <= self._state_timeout_ns

    def _publish_world_state(self) -> None:
        world = WorldState()
        world.header.stamp = self.get_clock().now().to_msg()
        world.header.frame_id = self._robot.header.frame_id if self._robot_received else "map"
        robot_fresh = self._fresh(self._robot_received, self._robot_received_at)
        detections_fresh = self._fresh(self._detections_received, self._detections_received_at)
        poses_fresh = self._fresh(self._poses_received, self._poses_received_at)
        gripper_fresh = self._fresh(self._gripper_received, self._gripper_received_at)
        if robot_fresh:
            world.robot = self._robot
        if detections_fresh:
            world.detections = self._detections
        if poses_fresh:
            world.object_poses = self._object_poses
        world.objects = build_world_objects(
            self._detections,
            self._object_poses,
            detections_fresh,
            poses_fresh,
        )
        if gripper_fresh:
            world.gripper = self._gripper
        world.valid = any((robot_fresh, detections_fresh, poses_fresh, gripper_fresh))
        self._publisher.publish(world)


def main(args=None) -> None:
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
