"""Convert the shared World State into Foxglove-friendly visualization topics."""

from __future__ import annotations

import json

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from irl_interfaces.msg import WorldObject, WorldState
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def _rotate_vector(quaternion, vector: tuple[float, float, float]) -> tuple[float, float, float]:
    """Rotate a vector by an xyzw quaternion without adding tf dependencies."""
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    vx, vy, vz = vector
    qx = w * vx + y * vz - z * vy
    qy = w * vy + z * vx - x * vz
    qz = w * vz + x * vy - y * vx
    qw = -x * vx - y * vy - z * vz
    return (
        qx * w + qw * -x + qy * -z - qz * -y,
        qy * w + qw * -y + qz * -x - qx * -z,
        qz * w + qw * -z + qx * -y - qy * -x,
    )


def _point(x: float, y: float, z: float) -> Point:
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def _color_for_object(item: WorldObject) -> tuple[float, float, float, float]:
    if not item.visible:
        return 0.45, 0.45, 0.45, 0.65
    confidence = max(0.0, min(1.0, float(item.confidence)))
    return 1.0 - confidence, confidence, 0.15, 0.9


class WorldStateVisualizer(Node):
    """Publish 3D object/robot markers and machine-readable state summaries."""

    def __init__(self) -> None:
        super().__init__("world_state_visualizer")
        self.declare_parameters("", [
            ("world_state_topic", "/state/world"),
            ("marker_topic", "/visualization/world_state_markers"),
            ("summary_topic", "/visualization/world_state_summary"),
            ("diagnostics_topic", "/visualization/world_state_diagnostics"),
            ("object_axis_length", 0.12),
            ("marker_lifetime_sec", 0.5),
        ])
        get = lambda name: self.get_parameter(name).value
        self._axis_length = float(get("object_axis_length"))
        self._lifetime = float(get("marker_lifetime_sec"))
        self._marker_pub = self.create_publisher(MarkerArray, str(get("marker_topic")), 10)
        self._summary_pub = self.create_publisher(String, str(get("summary_topic")), 10)
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, str(get("diagnostics_topic")), 10
        )
        self.create_subscription(
            WorldState, str(get("world_state_topic")), self._on_world_state, 10
        )

    def _marker_base(self, namespace: str, marker_id: int, header, marker_type: int) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.lifetime.sec = int(self._lifetime)
        marker.lifetime.nanosec = int((self._lifetime % 1.0) * 1e9)
        return marker

    def _object_markers(self, item: WorldObject, index: int) -> list[Marker]:
        header = item.header
        if not header.frame_id or not item.pose_valid:
            return []
        position = item.pose.position
        orientation = item.pose.orientation
        color = _color_for_object(item)
        markers: list[Marker] = []
        base = index * 10

        body = self._marker_base("world_objects", base, header, Marker.SPHERE)
        body.pose = item.pose
        body.scale.x = 0.075
        body.scale.y = 0.075
        body.scale.z = 0.075
        body.color.r, body.color.g, body.color.b, body.color.a = color
        markers.append(body)

        for axis_index, axis_color, axis in (
            (1, (1.0, 0.0, 0.0, 1.0), (self._axis_length, 0.0, 0.0)),
            (2, (0.0, 1.0, 0.0, 1.0), (0.0, self._axis_length, 0.0)),
            (3, (0.0, 0.0, 1.0, 1.0), (0.0, 0.0, self._axis_length)),
        ):
            endpoint = _rotate_vector(orientation, axis)
            axis_marker = self._marker_base(
                "world_object_axes", base + axis_index, header, Marker.ARROW
            )
            axis_marker.points = [
                _point(position.x, position.y, position.z),
                _point(
                    position.x + endpoint[0],
                    position.y + endpoint[1],
                    position.z + endpoint[2],
                ),
            ]
            axis_marker.scale.x = 0.012
            axis_marker.scale.y = 0.025
            axis_marker.scale.z = 0.025
            axis_marker.color.r, axis_marker.color.g, axis_marker.color.b, axis_marker.color.a = axis_color
            markers.append(axis_marker)

        label = self._marker_base(
            "world_object_labels", base + 4, header, Marker.TEXT_VIEW_FACING
        )
        label.pose.position.x = position.x
        label.pose.position.y = position.y
        label.pose.position.z = position.z + self._axis_length
        label.scale.z = 0.045
        label.color.r = label.color.g = label.color.b = 1.0
        label.color.a = 1.0
        pose_status = "pose=valid" if item.pose_valid else "pose=unknown"
        label.text = (
            f"{item.object_id} | {item.class_name or 'unknown'}\n"
            f"conf={item.confidence:.2f} | {pose_status}"
        )
        markers.append(label)
        return markers

    def _robot_marker(self, world: WorldState) -> Marker | None:
        if not world.robot.valid or not world.robot.header.frame_id:
            return None
        marker = self._marker_base("robot_state", 0, world.robot.header, Marker.ARROW)
        marker.pose = world.robot.pose.pose
        marker.scale.x = 0.35
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.9
        return marker

    def _publish_summary(self, world: WorldState) -> None:
        objects = [
            {
                "id": item.object_id,
                "class": item.class_name,
                "confidence": round(float(item.confidence), 3),
                "visible": bool(item.visible),
                "pose_valid": bool(item.pose_valid),
            }
            for item in world.objects
        ]
        summary = {
            "valid": bool(world.valid),
            "robot": {
                "valid": bool(world.robot.valid),
                "status": world.robot.system_status,
                "joint_count": len(world.robot.joint_position),
            },
            "objects": objects,
            "gripper": {
                "valid": bool(world.gripper.valid),
                "occupied": bool(world.gripper.occupied),
                "held_object_id": world.gripper.held_object_id,
                "opening_width": round(float(world.gripper.opening_width), 4),
            },
        }
        message = String()
        message.data = json.dumps(summary, ensure_ascii=False, separators=(",", ":"))
        self._summary_pub.publish(message)

    def _publish_diagnostics(self, world: WorldState) -> None:
        diagnostic = DiagnosticArray()
        diagnostic.header = world.header
        status = DiagnosticStatus()
        status.name = "world_state"
        status.level = DiagnosticStatus.OK if world.valid else DiagnosticStatus.WARN
        status.message = "World State valid" if world.valid else "World State has no fresh input"
        values = {
            "robot_valid": str(bool(world.robot.valid)),
            "object_count": str(len(world.objects)),
            "pose_valid_count": str(sum(item.pose_valid for item in world.objects)),
            "gripper_valid": str(bool(world.gripper.valid)),
            "gripper_occupied": str(bool(world.gripper.occupied)),
        }
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        diagnostic.status.append(status)
        self._diagnostics_pub.publish(diagnostic)

    def _on_world_state(self, world: WorldState) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        robot_marker = self._robot_marker(world)
        if robot_marker is not None:
            markers.markers.append(robot_marker)
        for index, item in enumerate(world.objects):
            markers.markers.extend(self._object_markers(item, index))
        self._marker_pub.publish(markers)
        self._publish_summary(world)
        self._publish_diagnostics(world)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldStateVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
