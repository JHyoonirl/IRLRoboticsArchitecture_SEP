"""Publish the fixed camera mounting pose saved by calibration."""

from __future__ import annotations

import math
from pathlib import Path

import yaml


def load_camera_extrinsics(path: str) -> dict:
    """Validate a saved calibration without estimating a new camera pose."""
    data = yaml.safe_load(Path(path).expanduser().read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("camera extrinsics must be a YAML mapping")
    for key in ("parent_frame", "child_frame"):
        if not isinstance(data.get(key), str) or not data[key].strip():
            raise ValueError(f"{key} must be a non-empty frame name")
    if data["parent_frame"] == data["child_frame"]:
        raise ValueError("camera parent and child frames must differ")
    for key, length in (("translation_m", 3), ("rotation_xyzw", 4)):
        values = data.get(key)
        if not isinstance(values, list) or len(values) != length:
            raise ValueError(f"{key} must contain {length} numbers")
        data[key] = [float(value) for value in values]
        if not all(math.isfinite(value) for value in data[key]):
            raise ValueError(f"{key} must contain finite numbers")
    norm = math.sqrt(sum(value * value for value in data["rotation_xyzw"]))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("camera rotation quaternion must be non-zero and finite")
    data["rotation_xyzw"] = [value / norm for value in data["rotation_xyzw"]]
    return data


def main(args=None) -> None:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from tf2_ros import StaticTransformBroadcaster

    rclpy.init(args=args)
    node = Node("camera_tf")
    try:
        default_file = str(Path(get_package_share_directory("irl_camera_calibration"))
                           / "config" / "camera_extrinsics.yaml")
        node.declare_parameter("camera_extrinsics_file", default_file)
        path = node.get_parameter("camera_extrinsics_file").value
        data = load_camera_extrinsics(path)
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = data["parent_frame"]
        transform.child_frame_id = data["child_frame"]
        t = transform.transform.translation
        t.x, t.y, t.z = data["translation_m"]
        q = transform.transform.rotation
        q.x, q.y, q.z, q.w = data["rotation_xyzw"]
        broadcaster = StaticTransformBroadcaster(node)
        broadcaster.sendTransform(transform)
        node.get_logger().info(
            f"Loaded fixed camera TF {data['parent_frame']} -> {data['child_frame']} "
            f"from {path}; restart this node after changing the calibration file"
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().fatal(f"Cannot publish camera TF: {exc}")
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
