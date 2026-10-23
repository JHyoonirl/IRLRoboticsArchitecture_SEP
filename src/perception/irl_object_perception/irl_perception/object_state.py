"""Keep each segmented object's identity and pose result in one record."""

from irl_interfaces.msg import WorldObject


def make_object_state(header, index, segment, pose=None, source="", status="pose_unavailable"):
    item = WorldObject()
    item.header = header
    item.object_id = f"{segment.label}_{index}"
    item.class_name = segment.label
    item.confidence = float(segment.score)
    item.detection_confidence = float(segment.score)
    x1, y1, x2, y2 = segment.bbox
    item.bbox.center.position.x = float((x1 + x2) / 2)
    item.bbox.center.position.y = float((y1 + y2) / 2)
    item.bbox.size_x = float(x2 - x1)
    item.bbox.size_y = float(y2 - y1)
    item.visible = True
    item.pose_source = source
    item.pose_status = status
    if pose is not None:
        item.pose = pose
        item.pose_valid = True
        item.pose_status = "ok"
    return item
