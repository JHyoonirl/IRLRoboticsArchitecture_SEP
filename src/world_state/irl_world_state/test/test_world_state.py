from geometry_msgs.msg import Pose, PoseArray
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from irl_world_state.world_state import build_world_objects


def test_build_world_objects_combines_detection_and_pose():
    detections = Detection2DArray()
    detection = Detection2D()
    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = "tool"
    result.hypothesis.score = 0.9
    detection.results.append(result)
    detections.detections.append(detection)

    poses = PoseArray()
    poses.poses.append(Pose())

    objects = build_world_objects(detections, poses, True, True)

    assert len(objects) == 1
    assert objects[0].object_id == "tool_0"
    assert objects[0].class_name == "tool"
    assert objects[0].confidence == 0.9
    assert objects[0].visible
    assert objects[0].pose_valid
