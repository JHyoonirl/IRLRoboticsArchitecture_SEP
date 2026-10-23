from types import SimpleNamespace

import numpy as np
from sensor_msgs.msg import Image

from irl_perception.perception_node import PerceptionNode
from irl_perception.pose_estimator import ObjectPoseEstimator, PoseEstimate
from irl_perception.segmenter import Segment
from irl_perception.visualization import draw_overlay


class Capture:
    def publish(self, message):
        self.message = message


def test_pose_failure_keeps_identity_and_does_not_shift_other_objects():
    labels = ["rigid_object", "black glove", "cup"]
    mask = np.ones((12, 12), dtype=bool)
    segments = [Segment(label, 0.8, (0, 0, 12, 12), mask) for label in labels]
    attempted = []

    def estimate(label, *args, **kwargs):
        attempted.append(label)
        if label == "black glove":
            return None
        return PoseEstimate(np.array([0., 0., float(labels.index(label) + 1)]),
                            np.array([0., 0., 0., 1.]), np.eye(3), "PCA")

    def reject_raw_tf(transforms):
        raise AssertionError("Frame-local detection indices must never be published as object TF")

    node = SimpleNamespace(
        intrinsics=(100., 100., 6., 6.), depth_scale=0.001, up_axis=(0., 0., 1.),
        grasp_pipeline=SimpleNamespace(estimate_table_plane=lambda *args: None),
        pose_estimator=SimpleNamespace(estimate=estimate), enable_rule_grasp=False,
        pose_pub=Capture(), objects_pub=Capture(), grasp_candidates_pub=Capture(),
        tf_broadcaster=SimpleNamespace(sendTransform=reject_raw_tf),
    )
    msg = Image()
    msg.header.frame_id = "camera_color_optical_frame"
    visuals = PerceptionNode.publish_3d_outputs(
        node, msg, np.zeros((12, 12, 3), np.uint8),
        np.full((12, 12), 600, np.uint16), segments,
    )
    assert attempted == labels
    objects = node.objects_pub.message.objects
    assert [item.class_name for item in objects] == labels
    assert [item.pose_valid for item in objects] == [True, False, True]
    assert objects[1].object_id == "black glove_1"
    assert objects[1].pose_status == "pose_unavailable"
    assert objects[2].pose.position.z == 3.
    assert objects[2].pose_source == "PCA"
    assert set(visuals) == {0, 2}


def test_depth_pca_defines_pose_for_non_target_class():
    foundationpose = SimpleNamespace(estimate=lambda *args: None, mesh_paths={})
    estimator = ObjectPoseEstimator(foundationpose, depth_scale=0.001,
                                    max_depth=3., up_axis=(0., 0., 1.))
    depth = np.full((20, 30), 600, np.uint16)
    result = estimator.estimate(
        "black glove", np.zeros((20, 30, 3), np.uint8), depth * 0.001,
        np.eye(3), depth, (100., 100., 15., 10.), np.ones(depth.shape, bool),
    )
    assert result is not None
    assert result.source == "PCA"
    assert np.isclose(result.center[2], 0.6)
    assert np.isclose(np.linalg.norm(result.quaternion), 1.)
    assert np.allclose(result.rotation.T @ result.rotation, np.eye(3))


def test_visualization_draws_axes_for_every_valid_pose(monkeypatch):
    drawn = []
    monkeypatch.setattr("irl_perception.visualization._draw_axes", lambda *args: drawn.append(args))
    image = np.zeros((60, 60, 3), np.uint8)
    segments = [Segment(name, 0.8, (0, 0, 30, 30), np.ones((60, 60), bool))
                for name in ("rigid_object", "black glove")]
    poses = {index: (np.array([0., 0., 0.6]), np.eye(3), "PCA") for index in range(2)}
    draw_overlay(image, segments, poses, np.eye(3), "rigid_object")
    assert len(drawn) == 2


def test_missing_camera_info_keeps_all_segments_with_explicit_invalid_pose():
    node = SimpleNamespace(objects_pub=Capture(), pose_pub=Capture())
    msg = Image()
    segments = [Segment(name, 0.8, (0, 0, 1, 1), np.ones((1, 1), bool))
                for name in ("rigid_object", "black glove")]
    PerceptionNode.publish_unposed_objects(node, msg, segments, "missing_camera_info")
    assert len(node.objects_pub.message.objects) == 2
    assert all(not item.pose_valid and item.pose_status == "missing_camera_info"
               for item in node.objects_pub.message.objects)
    assert node.pose_pub.message.poses == []
