import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from irl_world_state.object_tracker import ObjectTracker, Observation, TrackerConfig


def observation(x=0., label="glove", score=0.9, q=None):
    return Observation(label, score, np.array([x, 0., 0.6]),
                       np.array([0., 0., 0., 1.]) if q is None else q,
                       np.array([x * 100 + 100, 100, x * 100 + 140, 150]), "PCA", "ok")


def test_ids_survive_reordered_same_class_detections():
    tracker = ObjectTracker()
    tracker.update([observation(-0.3), observation(0.3)], 0., "base_link")
    left, right = [s.track.object_id for s in tracker.snapshot(0.)]
    for i in range(1, 20):
        tracker.update([observation(0.3), observation(-0.3)], i * 0.1, "base_link")
        items = tracker.snapshot(i * 0.1)
        assert len(items) == 2
        positions = {s.track.object_id: s.track.state[0] for s in items}
        assert positions[left] < 0 and positions[right] > 0


def test_filter_reduces_position_noise():
    tracker = ObjectTracker()
    rng = np.random.default_rng(42)
    measurements, filtered = [], []
    for i in range(150):
        x = float(rng.normal(0, 0.01))
        tracker.update([observation(x)], i * 0.05, "base_link")
        sample = tracker.snapshot(i * 0.05)[0]
        measurements.append(x)
        filtered.append(sample.track.state[0])
    assert np.std(filtered[20:]) < np.std(measurements[20:]) * 0.85


def test_short_occlusion_keeps_id_but_disables_action_and_damps_confidence():
    tracker = ObjectTracker()
    for i in range(60):
        tracker.update([observation()], i * 0.1, "base_link")
        before = tracker.snapshot(i * 0.1)[0]
    old_id, old_confidence = before.track.object_id, before.track.confidence
    assert before.track.confirmed and before.track.actionable
    tracker.update([], 6., "base_link")
    missing = tracker.snapshot(6.)[0]
    assert not missing.visible and not missing.track.actionable
    assert 0 < missing.track.confidence < old_confidence
    tracker.update([observation()], 6.3, "base_link")
    assert tracker.snapshot(6.3)[0].track.object_id == old_id
    assert tracker.snapshot(8.4) == []


def test_sudden_detector_score_changes_do_not_step_operational_confidence():
    tracker = ObjectTracker()
    confidence = []
    for i in range(100):
        tracker.update([observation(score=1. if i < 50 else 0.)], i * 0.1, "base_link")
        confidence.append(tracker.snapshot(i * 0.1)[0].track.confidence)
    assert confidence[0] == 0.
    assert max(abs(np.diff(confidence))) < 0.1
    assert 0 < confidence[-1] < confidence[50]
    assert max(confidence) < 1.


def test_sensor_dropout_decays_confidence_without_reprocessing_old_measurements():
    tracker = ObjectTracker()
    for i in range(50):
        tracker.update([observation()], i * 0.1, "base_link")
        tracker.snapshot(i * 0.1)
    last = tracker.tracks[0].confidence
    hits = tracker.tracks[0].hits
    confidences = [tracker.snapshot(4.9 + dt)[0].track.confidence for dt in (0.1, 0.3, 0.8, 1.5)]
    assert all(a > b for a, b in zip([last] + confidences[:-1], confidences))
    assert tracker.tracks[0].hits == hits
    assert not tracker.tracks[0].actionable


def test_depth_loss_retains_id_using_bbox_but_never_actionable():
    tracker = ObjectTracker()
    tracker.update([observation()], 0., "base_link")
    original = tracker.snapshot(0.)[0].track.object_id
    tracker.update([Observation("glove", 0.9, bbox=observation().bbox)], 0.1, "base_link")
    missing = tracker.snapshot(0.1)[0]
    assert missing.track.object_id == original
    assert not missing.track.actionable
    assert missing.track.pose_hits == 1
    tracker.update([observation()], 0.2, "base_link")
    assert tracker.snapshot(0.2)[0].track.object_id == original


def test_outlier_does_not_teleport_confirmed_track():
    tracker = ObjectTracker()
    for i in range(10):
        tracker.update([observation()], i * 0.1, "base_link")
        tracker.snapshot(i * 0.1)
    original = tracker.tracks[0].object_id
    tracker.update([observation(1.)], 1., "base_link")
    snapshots = tracker.snapshot(1.)
    assert len(snapshots) == 2
    assert abs(next(s.track.state[0] for s in snapshots if s.track.object_id == original)) < 0.01
    assert not snapshots[-1].track.actionable


def test_quaternion_sign_and_pca_axis_flip_are_stable():
    tracker = ObjectTracker()
    for i, q in enumerate(([0, 0, 0, 1], [0, 0, 0, -1], [0, 0, 1, 0])):
        tracker.update([observation(q=np.array(q, float))], i * 0.1, "base_link")
        snapshot = tracker.snapshot(i * 0.1)[0]
        assert abs(snapshot.track.quaternion[3]) > 0.99


def test_rotation_changes_are_damped():
    tracker = ObjectTracker()
    tracker.update([observation()], 0., "base_link")
    tracker.update([observation(q=Rotation.from_euler("z", 45, degrees=True).as_quat())], 0.1, "base_link")
    angle = Rotation.from_quat(tracker.snapshot(0.1)[0].track.quaternion).magnitude()
    assert 0 < angle < math.pi / 4


def test_class_and_frame_changes_do_not_reuse_identity():
    tracker = ObjectTracker()
    tracker.update([observation()], 0., "base_link")
    first = tracker.tracks[0].object_id
    tracker.update([observation(label="cup")], 0.1, "base_link")
    assert tracker.tracks[-1].object_id != first
    tracker.update([observation()], 0.2, "other_frame")
    assert len(tracker.tracks) == 1 and tracker.tracks[0].object_id != first


def test_velocity_prediction_keeps_ids_when_same_class_objects_cross():
    tracker = ObjectTracker()
    identities = None
    for i in range(41):
        t = i * 0.1
        left_to_right, right_to_left = -0.2 + 0.1 * t, 0.2 - 0.1 * t
        measurements = [observation(left_to_right), observation(right_to_left)]
        if i % 2:
            measurements.reverse()
        tracker.update(measurements, t, "base_link")
        snapshots = tracker.snapshot(t)
        if identities is None:
            identities = [x.track.object_id for x in snapshots]
        assert [x.track.object_id for x in snapshots] == identities
    assert tracker.tracks[0].state[0] > 0.15
    assert tracker.tracks[1].state[0] < -0.15


def test_old_camera_measurement_cannot_enable_action_even_with_high_confidence():
    tracker = ObjectTracker()
    for i in range(60):
        tracker.update([observation()], i * 0.1, "base_link")
        tracker.snapshot(i * 0.1)
    delayed = observation()
    delayed.latency = 0.4
    tracker.update([delayed], 6., "base_link")
    result = tracker.snapshot(6.)[0]
    assert result.age == pytest.approx(0.4)
    assert not result.track.actionable
    assert result.track.confidence > 0.4


def test_output_pose_compensates_latency_and_increases_uncertainty():
    tracker = ObjectTracker()
    for i in range(20):
        obs = observation(i * 0.01)
        obs.latency = 0.2
        tracker.update([obs], i * 0.1, "base_link")
        snapshot = tracker.snapshot(i * 0.1)[0]
    assert snapshot.state[0] > snapshot.track.state[0]
    assert snapshot.state[0] == pytest.approx(0.21, abs=0.01)
    assert snapshot.covariance[0, 0] > snapshot.track.covariance[0, 0]


def test_orientation_jump_waits_for_filter_to_settle_before_action():
    tracker = ObjectTracker()
    for i in range(60):
        tracker.update([observation()], i * 0.1, "base_link")
        assert tracker.snapshot(i * 0.1)
    assert tracker.tracks[0].actionable
    tracker.update([observation(q=Rotation.from_euler("z", 90, degrees=True).as_quat())], 6., "base_link")
    assert not tracker.snapshot(6.)[0].track.actionable


def test_backward_clock_resets_tracks_without_reusing_old_ids():
    tracker = ObjectTracker()
    tracker.update([observation()], 5., "base_link")
    identity = tracker.tracks[0].object_id
    tracker.update([observation()], 1., "base_link")
    assert len(tracker.tracks) == 1
    assert tracker.tracks[0].object_id != identity


def test_bad_depth_or_quaternion_never_produces_actionable_pose():
    tracker = ObjectTracker()
    for i in range(20):
        bad = Observation("glove", 1., np.array([math.nan, 0., 0.]), np.zeros(4), observation().bbox)
        tracker.update([bad], i * 0.1, "base_link")
        result = tracker.snapshot(i * 0.1)[0]
        assert not result.pose_valid and not result.track.actionable
        assert result.track.confidence == 0.


@pytest.mark.parametrize("kwargs", [{"confidence_rise_tau": 0}, {"ready_on": 0.2}, {"measurement_std": math.nan}])
def test_invalid_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        TrackerConfig(**kwargs)
