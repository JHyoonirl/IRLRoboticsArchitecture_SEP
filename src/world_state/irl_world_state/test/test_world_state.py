import math
import time

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from tf2_ros import StaticTransformBroadcaster

from irl_interfaces.msg import WorldObject, WorldObjectArray, WorldState
from irl_world_state.world_state import WorldStateManager


@pytest.fixture
def graph():
    rclpy.init()
    manager = WorldStateManager()
    probe = rclpy.create_node("tracking_publication_test")
    executor = SingleThreadedExecutor()
    executor.add_node(manager)
    executor.add_node(probe)

    def wait_until(condition, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if condition():
                return
        raise AssertionError("Timed out waiting for tracking publication")

    yield manager, probe, wait_until
    executor.shutdown()
    probe.destroy_node()
    manager.destroy_node()
    rclpy.shutdown()


def message(probe, positions=(0., 0.3), frame="camera_color_optical_frame"):
    objects = WorldObjectArray()
    objects.header.frame_id = frame
    objects.header.stamp = probe.get_clock().now().to_msg()
    for index, x in enumerate(positions):
        item = WorldObject(header=objects.header, object_id=f"raw_{index}", class_name="glove",
                           visible=True, pose_valid=True, pose_source="PCA", pose_status="ok",
                           detection_confidence=0.9, confidence=0.9)
        item.pose.position.x, item.pose.position.z = x, 0.6
        item.pose.orientation.w = 1.
        item.bbox.center.position.x = 100. + x * 100
        item.bbox.center.position.y = 100.
        item.bbox.size_x = item.bbox.size_y = 30.
        objects.objects.append(item)
    return objects


def test_robot_topics_transform_filter_and_keep_ids_across_order_and_occlusion(graph):
    manager, probe, wait_until = graph
    snapshots, worlds = [], []
    probe.create_subscription(WorldObjectArray, "/state/objects", snapshots.append, 10)
    probe.create_subscription(WorldState, "/state/world", worlds.append, 10)
    publisher = probe.create_publisher(WorldObjectArray, "/perception/objects", 10)
    broadcaster = StaticTransformBroadcaster(probe)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = "base_link", "camera_color_optical_frame"
    tf.header.stamp = probe.get_clock().now().to_msg()
    tf.transform.translation.x, tf.transform.translation.y = 1., 2.
    tf.transform.rotation.z = tf.transform.rotation.w = math.sqrt(0.5)
    broadcaster.sendTransform(tf)
    wait_until(lambda: snapshots and worlds and publisher.get_subscription_count()
               and manager._tf_buffer.can_transform("base_link", tf.child_frame_id, rclpy.time.Time()))
    assert snapshots[-1].objects == []
    publisher.publish(message(probe))
    wait_until(lambda: len(snapshots[-1].objects) == 2)
    first, second = snapshots[-1].objects
    assert first.object_id != "raw_0"
    assert first.header.frame_id == "base_link"
    assert first.pose.position.x == pytest.approx(1.)
    assert first.pose.position.y == pytest.approx(2.)
    assert second.pose.position.y == pytest.approx(2.3)
    assert first.confidence < first.detection_confidence
    assert not first.actionable
    ids = [first.object_id, second.object_id]
    for count in range(2, 6):
        publisher.publish(message(probe, (0.3, 0.)))
        wait_until(lambda: snapshots[-1].objects[0].observation_count >= count)
        assert [x.object_id for x in snapshots[-1].objects] == ids
        assert snapshots[-1].objects[0].pose.position.y == pytest.approx(2.)
    publisher.publish(message(probe, ()))
    wait_until(lambda: snapshots[-1].objects and not snapshots[-1].objects[0].visible)
    assert not snapshots[-1].objects[0].actionable
    assert snapshots[-1].objects[0].confidence > 0
    publisher.publish(message(probe))
    wait_until(lambda: snapshots[-1].objects and snapshots[-1].objects[0].visible)
    assert [x.object_id for x in snapshots[-1].objects] == ids
    wait_until(lambda: not snapshots[-1].objects and not worlds[-1].objects)


def test_missing_tf_preserves_detection_but_prevents_action(graph):
    manager, probe, _ = graph
    manager._on_objects(message(probe, (0.,)))
    result = manager._tracked_objects(probe.get_clock().now()).objects[0]
    assert not result.pose_valid and not result.actionable
    assert result.pose_status == "missing_transform"
    assert result.confidence == 0


def test_delayed_or_replayed_messages_do_not_refresh_tracks(graph):
    manager, probe, _ = graph
    old = message(probe, (0.,), "base_link")
    old.header.stamp.sec -= 5
    manager._on_objects(old)
    assert not manager._tracker.tracks
    fresh = message(probe, (0.,), "base_link")
    manager._on_objects(fresh)
    manager._on_objects(fresh)
    assert manager._tracker.tracks[0].hits == 1
