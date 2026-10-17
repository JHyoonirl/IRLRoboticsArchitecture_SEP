from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from rclpy.node import Node
from sensor_msgs.msg import JointState

from irl_interfaces.msg import RobotState


class MockStateEstimation(Node):
    """Keep the RobotState contract stable before a real fusion backend exists."""

    def __init__(self) -> None:
        super().__init__("mock_state_estimation")
        self.declare_parameters("", [
            ("joint_state_topic", "/sensing/joint_states"),
            ("robot_state_topic", "/state/robot"),
            ("world_frame", "map"),
            ("publish_rate_hz", 10.0),
        ])
        get = lambda name: self.get_parameter(name).value
        self._world_frame = str(get("world_frame"))
        self._joint_position: list[float] = []
        self._publisher = self.create_publisher(RobotState, str(get("robot_state_topic")), 10)
        self.create_subscription(
            JointState, str(get("joint_state_topic")), self._on_joint_state, 10
        )
        rate = max(0.1, float(get("publish_rate_hz")))
        self.create_timer(1.0 / rate, self._publish_state)
        self.get_logger().warning(
            "Using mock state estimation: identity pose and zero velocity"
        )

    def _on_joint_state(self, message: JointState) -> None:
        self._joint_position = list(message.position)

    def _publish_state(self) -> None:
        # shortcut: identity pose is integration-only; replace before motion-critical use.
        state = RobotState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = self._world_frame
        state.pose = PoseWithCovariance()
        state.pose.pose.orientation.w = 1.0
        state.pose.covariance[0] = 1.0
        state.pose.covariance[7] = 1.0
        state.pose.covariance[35] = 1.0
        state.twist = TwistWithCovariance()
        state.joint_position = self._joint_position
        state.system_status = "mock"
        state.valid = True
        self._publisher.publish(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockStateEstimation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
