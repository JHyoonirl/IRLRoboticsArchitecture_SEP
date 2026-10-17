from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray


class ActivePerception(Node):
    def __init__(self) -> None:
        super().__init__("active_perception")
        self.declare_parameters("", [
            ("detections_topic", "/perception/detections"),
            ("inspection_needed_topic", "/perception/inspection_needed"),
            ("minimum_confidence", 0.3),
        ])
        get = lambda name: self.get_parameter(name).value
        self._minimum_confidence = float(get("minimum_confidence"))
        self._publisher = self.create_publisher(
            Bool, str(get("inspection_needed_topic")), 10
        )
        self.create_subscription(
            Detection2DArray, str(get("detections_topic")), self._on_detections, 10
        )

    def _on_detections(self, message: Detection2DArray) -> None:
        enough_observation = any(
            result.hypothesis.score >= self._minimum_confidence
            for detection in message.detections
            for result in detection.results
        )
        output = Bool()
        output.data = not enough_observation
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActivePerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
