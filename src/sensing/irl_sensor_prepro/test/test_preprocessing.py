import math

import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState, LaserScan

from irl_sensor_prepro.preprocessing import sanitize_joint_state, sanitize_scan


def test_joint_state_normalizes_header_and_invalid_values():
    message = JointState()
    message.position = [1.0, math.nan]
    result = sanitize_joint_state(message, "base_link", Time(sec=4))

    assert result.header.frame_id == "base_link"
    assert result.header.stamp.sec == 4
    assert list(result.position) == [1.0, 0.0]


def test_scan_clamps_ranges_and_marks_invalid_values():
    message = LaserScan(range_min=0.2, range_max=5.0)
    message.ranges = [0.1, math.nan, 2.0, 8.0]
    result = sanitize_scan(message)

    assert result.ranges[0] == pytest.approx(0.2)
    assert math.isinf(result.ranges[1])
    assert list(result.ranges[2:]) == pytest.approx([2.0, 5.0])
