"""Small, dependency-free sensor message cleanup helpers."""

from __future__ import annotations

import copy
import math


def normalize_header(header, frame_id: str = "", stamp=None):
    result = copy.deepcopy(header)
    if not result.frame_id and frame_id:
        result.frame_id = frame_id
    if stamp is not None and result.stamp.sec == 0 and result.stamp.nanosec == 0:
        result.stamp = stamp
    return result


def _finite_or_zero(value: float) -> float:
    return float(value) if math.isfinite(value) else 0.0


def sanitize_joint_state(message, frame_id: str = "", stamp=None):
    result = copy.deepcopy(message)
    result.header = normalize_header(message.header, frame_id, stamp)
    result.position = [_finite_or_zero(value) for value in message.position]
    result.velocity = [_finite_or_zero(value) for value in message.velocity]
    result.effort = [_finite_or_zero(value) for value in message.effort]
    return result


def sanitize_scan(message, frame_id: str = "", stamp=None):
    result = copy.deepcopy(message)
    result.header = normalize_header(message.header, frame_id, stamp)
    result.ranges = [
        math.inf if not math.isfinite(value)
        else min(max(value, message.range_min), message.range_max)
        for value in message.ranges
    ]
    return result


def copy_sensor_message(message, frame_id: str = "", stamp=None):
    result = copy.deepcopy(message)
    result.header = normalize_header(message.header, frame_id, stamp)
    return result
