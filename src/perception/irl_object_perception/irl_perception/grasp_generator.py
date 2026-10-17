"""Simple top-down parallel-jaw grasp generation."""

from __future__ import annotations

import numpy as np

from .types import GraspCandidate, ObjectGeometry


def _normalize(vector: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vector)
    if norm <= 1e-9:
        raise ValueError("cannot normalize a near-zero vector")
    return vector / norm


def _rotate_about_axis(vector: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    axis = _normalize(axis)
    return (
        vector * np.cos(angle)
        + np.cross(axis, vector) * np.sin(angle)
        + axis * (axis @ vector) * (1.0 - np.cos(angle))
    )


def generate_top_down_grasps(
    geometry: ObjectGeometry,
    *,
    gripper_min_width: float = 0.0,
    gripper_max_width: float = 0.085,
    width_clearance: float = 0.005,
    grasp_offset: float = 0.0,
    pregrasp_offset: float = 0.10,
    angle_offsets_degrees: tuple[float, ...] = (0.0, 15.0, -15.0, 90.0, 105.0, 75.0),
) -> tuple[GraspCandidate, ...]:
    """Generate grasp poses; x is closing direction and z is approach direction."""
    major, minor, up = geometry.axes.T
    bases = ((minor, geometry.dimensions[1]), (major, geometry.dimensions[0]))
    candidates: list[GraspCandidate] = []
    for base_axis, object_width in bases:
        requested_width = float(object_width + width_clearance)
        for angle_degrees in angle_offsets_degrees:
            closing_axis = _normalize(
                _rotate_about_axis(base_axis, up, np.deg2rad(float(angle_degrees)))
            )
            lateral_axis = _normalize(np.cross(up, closing_axis))
            rotation = np.column_stack((closing_axis, lateral_axis, up))
            position = geometry.top_center + up * float(grasp_offset)
            pregrasp = position + up * float(pregrasp_offset)
            valid = gripper_min_width <= requested_width <= gripper_max_width
            reason = "" if valid else "object width is outside gripper limits"
            width_margin = max(0.0, 1.0 - requested_width / max(gripper_max_width, 1e-6))
            score = 0.6 * geometry.orientation_confidence + 0.4 * width_margin
            candidates.append(GraspCandidate(
                position=position,
                pregrasp_position=pregrasp,
                rotation=rotation,
                width=requested_width,
                score=float(score),
                valid=valid,
                reason=reason,
            ))
    return tuple(candidates)
