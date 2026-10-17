"""Data structures shared by the depth-to-grasp perception pipeline."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PlaneModel:
    """A normalized plane ``normal @ point + offset = 0``."""

    normal: np.ndarray
    offset: float
    inlier_count: int
    inlier_ratio: float

    def signed_distance(self, points: np.ndarray) -> np.ndarray:
        return np.asarray(points) @ self.normal + self.offset


@dataclass(frozen=True)
class ObjectGeometry:
    """Robust geometry estimated from an object's visible point cloud."""

    center: np.ndarray
    top_center: np.ndarray
    axes: np.ndarray
    dimensions: np.ndarray
    top_normal: np.ndarray
    orientation_confidence: float
    point_count: int


@dataclass(frozen=True)
class GraspCandidate:
    """A top-down parallel-jaw grasp expressed in one coordinate frame."""

    position: np.ndarray
    pregrasp_position: np.ndarray
    rotation: np.ndarray
    width: float
    score: float
    valid: bool = True
    reason: str = ""


@dataclass(frozen=True)
class GraspPlan:
    """Result of one segmentation-to-grasp pipeline evaluation."""

    geometry: ObjectGeometry | None
    table_plane: PlaneModel | None
    candidates: tuple[GraspCandidate, ...]
    selected: GraspCandidate | None
    reason: str = ""
