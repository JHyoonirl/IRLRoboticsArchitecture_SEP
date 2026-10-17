"""Robust table-plane estimation for RGB-D scenes."""

from __future__ import annotations

import numpy as np

from .types import PlaneModel


def _normalize(vector: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vector)
    if norm <= 1e-9:
        raise ValueError("cannot normalize a near-zero vector")
    return vector / norm


def _plane_from_three(points: np.ndarray) -> tuple[np.ndarray, float] | None:
    normal = np.cross(points[1] - points[0], points[2] - points[0])
    norm = np.linalg.norm(normal)
    if norm <= 1e-9:
        return None
    normal = normal / norm
    return normal, -float(normal @ points[0])


def fit_plane_ransac(
    points: np.ndarray,
    *,
    distance_threshold: float = 0.008,
    iterations: int = 100,
    min_inliers: int = 100,
    normal_hint: np.ndarray | None = None,
    normal_tolerance_degrees: float = 30.0,
    seed: int = 0,
) -> PlaneModel | None:
    """Fit a dominant plane, optionally constrained by an expected up axis."""
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
        return None
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 3:
        return None

    hint = None
    min_alignment = -1.0
    if normal_hint is not None:
        hint = _normalize(np.asarray(normal_hint, dtype=np.float64))
        min_alignment = np.cos(np.deg2rad(float(normal_tolerance_degrees)))

    rng = np.random.default_rng(seed)
    best: tuple[int, np.ndarray, float] | None = None
    for _ in range(max(1, int(iterations))):
        sample = points[rng.choice(len(points), size=3, replace=False)]
        plane = _plane_from_three(sample)
        if plane is None:
            continue
        normal, offset = plane
        alignment = abs(float(normal @ hint)) if hint is not None else 1.0
        if alignment < min_alignment:
            continue
        distances = np.abs(points @ normal + offset)
        inliers = int(np.count_nonzero(distances <= distance_threshold))
        if best is None or inliers > best[0]:
            best = (inliers, normal, offset)

    if best is None or best[0] < max(3, int(min_inliers)):
        return None
    inlier_count, normal, offset = best
    if hint is not None and normal @ hint < 0.0:
        normal = -normal
        offset = -offset
    return PlaneModel(normal, float(offset), inlier_count, inlier_count / len(points))


def remove_plane_points(
    points: np.ndarray,
    plane: PlaneModel,
    distance_threshold: float = 0.008,
) -> np.ndarray:
    """Return points that are not part of the estimated plane."""
    points = np.asarray(points, dtype=np.float64)
    distances = np.abs(plane.signed_distance(points))
    return points[distances > float(distance_threshold)]
