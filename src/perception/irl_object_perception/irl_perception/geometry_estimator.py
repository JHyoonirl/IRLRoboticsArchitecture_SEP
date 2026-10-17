"""Visible-object geometry estimation for grasp planning."""

from __future__ import annotations

import numpy as np

from .types import ObjectGeometry


def _normalize(vector: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vector)
    if norm <= 1e-9:
        raise ValueError("cannot normalize a near-zero vector")
    return vector / norm


def estimate_object_geometry(
    points: np.ndarray,
    up_axis: np.ndarray | tuple[float, float, float],
    *,
    min_points: int = 30,
) -> ObjectGeometry:
    """Estimate center, top surface, dimensions, and horizontal principal axes."""
    points = np.asarray(points, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < min_points:
        raise ValueError(f"only {len(points)} usable object points; need {min_points}")

    up = _normalize(np.asarray(up_axis, dtype=np.float64))
    center = np.median(points, axis=0)
    centered = points - center
    projected = centered - np.outer(centered @ up, up)
    covariance = (projected.T @ projected) / max(1, len(points) - 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    major = eigenvectors[:, int(np.argmax(eigenvalues))]
    major = _normalize(major - (major @ up) * up)
    minor = _normalize(np.cross(up, major))
    major = _normalize(np.cross(minor, up))
    axes = np.column_stack((major, minor, up))
    if np.linalg.det(axes) < 0.0:
        minor = -minor
        axes = np.column_stack((major, minor, up))

    coordinates = (points - center) @ axes
    lower = np.percentile(coordinates, 2.0, axis=0)
    upper = np.percentile(coordinates, 98.0, axis=0)
    dimensions = np.maximum(upper - lower, 0.0)

    heights = points @ up
    top_threshold = np.percentile(heights, 90.0)
    top_points = points[heights >= top_threshold]
    top_center = np.median(top_points, axis=0)
    top_center = top_center - up * float(top_center @ up - np.percentile(heights, 95.0))

    horizontal_eigenvalues = np.sort(eigenvalues)[::-1][:2]
    orientation_confidence = float(
        max(0.0, (horizontal_eigenvalues[0] - horizontal_eigenvalues[1])
             / (horizontal_eigenvalues[0] + horizontal_eigenvalues[1] + 1e-9))
    )
    return ObjectGeometry(
        center=center,
        top_center=top_center,
        axes=axes,
        dimensions=dimensions,
        top_normal=up,
        orientation_confidence=orientation_confidence,
        point_count=len(points),
    )
