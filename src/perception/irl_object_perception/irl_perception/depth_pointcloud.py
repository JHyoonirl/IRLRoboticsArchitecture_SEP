"""Depth-image to point-cloud utilities."""

from __future__ import annotations

import numpy as np


def _erode_mask(mask: np.ndarray, pixels: int) -> np.ndarray:
    result = np.asarray(mask, dtype=bool)
    for _ in range(max(0, int(pixels))):
        if min(result.shape) < 3:
            return np.zeros_like(result)
        eroded = np.zeros_like(result)
        eroded[1:-1, 1:-1] = (
            result[:-2, :-2]
            & result[:-2, 1:-1]
            & result[:-2, 2:]
            & result[1:-1, :-2]
            & result[1:-1, 1:-1]
            & result[1:-1, 2:]
            & result[2:, :-2]
            & result[2:, 1:-1]
            & result[2:, 2:]
        )
        result = eroded
    return result


def _subsample(points: np.ndarray, max_points: int | None) -> np.ndarray:
    if max_points is None or len(points) <= max_points:
        return points
    step = int(np.ceil(len(points) / max_points))
    return points[::step][:max_points]


def depth_to_point_cloud(
    depth: np.ndarray,
    intrinsics: tuple[float, float, float, float],
    depth_scale: float,
    max_depth: float,
    max_points: int | None = None,
) -> np.ndarray:
    """Back-project all valid depth pixels into the camera frame."""
    depth = np.asarray(depth)
    if depth.ndim != 2:
        raise ValueError("depth must be a 2D array")
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    if fx <= 0.0 or fy <= 0.0 or depth_scale <= 0.0:
        raise ValueError("intrinsics and depth_scale must be positive")

    metric_depth = depth.astype(np.float64) * float(depth_scale)
    valid = np.isfinite(metric_depth) & (metric_depth > 0.0) & (metric_depth <= max_depth)
    rows, cols = np.nonzero(valid)
    z = metric_depth[rows, cols]
    points = np.column_stack(((cols - cx) * z / fx, (rows - cy) * z / fy, z))
    return _subsample(points, max_points)


def mask_to_point_cloud(
    mask: np.ndarray,
    depth: np.ndarray,
    intrinsics: tuple[float, float, float, float],
    depth_scale: float,
    max_depth: float,
    erosion_pixels: int = 0,
    max_points: int | None = 20_000,
) -> np.ndarray:
    """Back-project valid depth pixels inside a segmentation mask."""
    mask = np.asarray(mask, dtype=bool)
    depth = np.asarray(depth)
    if mask.shape != depth.shape:
        raise ValueError("mask and depth must have the same shape")
    if erosion_pixels > 0:
        mask = _erode_mask(mask, erosion_pixels)

    metric_depth = depth.astype(np.float64) * float(depth_scale)
    valid = mask & np.isfinite(metric_depth) & (metric_depth > 0.0)
    valid &= metric_depth <= float(max_depth)
    rows, cols = np.nonzero(valid)
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    if fx <= 0.0 or fy <= 0.0 or depth_scale <= 0.0:
        raise ValueError("intrinsics and depth_scale must be positive")
    z = metric_depth[rows, cols]
    points = np.column_stack(((cols - cx) * z / fx, (rows - cy) * z / fy, z))
    return _subsample(points, max_points)
