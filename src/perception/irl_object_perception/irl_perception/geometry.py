"""Small, testable geometry core for depth-mask object frames."""

from __future__ import annotations

import numpy as np


def object_frame_from_mask(
    mask: np.ndarray,
    depth: np.ndarray,
    intrinsics: tuple[float, float, float, float],
    depth_scale: float,
    max_depth: float,
    up_axis: tuple[float, float, float],
    min_points: int = 30,
) -> tuple[np.ndarray, np.ndarray]:
    """Return robust 3D center and xyzw quaternion in the camera frame."""
    if mask.shape != depth.shape:
        raise ValueError("mask and depth must have the same shape")

    metric_depth = depth.astype(np.float64) * depth_scale
    valid = mask.astype(bool) & np.isfinite(metric_depth) & (metric_depth > 0.0)
    valid &= metric_depth <= max_depth
    rows, cols = np.nonzero(valid)
    if rows.size < min_points:
        raise ValueError(f"only {rows.size} valid depth points")

    # ponytail: deterministic thinning caps per-object PCA cost on large masks.
    step = max(1, rows.size // 20_000)
    rows, cols = rows[::step], cols[::step]
    z = metric_depth[rows, cols]
    fx, fy, cx, cy = intrinsics
    points = np.column_stack(((cols - cx) * z / fx, (rows - cy) * z / fy, z))
    center = np.median(points, axis=0)

    _, eigenvectors = np.linalg.eigh(np.cov((points - center).T))
    up = np.asarray(up_axis, dtype=np.float64)
    up /= np.linalg.norm(up)
    for candidate in eigenvectors[:, ::-1].T:
        x_axis = candidate - np.dot(candidate, up) * up
        if np.linalg.norm(x_axis) > 1e-6:
            break
    else:
        raise ValueError("cannot determine a horizontal principal axis")

    x_axis /= np.linalg.norm(x_axis)
    if x_axis[0] < 0.0:
        x_axis = -x_axis
    y_axis = np.cross(up, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, up)
    rotation = np.column_stack((x_axis, y_axis, up))
    return center, quaternion_from_matrix(rotation)


def quaternion_from_matrix(rotation: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to an xyzw quaternion."""
    trace = np.trace(rotation)
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        quaternion = np.array([
            (rotation[2, 1] - rotation[1, 2]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
            (rotation[1, 0] - rotation[0, 1]) / s,
            0.25 * s,
        ])
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(1.0 + rotation[i, i] - rotation[j, j] - rotation[k, k]) * 2.0
        quaternion = np.zeros(4)
        quaternion[i] = 0.25 * s
        quaternion[j] = (rotation[j, i] + rotation[i, j]) / s
        quaternion[k] = (rotation[k, i] + rotation[i, k]) / s
        quaternion[3] = (rotation[k, j] - rotation[j, k]) / s
    return quaternion / np.linalg.norm(quaternion)


def rotation_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
    """Convert an xyzw quaternion to a 3x3 rotation matrix."""
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])
