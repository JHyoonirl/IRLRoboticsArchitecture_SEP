"""Object-frame estimation independent of ROS message handling."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .geometry import object_frame_from_mask, quaternion_from_matrix, rotation_from_quaternion


@dataclass(frozen=True)
class PoseEstimate:
    center: np.ndarray
    quaternion: np.ndarray
    rotation: np.ndarray | None
    source: str


class ObjectPoseEstimator:
    """Use optional FoundationPose, with the existing depth-PCA fallback."""

    def __init__(
        self,
        foundationpose: Any,
        *,
        depth_scale: float,
        max_depth: float,
        up_axis: tuple[float, float, float],
    ) -> None:
        self.foundationpose = foundationpose
        self.depth_scale = depth_scale
        self.max_depth = max_depth
        self.up_axis = up_axis

    def estimate(
        self,
        label: str,
        rgb: np.ndarray,
        depth_m: np.ndarray,
        camera_matrix: np.ndarray,
        depth: np.ndarray,
        intrinsics: tuple[float, float, float, float],
        mask: np.ndarray,
        up_axis: np.ndarray | tuple[float, float, float] | None = None,
    ) -> PoseEstimate | None:
        foundation_pose = self.foundationpose.estimate(
            label, rgb, depth_m, camera_matrix, mask
        )
        if foundation_pose is not None:
            rotation = foundation_pose[:3, :3]
            return PoseEstimate(
                center=foundation_pose[:3, 3],
                quaternion=quaternion_from_matrix(rotation),
                rotation=rotation,
                source="FoundationPose",
            )
        try:
            pose_up_axis = self.up_axis if up_axis is None else up_axis
            center, quaternion = object_frame_from_mask(
                mask,
                depth,
                intrinsics,
                self.depth_scale,
                self.max_depth,
                pose_up_axis,
            )
        except ValueError:
            return None
        source = "PCA fallback" if label in self.foundationpose.mesh_paths else "PCA"
        return PoseEstimate(center, quaternion, rotation_from_quaternion(quaternion), source)
