"""Mask + RGB-D to top-down grasp pipeline."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .depth_pointcloud import depth_to_point_cloud, mask_to_point_cloud
from .geometry_estimator import estimate_object_geometry
from .grasp_generator import generate_top_down_grasps
from .grasp_validator import validate_candidate
from .table_filter import fit_plane_ransac, remove_plane_points
from .types import GraspPlan, PlaneModel


@dataclass(frozen=True)
class GraspPipelineConfig:
    depth_scale: float = 0.001
    max_depth: float = 3.0
    mask_erosion_pixels: int = 3
    min_points: int = 30
    table_ransac_enabled: bool = True
    table_distance_threshold: float = 0.008
    table_ransac_iterations: int = 100
    table_min_inliers: int = 100
    table_normal_tolerance_degrees: float = 30.0
    gripper_min_width: float = 0.0
    gripper_max_width: float = 0.085
    grasp_width_clearance: float = 0.005
    grasp_offset: float = 0.0
    pregrasp_offset: float = 0.10
    table_clearance: float = 0.005


class RuleBasedGraspPipeline:
    """Generate camera-frame grasp candidates without object 6D pose tracking."""

    def __init__(self, config: GraspPipelineConfig | None = None) -> None:
        self.config = config or GraspPipelineConfig()

    def estimate_table_plane(
        self,
        depth: np.ndarray,
        intrinsics: tuple[float, float, float, float],
        normal_hint: np.ndarray | tuple[float, float, float] | None = None,
    ) -> PlaneModel | None:
        if not self.config.table_ransac_enabled:
            return None
        scene_points = depth_to_point_cloud(
            depth,
            intrinsics,
            self.config.depth_scale,
            self.config.max_depth,
            max_points=8_000,
        )
        plane = fit_plane_ransac(
            scene_points,
            distance_threshold=self.config.table_distance_threshold,
            iterations=self.config.table_ransac_iterations,
            min_inliers=self.config.table_min_inliers,
            normal_hint=None if normal_hint is None else np.asarray(normal_hint, dtype=np.float64),
            normal_tolerance_degrees=self.config.table_normal_tolerance_degrees,
        )
        if plane is None:
            return None

        # Plane fitting has an arbitrary sign.  The object/grasp "up" direction
        # should point from the table toward the camera/object, so orient it
        # toward the camera origin instead of using a fixed image-axis guess.
        scene_center = np.mean(scene_points, axis=0)
        if float(plane.normal @ scene_center) > 0.0:
            return PlaneModel(
                normal=-plane.normal,
                offset=-plane.offset,
                inlier_count=plane.inlier_count,
                inlier_ratio=plane.inlier_ratio,
            )
        return plane

    def process(
        self,
        mask: np.ndarray,
        depth: np.ndarray,
        intrinsics: tuple[float, float, float, float],
        up_axis: np.ndarray | tuple[float, float, float],
        *,
        table_plane: PlaneModel | None = None,
    ) -> GraspPlan:
        points = mask_to_point_cloud(
            mask,
            depth,
            intrinsics,
            self.config.depth_scale,
            self.config.max_depth,
            erosion_pixels=self.config.mask_erosion_pixels,
        )
        if len(points) < self.config.min_points:
            return GraspPlan(None, table_plane, (), None, "not enough valid object depth points")

        if table_plane is not None:
            filtered = remove_plane_points(
                points, table_plane, self.config.table_distance_threshold
            )
            if len(filtered) >= self.config.min_points:
                points = filtered

        try:
            geometry = estimate_object_geometry(points, up_axis, min_points=self.config.min_points)
        except ValueError as exc:
            return GraspPlan(None, table_plane, (), None, str(exc))

        candidates = generate_top_down_grasps(
            geometry,
            gripper_min_width=self.config.gripper_min_width,
            gripper_max_width=self.config.gripper_max_width,
            width_clearance=self.config.grasp_width_clearance,
            grasp_offset=self.config.grasp_offset,
            pregrasp_offset=self.config.pregrasp_offset,
        )
        validated = tuple(
            validate_candidate(
                candidate,
                table_plane=table_plane,
                table_clearance=self.config.table_clearance,
            )
            for candidate in candidates
        )
        valid = tuple(sorted((item for item in validated if item.valid), key=lambda item: -item.score))
        return GraspPlan(
            geometry=geometry,
            table_plane=table_plane,
            candidates=validated,
            selected=valid[0] if valid else None,
            reason="" if valid else "no grasp candidate passed validation",
        )
