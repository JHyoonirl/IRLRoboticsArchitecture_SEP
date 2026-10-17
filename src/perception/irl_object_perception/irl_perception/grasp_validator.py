"""Execution-independent grasp candidate validation."""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from .types import GraspCandidate, PlaneModel


def validate_candidate(
    candidate: GraspCandidate,
    *,
    table_plane: PlaneModel | None = None,
    table_clearance: float = 0.005,
    workspace_min: np.ndarray | None = None,
    workspace_max: np.ndarray | None = None,
) -> GraspCandidate:
    """Apply finite-value, table-clearance, and optional workspace checks."""
    if not candidate.valid:
        return candidate
    if not np.isfinite(candidate.position).all() or not np.isfinite(candidate.rotation).all():
        return replace(candidate, valid=False, reason="non-finite grasp pose")
    if table_plane is not None:
        if table_plane.signed_distance(candidate.position) < table_clearance:
            return replace(candidate, valid=False, reason="grasp pose is too close to table")
        if table_plane.signed_distance(candidate.pregrasp_position) < table_clearance:
            return replace(candidate, valid=False, reason="pre-grasp pose is too close to table")
    if workspace_min is not None and np.any(candidate.pregrasp_position < workspace_min):
        return replace(candidate, valid=False, reason="pre-grasp is outside workspace")
    if workspace_max is not None and np.any(candidate.pregrasp_position > workspace_max):
        return replace(candidate, valid=False, reason="pre-grasp is outside workspace")
    return candidate
