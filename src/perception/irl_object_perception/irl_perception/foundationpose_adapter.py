"""Optional adapter for the official NVlabs FoundationPose implementation."""

from __future__ import annotations

import re
import sys
from pathlib import Path
from typing import Any

import numpy as np


def parse_mesh_specs(spec: str) -> dict[str, str]:
    """Parse ``label=/path/to/mesh.obj;other=/path/to/other.obj``."""
    meshes: dict[str, str] = {}
    for item in str(spec).split(";"):
        item = item.strip()
        if not item:
            continue
        label, separator, path = item.partition("=")
        if not separator or not label.strip() or not path.strip():
            raise ValueError(f"invalid FoundationPose mesh spec: {item!r}")
        meshes[label.strip()] = path.strip()
    return meshes


def parse_mesh_scales(spec: str) -> dict[str, float]:
    """Parse ``label=scale;other=scale`` object-space mesh scale factors."""
    scales: dict[str, float] = {}
    for item in str(spec).split(";"):
        item = item.strip()
        if not item:
            continue
        label, separator, value = item.partition("=")
        if not separator or not label.strip() or not value.strip():
            raise ValueError(f"invalid FoundationPose mesh scale: {item!r}")
        try:
            scale = float(value)
        except ValueError as exc:
            raise ValueError(f"invalid FoundationPose mesh scale: {item!r}") from exc
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError(f"mesh scale must be finite and positive: {item!r}")
        scales[label.strip()] = scale
    return scales


class FoundationPoseAdapter:
    """Run FoundationPose per detected label when its optional runtime exists.

    The official implementation keeps tracking state inside one estimator, so
    one estimator is created for each configured label. The first valid mask
    calls ``register`` and later frames call ``track_one``.
    """

    def __init__(
        self,
        backend: str,
        root: str,
        mesh_specs: str,
        register_iterations: int,
        track_iterations: int,
        debug: int,
        debug_dir: str,
        logger: Any = None,
        mesh_scales: str = "",
    ) -> None:
        self.backend = backend.lower().strip()
        self.root = Path(root).expanduser()
        self.mesh_paths = parse_mesh_specs(mesh_specs)
        self.mesh_scales = parse_mesh_scales(mesh_scales)
        self.register_iterations = max(1, int(register_iterations))
        self.track_iterations = max(1, int(track_iterations))
        self.debug = int(debug)
        self.debug_dir = Path(debug_dir)
        self.logger = logger
        self._estimators: dict[str, Any] = {}
        self._registered: set[str] = set()
        self._warned: set[str] = set()
        self._foundation_pose = None
        self._score_predictor = None
        self._pose_refine_predictor = None
        self._glctx = None
        self._trimesh = None

        if self.backend not in {"auto", "foundationpose"} or not self.mesh_paths:
            return
        if not self.root.is_dir():
            self._warn_once("root", f"FoundationPose root not found: {self.root}")
            return
        try:
            self._import_backend()
        except Exception as exc:  # Optional third-party runtime; keep PCA usable.
            self._warn_once("runtime", f"FoundationPose unavailable: {exc}")

    @property
    def available(self) -> bool:
        return self._foundation_pose is not None

    def _warn_once(self, key: str, message: str) -> None:
        if key in self._warned:
            return
        self._warned.add(key)
        if self.logger is not None:
            self.logger.warning(message)

    def _import_backend(self) -> None:
        root = str(self.root)
        if root not in sys.path:
            sys.path.insert(0, root)

        import torch
        import nvdiffrast.torch as dr
        import trimesh
        from estimater import FoundationPose
        from learning.training.predict_pose_refine import PoseRefinePredictor
        from learning.training.predict_score import ScorePredictor

        if not torch.cuda.is_available():
            raise RuntimeError("PyTorch CUDA is not available")
        self._foundation_pose = FoundationPose
        self._score_predictor = ScorePredictor
        self._pose_refine_predictor = PoseRefinePredictor
        self._glctx_factory = dr.RasterizeCudaContext
        self._trimesh = trimesh

    def _runtime(self) -> tuple[Any, Any, Any]:
        if self._glctx is None:
            self.debug_dir.mkdir(parents=True, exist_ok=True)
            self._shared_scorer = self._score_predictor()
            self._shared_refiner = self._pose_refine_predictor()
            self._glctx = self._glctx_factory()
        return self._shared_scorer, self._shared_refiner, self._glctx

    def _estimator_for(self, label: str) -> Any:
        estimator = self._estimators.get(label)
        if estimator is not None:
            return estimator

        mesh_path = Path(self.mesh_paths[label]).expanduser()
        if not mesh_path.is_file():
            raise FileNotFoundError(f"mesh for {label!r} not found: {mesh_path}")
        mesh = self._trimesh.load(str(mesh_path), force="mesh")
        if not hasattr(mesh, "vertices") or len(mesh.vertices) == 0:
            raise ValueError(f"mesh has no vertices: {mesh_path}")
        scale = self.mesh_scales.get(label, 1.0)
        if scale != 1.0:
            mesh.apply_scale(scale)
            if self.logger is not None:
                self.logger.info(f"FoundationPose mesh scale for {label!r}: {scale:.9f}")

        scorer, refiner, glctx = self._runtime()
        safe_label = re.sub(r"[^A-Za-z0-9_.-]", "_", label)
        estimator = self._foundation_pose(
            model_pts=mesh.vertices,
            model_normals=mesh.vertex_normals,
            mesh=mesh,
            scorer=scorer,
            refiner=refiner,
            glctx=glctx,
            debug=self.debug,
            debug_dir=str(self.debug_dir / safe_label),
        )
        self._estimators[label] = estimator
        return estimator

    def estimate(
        self,
        label: str,
        rgb: np.ndarray,
        depth_m: np.ndarray,
        camera_matrix: np.ndarray,
        mask: np.ndarray,
    ) -> np.ndarray | None:
        """Return object-to-camera 4x4 pose, or ``None`` for PCA fallback."""
        if label not in self.mesh_paths or not self.available:
            return None

        try:
            estimator = self._estimator_for(label)
            rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
            depth_m = np.ascontiguousarray(np.nan_to_num(depth_m, nan=0.0), dtype=np.float32)
            mask = np.ascontiguousarray(mask, dtype=bool)
            camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
            if label in self._registered:
                pose = estimator.track_one(
                    rgb=rgb,
                    depth=depth_m,
                    K=camera_matrix,
                    iteration=self.track_iterations,
                )
            else:
                pose = estimator.register(
                    K=camera_matrix,
                    rgb=rgb,
                    depth=depth_m,
                    ob_mask=mask,
                    iteration=self.register_iterations,
                )
                self._registered.add(label)
            pose = np.asarray(pose, dtype=np.float64)
            if pose.shape != (4, 4) or not np.isfinite(pose).all():
                raise ValueError(f"FoundationPose returned invalid pose shape={pose.shape}")
            return pose
        except Exception as exc:  # A failed optional backend must not stop YOLOE/PCA.
            self._warn_once(f"estimate:{label}", f"FoundationPose failed for {label!r}: {exc}")
            return None
