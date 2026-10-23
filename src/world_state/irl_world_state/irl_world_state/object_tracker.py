"""Time-based 3D tracking and operational reliability, independent of ROS."""

from __future__ import annotations

from dataclasses import dataclass
import math
import re

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class TrackerConfig:
    measurement_std: float = 0.015
    acceleration_std: float = 0.5
    association_distance: float = 0.20
    mahalanobis_gate: float = 11.34
    min_observations: int = 3
    min_age: float = 0.4
    retention: float = 2.0
    pose_timeout: float = 0.7
    observation_timeout: float = 0.75
    action_max_age: float = 0.3
    max_position_std: float = 0.08
    confidence_rise_tau: float = 1.0
    confidence_fall_tau: float = 0.7
    orientation_tau: float = 0.25
    max_orientation_error: float = 0.35
    ready_on: float = 0.60
    ready_off: float = 0.40

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not 0 < self.ready_off < self.ready_on < 1:
            raise ValueError("Require 0 < ready_off < ready_on < 1")
        if self.retention < max(self.pose_timeout, self.observation_timeout):
            raise ValueError("retention must cover pose and observation timeouts")


@dataclass
class Observation:
    label: str
    score: float
    position: np.ndarray | None = None
    quaternion: np.ndarray | None = None
    bbox: np.ndarray | None = None  # camera pixels: x1, y1, x2, y2
    source: str = ""
    status: str = "pose_unavailable"
    latency: float = 0.0

    def __post_init__(self):
        self.score = float(np.clip(self.score, 0, 1)) if math.isfinite(self.score) else 0.0
        if not math.isfinite(self.latency) or self.latency < 0:
            raise ValueError("measurement latency must be finite and non-negative")
        for name, size in (("position", 3), ("quaternion", 4), ("bbox", 4)):
            value = getattr(self, name)
            if value is not None:
                value = np.asarray(value, dtype=float)
                if value.shape != (size,) or not np.isfinite(value).all():
                    value = None
                setattr(self, name, value)
        if self.quaternion is not None:
            norm = np.linalg.norm(self.quaternion)
            self.quaternion = self.quaternion / norm if norm > 1e-12 else None
        if self.quaternion is None:
            self.position = None
        if self.position is None and self.status == "ok":
            self.status = "invalid_pose"
        if self.bbox is not None and np.any(self.bbox[2:] <= self.bbox[:2]):
            self.bbox = None


@dataclass
class Track:
    object_id: str
    label: str
    created: float
    last_seen: float
    filter_time: float
    confidence_time: float
    state: np.ndarray | None = None
    covariance: np.ndarray | None = None
    quaternion: np.ndarray | None = None
    bbox: np.ndarray | None = None
    last_pose: float | None = None
    hits: int = 0
    pose_hits: int = 0
    detection_confidence: float = 0.0
    detector_ema: float = 0.0
    confidence: float = 0.0
    consistency: float = 1.0
    orientation_consistency: float = 1.0
    orientation_error: float = 0.0
    observed: bool = True
    confirmed: bool = False
    actionable: bool = False
    source: str = ""
    status: str = "pose_unavailable"
    measurement_latency: float = 0.0
    pose_latency: float = 0.0


@dataclass(frozen=True)
class Snapshot:
    track: Track
    visible: bool
    pose_valid: bool
    age: float
    tracking_status: str
    state: np.ndarray | None
    covariance: np.ndarray | None


def bbox_iou(a, b):
    if a is None or b is None:
        return 0.0
    overlap = np.maximum(0, np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2])).prod()
    union = (a[2:] - a[:2]).prod() + (b[2:] - b[:2]).prod() - overlap
    return float(overlap / union) if union > 0 else 0.0


class ObjectTracker:
    def __init__(self, config: TrackerConfig | None = None):
        self.config = config or TrackerConfig()
        self.tracks: list[Track] = []
        self.next_id = 0
        self.frame = ""
        self.last_time: float | None = None

    def _clock(self, now):
        if not math.isfinite(now):
            raise ValueError("tracking time must be finite")
        if self.last_time is not None and now < self.last_time:
            self.tracks.clear()  # simulation clock reset; never reuse IDs
        self.last_time = now
        self.tracks = [t for t in self.tracks if now - t.last_seen + t.measurement_latency < self.config.retention]

    def _motion_model(self, dt):
        f = np.eye(6)
        f[:3, 3:] = np.eye(3) * dt
        q = np.block([[np.eye(3) * dt**3 / 3, np.eye(3) * dt**2 / 2],
                      [np.eye(3) * dt**2 / 2, np.eye(3) * dt]])
        return f, q * self.config.acceleration_std**2

    def _predict(self, track, now):
        dt = max(0., now - track.filter_time)
        if track.state is not None and dt > 0:
            f, q = self._motion_model(dt)
            track.state = f @ track.state
            track.covariance = f @ track.covariance @ f.T + q
        track.filter_time = now

    def _cost(self, track, observation):
        if track.label.casefold() != observation.label.casefold():
            return 1e6
        if track.state is not None and observation.position is not None:
            residual = observation.position - track.state[:3]
            distance = np.linalg.norm(residual)
            s = track.covariance[:3, :3] + np.eye(3) * self.config.measurement_std**2
            mahalanobis = float(residual @ np.linalg.solve(s, residual))
            if distance > self.config.association_distance or mahalanobis > self.config.mahalanobis_gate:
                return 1e6
            return mahalanobis / self.config.mahalanobis_gate + distance / self.config.association_distance * 0.1
        # A temporary depth failure can retain identity through the 2D box.
        overlap = bbox_iou(track.bbox, observation.bbox)
        return 1.5 - overlap if overlap >= 0.2 else 1e6

    def _update(self, track, observation, now):
        elapsed = max(0., now - track.last_seen)
        track.detection_confidence = observation.score
        alpha = 1 - math.exp(-elapsed / 0.5)
        track.detector_ema += alpha * (observation.score - track.detector_ema)
        if track.hits == 0:
            track.detector_ema = observation.score
        track.last_seen = now
        track.measurement_latency = observation.latency
        track.observed = True
        track.hits += 1
        track.bbox = observation.bbox
        track.status = observation.status
        if observation.position is None:
            return
        if track.state is None:
            track.state = np.r_[observation.position, np.zeros(3)]
            track.covariance = np.diag([self.config.measurement_std**2] * 3 + [0.1**2] * 3)
        else:
            r = np.eye(3) * self.config.measurement_std**2
            residual = observation.position - track.state[:3]
            s = track.covariance[:3, :3] + r
            gain = np.linalg.solve(s, track.covariance[:3, :]).T
            track.state += gain @ residual
            a = np.eye(6)
            a[:, :3] -= gain
            track.covariance = a @ track.covariance @ a.T + gain @ r @ gain.T
            quality = math.exp(-0.5 * float(residual @ np.linalg.solve(s, residual)) / 3)
            track.consistency = 0.8 * track.consistency + 0.2 * quality
        q = observation.quaternion.copy()
        track.orientation_error = 0.
        if track.quaternion is not None:
            if observation.source.startswith("PCA"):
                # PCA's horizontal axes have a 180-degree sign ambiguity.
                alternative = (Rotation.from_quat(q) * Rotation.from_rotvec([0., 0., math.pi])).as_quat()
                if abs(alternative @ track.quaternion) > abs(q @ track.quaternion):
                    q = alternative
            if q @ track.quaternion < 0:
                q = -q
            angular_change = 2 * math.acos(float(np.clip(q @ track.quaternion, -1., 1.)))
            angular_quality = math.exp(-0.5 * (angular_change / 0.5)**2)
            track.orientation_consistency = 0.8 * track.orientation_consistency + 0.2 * angular_quality
            dt = max(0., now - track.last_pose)
            alpha = 1 - math.exp(-dt / self.config.orientation_tau)
            filtered = (1 - alpha) * track.quaternion + alpha * q
            filtered /= np.linalg.norm(filtered)
            track.orientation_error = 2 * math.acos(float(np.clip(abs(filtered @ q), 0., 1.)))
            q = filtered
        track.quaternion = q
        track.last_pose = now
        track.pose_latency = observation.latency
        track.pose_hits += 1
        track.source = observation.source
        track.status = "ok"
        track.confirmed |= track.pose_hits >= self.config.min_observations and now - track.created >= self.config.min_age

    def update(self, observations: list[Observation], now: float, frame: str):
        self._clock(now)
        if frame != self.frame:
            self.tracks.clear()  # never associate positions expressed in different frames
            self.frame = frame
        for track in self.tracks:
            self._predict(track, now)
            track.observed = False
        matched = set()
        if self.tracks and observations:
            costs = np.array([[self._cost(t, o) for o in observations] for t in self.tracks])
            rows, columns = linear_sum_assignment(costs)
            for row, column in zip(rows, columns):
                if costs[row, column] < 1e5:
                    self._update(self.tracks[row], observations[column], now)
                    matched.add(column)
        for index, observation in enumerate(observations):
            if index not in matched:
                label = re.sub(r"[^A-Za-z0-9_]", "_", observation.label) or "object"
                track = Track(f"{label}_{self.next_id}", observation.label, now, now, now, now)
                self.next_id += 1
                self._update(track, observation, now)
                self.tracks.append(track)

    def snapshot(self, now: float) -> list[Snapshot]:
        self._clock(now)
        results = []
        for track in self.tracks:
            self._predict(track, now)
            age = max(0., now - track.last_seen + track.measurement_latency)
            visible = track.observed and age <= self.config.observation_timeout
            state, covariance = track.state, track.covariance
            if state is not None:
                # Receipt-time filtering estimates a delayed trajectory. Extrapolate
                # through measured sensor latency for the current robot-facing pose,
                # and include the extra prediction uncertainty.
                f, q = self._motion_model(track.measurement_latency)
                state = f @ state
                covariance = f @ covariance @ f.T + q
            sigma = math.sqrt(max(np.diag(covariance)[:3])) if state is not None else math.inf
            pose_valid = (track.last_pose is not None and now - track.last_pose + track.pose_latency <= self.config.pose_timeout
                          and sigma <= self.config.max_position_std)
            quality = 1 / (1 + sigma / 0.05)
            maturity = 1 - math.exp(-track.pose_hits / 5)
            freshness = math.exp(-age / self.config.observation_timeout)
            target = (track.detector_ema * maturity * quality * track.consistency
                      * track.orientation_consistency * freshness)
            current_pose = track.last_pose == track.last_seen
            if not (visible and pose_valid and current_pose):
                target = 0.
            elapsed = max(0., now - track.confidence_time)
            tau = self.config.confidence_rise_tau if target > track.confidence else self.config.confidence_fall_tau
            track.confidence += (1 - math.exp(-elapsed / tau)) * (target - track.confidence)
            track.confidence = float(np.clip(track.confidence, 0., 0.99))
            track.confidence_time = now
            threshold = self.config.ready_off if track.actionable else self.config.ready_on
            track.actionable = bool(track.confirmed and visible and pose_valid and current_pose
                                    and age <= self.config.action_max_age
                                    and track.orientation_error <= self.config.max_orientation_error
                                    and track.confidence >= threshold)
            status = "tracking" if visible and current_pose else "coasting"
            if not track.confirmed:
                status = "tentative"
            results.append(Snapshot(track, visible, pose_valid, age, status, state, covariance))
        return results
