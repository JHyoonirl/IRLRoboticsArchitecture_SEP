"""OpenCV overlay for live segmentation and object-pose feedback."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np


def draw_overlay(
    image: np.ndarray,
    segments: list[Any],
    poses: dict[int, tuple[np.ndarray, np.ndarray | None, str]],
    camera_matrix: np.ndarray | None,
    target_label: str,
) -> np.ndarray:
    """Draw masks, labels, and the target's estimated pose on a BGR image."""
    frame = image.copy()
    overlay = frame.copy()
    palette = ((255, 170, 30), (200, 80, 230), (60, 220, 120), (40, 180, 255))
    target_label = target_label.casefold()
    target_indices = [
        index for index, item in enumerate(segments)
        if item.label.casefold() == target_label
    ]

    for index, item in enumerate(segments):
        color = (0, 220, 255) if item.label.casefold() == target_label else palette[index % len(palette)]
        if item.mask.shape == frame.shape[:2]:
            overlay[item.mask] = color
    frame = cv2.addWeighted(overlay, 0.28, frame, 0.72, 0.0)

    for index, item in enumerate(segments):
        x1, y1, x2, y2 = (int(round(value)) for value in item.bbox)
        is_target = item.label.casefold() == target_label
        color = (0, 220, 255) if is_target else palette[index % len(palette)]
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3 if is_target else 1)
        label = f"{item.label} {item.score:.2f}"

        pose_info = poses.get(index)
        if pose_info is not None and is_target:
            center, rotation, source = pose_info
            label += f" | {source}"
            if rotation is not None and camera_matrix is not None:
                _draw_axes(frame, center, rotation, camera_matrix)
            position_text = f"xyz [{center[0]:+.2f}, {center[1]:+.2f}, {center[2]:.2f}] m"
            cv2.putText(
                frame, position_text, (x1, min(frame.shape[0] - 8, y2 + 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 2, cv2.LINE_AA,
            )
        elif is_target:
            label += " | pose pending"

        cv2.putText(
            frame, label, (x1, max(18, y1 - 7)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2, cv2.LINE_AA,
        )

    if target_indices:
        confidence = max(segments[index].score for index in target_indices)
        sources = {poses[index][2] for index in target_indices if index in poses}
        status = ", ".join(sorted(sources)) if sources else "pose pending"
        summary = f"{target_label}: detected ({confidence:.0%}) | {status} | press q to close"
    else:
        summary = f"{target_label}: waiting for YOLOE detection | press q to close"
    cv2.rectangle(frame, (0, 0), (frame.shape[1], 34), (20, 20, 20), -1)
    cv2.putText(
        frame, summary, (10, 23),
        cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1, cv2.LINE_AA,
    )
    return frame


def _draw_axes(
    frame: np.ndarray,
    center: np.ndarray,
    rotation: np.ndarray,
    camera_matrix: np.ndarray,
) -> None:
    axes = np.array(((0.0, 0.0, 0.0), (0.15, 0.0, 0.0), (0.0, 0.08, 0.0), (0.0, 0.0, 0.08)))
    camera_points = (rotation @ axes.T).T + center
    if np.any(camera_points[:, 2] <= 0.0):
        return
    projected = (camera_matrix @ camera_points.T).T
    pixels = np.rint(projected[:, :2] / projected[:, 2:3]).astype(int)
    origin = tuple(pixels[0])
    for endpoint, color, axis in zip(
        pixels[1:], ((0, 0, 255), (0, 255, 0), (255, 0, 0)), "XYZ"
    ):
        point = tuple(endpoint)
        cv2.arrowedLine(frame, origin, point, color, 2, cv2.LINE_AA, tipLength=0.2)
        cv2.putText(frame, axis, point, cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
