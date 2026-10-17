"""YOLOE instance segmentation adapter."""

from __future__ import annotations

import importlib
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass
class Segment:
    label: str
    score: float
    bbox: tuple[float, float, float, float]
    mask: np.ndarray


class YoloESegmenter:
    """Load YOLOE lazily so geometry-only tests do not need Ultralytics."""

    def __init__(self, weights: str, confidence: float, text_prompt: str = "") -> None:
        self.confidence = confidence
        self.text_prompt = text_prompt.strip()
        self.model = None
        try:
            ultralytics = importlib.import_module("ultralytics")
            model_class = getattr(ultralytics, "YOLOE", ultralytics.YOLO)
            self.model = model_class(weights)
            if self.text_prompt:
                set_classes = getattr(self.model, "set_classes", None)
                if set_classes is None:
                    raise RuntimeError("the selected model does not support text prompts")
                set_classes([self.text_prompt])
        except (ImportError, FileNotFoundError, RuntimeError) as exc:
            print(f"YOLOE unavailable ({exc}); publishing empty masks")
        except Exception as exc:
            print(f"YOLOE text-prompt setup failed ({exc}); publishing empty masks")

    def segment(self, image: Any) -> list[Segment]:
        if self.model is None:
            return []
        segments: list[Segment] = []
        for result in self.model.predict(image, conf=self.confidence, verbose=False):
            if result.masks is None:
                continue
            masks = result.masks.data.cpu().numpy()
            for box, mask in zip(result.boxes, masks):
                if mask.shape != image.shape[:2]:
                    mask = cv2.resize(mask, image.shape[1::-1], interpolation=cv2.INTER_NEAREST)
                cls = int(box.cls[0])
                segments.append(Segment(
                    result.names[cls],
                    float(box.conf[0]),
                    tuple(box.xyxy[0].tolist()),
                    mask > 0.5,
                ))
        return segments
