"""YOLOE instance segmentation adapter."""

from __future__ import annotations

import importlib
import json
from dataclasses import dataclass
from pathlib import Path
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

    def __init__(
        self, weights: str, confidence: float, text_prompt: str = "",
        image_prompt_dir: str = "", image_prompt_label: str = "rigid_object",
        prompt_mode: str = "auto", inference_imgsz: int = 960,
        text_prompt_aliases: str = "{}",
    ) -> None:
        self.confidence = confidence
        self.text_prompt = text_prompt.strip()
        self.text_classes = [name.strip() for name in self.text_prompt.split(",") if name.strip()]
        self.inference_imgsz = inference_imgsz
        self.model = None
        try:
            aliases = json.loads(text_prompt_aliases)
            if not isinstance(aliases, dict) or not all(
                isinstance(key, str) and isinstance(value, str) and value.strip()
                for key, value in aliases.items()
            ):
                raise ValueError("text_prompt_aliases must map class names to non-empty text descriptions")
            self.text_descriptions = [aliases.get(name, name) for name in self.text_classes]
            if inference_imgsz <= 0 or inference_imgsz % 32:
                raise ValueError("inference_imgsz must be a positive multiple of 32")
            if len(set(self.text_classes)) != len(self.text_classes):
                raise ValueError("Text prompt class names must be unique")
            mode = prompt_mode.strip().lower()
            if mode == "auto":
                mode = "hybrid" if image_prompt_dir and self.text_prompt else (
                    "image" if image_prompt_dir else "text"
                )
            if mode not in {"text", "image", "hybrid"}:
                raise ValueError("prompt_mode must be auto, text, image, or hybrid")
            if mode in {"image", "hybrid"} and not image_prompt_dir:
                raise ValueError(f"{mode} prompting requires image_prompt_dir")
            if mode == "hybrid" and not self.text_prompt:
                raise ValueError("hybrid prompting requires segmentation_prompt")
            ultralytics = importlib.import_module("ultralytics")
            model_class = getattr(ultralytics, "YOLOE", ultralytics.YOLO)
            self.model = model_class(weights)
            if mode in {"image", "hybrid"}:
                self._set_image_prompts(
                    image_prompt_dir, image_prompt_label,
                    text_classes=self.text_classes if mode == "hybrid" else [],
                )
            elif self.text_prompt:
                set_classes = getattr(self.model, "set_classes", None)
                if set_classes is None:
                    raise RuntimeError("the selected model does not support text prompts")
                set_classes(self.text_classes, self.model.get_text_pe(self.text_descriptions))
        except (ImportError, FileNotFoundError, RuntimeError) as exc:
            self.model = None
            print(f"YOLOE unavailable ({exc}); publishing empty masks")
        except Exception as exc:
            self.model = None
            print(f"YOLOE prompt setup failed ({exc}); publishing empty masks")

    def _set_image_prompts(
        self, directory: str, label: str, text_classes: list[str],
    ) -> None:
        """Pool visual embeddings from tightly cropped views of one object."""
        import torch
        from ultralytics.models.yolo.yoloe.predict import YOLOEVPSegPredictor

        if not label.strip():
            raise ValueError("image_prompt_label must not be empty")
        if label in text_classes:
            raise ValueError("Image and text prompts must have distinct class names")
        paths = sorted(path for path in Path(directory).expanduser().iterdir()
                       if path.is_file() and path.suffix.lower() in {".jpg", ".jpeg", ".png"})
        if not paths:
            raise FileNotFoundError(f"No reference images in {directory}")
        predictor = YOLOEVPSegPredictor(overrides={
            "task": "segment", "mode": "predict", "imgsz": 640,
            "save": False, "verbose": False, "batch": 1,
        })
        predictor.setup_model(model=self.model.model, verbose=False)
        predictor.model.names = {0: label}
        embeddings = []
        with torch.inference_mode():
            for path in paths:
                image = cv2.imread(str(path))
                if image is None:
                    raise ValueError(f"Cannot decode reference image: {path}")
                height, width = image.shape[:2]
                predictor.set_prompts({
                    "bboxes": np.array([[0, 0, width, height]], dtype=np.float32),
                    "cls": np.array([0], dtype=np.int32),
                })
                embeddings.append(predictor.get_vpe(image).clone())
            pooled = torch.nn.functional.normalize(torch.stack(embeddings).mean(dim=0), dim=-1)
            if text_classes:
                text_pe = self.model.get_text_pe(self.text_descriptions).to(device=pooled.device, dtype=pooled.dtype)
                if text_pe.shape[0] != pooled.shape[0] or text_pe.shape[-1] != pooled.shape[-1]:
                    raise ValueError(f"Text/image embedding shapes differ: {text_pe.shape}, {pooled.shape}")
                # Keep every target as its own class: concatenate prompt slots,
                # never average an unrelated text object into the image target.
                pooled = torch.cat([pooled, text_pe], dim=1)
            self.model.set_classes([label, *text_classes], pooled)
        mode = "hybrid" if text_classes else "image"
        print(f"YOLOE {mode} prompt ready: {label}, {len(paths)} reference images from {directory}")
        if text_classes:
            print(f"YOLOE independent targets: image={[label]}, text={text_classes}")
            print(f"YOLOE text descriptions: {dict(zip(text_classes, self.text_descriptions))}")

    def segment(self, image: Any) -> list[Segment]:
        if self.model is None:
            return []
        segments: list[Segment] = []
        for result in self.model.predict(
            image, conf=self.confidence, imgsz=self.inference_imgsz,
            agnostic_nms=False, verbose=False,
        ):
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
