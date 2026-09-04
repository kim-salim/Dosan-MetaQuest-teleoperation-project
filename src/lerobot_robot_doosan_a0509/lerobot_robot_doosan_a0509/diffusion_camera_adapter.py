"""Image adaptation used only by the A0509 Diffusion policy path."""

from __future__ import annotations

import cv2
import numpy as np


def letterbox_rgb_image(
    image: np.ndarray,
    *,
    width: int = 640,
    height: int = 480,
) -> np.ndarray:
    """Aspect-preserving RGB resize followed by centered black padding."""

    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"expected an HWC RGB image, got shape {image.shape}")
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")

    source_height, source_width = image.shape[:2]
    scale = min(width / source_width, height / source_height)
    resized_width = max(1, min(width, int(round(source_width * scale))))
    resized_height = max(1, min(height, int(round(source_height * scale))))
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=cv2.INTER_LANCZOS4,
    )

    top = (height - resized_height) // 2
    bottom = height - resized_height - top
    left = (width - resized_width) // 2
    right = width - resized_width - left
    return cv2.copyMakeBorder(
        resized,
        top,
        bottom,
        left,
        right,
        borderType=cv2.BORDER_CONSTANT,
        value=(0, 0, 0),
    )
