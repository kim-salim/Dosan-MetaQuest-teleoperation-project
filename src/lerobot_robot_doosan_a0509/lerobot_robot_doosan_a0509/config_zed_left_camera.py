"""LeRobot camera config for the left RGB image of a ZED2 UVC stream."""

from __future__ import annotations

from dataclasses import dataclass

from lerobot.cameras.configs import CameraConfig
from lerobot.cameras.opencv.configuration_opencv import (
    ColorMode,
    Cv2Backends,
)


@CameraConfig.register_subclass("zed_left_opencv")
@dataclass(kw_only=True)
class ZedLeftCameraConfig(CameraConfig):
    """Capture the 1344x376 stereo stream and expose only its left 672x376 RGB view."""

    index_or_path: str = (
        "/dev/v4l/by-id/usb-Technologies__Inc._ZED_2-video-index0"
    )
    fps: int = 30
    width: int = 672
    height: int = 376
    capture_width: int = 1344
    capture_height: int = 376
    color_mode: ColorMode = ColorMode.RGB
    warmup_s: int = 5
    fourcc: str | None = "YUYV"
    backend: Cv2Backends = Cv2Backends.ANY

    def __post_init__(self) -> None:
        self.color_mode = ColorMode(self.color_mode)
        self.backend = Cv2Backends(self.backend)
        if self.fps <= 0:
            raise ValueError("fps must be positive")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("output width and height must be positive")
        if self.capture_width != self.width * 2:
            raise ValueError("capture_width must be exactly twice output width")
        if self.capture_height != self.height:
            raise ValueError("capture_height must match output height")
        if self.warmup_s < 0:
            raise ValueError("warmup_s must be non-negative")
        if self.fourcc is not None and len(self.fourcc) != 4:
            raise ValueError("fourcc must contain exactly four characters")
