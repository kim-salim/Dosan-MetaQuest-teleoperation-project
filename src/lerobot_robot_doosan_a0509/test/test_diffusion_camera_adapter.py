from __future__ import annotations

import numpy as np

from lerobot_robot_doosan_a0509.diffusion_camera_adapter import (
    letterbox_rgb_image,
)


def test_zed_letterbox_matches_training_geometry() -> None:
    image = np.full((376, 672, 3), 127, dtype=np.uint8)

    result = letterbox_rgb_image(image, width=640, height=480)

    assert result.shape == (480, 640, 3)
    assert result.dtype == image.dtype
    assert np.count_nonzero(result[:61]) == 0
    assert np.count_nonzero(result[-61:]) == 0
    assert np.all(result[61:-61] == 127)


def test_letterbox_rejects_non_rgb_input() -> None:
    image = np.zeros((376, 672), dtype=np.uint8)

    try:
        letterbox_rgb_image(image)
    except ValueError as exc:
        assert "HWC RGB" in str(exc)
    else:
        raise AssertionError("non-RGB input should have been rejected")
