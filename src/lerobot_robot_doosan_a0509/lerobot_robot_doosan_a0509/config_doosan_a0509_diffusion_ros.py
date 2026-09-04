"""Diffusion-only observation configuration for the A0509 ROS robot."""

from __future__ import annotations

from dataclasses import dataclass

from lerobot.robots.config import RobotConfig

from lerobot_robot_doosan_a0509.config_doosan_a0509_ros import (
    DoosanA0509RosConfig,
)


@RobotConfig.register_subclass("doosan_a0509_diffusion_ros")
@dataclass(kw_only=True)
class DoosanA0509DiffusionRosConfig(DoosanA0509RosConfig):
    """Keep all proven A0509 control settings and adapt only ZED RGB shape."""

    diffusion_zed_camera_key: str = "zed_rgb"
    diffusion_image_width: int = 640
    diffusion_image_height: int = 480

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.diffusion_zed_camera_key not in self.cameras:
            raise ValueError(
                "Diffusion ZED camera key is missing from cameras: "
                f"{self.diffusion_zed_camera_key!r}"
            )
        if self.diffusion_image_width <= 0 or self.diffusion_image_height <= 0:
            raise ValueError("Diffusion image width and height must be positive")
