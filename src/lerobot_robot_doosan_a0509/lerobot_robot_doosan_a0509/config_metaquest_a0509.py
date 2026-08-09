"""Configuration registered as ``--teleop.type=metaquest_a0509``."""

from __future__ import annotations

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("metaquest_a0509")
@dataclass(kw_only=True)
class MetaQuestA0509Config(TeleoperatorConfig):
    target_posx_topic: str = "/control/metaquest/target_posx"
    gripper_commanded_state_topic: str = "/jrt_gripper/commanded_state"
    heartbeat_topic: str = "/control/metaquest/valid_pose_heartbeat"
    calibration_valid_topic: str = "/vr/metaquest_calibration/valid"
    teleop_ready_topic: str = "/vr/teleop_ready"
    require_calibration: bool = True
    max_age_sec: float = 1.0
    connect_timeout_sec: float = 3.0
    require_fresh_action_on_connect: bool = True
    allow_latched_state_in_shadow_record: bool = False

    def __post_init__(self) -> None:
        if self.max_age_sec <= 0.0:
            raise ValueError("max_age_sec must be positive")
        if self.connect_timeout_sec < 0.0:
            raise ValueError("connect_timeout_sec must be non-negative")
