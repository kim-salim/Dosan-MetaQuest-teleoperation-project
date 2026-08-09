"""Configuration registered as ``--robot.type=doosan_a0509_ros``."""

from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.cameras.configs import CameraConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.robots.config import RobotConfig

from lerobot_robot_doosan_a0509.config_zed_left_camera import (
    ZedLeftCameraConfig,
)


def _default_cameras() -> dict[str, CameraConfig]:
    return {
        "front": OpenCVCameraConfig(
            index_or_path=(
                "/dev/v4l/by-id/"
                "usb-046d_HD_Pro_Webcam_C920-video-index0"
            ),
            fps=30,
            width=640,
            height=480,
            fourcc="MJPG",
            warmup_s=5,
        ),
        "side": OpenCVCameraConfig(
            index_or_path=(
                "/dev/v4l/by-id/"
                "usb-046d_HD_Pro_Webcam_C920_947C90BF-video-index0"
            ),
            fps=30,
            width=640,
            height=480,
            fourcc="MJPG",
            warmup_s=5,
        ),
        "zed_rgb": ZedLeftCameraConfig(),
    }


@RobotConfig.register_subclass("doosan_a0509_ros")
@dataclass(kw_only=True)
class DoosanA0509RosConfig(RobotConfig):
    # Draccus 0.11 (used by LeRobot 0.6) cannot decode typing.Literal on
    # Python 3.12. Keep the CLI-facing field as a string and enforce the same
    # closed set in __post_init__ below.
    mode: str = "shadow_record"
    cameras: dict[str, CameraConfig] = field(default_factory=_default_cameras)
    require_camera: bool = True
    front_camera_key: str = "front"

    joint_states_topic: str = "/dsr01/joint_states"
    actual_tcp_position_topic: str = "/rt_topic/actual_tcp_position"
    robot_state_topic: str = "/rt_topic/robot_state"
    solution_space_topic: str = "/rt_topic/solution_space"
    gripper_commanded_state_topic: str = "/jrt_gripper/commanded_state"
    teleop_ready_topic: str = "/vr/teleop_ready"
    live_state_topic: str = "/vr/live_robot_output_enabled"

    lerobot_target_topic: str = "/control/lerobot/target_posx"
    lerobot_gripper_topic: str = "/control/lerobot/gripper_target"
    debug_action_topic: str = "/control/lerobot/debug_action"

    state_max_age_sec: float = 0.5
    state_startup_max_age_sec: float = 1.0
    state_startup_grace_reads: int = 30
    connect_timeout_sec: float = 3.0
    require_fresh_state_on_connect: bool = True
    camera_frame_max_age_ms: int = 500
    camera_startup_frame_max_age_ms: int = 1000
    camera_startup_grace_reads: int = 30
    allowed_robot_states: tuple[int, ...] = (1, 2)
    joint_names: tuple[str, ...] = (
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    )

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.mode not in {"shadow_record", "policy_dry_run", "policy_live"}:
            raise ValueError(f"unsupported robot mode: {self.mode}")
        if self.state_max_age_sec <= 0.0:
            raise ValueError("state_max_age_sec must be positive")
        if self.state_startup_max_age_sec < self.state_max_age_sec:
            raise ValueError(
                "state_startup_max_age_sec must be greater than or equal to "
                "state_max_age_sec"
            )
        if self.state_startup_grace_reads < 0:
            raise ValueError("state_startup_grace_reads must be non-negative")
        if self.connect_timeout_sec < 0.0:
            raise ValueError("connect_timeout_sec must be non-negative")
        if self.camera_frame_max_age_ms <= 0:
            raise ValueError("camera_frame_max_age_ms must be positive")
        if self.camera_startup_frame_max_age_ms < self.camera_frame_max_age_ms:
            raise ValueError(
                "camera_startup_frame_max_age_ms must be greater than or equal to "
                "camera_frame_max_age_ms"
            )
        if self.camera_startup_grace_reads < 0:
            raise ValueError("camera_startup_grace_reads must be non-negative")
        if len(self.joint_names) != 6:
            raise ValueError("joint_names must contain exactly six names")
        if self.require_camera and self.front_camera_key not in self.cameras:
            raise ValueError(
                f"required front camera {self.front_camera_key!r} is missing from cameras"
            )
