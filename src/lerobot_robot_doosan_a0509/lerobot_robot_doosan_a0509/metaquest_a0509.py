"""LeRobot Teleoperator that consumes the existing MetaQuest mapper outputs."""

from __future__ import annotations

import math
import re
import time
from typing import Any, Iterable

from lerobot.teleoperators.teleoperator import Teleoperator
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray

from lerobot_robot_doosan_a0509.config_metaquest_a0509 import MetaQuestA0509Config
from lerobot_robot_doosan_a0509.gripper_latch import GripperLatch
from lerobot_robot_doosan_a0509.ros_runtime import RosRuntime
from lerobot_robot_doosan_a0509.topic_cache import TopicCache, TopicUnavailableError


ACTION_KEYS = (
    "target_x_mm",
    "target_y_mm",
    "target_z_mm",
    "target_o1_deg",
    "target_o2_deg",
    "target_o3_deg",
    "gripper_target",
)


def _finite_posx(values: Iterable[float]) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != 6:
        raise ValueError(f"MetaQuest target_posx must contain 6 values, got {len(result)}")
    if any(not math.isfinite(value) for value in result):
        raise ValueError(f"MetaQuest target_posx contains non-finite values: {result}")
    return result


def _safe_node_suffix(value: str | None) -> str:
    rendered = "default" if value is None else str(value)
    return re.sub(r"[^a-zA-Z0-9_]", "_", rendered)


class MetaQuestA0509(Teleoperator):
    config_class = MetaQuestA0509Config
    name = "metaquest_a0509"

    def __init__(self, config: MetaQuestA0509Config):
        super().__init__(config)
        self.config = config
        self.cache = TopicCache()
        self.gripper_latch = GripperLatch()
        self._runtime: RosRuntime | None = None
        self._node = None
        self._connected = False
        self._subscriptions: list[Any] = []

    @property
    def action_features(self) -> dict[str, type]:
        return {key: float for key in ACTION_KEYS}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        if not self.config.require_calibration:
            return True
        sample = self.cache.sample("calibration_valid")
        return sample is not None and bool(sample.value)

    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            raise RuntimeError(f"{self} is already connected")
        self.cache = TopicCache()
        self._runtime = RosRuntime.acquire()
        try:
            self._node = self._runtime.create_node(
                f"lerobot_metaquest_a0509_{_safe_node_suffix(self.id)}"
            )
            self._create_ros_entities()
            if self.config.require_fresh_action_on_connect:
                self._wait_for_required_input(self.config.connect_timeout_sec)
            elif calibrate and self.config.require_calibration:
                self._wait_for_calibration(self.config.connect_timeout_sec)
            self._connected = True
        except Exception:
            self._disconnect_partial()
            raise

    def calibrate(self) -> None:
        if not self.is_calibrated:
            raise RuntimeError(
                "MetaQuest XY calibration is invalid. Run the calibration-only GUI "
                "and complete Calibrate XY +X before recording."
            )

    def configure(self) -> None:
        return None

    def get_action(self) -> dict[str, float]:
        if not self._connected:
            raise RuntimeError(f"{self} is not connected")
        now = time.monotonic()
        if self.config.require_calibration and not self.is_calibrated:
            raise RuntimeError(
                "MetaQuest action rejected because XY calibration is invalid"
            )
        target = self.cache.require(
            "target_posx", max_age_sec=self.config.max_age_sec, now=now
        ).value
        heartbeat = self.cache.require(
            "valid_pose_heartbeat", max_age_sec=self.config.max_age_sec, now=now
        )
        del heartbeat
        if self.config.allow_latched_state_in_shadow_record:
            ready = self.cache.require_present("teleop_ready").value
            gripper = self.cache.require_present("gripper_commanded_state").value
        else:
            ready = self.cache.require(
                "teleop_ready", max_age_sec=self.config.max_age_sec, now=now
            ).value
            gripper = self.cache.require(
                "gripper_commanded_state", max_age_sec=self.config.max_age_sec, now=now
            ).value
        if not bool(ready):
            raise RuntimeError("MetaQuest action rejected because teleop_ready=false")
        values = (*target, float(gripper))
        return dict(zip(ACTION_KEYS, values, strict=True))

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        del feedback
        return None

    def disconnect(self) -> None:
        self._disconnect_partial()

    def _create_ros_entities(self) -> None:
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscriptions = [
            self._node.create_subscription(
                Float64MultiArray,
                self.config.target_posx_topic,
                self._on_target_posx,
                10,
            ),
            self._node.create_subscription(
                Float64,
                self.config.gripper_commanded_state_topic,
                self._on_gripper_commanded_state,
                state_qos,
            ),
            self._node.create_subscription(
                Empty,
                self.config.heartbeat_topic,
                self._on_valid_pose_heartbeat,
                10,
            ),
            self._node.create_subscription(
                Bool,
                self.config.calibration_valid_topic,
                self._on_calibration_valid,
                state_qos,
            ),
            self._node.create_subscription(
                Bool,
                self.config.teleop_ready_topic,
                self._on_teleop_ready,
                state_qos,
            ),
        ]

    def _on_target_posx(self, message: Float64MultiArray) -> None:
        try:
            target = _finite_posx(message.data)
        except (TypeError, ValueError) as exc:
            self._node.get_logger().warning(f"ignored invalid MetaQuest target: {exc}")
            return
        self.cache.update("target_posx", target)

    def _on_gripper_commanded_state(self, message: Float64) -> None:
        try:
            state = self.gripper_latch.set_commanded_state(message.data)
        except (TypeError, ValueError) as exc:
            self._node.get_logger().warning(f"ignored invalid gripper commanded state: {exc}")
            return
        self.cache.update("gripper_commanded_state", state)

    def _on_valid_pose_heartbeat(self, _message: Empty) -> None:
        self.cache.update("valid_pose_heartbeat", True)

    def _on_calibration_valid(self, message: Bool) -> None:
        self.cache.update("calibration_valid", bool(message.data))

    def _on_teleop_ready(self, message: Bool) -> None:
        self.cache.update("teleop_ready", bool(message.data))

    def _wait_for_required_input(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        motion_inputs = ("target_posx", "valid_pose_heartbeat")
        state_inputs = ("gripper_commanded_state", "teleop_ready")
        last_error = ""
        while True:
            now = time.monotonic()
            try:
                for key in motion_inputs:
                    self.cache.require(key, max_age_sec=self.config.max_age_sec, now=now)
                for key in state_inputs:
                    if self.config.allow_latched_state_in_shadow_record:
                        self.cache.require_present(key)
                    else:
                        self.cache.require(
                            key,
                            max_age_sec=self.config.max_age_sec,
                            now=now,
                        )
                if self.config.require_calibration and not self.is_calibrated:
                    raise TopicUnavailableError(
                        "MetaQuest XY calibration state is missing or invalid"
                    )
                return
            except TopicUnavailableError as exc:
                last_error = str(exc)
            if now >= deadline:
                raise RuntimeError(
                    "required MetaQuest input was not fresh within "
                    f"{timeout_sec:.2f}s: {last_error}"
                )
            time.sleep(0.02)

    def _wait_for_calibration(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while not self.is_calibrated:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "MetaQuest XY calibration is missing or invalid. Run the "
                    "calibration-only GUI before connecting the teacher teleoperator."
                )
            time.sleep(0.02)

    def _disconnect_partial(self) -> None:
        runtime = self._runtime
        node = self._node
        self._runtime = None
        self._node = None
        self._subscriptions = []
        self._connected = False
        if runtime is not None:
            runtime.release_node(node)


MetaQuestA0509Teleoperator = MetaQuestA0509
