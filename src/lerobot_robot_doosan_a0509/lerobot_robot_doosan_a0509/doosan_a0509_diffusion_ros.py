"""A0509 robot adapter that changes only the Diffusion camera observation."""

from __future__ import annotations

import math
import os
from collections import deque
from typing import Any

from std_msgs.msg import Bool, Float64, Float64MultiArray

from lerobot_robot_doosan_a0509.act_async_rollout import (
    configure_policy_live_queue as configure_act_policy_live_queue,
)
from lerobot_robot_doosan_a0509.config_doosan_a0509_diffusion_ros import (
    DoosanA0509DiffusionRosConfig,
)
from lerobot_robot_doosan_a0509.diffusion_camera_adapter import (
    letterbox_rgb_image,
)
from lerobot_robot_doosan_a0509.diffusion_async_rollout import (
    configure_diffusion_policy_live_queue,
    diffusion_policy_live_queue_ready,
    notify_diffusion_policy_live_state,
)
from lerobot_robot_doosan_a0509.doosan_a0509_ros import DoosanA0509Ros
from lerobot_robot_doosan_a0509.topic_cache import TopicUnavailableError


class DiffusionGripperDecisionFilter:
    """Detect one grasp event and latch closed for the current Live session."""

    def __init__(
        self,
        *,
        support_threshold: float = 0.25,
        close_threshold: float = 0.4,
        window_ticks: int = 4,
        support_ticks: int = 2,
    ) -> None:
        if (
            not math.isfinite(support_threshold)
            or not math.isfinite(close_threshold)
            or not 0.0 <= support_threshold < close_threshold <= 1.0
        ):
            raise ValueError(
                "Diffusion gripper thresholds must satisfy "
                "0 <= support < close <= 1"
            )
        if window_ticks <= 0:
            raise ValueError("Diffusion gripper window_ticks must be positive")
        if not 0 < support_ticks <= window_ticks:
            raise ValueError(
                "Diffusion gripper support_ticks must be in [1, window_ticks]"
            )
        self.support_threshold = float(support_threshold)
        self.close_threshold = float(close_threshold)
        self.window_ticks = int(window_ticks)
        self.support_ticks = int(support_ticks)
        self.state: float | None = None
        self._window: deque[float] = deque(maxlen=self.window_ticks)

    def reset(self, initial_state: float | None = None) -> None:
        self.state = (
            None if initial_state is None else float(float(initial_state) >= 0.5)
        )
        self._window.clear()

    def update(self, target: float) -> tuple[float, bool]:
        value = float(target)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError(
                f"Diffusion gripper target must be finite and in [0, 1], got {value}"
            )
        if self.state is None:
            # Before the Live-state callback arrives, use the physical commanded
            # state when available. Default-open is the safer fallback.
            self.state = 0.0

        self._window.append(value)
        if self.state == 1.0:
            return self.state, False

        has_close_peak = any(
            sample > self.close_threshold for sample in self._window
        )
        support_count = sum(
            sample >= self.support_threshold for sample in self._window
        )
        if has_close_peak and support_count >= self.support_ticks:
            self.state = 1.0
            return self.state, True
        return self.state, False


class DoosanA0509DiffusionRos(DoosanA0509Ros):
    """Reuse the verified robot/MUX path and letterbox ZED RGB for Diffusion."""

    config_class = DoosanA0509DiffusionRosConfig
    name = "doosan_a0509_diffusion_ros"

    def __init__(self, config: DoosanA0509DiffusionRosConfig):
        super().__init__(config)
        self.config = config
        self._diffusion_gripper_filter = DiffusionGripperDecisionFilter(
            support_threshold=float(
                os.environ.get(
                    "LEROBOT_A0509_DIFFUSION_GRIPPER_SUPPORT_THRESHOLD", "0.25"
                )
            ),
            close_threshold=float(
                os.environ.get(
                    "LEROBOT_A0509_DIFFUSION_GRIPPER_CLOSE_THRESHOLD", "0.4"
                )
            ),
            window_ticks=int(
                os.environ.get("LEROBOT_A0509_DIFFUSION_GRIPPER_WINDOW_TICKS", "4")
            ),
            support_ticks=int(
                os.environ.get("LEROBOT_A0509_DIFFUSION_GRIPPER_SUPPORT_TICKS", "2")
            ),
        )
        self._last_diffusion_live_state: bool | None = None
        self._raw_gripper_pub = None

    @property
    def observation_features(self) -> dict[str, type | tuple[int, int, int]]:
        features = dict(super().observation_features)
        features[self.config.diffusion_zed_camera_key] = (
            self.config.diffusion_image_height,
            self.config.diffusion_image_width,
            3,
        )
        return features

    def get_observation(self) -> dict[str, Any]:
        observation = super().get_observation()
        camera_key = self.config.diffusion_zed_camera_key
        observation[camera_key] = letterbox_rgb_image(
            observation[camera_key],
            width=self.config.diffusion_image_width,
            height=self.config.diffusion_image_height,
        )
        return observation

    def send_action(self, action: dict[str, Any]) -> dict[str, float]:
        """Apply Diffusion-only gripper confirmation before the proven MUX path."""

        if self.config.mode == "shadow_record":
            return super().send_action(action)
        sanitized = self._validate_action(action)
        if self._diffusion_gripper_filter.state is None:
            try:
                initial_state = float(
                    self.cache.require_present("gripper_commanded_state").value
                )
            except (TopicUnavailableError, TypeError, ValueError):
                initial_state = None
            self._diffusion_gripper_filter.reset(initial_state)
        raw_target = sanitized["gripper_target"]
        if self._raw_gripper_pub is not None:
            self._raw_gripper_pub.publish(Float64(data=raw_target))
        filtered_target, changed = self._diffusion_gripper_filter.update(raw_target)
        sanitized["gripper_target"] = filtered_target
        if changed and self._node is not None:
            self._node.get_logger().info(
                "Diffusion gripper grasp event confirmed and latched closed: "
                "raw_target=%.6f window_ticks=%d support_ticks=%d "
                "support_threshold=%.3f close_threshold=%.3f"
                % (
                    raw_target,
                    self._diffusion_gripper_filter.window_ticks,
                    self._diffusion_gripper_filter.support_ticks,
                    self._diffusion_gripper_filter.support_threshold,
                    self._diffusion_gripper_filter.close_threshold,
                )
            )
        return super().send_action(sanitized)

    def _create_ros_entities(self) -> None:
        """Create the proven ROS publishers but bind the Diffusion Live gate."""

        super()._create_ros_entities()
        self._raw_gripper_pub = self._node.create_publisher(
            Float64,
            "/control/lerobot/diffusion_gripper_raw",
            10,
        )
        # The base implementation enables the ACT-local gate. No ACT queue is
        # used in this process, so explicitly leave it disabled and configure
        # only the independent Diffusion generation state.
        configure_act_policy_live_queue(False)
        is_live = self.config.mode == "policy_live"
        configure_diffusion_policy_live_queue(is_live)
        if is_live:
            notify_diffusion_policy_live_state(False)

    def _on_live_state(self, message: Bool) -> None:
        enabled = bool(message.data)
        self.cache.update("live_state", enabled)
        if self.config.mode == "policy_live":
            if enabled != self._last_diffusion_live_state:
                # A close decision is latched only inside one Live session.
                # Start every newly armed trial open; the verified common
                # gripper path performs the physical open on its first target.
                self._diffusion_gripper_filter.reset(0.0 if enabled else None)
                self._last_diffusion_live_state = enabled
            notify_diffusion_policy_live_state(enabled)

    def _publish_policy_hold_target(self) -> None:
        """Hold actual TCP until the current Live generation has a fresh chunk."""

        if self.config.mode != "policy_live" or self._target_pub is None:
            return
        queue_ready = diffusion_policy_live_queue_ready()
        if self._policy_ready_pub is not None:
            self._policy_ready_pub.publish(Bool(data=queue_ready))
        try:
            live_enabled = bool(self.cache.require_present("live_state").value)
            if live_enabled and queue_ready:
                return
            tcp = self.cache.require_present("actual_tcp_position").value
            robot_state = int(
                round(float(self.cache.require_present("robot_state").value))
            )
            if robot_state not in self.config.allowed_robot_states:
                return
        except (TopicUnavailableError, TypeError, ValueError):
            return

        message = Float64MultiArray()
        message.data = list(tcp)
        self._target_pub.publish(message)
        self.hold_publish_count += 1

    def _disconnect_partial(self) -> None:
        if self._node is not None:
            self._node.get_logger().info(
                "A0509 Diffusion publish metrics: mode=%s debug=%d live=%d hold=%d"
                % (
                    self.config.mode,
                    self.debug_publish_count,
                    self.live_publish_count,
                    self.hold_publish_count,
                )
            )
        if self.config.mode == "policy_live":
            notify_diffusion_policy_live_state(False)
        configure_diffusion_policy_live_queue(False)
        self._raw_gripper_pub = None
        super()._disconnect_partial()
