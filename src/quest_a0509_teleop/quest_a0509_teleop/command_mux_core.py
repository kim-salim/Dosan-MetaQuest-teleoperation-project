"""ROS-independent state machine for the A0509 command multiplexer."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Iterable


class ControlSource(str, Enum):
    DISABLED = "DISABLED"
    METAQUEST = "METAQUEST"
    LEROBOT = "LEROBOT"

    @classmethod
    def parse(cls, value: "ControlSource | str") -> "ControlSource":
        if isinstance(value, cls):
            return value
        return cls(str(value).strip().upper())


@dataclass(frozen=True)
class MuxDecision:
    accepted: bool
    reason: str
    target_posx: tuple[float, ...] | None = None
    gripper_command: str | None = None
    emit_heartbeat: bool = False
    source_changed: bool = False
    timed_out: bool = False
    disable_and_hold: bool = False


def validate_posx(values: Iterable[float]) -> tuple[float, ...]:
    posx = tuple(float(value) for value in values)
    if len(posx) != 6:
        raise ValueError(f"target_posx must contain exactly 6 values, got {len(posx)}")
    if any(not math.isfinite(value) for value in posx):
        raise ValueError(f"target_posx contains non-finite values: {list(posx)}")
    return posx


def validate_gripper_target(value: float) -> float:
    target = float(value)
    if not math.isfinite(target):
        raise ValueError(f"gripper target must be finite, got {target}")
    if not 0.0 <= target <= 1.0:
        raise ValueError(f"gripper target must be in [0.0, 1.0], got {target}")
    return target


class CommandMuxCore:
    """Own source selection, freshness, validation, and gripper hysteresis."""

    def __init__(
        self,
        *,
        metaquest_timeout_sec: float = 1.0,
        lerobot_timeout_sec: float = 0.3,
        gripper_open_threshold: float = 0.3,
        gripper_close_threshold: float = 0.7,
        metaquest_gripper_min_pulse_sec: float = 1.50,
        require_metaquest_calibration: bool = True,
    ) -> None:
        if metaquest_timeout_sec <= 0.0 or lerobot_timeout_sec <= 0.0:
            raise ValueError("source timeouts must be positive")
        if not 0.0 <= gripper_open_threshold < gripper_close_threshold <= 1.0:
            raise ValueError("gripper thresholds must satisfy 0 <= open < close <= 1")
        if metaquest_gripper_min_pulse_sec < 0.0:
            raise ValueError("metaquest_gripper_min_pulse_sec must be >= 0")

        self.metaquest_timeout_sec = float(metaquest_timeout_sec)
        self.lerobot_timeout_sec = float(lerobot_timeout_sec)
        self.gripper_open_threshold = float(gripper_open_threshold)
        self.gripper_close_threshold = float(gripper_close_threshold)
        self.metaquest_gripper_min_pulse_sec = float(
            metaquest_gripper_min_pulse_sec
        )
        self.require_metaquest_calibration = bool(require_metaquest_calibration)

        self.source = ControlSource.DISABLED
        self.live_enabled = False
        self.teleop_ready = False
        self.metaquest_calibration_valid = False
        self.last_metaquest_valid_pose_time: float | None = None
        self.last_lerobot_target_time: float | None = None
        self.selected_at: float | None = None
        self.has_new_selected_target = False
        self.commanded_gripper_state: float | None = None
        self._last_gripper_output: str | None = None
        self._last_metaquest_gripper_motion_time: float | None = None

    def set_live_enabled(self, enabled: bool) -> MuxDecision | None:
        next_enabled = bool(enabled)
        previous_enabled = self.live_enabled
        self.live_enabled = next_enabled
        if previous_enabled and not next_enabled:
            self._last_gripper_output = "stop"
            self._last_metaquest_gripper_motion_time = None
            return MuxDecision(
                True,
                "live output disabled; gripper stop requested",
                gripper_command="stop",
            )
        return None

    def set_teleop_ready(self, ready: bool) -> None:
        self.teleop_ready = bool(ready)

    def update_commanded_gripper_state(self, value: float) -> None:
        state = validate_gripper_target(value)
        self.commanded_gripper_state = 1.0 if state >= 0.5 else 0.0

    def note_metaquest_valid_pose(self, now: float) -> None:
        self.last_metaquest_valid_pose_time = float(now)

    def set_metaquest_calibration_valid(
        self,
        valid: bool,
        *,
        now: float,
    ) -> MuxDecision | None:
        self.metaquest_calibration_valid = bool(valid)
        if (
            self.metaquest_calibration_valid
            or not self.require_metaquest_calibration
            or self.source is not ControlSource.METAQUEST
        ):
            return None

        previous_live = self.live_enabled
        self.source = ControlSource.DISABLED
        self.live_enabled = False
        self.has_new_selected_target = False
        self.selected_at = float(now)
        self._last_gripper_output = "stop"
        self._last_metaquest_gripper_motion_time = None
        return MuxDecision(
            True,
            (
                "MetaQuest calibration became invalid; source disabled"
                f" (live_was_enabled={previous_live})"
            ),
            gripper_command="stop",
            source_changed=True,
            disable_and_hold=True,
        )

    def select_source(
        self,
        requested: ControlSource | str,
        *,
        now: float,
    ) -> MuxDecision:
        try:
            next_source = ControlSource.parse(requested)
        except ValueError:
            return MuxDecision(False, f"unknown source: {requested}")

        if (
            next_source is ControlSource.METAQUEST
            and self.require_metaquest_calibration
            and not self.metaquest_calibration_valid
        ):
            return MuxDecision(
                False,
                "MetaQuest source selection rejected because XY calibration is invalid",
            )

        if next_source == self.source:
            if next_source is ControlSource.DISABLED:
                self.live_enabled = False
                return MuxDecision(
                    True,
                    "source already DISABLED; safety hold requested",
                    gripper_command="stop",
                    disable_and_hold=True,
                )
            return MuxDecision(True, f"source already {self.source.value}")

        if next_source is not ControlSource.DISABLED and self.live_enabled:
            return MuxDecision(
                False,
                "active source selection rejected while live robot output is enabled",
            )

        previous = self.source
        self.source = next_source
        if next_source is ControlSource.DISABLED:
            self.live_enabled = False
        self.selected_at = float(now)
        self.has_new_selected_target = False
        self.last_lerobot_target_time = None
        self._last_gripper_output = "stop"
        self._last_metaquest_gripper_motion_time = None
        return MuxDecision(
            True,
            f"source transition {previous.value}->{next_source.value}",
            gripper_command="stop",
            source_changed=True,
            disable_and_hold=next_source is ControlSource.DISABLED,
        )

    def receive_arm_target(
        self,
        source: ControlSource | str,
        values: Iterable[float],
        *,
        now: float,
    ) -> MuxDecision:
        try:
            source_value = ControlSource.parse(source)
            posx = validate_posx(values)
        except (TypeError, ValueError) as exc:
            return MuxDecision(False, str(exc))

        if source_value is ControlSource.LEROBOT:
            self.last_lerobot_target_time = float(now)

        if source_value is not self.source:
            return MuxDecision(
                False,
                f"ignored {source_value.value} target while source={self.source.value}",
            )
        if source_value is ControlSource.DISABLED:
            return MuxDecision(False, "target rejected while source=DISABLED")
        if (
            source_value is ControlSource.METAQUEST
            and self.require_metaquest_calibration
            and not self.metaquest_calibration_valid
        ):
            return MuxDecision(
                False,
                "MetaQuest target rejected because XY calibration is invalid",
            )
        if not self.teleop_ready:
            return MuxDecision(False, "target rejected because teleop_ready=false")
        if not self._source_is_fresh(source_value, now):
            return MuxDecision(False, f"{source_value.value} heartbeat is stale")

        self.has_new_selected_target = True
        return MuxDecision(
            True,
            f"forwarded {source_value.value} target",
            target_posx=posx,
            emit_heartbeat=True,
        )

    def receive_metaquest_gripper(self, command: str, *, now: float) -> MuxDecision:
        normalized = str(command).strip().lower()
        if normalized not in {"open", "close", "stop"}:
            return MuxDecision(False, f"invalid MetaQuest gripper command: {command!r}")
        if self.source is not ControlSource.METAQUEST:
            return MuxDecision(
                False,
                f"ignored MetaQuest gripper command while source={self.source.value}",
            )
        if not self._gripper_output_allowed(ControlSource.METAQUEST, now):
            return MuxDecision(
                False,
                "MetaQuest gripper output is not armed by a fresh arm target",
            )
        now_value = float(now)
        if normalized in {"open", "close"}:
            decision = self._emit_gripper_if_changed(normalized)
            if decision.gripper_command is not None:
                self._last_metaquest_gripper_motion_time = now_value
            return decision

        if self._last_metaquest_gripper_motion_time is not None:
            elapsed = now_value - self._last_metaquest_gripper_motion_time
            if elapsed < self.metaquest_gripper_min_pulse_sec:
                self._last_gripper_output = "stop"
                return MuxDecision(
                    True,
                    (
                        "MetaQuest release stop suppressed until the Tool DO "
                        "pulse can self-complete: "
                        f"{elapsed:.3f}s < "
                        f"{self.metaquest_gripper_min_pulse_sec:.3f}s"
                    ),
                )
        return self._emit_gripper_if_changed(normalized)

    def receive_lerobot_gripper(self, value: float, *, now: float) -> MuxDecision:
        try:
            target = validate_gripper_target(value)
        except (TypeError, ValueError) as exc:
            return MuxDecision(False, str(exc))
        if self.source is not ControlSource.LEROBOT:
            return MuxDecision(
                False,
                f"ignored LeRobot gripper target while source={self.source.value}",
            )
        if not self._gripper_output_allowed(ControlSource.LEROBOT, now):
            return MuxDecision(False, "LeRobot gripper output is not armed by a fresh arm target")

        if target > self.gripper_close_threshold:
            desired = "close"
            desired_state = 1.0
        elif target < self.gripper_open_threshold:
            desired = "open"
            desired_state = 0.0
        else:
            return MuxDecision(True, "LeRobot gripper target is inside hysteresis hold band")

        if (
            self.commanded_gripper_state is not None
            and self.commanded_gripper_state == desired_state
        ):
            return MuxDecision(True, f"gripper already commanded {desired}")
        return self._emit_gripper_if_changed(desired)

    def check_timeout(self, *, now: float) -> MuxDecision | None:
        if self.source is ControlSource.DISABLED:
            return None
        if self._source_is_fresh(self.source, now):
            return None

        timed_out_source = self.source
        previous_live = self.live_enabled
        self.source = ControlSource.DISABLED
        self.live_enabled = False
        self.has_new_selected_target = False
        self.selected_at = float(now)
        self._last_gripper_output = "stop"
        self._last_metaquest_gripper_motion_time = None
        return MuxDecision(
            True,
            (
                f"{timed_out_source.value} timeout; source disabled"
                f" (live_was_enabled={previous_live})"
            ),
            gripper_command="stop",
            source_changed=True,
            timed_out=True,
            disable_and_hold=True,
        )

    def _source_is_fresh(self, source: ControlSource, now: float) -> bool:
        now_value = float(now)
        if source is ControlSource.METAQUEST:
            timestamp = self.last_metaquest_valid_pose_time
            timeout = self.metaquest_timeout_sec
        elif source is ControlSource.LEROBOT:
            timestamp = self.last_lerobot_target_time
            timeout = self.lerobot_timeout_sec
        else:
            return False

        if timestamp is None:
            if self.selected_at is None:
                return False
            return now_value - self.selected_at <= timeout
        return now_value - timestamp <= timeout

    def _gripper_output_allowed(self, source: ControlSource, now: float) -> bool:
        return (
            self.source is source
            and self.live_enabled
            and self.teleop_ready
            and (
                source is not ControlSource.METAQUEST
                or not self.require_metaquest_calibration
                or self.metaquest_calibration_valid
            )
            and self.has_new_selected_target
            and self._source_is_fresh(source, now)
        )

    def _emit_gripper_if_changed(self, command: str) -> MuxDecision:
        if command == self._last_gripper_output:
            return MuxDecision(True, f"duplicate gripper {command} suppressed")
        self._last_gripper_output = command
        return MuxDecision(
            True,
            f"forwarded gripper {command}",
            gripper_command=command,
        )
