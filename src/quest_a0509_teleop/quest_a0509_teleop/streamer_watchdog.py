"""ROS-independent safety watchdogs for the ServoL streamer."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class MuxHeartbeatWatchdog:
    required: bool = False
    timeout_sec: float = 1.0
    last_heartbeat_time: float | None = None

    def __post_init__(self) -> None:
        if self.timeout_sec <= 0.0:
            raise ValueError("mux heartbeat timeout must be positive")

    def mark(self, now: float) -> None:
        self.last_heartbeat_time = float(now)

    def is_fresh(self, now: float) -> bool:
        if not self.required:
            return True
        if self.last_heartbeat_time is None:
            return False
        return float(now) - self.last_heartbeat_time <= self.timeout_sec

    def enable_reject_reason(self, now: float) -> str | None:
        if self.is_fresh(now):
            return None
        if self.last_heartbeat_time is None:
            return "no selected-command heartbeat has been received"
        age = max(0.0, float(now) - self.last_heartbeat_time)
        return (
            f"selected-command heartbeat is stale: age={age:.3f}s "
            f"timeout={self.timeout_sec:.3f}s"
        )

    def should_disable_live(self, live_enabled: bool, now: float) -> bool:
        return bool(live_enabled) and not self.is_fresh(now)


@dataclass
class RobotStateWatchdog:
    required: bool = True
    safe_states: tuple[int, ...] = (1, 2)
    timeout_sec: float = 1.0
    last_state: int | None = None
    last_update_time: float | None = None

    def __post_init__(self) -> None:
        self.safe_states = tuple(int(state) for state in self.safe_states)
        if not self.safe_states:
            raise ValueError("safe robot states must not be empty")
        if self.timeout_sec <= 0.0:
            raise ValueError("robot state timeout must be positive")

    def mark(self, state: int, now: float) -> None:
        self.last_state = int(state)
        self.last_update_time = float(now)

    def enable_reject_reason(self, now: float) -> str | None:
        if not self.required:
            return None
        if self.last_state is None or self.last_update_time is None:
            return "no robot-state topic sample has been received"
        age = max(0.0, float(now) - self.last_update_time)
        if age > self.timeout_sec:
            return (
                f"robot-state topic sample is stale: age={age:.3f}s "
                f"timeout={self.timeout_sec:.3f}s"
            )
        if self.last_state not in self.safe_states:
            return f"robot_state={self.last_state} is not ready for ServoL RT"
        return None

    def should_disable_live(self, live_enabled: bool, now: float) -> bool:
        return bool(live_enabled) and self.enable_reject_reason(now) is not None
