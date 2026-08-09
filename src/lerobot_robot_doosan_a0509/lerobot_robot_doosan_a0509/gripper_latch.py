"""Commanded-state latch shared by the MetaQuest adapter tests and callbacks."""

from __future__ import annotations

import math


class GripperLatch:
    def __init__(self, initial: float | None = None) -> None:
        self.value: float | None = None
        if initial is not None:
            self.set_commanded_state(initial)

    def set_commanded_state(self, value: float) -> float:
        numeric = float(value)
        if not math.isfinite(numeric) or not 0.0 <= numeric <= 1.0:
            raise ValueError(f"gripper commanded state must be in [0, 1], got {numeric}")
        self.value = 1.0 if numeric >= 0.5 else 0.0
        return self.value

    def update_command(self, command: str) -> float | None:
        normalized = str(command).strip().lower()
        if normalized == "open":
            self.value = 0.0
        elif normalized == "close":
            self.value = 1.0
        elif normalized != "stop":
            raise ValueError(f"unsupported gripper command: {command}")
        return self.value
