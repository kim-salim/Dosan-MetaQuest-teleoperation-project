"""Pure diagnostic state transitions for the JRT Tool I/O driver."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class GripperDriverDiagnostics:
    accepted_command: str | None = None
    commanded_state: float | None = None
    busy: bool = False
    last_command_ok: bool = False

    def accept(self, command: str) -> None:
        self._validate_command(command)
        self.accepted_command = command
        self.busy = True
        self.last_command_ok = False

    def complete(
        self,
        command: str,
        *,
        update_commanded_state: bool = True,
        preserve_failure: bool = False,
    ) -> None:
        self._validate_command(command)
        self.busy = False
        self.last_command_ok = not preserve_failure
        if not update_commanded_state:
            return
        if command == "open":
            self.commanded_state = 0.0
        elif command == "close":
            self.commanded_state = 1.0

    def fail(self) -> None:
        self.busy = False
        self.last_command_ok = False

    @staticmethod
    def _validate_command(command: str) -> None:
        if command not in {"open", "close", "stop"}:
            raise ValueError(f"unsupported gripper command: {command}")


def commanded_state_for_command(command: str) -> float | None:
    if command == "open":
        return 0.0
    if command == "close":
        return 1.0
    if command == "stop":
        return None
    raise ValueError(f"unsupported gripper command: {command}")
