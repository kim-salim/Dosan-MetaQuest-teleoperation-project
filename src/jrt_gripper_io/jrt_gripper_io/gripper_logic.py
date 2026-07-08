"""Pure gripper command mapping helpers."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass


VALID_COMMANDS = {"close", "open", "stop"}
VALID_COMMAND_MODES = {"pulse", "level"}
OFF_VALUE = 0
ON_VALUE = 1


@dataclass(frozen=True)
class ToolDoStep:
    index: int
    value: int
    delay_after_sec: float = 0.0


def is_button_pressed(buttons: Sequence[int | float], index: int) -> bool:
    """Return True when a Joy button index exists and has a non-zero value."""
    if index < 0 or index >= len(buttons):
        return False
    return bool(buttons[index])


def command_from_buttons(
    buttons: Sequence[int | float],
    a_button_index: int = 0,
    b_button_index: int = 1,
) -> str:
    """Map Quest A/B button state to a gripper command string."""
    a_pressed = is_button_pressed(buttons, int(a_button_index))
    b_pressed = is_button_pressed(buttons, int(b_button_index))

    if a_pressed and not b_pressed:
        return "close"
    if b_pressed and not a_pressed:
        return "open"
    return "stop"


def normalize_command(command: str) -> str:
    """Normalize command strings; unknown values are treated as stop."""
    value = (command or "").strip().lower()
    if value in VALID_COMMANDS:
        return value
    return "stop"


def normalize_command_mode(command_mode: str) -> str:
    """Normalize command mode strings; unknown values fall back to pulse."""
    value = (command_mode or "").strip().lower()
    if value in VALID_COMMAND_MODES:
        return value
    return "pulse"


def plan_tool_do_command(
    command: str,
    close_do_index: int,
    open_do_index: int,
    active_value: int = ON_VALUE,
    inactive_value: int = OFF_VALUE,
    command_mode: str = "pulse",
) -> list[tuple[int, int]]:
    """Return ordered Tool DO writes for a gripper command."""
    return [
        (step.index, step.value)
        for step in plan_tool_do_sequence(
            command,
            close_do_index,
            open_do_index,
            active_value,
            inactive_value,
            command_mode=command_mode,
        )
    ]


def plan_tool_do_sequence(
    command: str,
    close_do_index: int,
    open_do_index: int,
    active_value: int = ON_VALUE,
    inactive_value: int = OFF_VALUE,
    *,
    command_mode: str = "pulse",
    pulse_sec: float = 0.20,
    interlock_sec: float = 0.05,
) -> list[ToolDoStep]:
    """Return ordered Tool DO writes for a gripper command.

    The order intentionally turns the opposite direction OFF before turning the
    requested direction ON.
    """
    close_index = int(close_do_index)
    open_index = int(open_do_index)
    active = int(active_value)
    inactive = int(inactive_value)
    normalized = normalize_command(command)
    mode = normalize_command_mode(command_mode)
    pulse_delay = max(0.0, float(pulse_sec))
    interlock_delay = max(0.0, float(interlock_sec))

    if normalized != "stop" and close_index == open_index:
        raise ValueError("close_do_index and open_do_index must be different")

    if normalized == "stop":
        return [
            ToolDoStep(close_index, inactive),
            ToolDoStep(open_index, inactive),
        ]

    if normalized == "close":
        steps = [
            ToolDoStep(open_index, inactive, interlock_delay),
            ToolDoStep(close_index, active),
        ]
        if mode == "pulse":
            steps[1] = ToolDoStep(close_index, active, pulse_delay)
            steps.append(ToolDoStep(close_index, inactive))
        return steps

    if normalized == "open":
        steps = [
            ToolDoStep(close_index, inactive, interlock_delay),
            ToolDoStep(open_index, active),
        ]
        if mode == "pulse":
            steps[1] = ToolDoStep(open_index, active, pulse_delay)
            steps.append(ToolDoStep(open_index, inactive))
        return steps

    return [
        ToolDoStep(close_index, inactive),
        ToolDoStep(open_index, inactive),
    ]
