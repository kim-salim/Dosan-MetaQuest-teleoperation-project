"""Fail-fast entry point for the read-only Task-C shadow rollout."""

from __future__ import annotations

import sys
from collections.abc import Sequence

from lerobot_robot_doosan_a0509.runtime_scheduling import pin_current_thread_from_env
from lerobot_robot_doosan_a0509.task_c_shadow_rollout import (
    install_task_c_shadow_strategy,
)


def _argument_value(arguments: Sequence[str], name: str) -> str | None:
    prefix = f"{name}="
    for index, argument in enumerate(arguments):
        if argument.startswith(prefix):
            return argument[len(prefix) :]
        if argument == name and index + 1 < len(arguments):
            return arguments[index + 1]
    return None


def require_read_only_shadow_arguments(arguments: Sequence[str]) -> None:
    required = {
        "--strategy.type": "task_c_shadow",
        "--robot.mode": "policy_shadow",
        "--return_to_initial_position": "false",
    }
    for name, expected in required.items():
        actual = _argument_value(arguments, name)
        if actual is None or actual.lower() != expected:
            raise ValueError(f"Task-C shadow requires {name}={expected}")


def main() -> None:
    try:
        require_read_only_shadow_arguments(sys.argv[1:])
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    main_cpu_set = pin_current_thread_from_env("LEROBOT_A0509_MAIN_CPU_SET")
    if main_cpu_set is not None:
        print(f"Task-C shadow main/camera threads pinned to CPUs {main_cpu_set}")
    install_task_c_shadow_strategy()

    from lerobot.scripts import lerobot_rollout

    lerobot_rollout.main()


if __name__ == "__main__":
    main()
