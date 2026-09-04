"""Fail-fast entry point for externally planned multi-stage Task-C V2."""

from __future__ import annotations

import sys
from collections.abc import Sequence

from lerobot_robot_doosan_a0509.act_async_rollout import (
    install_act_async_chunk_compat,
    require_async_fifo_arguments,
)
from lerobot_robot_doosan_a0509.runtime_scheduling import (
    pin_current_thread_from_env,
)
from lerobot_robot_doosan_a0509.task_c_live_rollout import (
    install_task_c_live_strategy,
)
from lerobot_robot_doosan_a0509.task_c_live_v2_rollout import (
    install_task_c_live_v2_strategy,
)
from lerobot_robot_doosan_a0509.task_c_multi_live_v2_rollout import (
    install_task_c_multi_live_v2_strategy,
)


def _argument_value(arguments: Sequence[str], name: str) -> str | None:
    prefix = f"{name}="
    for index, argument in enumerate(arguments):
        if argument.startswith(prefix):
            return argument[len(prefix) :]
        if argument == name and index + 1 < len(arguments):
            return arguments[index + 1]
    return None


def require_task_c_multi_live_v2_arguments(arguments: Sequence[str]) -> None:
    required = {
        "--strategy.type": "task_c_multi_live_v2",
        "--strategy.handoff_mode": "async_window_v2",
        "--strategy.acknowledge_uncertified_manifest": "true",
        "--robot.mode": "policy_live",
        "--return_to_initial_position": "false",
    }
    for name, expected in required.items():
        actual = _argument_value(arguments, name)
        if actual is None or actual.lower() != expected:
            raise ValueError(f"Task-C multi V2 requires {name}={expected}")
    endpoint_fallback = _argument_value(
        arguments,
        "--strategy.enable_endpoint_fallback",
    )
    if endpoint_fallback is None or endpoint_fallback.lower() not in {
        "true",
        "false",
    }:
        raise ValueError(
            "Task-C multi V2 requires an explicit boolean "
            "--strategy.enable_endpoint_fallback"
        )
    for required_path in (
        "--strategy.runtime_manifest",
        "--strategy.multi_stage_plan",
        "--strategy.event_jsonl_path",
        "--strategy.v2_trace_jsonl_path",
    ):
        if not _argument_value(arguments, required_path):
            raise ValueError(f"Task-C multi V2 requires {required_path}")
    require_async_fifo_arguments(list(arguments))


def main() -> None:
    try:
        require_task_c_multi_live_v2_arguments(sys.argv[1:])
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    main_cpu_set = pin_current_thread_from_env("LEROBOT_A0509_MAIN_CPU_SET")
    if main_cpu_set is not None:
        print(f"Task-C multi V2 main/camera threads pinned to CPUs {main_cpu_set}")
    install_act_async_chunk_compat()
    install_task_c_live_strategy()
    install_task_c_live_v2_strategy()
    install_task_c_multi_live_v2_strategy()

    from lerobot.scripts import lerobot_rollout

    lerobot_rollout.main()


if __name__ == "__main__":
    main()
