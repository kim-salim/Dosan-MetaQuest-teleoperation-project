"""A0509 rollout entry point with ACT asynchronous FIFO prefetch."""

from __future__ import annotations

import sys

from lerobot_robot_doosan_a0509.act_async_rollout import (
    install_act_async_chunk_compat,
    require_async_fifo_arguments,
)
from lerobot_robot_doosan_a0509.runtime_scheduling import pin_current_thread_from_env


def main() -> None:
    main_cpu_set = pin_current_thread_from_env("LEROBOT_A0509_MAIN_CPU_SET")
    if main_cpu_set is not None:
        print(f"A0509 rollout main/camera threads pinned to CPUs {main_cpu_set}")
    try:
        require_async_fifo_arguments(sys.argv[1:])
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    install_act_async_chunk_compat()

    from lerobot.scripts import lerobot_rollout

    lerobot_rollout.main()


if __name__ == "__main__":
    main()
