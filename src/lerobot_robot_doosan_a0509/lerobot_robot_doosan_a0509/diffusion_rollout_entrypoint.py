"""A0509 Diffusion rollout entry point with independent async chunking."""

from __future__ import annotations

import sys

# Import the config and implementation before Draccus parses ``--robot.type``.
from lerobot_robot_doosan_a0509.config_doosan_a0509_diffusion_ros import (  # noqa: F401
    DoosanA0509DiffusionRosConfig,
)
from lerobot_robot_doosan_a0509.diffusion_async_rollout import (
    install_diffusion_async_chunk_compat,
    install_diffusion_rtc_engine,
    load_and_validate_diffusion_checkpoint,
    require_diffusion_async_fifo_arguments,
)
from lerobot_robot_doosan_a0509.doosan_a0509_diffusion_ros import (  # noqa: F401
    DoosanA0509DiffusionRos,
)
from lerobot_robot_doosan_a0509.runtime_scheduling import pin_current_thread_from_env


def main() -> None:
    main_cpu_set = pin_current_thread_from_env("LEROBOT_A0509_MAIN_CPU_SET")
    if main_cpu_set is not None:
        print(f"A0509 Diffusion main/camera threads pinned to CPUs {main_cpu_set}")

    arguments = sys.argv[1:]
    try:
        require_diffusion_async_fifo_arguments(arguments)
        checkpoint = load_and_validate_diffusion_checkpoint(arguments)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    install_diffusion_async_chunk_compat()
    install_diffusion_rtc_engine()
    print(f"A0509 Diffusion checkpoint contract valid: {checkpoint}")

    from lerobot.scripts import lerobot_rollout

    lerobot_rollout.main()


if __name__ == "__main__":
    main()
