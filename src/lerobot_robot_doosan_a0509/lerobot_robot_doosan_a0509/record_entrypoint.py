"""LeRobot record entry point with optional A0509 process-isolated encoding."""

from __future__ import annotations

import os


def _environment_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{name} must be true or false, got {value!r}")


def main() -> None:
    if _environment_bool("LEROBOT_A0509_MULTIPROCESS_ENCODER", True):
        from lerobot_robot_doosan_a0509.multiprocess_streaming_encoder import (
            install_shared_memory_streaming_encoder,
        )

        install_shared_memory_streaming_encoder()

    from lerobot.scripts import lerobot_record

    if _environment_bool("LEROBOT_A0509_ABSOLUTE_DEADLINE", True):
        from lerobot_robot_doosan_a0509.deadline_record_loop import (
            install_absolute_deadline_record_loop,
        )

        install_absolute_deadline_record_loop(lerobot_record)

    episode_reset_hook = None
    if _environment_bool("LEROBOT_A0509_EPISODE_RESET", True):
        from lerobot_robot_doosan_a0509.episode_reset_orchestrator import (
            install_episode_reset_recording,
        )

        episode_reset_hook = install_episode_reset_recording(lerobot_record)

    try:
        lerobot_record.main()
    finally:
        if episode_reset_hook is not None:
            episode_reset_hook.close()


if __name__ == "__main__":
    main()
