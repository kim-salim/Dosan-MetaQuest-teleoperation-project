#!/usr/bin/env python3
"""Exercise the Diffusion RTC worker at 30 Hz without ROS robot commands."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import torch

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies import make_pre_post_processors
from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig
from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.policies.rtc.configuration_rtc import RTCConfig
from lerobot_robot_doosan_a0509.diffusion_async_rollout import (
    A0509DiffusionRTCInferenceEngine,
    DiffusionActionQueue,
    install_diffusion_async_chunk_compat,
    validate_diffusion_policy_config,
)


STATE_NAMES = (
    "joint_1_rad.pos",
    "joint_2_rad.pos",
    "joint_3_rad.pos",
    "joint_4_rad.pos",
    "joint_5_rad.pos",
    "joint_6_rad.pos",
    "tcp_x_mm.pos",
    "tcp_y_mm.pos",
    "tcp_z_mm.pos",
    "tcp_o1_deg.pos",
    "tcp_o2_deg.pos",
    "tcp_o3_deg.pos",
    "gripper_commanded_state.pos",
)


def _hardware_features() -> dict[str, dict[str, object]]:
    features: dict[str, dict[str, object]] = {
        "observation.state": {
            "dtype": "float32",
            "shape": (13,),
            "names": list(STATE_NAMES),
        }
    }
    for camera in ("front", "side", "zed_rgb"):
        features[f"observation.images.{camera}"] = {
            "dtype": "image",
            "shape": (480, 640, 3),
            "names": ["height", "width", "channels"],
        }
    return features


def _synthetic_observation() -> dict[str, object]:
    state = (0.0,) * 6 + (420.0, 0.0, 420.0, 0.0, 150.0, 0.0, 0.0)
    observation: dict[str, object] = dict(zip(STATE_NAMES, state, strict=True))
    for index, camera in enumerate(("front", "side", "zed_rgb")):
        observation[camera] = np.full(
            (480, 640, 3),
            96 + index * 32,
            dtype=np.uint8,
        )
    return observation


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()
    if args.duration_sec <= 0.0 or args.fps <= 0.0:
        raise SystemExit("duration and fps must be positive")

    checkpoint = args.checkpoint.expanduser().resolve()
    config = PreTrainedConfig.from_pretrained(checkpoint, local_files_only=True)
    validate_diffusion_policy_config(config)
    assert isinstance(config, DiffusionConfig)
    config.device = "cuda"
    config.n_action_steps = 15

    policy = DiffusionPolicy.from_pretrained(
        checkpoint,
        config=config,
        local_files_only=True,
        strict=True,
    ).to("cuda")
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=config,
        pretrained_path=str(checkpoint),
        preprocessor_overrides={"device_processor": {"device": "cuda"}},
    )
    install_diffusion_async_chunk_compat(warmup_inferences=2)

    fake_robot = SimpleNamespace(robot_type="doosan_a0509_diffusion_ros")
    engine = A0509DiffusionRTCInferenceEngine(
        policy=policy,
        preprocessor=preprocessor,
        postprocessor=postprocessor,
        robot_wrapper=fake_robot,
        rtc_config=RTCConfig(enabled=False),
        hw_features=_hardware_features(),
        task="Pick up the blue block on the table",
        fps=args.fps,
        device="cuda",
        rtc_queue_threshold=9,
    )

    observation = _synthetic_observation()
    period = 1.0 / args.fps
    initial_empty_ticks = 0
    steady_empty_ticks = 0
    action_count = 0
    first_action_time: float | None = None
    startup_metrics: dict[str, int | float | None] | None = None
    started = time.monotonic()
    deadline = started
    latest_action: torch.Tensor | None = None

    engine.reset()
    engine.start()
    engine.resume()
    queue = engine.action_queue
    if not isinstance(queue, DiffusionActionQueue):
        raise RuntimeError(f"unexpected queue type: {type(queue).__name__}")
    try:
        while True:
            now = time.monotonic()
            if first_action_time is not None and now - first_action_time >= args.duration_sec:
                break
            if now - started > args.duration_sec + 10.0:
                raise TimeoutError("Diffusion async worker did not produce its first chunk")

            engine.notify_observation(observation)
            action = engine.get_action(None)
            if action is None:
                if first_action_time is None:
                    initial_empty_ticks += 1
                else:
                    steady_empty_ticks += 1
            else:
                if first_action_time is None:
                    first_action_time = now
                    startup_metrics = queue.metrics()
                latest_action = action
                action_count += 1

            deadline += period
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                deadline = time.monotonic()
    finally:
        engine.stop()

    metrics = queue.metrics()
    if engine.failed:
        raise RuntimeError("Diffusion async worker reported a fatal error")
    if latest_action is None or not torch.isfinite(latest_action).all():
        raise RuntimeError("Diffusion async worker produced no finite action")
    if steady_empty_ticks:
        raise RuntimeError(
            f"Diffusion action queue underrun after startup: {steady_empty_ticks} ticks"
        )
    if startup_metrics is None:
        raise RuntimeError("Diffusion async worker never reached steady execution")
    stale_action_delta = int(metrics["stale_action_drop_count"]) - int(
        startup_metrics["stale_action_drop_count"]
    )
    stale_inference_delta = int(metrics["stale_inference_drop_count"]) - int(
        startup_metrics["stale_inference_drop_count"]
    )
    if stale_action_delta or stale_inference_delta:
        raise RuntimeError(
            "Diffusion async worker dropped stale work after startup: "
            f"action_delta={stale_action_delta} inference_delta={stale_inference_delta}"
        )

    result = {
        "status": "valid",
        "mode": "offline_async_no_robot_commands",
        "checkpoint": str(checkpoint),
        "device": torch.cuda.get_device_name(0),
        "fps": args.fps,
        "duration_sec": args.duration_sec,
        "initial_empty_ticks": initial_empty_ticks,
        "steady_empty_ticks": steady_empty_ticks,
        "actions_consumed": action_count,
        "startup_metrics": startup_metrics,
        "queue_metrics": metrics,
        "latest_action": latest_action.detach().float().cpu().tolist(),
    }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
