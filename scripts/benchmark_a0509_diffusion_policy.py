#!/usr/bin/env python3
"""Load and benchmark an A0509 Diffusion checkpoint without robot commands."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from contextlib import nullcontext
from pathlib import Path
from typing import Any

import numpy as np
import torch

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies import make_pre_post_processors
from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig
from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot_robot_doosan_a0509.diffusion_camera_adapter import (
    letterbox_rgb_image,
)


def _percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    index = min(round((len(ordered) - 1) * probability), len(ordered) - 1)
    return ordered[index]


def _adapt_image(image: torch.Tensor, expected_shape: tuple[int, int, int]) -> torch.Tensor:
    if image.ndim != 3:
        raise ValueError(f"expected CHW image tensor, got {tuple(image.shape)}")
    if tuple(image.shape) == expected_shape:
        return image
    expected_channels, expected_height, expected_width = expected_shape
    if image.shape[0] != expected_channels:
        raise ValueError(
            f"camera channel mismatch: got {tuple(image.shape)}, expected {expected_shape}"
        )
    hwc = image.detach().cpu().permute(1, 2, 0).contiguous().numpy()
    adapted = letterbox_rgb_image(hwc, width=expected_width, height=expected_height)
    return torch.from_numpy(np.ascontiguousarray(adapted)).permute(2, 0, 1)


def _synthetic_observation(config: DiffusionConfig) -> dict[str, torch.Tensor]:
    batch: dict[str, torch.Tensor] = {
        "observation.state": torch.zeros(
            (1, config.n_obs_steps, *config.robot_state_feature.shape),
            dtype=torch.float32,
        )
    }
    for feature_name, feature in config.image_features.items():
        batch[feature_name] = torch.full(
            (1, config.n_obs_steps, *feature.shape),
            0.5,
            dtype=torch.float32,
        )
    return batch


def _dataset_observation(
    config: DiffusionConfig,
    *,
    root: Path,
    repo_id: str,
) -> dict[str, torch.Tensor]:
    from lerobot.datasets import LeRobotDataset

    dataset = LeRobotDataset(repo_id=repo_id, root=root)
    if dataset.num_frames < config.n_obs_steps:
        raise ValueError(
            f"dataset has {dataset.num_frames} frames but policy needs {config.n_obs_steps}"
        )
    frames = [dataset[index] for index in range(config.n_obs_steps)]
    batch: dict[str, torch.Tensor] = {
        "observation.state": torch.stack(
            [frame["observation.state"] for frame in frames], dim=0
        ).unsqueeze(0)
    }
    for feature_name, feature in config.image_features.items():
        images = [
            _adapt_image(frame[feature_name], tuple(feature.shape))
            for frame in frames
        ]
        batch[feature_name] = torch.stack(images, dim=0).unsqueeze(0)
    return batch


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--dataset-root", type=Path)
    parser.add_argument("--dataset-repo-id", default="local/a0509_blue_block_v1")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--warmup-runs", type=int, default=3)
    parser.add_argument("--runs", type=int, default=20)
    args = parser.parse_args()

    checkpoint = args.checkpoint.expanduser().resolve()
    if not (checkpoint / "model.safetensors").is_file():
        raise SystemExit(f"checkpoint is missing model.safetensors: {checkpoint}")
    if args.warmup_runs < 0 or args.runs <= 0:
        raise SystemExit("--warmup-runs must be non-negative and --runs must be positive")

    config = PreTrainedConfig.from_pretrained(checkpoint, local_files_only=True)
    if not isinstance(config, DiffusionConfig):
        raise SystemExit(
            f"expected a Diffusion checkpoint, got {type(config).__name__}"
        )
    config.device = args.device
    expected_camera_shapes = {
        feature_name: list(feature.shape)
        for feature_name, feature in config.image_features.items()
    }
    if len(expected_camera_shapes) != 3:
        raise SystemExit(f"expected three camera features, got {expected_camera_shapes}")
    if config.n_obs_steps != 2 or config.horizon != 16 or config.n_action_steps != 8:
        raise SystemExit(
            "unexpected temporal config: "
            f"n_obs_steps={config.n_obs_steps}, horizon={config.horizon}, "
            f"n_action_steps={config.n_action_steps}"
        )
    if config.noise_scheduler_type != "DDIM" or config.num_inference_steps != 5:
        raise SystemExit(
            "unexpected denoising config: "
            f"scheduler={config.noise_scheduler_type}, steps={config.num_inference_steps}"
        )

    policy = DiffusionPolicy.from_pretrained(
        checkpoint,
        config=config,
        local_files_only=True,
        strict=True,
    )
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=config,
        pretrained_path=str(checkpoint),
        preprocessor_overrides={"device_processor": {"device": args.device}},
    )

    if args.dataset_root is None:
        raw_observation = _synthetic_observation(config)
        observation_source = "synthetic_midpoint"
    else:
        raw_observation = _dataset_observation(
            config,
            root=args.dataset_root.expanduser().resolve(),
            repo_id=args.dataset_repo_id,
        )
        observation_source = str(args.dataset_root.expanduser().resolve())
    observation = preprocessor(raw_observation)

    device = torch.device(args.device)
    amp_context: Any = (
        torch.autocast(device_type=device.type) if config.use_amp else nullcontext()
    )

    @torch.inference_mode()
    def infer_once() -> tuple[torch.Tensor, torch.Tensor]:
        with amp_context:
            normalized_chunk = policy.predict_action_chunk(observation)
        physical_chunk = postprocessor(normalized_chunk)
        return normalized_chunk, physical_chunk

    for _ in range(args.warmup_runs):
        infer_once()
    if device.type == "cuda":
        torch.cuda.synchronize(device)
        torch.cuda.reset_peak_memory_stats(device)

    latencies_ms: list[float] = []
    normalized_chunk = physical_chunk = None
    for _ in range(args.runs):
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        started = time.perf_counter()
        normalized_chunk, physical_chunk = infer_once()
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        latencies_ms.append((time.perf_counter() - started) * 1000.0)

    assert normalized_chunk is not None and physical_chunk is not None
    expected_action_shape = (1, config.n_action_steps, config.action_feature.shape[0])
    if tuple(normalized_chunk.shape) != expected_action_shape:
        raise RuntimeError(
            f"unexpected normalized action shape {tuple(normalized_chunk.shape)}, "
            f"expected {expected_action_shape}"
        )
    if tuple(physical_chunk.shape) != expected_action_shape:
        raise RuntimeError(f"unexpected physical action shape {tuple(physical_chunk.shape)}")
    if not torch.isfinite(normalized_chunk).all() or not torch.isfinite(physical_chunk).all():
        raise RuntimeError("checkpoint produced non-finite actions")

    physical_cpu = physical_chunk.detach().float().cpu()
    result = {
        "status": "valid",
        "checkpoint": str(checkpoint),
        "observation_source": observation_source,
        "device": args.device,
        "cuda_device": torch.cuda.get_device_name(device) if device.type == "cuda" else None,
        "torch": torch.__version__,
        "amp": config.use_amp,
        "scheduler": config.noise_scheduler_type,
        "num_inference_steps": config.num_inference_steps,
        "n_obs_steps": config.n_obs_steps,
        "horizon": config.horizon,
        "n_action_steps": config.n_action_steps,
        "camera_shapes": expected_camera_shapes,
        "action_shape": list(physical_chunk.shape),
        "action_min": physical_cpu.amin(dim=(0, 1)).tolist(),
        "action_max": physical_cpu.amax(dim=(0, 1)).tolist(),
        "latency_ms": {
            "mean": statistics.mean(latencies_ms),
            "p50": _percentile(latencies_ms, 0.50),
            "p95": _percentile(latencies_ms, 0.95),
            "p99": _percentile(latencies_ms, 0.99),
            "max": max(latencies_ms),
        },
        "peak_cuda_memory_gib": (
            torch.cuda.max_memory_allocated(device) / math.pow(1024, 3)
            if device.type == "cuda"
            else None
        ),
    }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
