#!/usr/bin/env python3
"""Validate the derived three-camera A0509 Diffusion dataset."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from lerobot.datasets import LeRobotDataset


CAMERA_KEYS = (
    "observation.images.front",
    "observation.images.side",
    "observation.images.zed_rgb",
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--expected-episodes", type=int, default=30)
    parser.add_argument("--expected-frames", type=int, default=31500)
    args = parser.parse_args()

    root = args.root.expanduser().resolve()
    marker = root / "DIFFUSION_DATASET_INCOMPLETE"
    if marker.exists():
        raise SystemExit(f"Dataset conversion is incomplete: {marker}")

    info = json.loads((root / "meta" / "info.json").read_text())
    if info["total_episodes"] != args.expected_episodes:
        raise AssertionError((info["total_episodes"], args.expected_episodes))
    if info["total_frames"] != args.expected_frames:
        raise AssertionError((info["total_frames"], args.expected_frames))
    for key in CAMERA_KEYS:
        if info["features"][key]["shape"] != [480, 640, 3]:
            raise AssertionError(f"Unexpected shape for {key}: {info['features'][key]['shape']}")

    dataset = LeRobotDataset(repo_id=args.repo_id, root=root)
    if dataset.num_episodes != args.expected_episodes or dataset.num_frames != args.expected_frames:
        raise AssertionError((dataset.num_episodes, dataset.num_frames))

    sample_indices = (0, dataset.num_frames // 2, dataset.num_frames - 1)
    samples = []
    for index in sample_indices:
        frame = dataset[index]
        camera_shapes = {}
        for key in CAMERA_KEYS:
            shape = tuple(frame[key].shape)
            if shape != (3, 480, 640):
                raise AssertionError(f"Decoded {key} shape at index {index}: {shape}")
            camera_shapes[key] = shape
        state = np.asarray(frame["observation.state"])
        action = np.asarray(frame["action"])
        if state.shape != (13,) or not np.isfinite(state).all():
            raise AssertionError(f"Invalid state at index {index}: {state.shape}")
        if action.shape != (7,) or not np.isfinite(action).all():
            raise AssertionError(f"Invalid action at index {index}: {action.shape}")
        samples.append({"index": index, "camera_shapes": camera_shapes})

    print(
        json.dumps(
            {
                "status": "valid",
                "root": str(root),
                "repo_id": args.repo_id,
                "episodes": dataset.num_episodes,
                "frames": dataset.num_frames,
                "fps": dataset.fps,
                "samples": samples,
            },
            indent=2,
            default=list,
        )
    )


if __name__ == "__main__":
    main()
