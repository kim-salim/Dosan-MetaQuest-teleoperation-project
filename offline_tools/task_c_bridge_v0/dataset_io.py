"""Read-only loader for the audited LeRobot v3 A0509 datasets."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

from .trajectory_states import Trajectory


EXPECTED_STATE_PREFIX = (
    "joint_1_rad",
    "joint_2_rad",
    "joint_3_rad",
    "joint_4_rad",
    "joint_5_rad",
    "joint_6_rad",
    "tcp_x_mm",
    "tcp_y_mm",
    "tcp_z_mm",
    "tcp_o1_deg",
    "tcp_o2_deg",
    "tcp_o3_deg",
    "gripper_commanded_state",
)


def load_lerobot_trajectories(root: Path, label: str) -> tuple[list[Trajectory], dict]:
    info = json.loads((root / "meta/info.json").read_text(encoding="utf-8"))
    tasks_path = root / "meta/tasks.parquet"
    info["task_descriptions"] = (
        [str(row["task"]) for row in pq.read_table(tasks_path).to_pylist()]
        if tasks_path.exists()
        else []
    )
    state_feature = info["features"]["observation.state"]
    if tuple(state_feature["names"]) != EXPECTED_STATE_PREFIX:
        raise ValueError("unexpected observation.state schema; refusing positional guess")
    if int(info["fps"]) <= 0:
        raise ValueError("dataset FPS must be positive")
    data_paths = sorted(root.glob("data/chunk-*/file-*.parquet"))
    if not data_paths:
        raise FileNotFoundError(f"no parquet data found under {root}")
    table = pq.read_table(
        data_paths,
        columns=["episode_index", "frame_index", "timestamp", "observation.state"],
    )
    episodes = np.asarray(table["episode_index"].to_numpy(), dtype=np.int64)
    frames = np.asarray(table["frame_index"].to_numpy(), dtype=np.int64)
    timestamps = np.asarray(table["timestamp"].to_numpy(), dtype=np.float64)
    state = np.asarray(table["observation.state"].to_pylist(), dtype=np.float64)
    output: list[Trajectory] = []
    for episode in np.unique(episodes):
        mask = episodes == episode
        order = np.argsort(frames[mask], kind="stable")
        episode_state = state[mask][order]
        output.append(
            Trajectory(
                dataset=label,
                episode=int(episode),
                xyz_mm=episode_state[:, 6:9],
                timestamp_s=timestamps[mask][order],
                gripper_closed=episode_state[:, 12] >= 0.5,
                frame_index=frames[mask][order],
                # V0 preserves but never ranks with Doosan orientation payload.
                orientation_payload=episode_state[:, 9:12],
                metadata={
                    "fps": int(info["fps"]),
                    "orientation_representation": "doosan_posx_o1_o2_o3_deg",
                    "holding_field": None,
                    "contact_field": None,
                },
            )
        )
    return output, info
