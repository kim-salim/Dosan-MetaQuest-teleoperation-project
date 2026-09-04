"""Task-agnostic helpers for semantic-phase trajectory standardization.

Raw frame indices are used only as private array addresses while loading an
episode. Aggregate outputs use semantic phase and physical Cartesian units;
this module never averages frame numbers.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from offline_tools.task_c_bridge_v0.trajectory_states import Trajectory


@dataclass(frozen=True)
class SemanticSegmentSpec:
    segment_id: str
    parent_subgoal: str
    semantic_label: str
    label_ko: str
    start_anchor: str
    end_anchor: str
    gripper_semantics: str
    manipulated_object: str | None
    entry_state: dict[str, str]
    exit_state: dict[str, str]
    evidence: tuple[str, ...]
    confidence: str


def detect_gripper_event_sequence(
    trajectory: Trajectory,
    expected_event_ids: Sequence[str],
) -> dict[str, int]:
    """Return private per-episode array indices for an audited C/O grammar."""

    closed = trajectory.gripper_closed
    events: list[tuple[str, int]] = []
    close_count = 0
    open_count = 0
    for index in range(1, len(closed)):
        if not bool(closed[index - 1]) and bool(closed[index]):
            close_count += 1
            events.append((f"C{close_count}", index))
        elif bool(closed[index - 1]) and not bool(closed[index]):
            open_count += 1
            events.append((f"O{open_count}", index))
    actual_ids = tuple(event_id for event_id, _ in events)
    expected_ids = tuple(expected_event_ids)
    if actual_ids != expected_ids:
        raise ValueError(
            f"episode {trajectory.episode} event grammar mismatch: "
            f"expected={expected_ids} actual={actual_ids}"
        )
    return dict(events)


def resolve_anchor_index(
    anchor: str,
    trajectory: Trajectory,
    events: dict[str, int],
) -> int:
    if anchor == "START":
        return 0
    if anchor == "END":
        return len(trajectory.xyz_mm) - 1
    if anchor not in events:
        raise KeyError(f"unknown semantic anchor: {anchor}")
    return int(events[anchor])


def resample_cartesian_span(
    trajectory: Trajectory,
    start_index: int,
    end_index: int,
    phase_points: int,
) -> dict[str, Any]:
    """Standardize one span by Cartesian arc length, never by frame index."""

    if start_index < 0 or end_index >= len(trajectory.xyz_mm):
        raise ValueError("semantic span lies outside the episode")
    if start_index >= end_index:
        raise ValueError("semantic span must contain forward progress")
    xyz = np.asarray(trajectory.xyz_mm[start_index : end_index + 1], dtype=np.float64)
    increments = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    arc = np.concatenate(([0.0], np.cumsum(increments)))
    keep = np.concatenate(([True], np.diff(arc) > 1e-9))
    xyz = xyz[keep]
    arc = arc[keep]
    if len(arc) < 2 or arc[-1] <= 1e-9:
        raise ValueError(
            f"episode {trajectory.episode} has a degenerate semantic span"
        )
    source_phase = arc / arc[-1]
    phase = np.linspace(0.0, 1.0, phase_points)
    standardized_xyz = np.stack(
        [np.interp(phase, source_phase, xyz[:, axis]) for axis in range(3)],
        axis=1,
    )
    return {
        "episode": trajectory.episode,
        "phase": phase,
        "xyz_mm": standardized_xyz,
        "path_length_mm": float(arc[-1]),
    }


def aggregate_standardized_segment(
    spec: SemanticSegmentSpec,
    standardized_episodes: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    if not standardized_episodes:
        raise ValueError("semantic segment requires source episodes")
    phase = np.asarray(standardized_episodes[0]["phase"], dtype=np.float64)
    positions = np.stack(
        [np.asarray(item["xyz_mm"], dtype=np.float64) for item in standardized_episodes],
        axis=0,
    )
    lengths = np.asarray(
        [float(item["path_length_mm"]) for item in standardized_episodes],
        dtype=np.float64,
    )
    median_xyz = np.median(positions, axis=0)
    mean_xyz = np.mean(positions, axis=0)
    residual = np.linalg.norm(positions - median_xyz[None, :, :], axis=2)
    covariance = np.stack(
        [
            np.cov(positions[:, phase_index, :], rowvar=False)
            for phase_index in range(len(phase))
        ],
        axis=0,
    )
    return {
        "spec": spec,
        "phase": phase,
        "episode_ids": np.asarray(
            [int(item["episode"]) for item in standardized_episodes],
            dtype=np.int64,
        ),
        "episode_xyz_mm": positions,
        "median_xyz_mm": median_xyz,
        "mean_xyz_mm": mean_xyz,
        "covariance_mm2": covariance,
        "mad_mm": np.median(
            np.abs(positions - median_xyz[None, :, :]), axis=0
        ),
        "residual_p50_mm": np.percentile(residual, 50, axis=0),
        "residual_p90_mm": np.percentile(residual, 90, axis=0),
        "residual_p95_mm": np.percentile(residual, 95, axis=0),
        "path_length_mm": {
            "median": float(np.median(lengths)),
            "mean": float(np.mean(lengths)),
            "std": float(np.std(lengths)),
            "min": float(np.min(lengths)),
            "max": float(np.max(lengths)),
        },
        "phase_residual_mm": {
            "p90_mean": float(np.mean(np.percentile(residual, 90, axis=0))),
            "p95_max": float(np.max(np.percentile(residual, 95, axis=0))),
        },
    }


def representative_episode_medoid(
    aggregate_segments: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    """Pick one real episode nearest to all median paths; never average images."""

    if not aggregate_segments:
        raise ValueError("at least one aggregate segment is required")
    episode_ids = np.asarray(aggregate_segments[0]["episode_ids"], dtype=np.int64)
    costs = np.zeros(len(episode_ids), dtype=np.float64)
    per_segment: dict[str, list[float]] = {}
    for aggregate in aggregate_segments:
        if not np.array_equal(episode_ids, aggregate["episode_ids"]):
            raise ValueError("aggregate segments do not share episode order")
        residual = np.linalg.norm(
            aggregate["episode_xyz_mm"]
            - aggregate["median_xyz_mm"][None, :, :],
            axis=2,
        )
        segment_cost = np.sqrt(np.mean(np.square(residual), axis=1))
        scale = max(float(np.median(segment_cost)), 1e-9)
        costs += segment_cost / scale
        per_segment[aggregate["spec"].segment_id] = segment_cost.tolist()
    medoid_offset = int(np.argmin(costs))
    return {
        "episode_index": int(episode_ids[medoid_offset]),
        "normalized_total_residual": float(costs[medoid_offset]),
        "all_episode_normalized_residual": costs.tolist(),
        "per_segment_rms_residual_mm": per_segment,
        "selection_rule": (
            "minimum sum of per-segment RMS Cartesian residual normalized by "
            "the segment median residual"
        ),
    }
