"""Build a real-episode semantic phase index from one LeRobot dataset."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from offline_tools.semantic_segmentation.semantic_phase_core import (
    detect_gripper_event_sequence,
    resolve_anchor_index,
)
from offline_tools.task_c_bridge_v0.dataset_io import load_lerobot_trajectories
from offline_tools.task_c_bridge_v0.velocity_estimation import estimate_local_velocity
from quest_a0509_teleop.doosan_orientation import doosan_zyz_deg_to_quaternion

from .schema import PhaseIndexPoint


def _semantic_state_facts(state: dict[str, Any], *, gripper_state: str, held_object: str) -> tuple[str, ...]:
    """Encode observable symbolic preconditions without inventing scene changes."""

    values = {str(key): str(value) for key, value in state.items()}
    values["gripper"] = gripper_state
    values["held_object"] = held_object
    return tuple(sorted(f"{key}={value}" for key, value in values.items()))


def _contact_mode(spec: dict[str, Any], *, closed: bool, held_object: str) -> str:
    """Conservatively separate portable-object transport from fixture contact."""

    if not closed or held_object in {"none", "unknown"}:
        return "free_motion_assumed"
    manipulated = str(spec.get("manipulated_object") or "").strip().lower()
    held = held_object.strip().lower()
    semantic = str(spec.get("semantic_label") or "").strip().lower()
    fixture_tokens = ("handle", "drawer", "door", "lid", "container")
    portable_tokens = ("block", "object", "item", "payload", "part")
    if any(token in held or token in manipulated for token in fixture_tokens):
        return "contact_manipulation_assumed"
    if (
        held == manipulated
        and any(token in held for token in portable_tokens)
        and any(token in semantic for token in ("transport", "carry", "transfer", "place"))
    ):
        return "free_transport_assumed"
    return "contact_unknown_assumed"


def _phase_indices(
    xyz_mm: np.ndarray,
    start_index: int,
    end_index: int,
    phases: np.ndarray,
) -> list[int]:
    xyz = xyz_mm[start_index : end_index + 1]
    arc = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)))
    )
    if arc[-1] <= 1e-9:
        raise ValueError("semantic segment has zero Cartesian arc length")
    normalized = arc / arc[-1]
    return [
        start_index + int(np.argmin(np.abs(normalized - phase)))
        for phase in phases
    ]


def build_phase_index(
    *,
    dataset_root: Path,
    semantic_graph_path: Path,
    task_id: str,
    phase_step: float = 0.05,
    velocity_window_frames: int = 15,
) -> dict[str, Any]:
    if not 0.0 < phase_step <= 1.0:
        raise ValueError("phase_step must be in (0, 1]")
    graph = json.loads(semantic_graph_path.read_text(encoding="utf-8"))
    graph_task_id = graph.get("task_id")
    if (
        graph_task_id is not None
        and str(graph_task_id).strip().upper() != task_id.strip().upper()
    ):
        raise ValueError("semantic graph task_id differs from requested task")
    trajectories, dataset_info = load_lerobot_trajectories(dataset_root, task_id)
    expected_events = tuple(str(item) for item in graph["event_grammar"])
    phases = np.arange(0.0, 1.0 + phase_step * 0.5, phase_step)
    phases = np.clip(phases, 0.0, 1.0)
    points: list[PhaseIndexPoint] = []
    segment_counts: dict[str, int] = {}
    for segment_value in graph["segments"]:
        spec = dict(segment_value["spec"])
        segment_id = str(spec["segment_id"])
        entry_state = dict(spec.get("entry_state", {}))
        residual = dict(segment_value.get("phase_residual_mm", {}))
        support_score = 1.0 / (1.0 + float(residual.get("p90_mean", 0.0)))
        for trajectory in trajectories:
            events = detect_gripper_event_sequence(trajectory, expected_events)
            start = resolve_anchor_index(
                str(spec["start_anchor"]), trajectory, events
            )
            end = resolve_anchor_index(str(spec["end_anchor"]), trajectory, events)
            if start >= end:
                raise ValueError(
                    f"episode {trajectory.episode} segment {segment_id} is reversed"
                )
            indices = _phase_indices(trajectory.xyz_mm, start, end, phases)
            for phase, index in zip(phases, indices, strict=True):
                closed = bool(trajectory.gripper_closed[index])
                held_object = str(entry_state.get("held_object", "none"))
                if closed and held_object == "none":
                    held_object = str(spec.get("manipulated_object") or "unknown")
                if not closed:
                    held_object = "none"
                contact_mode = _contact_mode(
                    spec,
                    closed=closed,
                    held_object=held_object,
                )
                state_facts = _semantic_state_facts(
                    entry_state,
                    gripper_state="closed" if closed else "open",
                    held_object=held_object,
                )
                orientation = trajectory.orientation_payload
                if orientation is None:
                    raise ValueError("A0509 phase index requires orientation payload")
                velocity = estimate_local_velocity(
                    trajectory.xyz_mm,
                    trajectory.timestamp_s,
                    index,
                    window_frames=velocity_window_frames,
                    method="linear_regression",
                    velocity_epsilon=1e-9,
                )
                points.append(
                    PhaseIndexPoint(
                        task=task_id,
                        segment=segment_id,
                        phase=float(phase),
                        support_episode=trajectory.episode,
                        support_frame=int(trajectory.frame_index[index]),
                        position_mm=trajectory.xyz_mm[index],
                        orientation_quat_xyzw=doosan_zyz_deg_to_quaternion(
                            orientation[index]
                        ),
                        velocity_mm_s=velocity,
                        semantic_state=str(spec["semantic_label"]),
                        gripper_state="closed" if closed else "open",
                        held_object=held_object,
                        contact_mode=contact_mode,
                        entry_preconditions=tuple(dict.fromkeys((
                            *state_facts,
                            *tuple(spec.get("entry_preconditions", ())),
                        ))),
                        completed_subgoals=tuple(dict.fromkeys((
                            *state_facts,
                            *tuple(spec.get("completed_subgoals", ())),
                        ))),
                        support_score=support_score,
                    )
                )
                segment_counts[segment_id] = segment_counts.get(segment_id, 0) + 1
    return {
        "schema_version": "a0509.cross_task_phase_index.v2",
        "task": task_id,
        "dataset_root": str(dataset_root.resolve()),
        "dataset_repo_id": graph.get("dataset_repo_id"),
        "task_description": graph.get("task_description"),
        "fps": int(dataset_info["fps"]),
        "episode_count": len(trajectories),
        "phase_step": phase_step,
        "orientation_representation": "quaternion_xyzw",
        "holding_evidence": (
            "gripper_closed_plus_semantic_held_object_assumption"
        ),
        "contact_evidence": "free_transport_assumed_not_force_sensed",
        "segment_counts": segment_counts,
        "points": [point.to_record() for point in points],
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--semantic-graph", type=Path, required=True)
    parser.add_argument("--task-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--phase-step", type=float, default=0.05)
    parser.add_argument("--velocity-window-frames", type=int, default=15)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    result = build_phase_index(
        dataset_root=args.dataset_root,
        semantic_graph_path=args.semantic_graph,
        task_id=args.task_id,
        phase_step=args.phase_step,
        velocity_window_frames=args.velocity_window_frames,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        f"PHASE_INDEX_OK task={args.task_id} points={len(result['points'])} "
        f"output={args.output}"
    )


if __name__ == "__main__":
    main()
