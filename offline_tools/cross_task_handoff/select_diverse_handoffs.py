"""Select reproducible representative handoffs with normalized FPS."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from .schema import HandoffCandidate, PhaseIndexPoint


def candidate_feature(candidate: HandoffCandidate) -> np.ndarray:
    return np.concatenate(
        (
            np.asarray([candidate.source.phase, candidate.successor.phase]),
            candidate.source.position_mm,
            candidate.successor.position_mm,
            candidate.source.velocity_mm_s,
            candidate.successor.velocity_mm_s,
            np.asarray([candidate.bridge_length_mm]),
        )
    )


def normalized_features(
    candidates: list[HandoffCandidate],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if not candidates:
        raise ValueError("diversity selection requires feasible candidates")
    raw = np.stack([candidate_feature(item) for item in candidates])
    minimum = np.min(raw, axis=0)
    maximum = np.max(raw, axis=0)
    scale = maximum - minimum
    scale[scale <= 1e-12] = 1.0
    return (raw - minimum) / scale, minimum, maximum


def farthest_point_selection(
    candidates: list[HandoffCandidate],
    *,
    count: int,
) -> tuple[list[HandoffCandidate], dict[str, Any]]:
    if count < 1:
        raise ValueError("selection count must be positive")
    features, minimum, maximum = normalized_features(candidates)
    first = min(
        range(len(candidates)),
        key=lambda index: (
            -candidates[index].support_score,
            candidates[index].jerk_metric,
            candidates[index].handoff_id,
        ),
    )
    selected = [first]
    selected_distance: list[float | None] = [None]
    minimum_distance = np.linalg.norm(features - features[first], axis=1)
    while len(selected) < min(count, len(candidates)):
        remaining = [index for index in range(len(candidates)) if index not in selected]
        next_index = min(
            remaining,
            key=lambda index: (
                -minimum_distance[index],
                -candidates[index].support_score,
                candidates[index].handoff_id,
            ),
        )
        selected.append(next_index)
        selected_distance.append(float(minimum_distance[next_index]))
        minimum_distance = np.minimum(
            minimum_distance,
            np.linalg.norm(features - features[next_index], axis=1),
        )
    return (
        [candidates[index] for index in selected],
        {
            "method": "normalized_farthest_point_sampling",
            "feature_order": [
                "source_phase",
                "successor_phase",
                "source_xyz_3",
                "successor_xyz_3",
                "source_velocity_3",
                "successor_velocity_3",
                "bridge_length",
            ],
            "feature_minimum": minimum.tolist(),
            "feature_maximum": maximum.tolist(),
            "selected_indices": selected,
            "selected_minimum_distance": selected_distance,
            "safety_role": "none_selection_runs_only_after_hard_filter",
        },
    )


def _boundary_record(
    point: PhaseIndexPoint,
    *,
    support_radius_mm: float,
    support_orientation_radius_deg: float,
    source: bool,
    prearm_radius_mm: float,
    commit_radius_mm: float,
    direction_cosine_minimum: float,
) -> dict[str, Any]:
    semantic_preconditions = (
        point.completed_subgoals if source else point.entry_preconditions
    )
    value = {
        "task": point.task,
        "segment": point.segment,
        "phase": point.phase,
        "support_episode": point.support_episode,
        "support_frame": point.support_frame,
        "nominal_pose": {
            "position_mm": point.position_mm.tolist(),
            "orientation_quat_xyzw": point.orientation_quat_xyzw.tolist(),
        },
        "nominal_velocity_mm_s": point.velocity_mm_s.tolist(),
        "support_radius_mm": support_radius_mm,
        "support_orientation_radius_deg": support_orientation_radius_deg,
        "semantic": {
            "semantic_state": point.semantic_state,
            "gripper_state": point.gripper_state,
            "held_object": point.held_object,
            "contact_mode": point.contact_mode,
            "entry_preconditions": list(semantic_preconditions),
        },
    }
    if source:
        value.update(
            {
                "prearm_radius_mm": prearm_radius_mm,
                "commit_radius_mm": commit_radius_mm,
                "direction_cosine_minimum": direction_cosine_minimum,
                "approach_frames": 3,
                "commit_stable_frames": 3,
            }
        )
    return value


def episode_manifest(
    composition_id: str,
    candidate: HandoffCandidate,
    *,
    support_radius_mm: float,
    support_orientation_radius_deg: float,
    prearm_radius_mm: float,
    commit_radius_mm: float,
    direction_cosine_minimum: float,
) -> dict[str, Any]:
    return {
        "schema_version": "a0509.task_c_handoff_episode.v2",
        "composition_id": composition_id,
        "handoff_id": candidate.handoff_id,
        "source": _boundary_record(
            candidate.source,
            support_radius_mm=support_radius_mm,
            support_orientation_radius_deg=support_orientation_radius_deg,
            source=True,
            prearm_radius_mm=prearm_radius_mm,
            commit_radius_mm=commit_radius_mm,
            direction_cosine_minimum=direction_cosine_minimum,
        ),
        "successor": _boundary_record(
            candidate.successor,
            support_radius_mm=support_radius_mm,
            support_orientation_radius_deg=support_orientation_radius_deg,
            source=False,
            prearm_radius_mm=prearm_radius_mm,
            commit_radius_mm=commit_radius_mm,
            direction_cosine_minimum=direction_cosine_minimum,
        ),
        "bridge": {
            "duration_s": candidate.bridge_duration_s,
            "nominal_length_mm": candidate.bridge_length_mm,
            "transport_floor_mm": candidate.transport_floor_mm,
        },
        "generation": {
            "method": "normalized_farthest_point",
            "boundary_condition_diversity": True,
            "bridge_algorithm": candidate.bridge_algorithm,
            "tangent_regularization": {
                "minimum_handle_chord_ratio": (
                    candidate.minimum_tangent_handle_chord_ratio
                ),
                "maximum_endpoint_speed_adjustment_mm_s": (
                    candidate.maximum_endpoint_speed_adjustment_mm_s
                ),
                "nominal_source_speed_adjustment_mm_s": (
                    candidate.source_endpoint_speed_adjustment_mm_s
                ),
                "nominal_successor_speed_adjustment_mm_s": (
                    candidate.successor_endpoint_speed_adjustment_mm_s
                ),
            },
        },
        "validation": {
            "hard_filter_passed": True,
            "semantic_authority": candidate.semantic_authority.value,
            "semantic_checks_enforced_by_runtime": (
                candidate.semantic_authority.runtime_semantic_checks_enforced
            ),
            "semantic_diagnostics": list(candidate.semantic_diagnostics),
            "ik_checked": candidate.ik_checked,
            "collision_checked": candidate.collision_checked,
            "robot_executable": False,
            "dry_run_only": True,
        },
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--count", type=int, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--support-radius-mm", type=float, required=True)
    parser.add_argument(
        "--support-orientation-radius-deg",
        type=float,
        default=30.0,
    )
    parser.add_argument("--prearm-radius-mm", type=float, default=40.0)
    parser.add_argument("--commit-radius-mm", type=float, default=20.0)
    parser.add_argument("--direction-cosine-minimum", type=float, default=0.7)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    library = json.loads(args.library.read_text(encoding="utf-8"))
    candidates = [
        HandoffCandidate.from_record(value)
        for value in library["candidates"]
        if bool(value["feasible"])
    ]
    selected, selection = farthest_point_selection(candidates, count=args.count)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    manifest_paths: list[str] = []
    for candidate in selected:
        value = episode_manifest(
            str(library["composition_id"]),
            candidate,
            support_radius_mm=args.support_radius_mm,
            support_orientation_radius_deg=(
                args.support_orientation_radius_deg
            ),
            prearm_radius_mm=args.prearm_radius_mm,
            commit_radius_mm=args.commit_radius_mm,
            direction_cosine_minimum=args.direction_cosine_minimum,
        )
        output = args.output_dir / f"{candidate.handoff_id}.json"
        output.write_text(
            json.dumps(value, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        manifest_paths.append(str(output.resolve()))
    summary = {
        "schema_version": "a0509.cross_task_handoff_selection.v2",
        "composition_id": library["composition_id"],
        "semantic_authority": library.get("semantic_authority", "runtime_guarded"),
        "semantic_checks_enforced_by_runtime": library.get(
            "semantic_checks_enforced_by_runtime", True
        ),
        "valid_candidate_count": len(candidates),
        "selected_count": len(selected),
        "selected_handoff_ids": [item.handoff_id for item in selected],
        "episode_manifests": manifest_paths,
        "selection": selection,
    }
    (args.output_dir / "selection_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        f"DIVERSE_HANDOFF_SELECTION_OK valid={len(candidates)} "
        f"selected={len(selected)} output={args.output_dir}"
    )


if __name__ == "__main__":
    main()
