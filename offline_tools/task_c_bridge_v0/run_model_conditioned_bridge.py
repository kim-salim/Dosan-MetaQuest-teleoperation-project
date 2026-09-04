#!/usr/bin/env python3
"""Apply real ACT inference velocities to the pure runtime bridge planner."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from .bridge_metrics import evaluate_bridge
from .dataset_io import load_lerobot_trajectories
from .runtime_bridge import (
    RuntimeBridgePlanner,
    runtime_entries_from_manifest,
)
from .serialization import runtime_candidate_record
from .trajectory_states import SemanticState
from .velocity_estimation import estimate_velocity_over_window
from .visualization import (
    plot_runtime_model_conditioned_dynamics,
    plot_runtime_model_conditioned_path,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime-manifest", type=Path, required=True)
    parser.add_argument("--dual-act-report", type=Path, required=True)
    parser.add_argument("--selected-offline-candidate", type=Path, required=True)
    parser.add_argument("--velocity-window-frames", type=int, default=15)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_output_checksums(output: Path) -> None:
    paths = sorted(
        path
        for path in output.iterdir()
        if path.is_file() and path.name != "checksums.sha256"
    )
    (output / "checksums.sha256").write_text(
        "".join(f"{file_sha256(path)}  {path.name}\n" for path in paths),
        encoding="utf-8",
    )


def semantic_state_from_record(value: dict[str, Any]) -> SemanticState:
    return SemanticState(
        gripper_closed=bool(value["gripper_closed"]),
        holding=value.get("holding"),
        contact_mode=value.get("contact_mode"),
        completed_subgoals=tuple(value.get("completed_subgoals", ())),
        object_state=value.get("object_state"),
        entry_preconditions=tuple(value.get("entry_preconditions", ())),
    )


def main() -> None:
    args = parse_args()
    if args.velocity_window_frames < 3:
        raise SystemExit("--velocity-window-frames must be at least 3")

    manifest = read_json(args.runtime_manifest)
    dual_report = read_json(args.dual_act_report)
    offline = read_json(args.selected_offline_candidate)
    if dual_report.get("dry_run_only") is not True:
        raise ValueError("dual ACT report must be dry-run-only")
    if dual_report.get("robot_executable") is not False:
        raise ValueError("dual ACT report must not claim robot executability")
    if dual_report.get("robot_commands_published") is not False:
        raise ValueError("dual ACT report unexpectedly claims command publication")
    if offline.get("robot_executable") is not False:
        raise ValueError("offline candidate unexpectedly claims robot executability")

    a_observation = dual_report["observations"]["ACT-A"]
    if (
        int(a_observation["episode"]) != int(offline["a_episode"])
        or int(a_observation["frame"]) != int(offline["a_frame"])
    ):
        raise ValueError("ACT-A dry-run observation differs from offline A cut")
    b_assessment = dual_report["fresh_chunks"]["ACT-B"][
        "entry_velocity_assessment"
    ]
    if b_assessment.get("valid") is not True:
        raise ValueError(
            "fresh ACT-B chunk did not pass its configured action-space gates"
        )

    trajectories_a, _ = load_lerobot_trajectories(
        Path(a_observation["dataset_root"]),
        str(offline["a_dataset"]),
    )
    matches = [
        trajectory
        for trajectory in trajectories_a
        if trajectory.episode == int(offline["a_episode"])
    ]
    if len(matches) != 1:
        raise RuntimeError("could not resolve exactly one ACT-A trajectory")
    trajectory_a = matches[0]
    frame_matches = np.flatnonzero(
        trajectory_a.frame_index == int(offline["a_frame"])
    )
    if len(frame_matches) != 1:
        raise RuntimeError("could not resolve exactly one ACT-A cut frame")
    a_index = int(frame_matches[0])
    history_start = max(0, a_index - args.velocity_window_frames + 1)
    history_positions = trajectory_a.xyz_mm[history_start : a_index + 1]
    history_timestamps = trajectory_a.timestamp_s[history_start : a_index + 1]
    measured_replay_velocity_a = estimate_velocity_over_window(
        history_positions,
        history_timestamps,
        method="linear_regression",
        velocity_epsilon=1e-9,
    )
    position_a = trajectory_a.xyz_mm[a_index]
    report_position_a = np.asarray(a_observation["tcp_position_mm"], dtype=float)
    if not np.allclose(position_a, report_position_a, atol=1e-5):
        raise ValueError("recorded A TCP does not match ACT-A inference observation")

    velocity_b = np.asarray(
        b_assessment["intended_velocity_mm_s"],
        dtype=np.float64,
    )
    first_target_b = np.asarray(
        dual_report["fresh_chunks"]["ACT-B"]["first_action"][:3],
        dtype=np.float64,
    )
    entries = runtime_entries_from_manifest(manifest)
    planner = RuntimeBridgePlanner.from_manifest(
        manifest,
        duration_search="initial_duration_search",
    )
    handoff = manifest["runtime_handoff"]
    result = planner.plan(
        position_a_mm=position_a,
        velocity_a_mm_s=measured_replay_velocity_a,
        semantic_state_a=semantic_state_from_record(offline["a_semantic_state"]),
        a_retained_length_mm=float(offline["A_retained_length_mm"]),
        b_entries=entries,
        terminal_velocity_override_mm_s=velocity_b,
        terminal_position_hint_mm=first_target_b,
        terminal_position_tolerance_mm=float(
            handoff["handoff_first_target_tolerance_mm"]
        ),
        terminal_velocity_source="fresh_act_b_postprocessed_chunk",
    )
    selected = result.selected
    sample_hz = float(manifest["planners"]["sampling"]["bridge_sample_hz"])
    curvature_epsilon = float(
        manifest["planners"]["sampling"]["curvature_epsilon"]
    )
    workspace_min = np.asarray(
        manifest["workspace"]["minimum_mm"],
        dtype=np.float64,
    )
    workspace_max = np.asarray(
        manifest["workspace"]["maximum_mm"],
        dtype=np.float64,
    )
    _, samples = evaluate_bridge(
        selected.bridge,
        sample_hz=sample_hz,
        curvature_epsilon=curvature_epsilon,
        workspace_min_mm=workspace_min,
        workspace_max_mm=workspace_max,
    )

    start_position_error = float(
        np.linalg.norm(selected.bridge.position(0.0) - position_a)
    )
    end_position_error = float(
        np.linalg.norm(
            selected.bridge.position(1.0) - selected.entry.position_mm
        )
    )
    start_velocity_error = float(
        np.linalg.norm(
            selected.bridge.velocity(0.0) - measured_replay_velocity_a
        )
    )
    end_velocity_error = float(
        np.linalg.norm(selected.bridge.velocity(1.0) - velocity_b)
    )
    if max(
        start_position_error,
        end_position_error,
        start_velocity_error,
        end_velocity_error,
    ) > 1e-8:
        raise RuntimeError("model-conditioned Bezier endpoint contract failed")

    args.output.mkdir(parents=True, exist_ok=True)
    selected_record = runtime_candidate_record(selected)
    top_k_records = [
        runtime_candidate_record(candidate) for candidate in result.top_k
    ]
    report = {
        "status": "model_conditioned_bridge_dry_run_complete",
        "mode": "offline_recorded_observation_replay_no_robot_io",
        "inputs": {
            "runtime_manifest": str(args.runtime_manifest),
            "dual_act_report": str(args.dual_act_report),
            "selected_offline_candidate": str(
                args.selected_offline_candidate
            ),
        },
        "velocity_sources": {
            "A_boundary": {
                "source": "causal_recorded_tcp_history_replay",
                "runtime_authoritative_source": "causal_live_measured_tcp_history",
                "window_frames": len(history_positions),
                "velocity_mm_s": measured_replay_velocity_a.tolist(),
                "ACT_A_policy_intent_mm_s": dual_report["fresh_chunks"]["ACT-A"][
                    "intent_assessment"
                ]["intended_velocity_mm_s"],
                "ACT_A_intent_is_boundary": False,
            },
            "B_boundary": {
                "source": "fresh_act_b_postprocessed_chunk",
                "velocity_mm_s": velocity_b.tolist(),
                "first_target_mm": first_target_b.tolist(),
                "velocity_window_steps": b_assessment[
                    "velocity_window_steps"
                ],
                "single_frame_difference_used": False,
                "selected_entry_demonstration_velocity_mm_s": (
                    selected.entry.demonstration_velocity_mm_s.tolist()
                ),
                "model_vs_demonstration_velocity_difference_mm_s": float(
                    np.linalg.norm(
                        velocity_b
                        - selected.entry.demonstration_velocity_mm_s
                    )
                ),
            },
        },
        "endpoint_contract_errors": {
            "start_position_mm": start_position_error,
            "end_position_mm": end_position_error,
            "start_velocity_mm_s": start_velocity_error,
            "end_velocity_mm_s": end_velocity_error,
        },
        "search": {
            "candidates_evaluated": result.candidates_evaluated,
            "semantic_rejected_entries": result.semantic_rejected_entries,
            "failure_counts": result.failure_counts,
            "primary_objective": (
                "A_retained + bridge + B_retained total path length"
            ),
            "near_shortest_delta": manifest["planners"]["selection"][
                "near_shortest_delta"
            ],
        },
        "selected": selected_record,
        "top_k": top_k_records,
        "orientation_status": "pending",
        "orientation_bridge_generated": False,
        "collision_status": "NOT_CHECKED_WITH_PAYLOAD",
        "ik_status": "NOT_CHECKED",
        "robot_executable": False,
        "dry_run_only": True,
        "robot_commands_published": False,
    }
    report_path = args.output / "model_conditioned_bridge_report.json"
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    np.savez_compressed(
        args.output / "model_conditioned_bridge_samples.npz",
        **samples,
        a_causal_history_mm=history_positions,
        measured_velocity_a_mm_s=measured_replay_velocity_a,
        act_b_velocity_mm_s=velocity_b,
        act_b_first_target_mm=first_target_b,
    )
    plot_runtime_model_conditioned_path(
        selected,
        a_history_mm=trajectory_a.xyz_mm[: a_index + 1],
        measured_velocity_a_mm_s=measured_replay_velocity_a,
        sample_hz=sample_hz,
        curvature_epsilon=curvature_epsilon,
        workspace_min_mm=workspace_min,
        workspace_max_mm=workspace_max,
        output=args.output / "model_conditioned_bridge_3d.png",
    )
    plot_runtime_model_conditioned_dynamics(
        selected,
        sample_hz=sample_hz,
        curvature_epsilon=curvature_epsilon,
        workspace_min_mm=workspace_min,
        workspace_max_mm=workspace_max,
        output=args.output / "model_conditioned_bridge_dynamics.png",
    )
    write_output_checksums(args.output)
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
