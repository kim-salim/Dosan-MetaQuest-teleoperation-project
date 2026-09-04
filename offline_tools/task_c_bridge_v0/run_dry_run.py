#!/usr/bin/env python3
"""Apply Task-C V0 to two LeRobot datasets without touching robot runtime."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from .bridge_metrics import evaluate_bridge
from .bridge_optimizer import BridgeOptimizer, CandidateEvaluation
from .dataset_io import load_lerobot_trajectories
from .semantic_candidates import (
    SemanticWindow,
    apply_representative_b_velocities,
    generate_candidates,
)
from .serialization import candidate_record, runtime_entry_record
from .visualization import plot_selected_dynamics, plot_selected_path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_composed_task_c_csv(
    path: Path,
    selected: CandidateEvaluation,
    selected_sample: dict[str, np.ndarray],
) -> None:
    """Write a review trajectory, omitting duplicated handoff endpoints."""

    fields = [
        "sequence_index",
        "segment",
        "source_dataset",
        "source_episode",
        "source_frame",
        "source_timestamp_s",
        "bridge_time_s",
        "x_mm",
        "y_mm",
        "z_mm",
        "gripper_closed",
        "holding_assumption",
        "payload_state",
        "orientation_status",
        "robot_executable",
    ]
    rows: list[dict[str, Any]] = []
    a = selected.a
    b = selected.b

    def payload_fields(gripper_closed: bool) -> dict[str, Any]:
        return {
            "holding_assumption": (
                "gripper_closed_implies_holding"
                if gripper_closed
                else "gripper_open_implies_not_holding"
            ),
            "payload_state": (
                "A_OBJECT_HELD_ASSUMED" if gripper_closed else "NO_PAYLOAD_ASSUMED"
            ),
        }

    for index in range(a.index + 1):
        gripper_closed = bool(a.trajectory.gripper_closed[index])
        rows.append(
            {
                "segment": "A_RETAINED",
                "source_dataset": a.trajectory.dataset,
                "source_episode": a.trajectory.episode,
                "source_frame": int(a.trajectory.frame_index[index]),
                "source_timestamp_s": float(a.trajectory.timestamp_s[index]),
                "bridge_time_s": "",
                "x_mm": float(a.trajectory.xyz_mm[index, 0]),
                "y_mm": float(a.trajectory.xyz_mm[index, 1]),
                "z_mm": float(a.trajectory.xyz_mm[index, 2]),
                "gripper_closed": int(gripper_closed),
                **payload_fields(gripper_closed),
            }
        )
    # P0 and P3 already appear as the final A and first B rows.
    for index in range(1, len(selected_sample["position_mm"]) - 1):
        point = selected_sample["position_mm"][index]
        rows.append(
            {
                "segment": "BEZIER_BRIDGE",
                "source_dataset": "generated",
                "source_episode": "",
                "source_frame": "",
                "source_timestamp_s": "",
                "bridge_time_s": float(selected_sample["time_s"][index]),
                "x_mm": float(point[0]),
                "y_mm": float(point[1]),
                "z_mm": float(point[2]),
                "gripper_closed": int(a.semantic_state.gripper_closed),
                **payload_fields(bool(a.semantic_state.gripper_closed)),
            }
        )
    for index in range(b.index, len(b.trajectory.xyz_mm)):
        gripper_closed = bool(b.trajectory.gripper_closed[index])
        rows.append(
            {
                "segment": "B_RETAINED",
                "source_dataset": b.trajectory.dataset,
                "source_episode": b.trajectory.episode,
                "source_frame": int(b.trajectory.frame_index[index]),
                "source_timestamp_s": float(b.trajectory.timestamp_s[index]),
                "bridge_time_s": "",
                "x_mm": float(b.trajectory.xyz_mm[index, 0]),
                "y_mm": float(b.trajectory.xyz_mm[index, 1]),
                "z_mm": float(b.trajectory.xyz_mm[index, 2]),
                "gripper_closed": int(gripper_closed),
                **payload_fields(gripper_closed),
            }
        )
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for sequence_index, row in enumerate(rows):
            row["sequence_index"] = sequence_index
            row["orientation_status"] = "pending"
            row["robot_executable"] = False
            writer.writerow(row)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-a", type=Path, required=True)
    parser.add_argument("--dataset-b", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    args.output.mkdir(parents=True, exist_ok=True)

    trajectories_a, info_a = load_lerobot_trajectories(
        args.dataset_a, config["dataset_a_label"]
    )
    trajectories_b, info_b = load_lerobot_trajectories(
        args.dataset_b, config["dataset_b_label"]
    )
    windows_a = [
        SemanticWindow.from_dict(item) for item in config["semantic_windows"]["a_cut"]
    ]
    windows_b = [
        SemanticWindow.from_dict(item) for item in config["semantic_windows"]["b_entry"]
    ]
    candidates_a = generate_candidates(
        trajectories_a,
        role="a_cut",
        windows=windows_a,
        velocity_config=config["velocity"],
    )
    candidates_b = generate_candidates(
        trajectories_b,
        role="b_entry",
        windows=windows_b,
        velocity_config=config["velocity"],
    )
    candidates_b = apply_representative_b_velocities(
        candidates_b,
        config.get("b_representative_velocity"),
    )
    if not candidates_a or not candidates_b:
        raise RuntimeError("semantic window produced no A or B candidates")

    observed = np.concatenate(
        [trajectory.xyz_mm for trajectory in trajectories_a + trajectories_b], axis=0
    )
    workspace_margin = float(config["workspace"]["margin_mm"])
    workspace_min = np.min(observed, axis=0) - workspace_margin
    workspace_max = np.max(observed, axis=0) + workspace_margin

    candidate_path = args.output / "all_duration_candidates.jsonl"
    counts_by_failure: dict[str, int] = {}
    optimizer = BridgeOptimizer(config)
    with candidate_path.open("w", encoding="utf-8") as candidate_handle:

        def record(candidate: CandidateEvaluation) -> None:
            candidate_handle.write(json.dumps(candidate_record(candidate), sort_keys=True) + "\n")
            for reason in candidate.failure_reasons:
                counts_by_failure[reason] = counts_by_failure.get(reason, 0) + 1

        selected, top_k, counters = optimizer.optimize(
            candidates_a,
            candidates_b,
            workspace_min_mm=workspace_min,
            workspace_max_mm=workspace_max,
            record=record,
        )

    top_k_records = [candidate_record(candidate) for candidate in top_k]
    selected_record = candidate_record(selected)
    write_json(args.output / "selected_candidate.json", selected_record)
    write_json(args.output / "top_k_candidates.json", top_k_records)
    runtime_config = config.get("runtime", {})
    runtime_entries = [
        runtime_entry_record(candidate)
        for candidate in optimizer.runtime_b_entry_evaluations
    ]
    runtime_manifest_path = args.output / "runtime_transition_manifest.json"
    runtime_manifest = {
        "version": config["version"],
        "mode": "position_only_dry_run_design",
        "policy_checkpoints": runtime_config.get("policy_checkpoints", {}),
        "model_lifecycle": {
            "load_both_before_act_a": True,
            "warmup_outputs_discarded": True,
            "separate_a_b_generations_and_queues": True,
            "shared_gpu_inference_arbiter": True,
        },
        "velocity_contract": {
            "a_boundary_source": "causal_measured_tcp_history",
            "a_policy_chunk_role": "diagnostic_intent_only",
            "initial_b_source": "phase_local_robust_demonstration",
            "final_b_source": "fresh_postprocessed_act_b_action_chunk",
            "single_frame_difference_used": False,
        },
        "transition_order": [
            "ACT_A_ACTIVE",
            "SEMANTIC_CUT",
            "CLEAR_ACT_A_QUEUE",
            "BEZIER_INITIAL",
            "PRIME_ACT_B_DURING_BRIDGE",
            "REPLAN_BRIDGE_TAIL_WITH_FRESH_ACT_B_VELOCITY",
            "GATED_ATOMIC_HANDOFF",
            "ACT_B_ACTIVE",
        ],
        "planners": {
            "initial_duration_search": config["duration_search"],
            "tail_duration_search": runtime_config.get(
                "tail_duration_search", config["duration_search"]
            ),
            "sampling": config["sampling"],
            "feasibility": config["feasibility"],
            "selection": config["selection"],
            "semantic_compatibility": config["semantic_compatibility"],
        },
        "workspace": {
            "minimum_mm": workspace_min.tolist(),
            "maximum_mm": workspace_max.tolist(),
            "robot_certified": False,
        },
        "runtime_handoff": runtime_config.get("handoff", {}),
        "b_entries": runtime_entries,
        "safety": config["safety"],
    }
    write_json(runtime_manifest_path, runtime_manifest)

    sample_hz = float(config["sampling"]["bridge_sample_hz"])
    curvature_epsilon = float(config["sampling"]["curvature_epsilon"])
    _, selected_sample = evaluate_bridge(
        selected.bridge,
        sample_hz=sample_hz,
        curvature_epsilon=curvature_epsilon,
        workspace_min_mm=workspace_min,
        workspace_max_mm=workspace_max,
    )
    np.savez_compressed(
        args.output / "selected_bridge_samples.npz",
        **selected_sample,
        a_retained=selected.a.trajectory.xyz_mm[: selected.a.index + 1],
        a_removed=selected.a.trajectory.xyz_mm[selected.a.index :],
        b_removed=selected.b.trajectory.xyz_mm[: selected.b.index + 1],
        b_retained=selected.b.trajectory.xyz_mm[selected.b.index :],
    )
    write_composed_task_c_csv(
        args.output / "selected_task_c_xyz.csv", selected, selected_sample
    )
    plot_selected_path(
        selected,
        sample_hz=sample_hz,
        curvature_epsilon=curvature_epsilon,
        workspace_min_mm=workspace_min,
        workspace_max_mm=workspace_max,
        output=args.output / "selected_task_c_3d.png",
    )
    plot_selected_dynamics(
        selected,
        sample_hz=sample_hz,
        curvature_epsilon=curvature_epsilon,
        workspace_min_mm=workspace_min,
        workspace_max_mm=workspace_max,
        output=args.output / "selected_bridge_dynamics.png",
    )

    report = {
        "version": config["version"],
        "input": {
            "dataset_a": str(args.dataset_a),
            "dataset_b": str(args.dataset_b),
            "dataset_a_task": info_a["task_descriptions"],
            "dataset_b_task": info_b["task_descriptions"],
            "dataset_a_fps": info_a["fps"],
            "dataset_b_fps": info_b["fps"],
            "config": str(args.config),
            "config_sha256": sha256(args.config),
        },
        "candidate_generation": {
            "low_speed_boundary_used": False,
            "a_candidates": len(candidates_a),
            "b_candidates": len(candidates_b),
            "representative_b_velocity_config": config.get(
                "b_representative_velocity", {"enabled": False}
            ),
            "representative_b_velocity_candidates": sum(
                candidate.velocity_method.startswith("representative_")
                for candidate in candidates_b
            ),
            **counters,
            "failure_counts": counts_by_failure,
        },
        "workspace": {
            "mode": config["workspace"]["mode"],
            "minimum_mm": workspace_min.tolist(),
            "maximum_mm": workspace_max.tolist(),
            "robot_certified": False,
        },
        "selection": {
            "primary_objective": "A_retained + bridge + B_retained path length",
            "near_shortest_delta": config["selection"]["near_shortest_delta"],
            "runtime_distinct_b_entries": len(runtime_entries),
            "selected": selected_record,
        },
        "semantic_limitations": {
            "holding_field": "ABSENT",
            "contact_field": "ABSENT",
            "semantic_source": "gripper transition windows only",
            "manual_or_external_subgoal_annotations": "ABSENT",
        },
        "safety": config["safety"],
        "outputs": {
            "all_candidates": str(candidate_path),
            "selected_candidate": str(args.output / "selected_candidate.json"),
            "top_k": str(args.output / "top_k_candidates.json"),
            "trajectory_3d": str(args.output / "selected_task_c_3d.png"),
            "bridge_dynamics": str(args.output / "selected_bridge_dynamics.png"),
            "bridge_samples": str(args.output / "selected_bridge_samples.npz"),
            "composed_task_c_csv": str(args.output / "selected_task_c_xyz.csv"),
            "runtime_transition_manifest": str(runtime_manifest_path),
        },
    }
    write_json(args.output / "dry_run_report.json", report)
    checksum_paths = sorted(
        path
        for path in args.output.iterdir()
        if path.is_file() and path.name != "checksums.sha256"
    )
    (args.output / "checksums.sha256").write_text(
        "".join(f"{sha256(path)}  {path.name}\n" for path in checksum_paths),
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
