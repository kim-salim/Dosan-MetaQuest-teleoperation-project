#!/usr/bin/env python3
"""Build a V1 representative Task-C contract without touching ROS or a robot."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import time
from pathlib import Path
from typing import Any

import numpy as np

from offline_tools.task_c_bridge_v0.dataset_io import load_lerobot_trajectories
from offline_tools.task_c_bridge_v1.representative_optimizer import (
    RepresentativeBridgeCandidate,
    RepresentativeBridgeOptimizer,
)
from offline_tools.task_c_bridge_v1.representative_trajectory import (
    RepresentativeTrajectory,
    TransportWindowConfig,
    build_representative_trajectory,
)
from offline_tools.task_c_bridge_v1.runtime_boundary import (
    RepresentativeBoundaryContract,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-a", type=Path, required=True)
    parser.add_argument("--dataset-b", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def _write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_config(config: dict[str, Any]) -> None:
    if config.get("version") != "task_c_representative_bridge_v1_closed_holding":
        raise ValueError("unexpected V1 representative config version")
    sampling = config["sampling"]
    if float(sampling["coarse_sample_hz"]) != 30.0:
        raise ValueError("V1 coarse search must remain 30 Hz")
    if float(sampling["final_sample_hz"]) != 60.0:
        raise ValueError("V1 final validation must remain 60 Hz")
    if float(sampling["runtime_sample_hz"]) != 60.0:
        raise ValueError("V1 runtime validation must remain 60 Hz")
    if config["selection"].get("primary_objective") != "total_C_length_mm":
        raise ValueError("V1 primary objective must remain total_C_length_mm")
    boundary = config["live_boundary"]
    prearm = float(boundary["prearm_radius_mm"])
    commit = float(boundary["commit_radius_mm"])
    if not prearm > commit > 0.0:
        raise ValueError("V1 boundary radii must satisfy prearm > commit > 0")
    for name in (
        "minimum_prearm_support_fraction",
        "minimum_commit_support_fraction",
    ):
        value = float(boundary[name])
        if not 0.0 <= value <= 1.0:
            raise ValueError(f"{name} must be in [0, 1]")
    if int(config["selection"]["runtime_entry_top_k"]) != 1:
        raise ValueError("V1 runtime must use exactly one representative B entry")


def _trajectory_summary(
    representative: RepresentativeTrajectory,
) -> dict[str, Any]:
    return {
        "role": representative.role,
        "dataset": representative.dataset,
        "episode_count": representative.episode_count,
        "phase_points": len(representative.phase),
        "minimum_bridge_z_mm": representative.minimum_bridge_z_mm,
        "source_episodes": [
            {
                "episode": episode.episode,
                "source_start_index": episode.source_start_index,
                "source_end_index": episode.source_end_index,
                "close_index": episode.close_index,
                "open_index": episode.open_index,
                "minimum_bridge_z_mm": episode.minimum_bridge_z_mm,
            }
            for episode in representative.episodes
        ],
    }


def _write_representative_csv(
    path: Path,
    representative_a: RepresentativeTrajectory,
    representative_b: RepresentativeTrajectory,
) -> None:
    fields = [
        "role",
        "phase",
        "x_mm",
        "y_mm",
        "z_mm",
        "vx_mm_s",
        "vy_mm_s",
        "vz_mm_s",
        "tx",
        "ty",
        "tz",
        "prefix_length_mm",
        "suffix_length_mm",
        "mad_x_mm",
        "mad_y_mm",
        "mad_z_mm",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for representative in (representative_a, representative_b):
            for index, phase in enumerate(representative.phase):
                writer.writerow(
                    {
                        "role": representative.role,
                        "phase": float(phase),
                        "x_mm": float(representative.xyz_mm[index, 0]),
                        "y_mm": float(representative.xyz_mm[index, 1]),
                        "z_mm": float(representative.xyz_mm[index, 2]),
                        "vx_mm_s": float(representative.velocity_mm_s[index, 0]),
                        "vy_mm_s": float(representative.velocity_mm_s[index, 1]),
                        "vz_mm_s": float(representative.velocity_mm_s[index, 2]),
                        "tx": float(representative.tangent[index, 0]),
                        "ty": float(representative.tangent[index, 1]),
                        "tz": float(representative.tangent[index, 2]),
                        "prefix_length_mm": float(
                            representative.prefix_length_mm[index]
                        ),
                        "suffix_length_mm": float(
                            representative.suffix_length_mm[index]
                        ),
                        "mad_x_mm": float(representative.mad_mm[index, 0]),
                        "mad_y_mm": float(representative.mad_mm[index, 1]),
                        "mad_z_mm": float(representative.mad_mm[index, 2]),
                    }
                )


def _runtime_b_entry(
    selected: RepresentativeBridgeCandidate,
    representative_b: RepresentativeTrajectory,
    minimum_bridge_z_mm: float,
) -> dict[str, Any]:
    phase = selected.b_phase
    return {
        "entry_id": f"representative_b_entry:phase{phase:.4f}",
        "dataset": representative_b.dataset,
        "episode": -1,
        "frame": -1,
        "timestamp_s": None,
        "semantic_label": "b_representative_closed_holding_transport",
        "semantic_phase": phase,
        "semantic_state": {
            "gripper_closed": True,
            "holding": True,
            "contact_mode": "free_transport_assumed",
            "completed_subgoals": ["grasp_complete"],
            "object_state": None,
            "entry_preconditions": [],
        },
        "position_mm": representative_b.position(phase).tolist(),
        "demonstration_velocity_mm_s": representative_b.velocity(phase).tolist(),
        "B_retained_length_mm": representative_b.suffix_length(phase),
        "minimum_bridge_z_mm": minimum_bridge_z_mm,
        "velocity_method": "phase_aligned_episode_component_median",
        "velocity_sample_count": representative_b.episode_count,
        "velocity_source_episodes": [
            episode.episode for episode in representative_b.episodes
        ],
        "orientation_status": "pending",
        "ik_status": "NOT_CHECKED",
        "collision_status": "NOT_CHECKED_WITH_PAYLOAD",
        "robot_executable": False,
        "dry_run_only": True,
    }


def main() -> None:
    args = _parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    _validate_config(config)
    args.output.mkdir(parents=True, exist_ok=True)
    protected = (
        "runtime_transition_manifest.json",
        "representative_summary.json",
        "all_boundary_candidates.jsonl",
    )
    existing = [name for name in protected if (args.output / name).exists()]
    if existing:
        raise FileExistsError(
            f"refusing to overwrite V1 output files: {existing}"
        )

    started = time.perf_counter()
    trajectories_a, info_a = load_lerobot_trajectories(
        args.dataset_a, config["dataset_a_label"]
    )
    trajectories_b, info_b = load_lerobot_trajectories(
        args.dataset_b, config["dataset_b_label"]
    )
    action_hz = float(config["runtime"]["action_hz"])
    if not np.isclose(float(info_a["fps"]), action_hz):
        raise ValueError("dataset A FPS differs from V1 action_hz")
    if not np.isclose(float(info_b["fps"]), action_hz):
        raise ValueError("dataset B FPS differs from V1 action_hz")
    representative_a = build_representative_trajectory(
        trajectories_a,
        role="a_exit",
        config=TransportWindowConfig.from_dict(
            config["standardization"]["a"]
        ),
    )
    representative_b = build_representative_trajectory(
        trajectories_b,
        role="b_entry",
        config=TransportWindowConfig.from_dict(
            config["standardization"]["b"]
        ),
    )
    standardized_s = time.perf_counter()

    observed = np.concatenate(
        [trajectory.xyz_mm for trajectory in trajectories_a + trajectories_b],
        axis=0,
    )
    margin = float(config["workspace"]["margin_mm"])
    workspace_min = np.min(observed, axis=0) - margin
    workspace_max = np.max(observed, axis=0) + margin
    optimizer = RepresentativeBridgeOptimizer(
        representative_a=representative_a,
        representative_b=representative_b,
        config=config,
        workspace_min_mm=workspace_min,
        workspace_max_mm=workspace_max,
    )
    candidate_path = args.output / "all_boundary_candidates.jsonl"
    with candidate_path.open("w", encoding="utf-8") as candidate_handle:

        def record(candidate: RepresentativeBridgeCandidate) -> None:
            candidate_handle.write(
                json.dumps(candidate.to_record(), sort_keys=True) + "\n"
            )

        result = optimizer.optimize(record=record)
    optimized_s = time.perf_counter()
    selected = result.selected

    live = config["live_boundary"]
    boundary_contract = RepresentativeBoundaryContract(
        boundary_id=(
            f"representative_a_exit:phase{selected.a_phase:.4f}"
        ),
        center_position_mm=representative_a.position(selected.a_phase),
        representative_velocity_mm_s=representative_a.velocity(
            selected.a_phase
        ),
        representative_tangent=representative_a.unit_tangent(
            selected.a_phase
        ),
        prearm_radius_mm=float(live["prearm_radius_mm"]),
        commit_radius_mm=float(live["commit_radius_mm"]),
        direction_cosine_minimum=float(live["direction_cosine_minimum"]),
        approach_frames=int(live["approach_frames"]),
        commit_stable_frames=int(live["commit_stable_frames"]),
        approach_epsilon_mm=float(live["approach_epsilon_mm"]),
    )
    boundary_record = boundary_contract.to_record()
    boundary_record.update(
        {
            "semantic_phase": selected.a_phase,
            "semantic_requirements": {
                "open_before_close_observed": True,
                "gripper_closed": True,
                "seconds_since_close_minimum": 1.0,
                "tcp_z_minimum_mm": 400.0,
            },
            "training_support": representative_a.support_record(
                selected.a_phase,
                boundary_contract.commit_radius_mm,
            ),
            "prearm_training_support": representative_a.support_record(
                selected.a_phase,
                boundary_contract.prearm_radius_mm,
            ),
            "position_source": "actual_tcp",
            "bridge_start_position_source": "acknowledged_commanded_posx",
            "bridge_start_velocity_source": "causal_measured_tcp_history",
        }
    )

    np.savez_compressed(
        args.output / "representative_trajectories.npz",
        phase=representative_a.phase,
        a_xyz_mm=representative_a.xyz_mm,
        a_velocity_mm_s=representative_a.velocity_mm_s,
        a_acceleration_mm_s2=representative_a.acceleration_mm_s2,
        a_tangent=representative_a.tangent,
        a_covariance_mm2=representative_a.covariance_mm2,
        a_mad_mm=representative_a.mad_mm,
        a_prefix_length_mm=representative_a.prefix_length_mm,
        a_suffix_length_mm=representative_a.suffix_length_mm,
        b_xyz_mm=representative_b.xyz_mm,
        b_velocity_mm_s=representative_b.velocity_mm_s,
        b_acceleration_mm_s2=representative_b.acceleration_mm_s2,
        b_tangent=representative_b.tangent,
        b_covariance_mm2=representative_b.covariance_mm2,
        b_mad_mm=representative_b.mad_mm,
        b_prefix_length_mm=representative_b.prefix_length_mm,
        b_suffix_length_mm=representative_b.suffix_length_mm,
    )
    _write_representative_csv(
        args.output / "representative_trajectories.csv",
        representative_a,
        representative_b,
    )
    _write_json(args.output / "selected_candidate.json", selected.to_record())
    _write_json(
        args.output / "top_k_candidates.json",
        [candidate.to_record() for candidate in result.top_k],
    )
    _write_json(args.output / "config.snapshot.json", config)

    runtime = config["runtime"]
    runtime_sampling = {
        "bridge_sample_hz": float(config["sampling"]["runtime_sample_hz"]),
        "curvature_epsilon": float(config["sampling"]["curvature_epsilon"]),
    }
    # Offline representative optimization checks both endpoints. Live starts
    # from a measured A state, so its entry contract mirrors V0 runtime and
    # carries the B-side transport floor only.
    offline_required_floor = max(
        representative_a.minimum_bridge_z_mm,
        representative_b.minimum_bridge_z_mm,
    )
    runtime_b_floor = representative_b.minimum_bridge_z_mm
    runtime_manifest = {
        "version": config["version"],
        "mode": "representative_boundary_dry_run_design",
        "policy_checkpoints": runtime["policy_checkpoints"],
        "model_lifecycle": {
            "load_both_before_act_a": True,
            "warmup_outputs_discarded": True,
            "separate_a_b_generations_and_queues": True,
            "shared_gpu_inference_arbiter": True,
        },
        "representative_boundary": boundary_record,
        "representative_paths": {
            "A": _trajectory_summary(representative_a),
            "B": _trajectory_summary(representative_b),
            "artifact": str(
                (args.output / "representative_trajectories.npz").resolve()
            ),
            "standardization": "semantic_closed_transport_arc_length",
            "single_representative_per_task": True,
        },
        "velocity_contract": {
            "a_boundary_source": "causal_measured_tcp_history",
            "a_policy_chunk_role": "diagnostic_intent_only",
            "initial_b_source": "single_phase_aligned_representative",
            "initial_terminal_velocity": "planned_endpoint_stop",
            "final_b_source": "fresh_postprocessed_act_b_action_chunk",
            "single_frame_difference_used": False,
        },
        "payload_floor_contract": {
            "offline_representative_floor_mm": offline_required_floor,
            "offline_source": "max(A_quantile, B_quantile)",
            "runtime_floor_mm": runtime_b_floor,
            "runtime_source": "B_episode_floor_p95",
            "runtime_matches_v0_b_entry_semantics": True,
        },
        "transition_order": [
            "ACT_A_ACTIVE",
            "REPRESENTATIVE_BOUNDARY_PREARM_40MM",
            "REPRESENTATIVE_BOUNDARY_COMMIT_20MM",
            "CLEAR_ACT_A_QUEUE_AFTER_FINAL_PLAN",
            "BEZIER_INITIAL",
            "B_ENDPOINT_SETTLE",
            "FRESH_ACT_B_INFERENCE",
            "VELOCITY_MATCHED_TAIL",
            "GATED_ATOMIC_HANDOFF",
            "ACT_B_ACTIVE",
        ],
        "planners": {
            "initial_duration_search": config["search"]["duration_search"],
            "tail_duration_search": runtime["tail_duration_search"],
            "sampling": runtime_sampling,
            "feasibility": config["feasibility"],
            "selection": config["selection"],
            "semantic_compatibility": config["semantic_compatibility"],
        },
        "workspace": {
            "minimum_mm": workspace_min.tolist(),
            "maximum_mm": workspace_max.tolist(),
            "robot_certified": False,
        },
        "runtime_handoff": runtime["handoff"],
        "b_entries": [
            _runtime_b_entry(
                selected,
                representative_b,
                runtime_b_floor,
            )
        ],
        "offline_optimization": {
            "primary_objective": "total_C_length_mm",
            "coarse_sample_hz": config["sampling"]["coarse_sample_hz"],
            "final_sample_hz": config["sampling"]["final_sample_hz"],
            "coarse_evaluated": result.coarse_evaluated,
            "refined_evaluated": result.refined_evaluated,
            "final_validated": result.final_validated,
            "robust_validated": result.robust_validated,
            "selected_a_phase": selected.a_phase,
            "selected_b_phase": selected.b_phase,
            "selected_duration_s": selected.duration_s,
            "selected_total_c_length_mm": selected.total_c_length_mm,
            "selected_robust_pass_fraction": selected.robust_pass_fraction,
            "selected_a_prearm_support_fraction": (
                selected.a_prearm_support_fraction
            ),
            "selected_a_commit_support_fraction": (
                selected.a_commit_support_fraction
            ),
        },
        "safety": config["safety"],
    }
    _write_json(
        args.output / "runtime_transition_manifest.json", runtime_manifest
    )
    summary = {
        "version": config["version"],
        "input": {
            "dataset_a": str(args.dataset_a.resolve()),
            "dataset_b": str(args.dataset_b.resolve()),
            "dataset_a_fps": info_a["fps"],
            "dataset_b_fps": info_b["fps"],
            "config": str(args.config.resolve()),
            "config_sha256": _sha256(args.config),
        },
        "representative_A": _trajectory_summary(representative_a),
        "representative_B": _trajectory_summary(representative_b),
        "search": {
            "coarse_evaluated": result.coarse_evaluated,
            "refined_evaluated": result.refined_evaluated,
            "final_validated": result.final_validated,
            "robust_validated": result.robust_validated,
            "feasible_candidates": result.feasible_candidates,
            "failures_by_reason": result.failures_by_reason,
        },
        "selected": selected.to_record(),
        "representative_boundary": boundary_record,
        "timing_s": {
            "standardization": standardized_s - started,
            "optimization": optimized_s - standardized_s,
            "total_before_serialization": optimized_s - started,
        },
        "legacy_v0_preserved": True,
        "robot_commands_published": False,
    }
    _write_json(args.output / "representative_summary.json", summary)

    generated_files = sorted(
        path
        for path in args.output.iterdir()
        if path.is_file() and path.name != "checksums.sha256"
    )
    checksum_lines = [
        f"{_sha256(path)}  {path.name}" for path in generated_files
    ]
    (args.output / "checksums.sha256").write_text(
        "\n".join(checksum_lines) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
