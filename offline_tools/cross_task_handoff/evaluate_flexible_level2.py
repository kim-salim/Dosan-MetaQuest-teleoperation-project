#!/usr/bin/env python3
"""Command-free STRICT/FLEXIBLE reevaluation of a handoff candidate library."""

from __future__ import annotations

import argparse
import json
import time
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np

from lerobot_robot_doosan_a0509.task_c_handoff.bridge_runtime import (
    BridgeRuntimeLimits,
    BridgeRuntimeSnapshot,
)
from lerobot_robot_doosan_a0509.task_c_handoff.flexible_bridge import (
    FlexibleBridgeSearchConfig,
    search_flexible_bridge_queue,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    EpisodeHandoffManifest,
    HandoffV2Config,
)
from quest_a0509_teleop.doosan_orientation import (
    quaternion_to_doosan_zyz_deg,
)


def candidate_boundary_record(
    candidate: dict[str, Any], side: str
) -> dict[str, Any]:
    value = dict(candidate[side])
    velocity_key = (
        "nominal_source_velocity" if side == "source" else "nominal_successor_velocity"
    )
    return {
        "task": value["task"],
        "segment": value["segment"],
        "phase": value["phase"],
        "support_episode": value["support_episode"],
        "support_frame": value["support_frame"],
        "nominal_pose": {
            "position_mm": value["position_mm"],
            "orientation_quat_xyzw": value["orientation_quat_xyzw"],
        },
        "nominal_velocity_mm_s": candidate[velocity_key],
        "support_radius_mm": 90.0,
        "support_orientation_radius_deg": 30.0,
        "semantic": {
            "gripper_state": value["gripper_state"],
            "held_object": value["held_object"],
            "contact_mode": value["contact_mode"],
            "semantic_state": value.get("semantic_state", ""),
            "entry_preconditions": value.get("entry_preconditions", ()),
        },
    }


def candidate_to_flexible_manifest(
    library: dict[str, Any], candidate: dict[str, Any]
) -> EpisodeHandoffManifest:
    tangent = {
        "minimum_handle_chord_ratio": float(
            candidate.get("minimum_tangent_handle_chord_ratio", 0.0)
        ),
        "maximum_endpoint_speed_adjustment_mm_s": candidate.get(
            "maximum_endpoint_speed_adjustment_mm_s"
        ),
    }
    return EpisodeHandoffManifest.from_mapping(
        {
            "composition_id": library["composition_id"],
            "handoff_id": candidate["handoff_id"],
            "source": candidate_boundary_record(candidate, "source"),
            "successor": candidate_boundary_record(candidate, "successor"),
            "bridge": {
                "duration_s": candidate["bridge_duration_s"],
                "nominal_length_mm": candidate.get("bridge_length"),
                "transport_floor_mm": candidate.get("transport_floor_mm"),
            },
            "generation": {
                "method": "flexible_reference",
                "bridge_admission_mode": "flexible_level2",
                "reference_only": True,
                "bridge_algorithm": candidate.get(
                    "bridge_algorithm", "cubic_bezier_fixed"
                ),
                "tangent_regularization": tangent,
            },
            "validation": {
                "semantic_authority": library.get(
                    "semantic_authority", "runtime_guarded"
                ),
                "semantic_diagnostics": candidate.get(
                    "semantic_diagnostics", ()
                ),
                "hard_filter_passed": False,
                "robot_executable": False,
                "dry_run_only": True,
                "ik_checked": bool(candidate.get("ik_checked", False)),
                "collision_checked": bool(
                    candidate.get("collision_checked", False)
                ),
            },
        }
    )


def runtime_limits_from_validation(
    value: dict[str, Any],
) -> BridgeRuntimeLimits:
    workspace = value["workspace"]
    dynamics = value["dynamics"]
    downstream = value["downstream"]
    return BridgeRuntimeLimits(
        workspace_min_mm=np.asarray(workspace["minimum_mm"], dtype=np.float64),
        workspace_max_mm=np.asarray(workspace["maximum_mm"], dtype=np.float64),
        workspace_min_limit_enabled=tuple(workspace["minimum_limit_enabled"]),
        velocity_limit_mm_s=float(dynamics["velocity_limit_mm_s"]),
        axis_velocity_limit_mm_s=float(dynamics["axis_velocity_limit_mm_s"]),
        acceleration_limit_mm_s2=float(dynamics["acceleration_limit_mm_s2"]),
        curvature_limit_per_mm=float(dynamics["curvature_limit_per_mm"]),
        jerk_limit_mm_s3=float(dynamics["jerk_limit_mm_s3"]),
        integrated_squared_jerk_limit=float(
            dynamics["integrated_squared_jerk_limit"]
        ),
        backtracking_ratio_limit=float(dynamics["backtracking_ratio_limit"]),
        linear_ramp_mm_per_tick=float(downstream["linear_ramp_mm_per_tick"]),
        orientation_ramp_deg_per_tick=float(
            downstream["orientation_ramp_deg_per_tick"]
        ),
        sample_hz=float(value["sample_hz"]),
        curvature_epsilon=float(value["curvature_epsilon"]),
        ack_pipeline_max_lag_steps=1,
    )


def evaluate(
    library_path: Path,
    validation_path: Path,
    *,
    limit: int | None = None,
) -> tuple[dict[str, Any], EpisodeHandoffManifest | None]:
    library = json.loads(library_path.read_text(encoding="utf-8"))
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    candidates = list(library["candidates"])
    if limit is not None:
        candidates = candidates[:limit]
    limits = runtime_limits_from_validation(validation)
    runtime = HandoffV2Config(
        enabled=True,
        control_hz=float(validation["downstream"]["control_hz"]),
        bridge_admission_mode="flexible_level2",
        adaptive_b_max_splice_index=8,
        adaptive_b_max_candidates=6,
        semantic_authority=library.get("semantic_authority", "runtime_guarded"),
    )
    search_config = FlexibleBridgeSearchConfig()
    records: list[dict[str, Any]] = []
    rejection_counts: Counter[str] = Counter()
    selected_values: list[tuple[float, EpisodeHandoffManifest, dict[str, Any]]] = []
    started = time.perf_counter()
    for candidate in candidates:
        manifest = candidate_to_flexible_manifest(library, candidate)
        source = manifest.source
        reference_angles = quaternion_to_doosan_zyz_deg(
            source.nominal_orientation_quat_xyzw,
            [0.0, 150.0, 0.0],
        )
        pose = np.concatenate((source.nominal_position_mm, reference_angles))
        snapshot = BridgeRuntimeSnapshot(
            timestamp_s=time.monotonic(),
            actual_pose_mm_deg=pose,
            acknowledged_pose_mm_deg=pose,
            actual_velocity_mm_s=source.nominal_velocity_mm_s,
            gripper_target=source.semantic.gripper_target,
        )
        result = search_flexible_bridge_queue(
            manifest,
            snapshot,
            runtime,
            limits,
            search_config,
            live_mode=False,
        )
        for reason, count in result.rejected_reason_counts.items():
            rejection_counts[reason] += count
        record = {
            "handoff_id": candidate["handoff_id"],
            "source_support_episode": candidate["source_support_episode"],
            "successor_support_episode": candidate["successor_support_episode"],
            "strict_feasible": bool(candidate.get("feasible", False)),
            "strict_rejection_reason": candidate.get("rejection_reason", []),
            "legacy_max_curvature_per_mm": candidate.get(
                "max_curvature_per_mm"
            ),
            "flexible": result.record(),
        }
        records.append(record)
        if result.valid and result.selected is not None:
            selected_values.append(
                (float(result.selected.score), manifest, record)
            )

    selected = None if not selected_values else min(selected_values, key=lambda item: item[0])
    search_latencies_ms = np.asarray(
        [item["flexible"]["search_latency_ms"] for item in records],
        dtype=np.float64,
    )
    valid_records = [item for item in records if item["flexible"]["valid"]]
    selected_generators = Counter(
        item["flexible"]["selected"]["generator_type"]
        for item in valid_records
    )
    selected_durations = Counter(
        str(item["flexible"]["selected"]["duration_s"])
        for item in valid_records
    )
    summary = {
        "schema_version": "a0509.flexible_level2_offline_evaluation.v1",
        "source_candidate_library": str(library_path.resolve()),
        "validation_config": str(validation_path.resolve()),
        "source_operator": library.get("source_operator"),
        "successor_operator": library.get("successor_operator"),
        "candidate_count": len(candidates),
        "strict_feasible_count": sum(item["strict_feasible"] for item in records),
        "strict_curvature_only_rejection_count": sum(
            item["strict_rejection_reason"] == ["curvature_limit"]
            for item in records
        ),
        "flexible_command_safe_count": sum(item["flexible"]["valid"] for item in records),
        "flexible_rejection_reason_counts": dict(rejection_counts),
        "flexible_search_latency_ms": {
            "p50": float(np.quantile(search_latencies_ms, 0.50)),
            "p95": float(np.quantile(search_latencies_ms, 0.95)),
            "p99": float(np.quantile(search_latencies_ms, 0.99)),
            "max": float(np.max(search_latencies_ms)),
        },
        "flexible_selected_generator_counts": dict(selected_generators),
        "flexible_selected_duration_s_counts": dict(selected_durations),
        "flexible_max_candidates_evaluated_per_pair": max(
            item["flexible"]["candidates_evaluated"] for item in records
        ),
        "selected": None if selected is None else selected[2],
        "elapsed_s": time.perf_counter() - started,
        "robot_commands_published": 0,
        "live_enabled": False,
        "mux_selected": False,
        "act_b_shadow_performed": False,
        "takeover_physically_validated": False,
        "ik_checker_available": bool(validation.get("ik_checker_available", False)),
        "collision_checker_available": bool(
            validation.get("collision_checker_available", False)
        ),
        "semantic_authority": library.get("semantic_authority"),
        "limitations": [
            "nominal support snapshots only; no live actual/ACK residual replay",
            "fresh ACT-B prefix admission requires a separate command-free shadow",
            "no IK or environment collision checker is available",
        ],
        "records": records,
    }
    return summary, None if selected is None else selected[1]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate-library", required=True, type=Path)
    parser.add_argument("--validation-config", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--limit", type=int)
    args = parser.parse_args()
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    summary, manifest = evaluate(
        args.candidate_library,
        args.validation_config,
        limit=args.limit,
    )
    (output / "evaluation.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if manifest is not None:
        (output / "selected_flexible_reference_manifest.json").write_text(
            json.dumps(manifest.to_record(), ensure_ascii=False, indent=2, sort_keys=True)
            + "\n",
            encoding="utf-8",
        )
    print(json.dumps({key: value for key, value in summary.items() if key != "records"}, indent=2))


if __name__ == "__main__":
    main()
