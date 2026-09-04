#!/usr/bin/env python3
"""Rebuild a semantic-local execution-tail reference without robot I/O.

The historical default remains T7 -> T3 so existing reproduction commands are
unchanged. Explicit operator/tail arguments let another reviewed edge reuse the
same support-bank and ``prefix + Bridge + suffix`` pipeline.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Mapping

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "src/lerobot_robot_doosan_a0509"
QUEST_PACKAGE_ROOT = REPOSITORY_ROOT / "src/quest_a0509_teleop"
for value in (
    str(REPOSITORY_ROOT),
    str(PACKAGE_ROOT),
    str(QUEST_PACKAGE_ROOT),
):
    if value not in sys.path:
        sys.path.insert(0, value)

from lerobot_robot_doosan_a0509.interior_policy.catalog import (  # noqa: E402
    load_operator_catalog,
)
from lerobot_robot_doosan_a0509.interior_policy.execution_tail import (  # noqa: E402
    ExecutionTailDerivationConfig,
    derive_zero_cost_execution_tail,
)
from lerobot_robot_doosan_a0509.interior_policy.execution_tail_reference import (  # noqa: E402
    MEDOID_SELECTION_METHOD,
    SEMANTIC_LOCAL_SELECTION_METHOD,
    boundary_mapping_from_reference_bank,
    build_runtime_reference_bank,
    derive_empirical_transport_phase_window,
)
from lerobot_robot_doosan_a0509.interior_policy.semantic_local_reference import (
    SemanticLocalSelectionConfig,
    select_semantic_local_reference,
)
from lerobot_robot_doosan_a0509.task_c_handoff.bridge_runtime import (  # noqa: E402
    BridgeRuntimeSnapshot,
)
from lerobot_robot_doosan_a0509.task_c_handoff.flexible_bridge import (  # noqa: E402
    FlexibleBridgeSearchConfig,
    search_flexible_bridge_queue,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (  # noqa: E402
    EpisodeHandoffManifest,
    HandoffV2Config,
)
from offline_tools.cross_task_handoff.evaluate_flexible_level2 import (  # noqa: E402
    runtime_limits_from_validation,
)
from quest_a0509_teleop.doosan_orientation import (  # noqa: E402
    quaternion_to_doosan_zyz_deg,
)


DEFAULT_CATALOG = (
    REPOSITORY_ROOT
    / "config/interior_policy/a0509_t1_t6_operator_catalog_v1.json"
)
DEFAULT_BASELINE = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_flexible_level2_registry_2026-08-30"
    / "edges/t7_acquire_from_drawer_top__to__t3_deliver_to_floor"
    / "flexible_reference_manifest.json"
)
DEFAULT_VALIDATION = (
    REPOSITORY_ROOT
    / "offline_tools/cross_task_handoff/validation_config_a0509_v2.json"
)
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t7_to_t3_s2_runtime_reference_2026-08-31"
)


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--baseline-manifest", type=Path, default=DEFAULT_BASELINE)
    parser.add_argument("--validation-config", type=Path, default=DEFAULT_VALIDATION)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--source-operator", default="T7.acquire_from_drawer_top"
    )
    parser.add_argument("--successor-operator", default="T3.deliver_to_floor")
    parser.add_argument(
        "--source-reference-mode",
        choices=("execution_tail", "exact_semantic_exit"),
        default="execution_tail",
    )
    parser.add_argument(
        "--allow-reviewed-empty-gripper-tail",
        action="store_true",
        help=(
            "Explicitly allow the selected empty-gripper/free-space source."
        ),
    )
    parser.add_argument(
        "--successor-reference-domain",
        choices=("empirical_transport", "operator_entry_segment"),
        default="empirical_transport",
    )
    parser.add_argument("--expected-source-segment", default="S2")
    parser.add_argument("--expected-commit-phase-low", type=float, default=0.16)
    parser.add_argument("--expected-commit-phase-high", type=float, default=0.21)
    parser.add_argument(
        "--successor-interior-path-margin-mm", type=float, default=15.0
    )
    parser.add_argument(
        "--successor-support-start-quantile", type=float, default=0.95
    )
    parser.add_argument(
        "--successor-support-end-quantile", type=float, default=0.05
    )
    parser.add_argument(
        "--minimum-transport-clearance-mm",
        type=float,
        default=50.0,
        help=(
            "Offline successor-window height margin above grasp/release events. "
            "This is reference-domain filtering, not a runtime Z safety gate."
        ),
    )
    parser.add_argument(
        "--bridge-minimum-z-mm",
        type=float,
        default=None,
        help=(
            "Absolute base-frame TCP Z floor enforced over the complete "
            "Bridge. Source and successor endpoints must also satisfy it."
        ),
    )
    parser.add_argument(
        "--successor-candidate-phase-step", type=float, default=0.02
    )
    parser.add_argument("--max-pair-candidates", type=int, default=384)
    parser.add_argument("--selection-rank", type=int, default=0)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args(argv)


def _write(path: Path, value: Mapping[str, Any], *, force: bool) -> None:
    if path.exists() and not force:
        raise FileExistsError(f"refusing to overwrite artifact: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _snapshot(manifest: EpisodeHandoffManifest) -> BridgeRuntimeSnapshot:
    orientation = quaternion_to_doosan_zyz_deg(
        manifest.source.nominal_orientation_quat_xyzw,
        [0.0, 150.0, 0.0],
    )
    pose = np.concatenate((manifest.source.nominal_position_mm, orientation))
    return BridgeRuntimeSnapshot(
        timestamp_s=time.monotonic(),
        actual_pose_mm_deg=pose,
        acknowledged_pose_mm_deg=pose,
        actual_velocity_mm_s=manifest.source.nominal_velocity_mm_s,
        gripper_target=manifest.source.semantic.gripper_target,
    )


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(argv)
    output_root = args.output_root.expanduser().resolve()
    catalog = load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        config_path=args.catalog,
        audit_raw_frames=False,
    )
    source_operator = catalog.by_id[str(args.source_operator)]
    successor_operator = catalog.by_id[str(args.successor_operator)]
    source_task = source_operator.policy_id
    successor_task = successor_operator.policy_id
    tail = None
    if args.source_reference_mode == "execution_tail":
        tail_config = (
            ExecutionTailDerivationConfig(
                reviewed_empty_gripper_free_space_operators=(source_operator.id,)
            )
            if args.allow_reviewed_empty_gripper_tail
            else None
        )
        tail_result = derive_zero_cost_execution_tail(source_operator, config=tail_config)
        if tail_result.profile is None or tail_result.reason != "derived":
            raise RuntimeError(
                f"{source_operator.id} execution tail unavailable: "
                f"{tail_result.reason}"
            )
        tail = tail_result.profile
        source_segment = tail.tracking_segment
        source_phase_window = (
            tail.commit_phase_low,
            tail.commit_phase_high,
        )
        source_nominal_phase = tail.nominal_phase
        source_role = "source_execution_tail"
        source_reference_phase_max = tail.nominal_phase
    else:
        source_segment = source_operator.evidence.exit_segment
        source_nominal_phase = source_operator.evidence.exit_phase
        source_phase_window = (
            source_nominal_phase,
            source_nominal_phase,
        )
        source_role = "source_semantic_exit"
        source_reference_phase_max = source_nominal_phase

    expected = (
        str(args.expected_source_segment),
        float(args.expected_commit_phase_low),
        float(args.expected_commit_phase_high),
    )
    actual = (
        source_segment,
        source_phase_window[0],
        source_phase_window[1],
    )
    if actual[0] != expected[0] or not np.allclose(
        actual[1:], expected[1:], atol=1.0e-12, rtol=0.0
    ):
        raise RuntimeError(
            f"{source_operator.id} source reference contract changed: "
            f"expected={expected} actual={actual}"
        )

    source_audit = catalog.audit.tasks[source_task]
    successor_audit = catalog.audit.tasks[successor_task]
    if args.successor_reference_domain == "empirical_transport":
        successor_window_evidence = derive_empirical_transport_phase_window(
            dataset_root=successor_audit.dataset_root,
            phase_support_artifact=successor_audit.phase_support_artifact,
            segment=successor_operator.evidence.entry_segment,
            expected_episode_count=successor_audit.episode_count,
            start_offset_frames=30,
            end_offset_frames=-30,
            minimum_transport_clearance_mm=args.minimum_transport_clearance_mm,
            start_quantile=args.successor_support_start_quantile,
            end_quantile=args.successor_support_end_quantile,
        )
    else:
        successor_low = float(successor_operator.evidence.entry_phase)
        successor_high = float(
            successor_operator.evidence.exit_phase
            if (
                successor_operator.evidence.entry_segment
                == successor_operator.evidence.exit_segment
            )
            else 1.0
        )
        if successor_low >= successor_high:
            raise RuntimeError(
                f"{successor_operator.id} entry semantic interval is empty"
            )
        successor_window_evidence = {
            "schema_version": (
                "a0509.operator_entry_semantic_phase_window.v1"
            ),
            "segment": successor_operator.evidence.entry_segment,
            "phase_window": [successor_low, successor_high],
            "episode_count": successor_audit.episode_count,
            "role": (
                "successor_pre_event_candidate_domain_not_runtime_safety_gate"
            ),
            "required_gripper_state": "open",
            "required_held_object": "none",
            "terminal_event_excluded_by_semantic_compatibility": True,
            "runtime_phase_gate": False,
            "robot_commands_published": 0,
            "physical_validation_performed": False,
        }
    successor_phase_window = tuple(
        float(item) for item in successor_window_evidence["phase_window"]
    )
    successor_nominal_phase = float(
        np.round(np.mean(successor_phase_window) / 0.005) * 0.005
    )
    source_bank = build_runtime_reference_bank(
        reference_id=(
            f"{source_operator.id}:{source_segment}_"
            f"{args.source_reference_mode}"
        ),
        role=source_role,
        source_operator=source_operator.id,
        task_id=source_task,
        dataset_root=source_audit.dataset_root,
        semantic_graph_path=source_audit.semantic_artifact,
        phase_support_artifact=source_audit.phase_support_artifact,
        segment=source_segment,
        phase_window=source_phase_window,
        nominal_phase=source_nominal_phase,
    )
    successor_bank = build_runtime_reference_bank(
        reference_id=(
            f"{successor_operator.id}:"
            f"{successor_operator.evidence.entry_segment}_"
            "empirical_transport_interior"
        ),
        role="successor_entry",
        source_operator=None,
        task_id=successor_task,
        dataset_root=successor_audit.dataset_root,
        semantic_graph_path=successor_audit.semantic_artifact,
        phase_support_artifact=successor_audit.phase_support_artifact,
        segment=successor_operator.evidence.entry_segment,
        phase_window=successor_phase_window,
        nominal_phase=successor_nominal_phase,
    )

    source_bank_path = output_root / (
        f"{source_task.lower()}_{source_segment.lower()}_"
        f"{args.source_reference_mode}_source_bank.json"
    )
    successor_bank_path = output_root / (
        f"{successor_task.lower()}_"
        f"{successor_operator.evidence.entry_segment.lower()}_"
        "entry_reference_bank.json"
    )
    successor_window_path = output_root / (
        f"{successor_task.lower()}_"
        f"{successor_operator.evidence.entry_segment.lower()}_"
        f"{args.successor_reference_domain}_phase_window.json"
    )
    selection_path = output_root / "semantic_local_reference_selection.json"

    baseline = EpisodeHandoffManifest.load(args.baseline_manifest)
    value = baseline.to_record()
    value["bridge"]["transport_floor_mm"] = args.bridge_minimum_z_mm
    value["source"] = boundary_mapping_from_reference_bank(source_bank)
    value["successor"] = boundary_mapping_from_reference_bank(
        successor_bank,
        support_radius_mm=90.0,
    )
    generation = dict(value["generation"])
    if tail is None:
        generation.pop("runtime_source_reference", None)
    else:
        generation["runtime_source_reference"] = {
            "support_bank": source_bank_path.name,
            "selection_method": MEDOID_SELECTION_METHOD,
            "phase_window": list(source_phase_window),
        }
    generation.pop("runtime_successor_reference", None)
    value["generation"] = generation
    validation = dict(value["validation"])
    validation["semantic_diagnostics"] = []
    validation["runtime_source_bank_validated"] = tail is not None
    value["validation"] = validation
    template_manifest = EpisodeHandoffManifest.from_mapping(value)

    validation_config = json.loads(
        args.validation_config.read_text(encoding="utf-8")
    )
    limits = runtime_limits_from_validation(validation_config)
    runtime = HandoffV2Config(
        enabled=True,
        control_hz=30.0,
        bridge_admission_mode="flexible_level2",
        adaptive_b_max_splice_index=8,
        adaptive_b_max_candidates=6,
        semantic_authority="external_planner",
    )
    bridge_search_config = FlexibleBridgeSearchConfig(
        max_candidates=64,
        max_search_time_s=0.20,
    )
    local_config = SemanticLocalSelectionConfig(
        successor_interior_path_margin_mm=(
            args.successor_interior_path_margin_mm
        ),
        source_phase_step=0.005,
        successor_phase_step=args.successor_candidate_phase_step,
        max_pair_candidates=args.max_pair_candidates,
        selected_candidate_rank=args.selection_rank,
        # An execution tail must remain causal; an exact semantic-exit source
        # is fixed to its declared boundary and therefore has the same bound.
        source_reference_phase_max=source_reference_phase_max,
    )
    selection = select_semantic_local_reference(
        base_manifest=template_manifest,
        source_bank=source_bank,
        successor_bank=successor_bank,
        source_phase_support_artifact=source_audit.phase_support_artifact,
        successor_phase_support_artifact=(
            successor_audit.phase_support_artifact
        ),
        runtime_successor_bank_path=successor_bank_path.name,
        runtime_config=runtime,
        limits=limits,
        bridge_search_config=bridge_search_config,
        config=local_config,
    )
    if not selection.valid:
        raise RuntimeError(
            f"semantic-local {source_task}->{successor_task} search produced "
            "no command-safe reference"
        )
    assert selection.manifest is not None
    assert selection.source_point is not None
    assert selection.successor_point is not None
    assert selection.geometry is not None
    source_bank["selected_boundary"] = selection.source_point.to_record()
    successor_bank["selected_boundary"] = (
        selection.successor_point.to_record()
    )
    objective = {
        "source_prefix_mm": selection.source_prefix_mm,
        "bridge_length_mm": selection.bridge_length_mm,
        "successor_suffix_mm": selection.successor_suffix_mm,
        "total_length_mm": selection.total_length_mm,
    }
    successor_bank["successor_candidate_phase_window"] = (
        successor_window_evidence
    )
    successor_bank["semantic_local_selection"] = {
        "method": SEMANTIC_LOCAL_SELECTION_METHOD,
        "runtime_phase_gate": False,
        "authority": "bridge_reference_and_metadata_only",
        "selected_phase": selection.successor_point.phase,
        "selected_episode": selection.successor_point.support_episode,
        "selected_frame": selection.successor_point.support_frame,
        "candidate_rank": selection.selected_candidate_rank,
        "interior_path_margin_mm": (
            args.successor_interior_path_margin_mm
        ),
        "path_margin_from_low_mm": selection.path_margin_from_low_mm,
        "path_margin_to_high_mm": selection.path_margin_to_high_mm,
        "semantic_local_objective": objective,
        "dijkstra_operator_cost_changed": False,
        "fixed_z_minimum_enabled": (
            selection.manifest.transport_floor_mm is not None
        ),
        "bridge_minimum_z_mm": selection.manifest.transport_floor_mm,
    }
    source_bank["semantic_local_selection"] = {
        "method": SEMANTIC_LOCAL_SELECTION_METHOD,
        "selected_phase": selection.source_point.phase,
        "selected_episode": selection.source_point.support_episode,
        "selected_frame": selection.source_point.support_frame,
        "candidate_rank": selection.selected_candidate_rank,
        "semantic_local_objective": objective,
        "source_reference_mode": args.source_reference_mode,
        "zero_symbolic_cost_execution_tail": tail is not None,
        "mandatory_semantic_exit_preserved": tail is None,
        "fixed_z_minimum_enabled": (
            selection.manifest.transport_floor_mm is not None
        ),
        "bridge_minimum_z_mm": selection.manifest.transport_floor_mm,
    }
    _write(
        successor_window_path,
        successor_window_evidence,
        force=args.force,
    )
    _write(source_bank_path, source_bank, force=args.force)
    _write(successor_bank_path, successor_bank, force=args.force)
    _write(selection_path, selection.to_record(), force=args.force)

    manifest = selection.manifest
    manifest_path = output_root / "flexible_reference_manifest.json"
    _write(manifest_path, manifest.to_record(), force=args.force)

    geometry = search_flexible_bridge_queue(
        manifest,
        _snapshot(manifest),
        runtime,
        limits,
        bridge_search_config,
        live_mode=False,
    )
    geometry_record = {
        "schema_version": "a0509.semantic_local_reference_geometry.v2",
        "manifest": str(manifest_path),
        "source_bank": str(source_bank_path),
        "successor_bank": str(successor_bank_path),
        "successor_candidate_window": str(successor_window_path),
        "semantic_local_selection": str(selection_path),
        "source_reference_mode": args.source_reference_mode,
        "source_execution_tail": (
            None if tail is None else tail.to_record()
        ),
        "source_selection": source_bank["selection"],
        "successor_selection": successor_bank["selection"],
        "selected_source_boundary": selection.source_point.to_record(),
        "selected_successor_boundary": selection.successor_point.to_record(),
        "semantic_local_objective": objective,
        "successor_phase_role": "bridge_reference_and_metadata_only",
        "runtime_successor_phase_gate": False,
        "successor_interior_path_margin_mm": (
            args.successor_interior_path_margin_mm
        ),
        "bridge_minimum_z_mm": manifest.transport_floor_mm,
        "fixed_z_minimum_enabled": (
            manifest.transport_floor_mm is not None
        ),
        "geometry": geometry.record(),
        "robot_commands_published": 0,
        "live_enabled": False,
        "mux_selected": False,
        "physical_validation_performed": False,
    }
    _write(
        output_root / "geometry_evaluation.json",
        geometry_record,
        force=args.force,
    )

    if not geometry.valid:
        raise RuntimeError(
            f"{manifest.source.segment}->{manifest.successor.segment} "
            "reference did not produce a command-safe offline Bridge"
        )
    overlay = {
        "schema_version": "a0509.flexible_level2_verified_overlay.v1",
        "description": (
            "Command-free semantic-local reference overlay. Registry refresh "
            "must independently verify the matching policy shadow."
        ),
        "edges": [
            {
                "source_operator": source_operator.id,
                "successor_operator": successor_operator.id,
                "handoff_manifest": manifest_path.name,
                "policy_shadow": "flexible_policy_shadow.json",
                "validation_method": (
                    (
                        f"execution_tail_{manifest.source.segment.lower()}"
                        if tail is not None
                        else (
                            "exact_semantic_exit_"
                            f"{manifest.source.segment.lower()}"
                        )
                    )
                    + "_semantic_local_interior_dynamic_future_join"
                    + (
                        "_transport_floor"
                        if manifest.transport_floor_mm is not None
                        else ""
                    )
                ),
                "uncertainty_penalty": 0.0,
                "evidence": {
                    "command_safe": True,
                    "physical_trials": 0,
                    "ik_checked": False,
                    "collision_checked": False,
                    "runtime_source_segment": manifest.source.segment,
                    "runtime_source_phase_window": list(
                        source_phase_window
                    ),
                    "runtime_source_reference_phase": manifest.source.phase,
                    "runtime_source_reference_phase_max": (
                        source_reference_phase_max
                    ),
                    "source_reference_mode": args.source_reference_mode,
                    "reviewed_empty_gripper_tail": bool(args.allow_reviewed_empty_gripper_tail),
                    "mandatory_source_semantic_exit_preserved": (
                        tail is None
                    ),
                    "runtime_source_medoid_episode": (
                        source_bank["selection"]["medoid_episode"]
                    ),
                    "successor_reference_segment": (
                        manifest.successor.segment
                    ),
                    "successor_reference_phase_window": list(
                        successor_phase_window
                    ),
                    "successor_reference_phase": (
                        manifest.successor.phase
                    ),
                    "successor_reference_medoid_episode": (
                        successor_bank["selection"]["medoid_episode"]
                    ),
                    "successor_reference_interior_path_margin_mm": (
                        args.successor_interior_path_margin_mm
                    ),
                    "successor_runtime_phase_gate": False,
                    "semantic_local_candidate_rank": (
                        selection.selected_candidate_rank
                    ),
                    "semantic_local_total_length_mm": (
                        manifest.semantic_local_total_length_mm
                    ),
                    "bridge_transport_floor_mm": (
                        manifest.transport_floor_mm
                    ),
                    "dijkstra_operator_cost_changed": False,
                },
            }
        ],
        "robot_commands_published": 0,
        "physical_validation_performed": False,
    }
    _write(
        output_root / "level2_registry_overlay.json",
        overlay,
        force=args.force,
    )
    print(
        f"{source_task}_{successor_task}_SEMANTIC_LOCAL_REFERENCE_OK "
        f"rank={selection.selected_candidate_rank} "
        f"source_phase={manifest.source.phase:.3f} "
        f"successor_phase={manifest.successor.phase:.3f} "
        f"successor_medoid={successor_bank['selection']['medoid_episode']} "
        f"total_mm={manifest.semantic_local_total_length_mm:.3f} "
        f"handoff_id={manifest.handoff_id} "
        f"output={output_root}"
    )


if __name__ == "__main__":
    main()
