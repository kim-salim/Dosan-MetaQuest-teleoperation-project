#!/usr/bin/env python3
"""Summarize command-free T7/T8 edge evidence and emit a strict V2 registry."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import os
from pathlib import Path
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ARTIFACT_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t7_t8_level2_edge_validation_2026-08-29"
)

EDGE_SPECS = (
    ("t1_acquire_to_t7_deliver", "T1.acquire_from_black_table", "T7.deliver_to_black_table", "open_then_closed"),
    ("t2_acquire_to_t7_deliver", "T2.acquire_from_floor", "T7.deliver_to_black_table", "open_then_closed"),
    ("t3_acquire_to_t7_deliver", "T3.acquire_from_black_table", "T7.deliver_to_black_table", "open_then_closed"),
    ("t4_acquire_to_t7_deliver", "T4.acquire_from_black_table", "T7.deliver_to_black_table", "open_then_closed"),
    ("t5_acquire_to_t7_deliver", "T5.acquire_from_drawer", "T7.deliver_to_black_table", "open_then_closed"),
    ("t6_acquire_to_t7_deliver", "T6.acquire_moving_block", "T7.deliver_to_black_table", "open_then_closed"),
    ("t6_acquire_to_t8_deliver", "T6.acquire_moving_block", "T8.deliver_unstacked_to_black_table", "open_then_closed"),
    ("t6_stack_to_t8_acquire", "T6.stack_on_support_block", "T8.acquire_top_block_from_stack", "closed_then_open"),
    ("t7_acquire_to_t2_deliver", "T7.acquire_from_drawer_top", "T2.deliver_to_black_table", "open_then_closed"),
    ("t7_acquire_to_t3_deliver", "T7.acquire_from_drawer_top", "T3.deliver_to_floor", "open_then_closed"),
    ("t7_deliver_to_t3_acquire", "T7.deliver_to_black_table", "T3.acquire_from_black_table", "closed_then_open"),
    ("t8_acquire_to_t2_deliver", "T8.acquire_top_block_from_stack", "T2.deliver_to_black_table", "open_then_closed"),
    ("t8_acquire_to_t3_deliver", "T8.acquire_top_block_from_stack", "T3.deliver_to_floor", "open_then_closed"),
    ("t8_acquire_to_t6_stack", "T8.acquire_top_block_from_stack", "T6.stack_on_support_block", "open_then_closed"),
    ("t8_acquire_to_t7_deliver", "T8.acquire_top_block_from_stack", "T7.deliver_to_black_table", "open_then_closed"),
    ("t8_deliver_to_t3_acquire", "T8.deliver_unstacked_to_black_table", "T3.acquire_from_black_table", "closed_then_open"),
)

EXISTING_LEVEL2_EDGES = (
    {
        "source_operator": "T4.open_drawer",
        "successor_operator": "T2.acquire_from_floor",
        "handoff_manifest": (
            "../task_c_t4_t2_t4_auto_phase_multi_v2_2026-08-29/"
            "t4_to_t2/episode_manifests/h_454934a7c93e.json"
        ),
        "policy_shadow": (
            "../task_c_t4_t2_t4_auto_phase_multi_v2_2026-08-29/"
            "policy_shadow_t4_to_t2_h_454934a7c93e.json"
        ),
        "gripper_event": "closed_then_open",
    },
    {
        "source_operator": "T2.acquire_from_floor",
        "successor_operator": "T4.deliver_to_drawer",
        "handoff_manifest": (
            "../task_c_t4_t2_t4_auto_phase_multi_v2_2026-08-29/"
            "t2_to_t4/episode_manifests/h_dfa226e0e5fd.json"
        ),
        "policy_shadow": (
            "../task_c_t4_t2_t4_auto_phase_multi_v2_2026-08-29/"
            "policy_shadow_t2_to_t4_h_dfa226e0e5fd.json"
        ),
        "gripper_event": "open_then_closed",
    },
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact-root", type=Path, default=DEFAULT_ARTIFACT_ROOT)
    return parser.parse_args()


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _shadow_passes(value: dict[str, Any]) -> bool:
    admission = value.get("prefix_admission")
    timing = value.get("control_timing")
    return bool(
        value.get("schema_version")
        == "a0509.task_c_handoff_v2_policy_shadow.v1"
        and value.get("robot_commands_published") == 0
        and value.get("live_enabled") is False
        and value.get("mux_selected") is False
        and value.get("terminal_state") == "RUN_B"
        and value.get("takeover_success") is True
        and value.get("fallback_required") is False
        and isinstance(admission, dict)
        and admission.get("valid") is True
        and isinstance(timing, dict)
        and int(timing.get("deadline_miss_count", -1)) == 0
    )


def _relative(path: Path, base: Path) -> str:
    return os.path.relpath(path.resolve(), start=base.resolve())


def _shadow_summary(path: Path, value: dict[str, Any]) -> dict[str, Any]:
    admission = value.get("prefix_admission") or {}
    dynamics = admission.get("dynamics") or {}
    execution = admission.get("execution_dynamics") or {}
    timing = value.get("control_timing") or {}
    return {
        "path": str(path.resolve()),
        "handoff_id": value.get("handoff_id"),
        "passes_level2_shadow_contract": _shadow_passes(value),
        "terminal_state": value.get("terminal_state"),
        "takeover_success": value.get("takeover_success"),
        "failure_reasons": list(admission.get("failure_reasons") or ()),
        "first_xyz_delta_mm": dynamics.get("first_xyz_delta_mm"),
        "first_rotation_delta_deg": dynamics.get("first_rotation_delta_deg"),
        "bridge_prefix_velocity_mismatch_mm_s": dynamics.get(
            "bridge_prefix_velocity_mismatch_mm_s"
        ),
        "crossfade_max_acceleration_mm_s2": execution.get(
            "max_acceleration_mm_s2"
        ),
        "control_p99_ms": timing.get("p99_ms"),
        "deadline_miss_count": timing.get("deadline_miss_count"),
        "robot_commands_published": value.get("robot_commands_published"),
    }


def run(artifact_root: Path) -> dict[str, Any]:
    artifact_root = artifact_root.expanduser().resolve()
    candidate_root = artifact_root / "candidates"
    edge_root = artifact_root / "edges"
    records: list[dict[str, Any]] = []
    registry_edges = [dict(item) for item in EXISTING_LEVEL2_EDGES]

    for edge_id, source, successor, gripper_event in EDGE_SPECS:
        library_path = candidate_root / f"{edge_id}.json"
        library = _load(library_path)
        candidates = list(library["candidates"])
        feasible = [item for item in candidates if bool(item["feasible"])]
        reasons: Counter[str] = Counter()
        for candidate in candidates:
            if candidate["feasible"]:
                continue
            raw = candidate.get("rejection_reason")
            if isinstance(raw, list):
                reasons.update(str(item) for item in raw)
            elif raw:
                reasons[str(raw)] += 1

        manifests_dir = edge_root / edge_id / "episode_manifests"
        selection_path = manifests_dir / "selection_summary.json"
        selection = _load(selection_path) if selection_path.is_file() else {}
        selected_ids = list(selection.get("selected_handoff_ids", ()))

        shadows: list[dict[str, Any]] = []
        shadow_by_id: dict[str, tuple[Path, dict[str, Any]]] = {}
        for shadow_path in sorted((edge_root / edge_id).glob("policy_shadow_*.json")):
            value = _load(shadow_path)
            shadows.append(_shadow_summary(shadow_path, value))
            shadow_by_id[str(value.get("handoff_id"))] = (shadow_path, value)

        passing_ids = [
            handoff_id
            for handoff_id in selected_ids
            if handoff_id in shadow_by_id
            and _shadow_passes(shadow_by_id[handoff_id][1])
        ]
        representative_id = passing_ids[0] if passing_ids else None
        level = (
            "LEVEL_2_RUNTIME_CONTRACT_COMPATIBLE"
            if representative_id is not None
            else "LEVEL_1_SYMBOLICALLY_PLANNABLE"
        )
        record = {
            "edge_id": edge_id,
            "source_operator": source,
            "successor_operator": successor,
            "symbolic_contract_compatible": True,
            "candidate_library": str(library_path.resolve()),
            "candidate_total": len(candidates),
            "candidate_feasible": len(feasible),
            "hard_rejection_counts": dict(sorted(reasons.items())),
            "selected_handoff_ids": selected_ids,
            "shadow_reports": shadows,
            "passing_shadow_handoff_ids": passing_ids,
            "representative_handoff_id": representative_id,
            "classification": level,
            "physical_validation_performed": False,
        }
        records.append(record)

        if representative_id is not None:
            manifest_path = manifests_dir / f"{representative_id}.json"
            shadow_path = shadow_by_id[representative_id][0]
            registry_edges.append(
                {
                    "source_operator": source,
                    "successor_operator": successor,
                    "handoff_manifest": _relative(manifest_path, artifact_root),
                    "policy_shadow": _relative(shadow_path, artifact_root),
                    "gripper_event": gripper_event,
                }
            )

    summary = {
        "schema_version": "a0509.t7_t8_level2_edge_validation.v1",
        "scope": "T1--T8 direct symbolic edges involving T7 or T8",
        "semantic_authority": "external_planner",
        "bridge_duration_s": 4.0,
        "raw_b_prefix_acceleration_hard_reject": False,
        "crossfade_command_acceleration_limit_mm_s2": 4000.0,
        "robot_commands_published": 0,
        "live_enabled": False,
        "mux_selected": False,
        "ik_checked": False,
        "collision_checked": False,
        "physical_validation_performed": False,
        "counts": {
            "symbolic_edges": len(records),
            "bridge_feasible_edges": sum(
                int(item["candidate_feasible"] > 0) for item in records
            ),
            "level2_edges": sum(
                int(
                    item["classification"]
                    == "LEVEL_2_RUNTIME_CONTRACT_COMPATIBLE"
                )
                for item in records
            ),
            "level1_edges": sum(
                int(
                    item["classification"]
                    == "LEVEL_1_SYMBOLICALLY_PLANNABLE"
                )
                for item in records
            ),
        },
        "edges": records,
    }
    registry = {
        "schema_version": "a0509.interior_policy_v2_edge_registry.v1",
        "composition_id": "a0509_t1_t8_all_command_free_level2_edges_20260829",
        "selection_contract": {
            "bridge_duration_s": 4.0,
            "semantic_authority": "external_planner",
            "candidate_safety_filter_precedes_diversity": True,
            "fresh_successor_policy_shadow_required": True,
            "raw_b_prefix_acceleration_hard_reject": False,
            "max_crossfade_command_acceleration_mm_s2": 4000.0,
            "physical_validation_performed": False,
            "robot_executable": False,
        },
        "edges": registry_edges,
    }
    (artifact_root / "edge_validation_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (artifact_root / "edge_registry_all_level2.json").write_text(
        json.dumps(registry, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return summary


def main() -> None:
    summary = run(_parse_args().artifact_root)
    print(
        "T7_T8_LEVEL2_SUMMARY_OK "
        f"symbolic={summary['counts']['symbolic_edges']} "
        f"bridge_feasible={summary['counts']['bridge_feasible_edges']} "
        f"level2={summary['counts']['level2_edges']} "
        f"physical={summary['physical_validation_performed']}"
    )


if __name__ == "__main__":
    main()
