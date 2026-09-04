#!/usr/bin/env python3
"""Exhaustive command-free Level-1 to Level-2 edge coverage for T1--T8.

The tool deliberately owns no ROS node, robot connection, MUX authority, or
physical command path.  It operates at offline/episode frequency:

1. enumerate every cross-policy edge admitted by the symbolic contracts,
2. build exact-boundary cubic-Bezier candidate libraries,
3. run frozen successor ACT shadows only for hard-filter-passing candidates,
4. emit a strict registry containing only command-free Level-2 evidence.
"""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Mapping


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))


from lerobot_robot_doosan_a0509.interior_policy.catalog import (
    LoadedOperatorCatalog,
    load_operator_catalog,
)
from lerobot_robot_doosan_a0509.interior_policy.contracts import (
    InteriorPolicyOperator,
    WorldState,
    apply_operator_effects,
)
from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (
    EdgeRuntimeRegistry,
)
from lerobot_robot_doosan_a0509.interior_policy.v2_adapter import (
    SymbolicTransitionValidator,
)
from offline_tools.cross_task_handoff.authority import SemanticAuthority
from offline_tools.cross_task_handoff.enumerate_handoff_candidates import (
    enumerate_candidates,
)
from offline_tools.cross_task_handoff.schema import PhaseIndexPoint
from offline_tools.cross_task_handoff.select_diverse_handoffs import (
    episode_manifest,
    farthest_point_selection,
)
from offline_tools.cross_task_handoff.validate_handoff_candidates import (
    CandidateValidationConfig,
)


DEFAULT_CATALOG = (
    REPOSITORY_ROOT
    / "config/interior_policy/a0509_t1_t6_operator_catalog_v1.json"
)
DEFAULT_PHASE_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t7_t8_level2_edge_validation_2026-08-29/phase_indices"
)
DEFAULT_ARTIFACT_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_full_level2_coverage_2026-08-29"
)
DEFAULT_VALIDATION_CONFIG = (
    REPOSITORY_ROOT
    / "offline_tools/cross_task_handoff/validation_config_a0509_v2.json"
)
DEFAULT_EXISTING_REGISTRY = (
    REPOSITORY_ROOT
    / "docs/artifacts/t7_t8_level2_edge_validation_2026-08-29"
    / "edge_registry_all_level2.json"
)

FACT_DEFAULTS = {
    "drawer": "unknown",
    "white_container": "unknown",
    "blue_block_location": "unknown",
    "holding": "unknown",
    "gripper": "unknown",
    "contact_mode": "unknown",
    "support_blue_block_location": "unknown",
    "stack": "unknown",
}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--stage",
        choices=("inventory", "candidates", "shadows", "summary", "all"),
        required=True,
    )
    parser.add_argument("--repository-root", type=Path, default=REPOSITORY_ROOT)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--phase-root", type=Path, default=DEFAULT_PHASE_ROOT)
    parser.add_argument("--artifact-root", type=Path, default=DEFAULT_ARTIFACT_ROOT)
    parser.add_argument(
        "--validation-config", type=Path, default=DEFAULT_VALIDATION_CONFIG
    )
    parser.add_argument(
        "--existing-registry", type=Path, default=DEFAULT_EXISTING_REGISTRY
    )
    parser.add_argument("--duration-s", type=float, default=4.0)
    parser.add_argument("--selected-count", type=int, default=5)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--edge-id")
    return parser.parse_args()


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _write(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _slug(operator_id: str) -> str:
    return operator_id.lower().replace(".", "_")


def _edge_id(source: str, successor: str) -> str:
    return f"{_slug(source)}__to__{_slug(successor)}"


def _relative(path: Path, base: Path) -> str:
    return os.path.relpath(path.resolve(), start=base.resolve())


def _gripper_event(operator: InteriorPolicyOperator) -> str:
    before = operator.precondition_map.get("gripper")
    after = operator.effect_map.get("gripper", before)
    if before == "open" and after == "closed":
        return "open_then_closed"
    if before == "closed" and after == "open":
        return "closed_then_open"
    if operator.id in {
        "T1.open_white_container",
        "T4.open_drawer",
        "T5.open_drawer",
    }:
        return "closed_then_open"
    return "state_match"


def _witness_before(
    source: InteriorPolicyOperator,
    successor: InteriorPolicyOperator,
) -> tuple[WorldState, dict[str, str]] | None:
    known_post = {**source.precondition_map, **source.effect_map}
    conflicts = {
        field: f"{known_post[field]}!={expected}"
        for field, expected in successor.precondition_map.items()
        if field in known_post and known_post[field] != expected
    }
    if conflicts:
        return None
    context = {
        field: expected
        for field, expected in successor.precondition_map.items()
        if field not in known_post
    }
    before = dict(FACT_DEFAULTS)
    before.update(source.precondition_map)
    before.update(context)
    return WorldState(**before), context


def build_inventory(
    catalog: LoadedOperatorCatalog,
    output: Path,
) -> dict[str, Any]:
    validator = SymbolicTransitionValidator()
    edges: list[dict[str, Any]] = []
    for source in sorted(catalog.operators, key=lambda item: item.id):
        for successor in sorted(catalog.operators, key=lambda item: item.id):
            if source.policy_id == successor.policy_id:
                continue
            witness = _witness_before(source, successor)
            if witness is None:
                continue
            state_before, required_context = witness
            state_after = apply_operator_effects(state_before, source)
            symbolic = validator.validate(source, successor, state_after)
            if not symbolic.valid:
                raise RuntimeError(
                    f"inventory witness rejected for {source.id}->{successor.id}: "
                    + ";".join(symbolic.reasons)
                )
            edges.append(
                {
                    "edge_id": _edge_id(source.id, successor.id),
                    "source_operator": source.id,
                    "source_policy": source.policy_id,
                    "source_segment": source.evidence.exit_segment,
                    "source_phase": source.evidence.exit_phase,
                    "successor_operator": successor.id,
                    "successor_policy": successor.policy_id,
                    "successor_segment": successor.evidence.entry_segment,
                    "successor_phase": successor.evidence.entry_phase,
                    "transition_type": symbolic.transition_type,
                    "required_persistent_context": required_context,
                    "context_independent": not required_context,
                    "witness_state_before": state_before.to_record(),
                    "witness_state_after_source": state_after.to_record(),
                    "gripper_event": _gripper_event(source),
                }
            )
    result = {
        "schema_version": "a0509.interior_policy_level1_edge_inventory.v1",
        "scope": "all direct cross-policy edges admitted by T1--T8 contracts",
        "same_policy_continuations_require_bridge": False,
        "context_contract": (
            "successor-only facts are explicit persistent world context; "
            "conflicting source postconditions are rejected"
        ),
        "operator_count": len(catalog.operators),
        "edge_count": len(edges),
        "context_independent_edge_count": sum(
            int(item["context_independent"]) for item in edges
        ),
        "context_dependent_edge_count": sum(
            int(not item["context_independent"]) for item in edges
        ),
        "robot_commands_published": 0,
        "physical_validation_performed": False,
        "edges": edges,
    }
    _write(output, result)
    print(
        "LEVEL1_EDGE_INVENTORY_OK "
        f"edges={result['edge_count']} "
        f"context_independent={result['context_independent_edge_count']} "
        f"context_dependent={result['context_dependent_edge_count']}",
        flush=True,
    )
    return result


def _phase_points(path: Path) -> list[PhaseIndexPoint]:
    value = _load(path)
    if value.get("schema_version") != "a0509.cross_task_phase_index.v2":
        raise ValueError(f"unsupported phase index: {path}")
    return [PhaseIndexPoint.from_record(item) for item in value["points"]]


def _boundary_points(
    points: list[PhaseIndexPoint],
    *,
    task: str,
    segment: str,
    phase: float,
) -> list[PhaseIndexPoint]:
    selected = [
        point
        for point in points
        if point.task == task
        and point.segment == segment
        and abs(point.phase - phase) <= 1.0e-9
    ]
    if not selected:
        raise ValueError(f"phase index has no {task}/{segment}@{phase:.6f}")
    return sorted(
        selected,
        key=lambda item: (item.support_episode, item.support_frame),
    )


def _selection_summary(
    *,
    edge: Mapping[str, Any],
    feasible_count: int,
    selected: list[Any],
    selection: Mapping[str, Any],
    manifest_paths: list[Path],
) -> dict[str, Any]:
    return {
        "schema_version": "a0509.cross_task_handoff_selection.v2",
        "composition_id": edge["edge_id"],
        "source_operator": edge["source_operator"],
        "successor_operator": edge["successor_operator"],
        "semantic_authority": SemanticAuthority.EXTERNAL_PLANNER.value,
        "semantic_checks_enforced_by_runtime": False,
        "valid_candidate_count": feasible_count,
        "selected_count": len(selected),
        "selected_handoff_ids": [item.handoff_id for item in selected],
        "episode_manifests": [str(path.resolve()) for path in manifest_paths],
        "selection": dict(selection),
    }


def build_candidates(
    *,
    inventory: Mapping[str, Any],
    phase_root: Path,
    artifact_root: Path,
    validation_path: Path,
    duration_s: float,
    selected_count: int,
    edge_filter: str | None,
) -> None:
    if duration_s <= 0.0 or selected_count < 1:
        raise ValueError("duration and selected count must be positive")
    validation = CandidateValidationConfig.from_mapping(_load(validation_path))
    task_ids = sorted(
        {
            str(edge["source_policy"])
            for edge in inventory["edges"]
        }
        | {
            str(edge["successor_policy"])
            for edge in inventory["edges"]
        }
    )
    indices = {
        task: _phase_points(phase_root / f"{task.lower()}.json")
        for task in task_ids
    }
    for index, edge in enumerate(inventory["edges"], start=1):
        edge_id = str(edge["edge_id"])
        if edge_filter is not None and edge_id != edge_filter:
            continue
        edge_root = artifact_root / "edges" / edge_id
        library_path = edge_root / "candidate_library.json"
        selection_path = edge_root / "episode_manifests/selection_summary.json"
        if library_path.is_file() and selection_path.is_file():
            library = _load(library_path)
            feasible_count = sum(
                int(item["feasible"]) for item in library["candidates"]
            )
            print(
                f"CANDIDATE_REUSE {index}/{len(inventory['edges'])} "
                f"edge={edge_id} feasible={feasible_count}",
                flush=True,
            )
            continue
        source_points = _boundary_points(
            indices[str(edge["source_policy"])],
            task=str(edge["source_policy"]),
            segment=str(edge["source_segment"]),
            phase=float(edge["source_phase"]),
        )
        successor_points = _boundary_points(
            indices[str(edge["successor_policy"])],
            task=str(edge["successor_policy"]),
            segment=str(edge["successor_segment"]),
            phase=float(edge["successor_phase"]),
        )
        candidates = enumerate_candidates(
            source_points=source_points,
            successor_points=successor_points,
            durations_s=[duration_s],
            config=validation,
            transport_floor_mm=None,
            max_pairs=None,
            semantic_authority=SemanticAuthority.EXTERNAL_PLANNER,
        )
        library = {
            "schema_version": "a0509.cross_task_handoff_library.v2",
            "composition_id": edge_id,
            "source_operator": edge["source_operator"],
            "successor_operator": edge["successor_operator"],
            "required_persistent_context": edge["required_persistent_context"],
            "generation_method": "corridor_diverse",
            "hard_filter_before_diversity": True,
            "semantic_authority": SemanticAuthority.EXTERNAL_PLANNER.value,
            "semantic_checks_enforced_by_runtime": False,
            "transport_floor_mm": None,
            "ik_checker_available": False,
            "collision_checker_available": False,
            "robot_commands_published": 0,
            "candidates": candidates,
        }
        _write(library_path, library)
        feasible = [
            item
            for item in candidates
            if bool(item["feasible"])
        ]
        if feasible:
            from offline_tools.cross_task_handoff.schema import HandoffCandidate

            candidate_objects = [HandoffCandidate.from_record(item) for item in feasible]
            selected, selection = farthest_point_selection(
                candidate_objects, count=selected_count
            )
            manifest_dir = edge_root / "episode_manifests"
            manifest_paths: list[Path] = []
            for candidate in selected:
                manifest_path = manifest_dir / f"{candidate.handoff_id}.json"
                _write(
                    manifest_path,
                    episode_manifest(
                        edge_id,
                        candidate,
                        support_radius_mm=40.0,
                        support_orientation_radius_deg=8.0,
                        prearm_radius_mm=50.0,
                        commit_radius_mm=25.0,
                        direction_cosine_minimum=0.25,
                    ),
                )
                manifest_paths.append(manifest_path)
            summary = _selection_summary(
                edge=edge,
                feasible_count=len(feasible),
                selected=selected,
                selection=selection,
                manifest_paths=manifest_paths,
            )
        else:
            summary = _selection_summary(
                edge=edge,
                feasible_count=0,
                selected=[],
                selection={
                    "method": "not_run_no_hard_filter_passing_candidates",
                    "safety_role": "none",
                },
                manifest_paths=[],
            )
        _write(selection_path, summary)
        print(
            f"CANDIDATE_EDGE {index}/{len(inventory['edges'])} "
            f"edge={edge_id} total={len(candidates)} feasible={len(feasible)}",
            flush=True,
        )


def _strict_shadow(value: Mapping[str, Any]) -> bool:
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
        and isinstance(admission, Mapping)
        and admission.get("valid") is True
        and isinstance(timing, Mapping)
        and int(timing.get("deadline_miss_count", -1)) == 0
    )


def _discover_shadows(repository_root: Path) -> dict[str, list[tuple[Path, dict[str, Any]]]]:
    result: dict[str, list[tuple[Path, dict[str, Any]]]] = {}
    for path in sorted(repository_root.glob("docs/artifacts/**/policy_shadow*.json")):
        try:
            value = _load(path)
            handoff_id = str(value["handoff_id"])
        except (KeyError, OSError, TypeError, json.JSONDecodeError):
            continue
        result.setdefault(handoff_id, []).append((path.resolve(), value))
    return result


def _existing_registry_map(path: Path) -> dict[tuple[str, str], Any]:
    if not path.is_file():
        return {}
    registry = EdgeRuntimeRegistry.load(path)
    return registry.by_key


def run_shadows(
    *,
    repository_root: Path,
    catalog_value: Mapping[str, Any],
    inventory: Mapping[str, Any],
    artifact_root: Path,
    validation_path: Path,
    existing_registry_path: Path,
    device: str,
    edge_filter: str | None,
) -> None:
    existing_registry = _existing_registry_map(existing_registry_path)
    discovered = _discover_shadows(repository_root)
    task_values = dict(catalog_value["tasks"])
    for index, edge in enumerate(inventory["edges"], start=1):
        edge_id = str(edge["edge_id"])
        if edge_filter is not None and edge_id != edge_filter:
            continue
        edge_root = artifact_root / "edges" / edge_id
        selection_path = edge_root / "episode_manifests/selection_summary.json"
        evaluation_path = edge_root / "shadow_evaluation.json"
        if evaluation_path.is_file():
            value = _load(evaluation_path)
            print(
                f"SHADOW_REUSE {index}/{len(inventory['edges'])} "
                f"edge={edge_id} pass={bool(value['passing_handoff_ids'])}",
                flush=True,
            )
            continue
        key = (str(edge["source_operator"]), str(edge["successor_operator"]))
        if key in existing_registry:
            spec = existing_registry[key]
            shadow_value = _load(spec.policy_shadow_path)
            if not _strict_shadow(shadow_value):
                raise RuntimeError(f"existing registry shadow is invalid: {key}")
            evaluation = {
                "schema_version": "a0509.level2_edge_shadow_evaluation.v1",
                "edge_id": edge_id,
                "reused_existing_level2": True,
                "evaluated_handoff_ids": [shadow_value["handoff_id"]],
                "passing_handoff_ids": [shadow_value["handoff_id"]],
                "shadow_reports": [str(spec.policy_shadow_path)],
                "subprocess_failures": [],
                "completed": True,
                "robot_commands_published": 0,
            }
            _write(evaluation_path, evaluation)
            print(
                f"SHADOW_EXISTING_LEVEL2 {index}/{len(inventory['edges'])} "
                f"edge={edge_id}",
                flush=True,
            )
            continue
        if not selection_path.is_file():
            raise FileNotFoundError(f"selection missing: {selection_path}")
        selection = _load(selection_path)
        manifest_paths = [Path(item) for item in selection["episode_manifests"]]
        evaluated: list[str] = []
        passing: list[str] = []
        report_paths: list[str] = []
        failures: list[dict[str, Any]] = []
        successor_task = str(edge["successor_policy"])
        successor = dict(task_values[successor_task])
        for manifest_path in manifest_paths:
            manifest = _load(manifest_path)
            handoff_id = str(manifest["handoff_id"])
            prior_reports = discovered.get(handoff_id, [])
            if prior_reports:
                report_path, report = prior_reports[0]
                evaluated.append(handoff_id)
                report_paths.append(str(report_path))
                if _strict_shadow(report):
                    passing.append(handoff_id)
                    break
                continue
            output_path = edge_root / f"policy_shadow_{handoff_id}.json"
            command = [
                sys.executable,
                "-m",
                "offline_tools.cross_task_handoff.run_v2_policy_shadow",
                "--episode-manifest",
                str(manifest_path),
                "--checkpoint-b",
                str(successor["checkpoint"]),
                "--dataset-b",
                str(successor["dataset_root"]),
                "--validation-config",
                str(validation_path),
                "--handoff-window-steps",
                "24",
                "--prefix-steps",
                "15",
                "--crossfade-steps",
                "15",
                "--max-result-age-s",
                "0.30",
                "--max-crossfade-command-acceleration-mm-s2",
                "4000",
                "--device",
                device,
                "--output",
                str(output_path),
            ]
            completed = subprocess.run(
                command,
                cwd=repository_root,
                text=True,
                capture_output=True,
                check=False,
            )
            if completed.stdout.strip():
                print(completed.stdout.strip(), flush=True)
            if completed.returncode != 0:
                failures.append(
                    {
                        "handoff_id": handoff_id,
                        "returncode": completed.returncode,
                        "stderr": completed.stderr[-4000:],
                    }
                )
                print(
                    f"SHADOW_SUBPROCESS_FAILED edge={edge_id} "
                    f"handoff={handoff_id} rc={completed.returncode}",
                    flush=True,
                )
                continue
            report = _load(output_path)
            discovered.setdefault(handoff_id, []).append(
                (output_path.resolve(), report)
            )
            evaluated.append(handoff_id)
            report_paths.append(str(output_path.resolve()))
            if _strict_shadow(report):
                passing.append(handoff_id)
                break
        evaluation = {
            "schema_version": "a0509.level2_edge_shadow_evaluation.v1",
            "edge_id": edge_id,
            "reused_existing_level2": False,
            "selected_handoff_count": len(manifest_paths),
            "evaluated_handoff_ids": evaluated,
            "passing_handoff_ids": passing,
            "shadow_reports": report_paths,
            "subprocess_failures": failures,
            "completed": len(evaluated) + len(failures) >= len(manifest_paths)
            or bool(passing),
            "robot_commands_published": 0,
        }
        _write(evaluation_path, evaluation)
        print(
            f"SHADOW_EDGE {index}/{len(inventory['edges'])} edge={edge_id} "
            f"evaluated={len(evaluated)} pass={bool(passing)}",
            flush=True,
        )


def _rejection_counts(candidates: list[Mapping[str, Any]]) -> dict[str, int]:
    counts: Counter[str] = Counter()
    for candidate in candidates:
        if candidate.get("feasible"):
            continue
        reason = candidate.get("rejection_reason")
        if isinstance(reason, list):
            counts.update(str(item) for item in reason)
        elif reason:
            counts[str(reason)] += 1
    return dict(sorted(counts.items()))


def build_summary(
    *,
    repository_root: Path,
    inventory: Mapping[str, Any],
    artifact_root: Path,
    existing_registry_path: Path,
    duration_s: float,
) -> dict[str, Any]:
    existing_registry = _existing_registry_map(existing_registry_path)
    records: list[dict[str, Any]] = []
    registry_records: list[dict[str, Any]] = []
    for edge in inventory["edges"]:
        edge_id = str(edge["edge_id"])
        edge_root = artifact_root / "edges" / edge_id
        library_path = edge_root / "candidate_library.json"
        selection_path = edge_root / "episode_manifests/selection_summary.json"
        evaluation_path = edge_root / "shadow_evaluation.json"
        library = _load(library_path)
        selection = _load(selection_path)
        evaluation = _load(evaluation_path) if evaluation_path.is_file() else None
        candidates = list(library["candidates"])
        feasible_count = sum(int(item["feasible"]) for item in candidates)
        key = (str(edge["source_operator"]), str(edge["successor_operator"]))
        manifest_path: Path | None = None
        shadow_path: Path | None = None
        passing_handoff_id: str | None = None
        if key in existing_registry:
            spec = existing_registry[key]
            manifest_path = spec.handoff_manifest_path
            shadow_path = spec.policy_shadow_path
            passing_handoff_id = str(_load(shadow_path)["handoff_id"])
        elif evaluation and evaluation["passing_handoff_ids"]:
            passing_handoff_id = str(evaluation["passing_handoff_ids"][0])
            manifest_path = (
                edge_root
                / "episode_manifests"
                / f"{passing_handoff_id}.json"
            )
            for item in evaluation["shadow_reports"]:
                candidate_path = Path(item)
                if str(_load(candidate_path).get("handoff_id")) == passing_handoff_id:
                    shadow_path = candidate_path
                    break
        if passing_handoff_id is not None:
            if manifest_path is None or shadow_path is None:
                raise RuntimeError(f"incomplete passing evidence: {edge_id}")
            shadow_value = _load(shadow_path)
            if not _strict_shadow(shadow_value):
                raise RuntimeError(f"non-strict shadow selected: {shadow_path}")
            classification = "LEVEL_2_RUNTIME_CONTRACT_COMPATIBLE"
            registry_records.append(
                {
                    "source_operator": key[0],
                    "successor_operator": key[1],
                    "handoff_manifest": _relative(manifest_path, artifact_root),
                    "policy_shadow": _relative(shadow_path, artifact_root),
                    "gripper_event": edge["gripper_event"],
                }
            )
        elif feasible_count == 0:
            classification = "LEVEL_1_NO_BRIDGE_CANDIDATE"
        elif evaluation and evaluation.get("completed"):
            classification = "BRIDGE_FEASIBLE_SHADOW_FAILED"
        else:
            classification = "BRIDGE_FEASIBLE_UNSHADOWED"
        records.append(
            {
                **dict(edge),
                "candidate_library": str(library_path.resolve()),
                "candidate_total": len(candidates),
                "candidate_feasible": feasible_count,
                "hard_rejection_counts": _rejection_counts(candidates),
                "selected_handoff_ids": list(
                    selection.get("selected_handoff_ids", ())
                ),
                "shadow_evaluation": (
                    None if evaluation is None else str(evaluation_path.resolve())
                ),
                "passing_handoff_id": passing_handoff_id,
                "classification": classification,
                "physical_validation_performed": False,
            }
        )
    counts = Counter(item["classification"] for item in records)
    summary = {
        "schema_version": "a0509.t1_t8_full_level2_coverage.v1",
        "scope": "all direct cross-policy Level-1 edges admitted by catalog",
        "semantic_authority": "external_planner",
        "bridge_duration_s": duration_s,
        "raw_b_prefix_acceleration_hard_reject": False,
        "crossfade_command_acceleration_limit_mm_s2": 4000.0,
        "robot_commands_published": 0,
        "live_enabled": False,
        "mux_selected": False,
        "ik_checked": False,
        "collision_checked": False,
        "physical_validation_performed": False,
        "counts": {
            "level1_direct_edges": len(records),
            "context_independent_edges": inventory[
                "context_independent_edge_count"
            ],
            "context_dependent_edges": inventory[
                "context_dependent_edge_count"
            ],
            "bridge_feasible_edges": sum(
                int(item["candidate_feasible"] > 0) for item in records
            ),
            "level2_edges": counts["LEVEL_2_RUNTIME_CONTRACT_COMPATIBLE"],
            "no_bridge_candidate_edges": counts[
                "LEVEL_1_NO_BRIDGE_CANDIDATE"
            ],
            "shadow_failed_edges": counts[
                "BRIDGE_FEASIBLE_SHADOW_FAILED"
            ],
            "unshadowed_edges": counts["BRIDGE_FEASIBLE_UNSHADOWED"],
        },
        "edges": records,
    }
    registry = {
        "schema_version": "a0509.interior_policy_v2_edge_registry.v1",
        "composition_id": "a0509_t1_t8_full_command_free_level2_20260829",
        "selection_contract": {
            "source_inventory": str(
                (artifact_root / "level1_edge_inventory.json").resolve()
            ),
            "bridge_duration_s": duration_s,
            "semantic_authority": "external_planner",
            "candidate_safety_filter_precedes_diversity": True,
            "fresh_successor_policy_shadow_required": True,
            "raw_b_prefix_acceleration_hard_reject": False,
            "max_crossfade_command_acceleration_mm_s2": 4000.0,
            "physical_validation_performed": False,
            "robot_executable": False,
        },
        "edges": sorted(
            registry_records,
            key=lambda item: (
                item["source_operator"], item["successor_operator"]
            ),
        ),
    }
    _write(artifact_root / "edge_coverage_summary.json", summary)
    _write(artifact_root / "edge_registry_all_level2.json", registry)
    csv_path = artifact_root / "edge_coverage_matrix.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        fieldnames = (
            "edge_id",
            "source_operator",
            "successor_operator",
            "transition_type",
            "context_independent",
            "required_persistent_context",
            "candidate_total",
            "candidate_feasible",
            "classification",
            "passing_handoff_id",
            "gripper_event",
            "physical_validation_performed",
        )
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for item in records:
            writer.writerow(
                {
                    field: (
                        json.dumps(
                            item[field],
                            ensure_ascii=False,
                            sort_keys=True,
                        )
                        if field == "required_persistent_context"
                        else item.get(field)
                    )
                    for field in fieldnames
                }
            )
    print(
        "FULL_LEVEL2_SUMMARY_OK "
        f"level1={summary['counts']['level1_direct_edges']} "
        f"bridge_feasible={summary['counts']['bridge_feasible_edges']} "
        f"level2={summary['counts']['level2_edges']} "
        f"unshadowed={summary['counts']['unshadowed_edges']} "
        f"physical={summary['physical_validation_performed']}",
        flush=True,
    )
    return summary


def main() -> None:
    args = _parse_args()
    repository_root = args.repository_root.expanduser().resolve()
    catalog_path = args.catalog.expanduser().resolve()
    phase_root = args.phase_root.expanduser().resolve()
    artifact_root = args.artifact_root.expanduser().resolve()
    validation_path = args.validation_config.expanduser().resolve()
    existing_registry_path = args.existing_registry.expanduser().resolve()
    catalog = load_operator_catalog(
        repository_root=repository_root,
        config_path=catalog_path,
    )
    catalog_value = _load(catalog_path)
    inventory_path = artifact_root / "level1_edge_inventory.json"
    if args.stage in {"inventory", "all"}:
        inventory = build_inventory(catalog, inventory_path)
    else:
        inventory = _load(inventory_path)
    if args.stage in {"candidates", "all"}:
        build_candidates(
            inventory=inventory,
            phase_root=phase_root,
            artifact_root=artifact_root,
            validation_path=validation_path,
            duration_s=args.duration_s,
            selected_count=args.selected_count,
            edge_filter=args.edge_id,
        )
    if args.stage in {"shadows", "all"}:
        run_shadows(
            repository_root=repository_root,
            catalog_value=catalog_value,
            inventory=inventory,
            artifact_root=artifact_root,
            validation_path=validation_path,
            existing_registry_path=existing_registry_path,
            device=args.device,
            edge_filter=args.edge_id,
        )
    if args.stage in {"summary", "all"}:
        build_summary(
            repository_root=repository_root,
            inventory=inventory,
            artifact_root=artifact_root,
            existing_registry_path=existing_registry_path,
            duration_s=args.duration_s,
        )


if __name__ == "__main__":
    main()
