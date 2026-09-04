#!/usr/bin/env python3
"""Command-free recovery probe for curvature-only T1--T8 Level-2 gaps.

The baseline fixed-cubic artifacts and registry remain immutable.  This tool
only revisits edges that had zero hard-filter-passing candidates, applies the
explicit opt-in tangent-regularized cubic, and emits separate evidence plus an
extended registry after a strict successor-policy shadow passes.
"""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
import os
from pathlib import Path
import sys
from typing import Any, Mapping


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))


from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (
    EdgeRuntimeRegistry,
)
from offline_tools.cross_task_handoff.authority import SemanticAuthority
from offline_tools.cross_task_handoff.schema import (
    HandoffCandidate,
    PhaseIndexPoint,
)
from offline_tools.cross_task_handoff.select_diverse_handoffs import (
    episode_manifest,
    farthest_point_selection,
)
from offline_tools.cross_task_handoff.validate_handoff_candidates import (
    CandidateValidationConfig,
    validate_candidate,
)
from offline_tools.task_c_bridge_v0.bezier_bridge import (
    CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
)
from scripts.validate_a0509_full_level2_coverage import (
    _strict_shadow,
    run_shadows,
)


DEFAULT_BASELINE_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_full_level2_coverage_2026-08-29"
)
DEFAULT_OUTPUT_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_borderline_curvature_recovery_2026-08-29"
)
DEFAULT_CATALOG = (
    REPOSITORY_ROOT
    / "config/interior_policy/a0509_t1_t6_operator_catalog_v1.json"
)
DEFAULT_VALIDATION_CONFIG = (
    REPOSITORY_ROOT
    / "offline_tools/cross_task_handoff/validation_config_a0509_v2.json"
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--stage",
        choices=("candidates", "shadows", "summary", "all"),
        required=True,
    )
    parser.add_argument("--repository-root", type=Path, default=REPOSITORY_ROOT)
    parser.add_argument("--baseline-root", type=Path, default=DEFAULT_BASELINE_ROOT)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument(
        "--validation-config",
        type=Path,
        default=DEFAULT_VALIDATION_CONFIG,
    )
    parser.add_argument("--minimum-handle-chord-ratio", type=float, default=0.04)
    parser.add_argument(
        "--maximum-endpoint-speed-adjustment-mm-s",
        type=float,
        default=12.0,
    )
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


def _relative(path: Path, base: Path) -> str:
    return os.path.relpath(path.resolve(), start=base.resolve())


def _target_edges(
    baseline_root: Path,
    edge_filter: str | None,
) -> list[dict[str, Any]]:
    summary = _load(baseline_root / "edge_coverage_summary.json")
    edges = [
        dict(item)
        for item in summary["edges"]
        if int(item["candidate_feasible"]) == 0
        and (edge_filter is None or item["edge_id"] == edge_filter)
    ]
    if edge_filter is not None and not edges:
        raise ValueError(
            f"baseline has no zero-candidate edge named {edge_filter}"
        )
    return edges


def _inventory(edges: list[Mapping[str, Any]]) -> dict[str, Any]:
    return {
        "schema_version": "a0509.borderline_curvature_recovery_inventory.v1",
        "scope": "baseline Level-1 edges with zero fixed-cubic candidates",
        "edge_count": len(edges),
        "robot_commands_published": 0,
        "physical_validation_performed": False,
        "edges": [dict(item) for item in edges],
    }


def build_candidates(
    *,
    edges: list[Mapping[str, Any]],
    output_root: Path,
    validation_path: Path,
    ratio: float,
    maximum_adjustment_mm_s: float,
    selected_count: int,
) -> None:
    if not 0.0 < ratio <= 0.25:
        raise ValueError("minimum handle ratio must be in (0, 0.25]")
    if maximum_adjustment_mm_s <= 0.0 or selected_count < 1:
        raise ValueError("speed adjustment and selected count must be positive")
    validation = CandidateValidationConfig.from_mapping(_load(validation_path))
    for index, edge in enumerate(edges, start=1):
        edge_id = str(edge["edge_id"])
        edge_root = output_root / "edges" / edge_id
        library_path = edge_root / "candidate_library.json"
        selection_path = edge_root / "episode_manifests/selection_summary.json"
        if library_path.is_file() and selection_path.is_file():
            library = _load(library_path)
            feasible_count = sum(
                int(item["feasible"]) for item in library["candidates"]
            )
            print(
                f"RECOVERY_CANDIDATE_REUSE {index}/{len(edges)} "
                f"edge={edge_id} feasible={feasible_count}",
                flush=True,
            )
            continue

        baseline_library_path = Path(str(edge["candidate_library"]))
        baseline_library = _load(baseline_library_path)
        candidates: list[dict[str, Any]] = []
        for value in baseline_library["candidates"]:
            candidate = validate_candidate(
                PhaseIndexPoint.from_record(value["source"]),
                PhaseIndexPoint.from_record(value["successor"]),
                duration_s=float(value["bridge_duration_s"]),
                config=validation,
                transport_floor_mm=value.get("transport_floor_mm"),
                semantic_authority=SemanticAuthority.EXTERNAL_PLANNER,
                bridge_algorithm=CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
                minimum_tangent_handle_chord_ratio=ratio,
                maximum_endpoint_speed_adjustment_mm_s=(
                    maximum_adjustment_mm_s
                ),
            )
            candidates.append(candidate.to_record())

        library = {
            "schema_version": "a0509.cross_task_handoff_library.v2",
            "composition_id": edge_id,
            "source_operator": edge["source_operator"],
            "successor_operator": edge["successor_operator"],
            "required_persistent_context": edge[
                "required_persistent_context"
            ],
            "baseline_candidate_library": str(
                baseline_library_path.resolve()
            ),
            "baseline_fixed_feasible_count": 0,
            "generation_method": "borderline_curvature_recovery_v1",
            "bridge_algorithm": CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
            "minimum_tangent_handle_chord_ratio": ratio,
            "maximum_endpoint_speed_adjustment_mm_s": (
                maximum_adjustment_mm_s
            ),
            "hard_filter_before_diversity": True,
            "curvature_limit_per_mm_unchanged": (
                validation.curvature_limit_per_mm
            ),
            "semantic_authority": SemanticAuthority.EXTERNAL_PLANNER.value,
            "semantic_checks_enforced_by_runtime": False,
            "transport_floor_mm": baseline_library.get(
                "transport_floor_mm"
            ),
            "ik_checker_available": False,
            "collision_checker_available": False,
            "robot_commands_published": 0,
            "physical_validation_performed": False,
            "candidates": candidates,
        }
        _write(library_path, library)

        feasible = [
            HandoffCandidate.from_record(item)
            for item in candidates
            if bool(item["feasible"])
        ]
        selected: list[HandoffCandidate] = []
        manifest_paths: list[Path] = []
        if feasible:
            selected, selection = farthest_point_selection(
                feasible,
                count=selected_count,
            )
            for candidate in selected:
                manifest_path = (
                    edge_root
                    / "episode_manifests"
                    / f"{candidate.handoff_id}.json"
                )
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
        else:
            selection = {
                "method": "not_run_no_hard_filter_passing_candidates",
                "safety_role": "none",
            }
        _write(
            selection_path,
            {
                "schema_version": "a0509.cross_task_handoff_selection.v2",
                "composition_id": edge_id,
                "source_operator": edge["source_operator"],
                "successor_operator": edge["successor_operator"],
                "semantic_authority": (
                    SemanticAuthority.EXTERNAL_PLANNER.value
                ),
                "semantic_checks_enforced_by_runtime": False,
                "valid_candidate_count": len(feasible),
                "selected_count": len(selected),
                "selected_handoff_ids": [
                    item.handoff_id for item in selected
                ],
                "episode_manifests": [
                    str(path.resolve()) for path in manifest_paths
                ],
                "selection": dict(selection),
            },
        )
        print(
            f"RECOVERY_CANDIDATE_EDGE {index}/{len(edges)} "
            f"edge={edge_id} total={len(candidates)} "
            f"feasible={len(feasible)}",
            flush=True,
        )


def _rejection_counts(candidates: list[Mapping[str, Any]]) -> dict[str, int]:
    counts: Counter[str] = Counter()
    for candidate in candidates:
        if candidate.get("feasible"):
            continue
        reason = candidate.get("rejection_reason", ())
        if isinstance(reason, list):
            counts.update(str(item) for item in reason)
        elif reason:
            counts[str(reason)] += 1
    return dict(sorted(counts.items()))


def build_summary(
    *,
    edges: list[Mapping[str, Any]],
    baseline_root: Path,
    output_root: Path,
    ratio: float,
    maximum_adjustment_mm_s: float,
) -> dict[str, Any]:
    records: list[dict[str, Any]] = []
    recovered_registry_records: list[dict[str, Any]] = []
    for edge in edges:
        edge_id = str(edge["edge_id"])
        edge_root = output_root / "edges" / edge_id
        library_path = edge_root / "candidate_library.json"
        selection_path = edge_root / "episode_manifests/selection_summary.json"
        evaluation_path = edge_root / "shadow_evaluation.json"
        library = _load(library_path)
        selection = _load(selection_path)
        evaluation = _load(evaluation_path) if evaluation_path.is_file() else None
        candidates = list(library["candidates"])
        feasible_count = sum(int(item["feasible"]) for item in candidates)
        passing_handoff_id = None
        manifest_path = None
        shadow_path = None
        if evaluation and evaluation.get("passing_handoff_ids"):
            passing_handoff_id = str(evaluation["passing_handoff_ids"][0])
            manifest_path = (
                edge_root
                / "episode_manifests"
                / f"{passing_handoff_id}.json"
            )
            for item in evaluation.get("shadow_reports", ()):
                candidate_path = Path(item)
                if str(_load(candidate_path).get("handoff_id")) == passing_handoff_id:
                    shadow_path = candidate_path
                    break
        if passing_handoff_id is not None:
            if manifest_path is None or shadow_path is None:
                raise RuntimeError(f"incomplete recovery evidence: {edge_id}")
            if not _strict_shadow(_load(shadow_path)):
                raise RuntimeError(f"non-strict recovery shadow: {shadow_path}")
            classification = "LEVEL_2_RUNTIME_CONTRACT_COMPATIBLE"
            recovered_registry_records.append(
                {
                    "source_operator": edge["source_operator"],
                    "successor_operator": edge["successor_operator"],
                    "handoff_manifest": str(manifest_path.resolve()),
                    "policy_shadow": str(shadow_path.resolve()),
                    "gripper_event": edge["gripper_event"],
                }
            )
        elif feasible_count == 0:
            classification = "NOT_RECOVERED_BY_TANGENT_REGULARIZATION"
        elif evaluation and evaluation.get("completed"):
            classification = "RECOVERED_BRIDGE_SHADOW_FAILED"
        else:
            classification = "RECOVERED_BRIDGE_UNSHADOWED"
        feasible_adjustments = [
            max(
                float(item["source_endpoint_speed_adjustment_mm_s"]),
                float(item["successor_endpoint_speed_adjustment_mm_s"]),
            )
            for item in candidates
            if item["feasible"]
        ]
        records.append(
            {
                "edge_id": edge_id,
                "source_operator": edge["source_operator"],
                "successor_operator": edge["successor_operator"],
                "transition_type": edge["transition_type"],
                "context_independent": edge["context_independent"],
                "required_persistent_context": edge[
                    "required_persistent_context"
                ],
                "baseline_candidate_feasible": 0,
                "regularized_candidate_total": len(candidates),
                "regularized_candidate_feasible": feasible_count,
                "maximum_feasible_endpoint_speed_adjustment_mm_s": (
                    None
                    if not feasible_adjustments
                    else max(feasible_adjustments)
                ),
                "hard_rejection_counts": _rejection_counts(candidates),
                "selected_handoff_ids": selection.get(
                    "selected_handoff_ids", []
                ),
                "passing_handoff_id": passing_handoff_id,
                "classification": classification,
                "candidate_library": str(library_path.resolve()),
                "shadow_evaluation": (
                    None
                    if evaluation is None
                    else str(evaluation_path.resolve())
                ),
                "physical_validation_performed": False,
            }
        )

    baseline_registry_path = baseline_root / "edge_registry_all_level2.json"
    baseline_registry = EdgeRuntimeRegistry.load(baseline_registry_path)
    baseline_records = [item.to_record() for item in baseline_registry.edges]
    all_records = baseline_records + recovered_registry_records
    keys = [
        (item["source_operator"], item["successor_operator"])
        for item in all_records
    ]
    if len(keys) != len(set(keys)):
        raise RuntimeError("extended registry contains duplicate edges")

    counts = Counter(item["classification"] for item in records)
    summary = {
        "schema_version": "a0509.borderline_curvature_recovery.v1",
        "scope": "44 baseline zero-candidate Level-1 edges only",
        "baseline_artifact_root": str(baseline_root.resolve()),
        "baseline_level2_edges": len(baseline_records),
        "bridge_algorithm": CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
        "minimum_tangent_handle_chord_ratio": ratio,
        "maximum_endpoint_speed_adjustment_mm_s": maximum_adjustment_mm_s,
        "curvature_limit_per_mm_unchanged": 0.25,
        "candidate_safety_filter_precedes_diversity": True,
        "robot_commands_published": 0,
        "live_enabled": False,
        "mux_selected": False,
        "ik_checked": False,
        "collision_checked": False,
        "physical_validation_performed": False,
        "counts": {
            "baseline_zero_candidate_edges": len(records),
            "bridge_recovered_edges": sum(
                int(item["regularized_candidate_feasible"] > 0)
                for item in records
            ),
            "level2_recovered_edges": counts[
                "LEVEL_2_RUNTIME_CONTRACT_COMPATIBLE"
            ],
            "recovered_bridge_shadow_failed_edges": counts[
                "RECOVERED_BRIDGE_SHADOW_FAILED"
            ],
            "recovered_bridge_unshadowed_edges": counts[
                "RECOVERED_BRIDGE_UNSHADOWED"
            ],
            "not_recovered_edges": counts[
                "NOT_RECOVERED_BY_TANGENT_REGULARIZATION"
            ],
            "extended_level2_edges": len(all_records),
        },
        "edges": records,
    }
    recovered_registry = {
        "schema_version": "a0509.interior_policy_v2_edge_registry.v1",
        "composition_id": "a0509_t1_t8_tangent_recovery_only_20260829",
        "selection_contract": {
            "baseline_registry": str(baseline_registry_path.resolve()),
            "bridge_algorithm": CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
            "minimum_tangent_handle_chord_ratio": ratio,
            "maximum_endpoint_speed_adjustment_mm_s": maximum_adjustment_mm_s,
            "curvature_limit_per_mm_unchanged": 0.25,
            "fresh_successor_policy_shadow_required": True,
            "physical_validation_performed": False,
            "robot_executable": False,
        },
        "edges": sorted(
            recovered_registry_records,
            key=lambda item: (
                item["source_operator"], item["successor_operator"]
            ),
        ),
    }
    extended_registry = {
        "schema_version": "a0509.interior_policy_v2_edge_registry.v1",
        "composition_id": "a0509_t1_t8_level2_plus_tangent_recovery_20260829",
        "selection_contract": {
            "baseline_registry": str(baseline_registry_path.resolve()),
            "recovery_summary": str(
                (output_root / "recovery_summary.json").resolve()
            ),
            "fixed_baseline_preserved": True,
            "regularization_is_opt_in_per_manifest": True,
            "fresh_successor_policy_shadow_required": True,
            "physical_validation_performed": False,
            "robot_executable": False,
        },
        "edges": sorted(
            all_records,
            key=lambda item: (
                item["source_operator"], item["successor_operator"]
            ),
        ),
    }
    _write(output_root / "recovery_summary.json", summary)
    if recovered_registry_records:
        _write(
            output_root / "edge_registry_recovered_level2.json",
            recovered_registry,
        )
    _write(
        output_root / "edge_registry_all_level2_extended.json",
        extended_registry,
    )
    csv_path = output_root / "recovery_matrix.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        fieldnames = (
            "edge_id",
            "source_operator",
            "successor_operator",
            "regularized_candidate_total",
            "regularized_candidate_feasible",
            "maximum_feasible_endpoint_speed_adjustment_mm_s",
            "classification",
            "passing_handoff_id",
            "physical_validation_performed",
        )
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for item in records:
            writer.writerow({field: item[field] for field in fieldnames})
    print(
        "BORDERLINE_RECOVERY_SUMMARY_OK "
        f"bridge_recovered={summary['counts']['bridge_recovered_edges']} "
        f"level2_recovered={summary['counts']['level2_recovered_edges']} "
        f"extended_level2={summary['counts']['extended_level2_edges']}",
        flush=True,
    )
    return summary


def main() -> None:
    args = _parse_args()
    repository_root = args.repository_root.expanduser().resolve()
    baseline_root = args.baseline_root.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()
    edges = _target_edges(baseline_root, args.edge_id)
    inventory = _inventory(edges)
    _write(output_root / "recovery_inventory.json", inventory)
    if args.stage in {"candidates", "all"}:
        build_candidates(
            edges=edges,
            output_root=output_root,
            validation_path=args.validation_config.expanduser().resolve(),
            ratio=args.minimum_handle_chord_ratio,
            maximum_adjustment_mm_s=(
                args.maximum_endpoint_speed_adjustment_mm_s
            ),
            selected_count=args.selected_count,
        )
    if args.stage in {"shadows", "all"}:
        run_shadows(
            repository_root=repository_root,
            catalog_value=_load(args.catalog.expanduser().resolve()),
            inventory=inventory,
            artifact_root=output_root,
            validation_path=args.validation_config.expanduser().resolve(),
            existing_registry_path=(
                baseline_root / "edge_registry_all_level2.json"
            ),
            device=args.device,
            edge_filter=args.edge_id,
        )
    if args.stage in {"summary", "all"}:
        build_summary(
            edges=edges,
            baseline_root=baseline_root,
            output_root=output_root,
            ratio=args.minimum_handle_chord_ratio,
            maximum_adjustment_mm_s=(
                args.maximum_endpoint_speed_adjustment_mm_s
            ),
        )


if __name__ == "__main__":
    main()
