#!/usr/bin/env python3
"""Build the spatial-floor symbolic inventory and migrate reviewed Level-2 evidence.

This command-free migration separates the operator-confirmed physical regions
used by T2 and T3:

* T2 source: ``left_floor``
* T3 destination: ``right_floor``

The raw LeRobot datasets and semantic-only artifacts remain immutable.  Stable
operator IDs and their physical ACT/Bridge evidence are retained.  An old edge
is copied only if it is still admitted by the newly generated symbolic
inventory; consequently the false ``right_floor -> T2 left_floor`` loop is
removed rather than silently reused.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from collections import Counter
from pathlib import Path
import sys
from typing import Any, Mapping


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "src/lerobot_robot_doosan_a0509"
for value in (str(REPOSITORY_ROOT), str(PACKAGE_ROOT)):
    if value not in sys.path:
        sys.path.insert(0, value)

from lerobot_robot_doosan_a0509.interior_policy.catalog import (  # noqa: E402
    load_operator_catalog,
)
from lerobot_robot_doosan_a0509.interior_policy.contracts import (  # noqa: E402
    WorldState,
)
from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (  # noqa: E402
    EdgeRuntimeRegistry,
    compile_plan_to_multi_v2,
    uniform_cost_search_level2,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (  # noqa: E402
    BridgeAdmissionMode,
)
from scripts.validate_a0509_full_level2_coverage import (  # noqa: E402
    build_inventory,
)


DEFAULT_CATALOG = (
    REPOSITORY_ROOT
    / "config/interior_policy/a0509_t1_t8_operator_catalog_spatial_v2.json"
)
DEFAULT_LEGACY_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_flexible_level2_registry_2026-08-30"
)
DEFAULT_OUTPUT_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_spatial_floor_level2_2026-09-02"
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--legacy-root", type=Path, default=DEFAULT_LEGACY_ROOT)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    return parser.parse_args()


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _write(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _key(value: Mapping[str, Any]) -> tuple[str, str]:
    return str(value["source_operator"]), str(value["successor_operator"])


def _relative_evidence_path(
    value: object,
    *,
    legacy_root: Path,
    output_root: Path,
) -> str | None:
    if value is None or not str(value).strip():
        return None
    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = legacy_root / path
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Level-2 evidence is missing: {path}")
    return os.path.relpath(path, start=output_root.resolve())


def _absolute_evidence_path(value: object, *, legacy_root: Path) -> str | None:
    if value is None or not str(value).strip():
        return None
    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = legacy_root / path
    return str(path.resolve())


def _migrate_registry(
    *,
    inventory: Mapping[str, Any],
    legacy_registry: Mapping[str, Any],
    legacy_root: Path,
    output_root: Path,
    inventory_path: Path,
    catalog_path: Path,
) -> tuple[dict[str, Any], list[tuple[str, str]]]:
    current_keys = [_key(item) for item in inventory["edges"]]
    legacy_by_key = {_key(item): dict(item) for item in legacy_registry["edges"]}
    missing = sorted(set(current_keys) - set(legacy_by_key))
    if missing:
        raise RuntimeError(
            "new spatial semantic edges lack reviewed Level-2 evidence: "
            + ", ".join(f"{a}->{b}" for a, b in missing)
        )
    removed = sorted(set(legacy_by_key) - set(current_keys))
    migrated: list[dict[str, Any]] = []
    for key in current_keys:
        record = dict(legacy_by_key[key])
        record["handoff_manifest"] = _relative_evidence_path(
            record.get("handoff_manifest"),
            legacy_root=legacy_root,
            output_root=output_root,
        )
        record["policy_shadow"] = _relative_evidence_path(
            record.get("policy_shadow"),
            legacy_root=legacy_root,
            output_root=output_root,
        )
        migrated.append(record)

    selection = dict(legacy_registry.get("selection_contract", {}))
    selection.update(
        {
            "source_inventory": str(inventory_path.resolve()),
            "source_catalog": str(catalog_path.resolve()),
            "legacy_level2_evidence_registry": str(
                (legacy_root / "edge_registry_level2_v3.json").resolve()
            ),
            "spatial_semantics": {
                "T2_source": "left_floor",
                "T3_destination": "right_floor",
                "legacy_floor": "accepted_by_parser_but_not_used_by_current_operators",
                "operator_ids_preserved": True,
                "raw_semantic_artifacts_modified": False,
            },
            "removed_symbolically_incompatible_edges": [
                {"source_operator": source, "successor_operator": successor}
                for source, successor in removed
            ],
        }
    )
    return (
        {
            "schema_version": "a0509.interior_policy_v2_edge_registry.v3",
            "composition_id": "a0509_t1_t8_spatial_floor_level2_20260902",
            "selection_contract": selection,
            "edges": migrated,
        },
        removed,
    )


def _migrate_summary(
    *,
    inventory: Mapping[str, Any],
    legacy_summary: Mapping[str, Any],
    legacy_root: Path,
    removed: list[tuple[str, str]],
) -> dict[str, Any]:
    legacy_by_key = {_key(item): dict(item) for item in legacy_summary["edges"]}
    records: list[dict[str, Any]] = []
    for symbolic in inventory["edges"]:
        key = _key(symbolic)
        if key not in legacy_by_key:
            raise RuntimeError(f"Level-2 summary lacks retained edge: {key}")
        record = dict(legacy_by_key[key])
        for field in (
            "edge_id",
            "source_operator",
            "source_policy",
            "source_segment",
            "source_phase",
            "successor_operator",
            "successor_policy",
            "successor_segment",
            "successor_phase",
            "transition_type",
            "required_persistent_context",
            "context_independent",
            "witness_state_before",
            "witness_state_after_source",
            "gripper_event",
        ):
            record[field] = symbolic[field]
        for path_field in (
            "flexible_manifest",
            "flexible_shadow",
            "geometry_evaluation",
        ):
            record[path_field] = _absolute_evidence_path(
                record.get(path_field), legacy_root=legacy_root
            )
        records.append(record)

    status = Counter(str(item["admission_status"]) for item in records)
    counts = {
        "level1_direct_edges": len(records),
        "command_safe_edges": sum(bool(item["command_safe"]) for item in records),
        "flexible_verified_edges": status["flexible_verified"],
        "flexible_semantic_candidate_edges": status[
            "flexible_semantic_candidate"
        ],
        "temporarily_unavailable_edges": status["temporarily_unavailable"],
    }
    result = dict(legacy_summary)
    result.update(
        {
            "counts": counts,
            "edges": records,
            "inventory": "level1_edge_inventory.json",
            "spatial_semantics": {
                "T2_source": "left_floor",
                "T3_destination": "right_floor",
                "removed_symbolically_incompatible_edges": [
                    {"source_operator": source, "successor_operator": successor}
                    for source, successor in removed
                ],
                "physical_bridge_evidence_reused": True,
                "physical_validation_performed": False,
            },
        }
    )
    return result


def _write_matrix(path: Path, records: list[Mapping[str, Any]]) -> None:
    fields = (
        "edge_id",
        "source_operator",
        "successor_operator",
        "transition_type",
        "context_independent",
        "admission_status",
        "command_safe",
        "fresh_successor_shadow_pass",
        "validation_method",
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for item in records:
            writer.writerow({field: item.get(field) for field in fields})


def _verify_floor_transfer(
    *,
    catalog,
    registry: EdgeRuntimeRegistry,
) -> dict[str, Any]:
    initial = WorldState(
        drawer="closed",
        white_container="closed",
        blue_block_location="left_floor",
        holding="none",
        gripper="open",
        contact_mode="free_space",
    )
    goal = {
        "blue_block_location": "right_floor",
        "holding": "none",
        "gripper": "open",
        "contact_mode": "free_space",
    }
    operators = (
        catalog.by_id["T2.acquire_from_floor"],
        catalog.by_id["T3.deliver_to_floor"],
    )
    search = uniform_cost_search_level2(
        initial,
        goal,
        operators,
        registry,
        cost_config=catalog.target.cost_config,
        bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
        allow_intermediate_release=False,
    )
    plan = search.best_plan
    if plan is None:
        raise RuntimeError("left_floor -> right_floor has no Level-2 plan")
    expected = ("T2.acquire_from_floor", "T3.deliver_to_floor")
    if plan.operator_ids != expected:
        raise RuntimeError(f"unexpected floor-transfer plan: {plan.operator_ids}")
    compiled = compile_plan_to_multi_v2(
        plan,
        catalog,
        registry,
        composition_id="spatial_left_floor_to_right_floor_command_free",
    )
    if tuple(spec.key for spec in compiled.edge_specs) != ((expected[0], expected[1]),):
        raise RuntimeError("compiled plan did not retain the T2->T3 Level-2 edge")
    return {
        "classification": "LEVEL_2_RUNTIME_CONTRACT_COMPATIBLE",
        "initial_blue_block_location": "left_floor",
        "goal_blue_block_location": "right_floor",
        "operators": list(plan.operator_ids),
        "policy_sequence": list(plan.policy_sequence),
        "total_cost": plan.total_cost,
        "runtime_compiler_passed": True,
        "robot_commands_published": 0,
        "physical_validation_performed": False,
    }


def main() -> None:
    args = _parse_args()
    catalog_path = args.catalog.expanduser().resolve()
    legacy_root = args.legacy_root.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    inventory_path = output_root / "level1_edge_inventory.json"
    registry_path = output_root / "edge_registry_level2_spatial_v4.json"
    summary_path = output_root / "edge_refresh_summary.json"

    catalog = load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        config_path=catalog_path,
        audit_raw_frames=False,
    )
    inventory = build_inventory(catalog, inventory_path)
    legacy_registry = _load(legacy_root / "edge_registry_level2_v3.json")
    legacy_summary = _load(legacy_root / "edge_refresh_summary.json")
    registry_value, removed = _migrate_registry(
        inventory=inventory,
        legacy_registry=legacy_registry,
        legacy_root=legacy_root,
        output_root=output_root,
        inventory_path=inventory_path,
        catalog_path=catalog_path,
    )
    _write(registry_path, registry_value)
    summary = _migrate_summary(
        inventory=inventory,
        legacy_summary=legacy_summary,
        legacy_root=legacy_root,
        removed=removed,
    )
    _write(summary_path, summary)
    _write_matrix(output_root / "edge_refresh_matrix.csv", summary["edges"])

    registry = EdgeRuntimeRegistry.load(registry_path)
    verification = _verify_floor_transfer(catalog=catalog, registry=registry)
    report = {
        "schema_version": "a0509.spatial_floor_level2_migration.v1",
        "catalog": str(catalog_path),
        "catalog_id": catalog.catalog_id,
        "legacy_registry": str((legacy_root / "edge_registry_level2_v3.json")),
        "new_inventory": str(inventory_path),
        "new_registry": str(registry_path),
        "legacy_edge_count": len(legacy_registry["edges"]),
        "spatial_edge_count": len(registry.edges),
        "removed_edges": [
            {"source_operator": source, "successor_operator": successor}
            for source, successor in removed
        ],
        "status_counts": dict(
            Counter(item.admission_status for item in registry.edges)
        ),
        "floor_transfer_verification": verification,
        "raw_datasets_modified": False,
        "semantic_artifacts_modified": False,
        "robot_commands_published": 0,
        "physical_validation_performed": False,
    }
    _write(output_root / "migration_report.json", report)
    print(
        "SPATIAL_FLOOR_LEVEL2_OK "
        f"edges={len(registry.edges)} removed={len(removed)} "
        f"plan={'->'.join(verification['policy_sequence'])}"
    )


if __name__ == "__main__":
    main()
