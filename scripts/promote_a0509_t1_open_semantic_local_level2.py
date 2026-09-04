#!/usr/bin/env python3
"""Promote exact-O1 T1 semantic-local edges into spatial Level-2 v7.

The v6 registry remains read-only. Only T1.open_white_container -> T2/T7
acquisition edges are replaced, after validating their command-free geometry
manifests and matching recorded-observation policy shadows.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "src/lerobot_robot_doosan_a0509"
for value in (str(REPOSITORY_ROOT), str(PACKAGE_ROOT)):
    if value not in sys.path:
        sys.path.insert(0, value)

from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (  # noqa: E402
    EdgeRuntimeRegistry,
)
from scripts.refresh_a0509_flexible_level2_registry import (  # noqa: E402
    _apply_verified_overlays,
)


ARTIFACT_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/"
    "t1_t8_spatial_floor_level2_semantic_local_all_2026-09-03"
)
DEFAULT_BASE = ARTIFACT_ROOT / "edge_registry_level2_spatial_v6.json"
DEFAULT_OVERLAYS = (
    ARTIFACT_ROOT
    / "edges/t1_open_white_container__to__t2_acquire_from_floor/"
    "level2_registry_overlay.json",
    ARTIFACT_ROOT
    / "edges/t1_open_white_container__to__t7_acquire_from_drawer_top/"
    "level2_registry_overlay.json",
)
DEFAULT_OUTPUT = ARTIFACT_ROOT / "edge_registry_level2_spatial_v7.json"
DEFAULT_COMPOSITION_ID = (
    "a0509_t1_t8_spatial_floor_semantic_local_level2_v7_20260903"
)
EXPECTED_EDGES = {
    ("T1.open_white_container", "T2.acquire_from_floor"),
    ("T1.open_white_container", "T7.acquire_from_drawer_top"),
}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-registry", type=Path, default=DEFAULT_BASE)
    parser.add_argument(
        "--verified-overlay",
        type=Path,
        action="append",
        default=None,
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--composition-id",
        default=DEFAULT_COMPOSITION_ID,
    )
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.resolve().read_text(encoding="utf-8"))


def _rebase_evidence_path(
    value: object,
    *,
    old_root: Path,
    new_root: Path,
) -> str | None:
    if value is None or not str(value).strip():
        return None
    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = old_root / path
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"inherited Level-2 evidence is missing: {path}")
    return os.path.relpath(path, start=new_root.resolve())


def main() -> None:
    args = _parse_args()
    base_path = args.base_registry.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    overlay_paths = tuple(
        path.expanduser().resolve()
        for path in (args.verified_overlay or DEFAULT_OVERLAYS)
    )
    if output_path.exists() and not args.force:
        raise FileExistsError(f"refusing to overwrite registry: {output_path}")

    value = _load(base_path)
    edges = [dict(item) for item in value["edges"]]
    for edge in edges:
        for field in ("handoff_manifest", "policy_shadow"):
            edge[field] = _rebase_evidence_path(
                edge.get(field),
                old_root=base_path.parent,
                new_root=output_path.parent,
            )
    records = [
        {
            "source_operator": item["source_operator"],
            "successor_operator": item["successor_operator"],
        }
        for item in edges
    ]
    applied = _apply_verified_overlays(
        SimpleNamespace(verified_overlay=overlay_paths),
        records=records,
        registry_edges=edges,
        output_root=output_path.parent,
    )
    actual = {
        (item["source_operator"], item["successor_operator"])
        for item in applied
    }
    if actual != EXPECTED_EDGES or len(applied) != len(EXPECTED_EDGES):
        raise RuntimeError(
            f"T1 exact-exit overlays differ: expected={EXPECTED_EDGES} "
            f"actual={actual}"
        )
    for edge in edges:
        key = (edge["source_operator"], edge["successor_operator"])
        if key not in EXPECTED_EDGES:
            continue
        evidence = dict(edge.get("evidence", {}))
        evidence.pop("semantic_local_not_applicable_reason", None)
        evidence["semantic_local_refresh"] = "promoted_exact_semantic_exit"
        edge["evidence"] = evidence

    selections = []
    for overlay_path in overlay_paths:
        selection = _load(
            overlay_path.parent / "semantic_local_reference_selection.json"
        )
        selected = dict(selection["selected"])
        applied_edge = next(
            item for item in applied if item["overlay"] == str(overlay_path)
        )
        selections.append(
            {
                "source_operator": applied_edge["source_operator"],
                "successor_operator": applied_edge["successor_operator"],
                "source_phase": selected["source"]["phase"],
                "successor_phase": selected["successor"]["phase"],
                "selected_rank": selection["selected_candidate_rank"],
                "total_length_mm": selected["total_length_mm"],
                "runtime_successor_phase_gate": False,
            }
        )

    contract = dict(value.get("selection_contract", {}))
    prior = list(contract.get("verified_semantic_local_overlays", ()))
    for overlay_path in overlay_paths:
        overlay_text = str(overlay_path)
        if overlay_text not in prior:
            prior.append(overlay_text)
    semantic_count = sum(
        "semantic_local" in str(edge.get("validation_method", ""))
        for edge in edges
    )
    contract.update(
        {
            "base_registry_read_only": str(base_path),
            "verified_semantic_local_overlays": prior,
            "semantic_local_scope": (
                "held-object transport collars plus exact-O1 T1 open exits"
            ),
            "eligible_edge_count": semantic_count,
            "exact_semantic_exit_edge_count": len(EXPECTED_EDGES),
            "t1_open_exact_exit_selection": selections,
            "mandatory_source_event": "T1.S2.O1.closed_then_open",
            "source_event_rearm": "geometry_prearm_then_closed_then_open",
            "runtime_successor_phase_gate": False,
            "dijkstra_operator_cost_changed": False,
            "robot_commands_published": 0,
            "physical_validation_performed": False,
        }
    )
    value.update(
        {
            "composition_id": str(args.composition_id),
            "selection_contract": contract,
            "edges": edges,
        }
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    registry = EdgeRuntimeRegistry.load(output_path)
    print(
        "T1_OPEN_SEMANTIC_LOCAL_LEVEL2_V7_PROMOTED "
        f"edges={len(registry.edges)} semantic_local={semantic_count} "
        f"overlays={len(applied)} output={output_path} "
        "robot_commands_published=0"
    )


if __name__ == "__main__":
    main()
