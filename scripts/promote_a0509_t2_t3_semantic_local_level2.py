#!/usr/bin/env python3
"""Promote the command-free T2 S2 -> T3 S2 evidence into spatial Level-2.

The spatial v4 registry remains read-only. Relative evidence paths inherited
from it are rebased, then the exact semantic-local manifest and its matching
fresh-successor policy shadow are revalidated by the existing overlay logic.
No ROS node or robot command is created by this script.
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


DEFAULT_BASE = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_spatial_floor_level2_2026-09-02/"
    "edge_registry_level2_spatial_v4.json"
)
DEFAULT_OVERLAY = (
    REPOSITORY_ROOT
    / "docs/artifacts/t2_to_t3_s2_runtime_reference_2026-09-03/"
    "level2_registry_overlay.json"
)
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT
    / "docs/artifacts/"
    "t1_t8_spatial_floor_level2_semantic_local_2026-09-03/"
    "edge_registry_level2_spatial_v5.json"
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-registry", type=Path, default=DEFAULT_BASE)
    parser.add_argument("--verified-overlay", type=Path, default=DEFAULT_OVERLAY)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
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
    overlay_path = args.verified_overlay.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
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
    overlay_args = SimpleNamespace(verified_overlay=(overlay_path,))
    applied = _apply_verified_overlays(
        overlay_args,
        records=records,
        registry_edges=edges,
        output_root=output_path.parent,
    )
    expected = ("T2.acquire_from_floor", "T3.deliver_to_floor")
    if len(applied) != 1 or (
        applied[0]["source_operator"], applied[0]["successor_operator"]
    ) != expected:
        raise RuntimeError("T2->T3 semantic-local overlay was not uniquely applied")

    selection_path = overlay_path.parent / "semantic_local_reference_selection.json"
    selection = _load(selection_path)
    contract = dict(value.get("selection_contract", {}))
    prior = list(contract.get("verified_semantic_local_overlays", ()))
    overlay_text = str(overlay_path)
    if overlay_text not in prior:
        prior.append(overlay_text)
    contract.update(
        {
            "base_registry_read_only": str(base_path),
            "verified_semantic_local_overlays": prior,
            "t2_t3_reference_selection": {
                "objective": selection["objective"],
                "selected_rank": selection["selected_candidate_rank"],
                "source_reference_phase_max": selection[
                    "source_reference_phase_max"
                ],
                "geometry_candidates_considered": selection["pairs_considered"],
                "geometry_candidates_hard_passed": selection[
                    "pairs_hard_passed"
                ],
                "runtime_successor_phase_gate": False,
                "dijkstra_operator_cost_changed": False,
                "robot_commands_published": 0,
                "physical_validation_performed": False,
            },
        }
    )
    value.update(
        {
            "composition_id": (
                "a0509_t1_t8_spatial_floor_semantic_local_level2_20260903"
            ),
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
        "T2_T3_SEMANTIC_LOCAL_LEVEL2_PROMOTED "
        f"edges={len(registry.edges)} handoff={applied[0]['handoff_id']} "
        f"output={output_path} robot_commands_published=0"
    )


if __name__ == "__main__":
    main()
