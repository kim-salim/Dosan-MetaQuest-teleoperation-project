#!/usr/bin/env python3
"""Apply semantic-local Bridge selection to every compatible Level-2 edge.

The symbolic operator cost remains unchanged.  For a cross-policy transition
whose source operator has a data-derived held-object/free-transport execution
tail, this command rebuilds its Bridge reference by minimizing

    source semantic prefix + Bridge + successor semantic suffix.

The command is deliberately command-free: it creates no ROS node, never
selects the MUX, never enables Live, and never publishes robot or gripper
commands.  A rebuilt edge replaces the baseline registry entry only after the
exact manifest passes the existing recorded-observation ACT-B policy shadow.
If rebuilding or shadow admission fails, the last reviewed baseline entry is
retained and the failed attempt is recorded in the refresh report.
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
import time
from typing import Any, Mapping


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "src/lerobot_robot_doosan_a0509"
QUEST_PACKAGE_ROOT = REPOSITORY_ROOT / "src/quest_a0509_teleop"
for value in (str(REPOSITORY_ROOT), str(PACKAGE_ROOT), str(QUEST_PACKAGE_ROOT)):
    if value not in sys.path:
        sys.path.insert(0, value)

from lerobot_robot_doosan_a0509.interior_policy.catalog import (  # noqa: E402
    load_operator_catalog,
)
from lerobot_robot_doosan_a0509.interior_policy.execution_tail import (  # noqa: E402
    derive_zero_cost_execution_tail,
)
from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (  # noqa: E402
    EdgeRuntimeRegistry,
    FLEXIBLE_VERIFIED,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (  # noqa: E402
    EpisodeHandoffManifest,
)
from scripts.rebuild_a0509_t7_t3_execution_tail_reference import (  # noqa: E402
    main as rebuild_semantic_local_reference,
)


DEFAULT_CATALOG = (
    REPOSITORY_ROOT
    / "config/interior_policy/a0509_t1_t8_operator_catalog_spatial_v2.json"
)
DEFAULT_BASE_REGISTRY = (
    REPOSITORY_ROOT
    / "docs/artifacts/"
    "t1_t8_spatial_floor_level2_semantic_local_2026-09-03/"
    "edge_registry_level2_spatial_v5.json"
)
DEFAULT_VALIDATION = (
    REPOSITORY_ROOT
    / "offline_tools/cross_task_handoff/validation_config_a0509_v2.json"
)
DEFAULT_OUTPUT_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/"
    "t1_t8_spatial_floor_level2_semantic_local_all_2026-09-03"
)
REGISTRY_NAME = "edge_registry_level2_spatial_v6.json"
SUMMARY_NAME = "semantic_local_refresh_summary.json"
MATRIX_NAME = "semantic_local_edge_matrix.csv"

# Recorded-observation policy-shadow retries found that the geometric minimum
# (rank 0) for this edge violates the 7.5 mm/tick crossfade command limit,
# while rank 11 preserves the same semantic-local objective and passes every
# command-space check. Keep that reviewed choice reproducible on a clean
# registry rebuild. Other newly generated edges use the CLI default rank.
REVIEWED_SELECTION_RANKS: dict[tuple[str, str], int] = {
    ("T8.acquire_top_block_from_stack", "T3.deliver_to_floor"): 11,
}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--stage",
        choices=("references", "shadows", "registry", "all"),
        default="all",
    )
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument(
        "--base-registry", type=Path, default=DEFAULT_BASE_REGISTRY
    )
    parser.add_argument(
        "--validation-config", type=Path, default=DEFAULT_VALIDATION
    )
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument(
        "--edge",
        action="append",
        default=None,
        help="Optional source->successor key; may be repeated.",
    )
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--force", action="store_true")
    parser.add_argument(
        "--selection-rank",
        type=int,
        default=0,
        help=(
            "Rank among hard-safe candidates for newly rebuilt edges. "
            "Existing reviewed semantic-local edges retain their selected rank."
        ),
    )
    return parser.parse_args()


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.expanduser().resolve().read_text(encoding="utf-8"))


def _write(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _edge_key(source: str, successor: str) -> str:
    return f"{source}->{successor}"


def _edge_slug(source: str, successor: str) -> str:
    return (
        source.lower().replace(".", "_")
        + "__to__"
        + successor.lower().replace(".", "_")
    )


def _selected(args: argparse.Namespace, source: str, successor: str) -> bool:
    if args.edge is None:
        return True
    return _edge_key(source, successor) in set(args.edge)


def _resolve_registry_path(value: object, registry_path: Path) -> Path | None:
    if value is None or not str(value).strip():
        return None
    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = registry_path.parent / path
    return path.resolve()


def _relative(path: Path | None, base: Path) -> str | None:
    if path is None:
        return None
    return os.path.relpath(path.resolve(), start=base.resolve())


def _shadow_pass(value: Mapping[str, Any], manifest: EpisodeHandoffManifest) -> bool:
    admission = value.get("prefix_admission")
    timing = value.get("control_timing")
    bridge = value.get("bridge")
    return bool(
        value.get("schema_version")
        == "a0509.task_c_handoff_v2_policy_shadow.v1"
        and value.get("handoff_id") == manifest.handoff_id
        and value.get("bridge_admission_mode") == "flexible_level2"
        and isinstance(bridge, Mapping)
        and bridge.get("bridge_mode") == "flexible_level2"
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


def _semantic_local_artifact_valid(
    manifest_path: Path | None,
    shadow_path: Path | None,
) -> bool:
    if manifest_path is None or shadow_path is None:
        return False
    if not manifest_path.is_file() or not shadow_path.is_file():
        return False
    try:
        manifest = EpisodeHandoffManifest.load(manifest_path)
        shadow = _load(shadow_path)
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
        return False
    return bool(
        manifest.semantic_local_total_length_mm is not None
        and manifest.runtime_successor_bank_path is not None
        and _shadow_pass(shadow, manifest)
    )


def _base_records(path: Path) -> tuple[dict[str, Any], dict[tuple[str, str], dict[str, Any]]]:
    value = _load(path)
    records = {
        (str(item["source_operator"]), str(item["successor_operator"])): dict(item)
        for item in value["edges"]
    }
    if len(records) != len(value["edges"]):
        raise ValueError("base registry contains duplicate edge keys")
    return value, records


def _status_path(output_root: Path, source: str, successor: str) -> Path:
    return output_root / "edges" / _edge_slug(source, successor) / "batch_status.json"


def _status(
    output_root: Path,
    source: str,
    successor: str,
    **updates: Any,
) -> dict[str, Any]:
    path = _status_path(output_root, source, successor)
    value: dict[str, Any]
    if path.is_file():
        value = _load(path)
    else:
        value = {
            "schema_version": "a0509.semantic_local_edge_refresh_status.v1",
            "source_operator": source,
            "successor_operator": successor,
            "robot_commands_published": 0,
            "live_enabled": False,
            "mux_selected": False,
            "physical_validation_performed": False,
        }
    value.update(updates)
    _write(path, value)
    return value


def _is_preserved_semantic_local(
    raw_edge: Mapping[str, Any],
    *,
    base_registry: Path,
) -> bool:
    if "semantic_local" not in str(raw_edge.get("validation_method", "")):
        return False
    return _semantic_local_artifact_valid(
        _resolve_registry_path(raw_edge.get("handoff_manifest"), base_registry),
        _resolve_registry_path(raw_edge.get("policy_shadow"), base_registry),
    )


def _eligible_edges(args: argparse.Namespace):
    base_path = args.base_registry.expanduser().resolve()
    base_value, base = _base_records(base_path)
    catalog = load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        config_path=args.catalog,
        audit_raw_frames=False,
    )
    values = []
    for key in sorted(base):
        source, successor = key
        source_operator = catalog.by_id[source]
        tail = derive_zero_cost_execution_tail(source_operator)
        values.append((source, successor, base[key], tail))
    return base_value, base, catalog, values


def build_references(args: argparse.Namespace) -> None:
    base_path = args.base_registry.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()
    _, _, catalog, edges = _eligible_edges(args)
    eligible_total = sum(item[3].profile is not None for item in edges)
    progress = 0
    for source, successor, raw_edge, tail_result in edges:
        if tail_result.profile is None or not _selected(args, source, successor):
            continue
        progress += 1
        tail = tail_result.profile
        if _is_preserved_semantic_local(raw_edge, base_registry=base_path):
            _status(
                output_root,
                source,
                successor,
                applicable=True,
                reference_status="preserved_reviewed_semantic_local",
                source_execution_tail=tail.to_record(),
                baseline_manifest=str(
                    _resolve_registry_path(raw_edge["handoff_manifest"], base_path)
                ),
                baseline_shadow=str(
                    _resolve_registry_path(raw_edge["policy_shadow"], base_path)
                ),
            )
            print(
                f"SEMANTIC_REFERENCE_PRESERVE {progress}/{eligible_total} "
                f"edge={_edge_key(source, successor)}",
                flush=True,
            )
            continue
        edge_root = output_root / "edges" / _edge_slug(source, successor)
        manifest_path = _resolve_registry_path(
            raw_edge.get("handoff_manifest"), base_path
        )
        if manifest_path is None or not manifest_path.is_file():
            _status(
                output_root,
                source,
                successor,
                applicable=True,
                reference_status="failed",
                reference_error="baseline manifest is missing",
                source_execution_tail=tail.to_record(),
            )
            print(
                f"SEMANTIC_REFERENCE_FAIL {progress}/{eligible_total} "
                f"edge={_edge_key(source, successor)} reason=missing_baseline",
                flush=True,
            )
            continue
        selection_path = edge_root / "semantic_local_reference_selection.json"
        if selection_path.is_file() and not args.force:
            _status(
                output_root,
                source,
                successor,
                applicable=True,
                reference_status="generated",
                source_execution_tail=tail.to_record(),
                baseline_manifest=str(manifest_path),
            )
            print(
                f"SEMANTIC_REFERENCE_REUSE {progress}/{eligible_total} "
                f"edge={_edge_key(source, successor)}",
                flush=True,
            )
            continue
        selected_rank = REVIEWED_SELECTION_RANKS.get(
            (source, successor), args.selection_rank
        )
        command = [
            "--catalog",
            str(args.catalog),
            "--baseline-manifest",
            str(manifest_path),
            "--validation-config",
            str(args.validation_config),
            "--output-root",
            str(edge_root),
            "--source-operator",
            source,
            "--successor-operator",
            successor,
            "--expected-source-segment",
            tail.tracking_segment,
            "--expected-commit-phase-low",
            repr(tail.commit_phase_low),
            "--expected-commit-phase-high",
            repr(tail.commit_phase_high),
            "--selection-rank",
            str(selected_rank),
            # T1/T4/T5 carry the block into a container/drawer and therefore
            # do not rise 50 mm above both grasp and release heights.  The
            # semantic segment, 30-frame endpoint margins, closed-gripper
            # mask, and 15 mm interior path margin define the B reference
            # domain.  No fixed Z minimum is a runtime authority here.
            "--minimum-transport-clearance-mm",
            str(
                0.0
                if catalog.by_id[successor].policy_id in {"T1", "T4", "T5"}
                else 50.0
            ),
        ]
        if args.force:
            command.append("--force")
        started = time.perf_counter()
        try:
            rebuild_semantic_local_reference(command)
        except Exception as exc:  # Continue so every edge receives a result.
            _status(
                output_root,
                source,
                successor,
                applicable=True,
                reference_status="failed",
                reference_error=f"{type(exc).__name__}: {exc}",
                reference_elapsed_s=time.perf_counter() - started,
                source_execution_tail=tail.to_record(),
                baseline_manifest=str(manifest_path),
            )
            print(
                f"SEMANTIC_REFERENCE_FAIL {progress}/{eligible_total} "
                f"edge={_edge_key(source, successor)} error={type(exc).__name__}",
                flush=True,
            )
            continue
        selection = _load(selection_path)
        _status(
            output_root,
            source,
            successor,
            applicable=True,
            reference_status="generated",
            reference_elapsed_s=time.perf_counter() - started,
            source_execution_tail=tail.to_record(),
            baseline_manifest=str(manifest_path),
            semantic_local_objective=selection.get("selected"),
        )
        print(
            f"SEMANTIC_REFERENCE_OK {progress}/{eligible_total} "
            f"edge={_edge_key(source, successor)}",
            flush=True,
        )


def run_shadows(args: argparse.Namespace) -> None:
    base_path = args.base_registry.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()
    _, _, catalog, edges = _eligible_edges(args)
    eligible_total = sum(item[3].profile is not None for item in edges)
    progress = 0
    for source, successor, raw_edge, tail_result in edges:
        if tail_result.profile is None or not _selected(args, source, successor):
            continue
        progress += 1
        if _is_preserved_semantic_local(raw_edge, base_registry=base_path):
            print(
                f"SEMANTIC_SHADOW_PRESERVE {progress}/{eligible_total} "
                f"edge={_edge_key(source, successor)}",
                flush=True,
            )
            continue
        edge_root = output_root / "edges" / _edge_slug(source, successor)
        manifest_path = edge_root / "flexible_reference_manifest.json"
        shadow_path = edge_root / "flexible_policy_shadow.json"
        if not manifest_path.is_file():
            _status(
                output_root,
                source,
                successor,
                shadow_status="skipped_missing_reference",
            )
            print(
                f"SEMANTIC_SHADOW_SKIP {progress}/{eligible_total} "
                f"edge={_edge_key(source, successor)} reason=missing_reference",
                flush=True,
            )
            continue
        manifest = EpisodeHandoffManifest.load(manifest_path)
        if shadow_path.is_file() and not args.force:
            passed = _shadow_pass(_load(shadow_path), manifest)
            _status(
                output_root,
                source,
                successor,
                shadow_status="passed" if passed else "failed",
                shadow_pass=passed,
            )
            print(
                f"SEMANTIC_SHADOW_REUSE {progress}/{eligible_total} "
                f"edge={_edge_key(source, successor)} pass={passed}",
                flush=True,
            )
            continue
        if shadow_path.exists():
            shadow_path.unlink()
        successor_task = catalog.audit.tasks[catalog.by_id[successor].policy_id]
        command = [
            sys.executable,
            "-m",
            "offline_tools.cross_task_handoff.run_v2_policy_shadow",
            "--episode-manifest",
            str(manifest_path),
            "--checkpoint-b",
            successor_task.checkpoint,
            "--dataset-b",
            successor_task.dataset_root,
            "--validation-config",
            str(args.validation_config),
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
            "--adaptive-b-max-splice-index",
            "8",
            "--adaptive-b-max-candidates",
            "6",
            "--device",
            args.device,
            "--output",
            str(shadow_path),
        ]
        environment = os.environ.copy()
        environment["PYTHONPATH"] = os.pathsep.join(
            item
            for item in (
                str(PACKAGE_ROOT),
                str(QUEST_PACKAGE_ROOT),
                str(REPOSITORY_ROOT),
                environment.get("PYTHONPATH", ""),
            )
            if item
        )
        started = time.perf_counter()
        completed = subprocess.run(
            command,
            cwd=REPOSITORY_ROOT,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        passed = False
        if shadow_path.is_file():
            passed = _shadow_pass(_load(shadow_path), manifest)
        execution = {
            "schema_version": "a0509.semantic_local_shadow_execution.v1",
            "returncode": completed.returncode,
            "elapsed_s": time.perf_counter() - started,
            "passed": passed,
            "stdout_tail": completed.stdout[-4000:],
            "stderr_tail": completed.stderr[-4000:],
            "robot_commands_published": 0,
        }
        _write(edge_root / "shadow_execution.json", execution)
        _status(
            output_root,
            source,
            successor,
            shadow_status="passed" if passed else "failed",
            shadow_pass=passed,
            shadow_execution=execution,
        )
        print(
            f"SEMANTIC_SHADOW {progress}/{eligible_total} "
            f"edge={_edge_key(source, successor)} "
            f"rc={completed.returncode} pass={passed}",
            flush=True,
        )


def _rebase_baseline_edge(
    edge: Mapping[str, Any],
    *,
    base_registry: Path,
    output_root: Path,
) -> dict[str, Any]:
    value = dict(edge)
    for field in ("handoff_manifest", "policy_shadow"):
        value[field] = _relative(
            _resolve_registry_path(value.get(field), base_registry), output_root
        )
    return value


def build_registry(args: argparse.Namespace) -> dict[str, Any]:
    base_path = args.base_registry.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()
    base_value, base, _, edges = _eligible_edges(args)
    registry_edges: list[dict[str, Any]] = []
    report_edges: list[dict[str, Any]] = []
    counts: Counter[str] = Counter()
    total_lengths: list[float] = []

    for source, successor, raw_edge, tail_result in edges:
        edge = _rebase_baseline_edge(
            raw_edge, base_registry=base_path, output_root=output_root
        )
        evidence = dict(edge.get("evidence", {}))
        if tail_result.profile is None:
            counts["not_applicable_source_contract"] += 1
            evidence.update(
                {
                    "semantic_local_refresh": "not_applicable",
                    "semantic_local_not_applicable_reason": tail_result.reason,
                }
            )
            edge["evidence"] = evidence
            report_edges.append(
                {
                    "source_operator": source,
                    "successor_operator": successor,
                    "result": "not_applicable_source_contract",
                    "reason": tail_result.reason,
                    "active_validation_method": edge["validation_method"],
                }
            )
            registry_edges.append(edge)
            continue

        if _is_preserved_semantic_local(raw_edge, base_registry=base_path):
            counts["preserved_reviewed_semantic_local"] += 1
            manifest_path = _resolve_registry_path(
                raw_edge["handoff_manifest"], base_path
            )
            assert manifest_path is not None
            manifest = EpisodeHandoffManifest.load(manifest_path)
            assert manifest.semantic_local_total_length_mm is not None
            total_lengths.append(manifest.semantic_local_total_length_mm)
            evidence.update(
                {
                    "semantic_local_refresh": "preserved_reviewed",
                    "semantic_local_objective": (
                        "source_semantic_prefix+Bridge+successor_semantic_suffix"
                    ),
                    "semantic_local_total_length_mm": (
                        manifest.semantic_local_total_length_mm
                    ),
                    "dijkstra_operator_cost_changed": False,
                    "runtime_successor_phase_gate": False,
                }
            )
            edge["evidence"] = evidence
            report_edges.append(
                {
                    "source_operator": source,
                    "successor_operator": successor,
                    "result": "preserved_reviewed_semantic_local",
                    "handoff_id": manifest.handoff_id,
                    "total_length_mm": manifest.semantic_local_total_length_mm,
                    "active_validation_method": edge["validation_method"],
                }
            )
            registry_edges.append(edge)
            continue

        edge_root = output_root / "edges" / _edge_slug(source, successor)
        manifest_path = edge_root / "flexible_reference_manifest.json"
        shadow_path = edge_root / "flexible_policy_shadow.json"
        promoted = _semantic_local_artifact_valid(manifest_path, shadow_path)
        if promoted:
            manifest = EpisodeHandoffManifest.load(manifest_path)
            overlay = _load(edge_root / "level2_registry_overlay.json")
            overlay_edges = list(overlay.get("edges", ()))
            if len(overlay_edges) != 1:
                raise ValueError(f"invalid semantic-local overlay: {edge_root}")
            overlay_edge = dict(overlay_edges[0])
            if (
                overlay_edge.get("source_operator") != source
                or overlay_edge.get("successor_operator") != successor
            ):
                raise ValueError(f"semantic-local overlay edge mismatch: {edge_root}")
            objective = {
                "source_prefix_mm": manifest.semantic_local_source_prefix_mm,
                "bridge_length_mm": manifest.semantic_local_bridge_length_mm,
                "successor_suffix_mm": manifest.semantic_local_successor_suffix_mm,
                "total_length_mm": manifest.semantic_local_total_length_mm,
            }
            assert objective["total_length_mm"] is not None
            total_lengths.append(float(objective["total_length_mm"]))
            evidence.update(dict(overlay_edge.get("evidence", {})))
            evidence.update(
                {
                    "command_safe": True,
                    "fresh_successor_shadow_pass": True,
                    "crossfade_pass": True,
                    "semantic_local_refresh": "promoted",
                    "semantic_local_objective": (
                        "source_semantic_prefix+Bridge+successor_semantic_suffix"
                    ),
                    "semantic_local_components_mm": objective,
                    "dijkstra_operator_cost_changed": False,
                    "runtime_successor_phase_gate": False,
                    "physical_trials": 0,
                    "ik_checked": False,
                    "collision_checked": False,
                }
            )
            edge.update(
                {
                    "handoff_manifest": _relative(manifest_path, output_root),
                    "policy_shadow": _relative(shadow_path, output_root),
                    "admission_status": FLEXIBLE_VERIFIED,
                    "validation_method": str(
                        overlay_edge["validation_method"]
                    ),
                    "uncertainty_penalty": 0.0,
                    "evidence": evidence,
                }
            )
            counts["promoted_semantic_local"] += 1
            report_edges.append(
                {
                    "source_operator": source,
                    "successor_operator": successor,
                    "result": "promoted_semantic_local",
                    "handoff_id": manifest.handoff_id,
                    "total_length_mm": objective["total_length_mm"],
                    "source_prefix_mm": objective["source_prefix_mm"],
                    "bridge_length_mm": objective["bridge_length_mm"],
                    "successor_suffix_mm": objective["successor_suffix_mm"],
                    "active_validation_method": edge["validation_method"],
                }
            )
        else:
            counts["baseline_retained_after_failed_attempt"] += 1
            status_path = _status_path(output_root, source, successor)
            status = _load(status_path) if status_path.is_file() else {}
            evidence.update(
                {
                    "semantic_local_refresh": "attempt_failed_baseline_retained",
                    "semantic_local_attempt_status": status.get(
                        "shadow_status", status.get("reference_status", "missing")
                    ),
                }
            )
            edge["evidence"] = evidence
            report_edges.append(
                {
                    "source_operator": source,
                    "successor_operator": successor,
                    "result": "baseline_retained_after_failed_attempt",
                    "reason": status.get(
                        "reference_error",
                        status.get("shadow_status", "artifacts_missing_or_invalid"),
                    ),
                    "active_validation_method": edge["validation_method"],
                }
            )
        registry_edges.append(edge)

    if len(registry_edges) != len(base):
        raise RuntimeError("semantic-local refresh changed registry edge count")
    contract = dict(base_value.get("selection_contract", {}))
    contract.update(
        {
            "base_registry_read_only": str(base_path),
            "bridge_reference_objective": (
                "source_semantic_prefix+Bridge+successor_semantic_suffix"
            ),
            "semantic_local_scope": (
                "all data-derived held-object/free-transport source collars"
            ),
            "eligible_edge_count": counts["promoted_semantic_local"]
            + counts["preserved_reviewed_semantic_local"]
            + counts["baseline_retained_after_failed_attempt"],
            "runtime_successor_phase_gate": False,
            "dijkstra_operator_cost_changed": False,
            "failed_refresh_retains_last_reviewed_edge": True,
            "robot_commands_published": 0,
            "physical_validation_performed": False,
        }
    )
    registry = {
        **base_value,
        "composition_id": (
            "a0509_t1_t8_spatial_floor_semantic_local_all_level2_20260903"
        ),
        "selection_contract": contract,
        "edges": sorted(
            registry_edges,
            key=lambda item: (
                str(item["source_operator"]), str(item["successor_operator"])
            ),
        ),
    }
    registry_path = output_root / REGISTRY_NAME
    _write(registry_path, registry)
    loaded = EdgeRuntimeRegistry.load(registry_path)
    summary = {
        "schema_version": "a0509.semantic_local_level2_refresh.v1",
        "base_registry": str(base_path),
        "output_registry": str(registry_path),
        "objective": "source_semantic_prefix+Bridge+successor_semantic_suffix",
        "level2_edge_count": len(loaded.edges),
        "counts": dict(sorted(counts.items())),
        "semantic_local_total_length_mm": {
            "samples": len(total_lengths),
            "minimum": min(total_lengths) if total_lengths else None,
            "maximum": max(total_lengths) if total_lengths else None,
            "mean": (
                sum(total_lengths) / len(total_lengths) if total_lengths else None
            ),
        },
        "runtime_successor_phase_gate": False,
        "dijkstra_operator_cost_changed": False,
        "failed_refresh_retains_last_reviewed_edge": True,
        "robot_commands_published": 0,
        "live_enabled": False,
        "mux_selected": False,
        "physical_validation_performed": False,
        "edges": report_edges,
    }
    _write(output_root / SUMMARY_NAME, summary)
    matrix_path = output_root / MATRIX_NAME
    matrix_path.parent.mkdir(parents=True, exist_ok=True)
    fields = (
        "source_operator",
        "successor_operator",
        "result",
        "active_validation_method",
        "handoff_id",
        "source_prefix_mm",
        "bridge_length_mm",
        "successor_suffix_mm",
        "total_length_mm",
        "reason",
    )
    with matrix_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(report_edges)
    print(
        "SEMANTIC_LOCAL_LEVEL2_REGISTRY_OK "
        f"edges={len(loaded.edges)} "
        f"promoted={counts['promoted_semantic_local']} "
        f"preserved={counts['preserved_reviewed_semantic_local']} "
        f"baseline_retained={counts['baseline_retained_after_failed_attempt']} "
        f"not_applicable={counts['not_applicable_source_contract']} "
        f"output={registry_path} robot_commands_published=0"
    )
    return summary


def main() -> None:
    args = _parse_args()
    if args.selection_rank < 0:
        raise ValueError("selection-rank must be non-negative")
    # Validate every inherited path before performing an expensive refresh.
    EdgeRuntimeRegistry.load(args.base_registry)
    if args.stage in {"references", "all"}:
        build_references(args)
    if args.stage in {"shadows", "all"}:
        run_shadows(args)
    if args.stage in {"registry", "all"}:
        build_registry(args)


if __name__ == "__main__":
    main()
