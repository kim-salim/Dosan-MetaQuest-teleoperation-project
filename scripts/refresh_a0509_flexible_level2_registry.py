#!/usr/bin/env python3
"""Rebuild the T1--T8 Level-2 edge registry around FLEXIBLE_LEVEL2.

This tool is command-free.  It never creates a ROS node, selects the MUX,
enables Live, or publishes a robot/gripper command.  Existing STRICT evidence
is retained as an immutable baseline reference, while every direct symbolic
edge receives a bounded reference-guided geometry evaluation.  Only edges
whose exact FLEXIBLE manifest also passes a fresh successor ACT shadow are
marked ``flexible_verified``.
"""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Iterable, Mapping

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "src/lerobot_robot_doosan_a0509"
for value in (str(REPOSITORY_ROOT), str(PACKAGE_ROOT)):
    if value not in sys.path:
        sys.path.insert(0, value)

from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (  # noqa: E402
    EdgeRuntimeRegistry,
    FLEXIBLE_SEMANTIC_CANDIDATE,
    FLEXIBLE_VERIFIED,
    TEMPORARILY_UNAVAILABLE,
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
    candidate_to_flexible_manifest,
    runtime_limits_from_validation,
)
from quest_a0509_teleop.doosan_orientation import (  # noqa: E402
    quaternion_to_doosan_zyz_deg,
)


DEFAULT_INVENTORY = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_full_level2_coverage_2026-08-29"
    / "level1_edge_inventory.json"
)
DEFAULT_CANDIDATE_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_full_level2_coverage_2026-08-29"
)
DEFAULT_RECOVERY_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_borderline_curvature_recovery_2026-08-29"
)
DEFAULT_STRICT_REGISTRY = (
    DEFAULT_RECOVERY_ROOT / "edge_registry_all_level2_extended.json"
)
DEFAULT_VALIDATION_CONFIG = (
    REPOSITORY_ROOT
    / "offline_tools/cross_task_handoff/validation_config_a0509_v2.json"
)
DEFAULT_CATALOG = (
    REPOSITORY_ROOT
    / "config/interior_policy/a0509_t1_t6_operator_catalog_v1.json"
)
DEFAULT_OUTPUT_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_flexible_level2_registry_2026-08-30"
)
DEFAULT_VERIFIED_OVERLAY = (
    REPOSITORY_ROOT
    / "docs/artifacts/t7_to_t3_s2_runtime_reference_2026-08-31"
    / "level2_registry_overlay.json"
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--stage",
        choices=("geometry", "shadows", "summary", "all"),
        default="all",
    )
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument(
        "--candidate-root", type=Path, default=DEFAULT_CANDIDATE_ROOT
    )
    parser.add_argument("--recovery-root", type=Path, default=DEFAULT_RECOVERY_ROOT)
    parser.add_argument(
        "--strict-registry", type=Path, default=DEFAULT_STRICT_REGISTRY
    )
    parser.add_argument(
        "--validation-config", type=Path, default=DEFAULT_VALIDATION_CONFIG
    )
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument(
        "--verified-overlay",
        type=Path,
        action="append",
        default=None,
        help=(
            "Command-free verified edge overlay; may be repeated. The "
            "manifest and policy-shadow identity are revalidated."
        ),
    )
    parser.add_argument("--edge-id")
    parser.add_argument("--max-reference-candidates", type=int, default=16)
    parser.add_argument("--max-safe-references", type=int, default=3)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.expanduser().resolve().read_text(encoding="utf-8"))


def _write(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _relative(path: Path | None, base: Path) -> str | None:
    if path is None:
        return None
    return os.path.relpath(path.resolve(), start=base.resolve())


def _edge_map(inventory: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(item["edge_id"]): dict(item) for item in inventory["edges"]}


def _edge_library_path(
    edge_id: str,
    *,
    candidate_root: Path,
    recovery_root: Path,
) -> Path:
    recovery = recovery_root / "edges" / edge_id / "candidate_library.json"
    if recovery.is_file():
        return recovery
    return candidate_root / "edges" / edge_id / "candidate_library.json"


def _finite_metric(candidate: Mapping[str, Any], name: str) -> float:
    try:
        value = float(candidate.get(name, math.inf))
    except (TypeError, ValueError):
        return math.inf
    return value if math.isfinite(value) else math.inf


def _candidate_rank(candidate: Mapping[str, Any]) -> tuple[Any, ...]:
    reasons = candidate.get("rejection_reason", ())
    reason_count = len(reasons) if isinstance(reasons, list) else int(bool(reasons))
    return (
        0 if bool(candidate.get("feasible", False)) else 1,
        reason_count,
        _finite_metric(candidate, "max_acceleration"),
        _finite_metric(candidate, "max_velocity"),
        _finite_metric(candidate, "jerk_metric"),
        _finite_metric(candidate, "max_orientation_step_deg"),
        _finite_metric(candidate, "bridge_length"),
        -_finite_metric(candidate, "support_score"),
        str(candidate.get("handoff_id", "")),
    )


def _shortlist_candidates(
    candidates: Iterable[dict[str, Any]],
    *,
    limit: int,
) -> list[dict[str, Any]]:
    values = list(candidates)
    if not values:
        return []
    ordered = sorted(values, key=_candidate_rank)
    primary_count = max(1, limit // 2)
    selected = list(ordered[:primary_count])

    # Add deterministic coverage over the episode-pair grid.  This prevents a
    # shortlist from collapsing to one source/support mode while keeping the
    # expensive reference deformation search bounded.
    episode_order = sorted(
        values,
        key=lambda item: (
            int(item.get("source_support_episode", -1)),
            int(item.get("successor_support_episode", -1)),
            str(item.get("handoff_id", "")),
        ),
    )
    remaining = max(0, limit - len(selected))
    if remaining:
        indices = np.linspace(
            0,
            len(episode_order) - 1,
            num=min(remaining, len(episode_order)),
            dtype=np.int64,
        )
        selected.extend(episode_order[int(index)] for index in indices)
    unique: dict[str, dict[str, Any]] = {}
    for item in selected + ordered:
        unique.setdefault(str(item["handoff_id"]), item)
        if len(unique) >= limit:
            break
    return list(unique.values())


def _flexible_copy(manifest: EpisodeHandoffManifest) -> EpisodeHandoffManifest:
    value = manifest.to_record()
    generation = dict(value.get("generation", {}))
    generation.update(
        {
            "method": "flexible_reference",
            "bridge_admission_mode": "flexible_level2",
            "reference_only": True,
            "legacy_reference_preserved": True,
        }
    )
    value["generation"] = generation
    for boundary_name in ("source", "successor"):
        boundary = dict(value[boundary_name])
        boundary["support_radius_mm"] = max(
            90.0, float(boundary.get("support_radius_mm", 0.0))
        )
        boundary["support_orientation_radius_deg"] = max(
            30.0,
            float(boundary.get("support_orientation_radius_deg", 0.0)),
        )
        value[boundary_name] = boundary
    validation = dict(value.get("validation", {}))
    validation.update(
        {
            "hard_filter_passed": False,
            "robot_executable": False,
            "dry_run_only": True,
            "flexible_runtime_revalidation_required": True,
        }
    )
    value["validation"] = validation
    return EpisodeHandoffManifest.from_mapping(value)


def _snapshot(manifest: EpisodeHandoffManifest) -> BridgeRuntimeSnapshot:
    source_angles = quaternion_to_doosan_zyz_deg(
        manifest.source.nominal_orientation_quat_xyzw,
        [0.0, 150.0, 0.0],
    )
    pose = np.concatenate((manifest.source.nominal_position_mm, source_angles))
    return BridgeRuntimeSnapshot(
        timestamp_s=time.monotonic(),
        actual_pose_mm_deg=pose,
        acknowledged_pose_mm_deg=pose,
        actual_velocity_mm_s=manifest.source.nominal_velocity_mm_s,
        gripper_target=manifest.source.semantic.gripper_target,
    )


def _search_record(result: Any) -> dict[str, Any]:
    selected = None if result.selected is None else result.selected.record()
    return {
        "valid": result.valid,
        "candidates_evaluated": result.candidates_evaluated,
        "candidates_hard_passed": result.candidates_hard_passed,
        "search_latency_ms": result.search_latency_s * 1000.0,
        "selected": selected,
        "rejected_reason_counts": dict(result.rejected_reason_counts),
        "timed_out": result.timed_out,
    }


def run_geometry(args: argparse.Namespace) -> None:
    inventory = _load(args.inventory)
    edges = _edge_map(inventory)
    strict = EdgeRuntimeRegistry.load(args.strict_registry).by_key
    validation = _load(args.validation_config)
    limits = runtime_limits_from_validation(validation)
    runtime = HandoffV2Config(
        enabled=True,
        control_hz=30.0,
        bridge_admission_mode="flexible_level2",
        adaptive_b_max_splice_index=8,
        adaptive_b_max_candidates=6,
        semantic_authority="external_planner",
    )
    search_config = FlexibleBridgeSearchConfig()
    output_root = args.output_root.expanduser().resolve()
    for index, edge in enumerate(inventory["edges"], start=1):
        edge_id = str(edge["edge_id"])
        if args.edge_id is not None and edge_id != args.edge_id:
            continue
        edge_root = output_root / "edges" / edge_id
        evaluation_path = edge_root / "geometry_evaluation.json"
        manifest_path = edge_root / "flexible_reference_manifest.json"
        if evaluation_path.is_file() and not args.force:
            cached = _load(evaluation_path)
            print(
                f"FLEX_GEOMETRY_REUSE {index}/{len(edges)} edge={edge_id} "
                f"safe={cached['command_safe']}",
                flush=True,
            )
            continue

        key = (str(edge["source_operator"]), str(edge["successor_operator"]))
        references: list[tuple[str, EpisodeHandoffManifest]] = []
        baseline = strict.get(key)
        if baseline is not None and baseline.handoff_manifest_path is not None:
            references.append(
                (
                    "strict_verified_reference",
                    _flexible_copy(
                        EpisodeHandoffManifest.load(
                            baseline.handoff_manifest_path
                        )
                    ),
                )
            )

        library_path = _edge_library_path(
            edge_id,
            candidate_root=args.candidate_root,
            recovery_root=args.recovery_root,
        )
        library = _load(library_path)
        shortlist = _shortlist_candidates(
            list(library["candidates"]),
            limit=args.max_reference_candidates,
        )
        seen = {item.handoff_id for _, item in references}
        for candidate in shortlist:
            manifest = candidate_to_flexible_manifest(library, candidate)
            if manifest.handoff_id in seen:
                continue
            seen.add(manifest.handoff_id)
            references.append(("bounded_library_shortlist", manifest))

        evaluated: list[dict[str, Any]] = []
        passing: list[tuple[float, EpisodeHandoffManifest, dict[str, Any]]] = []
        for origin, manifest in references:
            result = search_flexible_bridge_queue(
                manifest,
                _snapshot(manifest),
                runtime,
                limits,
                search_config,
                live_mode=False,
            )
            record = {
                "handoff_id": manifest.handoff_id,
                "reference_origin": origin,
                "source_support_episode": manifest.source.support_episode,
                "successor_support_episode": manifest.successor.support_episode,
                "search": _search_record(result),
            }
            evaluated.append(record)
            if result.valid and result.selected is not None:
                passing.append((float(result.selected.score), manifest, record))
            if len(passing) >= args.max_safe_references:
                break

        selected = None if not passing else min(passing, key=lambda item: item[0])
        if selected is not None:
            _write(manifest_path, selected[1].to_record())
        elif manifest_path.exists():
            manifest_path.unlink()
        evaluation = {
            "schema_version": "a0509.flexible_level2_edge_geometry.v1",
            "edge_id": edge_id,
            "source_operator": edge["source_operator"],
            "successor_operator": edge["successor_operator"],
            "baseline_strict_verified": baseline is not None,
            "candidate_library": str(library_path.resolve()),
            "reference_candidates_considered": len(references),
            "reference_candidates_evaluated": len(evaluated),
            "safe_reference_count": len(passing),
            "command_safe": selected is not None,
            "selected_handoff_id": (
                None if selected is None else selected[1].handoff_id
            ),
            "selected_manifest": (
                None if selected is None else str(manifest_path.resolve())
            ),
            "selected_score": None if selected is None else selected[0],
            "evaluations": evaluated,
            "bounded_shortlist": True,
            "max_reference_candidates": args.max_reference_candidates,
            "robot_commands_published": 0,
            "live_enabled": False,
            "mux_selected": False,
            "physical_validation_performed": False,
        }
        _write(evaluation_path, evaluation)
        print(
            f"FLEX_GEOMETRY {index}/{len(edges)} edge={edge_id} "
            f"safe={evaluation['command_safe']} "
            f"evaluated={len(evaluated)}",
            flush=True,
        )


def _shadow_pass(value: Mapping[str, Any]) -> bool:
    admission = value.get("prefix_admission")
    timing = value.get("control_timing")
    bridge = value.get("bridge")
    return bool(
        value.get("schema_version")
        == "a0509.task_c_handoff_v2_policy_shadow.v1"
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


def run_shadows(args: argparse.Namespace) -> None:
    inventory = _load(args.inventory)
    catalog = _load(args.catalog)
    tasks = dict(catalog["tasks"])
    output_root = args.output_root.expanduser().resolve()
    for index, edge in enumerate(inventory["edges"], start=1):
        edge_id = str(edge["edge_id"])
        if args.edge_id is not None and edge_id != args.edge_id:
            continue
        edge_root = output_root / "edges" / edge_id
        geometry_path = edge_root / "geometry_evaluation.json"
        if not geometry_path.is_file():
            raise FileNotFoundError(f"geometry evaluation missing: {geometry_path}")
        geometry = _load(geometry_path)
        if not geometry["command_safe"]:
            print(
                f"FLEX_SHADOW_SKIP {index}/{len(inventory['edges'])} "
                f"edge={edge_id} reason=no_command_safe_reference",
                flush=True,
            )
            continue
        report_path = edge_root / "flexible_policy_shadow.json"
        execution_path = edge_root / "shadow_execution.json"
        if report_path.is_file() and not args.force:
            report = _load(report_path)
            print(
                f"FLEX_SHADOW_REUSE {index}/{len(inventory['edges'])} "
                f"edge={edge_id} pass={_shadow_pass(report)}",
                flush=True,
            )
            continue
        if report_path.exists():
            report_path.unlink()
        successor = dict(tasks[str(edge["successor_policy"])])
        command = [
            sys.executable,
            "-m",
            "offline_tools.cross_task_handoff.run_v2_policy_shadow",
            "--episode-manifest",
            str(edge_root / "flexible_reference_manifest.json"),
            "--checkpoint-b",
            str(successor["checkpoint"]),
            "--dataset-b",
            str(successor["dataset_root"]),
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
            str(report_path),
        ]
        started = time.perf_counter()
        shadow_env = os.environ.copy()
        existing_pythonpath = shadow_env.get("PYTHONPATH", "")
        shadow_env["PYTHONPATH"] = os.pathsep.join(
            item
            for item in (
                str(PACKAGE_ROOT),
                str(REPOSITORY_ROOT),
                existing_pythonpath,
            )
            if item
        )
        completed = subprocess.run(
            command,
            cwd=REPOSITORY_ROOT,
            env=shadow_env,
            text=True,
            capture_output=True,
            check=False,
        )
        execution = {
            "schema_version": "a0509.flexible_level2_shadow_execution.v1",
            "edge_id": edge_id,
            "returncode": completed.returncode,
            "elapsed_s": time.perf_counter() - started,
            "stdout_tail": completed.stdout[-4000:],
            "stderr_tail": completed.stderr[-4000:],
            "report": str(report_path.resolve()) if report_path.is_file() else None,
            "robot_commands_published": 0,
        }
        _write(execution_path, execution)
        passed = report_path.is_file() and _shadow_pass(_load(report_path))
        print(
            f"FLEX_SHADOW {index}/{len(inventory['edges'])} edge={edge_id} "
            f"rc={completed.returncode} pass={passed}",
            flush=True,
        )


def _apply_verified_overlays(
    args: argparse.Namespace,
    *,
    records: list[dict[str, Any]],
    registry_edges: list[dict[str, Any]],
    output_root: Path,
) -> list[dict[str, Any]]:
    """Apply reviewed semantic-local artifacts without trusting stale labels."""

    applied: list[dict[str, Any]] = []
    for overlay_path in args.verified_overlay:
        if not overlay_path.is_file():
            continue
        overlay = _load(overlay_path)
        if (
            overlay.get("schema_version")
            != "a0509.flexible_level2_verified_overlay.v1"
        ):
            raise ValueError(f"unsupported verified overlay: {overlay_path}")
        for edge in overlay.get("edges", ()):
            source = str(edge["source_operator"])
            successor = str(edge["successor_operator"])
            manifest_path = (
                overlay_path.parent / str(edge["handoff_manifest"])
            ).resolve()
            shadow_path = (
                overlay_path.parent / str(edge["policy_shadow"])
            ).resolve()
            manifest = EpisodeHandoffManifest.load(manifest_path)
            shadow = _load(shadow_path)
            if manifest.bridge_admission_mode.value != "flexible_level2":
                raise ValueError("verified overlay manifest is not FLEXIBLE_LEVEL2")
            if manifest.runtime_successor_bank_path is None:
                raise ValueError(
                    "verified semantic-local overlay lacks successor support bank"
                )
            if shadow.get("handoff_id") != manifest.handoff_id or not _shadow_pass(
                shadow
            ):
                raise ValueError(
                    "verified overlay policy shadow is stale or did not take over"
                )
            registry_matches = [
                item
                for item in registry_edges
                if item["source_operator"] == source
                and item["successor_operator"] == successor
            ]
            record_matches = [
                item
                for item in records
                if item["source_operator"] == source
                and item["successor_operator"] == successor
            ]
            if len(registry_matches) != 1 or len(record_matches) != 1:
                raise ValueError(
                    f"verified overlay edge is absent or duplicated: {source}->{successor}"
                )
            registry_record = registry_matches[0]
            evidence = {
                **dict(registry_record.get("evidence", {})),
                **dict(edge.get("evidence", {})),
                "command_safe": True,
                "fresh_successor_shadow_pass": True,
                "crossfade_pass": True,
                "physical_trials": 0,
                "ik_checked": False,
                "collision_checked": False,
            }
            registry_record.update(
                {
                    "handoff_manifest": _relative(manifest_path, output_root),
                    "policy_shadow": _relative(shadow_path, output_root),
                    "admission_status": FLEXIBLE_VERIFIED,
                    "validation_method": str(edge["validation_method"]),
                    "uncertainty_penalty": float(
                        edge.get("uncertainty_penalty", 0.0)
                    ),
                    "evidence": evidence,
                }
            )
            record = record_matches[0]
            record.update(
                {
                    "flexible_manifest": str(manifest_path),
                    "flexible_shadow": str(shadow_path),
                    "command_safe": True,
                    "fresh_successor_shadow_pass": True,
                    "admission_status": FLEXIBLE_VERIFIED,
                    "validation_method": str(edge["validation_method"]),
                    "physical_validation_performed": False,
                }
            )
            applied.append(
                {
                    "overlay": str(overlay_path),
                    "source_operator": source,
                    "successor_operator": successor,
                    "handoff_id": manifest.handoff_id,
                    "successor_reference_phase": manifest.successor.phase,
                    "runtime_successor_phase_gate": False,
                }
            )
    return applied


def build_summary(args: argparse.Namespace) -> dict[str, Any]:
    inventory = _load(args.inventory)
    strict = EdgeRuntimeRegistry.load(args.strict_registry).by_key
    output_root = args.output_root.expanduser().resolve()
    records: list[dict[str, Any]] = []
    registry_edges: list[dict[str, Any]] = []
    generator_counts: Counter[str] = Counter()
    shadow_failure_reasons: Counter[str] = Counter()
    splice_index_counts: Counter[int] = Counter()
    geometry_search_latency_ms: list[float] = []
    control_tick_p99_ms: list[float] = []
    control_tick_max_ms: list[float] = []
    control_deadline_misses = 0
    for edge in inventory["edges"]:
        edge_id = str(edge["edge_id"])
        edge_root = output_root / "edges" / edge_id
        geometry_path = edge_root / "geometry_evaluation.json"
        geometry = _load(geometry_path)
        manifest_path = edge_root / "flexible_reference_manifest.json"
        shadow_path = edge_root / "flexible_policy_shadow.json"
        shadow = _load(shadow_path) if shadow_path.is_file() else None
        passed = shadow is not None and _shadow_pass(shadow)
        command_safe = bool(geometry["command_safe"])
        if passed:
            status = FLEXIBLE_VERIFIED
        elif command_safe:
            status = FLEXIBLE_SEMANTIC_CANDIDATE
        else:
            status = TEMPORARILY_UNAVAILABLE
        selected = None
        selected_search: Mapping[str, Any] | None = None
        for evaluation in geometry["evaluations"]:
            if evaluation["handoff_id"] == geometry["selected_handoff_id"]:
                selected_search = evaluation["search"]
                selected = selected_search.get("selected")
                break
        generator = (
            "none"
            if selected is None
            else str(selected.get("generator_type", "reference_guided_flexible"))
        )
        generator_counts[generator] += 1
        if selected_search is not None:
            geometry_search_latency_ms.append(
                float(selected_search.get("search_latency_ms", 0.0))
            )
        if shadow is not None:
            timing = shadow.get("control_timing", {})
            if isinstance(timing, Mapping):
                control_deadline_misses += int(
                    timing.get("deadline_miss_count", 0)
                )
                if timing.get("p99_ms") is not None:
                    control_tick_p99_ms.append(float(timing["p99_ms"]))
                if timing.get("max_ms") is not None:
                    control_tick_max_ms.append(float(timing["max_ms"]))
            admission = shadow.get("prefix_admission", {})
            if isinstance(admission, Mapping):
                if passed and admission.get("splice_index") is not None:
                    splice_index_counts[int(admission["splice_index"])] += 1
                if not passed:
                    reasons = admission.get("failure_reasons", ())
                    if isinstance(reasons, (list, tuple)):
                        shadow_failure_reasons.update(str(item) for item in reasons)
        key = (str(edge["source_operator"]), str(edge["successor_operator"]))
        record = {
            **dict(edge),
            "baseline_strict_verified": key in strict,
            "geometry_evaluation": str(geometry_path.resolve()),
            "flexible_manifest": (
                str(manifest_path.resolve()) if manifest_path.is_file() else None
            ),
            "flexible_shadow": (
                str(shadow_path.resolve()) if shadow_path.is_file() else None
            ),
            "command_safe": command_safe,
            "fresh_successor_shadow_pass": passed,
            "admission_status": status,
            "validation_method": generator,
            "physical_validation_performed": False,
        }
        records.append(record)
        registry_edges.append(
            {
                "source_operator": edge["source_operator"],
                "successor_operator": edge["successor_operator"],
                "handoff_manifest": (
                    _relative(manifest_path, output_root)
                    if manifest_path.is_file()
                    else None
                ),
                "policy_shadow": (
                    _relative(shadow_path, output_root) if passed else None
                ),
                "gripper_event": edge["gripper_event"],
                "admission_status": status,
                "validation_method": generator,
                "uncertainty_penalty": (
                    0.0 if passed else (0.75 if command_safe else 10.0)
                ),
                "evidence": {
                    "baseline_strict_verified": key in strict,
                    "command_safe": command_safe,
                    "fresh_successor_shadow_pass": passed,
                    "crossfade_pass": passed,
                    "physical_trials": 0,
                    "ik_checked": False,
                    "collision_checked": False,
                },
            }
        )

    applied_overlays = _apply_verified_overlays(
        args,
        records=records,
        registry_edges=registry_edges,
        output_root=output_root,
    )
    counts = Counter(item["admission_status"] for item in records)
    strict_keys = set(strict)
    verified_keys = {
        (str(item["source_operator"]), str(item["successor_operator"]))
        for item in records
        if item["admission_status"] == FLEXIBLE_VERIFIED
    }
    edge_id_by_key = {
        (str(item["source_operator"]), str(item["successor_operator"])): str(
            item["edge_id"]
        )
        for item in records
    }

    def distribution(values: list[float]) -> dict[str, float | int | None]:
        if not values:
            return {
                "samples": 0,
                "p50": None,
                "p95": None,
                "p99": None,
                "max": None,
            }
        array = np.asarray(values, dtype=np.float64)
        return {
            "samples": int(len(array)),
            "p50": float(np.percentile(array, 50)),
            "p95": float(np.percentile(array, 95)),
            "p99": float(np.percentile(array, 99)),
            "max": float(np.max(array)),
        }

    summary = {
        "schema_version": "a0509.t1_t8_flexible_level2_refresh.v1",
        "scope": "all 128 direct cross-policy symbolic edges",
        "baseline_strict_registry": str(args.strict_registry.resolve()),
        "baseline_strict_verified_edges": len(strict),
        "counts": {
            "level1_direct_edges": len(records),
            "flexible_verified_edges": counts[FLEXIBLE_VERIFIED],
            "flexible_semantic_candidate_edges": counts[
                FLEXIBLE_SEMANTIC_CANDIDATE
            ],
            "temporarily_unavailable_edges": counts[TEMPORARILY_UNAVAILABLE],
            "command_safe_edges": sum(int(item["command_safe"]) for item in records),
        },
        "strict_to_flexible_comparison": {
            "intersection_verified_edges": len(strict_keys & verified_keys),
            "newly_verified_edges": len(verified_keys - strict_keys),
            "strict_regressed_to_candidate_edges": len(strict_keys - verified_keys),
            "newly_verified_edge_ids": sorted(
                edge_id_by_key[key] for key in verified_keys - strict_keys
            ),
            "strict_regressed_edge_ids": sorted(
                edge_id_by_key[key] for key in strict_keys - verified_keys
            ),
        },
        "geometry_validation": {
            "selected_generator_counts": dict(sorted(generator_counts.items())),
            "search_latency_ms_across_edges": distribution(
                geometry_search_latency_ms
            ),
        },
        "successor_shadow_validation": {
            "adaptive_splice_index_counts": {
                str(key): value for key, value in sorted(splice_index_counts.items())
            },
            "adaptive_nonzero_splice_edges": sum(
                value for key, value in splice_index_counts.items() if key > 0
            ),
            "failure_reason_counts": dict(
                sorted(shadow_failure_reasons.items())
            ),
            "control_deadline_misses": control_deadline_misses,
            "control_tick_p99_ms_across_edges": distribution(
                control_tick_p99_ms
            ),
            "control_tick_max_ms_across_edges": distribution(
                control_tick_max_ms
            ),
        },
        "planner_contract": {
            "runtime_default": "flexible_level2",
            "verified_only": True,
            "unverified_candidates_executable": False,
            "allow_intermediate_release": False,
        },
        "robot_commands_published": 0,
        "live_enabled": False,
        "mux_selected": False,
        "verified_semantic_local_overlays": applied_overlays,
        "physical_validation_performed": False,
        "ik_checked": False,
        "collision_checked": False,
        "edges": records,
    }
    registry = {
        "schema_version": "a0509.interior_policy_v2_edge_registry.v3",
        "composition_id": "a0509_t1_t8_flexible_level2_20260830",
        "selection_contract": {
            "runtime_default": "flexible_level2",
            "source_inventory": str(args.inventory.resolve()),
            "strict_baseline_read_only": str(args.strict_registry.resolve()),
            "semantic_authority": "external_planner",
            "verified_requires_fresh_successor_shadow": True,
            "unverified_candidates_executable": False,
            "candidate_search": "bounded_offline_reference_shortlist",
            "runtime_bridge_search": "bounded_reference_guided_worker",
            "adaptive_b_prefix_indices": list(range(0, 9)),
            "allow_intermediate_release": False,
            "verified_semantic_local_overlays": [
                item["overlay"] for item in applied_overlays
            ],
            "robot_executable": False,
            "physical_validation_performed": False,
        },
        "edges": sorted(
            registry_edges,
            key=lambda item: (item["source_operator"], item["successor_operator"]),
        ),
    }
    _write(output_root / "edge_refresh_summary.json", summary)
    _write(output_root / "edge_registry_level2_v3.json", registry)
    csv_path = output_root / "edge_refresh_matrix.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = (
        "edge_id",
        "source_operator",
        "successor_operator",
        "baseline_strict_verified",
        "command_safe",
        "fresh_successor_shadow_pass",
        "admission_status",
        "validation_method",
        "physical_validation_performed",
    )
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for record in records:
            writer.writerow({name: record.get(name) for name in fieldnames})
    print(
        "FLEX_LEVEL2_REFRESH_SUMMARY "
        f"verified={counts[FLEXIBLE_VERIFIED]} "
        f"candidate={counts[FLEXIBLE_SEMANTIC_CANDIDATE]} "
        f"unavailable={counts[TEMPORARILY_UNAVAILABLE]}",
        flush=True,
    )
    return summary


def main() -> None:
    args = _parse_args()
    for name in (
        "inventory",
        "candidate_root",
        "recovery_root",
        "strict_registry",
        "validation_config",
        "catalog",
        "output_root",
    ):
        setattr(args, name, getattr(args, name).expanduser().resolve())
    overlay_values = (
        [DEFAULT_VERIFIED_OVERLAY]
        if args.verified_overlay is None
        else args.verified_overlay
    )
    args.verified_overlay = tuple(
        item.expanduser().resolve() for item in overlay_values
    )
    if args.max_reference_candidates < 1 or args.max_safe_references < 1:
        raise ValueError("candidate bounds must be positive")
    if args.max_safe_references > args.max_reference_candidates + 1:
        raise ValueError("max_safe_references exceeds reference candidate bound")
    if args.stage in {"geometry", "all"}:
        run_geometry(args)
    if args.stage in {"shadows", "all"}:
        run_shadows(args)
    if args.stage in {"summary", "all"}:
        build_summary(args)


if __name__ == "__main__":
    main()
