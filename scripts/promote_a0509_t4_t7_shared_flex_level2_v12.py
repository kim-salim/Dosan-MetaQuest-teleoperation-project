#!/usr/bin/env python3
"""Promote the shared-profile T4->T7 edge to v12, without robot I/O.

The v11 registry is immutable rollback state.  Promotion requires a fresh
policy shadow plus all-episode geometry coverage over the latched retry scan.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "src/lerobot_robot_doosan_a0509"
for value in (str(REPOSITORY_ROOT), str(PACKAGE_ROOT)):
    if value not in sys.path:
        sys.path.insert(0, value)

from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (  # noqa: E402
    EdgeRuntimeRegistry,
)
from lerobot_robot_doosan_a0509.task_c_handoff.bridge_profile import (  # noqa: E402
    load_bridge_generation_profile,
)
from scripts.promote_a0509_t4_t7_execution_tail_level2 import (  # noqa: E402
    main as promote_execution_tail,
)


ARTIFACT_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_spatial_floor_level2_semantic_local_all_2026-09-03"
)
EDGE_ROOT = (
    ARTIFACT_ROOT
    / "edges/t4_open_drawer__to__t7_acquire_from_drawer_top_"
    "execution_tail_s3_shared_flex_v1"
)
DEFAULT_BASE = ARTIFACT_ROOT / "edge_registry_level2_spatial_v11.json"
DEFAULT_OUTPUT = ARTIFACT_ROOT / "edge_registry_level2_spatial_v12.json"
DEFAULT_OVERLAY = EDGE_ROOT / "level2_registry_overlay.json"
BRIDGE_PROFILE_CONFIG = (
    REPOSITORY_ROOT
    / "config/realtime/a0509_flexible_bridge_profiles_v1.json"
)
TAIL_PROFILE_CONFIG = (
    REPOSITORY_ROOT
    / "config/realtime/a0509_execution_tail_handoff_profile_v2.json"
)
COMPOSITION_ID = (
    "a0509_t1_t8_spatial_floor_shared_profile_t4_t7_level2_v12_20260904"
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-registry", type=Path, default=DEFAULT_BASE)
    parser.add_argument("--verified-overlay", type=Path, default=DEFAULT_OVERLAY)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.expanduser().resolve().read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.resolve().read_bytes()).hexdigest()


def _assert_offline_evidence(edge_root: Path) -> dict[str, Any]:
    manifest = _load(edge_root / "flexible_reference_manifest.json")
    geometry = _load(edge_root / "geometry_evaluation.json")
    primary = _load(edge_root / "profiled_bridge_primary_window_replay.json")
    scan = _load(edge_root / "profiled_bridge_latched_scan_replay.json")
    tail = dict(geometry["source_execution_tail"])
    profile = load_bridge_generation_profile(
        BRIDGE_PROFILE_CONFIG,
        source_reference_mode="execution_tail",
    )
    declared = dict(manifest["generation"]["bridge_profile"])
    if declared.get("profile_id") != profile.profile_id or (
        declared.get("profile_sha256") != profile.profile_sha256
    ):
        raise ValueError("T4->T7 manifest has stale shared-profile provenance")
    if profile.edge_specific_tuning:
        raise ValueError("T4->T7 profile contains edge-specific tuning")
    if tail.get("commit_phase_low") != 0.17 or tail.get("commit_phase_high") != 0.23:
        raise ValueError("T4->T7 data-derived primary commit window changed")
    if tail.get("deadline_phase") != 0.35 or (
        tail.get("latch_commit_window_until_deadline") is not True
    ):
        raise ValueError("T4->T7 shared scan latch contract changed")
    if tail.get("fixed_z_minimum_used") is not False:
        raise ValueError("T4->T7 unexpectedly uses a fixed Z derivation")
    scan_summary = dict(scan["summary"])
    primary_summary = dict(primary["summary"])
    if scan_summary.get("episode_count") != 30 or (
        scan_summary.get("all_episodes_command_safe_in_full_window") is not True
    ):
        raise ValueError("T4->T7 latched scan is not command-safe for 30/30 episodes")
    return {
        "bridge_profile": profile.provenance_record(),
        "execution_tail": tail,
        "primary_window_replay_summary": primary_summary,
        "latched_scan_replay_summary": scan_summary,
    }


def main() -> None:
    args = _parse_args()
    base = args.base_registry.expanduser().resolve()
    overlay = args.verified_overlay.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if output.exists() and not args.force:
        raise FileExistsError(f"refusing to overwrite registry: {output}")
    evidence = _assert_offline_evidence(overlay.parent)

    prior_argv = sys.argv
    try:
        sys.argv = [
            "promote_a0509_t4_t7_execution_tail_level2.py",
            "--base-registry",
            str(base),
            "--verified-overlay",
            str(overlay),
            "--output",
            str(output),
            "--composition-id",
            COMPOSITION_ID,
        ]
        if args.force:
            sys.argv.append("--force")
        promote_execution_tail()
    finally:
        sys.argv = prior_argv

    value = _load(output)
    contract = dict(value.get("selection_contract", {}))
    tail_profile = _load(TAIL_PROFILE_CONFIG)
    contract["execution_tail_profile_id"] = tail_profile["profile_id"]
    contract["shared_execution_tail_handoff"] = {
        "profile_id": tail_profile["profile_id"],
        "profile_config": str(TAIL_PROFILE_CONFIG.resolve()),
        "profile_sha256": _sha256(TAIL_PROFILE_CONFIG),
        "selection_basis": tail_profile["selection_basis"],
        "fixed_bridge_geometry": False,
        "fixed_phase_window": False,
        "edge_specific_numeric_tuning": False,
        "bridge_generation_profile": evidence["bridge_profile"],
        "t4_t7_execution_tail": evidence["execution_tail"],
        "primary_window_replay_summary": evidence[
            "primary_window_replay_summary"
        ],
        "latched_scan_replay_summary": evidence[
            "latched_scan_replay_summary"
        ],
    }
    contract["regression_registry"] = str(base)
    contract["regression_ready"] = base.is_file()
    contract["robot_commands_published"] = 0
    contract["physical_validation_performed"] = False
    value["selection_contract"] = contract
    output.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    registry = EdgeRuntimeRegistry.load(output)
    print(
        "T4_T7_SHARED_FLEX_LEVEL2_V12_PROMOTED "
        f"edges={len(registry.edges)} output={output} "
        f"rollback={base} robot_commands_published=0"
    )


if __name__ == "__main__":
    main()
