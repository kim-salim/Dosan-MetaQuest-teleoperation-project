#!/usr/bin/env python3
"""Promote the immutable v12 edge set to the opt-in 8.5 mm/tick v13 profile.

This is a metadata-only, command-free promotion.  Existing edge evidence stays
bound to its stricter 7.5 mm/tick validation provenance, while every newly
compiled plan selects the v13 runtime profile and its prospective ACK-span
revalidation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = PROJECT_ROOT / "src/lerobot_robot_doosan_a0509"
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "src/quest_a0509_teleop"))

from lerobot_robot_doosan_a0509.task_c_handoff.runtime_command_profile import (  # noqa: E402
    LEGACY_RUNTIME_COMMAND_PROFILE_ID,
    RAMP8P5_RUNTIME_COMMAND_PROFILE_ID,
    get_runtime_command_profile,
)


ARTIFACT_ROOT = (
    PROJECT_ROOT
    / "docs/artifacts/t1_t8_spatial_floor_level2_semantic_local_all_2026-09-03"
)
DEFAULT_BASE = ARTIFACT_ROOT / "edge_registry_level2_spatial_v12.json"
DEFAULT_OUTPUT = ARTIFACT_ROOT / "edge_registry_level2_spatial_v13.json"
PROFILE_CONFIG = (
    PROJECT_ROOT / "config/realtime/a0509_runtime_command_profiles_v1.json"
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def promote(base: Path, output: Path) -> Path:
    base = base.expanduser().resolve()
    output = output.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite v13 registry: {output}")
    value = json.loads(base.read_text(encoding="utf-8"))
    selection = dict(value["selection_contract"])
    profile = get_runtime_command_profile(RAMP8P5_RUNTIME_COMMAND_PROFILE_ID)
    selection.update(
        {
            "base_registry_read_only": str(base),
            "base_registry_sha256": _sha256(base),
            "regression_registry": str(base),
            "runtime_command_profile_id": profile.profile_id,
            "runtime_command_profile": profile.to_record(),
            "runtime_command_profile_config": str(PROFILE_CONFIG),
            "runtime_command_profile_config_sha256": _sha256(PROFILE_CONFIG),
            "offline_edge_evidence_command_profile_id": (
                LEGACY_RUNTIME_COMMAND_PROFILE_ID
            ),
            "runtime_ack_span_revalidation": (
                "mandatory_for_bridge_and_soft_crossfade_before_emission"
            ),
            "promotion_scope": (
                "runtime_command_contract_only_edges_and_dijkstra_costs_unchanged"
            ),
            "physical_validation_performed": False,
            "robot_commands_published": 0,
            "regression_ready": True,
        }
    )
    value["selection_contract"] = selection
    value["composition_id"] = (
        "a0509_t1_t8_spatial_floor_shared_profile_ramp8p5_"
        "level2_v13_20260904"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    return output


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=Path, default=DEFAULT_BASE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    print(promote(args.base, args.output))


if __name__ == "__main__":
    main()
