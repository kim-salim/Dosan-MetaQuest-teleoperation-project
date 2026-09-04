#!/usr/bin/env python3
"""Replay a profiled Bridge over a recorded source bank without robot I/O."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import Any, Mapping

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "src/lerobot_robot_doosan_a0509"
QUEST_PACKAGE_ROOT = REPOSITORY_ROOT / "src/quest_a0509_teleop"
for value in (
    str(REPOSITORY_ROOT),
    str(PACKAGE_ROOT),
    str(QUEST_PACKAGE_ROOT),
):
    if value not in sys.path:
        sys.path.insert(0, value)

from lerobot_robot_doosan_a0509.task_c_handoff.bridge_profile import (  # noqa: E402
    load_bridge_generation_profile,
    validate_profile_against_validation_config,
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
    runtime_limits_from_validation,
)
from quest_a0509_teleop.doosan_orientation import (  # noqa: E402
    quaternion_to_doosan_zyz_deg,
)


DEFAULT_PROFILE_CONFIG = (
    REPOSITORY_ROOT
    / "config/realtime/a0509_flexible_bridge_profiles_v1.json"
)
DEFAULT_VALIDATION_CONFIG = (
    REPOSITORY_ROOT
    / "offline_tools/cross_task_handoff/validation_config_a0509_v2.json"
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--source-bank", type=Path, default=None)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--source-reference-mode",
        choices=("execution_tail", "exact_semantic_exit"),
        default="execution_tail",
    )
    parser.add_argument(
        "--bridge-profile-config",
        type=Path,
        default=DEFAULT_PROFILE_CONFIG,
    )
    parser.add_argument("--bridge-profile-id", default=None)
    parser.add_argument(
        "--validation-config",
        type=Path,
        default=DEFAULT_VALIDATION_CONFIG,
    )
    parser.add_argument("--max-candidates", type=int, default=64)
    parser.add_argument("--max-search-time-s", type=float, default=0.20)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.expanduser().resolve().read_text(encoding="utf-8"))


def _write(path: Path, value: Mapping[str, Any], *, force: bool) -> None:
    resolved = path.expanduser().resolve()
    if resolved.exists() and not force:
        raise FileExistsError(f"refusing to overwrite replay: {resolved}")
    resolved.parent.mkdir(parents=True, exist_ok=True)
    resolved.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _sample_snapshot(
    sample: Mapping[str, Any],
    manifest: EpisodeHandoffManifest,
) -> BridgeRuntimeSnapshot:
    orientation = quaternion_to_doosan_zyz_deg(
        sample["orientation_quat_xyzw"],
        [0.0, 150.0, 0.0],
    )
    pose = np.concatenate(
        (
            np.asarray(sample["position_mm"], dtype=np.float64),
            orientation,
        )
    )
    return BridgeRuntimeSnapshot(
        timestamp_s=time.monotonic(),
        actual_pose_mm_deg=pose,
        acknowledged_pose_mm_deg=pose,
        actual_velocity_mm_s=np.asarray(
            sample["velocity_mm_s"], dtype=np.float64
        ),
        gripper_target=manifest.source.semantic.gripper_target,
    )


def _compact_geometry(
    *,
    episode: int,
    sample: Mapping[str, Any],
    geometry: Any,
) -> dict[str, Any]:
    record = geometry.record()
    selected = record.get("selected")
    metrics = None
    if isinstance(selected, Mapping):
        metrics = {
            name: selected.get(name)
            for name in (
                "generator_type",
                "duration_s",
                "max_ack_span_axis_step_mm",
                "max_ack_span_orientation_step_deg",
                "max_command_acceleration_mm_s2",
                "max_command_jerk_mm_s3",
                "max_orientation_step_deg",
                "max_position_axis_step_mm",
                "max_velocity_mm_s",
                "minimum_position_z_mm",
                "tangent_handle_chord_ratio",
            )
        }
    return {
        "episode": episode,
        "phase": float(sample["phase"]),
        "source_max_axis_speed_mm_s": max(
            abs(float(item)) for item in sample["velocity_mm_s"]
        ),
        "valid": bool(record["valid"]),
        "timed_out": bool(record["timed_out"]),
        "candidates_evaluated": int(record["candidates_evaluated"]),
        "candidates_hard_passed": int(record["candidates_hard_passed"]),
        "rejected_reason_counts": dict(record["rejected_reason_counts"]),
        "selected": metrics,
    }


def main() -> None:
    args = _parse_args()
    manifest_path = args.manifest.expanduser().resolve()
    manifest_raw = _load(manifest_path)
    manifest = EpisodeHandoffManifest.from_mapping(manifest_raw)
    profile = load_bridge_generation_profile(
        args.bridge_profile_config,
        source_reference_mode=args.source_reference_mode,
        profile_id=args.bridge_profile_id,
    )
    declared_profile = dict(
        manifest_raw.get("generation", {}).get("bridge_profile", {})
    )
    if declared_profile.get("profile_id") != profile.profile_id or (
        declared_profile.get("profile_sha256") != profile.profile_sha256
    ):
        raise ValueError("manifest Bridge profile provenance is stale")
    if manifest.bridge_algorithm != profile.bridge_algorithm:
        raise ValueError("manifest Bridge algorithm differs from profile")
    validation = _load(args.validation_config)
    validate_profile_against_validation_config(profile, validation)
    limits = runtime_limits_from_validation(validation)
    runtime = HandoffV2Config(
        enabled=True,
        control_hz=float(profile.runtime_contract["control_hz"]),
        bridge_admission_mode="flexible_level2",
        adaptive_b_max_splice_index=8,
        adaptive_b_max_candidates=6,
        semantic_authority="external_planner",
    )
    search = FlexibleBridgeSearchConfig(
        max_candidates=args.max_candidates,
        max_search_time_s=args.max_search_time_s,
    )
    source_bank_path = args.source_bank
    if source_bank_path is None:
        if manifest.runtime_source_bank_path is None:
            raise ValueError("manifest does not declare a runtime source bank")
        source_bank_path = manifest_path.parent / manifest.runtime_source_bank_path
    source_bank = _load(source_bank_path)
    nominal_phase = float(source_bank["nominal_phase"])

    episode_records: list[dict[str, Any]] = []
    started = time.perf_counter()
    for item in source_bank["episodes"]:
        episode = int(item["episode"])
        attempts: list[dict[str, Any]] = []
        first_valid = None
        first_valid_in_nominal_prefix = None
        for sample in sorted(item["samples"], key=lambda value: value["phase"]):
            geometry = search_flexible_bridge_queue(
                manifest,
                _sample_snapshot(sample, manifest),
                runtime,
                limits,
                search,
                live_mode=False,
            )
            compact = _compact_geometry(
                episode=episode,
                sample=sample,
                geometry=geometry,
            )
            attempts.append(compact)
            if compact["valid"] and first_valid is None:
                first_valid = compact
            if (
                compact["valid"]
                and compact["phase"] <= nominal_phase + 1.0e-9
                and first_valid_in_nominal_prefix is None
            ):
                first_valid_in_nominal_prefix = compact
            if first_valid_in_nominal_prefix is not None:
                break
            if first_valid is not None and compact["phase"] > nominal_phase:
                break
        episode_records.append(
            {
                "episode": episode,
                "valid_in_nominal_phase_prefix": (
                    first_valid_in_nominal_prefix is not None
                ),
                "valid_in_full_source_window": first_valid is not None,
                "first_valid_phase": (
                    None if first_valid is None else first_valid["phase"]
                ),
                "first_valid_generator_type": (
                    None
                    if first_valid is None
                    else first_valid["selected"]["generator_type"]
                ),
                "attempts": attempts,
            }
        )

    total = len(episode_records)
    prefix_pass = sum(
        item["valid_in_nominal_phase_prefix"] for item in episode_records
    )
    full_pass = sum(
        item["valid_in_full_source_window"] for item in episode_records
    )
    report = {
        "schema_version": "a0509.profiled_bridge_source_bank_replay.v1",
        "manifest": str(manifest_path),
        "source_bank": str(Path(source_bank_path).expanduser().resolve()),
        "bridge_generation_profile": profile.provenance_record(),
        "source_nominal_phase": nominal_phase,
        "source_phase_window": list(source_bank["phase_window"]),
        "search_contract": {
            "actual_equals_acknowledged_pose": True,
            "recorded_position_orientation_velocity_used": True,
            "max_candidates": args.max_candidates,
            "max_search_time_s": args.max_search_time_s,
            "live_mode": False,
        },
        "summary": {
            "episode_count": total,
            "valid_in_nominal_phase_prefix_count": prefix_pass,
            "valid_in_nominal_phase_prefix_fraction": prefix_pass / total,
            "valid_in_full_source_window_count": full_pass,
            "valid_in_full_source_window_fraction": full_pass / total,
            "all_episodes_command_safe_in_full_window": full_pass == total,
        },
        "episodes": episode_records,
        "elapsed_s": time.perf_counter() - started,
        "robot_commands_published": 0,
        "live_enabled": False,
        "mux_selected": False,
        "physical_validation_performed": False,
    }
    _write(args.output, report, force=args.force)
    print(
        "PROFILED_BRIDGE_SOURCE_BANK_REPLAY_COMPLETE "
        f"prefix={prefix_pass}/{total} full={full_pass}/{total} "
        f"output={args.output.expanduser().resolve()} "
        "robot_commands_published=0"
    )
    if full_pass != total:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
