"""Generate the setup-only support manifest required by Multi-V2 web runs."""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from lerobot_robot_doosan_a0509.task_c_handoff.multi_stage import MultiStagePlan
from lerobot_robot_doosan_a0509.task_c_handoff.runtime_command_profile import (
    LEGACY_RUNTIME_COMMAND_PROFILE_ID,
    RuntimeCommandProfile,
    get_runtime_command_profile,
)


_LEGACY_WEB_PROFILE = get_runtime_command_profile(
    LEGACY_RUNTIME_COMMAND_PROFILE_ID
)
WEB_TASK_C_COMMAND_ACCELERATION_LIMIT_MM_S2 = (
    _LEGACY_WEB_PROFILE.acceleration_limit_mm_s2
)
WEB_TASK_C_AXIS_VELOCITY_LIMIT_MM_S = (
    _LEGACY_WEB_PROFILE.axis_velocity_limit_mm_s
)
WEB_TASK_C_JERK_LIMIT_MM_S3 = _LEGACY_WEB_PROFILE.jerk_limit_mm_s3
WEB_TASK_C_INTEGRATED_SQUARED_JERK_LIMIT = (
    _LEGACY_WEB_PROFILE.integrated_squared_jerk_limit
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_new_json(path: Path, value: Mapping[str, Any]) -> Path:
    output = path.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    return output


def build_web_runtime_support_manifest(
    *,
    multi_stage_plan_path: str | Path,
    template_path: str | Path,
    output_path: str | Path,
    command_profile: RuntimeCommandProfile | None = None,
) -> Path:
    """Derive a checkpoint-correct setup shell from the first reviewed edge.

    The inherited Task-C setup constructs an inert V1 coordinator before the
    Multi-V2 strategy binds its exact episode edge.  The generated manifest is
    only for that setup phase; it stays explicitly dry-run-only and never
    claims IK, collision, or physical certification.
    """

    profile = command_profile or _LEGACY_WEB_PROFILE
    plan_path = Path(multi_stage_plan_path).expanduser().resolve()
    plan = MultiStagePlan.load(plan_path)
    if (
        plan.runtime_command_profile_id is not None
        and plan.runtime_command_profile_id != profile.profile_id
    ):
        raise ValueError(
            "support manifest command profile differs from multi-stage plan"
        )
    template_source = Path(template_path).expanduser().resolve()
    template = json.loads(template_source.read_text(encoding="utf-8"))
    manifest = copy.deepcopy(template)
    first_edge = plan.transitions[0].handoff_manifest
    source = first_edge.source
    successor = first_edge.successor
    first_policy = plan.initial_policy
    first_successor = plan.policy(plan.stages[1].policy_id)
    source_semantic = source.semantic
    successor_semantic = successor.semantic
    source_speed = float(np.linalg.norm(source.nominal_velocity_mm_s))
    if source_speed <= 1.0e-9:
        raise ValueError("first Multi-V2 source boundary has zero nominal speed")

    manifest.update(
        {
            "schema_version": "a0509.task_c_v2_support_runtime.v1",
            "composition_id": plan.composition_id,
            "handoff_id": first_edge.handoff_id,
            "mode": "representative_boundary_dry_run_design",
            "purpose": (
                "Setup-only support shell; actual transitions are owned by "
                "the exact planner-compiled Multi-V2 episode manifests."
            ),
            "policy_checkpoints": {
                "ACT-A": str(first_policy.checkpoint),
                "ACT-B": str(first_successor.checkpoint),
            },
            "b_entries": [
                {
                    "entry_id": (
                        f"web:{first_edge.handoff_id}:{successor.task}:"
                        f"ep{successor.support_episode}:f{successor.support_frame}"
                    ),
                    "dataset": successor.task,
                    "episode": successor.support_episode,
                    "frame": successor.support_frame,
                    "semantic_label": (
                        successor_semantic.semantic_state
                        or f"{successor.task}_{successor.segment}"
                    ),
                    "semantic_state": {
                        "gripper_closed": (
                            successor_semantic.gripper_state == "closed"
                        ),
                        "holding": (
                            successor_semantic.held_object
                            not in {"none", "unknown", ""}
                        ),
                        "contact_mode": successor_semantic.contact_mode,
                        "completed_subgoals": list(
                            successor_semantic.entry_preconditions
                        ),
                        "object_state": successor_semantic.semantic_state,
                        "entry_preconditions": list(
                            successor_semantic.entry_preconditions
                        ),
                    },
                    "position_mm": successor.nominal_position_mm.tolist(),
                    "demonstration_velocity_mm_s": (
                        successor.nominal_velocity_mm_s.tolist()
                    ),
                    "B_retained_length_mm": 0.0,
                    "minimum_bridge_z_mm": first_edge.transport_floor_mm,
                    "velocity_method": "episode_manifest_nominal",
                    "velocity_sample_count": 1,
                    "velocity_source_episodes": [successor.support_episode],
                    "robot_executable": False,
                    "dry_run_only": True,
                    "ik_status": "NOT_CHECKED",
                    "collision_status": "NOT_CHECKED",
                }
            ],
            "representative_boundary": {
                "enabled": True,
                "role": "a_exit",
                "boundary_id": f"web:{first_edge.handoff_id}:source_exit",
                "center_position_mm": source.nominal_position_mm.tolist(),
                "representative_velocity_mm_s": (
                    source.nominal_velocity_mm_s.tolist()
                ),
                "representative_tangent": (
                    source.nominal_velocity_mm_s / source_speed
                ).tolist(),
                "prearm_radius_mm": source.prearm_radius_mm,
                "commit_radius_mm": source.commit_radius_mm,
                "direction_cosine_minimum": source.direction_cosine_minimum,
                "approach_frames": source.approach_frames,
                "commit_stable_frames": source.commit_stable_frames,
                "approach_epsilon_mm": 0.05,
            },
            "representative_paths": {
                "single_representative_per_task": True,
                "A": {
                    "dataset": source.task,
                    "segment": source.segment,
                    "phase": source.phase,
                    "support_episode": source.support_episode,
                    "support_frame": source.support_frame,
                    "role": "a_exit",
                },
                "B": {
                    "dataset": successor.task,
                    "segment": successor.segment,
                    "phase": successor.phase,
                    "support_episode": successor.support_episode,
                    "support_frame": successor.support_frame,
                    "role": "b_entry",
                },
            },
        }
    )

    planners = dict(manifest["planners"])
    semantic = dict(planners["semantic_compatibility"])
    semantic.update(
        {
            "a_allowed_labels": [
                source_semantic.semantic_state
                or f"{source.task}_{source.segment}"
            ],
            "b_allowed_labels": [
                successor_semantic.semantic_state
                or f"{successor.task}_{successor.segment}"
            ],
            "require_same_gripper": (
                source_semantic.gripper_state
                == successor_semantic.gripper_state
            ),
            "required_a_completed_subgoals": list(
                source_semantic.entry_preconditions
            ),
        }
    )
    planners["semantic_compatibility"] = semantic
    feasibility = dict(planners["feasibility"])
    feasibility["enforce_payload_transport_floor"] = (
        first_edge.transport_floor_mm is not None
    )
    feasibility["velocity_limit_mm_s"] = profile.cartesian_velocity_limit_mm_s
    feasibility["acceleration_limit_mm_s2"] = profile.acceleration_limit_mm_s2
    feasibility["axis_velocity_limit_mm_s"] = profile.axis_velocity_limit_mm_s
    feasibility["jerk_limit_mm_s3"] = profile.jerk_limit_mm_s3
    feasibility["integrated_squared_jerk_limit"] = (
        profile.integrated_squared_jerk_limit
    )
    if "boundary_acceleration_jump_limit_mm_s2" in feasibility:
        feasibility["boundary_acceleration_jump_limit_mm_s2"] = (
            profile.acceleration_limit_mm_s2
        )
    planners["feasibility"] = feasibility
    manifest["planners"] = planners

    safety = dict(manifest["safety"])
    safety.update(
        {
            "robot_executable": False,
            "dry_run_only": True,
            "publish_robot_commands": False,
            "live_authority": "external_gate_only",
            "endpoint_fallback_enabled": True,
            "gripper_bridge_state": source_semantic.gripper_state,
            "holding_state": source_semantic.held_object,
            "ik_status": "NOT_CHECKED",
            "collision_status": "NOT_CHECKED",
        }
    )
    manifest["safety"] = safety
    manifest["web_generation"] = {
        "template": str(template_source),
        "template_sha256": _sha256(template_source),
        "multi_stage_plan": str(plan_path),
        "first_edge_handoff_id": first_edge.handoff_id,
        "setup_only": True,
        "runtime_command_profile": profile.to_record(),
        "physical_validation_performed": False,
    }
    return _write_new_json(Path(output_path), manifest)
