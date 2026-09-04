"""Stable JSON records for candidate audit artifacts."""

from __future__ import annotations

from typing import Any

import numpy as np

from .bridge_optimizer import CandidateEvaluation
from .trajectory_states import SemanticState


def _vector(value: np.ndarray) -> list[float]:
    return [float(item) for item in np.asarray(value).tolist()]


def _semantic_state(value: SemanticState) -> dict[str, Any]:
    return {
        "gripper_closed": bool(value.gripper_closed),
        "holding": value.holding,
        "contact_mode": value.contact_mode,
        "object_state": value.object_state,
        "completed_subgoals": list(value.completed_subgoals),
        "entry_preconditions": list(value.entry_preconditions),
    }


def candidate_record(candidate: CandidateEvaluation) -> dict[str, Any]:
    a = candidate.a
    b = candidate.b
    bridge = candidate.bridge
    metrics = candidate.metrics
    return {
        "a_dataset": a.trajectory.dataset,
        "a_timestamp_s": a.timestamp_s,
        "a_semantic_phase": a.semantic_phase,
        "a_velocity_method": a.velocity_method,
        "a_velocity_sample_count": a.velocity_sample_count,
        "a_velocity_source_episodes": list(a.velocity_source_episodes),
        "a_semantic_state": _semantic_state(a.semantic_state),
        "a_episode": a.trajectory.episode,
        "a_frame": a.frame,
        "a_subgoal": a.semantic_label,
        "a_position_mm": _vector(a.position_mm),
        "a_velocity_mm_s": _vector(a.velocity_mm_s),
        "a_holding_assumption": a.holding_assumption,
        "a_minimum_bridge_z_mm": a.minimum_bridge_z_mm,
        "b_dataset": b.trajectory.dataset,
        "b_timestamp_s": b.timestamp_s,
        "b_semantic_phase": b.semantic_phase,
        "b_velocity_method": b.velocity_method,
        "b_velocity_sample_count": b.velocity_sample_count,
        "b_velocity_source_episodes": list(b.velocity_source_episodes),
        "b_semantic_state": _semantic_state(b.semantic_state),
        "b_episode": b.trajectory.episode,
        "b_frame": b.frame,
        "b_subgoal": b.semantic_label,
        "b_position_mm": _vector(b.position_mm),
        "b_velocity_mm_s": _vector(b.velocity_mm_s),
        "b_holding_assumption": b.holding_assumption,
        "b_minimum_bridge_z_mm": b.minimum_bridge_z_mm,
        "bridge_duration_s": bridge.duration_s,
        "P0": _vector(bridge.p0),
        "P1": _vector(bridge.p1),
        "P2": _vector(bridge.p2),
        "P3": _vector(bridge.p3),
        "A_retained_length_mm": candidate.a_retained_length_mm,
        "bridge_length_mm": metrics.length_mm,
        "B_retained_length_mm": candidate.b_retained_length_mm,
        "total_C_length_mm": candidate.total_c_length_mm,
        "max_velocity_mm_s": metrics.max_velocity_mm_s,
        "max_acceleration_mm_s2": metrics.max_acceleration_mm_s2,
        "max_curvature_per_mm": metrics.max_curvature_per_mm,
        "max_jerk_mm_s3": metrics.max_jerk_mm_s3,
        "integrated_squared_jerk": metrics.integrated_squared_jerk,
        "backtracking_ratio": metrics.backtracking_ratio,
        "semantic_compatible": candidate.semantic_compatible,
        "semantic_unchecked": candidate.semantic_unchecked,
        "bridge_gripper_closed": bool(a.semantic_state.gripper_closed),
        "payload_state": (
            "A_OBJECT_HELD_ASSUMED"
            if a.semantic_state.holding is True
            else "NO_PAYLOAD_ASSUMED"
        ),
        "feasible": candidate.feasible,
        "failure_reasons": candidate.failure_reasons,
        "orientation_status": "pending",
        "orientation_bridge_generated": False,
        "collision_status": (
            "NOT_CHECKED_WITH_PAYLOAD"
            if a.semantic_state.holding is True
            else "NOT_CHECKED"
        ),
        "ik_status": "NOT_CHECKED",
        "robot_executable": False,
        "dry_run_only": True,
    }


def runtime_entry_record(candidate: CandidateEvaluation) -> dict[str, Any]:
    """Serialize one distinct B entry selected from feasible offline search."""

    b = candidate.b
    return {
        "entry_id": (
            f"{b.trajectory.dataset}:ep{b.trajectory.episode}:"
            f"f{b.frame}:{b.semantic_label}"
        ),
        "dataset": b.trajectory.dataset,
        "episode": b.trajectory.episode,
        "frame": b.frame,
        "timestamp_s": b.timestamp_s,
        "semantic_label": b.semantic_label,
        "semantic_phase": b.semantic_phase,
        "semantic_state": _semantic_state(b.semantic_state),
        "position_mm": _vector(b.position_mm),
        "demonstration_velocity_mm_s": _vector(b.velocity_mm_s),
        "velocity_method": b.velocity_method,
        "velocity_sample_count": b.velocity_sample_count,
        "velocity_source_episodes": list(b.velocity_source_episodes),
        "B_retained_length_mm": candidate.b_retained_length_mm,
        "minimum_bridge_z_mm": b.minimum_bridge_z_mm,
        "offline_reference_total_C_length_mm": candidate.total_c_length_mm,
        "orientation_status": "pending",
        "orientation_bridge_generated": False,
        "collision_status": "NOT_CHECKED_WITH_PAYLOAD",
        "ik_status": "NOT_CHECKED",
        "robot_executable": False,
        "dry_run_only": True,
    }


def runtime_candidate_record(candidate: Any) -> dict[str, Any]:
    """Serialize a RuntimeBridgeCandidate without importing policy code."""

    bridge = candidate.bridge
    metrics = candidate.metrics
    return {
        "entry_id": candidate.entry.entry_id,
        "b_dataset": candidate.entry.dataset,
        "b_episode": candidate.entry.episode,
        "b_frame": candidate.entry.frame,
        "b_semantic_label": candidate.entry.semantic_label,
        "b_position_mm": _vector(candidate.entry.position_mm),
        "b_demonstration_velocity_mm_s": _vector(
            candidate.entry.demonstration_velocity_mm_s
        ),
        "b_terminal_velocity_mm_s": _vector(candidate.terminal_velocity_mm_s),
        "terminal_velocity_source": candidate.terminal_velocity_source,
        "bridge_duration_s": bridge.duration_s,
        "P0": _vector(bridge.p0),
        "P1": _vector(bridge.p1),
        "P2": _vector(bridge.p2),
        "P3": _vector(bridge.p3),
        "A_retained_length_mm": candidate.a_retained_length_mm,
        "committed_bridge_prefix_length_mm": (
            candidate.committed_bridge_prefix_length_mm
        ),
        "bridge_length_mm": metrics.length_mm,
        "B_retained_length_mm": candidate.entry.b_retained_length_mm,
        "total_C_estimate_mm": candidate.total_c_estimate_mm,
        "max_velocity_mm_s": metrics.max_velocity_mm_s,
        "max_acceleration_mm_s2": metrics.max_acceleration_mm_s2,
        "max_curvature_per_mm": metrics.max_curvature_per_mm,
        "max_jerk_mm_s3": metrics.max_jerk_mm_s3,
        "integrated_squared_jerk": metrics.integrated_squared_jerk,
        "backtracking_ratio": metrics.backtracking_ratio,
        "workspace_satisfied": metrics.workspace_satisfied,
        "semantic_unchecked": list(candidate.semantic_unchecked),
        "feasible": candidate.feasible,
        "failure_reasons": list(candidate.failure_reasons),
        "orientation_status": "pending",
        "orientation_bridge_generated": False,
        "collision_status": "NOT_CHECKED_WITH_PAYLOAD",
        "ik_status": "NOT_CHECKED",
        "robot_executable": False,
        "dry_run_only": True,
    }
