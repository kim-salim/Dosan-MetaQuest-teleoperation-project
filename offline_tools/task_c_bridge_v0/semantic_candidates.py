"""Asymmetric semantic A-cut and B-entry candidate generation."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from .trajectory_states import CandidatePoint, SemanticState, Trajectory
from .velocity_estimation import (
    estimate_local_velocity,
    robust_representative_velocity,
)


@dataclass(frozen=True)
class SemanticWindow:
    anchor: str
    start_offset_frames: int
    end_offset_frames: int
    stride_frames: int
    semantic_label: str
    expected_gripper: str
    minimum_transport_clearance_mm: float = 0.0

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "SemanticWindow":
        return cls(**value)


def _transition_indices(trajectory: Trajectory, anchor: str) -> np.ndarray:
    before = trajectory.gripper_closed[:-1]
    after = trajectory.gripper_closed[1:]
    if anchor == "gripper_close":
        return np.flatnonzero((~before) & after) + 1
    if anchor == "gripper_open":
        return np.flatnonzero(before & (~after)) + 1
    if anchor == "closed_transport":
        # The paired close/open transitions are handled by generate_candidates.
        return np.empty(0, dtype=np.int64)
    raise ValueError(f"unsupported semantic anchor: {anchor}")


def _gripper_matches(closed: bool, expected: str) -> bool:
    if expected == "closed":
        return bool(closed)
    if expected == "open":
        return not bool(closed)
    if expected == "any":
        return True
    raise ValueError(f"unsupported expected_gripper value: {expected}")


def is_a_cut_valid(candidate: CandidatePoint, config: dict[str, Any]) -> bool:
    if candidate.role != "a_cut":
        return False
    allowed = set(config.get("a_allowed_labels", ()))
    return not allowed or candidate.semantic_label in allowed


def is_b_entry_valid(candidate: CandidatePoint, config: dict[str, Any]) -> bool:
    if candidate.role != "b_entry":
        return False
    allowed = set(config.get("b_allowed_labels", ()))
    return not allowed or candidate.semantic_label in allowed


def semantic_state_compatibility(
    a_state: SemanticState,
    b_state: SemanticState,
    config: dict[str, Any],
) -> tuple[bool, list[str], list[str]]:
    """Compare only state that a Cartesian bridge cannot change."""

    reasons: list[str] = []
    unchecked: list[str] = []
    if config.get("require_same_gripper", True):
        if a_state.gripper_closed != b_state.gripper_closed:
            reasons.append("gripper_state_mismatch")

    for field in ("holding", "contact_mode", "object_state"):
        left = getattr(a_state, field)
        right = getattr(b_state, field)
        if left is None or right is None:
            unchecked.append(field)
        elif left != right:
            reasons.append(f"{field}_mismatch")

    required_a = set(config.get("required_a_completed_subgoals", ()))
    if not required_a.issubset(a_state.completed_subgoals):
        reasons.append("a_required_subgoal_incomplete")

    available_conditions = set(a_state.completed_subgoals)
    available_conditions.update(config.get("runtime_satisfied_conditions", ()))
    required_entry = set(b_state.entry_preconditions)
    required_entry.update(config.get("required_b_entry_preconditions", ()))
    if not required_entry.issubset(available_conditions):
        reasons.append("b_entry_precondition_unsatisfied")
    return not reasons, reasons, unchecked


def semantic_compatibility(
    a: CandidatePoint,
    b: CandidatePoint,
    config: dict[str, Any],
) -> tuple[bool, list[str], list[str]]:
    """Return hard compatibility, failure reasons, and unchecked fields."""

    return semantic_state_compatibility(
        a.semantic_state,
        b.semantic_state,
        config,
    )


def is_semantically_compatible(
    a: CandidatePoint,
    b: CandidatePoint,
    config: dict[str, Any],
) -> bool:
    return semantic_compatibility(a, b, config)[0]


def generate_candidates(
    trajectories: list[Trajectory],
    *,
    role: str,
    windows: list[SemanticWindow],
    velocity_config: dict[str, Any],
) -> list[CandidatePoint]:
    """Generate candidates solely from semantic boundaries/windows.

    Speed and low-speed minima are intentionally absent from this function.
    """

    if role not in {"a_cut", "b_entry"}:
        raise ValueError(f"unsupported candidate role: {role}")
    output: list[CandidatePoint] = []
    seen: set[tuple[str, int, int, str]] = set()
    for trajectory in trajectories:
        for window in windows:
            if window.anchor == "closed_transport":
                close_indices = _transition_indices(trajectory, "gripper_close")
                open_indices = _transition_indices(trajectory, "gripper_open")
                spans = [
                    (int(close), int(open_index))
                    for close in close_indices
                    for open_index in open_indices
                    if open_index > close
                ]
                # Pair each close with its first following open only.
                spans = [
                    (close, min(open_index for c, open_index in spans if c == close))
                    for close in sorted({close for close, _ in spans})
                ]
            else:
                spans = [
                    (int(anchor), int(anchor))
                    for anchor in _transition_indices(trajectory, window.anchor)
                ]

            for start_anchor, end_anchor in spans:
                first = int(start_anchor + window.start_offset_frames)
                last = int(end_anchor + window.end_offset_frames)
                if last < first:
                    continue
                minimum_bridge_z_mm: float | None = None
                if window.anchor == "closed_transport":
                    event_height = max(
                        float(trajectory.xyz_mm[start_anchor, 2]),
                        float(trajectory.xyz_mm[end_anchor, 2]),
                    )
                    minimum_bridge_z_mm = (
                        event_height + window.minimum_transport_clearance_mm
                    )
                for index in range(first, last + 1, window.stride_frames):
                    if index < 0 or index >= len(trajectory.xyz_mm):
                        continue
                    if not _gripper_matches(
                        trajectory.gripper_closed[index], window.expected_gripper
                    ):
                        continue
                    if (
                        minimum_bridge_z_mm is not None
                        and trajectory.xyz_mm[index, 2] < minimum_bridge_z_mm
                    ):
                        continue
                    key = (trajectory.dataset, trajectory.episode, index, window.semantic_label)
                    if key in seen:
                        continue
                    seen.add(key)
                    phase_start = start_anchor if end_anchor > start_anchor else first
                    phase_stop = end_anchor if end_anchor > start_anchor else last
                    semantic_phase = (
                        float(np.clip((index - phase_start) / (phase_stop - phase_start), 0.0, 1.0))
                        if phase_stop > phase_start else 0.0
                    )
                    completed = ()
                    if window.anchor == "gripper_open" and index >= start_anchor:
                        completed = ("gripper_close_then_open_complete",)
                    elif window.anchor == "closed_transport":
                        completed = ("grasp_complete",)
                    state = SemanticState(
                        gripper_closed=bool(trajectory.gripper_closed[index]),
                        holding=True if window.anchor == "closed_transport" else None,
                        contact_mode=(
                            "free_transport_assumed"
                            if window.anchor == "closed_transport"
                            else None
                        ),
                        completed_subgoals=completed,
                    )
                    velocity = estimate_local_velocity(
                        trajectory.xyz_mm,
                        trajectory.timestamp_s,
                        index,
                        window_frames=int(velocity_config["velocity_window_frames"]),
                        method=str(velocity_config["velocity_smoothing_method"]),
                        velocity_epsilon=float(velocity_config["velocity_epsilon"]),
                    )
                    output.append(
                        CandidatePoint(
                            role=role,  # type: ignore[arg-type]
                            trajectory=trajectory,
                            index=index,
                            semantic_label=window.semantic_label,
                            semantic_state=state,
                            velocity_mm_s=velocity,
                            velocity_method=str(velocity_config["velocity_smoothing_method"]),
                            provenance=(
                                "closed_between_grasp_and_release_with_clearance"
                                if window.anchor == "closed_transport"
                                else "gripper_semantic_boundary"
                            ),
                            minimum_bridge_z_mm=minimum_bridge_z_mm,
                            holding_assumption=(
                                "gripper_closed_implies_holding"
                                if window.anchor == "closed_transport"
                                else "UNKNOWN"
                            ),
                            semantic_phase=semantic_phase,
                        )
                    )
    return output


def apply_representative_b_velocities(
    candidates: list[CandidatePoint],
    config: dict[str, Any] | None,
) -> list[CandidatePoint]:
    """Replace per-demo B velocity with a phase-local robust representative.

    Position, episode, frame, orientation payload, and every other source field
    remain unchanged. Each episode contributes one median vector to a phase
    group so episodes with more candidates cannot dominate the representative.
    """

    values = list(candidates)
    if not config or not bool(config.get("enabled", False)):
        return values
    if any(candidate.role != "b_entry" for candidate in values):
        raise ValueError("representative B velocity accepts only b_entry candidates")

    phase_bin_width = float(config.get("phase_bin_width", 0.1))
    minimum_demonstrations = int(config.get("minimum_demonstrations", 3))
    method = str(config.get("method", "component_median"))
    trim_fraction = float(config.get("trim_fraction", 0.1))
    if not 0.0 < phase_bin_width <= 1.0:
        raise ValueError("phase_bin_width must be in (0, 1]")
    if minimum_demonstrations < 1:
        raise ValueError("minimum_demonstrations must be positive")

    groups: dict[tuple[str, int], list[CandidatePoint]] = {}
    for candidate in values:
        phase = 0.0 if candidate.semantic_phase is None else candidate.semantic_phase
        phase_bin = min(int(np.floor(phase / phase_bin_width)), int(1.0 / phase_bin_width))
        groups.setdefault((candidate.semantic_label, phase_bin), []).append(candidate)

    replacements: dict[int, CandidatePoint] = {}
    for group in groups.values():
        by_episode: dict[int, list[np.ndarray]] = {}
        for candidate in group:
            by_episode.setdefault(candidate.trajectory.episode, []).append(
                candidate.velocity_mm_s
            )
        if len(by_episode) < minimum_demonstrations:
            continue
        episode_velocities = np.stack(
            [
                np.median(np.stack(vectors, axis=0), axis=0)
                for _, vectors in sorted(by_episode.items())
            ],
            axis=0,
        )
        representative = robust_representative_velocity(
            episode_velocities,
            method=method,
            trim_fraction=trim_fraction,
        )
        source_episodes = tuple(sorted(by_episode))
        for candidate in group:
            replacements[id(candidate)] = replace(
                candidate,
                velocity_mm_s=representative,
                velocity_method=f"representative_{method}",
                provenance=f"{candidate.provenance}+phase_representative_velocity",
                velocity_sample_count=len(source_episodes),
                velocity_source_episodes=source_episodes,
            )
    return [replacements.get(id(candidate), candidate) for candidate in values]
