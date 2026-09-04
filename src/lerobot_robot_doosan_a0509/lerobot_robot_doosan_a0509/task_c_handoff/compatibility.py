"""Outer semantic and inner ACT-B prefix admission for Task-C V2."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

import numpy as np

from offline_tools.cross_task_handoff.authority import (
    SemanticAuthority,
    parse_semantic_authority,
)
from offline_tools.task_c_bridge_v0.live_transition import pose_delta_metrics
from offline_tools.task_c_bridge_v0.runtime_policy import PolicyChunk
from quest_a0509_teleop.doosan_orientation import (
    doosan_zyz_deg_to_quaternion,
    quaternion_angle_deg,
)

from .models import (
    BoundarySemanticState,
    EpisodeHandoffManifest,
    HandoffCompatibilityConfig,
)
from .soft_handoff import SoftHandoffPlan


@dataclass(frozen=True)
class PrefixDynamics:
    prefix_steps: int
    first_xyz_axis_delta_mm: np.ndarray
    first_xyz_delta_mm: float
    first_rotation_delta_deg: float
    max_velocity_mm_s: float
    max_axis_velocity_mm_s: float
    max_acceleration_mm_s2: float
    bridge_prefix_velocity_mismatch_mm_s: float
    initial_b_velocity_mm_s: np.ndarray

    def __post_init__(self) -> None:
        first_axis = np.asarray(self.first_xyz_axis_delta_mm, dtype=np.float64)
        velocity = np.asarray(self.initial_b_velocity_mm_s, dtype=np.float64)
        if first_axis.shape != (3,) or velocity.shape != (3,):
            raise ValueError("prefix dynamics vectors must be XYZ")
        if not np.all(np.isfinite(first_axis)) or not np.all(np.isfinite(velocity)):
            raise ValueError("prefix dynamics vectors must be finite")
        object.__setattr__(self, "first_xyz_axis_delta_mm", first_axis.copy())
        object.__setattr__(self, "initial_b_velocity_mm_s", velocity.copy())

    def record(self) -> dict[str, Any]:
        value = asdict(self)
        value["first_xyz_axis_delta_mm"] = self.first_xyz_axis_delta_mm.tolist()
        value["initial_b_velocity_mm_s"] = self.initial_b_velocity_mm_s.tolist()
        return value


@dataclass(frozen=True)
class HandoffExecutionDynamics:
    """Dynamics of commands that would actually be emitted during takeover."""

    crossfade_steps: int
    continuation_steps: int
    first_xyz_axis_step_mm: np.ndarray
    max_xyz_axis_step_mm: float
    first_rotation_step_deg: float
    max_rotation_step_deg: float
    acknowledged_command_span_steps: int
    max_acknowledged_xyz_axis_span_mm: float
    max_acknowledged_rotation_span_deg: float
    max_velocity_mm_s: float
    max_axis_velocity_mm_s: float
    max_acceleration_mm_s2: float
    command_history_steps: int
    previous_command_velocity_mm_s: np.ndarray
    first_crossfade_velocity_mm_s: np.ndarray
    max_acceleration_vector_mm_s2: np.ndarray
    max_acceleration_transition_index: int

    def __post_init__(self) -> None:
        first_axis = np.asarray(self.first_xyz_axis_step_mm, dtype=np.float64)
        previous_velocity = np.asarray(
            self.previous_command_velocity_mm_s, dtype=np.float64
        )
        first_velocity = np.asarray(
            self.first_crossfade_velocity_mm_s, dtype=np.float64
        )
        max_acceleration = np.asarray(
            self.max_acceleration_vector_mm_s2, dtype=np.float64
        )
        vectors = (first_axis, previous_velocity, first_velocity, max_acceleration)
        if any(vector.shape != (3,) for vector in vectors) or not all(
            np.all(np.isfinite(vector)) for vector in vectors
        ):
            raise ValueError("execution dynamics vectors must be finite XYZ")
        if self.command_history_steps != 2:
            raise ValueError("execution dynamics require two command history steps")
        if self.acknowledged_command_span_steps not in {1, 2}:
            raise ValueError("execution dynamics ACK span must be one or two steps")
        if (
            not np.isfinite(self.max_acknowledged_xyz_axis_span_mm)
            or self.max_acknowledged_xyz_axis_span_mm < 0.0
            or not np.isfinite(self.max_acknowledged_rotation_span_deg)
            or self.max_acknowledged_rotation_span_deg < 0.0
        ):
            raise ValueError("execution dynamics ACK spans must be finite")
        if self.max_acceleration_transition_index < 0:
            raise ValueError("acceleration transition index must be non-negative")
        object.__setattr__(self, "first_xyz_axis_step_mm", first_axis.copy())
        object.__setattr__(self, "previous_command_velocity_mm_s", previous_velocity.copy())
        object.__setattr__(self, "first_crossfade_velocity_mm_s", first_velocity.copy())
        object.__setattr__(self, "max_acceleration_vector_mm_s2", max_acceleration.copy())

    def record(self) -> dict[str, Any]:
        value = asdict(self)
        value["first_xyz_axis_step_mm"] = self.first_xyz_axis_step_mm.tolist()
        value["previous_command_velocity_mm_s"] = self.previous_command_velocity_mm_s.tolist()
        value["first_crossfade_velocity_mm_s"] = self.first_crossfade_velocity_mm_s.tolist()
        value["max_acceleration_vector_mm_s2"] = self.max_acceleration_vector_mm_s2.tolist()
        return value


@dataclass(frozen=True)
class PrefixAdmission:
    valid: bool
    failure_reasons: tuple[str, ...]
    dynamics: PrefixDynamics
    gripper_compatible: bool
    splice_index: int = 0
    execution_dynamics: HandoffExecutionDynamics | None = None
    semantic_authority: str = SemanticAuthority.RUNTIME_GUARDED.value
    gripper_compatibility_enforced: bool = True

    def record(self) -> dict[str, Any]:
        return {
            "valid": self.valid,
            "failure_reasons": list(self.failure_reasons),
            "dynamics": self.dynamics.record(),
            "gripper_compatible": self.gripper_compatible,
            "semantic_authority": self.semantic_authority,
            "gripper_compatibility_enforced": self.gripper_compatibility_enforced,
            "splice_index": self.splice_index,
            "raw_prefix_acceleration_hard_reject": False,
            "execution_dynamics": (
                None
                if self.execution_dynamics is None
                else self.execution_dynamics.record()
            ),
        }


class HandoffSpliceSelector:
    """Bounded selector over one fresh ACT-B semantic prefix.

    STRICT constructs this with the defaults and therefore still evaluates
    only B[0]. FLEXIBLE may inspect a small initial window without requesting
    additional GPU inference.
    """

    def __init__(self, *, max_splice_index: int = 0, max_candidates: int = 1) -> None:
        if max_splice_index < 0 or max_candidates < 1:
            raise ValueError("invalid bounded splice selector limits")
        self.max_splice_index = int(max_splice_index)
        self.max_candidates = int(max_candidates)

    def candidate_indices(
        self,
        chunk: PolicyChunk,
        *,
        prefix_steps: int,
    ) -> tuple[int, ...]:
        maximum = min(
            self.max_splice_index,
            max(0, len(chunk.actions) - prefix_steps),
        )
        values = list(range(maximum + 1))
        if len(values) <= self.max_candidates:
            return tuple(values)
        selected = np.linspace(0, maximum, self.max_candidates, dtype=np.int64)
        return tuple(dict.fromkeys(int(value) for value in selected))

    def select(self, chunk: PolicyChunk, _reference_action: np.ndarray) -> int:
        return self.candidate_indices(chunk, prefix_steps=2)[0]


class HandoffCompatibilityEvaluator:
    def __init__(
        self,
        config: HandoffCompatibilityConfig,
        *,
        prefix_steps: int,
        action_hz: float,
        semantic_authority: SemanticAuthority | str = SemanticAuthority.RUNTIME_GUARDED,
        splice_selector: HandoffSpliceSelector | None = None,
    ) -> None:
        config.require_live_thresholds()
        if prefix_steps < 2:
            raise ValueError("B prefix admission requires at least two steps")
        if action_hz <= 0.0:
            raise ValueError("action_hz must be positive")
        self.config = config
        self.prefix_steps = int(prefix_steps)
        self.action_hz = float(action_hz)
        self.semantic_authority = parse_semantic_authority(semantic_authority)
        self.splice_selector = splice_selector or HandoffSpliceSelector()

    @staticmethod
    def semantic_compatibility(
        actual: BoundarySemanticState,
        expected: BoundarySemanticState,
    ) -> tuple[bool, tuple[str, ...]]:
        reasons: list[str] = []
        if actual.gripper_state != expected.gripper_state:
            reasons.append("gripper_state_mismatch")
        if actual.held_object != expected.held_object:
            reasons.append("held_object_mismatch")
        if actual.contact_mode != expected.contact_mode:
            reasons.append("contact_mode_mismatch")
        if expected.contact_mode not in {
            "free_transport",
            "free_transport_assumed",
        }:
            reasons.append("contact_transition_not_free_transport")
        actual_preconditions = set(actual.entry_preconditions)
        missing = [
            item
            for item in expected.entry_preconditions
            if item not in actual_preconditions
        ]
        if missing:
            reasons.append("successor_precondition_unmet")
        return not reasons, tuple(reasons)

    def outer_ready(
        self,
        *,
        manifest: EpisodeHandoffManifest,
        actual_semantic: BoundarySemanticState,
        current_pose_mm_deg: np.ndarray,
    ) -> tuple[bool, tuple[str, ...], dict[str, Any]]:
        _semantic_valid, semantic_reasons = self.semantic_compatibility(
            actual_semantic,
            manifest.successor.semantic,
        )
        pose = np.asarray(current_pose_mm_deg, dtype=np.float64)
        if pose.shape != (6,) or not np.all(np.isfinite(pose)):
            return False, ("non_finite_pose",), {
                "semantic_authority": self.semantic_authority.value,
                "semantic_checks_enforced_by_runtime": (
                    self.semantic_authority.runtime_semantic_checks_enforced
                ),
                "semantic_diagnostics": list(semantic_reasons),
                "support_diagnostics": [],
            }
        position_delta = float(
            np.linalg.norm(pose[:3] - manifest.successor.nominal_position_mm)
        )
        orientation_delta = quaternion_angle_deg(
            doosan_zyz_deg_to_quaternion(pose[3:6]),
            manifest.successor.nominal_orientation_quat_xyzw,
        )
        support_diagnostics: list[str] = []
        if position_delta > manifest.successor.support_radius_mm:
            support_diagnostics.append("outside_successor_position_support")
        if (
            orientation_delta
            > manifest.successor.support_orientation_radius_deg
        ):
            support_diagnostics.append("outside_successor_orientation_support")
        hard_reasons: list[str] = []
        if self.semantic_authority.runtime_semantic_checks_enforced:
            hard_reasons.extend(semantic_reasons)
            hard_reasons.extend(support_diagnostics)
        return (
            not hard_reasons,
            tuple(hard_reasons),
            {
                "semantic_authority": self.semantic_authority.value,
                "semantic_checks_enforced_by_runtime": (
                    self.semantic_authority.runtime_semantic_checks_enforced
                ),
                "semantic_diagnostics": list(semantic_reasons),
                "support_diagnostics": support_diagnostics,
                "successor_support_position_delta_mm": position_delta,
                "successor_support_radius_mm": (
                    manifest.successor.support_radius_mm
                ),
                "successor_support_orientation_delta_deg": orientation_delta,
                "successor_support_orientation_radius_deg": (
                    manifest.successor.support_orientation_radius_deg
                ),
            },
        )

    def evaluate_prefix(
        self,
        chunk: PolicyChunk,
        *,
        bridge_reference_action: np.ndarray,
        bridge_velocity_mm_s: np.ndarray,
        expected_semantic: BoundarySemanticState,
        soft_handoff_plan: SoftHandoffPlan | None = None,
        command_history_mm_deg: np.ndarray | None = None,
        post_crossfade_action: np.ndarray | None = None,
        splice_index: int | None = None,
    ) -> PrefixAdmission:
        reference = np.asarray(bridge_reference_action, dtype=np.float64)
        bridge_velocity = np.asarray(bridge_velocity_mm_s, dtype=np.float64)
        if reference.shape != (7,) or not np.all(np.isfinite(reference)):
            raise ValueError("Bridge admission reference must be finite 7D")
        if bridge_velocity.shape != (3,) or not np.all(np.isfinite(bridge_velocity)):
            raise ValueError("Bridge admission velocity must be finite XYZ")
        selected_index = (
            self.splice_selector.select(chunk, reference)
            if splice_index is None
            else int(splice_index)
        )
        if selected_index < 0 or selected_index >= len(chunk.actions):
            raise ValueError("ACT-B splice index lies outside the fresh chunk")
        steps = min(self.prefix_steps, len(chunk.actions) - selected_index)
        if steps < 2:
            raise ValueError("ACT-B chunk is too short for prefix admission")
        prefix = chunk.actions[selected_index : selected_index + steps]
        first_axis_delta = np.abs(prefix[0, :3] - reference[:3])
        first_delta, first_rotation = pose_delta_metrics(
            reference[:6],
            prefix[0, :6],
        )
        velocity = np.diff(prefix[:, :3], axis=0) * self.action_hz
        speed = np.linalg.norm(velocity, axis=1)
        axis_speed = np.abs(velocity)
        if len(velocity) >= 2:
            acceleration = np.diff(velocity, axis=0) * self.action_hz
            max_acceleration = float(
                np.max(np.linalg.norm(acceleration, axis=1))
            )
        else:
            max_acceleration = 0.0
        initial_velocity = velocity[0].copy()
        mismatch = float(np.linalg.norm(initial_velocity - bridge_velocity))

        config = self.config
        assert config.max_first_xyz_axis_delta_mm is not None
        assert config.max_first_rotation_delta_deg is not None
        assert config.max_prefix_velocity_mm_s is not None
        assert config.max_bridge_prefix_velocity_mismatch_mm_s is not None
        reasons: list[str] = []
        if float(np.max(first_axis_delta)) > config.max_first_xyz_axis_delta_mm:
            reasons.append("first_xyz_axis_delta")
        if first_rotation > config.max_first_rotation_delta_deg:
            reasons.append("first_rotation_delta")
        max_velocity = float(np.max(speed))
        max_axis_velocity = float(np.max(axis_speed))
        if max_velocity > config.max_prefix_velocity_mm_s:
            reasons.append("prefix_velocity")
        if mismatch > config.max_bridge_prefix_velocity_mismatch_mm_s:
            reasons.append("bridge_prefix_velocity_mismatch")

        # A candidate may not skip across an OPEN/CLOSE event. This is the
        # bounded semantic-window guard available from the current 7D chunk;
        # the ACT checkpoint has no explicit semantic token input.
        traversed = chunk.actions[: selected_index + steps]
        if expected_semantic.gripper_state == "closed":
            gripper_compatible = bool(
                np.all(traversed[:, 6] >= config.gripper_close_threshold)
            )
        else:
            gripper_compatible = bool(
                np.all(traversed[:, 6] <= config.gripper_open_threshold)
            )
        if (
            not gripper_compatible
            and self.semantic_authority.runtime_semantic_checks_enforced
        ):
            reasons.append("prefix_gripper_semantic_mismatch")

        execution_dynamics = None
        if soft_handoff_plan is not None:
            if command_history_mm_deg is None:
                raise ValueError(
                    "prospective handoff admission requires command history"
                )
            execution_dynamics = self._execution_dynamics(
                soft_handoff_plan,
                command_history_mm_deg=command_history_mm_deg,
                post_crossfade_action=post_crossfade_action,
            )
            assert config.max_crossfade_xyz_axis_step_mm is not None
            assert config.max_crossfade_rotation_step_deg is not None
            assert config.max_acknowledged_xyz_axis_span_mm is not None
            assert config.max_acknowledged_rotation_span_deg is not None
            assert config.max_crossfade_command_acceleration_mm_s2 is not None
            if (
                execution_dynamics.max_xyz_axis_step_mm
                > config.max_crossfade_xyz_axis_step_mm
            ):
                reasons.append("crossfade_xyz_axis_step")
            if (
                execution_dynamics.max_rotation_step_deg
                > config.max_crossfade_rotation_step_deg
            ):
                reasons.append("crossfade_rotation_step")
            if (
                execution_dynamics.max_acknowledged_xyz_axis_span_mm
                > config.max_acknowledged_xyz_axis_span_mm
            ):
                reasons.append("crossfade_acknowledged_xyz_axis_span")
            if (
                execution_dynamics.max_acknowledged_rotation_span_deg
                > config.max_acknowledged_rotation_span_deg
            ):
                reasons.append("crossfade_acknowledged_rotation_span")
            if execution_dynamics.max_velocity_mm_s > config.max_prefix_velocity_mm_s:
                reasons.append("crossfade_velocity")
            if (
                execution_dynamics.max_acceleration_mm_s2
                > config.max_crossfade_command_acceleration_mm_s2
            ):
                reasons.append("crossfade_command_acceleration")

        dynamics = PrefixDynamics(
            prefix_steps=steps,
            first_xyz_axis_delta_mm=first_axis_delta,
            first_xyz_delta_mm=first_delta,
            first_rotation_delta_deg=first_rotation,
            max_velocity_mm_s=max_velocity,
            max_axis_velocity_mm_s=max_axis_velocity,
            max_acceleration_mm_s2=max_acceleration,
            bridge_prefix_velocity_mismatch_mm_s=mismatch,
            initial_b_velocity_mm_s=initial_velocity,
        )
        return PrefixAdmission(
            valid=not reasons,
            failure_reasons=tuple(reasons),
            dynamics=dynamics,
            gripper_compatible=gripper_compatible,
            splice_index=selected_index,
            execution_dynamics=execution_dynamics,
            semantic_authority=self.semantic_authority.value,
            gripper_compatibility_enforced=(
                self.semantic_authority.runtime_semantic_checks_enforced
            ),
        )

    def _execution_dynamics(
        self,
        plan: SoftHandoffPlan,
        *,
        command_history_mm_deg: np.ndarray,
        post_crossfade_action: np.ndarray | None,
    ) -> HandoffExecutionDynamics:
        history = np.asarray(command_history_mm_deg, dtype=np.float64)
        if history.shape != (2, 6) or not np.all(np.isfinite(history)):
            raise ValueError("command history must be finite shape (2, 6)")
        commands = plan.actions
        continuation_steps = 0
        if post_crossfade_action is not None:
            continuation = np.asarray(post_crossfade_action, dtype=np.float64)
            if continuation.shape != (7,) or not np.all(np.isfinite(continuation)):
                raise ValueError("post-crossfade action must be finite 7D")
            commands = np.concatenate((commands, continuation[None, :]), axis=0)
            continuation_steps = 1

        # Reconstruct command-space dynamics only. Using actual TCP here folds
        # tracking lag into a fictitious one-tick move (the former 5162 value).
        # Sequence: cmd[k-1], cmd[k], prospective C[0..N], optional B tail.
        poses = np.concatenate((history, commands[:, :6]), axis=0)
        xyz_steps = np.diff(poses[:, :3], axis=0)
        axis_steps = np.abs(xyz_steps)
        velocity = xyz_steps * self.action_hz
        acceleration = np.diff(velocity, axis=0) * self.action_hz
        future_velocity = velocity[1:]
        future_axis_steps = axis_steps[1:]
        speed = np.linalg.norm(future_velocity, axis=1)
        rotation_steps = np.asarray(
            [
                pose_delta_metrics(poses[index, :6], poses[index + 1, :6])[1]
                for index in range(len(poses) - 1)
            ],
            dtype=np.float64,
        )
        future_rotation_steps = rotation_steps[1:]
        ack_span = self.config.acknowledged_command_span_steps
        # Include every span whose destination is prospective. With span=2,
        # the first pair is cmd[k-1] -> C[0], exactly matching a one-command
        # decision-time ACK lag in the bounded pipeline.
        first_ack_source_index = max(0, 2 - ack_span)
        ack_source_indices = range(
            first_ack_source_index,
            len(poses) - ack_span,
        )
        ack_pairs = tuple(
            (index, index + ack_span) for index in ack_source_indices
        )
        if not ack_pairs:
            raise ValueError("prospective handoff has no ACK span to validate")
        acknowledged_axis_spans = np.asarray(
            [
                np.abs(poses[target, :3] - poses[source, :3])
                for source, target in ack_pairs
            ],
            dtype=np.float64,
        )
        acknowledged_rotation_spans = np.asarray(
            [
                pose_delta_metrics(poses[source, :6], poses[target, :6])[1]
                for source, target in ack_pairs
            ],
            dtype=np.float64,
        )
        acceleration_norm = np.linalg.norm(acceleration, axis=1)
        max_acceleration_index = int(np.argmax(acceleration_norm))
        return HandoffExecutionDynamics(
            crossfade_steps=len(plan.actions),
            continuation_steps=continuation_steps,
            first_xyz_axis_step_mm=future_axis_steps[0],
            max_xyz_axis_step_mm=float(np.max(future_axis_steps)),
            first_rotation_step_deg=float(future_rotation_steps[0]),
            max_rotation_step_deg=float(np.max(future_rotation_steps)),
            acknowledged_command_span_steps=ack_span,
            max_acknowledged_xyz_axis_span_mm=float(
                np.max(acknowledged_axis_spans)
            ),
            max_acknowledged_rotation_span_deg=float(
                np.max(acknowledged_rotation_spans)
            ),
            max_velocity_mm_s=float(np.max(speed)),
            max_axis_velocity_mm_s=float(np.max(np.abs(future_velocity))),
            max_acceleration_mm_s2=float(acceleration_norm[max_acceleration_index]),
            command_history_steps=2,
            previous_command_velocity_mm_s=velocity[0],
            first_crossfade_velocity_mm_s=velocity[1],
            max_acceleration_vector_mm_s2=acceleration[max_acceleration_index],
            max_acceleration_transition_index=max_acceleration_index,
        )
