"""One-shot, actual-state cubic Bezier instantiation for Task-C V2."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from offline_tools.task_c_bridge_v0.bezier_bridge import (
    CUBIC_BEZIER_FIXED,
    CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
    CubicBezierBridge,
    build_tangent_regularized_bezier,
    build_velocity_matched_bezier,
)
from offline_tools.task_c_bridge_v0.bridge_metrics import (
    BridgeMetrics,
    evaluate_bridge,
    feasibility_reasons,
)
from quest_a0509_teleop.doosan_orientation import (
    doosan_zyz_deg_to_quaternion,
    quaternion_angle_deg,
    quaternion_slerp,
    quaternion_to_doosan_zyz_deg,
)

from .models import EpisodeHandoffManifest, HandoffV2Config
from .soft_handoff import quintic_smoothstep


class BridgeGenerationError(RuntimeError):
    """Raised before execution when a selected V2 boundary is infeasible."""

    def __init__(self, reasons: list[str] | tuple[str, ...]) -> None:
        self.reasons = tuple(dict.fromkeys(str(reason) for reason in reasons))
        super().__init__("V2 Bridge rejected: " + ",".join(self.reasons))


def _finite_vector(value: Any, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite shape ({size},)")
    return result.copy()


@dataclass(frozen=True)
class BridgeRuntimeSnapshot:
    timestamp_s: float
    actual_pose_mm_deg: np.ndarray
    acknowledged_pose_mm_deg: np.ndarray
    actual_velocity_mm_s: np.ndarray
    gripper_target: float | None = None

    def __post_init__(self) -> None:
        if not np.isfinite(self.timestamp_s):
            raise ValueError("Bridge snapshot timestamp must be finite")
        object.__setattr__(
            self,
            "actual_pose_mm_deg",
            _finite_vector(self.actual_pose_mm_deg, 6, "actual pose"),
        )
        object.__setattr__(
            self,
            "acknowledged_pose_mm_deg",
            _finite_vector(
                self.acknowledged_pose_mm_deg,
                6,
                "acknowledged pose",
            ),
        )
        object.__setattr__(
            self,
            "actual_velocity_mm_s",
            _finite_vector(self.actual_velocity_mm_s, 3, "actual velocity"),
        )
        if self.gripper_target is not None:
            gripper = float(self.gripper_target)
            if not np.isfinite(gripper) or gripper not in {0.0, 1.0}:
                raise ValueError("Bridge snapshot gripper target must be 0 or 1")
            object.__setattr__(self, "gripper_target", gripper)


@dataclass(frozen=True)
class BridgeRuntimeLimits:
    workspace_min_mm: np.ndarray
    workspace_max_mm: np.ndarray
    velocity_limit_mm_s: float
    axis_velocity_limit_mm_s: float
    acceleration_limit_mm_s2: float
    curvature_limit_per_mm: float
    jerk_limit_mm_s3: float
    integrated_squared_jerk_limit: float
    backtracking_ratio_limit: float
    linear_ramp_mm_per_tick: float
    orientation_ramp_deg_per_tick: float
    sample_hz: float = 60.0
    curvature_epsilon: float = 1.0e-9
    ack_pipeline_max_lag_steps: int = 0
    workspace_min_limit_enabled: tuple[bool, bool, bool] = (True, True, True)

    def __post_init__(self) -> None:
        minimum = _finite_vector(self.workspace_min_mm, 3, "workspace minimum")
        maximum = _finite_vector(self.workspace_max_mm, 3, "workspace maximum")
        minimum_enabled = np.asarray(
            self.workspace_min_limit_enabled, dtype=np.bool_
        )
        if minimum_enabled.shape != (3,):
            raise ValueError("workspace_min_limit_enabled must contain XYZ")
        if np.any(minimum[minimum_enabled] >= maximum[minimum_enabled]):
            raise ValueError("workspace bounds must be strictly ordered")
        positive = (
            self.velocity_limit_mm_s,
            self.axis_velocity_limit_mm_s,
            self.acceleration_limit_mm_s2,
            self.curvature_limit_per_mm,
            self.jerk_limit_mm_s3,
            self.integrated_squared_jerk_limit,
            self.backtracking_ratio_limit,
            self.linear_ramp_mm_per_tick,
            self.orientation_ramp_deg_per_tick,
            self.sample_hz,
            self.curvature_epsilon,
        )
        if any(not np.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("Bridge runtime limits must be finite and positive")
        if (
            not isinstance(self.ack_pipeline_max_lag_steps, int)
            or isinstance(self.ack_pipeline_max_lag_steps, bool)
            or self.ack_pipeline_max_lag_steps not in {0, 1}
        ):
            raise ValueError("ack_pipeline_max_lag_steps must be 0 or 1")
        object.__setattr__(self, "workspace_min_mm", minimum)
        object.__setattr__(self, "workspace_max_mm", maximum)
        object.__setattr__(
            self,
            "workspace_min_limit_enabled",
            tuple(bool(value) for value in minimum_enabled),
        )

    @property
    def feasibility_mapping(self) -> dict[str, float]:
        return {
            "velocity_limit_mm_s": self.velocity_limit_mm_s,
            "acceleration_limit_mm_s2": self.acceleration_limit_mm_s2,
            "curvature_limit_per_mm": self.curvature_limit_per_mm,
            "jerk_limit_mm_s3": self.jerk_limit_mm_s3,
            "integrated_squared_jerk_limit": (
                self.integrated_squared_jerk_limit
            ),
            "backtracking_ratio_limit": self.backtracking_ratio_limit,
        }


@dataclass(frozen=True)
class PrecomputedBridgeQueue:
    # STRICT uses CubicBezierBridge. FLEXIBLE may supply a C2 piecewise path
    # exposing the same duration/position/velocity/acceleration contract.
    bridge: Any
    metrics: BridgeMetrics
    actions: np.ndarray
    velocity_mm_s: np.ndarray
    acceleration_mm_s2: np.ndarray
    handoff_window_start_index: int
    source_snapshot: BridgeRuntimeSnapshot
    generation_latency_s: float
    maximum_axis_velocity_mm_s: float
    maximum_axis_step_mm: float
    maximum_orientation_step_deg: float
    ack_span_steps: int
    maximum_ack_span_axis_step_mm: float
    maximum_ack_span_orientation_step_deg: float
    transport_floor_mm: float | None
    workspace_min_limit_enabled: tuple[bool, bool, bool]
    bridge_algorithm: str
    minimum_tangent_handle_chord_ratio: float
    maximum_endpoint_speed_adjustment_mm_s: float | None
    source_endpoint_speed_adjustment_mm_s: float
    successor_endpoint_speed_adjustment_mm_s: float
    ik_checked: bool
    collision_checked: bool
    bridge_mode: str = "strict_level2"
    candidate_generator_type: str = "strict_velocity_matched"
    selected_candidate_score: float | None = None
    candidate_diagnostics: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        actions = np.asarray(self.actions, dtype=np.float64)
        velocity = np.asarray(self.velocity_mm_s, dtype=np.float64)
        acceleration = np.asarray(self.acceleration_mm_s2, dtype=np.float64)
        if actions.ndim != 2 or actions.shape[1] != 7 or len(actions) < 1:
            raise ValueError("Bridge action queue must have shape [N, 7]")
        if velocity.shape != (len(actions), 3):
            raise ValueError("Bridge velocity queue must align with actions")
        if acceleration.shape != (len(actions), 3):
            raise ValueError("Bridge acceleration queue must align with actions")
        if not all(np.all(np.isfinite(value)) for value in (actions, velocity, acceleration)):
            raise ValueError("Bridge queue contains non-finite values")
        if not 0 <= self.handoff_window_start_index < len(actions):
            raise ValueError("handoff window start lies outside Bridge queue")
        if self.ack_span_steps not in {1, 2}:
            raise ValueError("Bridge ACK span must be one or two steps")
        for value in (
            self.maximum_ack_span_axis_step_mm,
            self.maximum_ack_span_orientation_step_deg,
            self.source_endpoint_speed_adjustment_mm_s,
            self.successor_endpoint_speed_adjustment_mm_s,
        ):
            if not np.isfinite(value) or value < 0.0:
                raise ValueError("Bridge ACK span metrics must be finite")
        object.__setattr__(self, "actions", actions.copy())
        object.__setattr__(self, "velocity_mm_s", velocity.copy())
        object.__setattr__(self, "acceleration_mm_s2", acceleration.copy())

    @property
    def steps(self) -> int:
        return len(self.actions)

    def in_handoff_window(self, action_index: int) -> bool:
        return self.handoff_window_start_index <= action_index < self.steps

    def remaining_steps(self, action_index: int) -> int:
        return max(0, self.steps - int(action_index))

    def record(self) -> dict[str, Any]:
        return {
            "duration_s": self.bridge.duration_s,
            "steps": self.steps,
            "gripper_target": float(self.actions[0, 6]),
            "handoff_window_start_index": self.handoff_window_start_index,
            "length_mm": self.metrics.length_mm,
            "max_velocity_mm_s": self.metrics.max_velocity_mm_s,
            "max_axis_velocity_mm_s": self.maximum_axis_velocity_mm_s,
            "max_acceleration_mm_s2": self.metrics.max_acceleration_mm_s2,
            "max_jerk_mm_s3": self.metrics.max_jerk_mm_s3,
            "integrated_squared_jerk": self.metrics.integrated_squared_jerk,
            "max_axis_step_mm": self.maximum_axis_step_mm,
            "max_orientation_step_deg": self.maximum_orientation_step_deg,
            "ack_span_steps": self.ack_span_steps,
            "max_ack_span_axis_step_mm": (
                self.maximum_ack_span_axis_step_mm
            ),
            "max_ack_span_orientation_step_deg": (
                self.maximum_ack_span_orientation_step_deg
            ),
            "generation_latency_ms": self.generation_latency_s * 1000.0,
            "transport_floor_mm": self.transport_floor_mm,
            "minimum_position_z_mm": float(
                min(
                    self.source_snapshot.acknowledged_pose_mm_deg[2],
                    np.min(self.actions[:, 2]),
                )
            ),
            "workspace_min_limit_enabled": list(
                self.workspace_min_limit_enabled
            ),
            "bridge_algorithm": self.bridge_algorithm,
            "minimum_tangent_handle_chord_ratio": (
                self.minimum_tangent_handle_chord_ratio
            ),
            "maximum_endpoint_speed_adjustment_mm_s": (
                self.maximum_endpoint_speed_adjustment_mm_s
            ),
            "source_endpoint_speed_adjustment_mm_s": (
                self.source_endpoint_speed_adjustment_mm_s
            ),
            "successor_endpoint_speed_adjustment_mm_s": (
                self.successor_endpoint_speed_adjustment_mm_s
            ),
            "ik_checked": self.ik_checked,
            "collision_checked": self.collision_checked,
            "bridge_mode": self.bridge_mode,
            "candidate_generator_type": self.candidate_generator_type,
            "selected_candidate_score": self.selected_candidate_score,
            "candidate_diagnostics": self.candidate_diagnostics,
        }


def instantiate_bridge_queue(
    manifest: EpisodeHandoffManifest,
    snapshot: BridgeRuntimeSnapshot,
    config: HandoffV2Config,
    limits: BridgeRuntimeLimits,
    *,
    clock: Any,
) -> PrecomputedBridgeQueue:
    """Instantiate and validate the complete queue once at A-exit commit.

    Command continuity is anchored to the latest acknowledged ServoL command,
    while the initial derivative is estimated causally from recent actual TCP.
    No candidate or duration search is performed here.
    """

    started_s = float(clock())
    source_speed_adjustment = 0.0
    successor_speed_adjustment = 0.0
    regularization_reasons: list[str] = []
    if manifest.bridge_algorithm == CUBIC_BEZIER_FIXED:
        bridge = build_velocity_matched_bezier(
            snapshot.acknowledged_pose_mm_deg[:3],
            snapshot.actual_velocity_mm_s,
            manifest.successor.nominal_position_mm,
            manifest.successor.nominal_velocity_mm_s,
            manifest.bridge_duration_s,
        )
    elif (
        manifest.bridge_algorithm
        == CUBIC_BEZIER_TANGENT_REGULARIZED_V1
    ):
        try:
            bridge, regularization = build_tangent_regularized_bezier(
                snapshot.acknowledged_pose_mm_deg[:3],
                snapshot.actual_velocity_mm_s,
                manifest.successor.nominal_position_mm,
                manifest.successor.nominal_velocity_mm_s,
                manifest.bridge_duration_s,
                minimum_handle_chord_ratio=(
                    manifest.minimum_tangent_handle_chord_ratio
                ),
            )
        except ValueError as exc:
            label = str(exc)
            if label in {
                "source_direction_undefined",
                "successor_direction_undefined",
            }:
                raise BridgeGenerationError([label]) from exc
            raise
        source_speed_adjustment = (
            regularization.source_speed_adjustment_mm_s
        )
        successor_speed_adjustment = (
            regularization.successor_speed_adjustment_mm_s
        )
        maximum_adjustment = (
            manifest.maximum_endpoint_speed_adjustment_mm_s
        )
        if maximum_adjustment is None or (
            regularization.maximum_speed_adjustment_mm_s
            > maximum_adjustment + 1.0e-9
        ):
            regularization_reasons.append(
                "endpoint_speed_adjustment_limit"
            )
    else:  # EpisodeHandoffManifest validates this before runtime entry.
        raise BridgeGenerationError(["unsupported_bridge_algorithm"])
    metrics, sampled = evaluate_bridge(
        bridge,
        sample_hz=limits.sample_hz,
        curvature_epsilon=limits.curvature_epsilon,
        workspace_min_mm=limits.workspace_min_mm,
        workspace_max_mm=limits.workspace_max_mm,
        workspace_min_limit_enabled=(
            limits.workspace_min_limit_enabled
        ),
    )
    reasons = regularization_reasons + feasibility_reasons(
        metrics, limits.feasibility_mapping
    )
    sampled_axis_velocity = np.abs(sampled["velocity_mm_s"])
    maximum_axis_velocity = float(np.max(sampled_axis_velocity))
    if maximum_axis_velocity > limits.axis_velocity_limit_mm_s + 1e-9:
        reasons.append("axis_velocity_limit")
    if manifest.transport_floor_mm is not None and float(
        np.min(sampled["position_mm"][:, 2])
    ) < manifest.transport_floor_mm - 1e-6:
        reasons.append("transport_floor_violation")

    count = max(1, int(round(bridge.duration_s * config.control_hz)))
    normalized = np.arange(1, count + 1, dtype=np.float64) / float(count)
    positions = bridge.position(normalized)
    velocity = bridge.velocity(normalized)
    acceleration = bridge.acceleration(normalized)

    start_quaternion = doosan_zyz_deg_to_quaternion(
        snapshot.acknowledged_pose_mm_deg[3:6]
    )
    target_quaternion = manifest.successor.nominal_orientation_quat_xyzw
    orientation = np.empty((count, 3), dtype=np.float64)
    reference = snapshot.acknowledged_pose_mm_deg[3:6].tolist()
    for index, fraction in enumerate(normalized):
        weight = quintic_smoothstep(float(fraction))
        quaternion = quaternion_slerp(
            start_quaternion,
            target_quaternion,
            weight,
        )
        reference = quaternion_to_doosan_zyz_deg(quaternion, reference)
        orientation[index] = reference

    gripper_target = (
        manifest.source.semantic.gripper_target
        if snapshot.gripper_target is None
        else snapshot.gripper_target
    )
    gripper = np.full(
        (count, 1),
        gripper_target,
        dtype=np.float64,
    )
    actions = np.concatenate((positions, orientation, gripper), axis=1)
    previous_xyz = np.concatenate(
        (snapshot.acknowledged_pose_mm_deg[:3][None, :], positions),
        axis=0,
    )
    maximum_axis_step = float(np.max(np.abs(np.diff(previous_xyz, axis=0))))
    if maximum_axis_step > limits.linear_ramp_mm_per_tick + 1e-9:
        reasons.append("linear_ramp_limit")

    previous_orientation = snapshot.acknowledged_pose_mm_deg[3:6]
    maximum_orientation_step = 0.0
    for target_orientation in orientation:
        angle = quaternion_angle_deg(
            doosan_zyz_deg_to_quaternion(previous_orientation),
            doosan_zyz_deg_to_quaternion(target_orientation),
        )
        maximum_orientation_step = max(maximum_orientation_step, angle)
        previous_orientation = target_orientation
    if maximum_orientation_step > limits.orientation_ramp_deg_per_tick + 1e-9:
        reasons.append("orientation_ramp_limit")

    # A decision-time ACK lag of one means the next candidate may transiently
    # be two generated targets ahead of the latest acknowledged pose. Validate
    # that complete span before ACT-A authority is invalidated.
    ack_span_steps = 1 + limits.ack_pipeline_max_lag_steps
    maximum_ack_span_axis_step = maximum_axis_step
    maximum_ack_span_orientation_step = maximum_orientation_step
    if ack_span_steps > 1 and len(previous_xyz) > ack_span_steps:
        maximum_ack_span_axis_step = float(
            np.max(
                np.abs(
                    previous_xyz[ack_span_steps:]
                    - previous_xyz[:-ack_span_steps]
                )
            )
        )
        all_orientation = np.concatenate(
            (
                snapshot.acknowledged_pose_mm_deg[3:6][None, :],
                orientation,
            ),
            axis=0,
        )
        maximum_ack_span_orientation_step = 0.0
        for index in range(ack_span_steps, len(all_orientation)):
            angle = quaternion_angle_deg(
                doosan_zyz_deg_to_quaternion(
                    all_orientation[index - ack_span_steps]
                ),
                doosan_zyz_deg_to_quaternion(all_orientation[index]),
            )
            maximum_ack_span_orientation_step = max(
                maximum_ack_span_orientation_step,
                angle,
            )
        if (
            maximum_ack_span_axis_step
            > limits.linear_ramp_mm_per_tick + 1e-9
        ):
            reasons.append("ack_pipeline_linear_ramp_limit")
        if (
            maximum_ack_span_orientation_step
            > limits.orientation_ramp_deg_per_tick + 1e-9
        ):
            reasons.append("ack_pipeline_orientation_ramp_limit")

    if reasons:
        raise BridgeGenerationError(reasons)
    window_start = max(0, count - config.handoff_window_steps)
    completed_s = float(clock())
    return PrecomputedBridgeQueue(
        bridge=bridge,
        metrics=metrics,
        actions=actions,
        velocity_mm_s=velocity,
        acceleration_mm_s2=acceleration,
        handoff_window_start_index=window_start,
        source_snapshot=snapshot,
        generation_latency_s=max(0.0, completed_s - started_s),
        maximum_axis_velocity_mm_s=maximum_axis_velocity,
        maximum_axis_step_mm=maximum_axis_step,
        maximum_orientation_step_deg=maximum_orientation_step,
        ack_span_steps=ack_span_steps,
        maximum_ack_span_axis_step_mm=maximum_ack_span_axis_step,
        maximum_ack_span_orientation_step_deg=(
            maximum_ack_span_orientation_step
        ),
        transport_floor_mm=manifest.transport_floor_mm,
        workspace_min_limit_enabled=limits.workspace_min_limit_enabled,
        bridge_algorithm=manifest.bridge_algorithm,
        minimum_tangent_handle_chord_ratio=(
            manifest.minimum_tangent_handle_chord_ratio
        ),
        maximum_endpoint_speed_adjustment_mm_s=(
            manifest.maximum_endpoint_speed_adjustment_mm_s
        ),
        source_endpoint_speed_adjustment_mm_s=source_speed_adjustment,
        successor_endpoint_speed_adjustment_mm_s=(
            successor_speed_adjustment
        ),
        ik_checked=manifest.ik_checked,
        collision_checked=manifest.collision_checked,
    )
