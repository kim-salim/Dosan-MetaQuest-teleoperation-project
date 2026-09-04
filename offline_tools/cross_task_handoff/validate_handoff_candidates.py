"""Hard-filter one or more Task-C V2 boundary candidates."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from offline_tools.task_c_bridge_v0.bezier_bridge import (
    CUBIC_BEZIER_FIXED,
    CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
    build_tangent_regularized_bezier,
    build_velocity_matched_bezier,
)
from offline_tools.task_c_bridge_v0.bridge_metrics import (
    evaluate_bridge,
    feasibility_reasons,
)
from quest_a0509_teleop.doosan_orientation import (
    quaternion_angle_deg,
    quaternion_slerp,
)

from .authority import SemanticAuthority, parse_semantic_authority
from .schema import HandoffCandidate, PhaseIndexPoint


@dataclass(frozen=True)
class CandidateValidationConfig:
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
    workspace_min_limit_enabled: tuple[bool, bool, bool] = (
        True,
        True,
        True,
    )
    control_hz: float = 30.0
    sample_hz: float = 60.0
    curvature_epsilon: float = 1.0e-9

    def __post_init__(self) -> None:
        minimum = np.asarray(self.workspace_min_mm, dtype=np.float64)
        maximum = np.asarray(self.workspace_max_mm, dtype=np.float64)
        minimum_enabled = np.asarray(
            self.workspace_min_limit_enabled, dtype=np.bool_
        )
        if (
            minimum.shape != (3,)
            or maximum.shape != (3,)
            or minimum_enabled.shape != (3,)
            or not np.all(np.isfinite(minimum))
            or not np.all(np.isfinite(maximum))
            or np.any(minimum[minimum_enabled] >= maximum[minimum_enabled])
        ):
            raise ValueError("candidate workspace bounds must be finite ordered XYZ")
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
            self.control_hz,
            self.sample_hz,
            self.curvature_epsilon,
        )
        if any(not np.isfinite(item) or item <= 0.0 for item in positive):
            raise ValueError("candidate validation limits must be positive")
        object.__setattr__(self, "workspace_min_mm", minimum.copy())
        object.__setattr__(self, "workspace_max_mm", maximum.copy())
        object.__setattr__(
            self,
            "workspace_min_limit_enabled",
            tuple(bool(value) for value in minimum_enabled),
        )

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "CandidateValidationConfig":
        workspace = dict(value["workspace"])
        dynamics = dict(value["dynamics"])
        downstream = dict(value["downstream"])
        return cls(
            workspace_min_mm=np.asarray(workspace["minimum_mm"], dtype=np.float64),
            workspace_max_mm=np.asarray(workspace["maximum_mm"], dtype=np.float64),
            velocity_limit_mm_s=float(dynamics["velocity_limit_mm_s"]),
            axis_velocity_limit_mm_s=float(dynamics["axis_velocity_limit_mm_s"]),
            acceleration_limit_mm_s2=float(dynamics["acceleration_limit_mm_s2"]),
            curvature_limit_per_mm=float(dynamics["curvature_limit_per_mm"]),
            jerk_limit_mm_s3=float(dynamics["jerk_limit_mm_s3"]),
            integrated_squared_jerk_limit=float(
                dynamics["integrated_squared_jerk_limit"]
            ),
            backtracking_ratio_limit=float(dynamics["backtracking_ratio_limit"]),
            linear_ramp_mm_per_tick=float(
                downstream["linear_ramp_mm_per_tick"]
            ),
            orientation_ramp_deg_per_tick=float(
                downstream["orientation_ramp_deg_per_tick"]
            ),
            workspace_min_limit_enabled=tuple(
                bool(item)
                for item in workspace.get(
                    "minimum_limit_enabled", (True, True, True)
                )
            ),
            control_hz=float(downstream.get("control_hz", 30.0)),
            sample_hz=float(value.get("sample_hz", 60.0)),
            curvature_epsilon=float(value.get("curvature_epsilon", 1e-9)),
        )

    @property
    def feasibility_mapping(self) -> dict[str, float]:
        return {
            "velocity_limit_mm_s": self.velocity_limit_mm_s,
            "acceleration_limit_mm_s2": self.acceleration_limit_mm_s2,
            "curvature_limit_per_mm": self.curvature_limit_per_mm,
            "jerk_limit_mm_s3": self.jerk_limit_mm_s3,
            "integrated_squared_jerk_limit": self.integrated_squared_jerk_limit,
            "backtracking_ratio_limit": self.backtracking_ratio_limit,
        }


def semantic_hard_rejections(
    source: PhaseIndexPoint,
    successor: PhaseIndexPoint,
) -> list[str]:
    reasons: list[str] = []
    if source.held_object != successor.held_object:
        reasons.append("held_object_mismatch")
    if source.gripper_state != successor.gripper_state:
        reasons.append("gripper_state_mismatch")
    if source.contact_mode != successor.contact_mode:
        reasons.append("contact_mode_mismatch")
    if source.contact_mode not in {
        "free_transport",
        "free_transport_assumed",
    }:
        reasons.append("source_not_free_transport")
    if successor.contact_mode not in {
        "free_transport",
        "free_transport_assumed",
    }:
        reasons.append("successor_not_free_transport")
    completed = set(source.completed_subgoals)
    if any(item not in completed for item in successor.entry_preconditions):
        reasons.append("successor_precondition_unmet")
    return reasons


def _orientation_max_step_deg(
    source: PhaseIndexPoint,
    successor: PhaseIndexPoint,
    *,
    duration_s: float,
    control_hz: float,
) -> float:
    count = max(1, int(round(duration_s * control_hz)))
    fractions = np.arange(1, count + 1, dtype=np.float64) / float(count)
    weights = 10.0 * fractions**3 - 15.0 * fractions**4 + 6.0 * fractions**5
    previous = source.orientation_quat_xyzw
    maximum = 0.0
    for weight in weights:
        current = quaternion_slerp(
            source.orientation_quat_xyzw,
            successor.orientation_quat_xyzw,
            float(weight),
        )
        maximum = max(maximum, quaternion_angle_deg(previous, current))
        previous = current
    return maximum


def validate_candidate(
    source: PhaseIndexPoint,
    successor: PhaseIndexPoint,
    *,
    duration_s: float,
    config: CandidateValidationConfig,
    transport_floor_mm: float | None,
    semantic_authority: SemanticAuthority | str = SemanticAuthority.RUNTIME_GUARDED,
    bridge_algorithm: str = CUBIC_BEZIER_FIXED,
    minimum_tangent_handle_chord_ratio: float = 0.0,
    maximum_endpoint_speed_adjustment_mm_s: float | None = None,
) -> HandoffCandidate:
    if not np.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("candidate Bridge duration must be positive")
    if transport_floor_mm is not None and not np.isfinite(transport_floor_mm):
        raise ValueError("candidate transport floor must be finite")
    authority = parse_semantic_authority(semantic_authority)
    semantic_diagnostics = tuple(
        semantic_hard_rejections(source, successor)
    )
    reasons = list(semantic_diagnostics) if authority.runtime_semantic_checks_enforced else []
    if float(np.linalg.norm(source.velocity_mm_s)) <= 1.0e-6:
        reasons.append("source_direction_undefined")
    source_speed_adjustment = 0.0
    successor_speed_adjustment = 0.0
    if bridge_algorithm == CUBIC_BEZIER_FIXED:
        if (
            float(minimum_tangent_handle_chord_ratio) != 0.0
            or maximum_endpoint_speed_adjustment_mm_s is not None
        ):
            raise ValueError(
                "fixed cubic cannot use tangent regularization parameters"
            )
        bridge = build_velocity_matched_bezier(
            source.position_mm,
            source.velocity_mm_s,
            successor.position_mm,
            successor.velocity_mm_s,
            duration_s,
        )
    elif bridge_algorithm == CUBIC_BEZIER_TANGENT_REGULARIZED_V1:
        if maximum_endpoint_speed_adjustment_mm_s is None or (
            not np.isfinite(maximum_endpoint_speed_adjustment_mm_s)
            or maximum_endpoint_speed_adjustment_mm_s <= 0.0
        ):
            raise ValueError(
                "regularized cubic requires a positive endpoint speed limit"
            )
        try:
            bridge, regularization = build_tangent_regularized_bezier(
                source.position_mm,
                source.velocity_mm_s,
                successor.position_mm,
                successor.velocity_mm_s,
                duration_s,
                minimum_handle_chord_ratio=(
                    minimum_tangent_handle_chord_ratio
                ),
            )
        except ValueError as exc:
            label = str(exc)
            if label in {
                "source_direction_undefined",
                "successor_direction_undefined",
            }:
                reasons.append(label)
                bridge = build_velocity_matched_bezier(
                    source.position_mm,
                    source.velocity_mm_s,
                    successor.position_mm,
                    successor.velocity_mm_s,
                    duration_s,
                )
                regularization = None
            else:
                raise
        if regularization is not None:
            source_speed_adjustment = (
                regularization.source_speed_adjustment_mm_s
            )
            successor_speed_adjustment = (
                regularization.successor_speed_adjustment_mm_s
            )
            if regularization.maximum_speed_adjustment_mm_s > (
                maximum_endpoint_speed_adjustment_mm_s + 1.0e-9
            ):
                reasons.append("endpoint_speed_adjustment_limit")
    else:
        raise ValueError(f"unsupported bridge algorithm: {bridge_algorithm}")
    metrics, sample = evaluate_bridge(
        bridge,
        sample_hz=config.sample_hz,
        curvature_epsilon=config.curvature_epsilon,
        workspace_min_mm=config.workspace_min_mm,
        workspace_max_mm=config.workspace_max_mm,
        workspace_min_limit_enabled=(
            config.workspace_min_limit_enabled
        ),
    )
    reasons.extend(feasibility_reasons(metrics, config.feasibility_mapping))
    max_axis_velocity = float(np.max(np.abs(sample["velocity_mm_s"])))
    if max_axis_velocity > config.axis_velocity_limit_mm_s + 1e-9:
        reasons.append("axis_velocity_limit")
    count = max(1, int(round(duration_s * config.control_hz)))
    xyz = bridge.position(
        np.arange(1, count + 1, dtype=np.float64) / float(count)
    )
    all_xyz = np.concatenate((source.position_mm[None, :], xyz), axis=0)
    if float(np.max(np.abs(np.diff(all_xyz, axis=0)))) > (
        config.linear_ramp_mm_per_tick + 1e-9
    ):
        reasons.append("linear_ramp_limit")
    if transport_floor_mm is not None and float(
        np.min(sample["position_mm"][:, 2])
    ) < transport_floor_mm - 1e-6:
        reasons.append("transport_floor_violation")
    max_orientation_step = _orientation_max_step_deg(
        source,
        successor,
        duration_s=duration_s,
        control_hz=config.control_hz,
    )
    if max_orientation_step > config.orientation_ramp_deg_per_tick + 1e-9:
        reasons.append("orientation_ramp_limit")
    reasons = list(dict.fromkeys(reasons))
    return HandoffCandidate(
        handoff_id=HandoffCandidate.identifier(
            source,
            successor,
            duration_s,
            bridge_algorithm=bridge_algorithm,
            minimum_tangent_handle_chord_ratio=(
                minimum_tangent_handle_chord_ratio
            ),
            maximum_endpoint_speed_adjustment_mm_s=(
                maximum_endpoint_speed_adjustment_mm_s
            ),
        ),
        source=source,
        successor=successor,
        bridge_duration_s=float(duration_s),
        bridge_length_mm=metrics.length_mm,
        max_velocity_mm_s=metrics.max_velocity_mm_s,
        max_axis_velocity_mm_s=max_axis_velocity,
        max_acceleration_mm_s2=metrics.max_acceleration_mm_s2,
        jerk_metric=metrics.integrated_squared_jerk,
        max_orientation_step_deg=max_orientation_step,
        support_score=0.5 * (source.support_score + successor.support_score),
        feasible=not reasons,
        rejection_reason=tuple(reasons),
        semantic_authority=authority,
        semantic_diagnostics=semantic_diagnostics,
        # No trusted robot IK or environment collision checker exists in this
        # repository. Never present geometric filtering as either guarantee.
        ik_checked=False,
        collision_checked=False,
        transport_floor_mm=transport_floor_mm,
        bridge_algorithm=bridge_algorithm,
        minimum_tangent_handle_chord_ratio=(
            minimum_tangent_handle_chord_ratio
        ),
        maximum_endpoint_speed_adjustment_mm_s=(
            maximum_endpoint_speed_adjustment_mm_s
        ),
        source_endpoint_speed_adjustment_mm_s=source_speed_adjustment,
        successor_endpoint_speed_adjustment_mm_s=(
            successor_speed_adjustment
        ),
        max_curvature_per_mm=metrics.max_curvature_per_mm,
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument(
        "--semantic-authority",
        choices=[item.value for item in SemanticAuthority],
    )
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    library = json.loads(args.input.read_text(encoding="utf-8"))
    authority = parse_semantic_authority(
        args.semantic_authority
        or library.get(
            "semantic_authority", SemanticAuthority.RUNTIME_GUARDED.value
        )
    )
    config = CandidateValidationConfig.from_mapping(
        json.loads(args.config.read_text(encoding="utf-8"))
    )
    output: list[dict[str, Any]] = []
    for value in library["candidates"]:
        source = PhaseIndexPoint.from_record(value["source"])
        successor = PhaseIndexPoint.from_record(value["successor"])
        candidate = validate_candidate(
            source,
            successor,
            duration_s=float(value["bridge_duration_s"]),
            config=config,
            transport_floor_mm=value.get("transport_floor_mm"),
            semantic_authority=authority,
            bridge_algorithm=str(
                value.get(
                    "bridge_algorithm",
                    library.get("bridge_algorithm", CUBIC_BEZIER_FIXED),
                )
            ),
            minimum_tangent_handle_chord_ratio=float(
                value.get(
                    "minimum_tangent_handle_chord_ratio",
                    library.get("minimum_tangent_handle_chord_ratio", 0.0),
                )
            ),
            maximum_endpoint_speed_adjustment_mm_s=(
                None
                if value.get(
                    "maximum_endpoint_speed_adjustment_mm_s",
                    library.get("maximum_endpoint_speed_adjustment_mm_s"),
                )
                is None
                else float(
                    value.get(
                        "maximum_endpoint_speed_adjustment_mm_s",
                        library.get(
                            "maximum_endpoint_speed_adjustment_mm_s"
                        ),
                    )
                )
            ),
        )
        output.append(candidate.to_record())
    result = {
        **{key: value for key, value in library.items() if key != "candidates"},
        "schema_version": "a0509.cross_task_handoff_library.v2",
        "validated": True,
        "semantic_authority": authority.value,
        "semantic_checks_enforced_by_runtime": authority.runtime_semantic_checks_enforced,
        "candidates": output,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        f"HANDOFF_VALIDATION_OK total={len(output)} "
        f"feasible={sum(item['feasible'] for item in output)}"
    )


if __name__ == "__main__":
    main()
