"""Serializable offline phase-index and handoff-candidate records."""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass
from typing import Any, Mapping

import numpy as np

from offline_tools.task_c_bridge_v0.bezier_bridge import (
    CUBIC_BEZIER_FIXED,
    CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
)

from .authority import SemanticAuthority, parse_semantic_authority


def _finite(value: Any, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite shape ({size},)")
    return result.copy()


@dataclass(frozen=True)
class PhaseIndexPoint:
    task: str
    segment: str
    phase: float
    support_episode: int
    support_frame: int
    position_mm: np.ndarray
    orientation_quat_xyzw: np.ndarray
    velocity_mm_s: np.ndarray
    semantic_state: str
    gripper_state: str
    held_object: str
    contact_mode: str
    entry_preconditions: tuple[str, ...]
    completed_subgoals: tuple[str, ...]
    support_score: float

    def __post_init__(self) -> None:
        if not self.task or not self.segment:
            raise ValueError("phase point task and segment must not be empty")
        if not 0.0 <= self.phase <= 1.0:
            raise ValueError("phase point phase must be in [0, 1]")
        if self.support_episode < 0 or self.support_frame < 0:
            raise ValueError("support episode/frame must be non-negative")
        if self.gripper_state not in {"open", "closed"}:
            raise ValueError("phase point gripper_state must be open or closed")
        if not np.isfinite(self.support_score) or self.support_score < 0.0:
            raise ValueError("support_score must be finite and non-negative")
        position = _finite(self.position_mm, 3, "phase position")
        orientation = _finite(
            self.orientation_quat_xyzw,
            4,
            "phase orientation quaternion",
        )
        norm = float(np.linalg.norm(orientation))
        if norm <= 1e-12:
            raise ValueError("phase orientation quaternion has zero norm")
        object.__setattr__(self, "position_mm", position)
        object.__setattr__(self, "orientation_quat_xyzw", orientation / norm)
        object.__setattr__(
            self,
            "velocity_mm_s",
            _finite(self.velocity_mm_s, 3, "phase velocity"),
        )

    @classmethod
    def from_record(cls, value: Mapping[str, Any]) -> "PhaseIndexPoint":
        return cls(
            task=str(value["task"]),
            segment=str(value["segment"]),
            phase=float(value["phase"]),
            support_episode=int(value["support_episode"]),
            support_frame=int(value["support_frame"]),
            position_mm=value["position_mm"],
            orientation_quat_xyzw=value["orientation_quat_xyzw"],
            velocity_mm_s=value["velocity_mm_s"],
            semantic_state=str(value["semantic_state"]),
            gripper_state=str(value["gripper_state"]),
            held_object=str(value["held_object"]),
            contact_mode=str(value["contact_mode"]),
            entry_preconditions=tuple(value.get("entry_preconditions", ())),
            completed_subgoals=tuple(value.get("completed_subgoals", ())),
            support_score=float(value.get("support_score", 0.0)),
        )

    def to_record(self) -> dict[str, Any]:
        value = asdict(self)
        for name in (
            "position_mm",
            "orientation_quat_xyzw",
            "velocity_mm_s",
        ):
            value[name] = getattr(self, name).tolist()
        value["entry_preconditions"] = list(self.entry_preconditions)
        value["completed_subgoals"] = list(self.completed_subgoals)
        return value


@dataclass(frozen=True)
class HandoffCandidate:
    handoff_id: str
    source: PhaseIndexPoint
    successor: PhaseIndexPoint
    bridge_duration_s: float
    bridge_length_mm: float
    max_velocity_mm_s: float
    max_axis_velocity_mm_s: float
    max_acceleration_mm_s2: float
    jerk_metric: float
    max_orientation_step_deg: float
    support_score: float
    feasible: bool
    rejection_reason: tuple[str, ...]
    semantic_authority: SemanticAuthority = SemanticAuthority.RUNTIME_GUARDED
    semantic_diagnostics: tuple[str, ...] = ()
    ik_checked: bool = False
    collision_checked: bool = False
    generation_method: str = "corridor_diverse"
    transport_floor_mm: float | None = None
    bridge_algorithm: str = CUBIC_BEZIER_FIXED
    minimum_tangent_handle_chord_ratio: float = 0.0
    maximum_endpoint_speed_adjustment_mm_s: float | None = None
    source_endpoint_speed_adjustment_mm_s: float = 0.0
    successor_endpoint_speed_adjustment_mm_s: float = 0.0
    max_curvature_per_mm: float | None = None

    def __post_init__(self) -> None:
        if not self.handoff_id:
            raise ValueError("handoff_id must not be empty")
        if not np.isfinite(self.bridge_duration_s) or self.bridge_duration_s <= 0.0:
            raise ValueError("candidate bridge duration must be positive")
        nonnegative = (
            self.bridge_length_mm,
            self.max_velocity_mm_s,
            self.max_axis_velocity_mm_s,
            self.max_acceleration_mm_s2,
            self.jerk_metric,
            self.max_orientation_step_deg,
            self.support_score,
        )
        if any(not np.isfinite(item) or item < 0.0 for item in nonnegative):
            raise ValueError("candidate metrics must be finite and non-negative")
        if self.transport_floor_mm is not None and not np.isfinite(
            self.transport_floor_mm
        ):
            raise ValueError("candidate transport floor must be finite")
        if self.bridge_algorithm not in {
            CUBIC_BEZIER_FIXED,
            CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
        }:
            raise ValueError("unsupported candidate bridge algorithm")
        ratio = float(self.minimum_tangent_handle_chord_ratio)
        maximum_adjustment = self.maximum_endpoint_speed_adjustment_mm_s
        endpoint_adjustments = (
            self.source_endpoint_speed_adjustment_mm_s,
            self.successor_endpoint_speed_adjustment_mm_s,
        )
        if any(
            not np.isfinite(item) or item < 0.0
            for item in endpoint_adjustments
        ):
            raise ValueError("endpoint speed adjustments must be non-negative")
        if self.bridge_algorithm == CUBIC_BEZIER_FIXED:
            if ratio != 0.0 or maximum_adjustment is not None or any(
                item != 0.0 for item in endpoint_adjustments
            ):
                raise ValueError(
                    "fixed cubic cannot contain tangent regularization values"
                )
        else:
            if not np.isfinite(ratio) or not 0.0 < ratio <= 0.25:
                raise ValueError(
                    "regularized candidate ratio must be in (0, 0.25]"
                )
            if maximum_adjustment is None or (
                not np.isfinite(maximum_adjustment)
                or maximum_adjustment <= 0.0
            ):
                raise ValueError(
                    "regularized candidate requires a positive speed limit"
                )
        if self.max_curvature_per_mm is not None and (
            not np.isfinite(self.max_curvature_per_mm)
            or self.max_curvature_per_mm < 0.0
        ):
            raise ValueError("candidate curvature must be non-negative")
        authority = parse_semantic_authority(self.semantic_authority)
        semantic_diagnostics = tuple(
            dict.fromkeys(str(item) for item in self.semantic_diagnostics)
        )
        reasons = tuple(dict.fromkeys(str(item) for item in self.rejection_reason))
        object.__setattr__(self, "semantic_authority", authority)
        object.__setattr__(self, "semantic_diagnostics", semantic_diagnostics)
        if self.feasible and reasons:
            raise ValueError("feasible candidate cannot contain rejection reasons")
        if not self.feasible and not reasons:
            raise ValueError("rejected candidate requires a rejection reason")
        object.__setattr__(self, "rejection_reason", reasons)

    @classmethod
    def identifier(
        cls,
        source: PhaseIndexPoint,
        successor: PhaseIndexPoint,
        duration_s: float,
        *,
        bridge_algorithm: str = CUBIC_BEZIER_FIXED,
        minimum_tangent_handle_chord_ratio: float = 0.0,
        maximum_endpoint_speed_adjustment_mm_s: float | None = None,
    ) -> str:
        payload = {
            "source": [
                source.task,
                source.segment,
                source.support_episode,
                source.support_frame,
            ],
            "successor": [
                successor.task,
                successor.segment,
                successor.support_episode,
                successor.support_frame,
            ],
            "duration_s": float(duration_s),
        }
        if bridge_algorithm != CUBIC_BEZIER_FIXED:
            payload["bridge_algorithm"] = str(bridge_algorithm)
            payload["minimum_tangent_handle_chord_ratio"] = float(
                minimum_tangent_handle_chord_ratio
            )
            payload["maximum_endpoint_speed_adjustment_mm_s"] = float(
                maximum_endpoint_speed_adjustment_mm_s
            )
        digest = hashlib.sha256(
            json.dumps(payload, sort_keys=True).encode("utf-8")
        ).hexdigest()[:12]
        return f"h_{digest}"

    @classmethod
    def from_record(cls, value: Mapping[str, Any]) -> "HandoffCandidate":
        return cls(
            handoff_id=str(value["handoff_id"]),
            source=PhaseIndexPoint.from_record(value["source"]),
            successor=PhaseIndexPoint.from_record(value["successor"]),
            bridge_duration_s=float(value["bridge_duration_s"]),
            bridge_length_mm=float(value.get("bridge_length_mm", value.get("bridge_length"))),
            max_velocity_mm_s=float(value.get("max_velocity_mm_s", value.get("max_velocity"))),
            max_axis_velocity_mm_s=float(value.get("max_axis_velocity_mm_s", value.get("max_axis_velocity"))),
            max_acceleration_mm_s2=float(value.get("max_acceleration_mm_s2", value.get("max_acceleration"))),
            jerk_metric=float(value["jerk_metric"]),
            max_orientation_step_deg=float(value["max_orientation_step_deg"]),
            support_score=float(value["support_score"]),
            feasible=bool(value["feasible"]),
            rejection_reason=tuple(value.get("rejection_reason", ())),
            semantic_authority=parse_semantic_authority(
                value.get("semantic_authority", SemanticAuthority.RUNTIME_GUARDED.value)
            ),
            semantic_diagnostics=tuple(value.get("semantic_diagnostics", ())),
            ik_checked=bool(value.get("ik_checked", False)),
            collision_checked=bool(value.get("collision_checked", False)),
            generation_method=str(value.get("generation_method", "corridor_diverse")),
            transport_floor_mm=(
                None
                if value.get("transport_floor_mm") is None
                else float(value["transport_floor_mm"])
            ),
            bridge_algorithm=str(
                value.get("bridge_algorithm", CUBIC_BEZIER_FIXED)
            ),
            minimum_tangent_handle_chord_ratio=float(
                value.get("minimum_tangent_handle_chord_ratio", 0.0)
            ),
            maximum_endpoint_speed_adjustment_mm_s=(
                None
                if value.get("maximum_endpoint_speed_adjustment_mm_s") is None
                else float(value["maximum_endpoint_speed_adjustment_mm_s"])
            ),
            source_endpoint_speed_adjustment_mm_s=float(
                value.get("source_endpoint_speed_adjustment_mm_s", 0.0)
            ),
            successor_endpoint_speed_adjustment_mm_s=float(
                value.get("successor_endpoint_speed_adjustment_mm_s", 0.0)
            ),
            max_curvature_per_mm=(
                None
                if value.get("max_curvature_per_mm") is None
                else float(value["max_curvature_per_mm"])
            ),
        )

    def to_record(self) -> dict[str, Any]:
        return {
            "handoff_id": self.handoff_id,
            "source_task": self.source.task,
            "source_segment": self.source.segment,
            "source_phase": self.source.phase,
            "source_support_episode": self.source.support_episode,
            "successor_task": self.successor.task,
            "successor_segment": self.successor.segment,
            "successor_phase": self.successor.phase,
            "successor_support_episode": self.successor.support_episode,
            "bridge_duration_s": self.bridge_duration_s,
            "semantic_state": self.source.semantic_state,
            "gripper_state": self.source.gripper_state,
            "held_object": self.source.held_object,
            "contact_mode": self.source.contact_mode,
            "nominal_source_pose": {
                "position_mm": self.source.position_mm.tolist(),
                "orientation_quat_xyzw": self.source.orientation_quat_xyzw.tolist(),
            },
            "nominal_source_velocity": self.source.velocity_mm_s.tolist(),
            "nominal_successor_pose": {
                "position_mm": self.successor.position_mm.tolist(),
                "orientation_quat_xyzw": self.successor.orientation_quat_xyzw.tolist(),
            },
            "nominal_successor_velocity": self.successor.velocity_mm_s.tolist(),
            "bridge_length": self.bridge_length_mm,
            "max_velocity": self.max_velocity_mm_s,
            "max_axis_velocity": self.max_axis_velocity_mm_s,
            "max_acceleration": self.max_acceleration_mm_s2,
            "jerk_metric": self.jerk_metric,
            "max_orientation_step_deg": self.max_orientation_step_deg,
            "support_score": self.support_score,
            "feasible": self.feasible,
            "rejection_reason": list(self.rejection_reason),
            "semantic_authority": self.semantic_authority.value,
            "semantic_checks_enforced_by_runtime": (
                self.semantic_authority.runtime_semantic_checks_enforced
            ),
            "semantic_diagnostics": list(self.semantic_diagnostics),
            "ik_checked": self.ik_checked,
            "collision_checked": self.collision_checked,
            "generation_method": self.generation_method,
            "transport_floor_mm": self.transport_floor_mm,
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
            "max_curvature_per_mm": self.max_curvature_per_mm,
            "source": self.source.to_record(),
            "successor": self.successor.to_record(),
        }
