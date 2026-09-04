"""Reference-guided, bounded FLEXIBLE_LEVEL2 Bridge generation.

The legacy STRICT_LEVEL2 generator intentionally remains in bridge_runtime.py.
This module treats the reviewed/demonstration-derived cubic as a reference,
anchors the entry to a fresh actual/ACK snapshot, keeps the middle close to the
reference, and leaves final ACT-B authority to the async handoff coordinator.
Candidate search is bounded and is designed to run in a worker, never in the
30 Hz command-consumption path.
"""

from __future__ import annotations

import concurrent.futures
import math
import multiprocessing
import os
import threading
import time
from dataclasses import asdict, dataclass, field, replace
from typing import Any, Callable, Iterable

import numpy as np

from offline_tools.task_c_bridge_v0.bezier_bridge import (
    CUBIC_BEZIER_FIXED,
    CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
    CubicBezierBridge,
    build_tangent_regularized_bezier,
    build_velocity_matched_bezier,
)
from offline_tools.task_c_bridge_v0.bridge_metrics import BridgeMetrics
from quest_a0509_teleop.doosan_orientation import (
    doosan_zyz_deg_to_quaternion,
    quaternion_angle_deg,
    quaternion_slerp,
    quaternion_to_doosan_zyz_deg,
)

from lerobot_robot_doosan_a0509.runtime_scheduling import (
    temporary_current_thread_affinity,
)
from .bridge_runtime import (
    BridgeGenerationError,
    BridgeRuntimeLimits,
    BridgeRuntimeSnapshot,
    PrecomputedBridgeQueue,
)
from .models import EpisodeHandoffManifest, HandoffV2Config
from .soft_handoff import quintic_smoothstep


def _bridge_worker_initializer(cpu_set: tuple[int, ...], nice_value: int) -> None:
    """Isolate the spawned Bridge worker before it evaluates candidates."""

    requested = set(int(cpu) for cpu in cpu_set)
    if not requested:
        raise RuntimeError("Bridge worker CPU set must not be empty")
    available = set(os.sched_getaffinity(0))
    if not requested.issubset(available):
        raise RuntimeError(
            "Bridge worker CPU set is outside its inherited affinity: "
            f"requested={sorted(requested)} available={sorted(available)}"
        )
    os.sched_setaffinity(0, requested)
    inherited_nice = os.getpriority(os.PRIO_PROCESS, 0)
    # An unprivileged child may lower its scheduling priority (larger nice)
    # but may not raise it above an already-niced parent. Preserve the safer
    # inherited value in that case.
    applied_nice = max(inherited_nice, int(nice_value))
    if applied_nice != inherited_nice:
        os.setpriority(os.PRIO_PROCESS, 0, applied_nice)
    # Affinity is the primary isolation. These also constrain native pools
    # that are created lazily after the worker initializer has run.
    for name in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ[name] = "1"


def _bridge_worker_identity() -> dict[str, Any]:
    return {
        "pid": os.getpid(),
        "cpu_set": sorted(os.sched_getaffinity(0)),
        "nice": os.getpriority(os.PRIO_PROCESS, 0),
        "backend": "process",
    }


@dataclass(frozen=True)
class FlexibleBridgeWeights:
    reference: float = 3.0
    support: float = 1.0
    target_prefix: float = 1.0
    velocity: float = 0.5
    orientation: float = 0.5
    curvature: float = 0.25
    acceleration: float = 0.5
    jerk: float = 0.25
    duration: float = 0.1
    length: float = 0.1
    ood: float = 1.0

    def __post_init__(self) -> None:
        if any(
            not np.isfinite(value) or value < 0.0
            for value in asdict(self).values()
        ):
            raise ValueError("flexible Bridge weights must be finite and non-negative")


@dataclass(frozen=True)
class FlexibleBridgeSearchConfig:
    reference_guided: bool = True
    use_actual_ack_start: bool = True
    enable_reference_deformation: bool = True
    enable_tangent_regularization: bool = True
    enable_settle_connector: bool = True
    enable_alignment_connector: bool = True
    enable_lift_transport_connector: bool = True
    enable_generic_safe_connector: bool = False
    allow_generic_safe_connector_live: bool = False
    allow_intermediate_release: bool = False
    curvature_policy: str = "soft_except_cusp"
    tangent_handle_chord_ratios: tuple[float, ...] = (
        0.0,
        0.05,
        0.10,
        0.20,
        0.25,
    )
    duration_multipliers: tuple[float, ...] = (1.0, 1.25, 1.50)
    reference_join_windows: tuple[tuple[float, float], ...] = (
        (0.18, 0.82),
        (0.25, 0.75),
        (0.35, 0.68),
    )
    curvature_speed_epsilon_mm_s: float = 5.0
    cusp_speed_epsilon_mm_s: float = 0.5
    severe_reversal_cosine: float = -0.5
    self_intersection_tolerance_mm: float = 0.5
    lift_clearance_mm: float = 35.0
    max_candidates: int = 64
    max_search_time_s: float = 0.50
    adaptive_entry_projection_max_fraction: float = 0.60
    adaptive_entry_max_join_fraction: float = 0.80
    adaptive_entry_join_lookahead_fractions: tuple[float, ...] = (
        0.05,
        0.10,
        0.15,
        0.20,
        0.30,
    )
    adaptive_entry_max_candidates: int = 6
    adaptive_entry_max_search_time_s: float = 0.12
    weights: FlexibleBridgeWeights = field(default_factory=FlexibleBridgeWeights)

    def __post_init__(self) -> None:
        if self.curvature_policy != "soft_except_cusp":
            raise ValueError("FLEXIBLE_LEVEL2 requires curvature_policy=soft_except_cusp")
        if not self.reference_guided:
            raise ValueError("FLEXIBLE_LEVEL2 requires reference_guided=true")
        if not self.use_actual_ack_start:
            raise ValueError("FLEXIBLE_LEVEL2 requires use_actual_ack_start=true")
        if self.allow_intermediate_release:
            raise ValueError("initial FLEXIBLE_LEVEL2 forbids intermediate release")
        if not isinstance(self.max_candidates, int) or self.max_candidates < 1:
            raise ValueError("max_candidates must be a positive integer")
        if (
            not isinstance(self.adaptive_entry_max_candidates, int)
            or isinstance(self.adaptive_entry_max_candidates, bool)
            or self.adaptive_entry_max_candidates < 1
        ):
            raise ValueError(
                "adaptive_entry_max_candidates must be a positive integer"
            )
        if not (
            0.0
            < self.adaptive_entry_projection_max_fraction
            < self.adaptive_entry_max_join_fraction
            < 1.0
        ):
            raise ValueError(
                "adaptive entry projection/join fractions are invalid"
            )
        if (
            not self.adaptive_entry_join_lookahead_fractions
            or any(
                not np.isfinite(value) or value <= 0.0
                for value in self.adaptive_entry_join_lookahead_fractions
            )
        ):
            raise ValueError(
                "adaptive entry lookahead fractions must be positive"
            )
        positive = (
            self.curvature_speed_epsilon_mm_s,
            self.cusp_speed_epsilon_mm_s,
            self.self_intersection_tolerance_mm,
            self.lift_clearance_mm,
            self.max_search_time_s,
            self.adaptive_entry_max_search_time_s,
        )
        if any(not np.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("flexible Bridge scalar limits must be positive")
        if not -1.0 <= self.severe_reversal_cosine < 0.0:
            raise ValueError("severe_reversal_cosine must be in [-1, 0)")
        if not self.tangent_handle_chord_ratios:
            raise ValueError("at least one tangent ratio is required")
        if any(
            not np.isfinite(value) or not 0.0 <= value <= 0.25
            for value in self.tangent_handle_chord_ratios
        ):
            raise ValueError("tangent ratios must be in [0, 0.25]")
        if any(
            not np.isfinite(value) or value < 1.0
            for value in self.duration_multipliers
        ):
            raise ValueError("duration multipliers must be finite and >= 1")
        for entry, exit_ in self.reference_join_windows:
            if not 0.0 < entry < exit_ < 1.0:
                raise ValueError("reference windows must satisfy 0 < entry < exit < 1")


@dataclass(frozen=True)
class _QuinticSegment:
    coefficients: np.ndarray
    duration_s: float

    @classmethod
    def from_boundaries(
        cls,
        p0: np.ndarray,
        v0: np.ndarray,
        a0: np.ndarray,
        p1: np.ndarray,
        v1: np.ndarray,
        a1: np.ndarray,
        duration_s: float,
    ) -> "_QuinticSegment":
        duration = float(duration_s)
        if not np.isfinite(duration) or duration <= 0.0:
            raise ValueError("quintic duration must be positive")
        values = [np.asarray(value, dtype=np.float64) for value in (p0, v0, a0, p1, v1, a1)]
        if any(value.shape != (3,) or not np.all(np.isfinite(value)) for value in values):
            raise ValueError("quintic boundaries must be finite XYZ vectors")
        p0v, v0v, a0v, p1v, v1v, a1v = values
        c0 = p0v
        c1 = v0v
        c2 = 0.5 * a0v
        t = duration
        matrix = np.asarray(
            (
                (t**3, t**4, t**5),
                (3.0 * t**2, 4.0 * t**3, 5.0 * t**4),
                (6.0 * t, 12.0 * t**2, 20.0 * t**3),
            ),
            dtype=np.float64,
        )
        rhs = np.stack(
            (
                p1v - (c0 + c1 * t + c2 * t**2),
                v1v - (c1 + 2.0 * c2 * t),
                a1v - 2.0 * c2,
            ),
            axis=0,
        )
        c3, c4, c5 = np.linalg.solve(matrix, rhs)
        return cls(np.stack((c0, c1, c2, c3, c4, c5)), duration)

    def _time(self, value: np.ndarray | float) -> np.ndarray:
        return np.clip(np.asarray(value, dtype=np.float64), 0.0, self.duration_s)

    def position(self, value: np.ndarray | float) -> np.ndarray:
        t = self._time(value)
        powers = np.stack(tuple(t**index for index in range(6)), axis=-1)
        return powers @ self.coefficients

    def velocity(self, value: np.ndarray | float) -> np.ndarray:
        t = self._time(value)
        powers = np.stack(
            (np.ones_like(t), 2.0 * t, 3.0 * t**2, 4.0 * t**3, 5.0 * t**4),
            axis=-1,
        )
        return powers @ self.coefficients[1:]

    def acceleration(self, value: np.ndarray | float) -> np.ndarray:
        t = self._time(value)
        powers = np.stack(
            (2.0 * np.ones_like(t), 6.0 * t, 12.0 * t**2, 20.0 * t**3),
            axis=-1,
        )
        return powers @ self.coefficients[2:]

    def jerk(self, value: np.ndarray | float) -> np.ndarray:
        t = self._time(value)
        powers = np.stack(
            (6.0 * np.ones_like(t), 24.0 * t, 60.0 * t**2),
            axis=-1,
        )
        return powers @ self.coefficients[3:]


@dataclass(frozen=True)
class ReferenceGuidedBridge:
    """C2 entry/reference-middle/exit deformation of a nominal cubic."""

    reference: CubicBezierBridge
    start_position_mm: np.ndarray
    start_velocity_mm_s: np.ndarray
    target_position_mm: np.ndarray
    target_velocity_mm_s: np.ndarray
    duration_s: float
    entry_fraction: float
    exit_fraction: float
    tangent_handle_chord_ratio: float = 0.0
    lift_clearance_mm: float = 0.0

    def __post_init__(self) -> None:
        if not 0.0 < self.entry_fraction < self.exit_fraction < 1.0:
            raise ValueError("invalid reference join interval")
        if not np.isfinite(self.duration_s) or self.duration_s <= 0.0:
            raise ValueError("reference-guided duration must be positive")
        start = np.asarray(self.start_position_mm, dtype=np.float64)
        start_velocity = np.asarray(self.start_velocity_mm_s, dtype=np.float64)
        target = np.asarray(self.target_position_mm, dtype=np.float64)
        target_velocity = np.asarray(self.target_velocity_mm_s, dtype=np.float64)
        if any(
            value.shape != (3,) or not np.all(np.isfinite(value))
            for value in (start, start_velocity, target, target_velocity)
        ):
            raise ValueError("reference-guided boundaries must be finite XYZ")

        start_delta = start - self.reference.p0
        target_delta = target - self.reference.p3
        points = [
            self.reference.p0 + start_delta,
            self.reference.p1 + (2.0 * start_delta + target_delta) / 3.0,
            self.reference.p2 + (start_delta + 2.0 * target_delta) / 3.0,
            self.reference.p3 + target_delta,
        ]
        if self.lift_clearance_mm > 0.0:
            points[1] = points[1].copy()
            points[2] = points[2].copy()
            points[1][2] += float(self.lift_clearance_mm)
            points[2][2] += float(self.lift_clearance_mm)

        ratio = float(self.tangent_handle_chord_ratio)
        chord = float(np.linalg.norm(target - start))
        if ratio > 0.0 and chord > 1.0e-9:
            for endpoint, handle, sign in ((0, 1, 1.0), (3, 2, -1.0)):
                direction = points[handle] - points[endpoint]
                norm = float(np.linalg.norm(direction))
                if norm > 1.0e-9:
                    applied = max(norm, ratio * chord)
                    points[handle] = points[endpoint] + direction / norm * applied

        warped = CubicBezierBridge(
            p0=points[0],
            p1=points[1],
            p2=points[2],
            p3=points[3],
            duration_s=float(self.duration_s),
        )
        entry = float(self.entry_fraction)
        exit_ = float(self.exit_fraction)
        entry_segment = _QuinticSegment.from_boundaries(
            start,
            start_velocity,
            np.zeros(3),
            warped.position(entry),
            warped.velocity(entry),
            warped.acceleration(entry),
            self.duration_s * entry,
        )
        exit_segment = _QuinticSegment.from_boundaries(
            warped.position(exit_),
            warped.velocity(exit_),
            warped.acceleration(exit_),
            target,
            target_velocity,
            np.zeros(3),
            self.duration_s * (1.0 - exit_),
        )
        object.__setattr__(self, "start_position_mm", start)
        object.__setattr__(self, "start_velocity_mm_s", start_velocity)
        object.__setattr__(self, "target_position_mm", target)
        object.__setattr__(self, "target_velocity_mm_s", target_velocity)
        object.__setattr__(self, "_warped_reference", warped)
        object.__setattr__(self, "_entry_segment", entry_segment)
        object.__setattr__(self, "_exit_segment", exit_segment)

    @property
    def p0(self) -> np.ndarray:
        return self.start_position_mm

    @property
    def p3(self) -> np.ndarray:
        return self.target_position_mm

    def _evaluate(self, normalized: np.ndarray | float, derivative: str) -> np.ndarray:
        values = np.asarray(normalized, dtype=np.float64)
        flat = np.clip(values.reshape(-1), 0.0, 1.0)
        output = np.empty((len(flat), 3), dtype=np.float64)
        entry_mask = flat <= self.entry_fraction
        exit_mask = flat >= self.exit_fraction
        middle_mask = ~(entry_mask | exit_mask)
        entry_time = flat[entry_mask] * self.duration_s
        exit_time = (flat[exit_mask] - self.exit_fraction) * self.duration_s
        output[entry_mask] = getattr(self._entry_segment, derivative)(entry_time)
        output[middle_mask] = getattr(self._warped_reference, derivative)(flat[middle_mask])
        output[exit_mask] = getattr(self._exit_segment, derivative)(exit_time)
        reshaped = output.reshape(values.shape + (3,))
        return reshaped

    def position(self, normalized: np.ndarray | float) -> np.ndarray:
        return self._evaluate(normalized, "position")

    def velocity(self, normalized: np.ndarray | float) -> np.ndarray:
        return self._evaluate(normalized, "velocity")

    def acceleration(self, normalized: np.ndarray | float) -> np.ndarray:
        return self._evaluate(normalized, "acceleration")

    def jerk(self, normalized: np.ndarray | float) -> np.ndarray:
        return self._evaluate(normalized, "jerk")

    def sample(self, sample_hz: float = 60.0) -> dict[str, np.ndarray]:
        count = max(2, int(math.ceil(self.duration_s * sample_hz)) + 1)
        normalized = np.linspace(0.0, 1.0, count)
        return {
            "u": normalized,
            "time_s": normalized * self.duration_s,
            "position_mm": self.position(normalized),
            "velocity_mm_s": self.velocity(normalized),
            "acceleration_mm_s2": self.acceleration(normalized),
            "jerk_mm_s3": self.jerk(normalized),
        }


@dataclass(frozen=True)
class CachedReferenceAdaptiveBridge:
    """Fresh C2 entry followed by an unchanged cached reference middle/tail.

    The expensive bounded search selects ``reference`` once. At commit time
    only the interval ``[0, join_fraction]`` is regenerated from the latest
    acknowledged pose and causal actual velocity. The remaining path is the
    already-validated representative-guided queue.
    """

    reference: Any
    start_position_mm: np.ndarray
    start_velocity_mm_s: np.ndarray
    join_fraction: float

    def __post_init__(self) -> None:
        if not 0.0 < float(self.join_fraction) < 1.0:
            raise ValueError("cached reference join_fraction must be in (0, 1)")
        start = np.asarray(self.start_position_mm, dtype=np.float64)
        velocity = np.asarray(self.start_velocity_mm_s, dtype=np.float64)
        if any(
            value.shape != (3,) or not np.all(np.isfinite(value))
            for value in (start, velocity)
        ):
            raise ValueError("cached adaptive entry requires finite XYZ boundaries")
        duration = float(self.reference.duration_s)
        if not np.isfinite(duration) or duration <= 0.0:
            raise ValueError("cached reference duration must be positive")
        join = float(self.join_fraction)
        segment = _QuinticSegment.from_boundaries(
            start,
            velocity,
            np.zeros(3, dtype=np.float64),
            np.asarray(self.reference.position(join), dtype=np.float64),
            np.asarray(self.reference.velocity(join), dtype=np.float64),
            np.asarray(self.reference.acceleration(join), dtype=np.float64),
            duration * join,
        )
        object.__setattr__(self, "start_position_mm", start)
        object.__setattr__(self, "start_velocity_mm_s", velocity)
        object.__setattr__(self, "duration_s", duration)
        object.__setattr__(self, "_entry_segment", segment)

    @property
    def p0(self) -> np.ndarray:
        return self.start_position_mm

    @property
    def p3(self) -> np.ndarray:
        return np.asarray(self.reference.p3, dtype=np.float64)

    def _evaluate(self, normalized: np.ndarray | float, derivative: str) -> np.ndarray:
        values = np.asarray(normalized, dtype=np.float64)
        flat = np.clip(values.reshape(-1), 0.0, 1.0)
        output = np.empty((len(flat), 3), dtype=np.float64)
        entry_mask = flat <= self.join_fraction
        middle_mask = ~entry_mask
        output[entry_mask] = getattr(self._entry_segment, derivative)(
            flat[entry_mask] * self.duration_s
        )
        output[middle_mask] = getattr(self.reference, derivative)(flat[middle_mask])
        return output.reshape(values.shape + (3,))

    def position(self, normalized: np.ndarray | float) -> np.ndarray:
        return self._evaluate(normalized, "position")

    def velocity(self, normalized: np.ndarray | float) -> np.ndarray:
        return self._evaluate(normalized, "velocity")

    def acceleration(self, normalized: np.ndarray | float) -> np.ndarray:
        return self._evaluate(normalized, "acceleration")

    def jerk(self, normalized: np.ndarray | float) -> np.ndarray:
        return self._evaluate(normalized, "jerk")

    def sample(self, sample_hz: float = 60.0) -> dict[str, np.ndarray]:
        count = max(2, int(math.ceil(self.duration_s * sample_hz)) + 1)
        normalized = np.linspace(0.0, 1.0, count)
        return {
            "u": normalized,
            "time_s": normalized * self.duration_s,
            "position_mm": self.position(normalized),
            "velocity_mm_s": self.velocity(normalized),
            "acceleration_mm_s2": self.acceleration(normalized),
            "jerk_mm_s3": self.jerk(normalized),
        }


@dataclass(frozen=True)
class FlexibleCandidateDiagnostics:
    generator_type: str
    tangent_handle_chord_ratio: float
    duration_s: float
    entry_fraction: float
    exit_fraction: float
    max_curvature_per_mm: float
    curvature_p95_moving_per_mm: float
    max_curvature_u: float
    speed_at_max_curvature_mm_s: float
    source_tangent_angle_deg: float | None
    max_velocity_mm_s: float
    max_acceleration_mm_s2: float
    max_jerk_mm_s3: float
    max_normal_acceleration_mm_s2: float
    max_command_acceleration_mm_s2: float
    max_command_jerk_mm_s3: float
    max_position_axis_step_mm: float
    max_orientation_step_deg: float
    max_ack_span_axis_step_mm: float
    max_ack_span_orientation_step_deg: float
    minimum_position_z_mm: float
    reference_rms_error_mm: float
    actual_reference_start_error_mm: float
    ood_score: float
    hard_rejection_reasons: tuple[str, ...]
    score: float | None
    ik_checked: bool = False
    collision_checked: bool = False
    self_intersection_checked: bool = True
    reference_projection_fraction: float | None = None
    future_join_lookahead_fraction: float | None = None

    def record(self) -> dict[str, Any]:
        value = asdict(self)
        value["hard_rejection_reasons"] = list(self.hard_rejection_reasons)
        return value


@dataclass(frozen=True)
class FlexibleBridgeSearchResult:
    queue: PrecomputedBridgeQueue | None
    candidates_evaluated: int
    candidates_hard_passed: int
    search_latency_s: float
    selected: FlexibleCandidateDiagnostics | None
    rejected_reason_counts: dict[str, int]
    timed_out: bool
    candidate_diagnostics: tuple[FlexibleCandidateDiagnostics, ...] = ()

    @property
    def valid(self) -> bool:
        return self.queue is not None

    def record(self) -> dict[str, Any]:
        return {
            "valid": self.valid,
            "candidates_evaluated": self.candidates_evaluated,
            "candidates_hard_passed": self.candidates_hard_passed,
            "search_latency_ms": self.search_latency_s * 1000.0,
            "selected": None if self.selected is None else self.selected.record(),
            "rejected_reason_counts": dict(self.rejected_reason_counts),
            "timed_out": self.timed_out,
            "candidate_diagnostics": [
                item.record() for item in self.candidate_diagnostics
            ],
        }


@dataclass(frozen=True)
class FlexibleBridgeTemplate:
    """Heavy-search result cached for one handoff edge/generation."""

    handoff_id: str
    search: FlexibleBridgeSearchResult
    prepared_from_snapshot_timestamp_s: float

    def __post_init__(self) -> None:
        if not self.handoff_id:
            raise ValueError("Bridge template handoff_id must not be empty")
        if not self.search.valid or self.search.queue is None:
            raise ValueError("Bridge template requires a valid cached queue")
        if not np.isfinite(self.prepared_from_snapshot_timestamp_s):
            raise ValueError("Bridge template timestamp must be finite")

    @property
    def queue(self) -> PrecomputedBridgeQueue:
        assert self.search.queue is not None
        return self.search.queue

    def record(self) -> dict[str, Any]:
        return {
            "handoff_id": self.handoff_id,
            "prepared_from_snapshot_timestamp_s": (
                self.prepared_from_snapshot_timestamp_s
            ),
            "search": self.search.record(),
        }


def nominal_medoid_bridge_snapshot(
    manifest: EpisodeHandoffManifest,
    *,
    timestamp_s: float,
) -> BridgeRuntimeSnapshot:
    """Build the command-free source-medoid snapshot used for setup caching.

    The manifest source is the reviewed real-episode medoid/reference selected
    by the offline compiler. This snapshot is not executable state: it only
    seeds the expensive reference-template search before Live is authorized.
    Runtime execution always performs a fresh actual/ACK adaptive rebase.
    """

    source = manifest.source
    orientation_deg = quaternion_to_doosan_zyz_deg(
        source.nominal_orientation_quat_xyzw
    )
    pose = np.concatenate(
        (source.nominal_position_mm, orientation_deg),
        dtype=np.float64,
    )
    return BridgeRuntimeSnapshot(
        timestamp_s=float(timestamp_s),
        actual_pose_mm_deg=pose,
        acknowledged_pose_mm_deg=pose,
        actual_velocity_mm_s=source.nominal_velocity_mm_s,
        gripper_target=source.semantic.gripper_target,
    )


def _nominal_reference(manifest: EpisodeHandoffManifest, duration_s: float) -> CubicBezierBridge:
    if manifest.bridge_algorithm == CUBIC_BEZIER_TANGENT_REGULARIZED_V1:
        bridge, _ = build_tangent_regularized_bezier(
            manifest.source.nominal_position_mm,
            manifest.source.nominal_velocity_mm_s,
            manifest.successor.nominal_position_mm,
            manifest.successor.nominal_velocity_mm_s,
            duration_s,
            minimum_handle_chord_ratio=max(
                1.0e-6,
                manifest.minimum_tangent_handle_chord_ratio,
            ),
        )
        return bridge
    return build_velocity_matched_bezier(
        manifest.source.nominal_position_mm,
        manifest.source.nominal_velocity_mm_s,
        manifest.successor.nominal_position_mm,
        manifest.successor.nominal_velocity_mm_s,
        duration_s,
    )


def _sample_metrics(
    bridge: Any,
    limits: BridgeRuntimeLimits,
    *,
    curvature_speed_epsilon_mm_s: float,
) -> tuple[BridgeMetrics, dict[str, np.ndarray], dict[str, float]]:
    sampled = bridge.sample(limits.sample_hz)
    position = sampled["position_mm"]
    velocity = sampled["velocity_mm_s"]
    acceleration = sampled["acceleration_mm_s2"]
    jerk = sampled["jerk_mm_s3"]
    speed = np.linalg.norm(velocity, axis=1)
    acceleration_magnitude = np.linalg.norm(acceleration, axis=1)
    jerk_magnitude = np.linalg.norm(jerk, axis=1)
    cross = np.linalg.norm(np.cross(velocity, acceleration), axis=1)
    curvature = cross / np.maximum(speed**3, limits.curvature_epsilon)
    normal_acceleration = speed**2 * curvature
    increments = np.linalg.norm(np.diff(position, axis=0), axis=1)
    length = float(np.sum(increments))
    chord = float(np.linalg.norm(position[-1] - position[0]))
    minimum_enabled = np.asarray(limits.workspace_min_limit_enabled, dtype=np.bool_)
    workspace = bool(
        np.all(position[:, minimum_enabled] >= limits.workspace_min_mm[None, minimum_enabled])
        and np.all(position <= limits.workspace_max_mm[None, :])
    )
    finite = all(
        np.all(np.isfinite(value))
        for value in (position, velocity, acceleration, jerk, curvature)
    )
    trapezoid = getattr(np, "trapezoid", np.trapz)
    integrated_jerk = float(
        trapezoid(jerk_magnitude**2, x=sampled["time_s"])
    )
    max_index = int(np.argmax(curvature))
    moving = curvature[speed >= curvature_speed_epsilon_mm_s]
    extra = {
        "max_curvature_u": float(sampled["u"][max_index]),
        "speed_at_max_curvature_mm_s": float(speed[max_index]),
        "curvature_p95_moving_per_mm": (
            0.0 if len(moving) == 0 else float(np.quantile(moving, 0.95))
        ),
        "max_normal_acceleration_mm_s2": float(np.max(normal_acceleration)),
    }
    sampled.update(
        {
            "speed_mm_s": speed,
            "acceleration_magnitude_mm_s2": acceleration_magnitude,
            "jerk_magnitude_mm_s3": jerk_magnitude,
            "curvature_per_mm": curvature,
            "normal_acceleration_mm_s2": normal_acceleration,
        }
    )
    return (
        BridgeMetrics(
            length_mm=length,
            max_velocity_mm_s=float(np.max(speed)),
            max_acceleration_mm_s2=float(np.max(acceleration_magnitude)),
            max_curvature_per_mm=float(np.max(curvature)),
            max_jerk_mm_s3=float(np.max(jerk_magnitude)),
            integrated_squared_jerk=integrated_jerk,
            backtracking_ratio=length / max(chord, 1.0e-9),
            workspace_satisfied=workspace,
            finite=finite,
        ),
        sampled,
        extra,
    )


def _orientation_actions(
    snapshot: BridgeRuntimeSnapshot,
    manifest: EpisodeHandoffManifest,
    positions: np.ndarray,
    *,
    cached_orientation_actions: np.ndarray | None = None,
    cached_join_index: int | None = None,
) -> tuple[np.ndarray, float]:
    count = len(positions)
    start_quaternion = doosan_zyz_deg_to_quaternion(
        snapshot.acknowledged_pose_mm_deg[3:6]
    )
    orientation = np.empty((count, 3), dtype=np.float64)
    reference_angles = snapshot.acknowledged_pose_mm_deg[3:6].tolist()
    max_step = 0.0
    previous_quaternion = start_quaternion
    if cached_orientation_actions is not None:
        cached = np.asarray(cached_orientation_actions, dtype=np.float64)
        if cached.shape != (count, 3) or not np.all(np.isfinite(cached)):
            raise ValueError("cached Bridge orientation queue must be finite [N, 3]")
        if cached_join_index is None or not 0 <= cached_join_index < count:
            raise ValueError("cached Bridge orientation join index is invalid")
        join_quaternion = doosan_zyz_deg_to_quaternion(cached[cached_join_index])
        for index in range(cached_join_index + 1):
            fraction = (index + 1) / float(cached_join_index + 1)
            quaternion = quaternion_slerp(
                start_quaternion,
                join_quaternion,
                quintic_smoothstep(fraction),
            )
            reference_angles = quaternion_to_doosan_zyz_deg(
                quaternion, reference_angles
            )
            orientation[index] = reference_angles
            max_step = max(
                max_step,
                quaternion_angle_deg(previous_quaternion, quaternion),
            )
            previous_quaternion = quaternion
        for index in range(cached_join_index + 1, count):
            quaternion = doosan_zyz_deg_to_quaternion(cached[index])
            reference_angles = quaternion_to_doosan_zyz_deg(
                quaternion, reference_angles
            )
            orientation[index] = reference_angles
            max_step = max(
                max_step,
                quaternion_angle_deg(previous_quaternion, quaternion),
            )
            previous_quaternion = quaternion
        return orientation, max_step

    target_quaternion = manifest.successor.nominal_orientation_quat_xyzw
    for index in range(count):
        fraction = (index + 1) / float(count)
        quaternion = quaternion_slerp(
            start_quaternion,
            target_quaternion,
            quintic_smoothstep(fraction),
        )
        reference_angles = quaternion_to_doosan_zyz_deg(quaternion, reference_angles)
        orientation[index] = reference_angles
        max_step = max(max_step, quaternion_angle_deg(previous_quaternion, quaternion))
        previous_quaternion = quaternion
    return orientation, max_step


def _severe_reversal(sampled: dict[str, np.ndarray], config: FlexibleBridgeSearchConfig) -> bool:
    velocity = sampled["velocity_mm_s"]
    speed = sampled["speed_mm_s"]
    valid = speed > config.cusp_speed_epsilon_mm_s
    indices = np.flatnonzero(valid)
    if len(indices) < 2:
        return False
    directions = velocity[indices] / speed[indices, None]
    cosine = np.sum(directions[1:] * directions[:-1], axis=1)
    return bool(np.any(cosine < config.severe_reversal_cosine))


def _has_cusp(sampled: dict[str, np.ndarray], config: FlexibleBridgeSearchConfig) -> bool:
    speed = sampled["speed_mm_s"]
    positions = sampled["position_mm"]
    for index in range(1, len(speed) - 1):
        if speed[index] > config.cusp_speed_epsilon_mm_s:
            continue
        before = positions[index] - positions[index - 1]
        after = positions[index + 1] - positions[index]
        denominator = float(np.linalg.norm(before) * np.linalg.norm(after))
        if denominator > 1.0e-9 and float(np.dot(before, after) / denominator) < -0.5:
            return True
    return False


def _has_self_intersection(sampled: dict[str, np.ndarray], tolerance_mm: float) -> bool:
    positions = sampled["position_mm"]
    velocity = sampled["velocity_mm_s"]
    speed = sampled["speed_mm_s"]
    separation = max(6, len(positions) // 20)
    for index in range(len(positions) - separation):
        if speed[index] < 5.0:
            continue
        offsets = np.flatnonzero(
            np.linalg.norm(
                positions[index + separation :] - positions[index], axis=1
            )
            < tolerance_mm
        )
        for offset in offsets:
            other = index + separation + int(offset)
            if speed[other] < 5.0:
                continue
            cosine = float(
                np.dot(velocity[index], velocity[other])
                / (speed[index] * speed[other])
            )
            # Slow progression along the same curve is not an intersection.
            # Only a nonlocal spatial revisit with a different tangent is
            # treated as the conservative sampled self-crossing signal.
            if abs(cosine) < 0.8:
                return True
    return False


def _reference_error_mm(candidate: Any, reference: Any) -> float:
    normalized = np.linspace(0.0, 1.0, 101)
    difference = candidate.position(normalized) - reference.position(normalized)
    return float(np.sqrt(np.mean(np.sum(difference**2, axis=1))))


def _build_queue_candidate(
    *,
    bridge: Any,
    reference: Any,
    manifest: EpisodeHandoffManifest,
    snapshot: BridgeRuntimeSnapshot,
    runtime_config: HandoffV2Config,
    limits: BridgeRuntimeLimits,
    search_config: FlexibleBridgeSearchConfig,
    generator_type: str,
    tangent_ratio: float,
    entry_fraction: float,
    exit_fraction: float,
    started_s: float,
    clock: Callable[[], float],
    cached_orientation_actions: np.ndarray | None = None,
    cached_join_index: int | None = None,
) -> tuple[PrecomputedBridgeQueue | None, FlexibleCandidateDiagnostics]:
    metrics, sampled, extra = _sample_metrics(
        bridge,
        limits,
        curvature_speed_epsilon_mm_s=(
            search_config.curvature_speed_epsilon_mm_s
        ),
    )
    reasons: list[str] = []
    if not metrics.finite:
        reasons.append("non_finite")
    if not metrics.workspace_satisfied:
        reasons.append("workspace_violation")
    if metrics.max_velocity_mm_s > limits.velocity_limit_mm_s + 1.0e-9:
        reasons.append("velocity_limit")
    maximum_axis_velocity = float(np.max(np.abs(sampled["velocity_mm_s"])))
    if maximum_axis_velocity > limits.axis_velocity_limit_mm_s + 1.0e-9:
        reasons.append("axis_velocity_limit")
    if metrics.max_acceleration_mm_s2 > limits.acceleration_limit_mm_s2 + 1.0e-9:
        reasons.append("acceleration_limit")
    if metrics.max_jerk_mm_s3 > limits.jerk_limit_mm_s3 + 1.0e-9:
        reasons.append("jerk_limit")
    if metrics.integrated_squared_jerk > limits.integrated_squared_jerk_limit + 1.0e-9:
        reasons.append("integrated_squared_jerk_limit")
    if metrics.backtracking_ratio > limits.backtracking_ratio_limit + 1.0e-9:
        reasons.append("backtracking_limit")
    if _has_cusp(sampled, search_config):
        reasons.append("cusp")
    if _severe_reversal(sampled, search_config):
        reasons.append("severe_direction_reversal")
    if _has_self_intersection(sampled, search_config.self_intersection_tolerance_mm):
        reasons.append("self_intersection")
    if manifest.transport_floor_mm is not None and float(np.min(sampled["position_mm"][:, 2])) < manifest.transport_floor_mm - 1.0e-6:
        reasons.append("transport_floor_violation")

    count = max(1, int(round(bridge.duration_s * runtime_config.control_hz)))
    normalized = np.arange(1, count + 1, dtype=np.float64) / float(count)
    positions = bridge.position(normalized)
    velocity = bridge.velocity(normalized)
    acceleration = bridge.acceleration(normalized)
    orientation, maximum_orientation_step = _orientation_actions(
        snapshot,
        manifest,
        positions,
        cached_orientation_actions=cached_orientation_actions,
        cached_join_index=cached_join_index,
    )
    gripper_target = (
        manifest.source.semantic.gripper_target
        if snapshot.gripper_target is None
        else snapshot.gripper_target
    )
    actions = np.concatenate(
        (positions, orientation, np.full((count, 1), gripper_target)),
        axis=1,
    )
    xyz_with_ack = np.concatenate(
        (snapshot.acknowledged_pose_mm_deg[:3][None, :], positions), axis=0
    )
    steps = np.diff(xyz_with_ack, axis=0)
    maximum_axis_step = float(np.max(np.abs(steps)))
    if maximum_axis_step > limits.linear_ramp_mm_per_tick + 1.0e-9:
        reasons.append("linear_ramp_limit")
    if maximum_orientation_step > limits.orientation_ramp_deg_per_tick + 1.0e-9:
        reasons.append("orientation_ramp_limit")

    dt = 1.0 / runtime_config.control_hz
    command_velocity = steps / dt
    velocity_with_actual = np.concatenate(
        (snapshot.actual_velocity_mm_s[None, :], command_velocity), axis=0
    )
    command_acceleration = np.diff(velocity_with_actual, axis=0) / dt
    command_jerk = (
        np.diff(command_acceleration, axis=0) / dt
        if len(command_acceleration) > 1
        else np.zeros((1, 3), dtype=np.float64)
    )
    max_command_acceleration = float(
        np.max(np.linalg.norm(command_acceleration, axis=1))
    )
    max_command_jerk = float(np.max(np.linalg.norm(command_jerk, axis=1)))
    if max_command_acceleration > limits.acceleration_limit_mm_s2 + 1.0e-9:
        reasons.append("command_acceleration_limit")
    if max_command_jerk > limits.jerk_limit_mm_s3 + 1.0e-9:
        reasons.append("command_jerk_limit")

    ack_span_steps = 1 + limits.ack_pipeline_max_lag_steps
    max_ack_axis_step = maximum_axis_step
    max_ack_orientation_step = maximum_orientation_step
    if ack_span_steps > 1 and len(xyz_with_ack) > ack_span_steps:
        max_ack_axis_step = float(
            np.max(np.abs(xyz_with_ack[ack_span_steps:] - xyz_with_ack[:-ack_span_steps]))
        )
        all_orientation = np.concatenate(
            (snapshot.acknowledged_pose_mm_deg[3:6][None, :], orientation), axis=0
        )
        max_ack_orientation_step = 0.0
        for index in range(ack_span_steps, len(all_orientation)):
            max_ack_orientation_step = max(
                max_ack_orientation_step,
                quaternion_angle_deg(
                    doosan_zyz_deg_to_quaternion(all_orientation[index - ack_span_steps]),
                    doosan_zyz_deg_to_quaternion(all_orientation[index]),
                ),
            )
        if max_ack_axis_step > limits.linear_ramp_mm_per_tick + 1.0e-9:
            reasons.append("ack_pipeline_linear_ramp_limit")
        if max_ack_orientation_step > limits.orientation_ramp_deg_per_tick + 1.0e-9:
            reasons.append("ack_pipeline_orientation_ramp_limit")

    reference_error = _reference_error_mm(bridge, reference)
    start_error = float(
        np.linalg.norm(
            snapshot.acknowledged_pose_mm_deg[:3]
            - manifest.source.nominal_position_mm
        )
    )
    support_radius = max(1.0, manifest.source.support_radius_mm)
    ood_score = start_error / support_radius
    curvature_penalty = max(
        0.0,
        extra["curvature_p95_moving_per_mm"] / limits.curvature_limit_per_mm - 1.0,
    )
    weights = search_config.weights
    candidate_start_velocity = np.asarray(bridge.velocity(0.0), dtype=np.float64)
    tangent_denominator = float(
        np.linalg.norm(snapshot.actual_velocity_mm_s)
        * np.linalg.norm(candidate_start_velocity)
    )
    source_tangent_angle_deg = (
        None
        if tangent_denominator <= 1.0e-9
        else float(
            np.degrees(
                np.arccos(
                    np.clip(
                        np.dot(
                            snapshot.actual_velocity_mm_s,
                            candidate_start_velocity,
                        )
                        / tangent_denominator,
                        -1.0,
                        1.0,
                    )
                )
            )
        )
    )
    score = (
        weights.reference * reference_error / support_radius
        + weights.support * start_error / support_radius
        + weights.velocity
        * float(np.linalg.norm(snapshot.actual_velocity_mm_s - manifest.source.nominal_velocity_mm_s))
        / max(1.0, limits.velocity_limit_mm_s)
        + weights.curvature * curvature_penalty
        + weights.acceleration * metrics.max_acceleration_mm_s2 / limits.acceleration_limit_mm_s2
        + weights.jerk * metrics.max_jerk_mm_s3 / limits.jerk_limit_mm_s3
        + weights.duration * bridge.duration_s / manifest.bridge_duration_s
        + weights.length * metrics.length_mm / max(1.0, manifest.nominal_length_mm or metrics.length_mm)
        + weights.ood * ood_score
    )
    unique_reasons = tuple(dict.fromkeys(reasons))
    diagnostics = FlexibleCandidateDiagnostics(
        generator_type=generator_type,
        tangent_handle_chord_ratio=tangent_ratio,
        duration_s=bridge.duration_s,
        entry_fraction=entry_fraction,
        exit_fraction=exit_fraction,
        max_curvature_per_mm=metrics.max_curvature_per_mm,
        curvature_p95_moving_per_mm=extra["curvature_p95_moving_per_mm"],
        max_curvature_u=extra["max_curvature_u"],
        speed_at_max_curvature_mm_s=extra["speed_at_max_curvature_mm_s"],
        source_tangent_angle_deg=source_tangent_angle_deg,
        max_velocity_mm_s=metrics.max_velocity_mm_s,
        max_acceleration_mm_s2=metrics.max_acceleration_mm_s2,
        max_jerk_mm_s3=metrics.max_jerk_mm_s3,
        max_normal_acceleration_mm_s2=extra["max_normal_acceleration_mm_s2"],
        max_command_acceleration_mm_s2=max_command_acceleration,
        max_command_jerk_mm_s3=max_command_jerk,
        max_position_axis_step_mm=maximum_axis_step,
        max_orientation_step_deg=maximum_orientation_step,
        max_ack_span_axis_step_mm=max_ack_axis_step,
        max_ack_span_orientation_step_deg=max_ack_orientation_step,
        minimum_position_z_mm=float(
            np.min(sampled["position_mm"][:, 2])
        ),
        reference_rms_error_mm=reference_error,
        actual_reference_start_error_mm=start_error,
        ood_score=ood_score,
        hard_rejection_reasons=unique_reasons,
        score=None if unique_reasons else float(score),
    )
    if unique_reasons:
        return None, diagnostics
    queue = PrecomputedBridgeQueue(
        bridge=bridge,
        metrics=metrics,
        actions=actions,
        velocity_mm_s=velocity,
        acceleration_mm_s2=acceleration,
        handoff_window_start_index=max(0, count - runtime_config.handoff_window_steps),
        source_snapshot=snapshot,
        generation_latency_s=max(0.0, float(clock()) - started_s),
        maximum_axis_velocity_mm_s=maximum_axis_velocity,
        maximum_axis_step_mm=maximum_axis_step,
        maximum_orientation_step_deg=maximum_orientation_step,
        ack_span_steps=ack_span_steps,
        maximum_ack_span_axis_step_mm=max_ack_axis_step,
        maximum_ack_span_orientation_step_deg=max_ack_orientation_step,
        transport_floor_mm=manifest.transport_floor_mm,
        workspace_min_limit_enabled=limits.workspace_min_limit_enabled,
        bridge_algorithm="reference_guided_c2_v1",
        minimum_tangent_handle_chord_ratio=tangent_ratio,
        maximum_endpoint_speed_adjustment_mm_s=None,
        source_endpoint_speed_adjustment_mm_s=0.0,
        successor_endpoint_speed_adjustment_mm_s=0.0,
        ik_checked=manifest.ik_checked,
        collision_checked=manifest.collision_checked,
        bridge_mode="flexible_level2",
        candidate_generator_type=generator_type,
        selected_candidate_score=float(score),
        candidate_diagnostics=diagnostics.record(),
    )
    return queue, diagnostics


def _candidate_specs(config: FlexibleBridgeSearchConfig) -> Iterable[tuple[str, float, float, float, float]]:
    primary_window = config.reference_join_windows[0]
    if config.enable_reference_deformation:
        for duration in config.duration_multipliers:
            yield ("reference_deformation", 0.0, duration, *primary_window)
    if config.enable_tangent_regularization:
        for ratio in config.tangent_handle_chord_ratios:
            if ratio <= 0.0:
                continue
            for duration in config.duration_multipliers:
                yield ("tangent_regularized_deformation", ratio, duration, *primary_window)
    if config.enable_settle_connector:
        for entry, exit_ in config.reference_join_windows[1:]:
            for duration in config.duration_multipliers:
                yield ("settle_reference_connector", 0.0, duration, entry, exit_)
    if config.enable_alignment_connector:
        # A shorter C2 entry/exit connector joins the representative middle
        # sooner. It is tried only after the gentler settle candidates fail.
        # The group remains bounded by the same duration/config limits.
        for duration in config.duration_multipliers:
            yield (
                "alignment_reference_connector",
                0.05,
                duration,
                0.12,
                0.88,
            )
    if config.enable_lift_transport_connector:
        for duration in config.duration_multipliers:
            yield ("lift_transport_reference_connector", 0.0, duration, 0.25, 0.75)


def _affine_reference_deformation(
    reference: CubicBezierBridge,
    *,
    start_position_mm: np.ndarray,
    target_position_mm: np.ndarray,
) -> CubicBezierBridge:
    """Warp only the endpoints while retaining the representative cubic shape."""

    start = np.asarray(start_position_mm, dtype=np.float64)
    target = np.asarray(target_position_mm, dtype=np.float64)
    start_delta = start - reference.p0
    target_delta = target - reference.p3
    return CubicBezierBridge(
        p0=reference.p0 + start_delta,
        p1=reference.p1 + (2.0 * start_delta + target_delta) / 3.0,
        p2=reference.p2 + (start_delta + 2.0 * target_delta) / 3.0,
        p3=reference.p3 + target_delta,
        duration_s=reference.duration_s,
    )


def search_flexible_bridge_queue(
    manifest: EpisodeHandoffManifest,
    snapshot: BridgeRuntimeSnapshot,
    runtime_config: HandoffV2Config,
    limits: BridgeRuntimeLimits,
    search_config: FlexibleBridgeSearchConfig,
    *,
    live_mode: bool,
    clock: Callable[[], float] = time.perf_counter,
) -> FlexibleBridgeSearchResult:
    """Bounded candidate search. Call from a worker, not the command loop."""

    started_s = float(clock())
    selected_queue: PrecomputedBridgeQueue | None = None
    selected_diagnostics: FlexibleCandidateDiagnostics | None = None
    evaluated = 0
    passed = 0
    timed_out = False
    rejected: dict[str, int] = {}
    candidate_diagnostics: list[FlexibleCandidateDiagnostics] = []
    target_position = manifest.successor.nominal_position_mm
    target_velocity = manifest.successor.nominal_velocity_mm_s

    current_group: str | None = None
    group_passes: list[tuple[PrecomputedBridgeQueue, FlexibleCandidateDiagnostics]] = []
    for generator, ratio, multiplier, entry, exit_ in _candidate_specs(search_config):
        if evaluated >= search_config.max_candidates:
            break
        if float(clock()) - started_s > search_config.max_search_time_s:
            timed_out = True
            break
        if current_group is not None and generator != current_group and group_passes:
            break
        current_group = generator
        duration = manifest.bridge_duration_s * multiplier
        reference = _nominal_reference(manifest, duration)
        lift = (
            search_config.lift_clearance_mm
            if generator == "lift_transport_reference_connector"
            else 0.0
        )
        if generator == "reference_deformation":
            bridge = _affine_reference_deformation(
                reference,
                start_position_mm=snapshot.acknowledged_pose_mm_deg[:3],
                target_position_mm=target_position,
            )
        else:
            bridge = ReferenceGuidedBridge(
                reference=reference,
                start_position_mm=snapshot.acknowledged_pose_mm_deg[:3],
                start_velocity_mm_s=snapshot.actual_velocity_mm_s,
                target_position_mm=target_position,
                target_velocity_mm_s=target_velocity,
                duration_s=duration,
                entry_fraction=entry,
                exit_fraction=exit_,
                tangent_handle_chord_ratio=ratio,
                lift_clearance_mm=lift,
            )
        queue, diagnostics = _build_queue_candidate(
            bridge=bridge,
            reference=reference,
            manifest=manifest,
            snapshot=snapshot,
            runtime_config=runtime_config,
            limits=limits,
            search_config=search_config,
            generator_type=generator,
            tangent_ratio=ratio,
            entry_fraction=entry,
            exit_fraction=exit_,
            started_s=started_s,
            clock=clock,
        )
        evaluated += 1
        candidate_diagnostics.append(diagnostics)
        if queue is None:
            for reason in diagnostics.hard_rejection_reasons:
                rejected[reason] = rejected.get(reason, 0) + 1
            continue
        passed += 1
        group_passes.append((queue, diagnostics))

    if group_passes:
        selected_queue, selected_diagnostics = min(
            group_passes,
            key=lambda value: float(value[1].score),
        )
    elif search_config.enable_generic_safe_connector and (
        not live_mode or search_config.allow_generic_safe_connector_live
    ):
        # This fallback is deliberately still command-validated and OOD-tagged.
        # It is not enabled for live mode by default because no collision/IK
        # checker is available in the current repository.
        reference = _nominal_reference(manifest, manifest.bridge_duration_s * 1.5)
        bridge = ReferenceGuidedBridge(
            reference=reference,
            start_position_mm=snapshot.acknowledged_pose_mm_deg[:3],
            start_velocity_mm_s=snapshot.actual_velocity_mm_s,
            target_position_mm=target_position,
            target_velocity_mm_s=target_velocity,
            duration_s=manifest.bridge_duration_s * 1.5,
            entry_fraction=0.4,
            exit_fraction=0.6,
        )
        queue, diagnostics = _build_queue_candidate(
            bridge=bridge,
            reference=reference,
            manifest=manifest,
            snapshot=snapshot,
            runtime_config=runtime_config,
            limits=limits,
            search_config=search_config,
            generator_type="generic_command_validated_ood",
            tangent_ratio=0.0,
            entry_fraction=0.4,
            exit_fraction=0.6,
            started_s=started_s,
            clock=clock,
        )
        evaluated += 1
        candidate_diagnostics.append(diagnostics)
        if queue is not None:
            passed += 1
            selected_queue, selected_diagnostics = queue, diagnostics
        else:
            for reason in diagnostics.hard_rejection_reasons:
                rejected[reason] = rejected.get(reason, 0) + 1

    latency = max(0.0, float(clock()) - started_s)
    if selected_queue is not None:
        selected_queue = PrecomputedBridgeQueue(
            **{
                **selected_queue.__dict__,
                "generation_latency_s": latency,
            }
        )
    return FlexibleBridgeSearchResult(
        queue=selected_queue,
        candidates_evaluated=evaluated,
        candidates_hard_passed=passed,
        search_latency_s=latency,
        selected=selected_diagnostics,
        rejected_reason_counts=rejected,
        timed_out=timed_out,
        candidate_diagnostics=tuple(candidate_diagnostics),
    )


def rebase_cached_flexible_bridge(
    template: FlexibleBridgeTemplate,
    manifest: EpisodeHandoffManifest,
    snapshot: BridgeRuntimeSnapshot,
    runtime_config: HandoffV2Config,
    limits: BridgeRuntimeLimits,
    search_config: FlexibleBridgeSearchConfig,
    *,
    clock: Callable[[], float] = time.perf_counter,
) -> FlexibleBridgeSearchResult:
    """Connect fresh actual/ACK state to a bounded *future* reference join.

    Heavy reference-family search remains cached. At commit, the newest ACK is
    projected onto the early/middle portion of that cached reference and a
    small, bounded set of strictly-future C2 join points is evaluated. The
    command loop never performs this search and never waits for it.
    """

    started_s = float(clock())
    if template.handoff_id != manifest.handoff_id:
        raise ValueError("cached Bridge template belongs to another handoff")
    cached = template.queue
    selected = template.search.selected
    if selected is None:
        raise ValueError("cached Bridge template lacks selected diagnostics")
    count = max(
        1,
        int(round(cached.bridge.duration_s * runtime_config.control_hz)),
    )
    if cached.actions.shape != (count, 7):
        raise ValueError("cached Bridge queue no longer matches runtime control rate")
    if count < 3:
        raise ValueError("cached Bridge queue is too short for adaptive entry")

    projection_limit = max(
        0,
        min(
            count - 2,
            int(
                math.floor(
                    search_config.adaptive_entry_projection_max_fraction
                    * count
                )
            ),
        ),
    )
    projection_u = np.arange(
        projection_limit + 1, dtype=np.float64
    ) / float(count)
    projection_positions = cached.bridge.position(projection_u)
    projection_index = int(
        np.argmin(
            np.linalg.norm(
                projection_positions
                - snapshot.acknowledged_pose_mm_deg[:3][None, :],
                axis=1,
            )
        )
    )
    projection_fraction = projection_index / float(count)

    requested_join = float(selected.entry_fraction)
    legacy_join_index = max(
        1,
        min(count - 2, int(math.ceil(requested_join * count)) - 1),
    )
    maximum_join_index = max(
        1,
        min(
            count - 2,
            int(
                math.floor(
                    search_config.adaptive_entry_max_join_fraction * count
                )
            )
            - 1,
        ),
    )
    candidate_indices: list[int] = []
    if legacy_join_index > projection_index:
        candidate_indices.append(legacy_join_index)
    for lookahead in search_config.adaptive_entry_join_lookahead_fractions:
        index = projection_index + max(1, int(math.ceil(lookahead * count)))
        index = min(maximum_join_index, max(1, index))
        if index > projection_index:
            candidate_indices.append(index)
    # Deterministic de-duplication preserves the legacy candidate first, then
    # increasingly farther future joins.
    candidate_indices = list(dict.fromkeys(candidate_indices))
    candidate_indices = candidate_indices[
        : search_config.adaptive_entry_max_candidates
    ]

    evaluated = 0
    passed: list[
        tuple[PrecomputedBridgeQueue, FlexibleCandidateDiagnostics]
    ] = []
    rejected: dict[str, int] = {}
    diagnostics_records: list[FlexibleCandidateDiagnostics] = []
    timed_out = False
    for join_index in candidate_indices:
        if (
            float(clock()) - started_s
            > search_config.adaptive_entry_max_search_time_s
        ):
            timed_out = True
            break
        join_fraction = (join_index + 1) / float(count)
        bridge = CachedReferenceAdaptiveBridge(
            reference=cached.bridge,
            start_position_mm=snapshot.acknowledged_pose_mm_deg[:3],
            start_velocity_mm_s=snapshot.actual_velocity_mm_s,
            join_fraction=join_fraction,
        )
        queue, diagnostics = _build_queue_candidate(
            bridge=bridge,
            reference=cached.bridge,
            manifest=manifest,
            snapshot=snapshot,
            runtime_config=runtime_config,
            limits=limits,
            search_config=search_config,
            generator_type="cached_reference_dynamic_future_join",
            tangent_ratio=selected.tangent_handle_chord_ratio,
            entry_fraction=join_fraction,
            exit_fraction=selected.exit_fraction,
            started_s=started_s,
            clock=clock,
            cached_orientation_actions=cached.actions[:, 3:6],
            cached_join_index=join_index,
        )
        diagnostics = replace(
            diagnostics,
            reference_projection_fraction=projection_fraction,
            future_join_lookahead_fraction=(
                join_fraction - projection_fraction
            ),
        )
        evaluated += 1
        diagnostics_records.append(diagnostics)
        if queue is None:
            for reason in diagnostics.hard_rejection_reasons:
                rejected[reason] = rejected.get(reason, 0) + 1
            continue
        queue = PrecomputedBridgeQueue(
            **{
                **queue.__dict__,
                "candidate_diagnostics": diagnostics.record(),
                "selected_candidate_score": float(diagnostics.score),
            }
        )
        passed.append((queue, diagnostics))

    latency = max(0.0, float(clock()) - started_s)
    if not passed:
        return FlexibleBridgeSearchResult(
            queue=None,
            candidates_evaluated=evaluated,
            candidates_hard_passed=0,
            search_latency_s=latency,
            selected=None,
            rejected_reason_counts=rejected,
            timed_out=timed_out,
            candidate_diagnostics=tuple(diagnostics_records),
        )

    queue, diagnostics = min(
        passed,
        key=lambda item: float(item[1].score),
    )
    queue = PrecomputedBridgeQueue(
        **{
            **queue.__dict__,
            "generation_latency_s": latency,
            "bridge_algorithm": (
                "cached_reference_dynamic_future_join_c2_v2"
            ),
        }
    )
    return FlexibleBridgeSearchResult(
        queue=queue,
        candidates_evaluated=evaluated,
        candidates_hard_passed=len(passed),
        search_latency_s=latency,
        selected=diagnostics,
        rejected_reason_counts=rejected,
        timed_out=timed_out,
        candidate_diagnostics=tuple(diagnostics_records),
    )


def validate_flexible_entry_rebase(
    queue: PrecomputedBridgeQueue,
    current_snapshot: BridgeRuntimeSnapshot,
    limits: BridgeRuntimeLimits,
) -> tuple[bool, tuple[str, ...], dict[str, float]]:
    """Revalidate a worker result against the newest ACK before A invalidation."""

    first = queue.actions[0]
    axis_delta = np.abs(first[:3] - current_snapshot.acknowledged_pose_mm_deg[:3])
    rotation_delta = quaternion_angle_deg(
        doosan_zyz_deg_to_quaternion(current_snapshot.acknowledged_pose_mm_deg[3:6]),
        doosan_zyz_deg_to_quaternion(first[3:6]),
    )
    reasons: list[str] = []
    if float(np.max(axis_delta)) > limits.linear_ramp_mm_per_tick + 1.0e-9:
        reasons.append("stale_entry_linear_ramp")
    if rotation_delta > limits.orientation_ramp_deg_per_tick + 1.0e-9:
        reasons.append("stale_entry_orientation_ramp")
    gripper_delta = 0.0
    if current_snapshot.gripper_target is not None:
        gripper_delta = abs(
            float(first[6]) - float(current_snapshot.gripper_target)
        )
        if gripper_delta > 1.0e-9:
            reasons.append("stale_entry_gripper")
    if len(queue.actions) >= 2:
        dt = queue.bridge.duration_s / float(queue.steps)
        first_velocity = (first[:3] - current_snapshot.acknowledged_pose_mm_deg[:3]) / dt
        acceleration = float(
            np.linalg.norm(first_velocity - current_snapshot.actual_velocity_mm_s) / dt
        )
        if acceleration > limits.acceleration_limit_mm_s2 + 1.0e-9:
            reasons.append("stale_entry_acceleration")
    else:
        acceleration = 0.0
    return (
        not reasons,
        tuple(reasons),
        {
            "max_entry_axis_delta_mm": float(np.max(axis_delta)),
            "entry_rotation_delta_deg": float(rotation_delta),
            "entry_gripper_delta": gripper_delta,
            "entry_acceleration_mm_s2": acceleration,
        },
    )


@dataclass(frozen=True)
class AsyncFlexibleBridgeResult:
    status: str
    generation: int | None
    search: FlexibleBridgeSearchResult | None = None
    stale: bool = False
    failure_reason: str | None = None
    request_kind: str | None = None


class AsyncFlexibleBridgePlanner:
    """Single-inflight generation-isolated Bridge planning worker."""

    def __init__(
        self,
        *,
        max_result_age_s: float = 0.30,
        backend: str = "thread",
        cpu_set: tuple[int, ...] | None = None,
        nice_value: int = 10,
    ) -> None:
        if not np.isfinite(max_result_age_s) or max_result_age_s <= 0.0:
            raise ValueError("max_result_age_s must be positive")
        if backend not in {"thread", "process"}:
            raise ValueError("Bridge worker backend must be thread or process")
        if not isinstance(nice_value, int) or not 0 <= nice_value <= 19:
            raise ValueError("Bridge worker nice value must be in [0, 19]")
        normalized_cpu_set = (
            None
            if cpu_set is None
            else tuple(sorted(dict.fromkeys(int(value) for value in cpu_set)))
        )
        if backend == "process" and not normalized_cpu_set:
            raise ValueError("process Bridge worker requires an explicit CPU set")
        self.max_result_age_s = float(max_result_age_s)
        self.backend = backend
        self.cpu_set = normalized_cpu_set
        self.nice_value = nice_value
        if backend == "process":
            self._executor = concurrent.futures.ProcessPoolExecutor(
                max_workers=1,
                mp_context=multiprocessing.get_context("spawn"),
                initializer=_bridge_worker_initializer,
                initargs=(normalized_cpu_set, nice_value),
            )
        else:
            self._executor = concurrent.futures.ThreadPoolExecutor(
                max_workers=1,
                thread_name_prefix="a0509-flexible-bridge",
            )
        self._lock = threading.Lock()
        self._generation = 0
        self._future: concurrent.futures.Future[FlexibleBridgeSearchResult] | None = None
        self._request_snapshot_timestamp_s: float | None = None
        self._request_kind: str | None = None
        self._freshness_required = True
        self._worker_info: dict[str, Any] | None = None

    def start(self, *, timeout_s: float = 15.0) -> dict[str, Any]:
        """Warm the process during setup, never during the 30 Hz loop."""

        if self.backend == "process":
            assert self.cpu_set is not None
            # ProcessPoolExecutor starts its worker lazily. Linux inherits the
            # affinity of this spawning thread, which setup-time ACT warmup may
            # have narrowed. Spawn on the requested Bridge CPUs, then restore
            # the caller before the fixed-rate loop can start.
            with temporary_current_thread_affinity(self.cpu_set):
                future = self._executor.submit(_bridge_worker_identity)
                info = dict(future.result(timeout=timeout_s))
        else:
            info = {
                "pid": os.getpid(),
                "cpu_set": sorted(os.sched_getaffinity(0)),
                "nice": os.getpriority(os.PRIO_PROCESS, 0),
                "backend": "thread",
            }
        self._worker_info = info
        return dict(info)

    @property
    def worker_info(self) -> dict[str, Any] | None:
        return None if self._worker_info is None else dict(self._worker_info)

    @property
    def inflight_generation(self) -> int | None:
        with self._lock:
            return self._generation if self._future is not None else None

    @property
    def inflight_kind(self) -> str | None:
        with self._lock:
            return self._request_kind if self._future is not None else None

    def _submit(
        self,
        request_kind: str,
        snapshot_timestamp_s: float,
        freshness_required: bool,
        function: Callable[..., FlexibleBridgeSearchResult],
        *args: Any,
        **kwargs: Any,
    ) -> int | None:
        with self._lock:
            if self._future is not None:
                return None
            self._generation += 1
            generation = self._generation
            self._request_snapshot_timestamp_s = float(snapshot_timestamp_s)
            self._request_kind = request_kind
            self._freshness_required = bool(freshness_required)
            self._future = self._executor.submit(function, *args, **kwargs)
            return generation

    def request(
        self,
        manifest: EpisodeHandoffManifest,
        snapshot: BridgeRuntimeSnapshot,
        runtime_config: HandoffV2Config,
        limits: BridgeRuntimeLimits,
        search_config: FlexibleBridgeSearchConfig,
        *,
        live_mode: bool,
    ) -> int | None:
        # Backward-compatible one-shot request. Its complete result is fresh.
        return self._submit(
            "legacy_search",
            snapshot.timestamp_s,
            True,
            search_flexible_bridge_queue,
            manifest,
            snapshot,
            runtime_config,
            limits,
            search_config,
            live_mode=live_mode,
        )

    def request_template(
        self,
        manifest: EpisodeHandoffManifest,
        snapshot: BridgeRuntimeSnapshot,
        runtime_config: HandoffV2Config,
        limits: BridgeRuntimeLimits,
        search_config: FlexibleBridgeSearchConfig,
        *,
        live_mode: bool,
    ) -> int | None:
        # A template is edge-scoped reference geometry, not a live command.
        # Its age therefore must not invalidate the expensive search result.
        return self._submit(
            "template_search",
            snapshot.timestamp_s,
            False,
            search_flexible_bridge_queue,
            manifest,
            snapshot,
            runtime_config,
            limits,
            search_config,
            live_mode=live_mode,
        )

    def request_rebase(
        self,
        template: FlexibleBridgeTemplate,
        manifest: EpisodeHandoffManifest,
        snapshot: BridgeRuntimeSnapshot,
        runtime_config: HandoffV2Config,
        limits: BridgeRuntimeLimits,
        search_config: FlexibleBridgeSearchConfig,
    ) -> int | None:
        return self._submit(
            "adaptive_rebase",
            snapshot.timestamp_s,
            True,
            rebase_cached_flexible_bridge,
            template,
            manifest,
            snapshot,
            runtime_config,
            limits,
            search_config,
        )

    def poll(self, *, now_s: float) -> AsyncFlexibleBridgeResult:
        with self._lock:
            future = self._future
            generation = self._generation if future is not None else None
            snapshot_timestamp = self._request_snapshot_timestamp_s
            request_kind = self._request_kind
            freshness_required = self._freshness_required
        if future is None:
            return AsyncFlexibleBridgeResult("idle", None)
        if not future.done():
            return AsyncFlexibleBridgeResult(
                "pending", generation, request_kind=request_kind
            )
        try:
            search = future.result()
        except BaseException as exc:
            search = None
            failure = f"{type(exc).__name__}: {exc}"
        else:
            failure = None
        with self._lock:
            if future is not self._future:
                return AsyncFlexibleBridgeResult(
                    "rejected",
                    generation,
                    stale=True,
                    failure_reason="generation_mismatch",
                    request_kind=request_kind,
                )
            self._future = None
            self._request_snapshot_timestamp_s = None
            self._request_kind = None
            self._freshness_required = True
        age = math.inf if snapshot_timestamp is None else float(now_s - snapshot_timestamp)
        if freshness_required and age > self.max_result_age_s:
            return AsyncFlexibleBridgeResult(
                "rejected",
                generation,
                search=search,
                stale=True,
                failure_reason=(
                    "stale_adaptive_rebase_result"
                    if request_kind == "adaptive_rebase"
                    else "stale_bridge_result"
                ),
                request_kind=request_kind,
            )
        if failure is not None:
            return AsyncFlexibleBridgeResult(
                "failed",
                generation,
                failure_reason=failure,
                request_kind=request_kind,
            )
        assert search is not None
        if not search.valid:
            return AsyncFlexibleBridgeResult(
                "rejected",
                generation,
                search=search,
                failure_reason="no_command_safe_candidate",
                request_kind=request_kind,
            )
        return AsyncFlexibleBridgeResult(
            "ready", generation, search=search, request_kind=request_kind
        )

    def wait_for_result(
        self,
        *,
        timeout_s: float,
        poll_interval_s: float = 0.005,
    ) -> AsyncFlexibleBridgeResult:
        """Bounded setup-only wait for a nominal template search.

        The live 30 Hz path uses poll exclusively. This helper exists so all
        expensive nominal-medoid work can be completed before external Live
        authorization. A timeout invalidates the generation fail-closed.
        """

        if not np.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("Bridge worker wait timeout must be positive")
        if not np.isfinite(poll_interval_s) or poll_interval_s <= 0.0:
            raise ValueError("Bridge worker poll interval must be positive")
        deadline_s = time.monotonic() + float(timeout_s)
        while True:
            result = self.poll(now_s=time.monotonic())
            if result.status != "pending":
                return result
            remaining_s = deadline_s - time.monotonic()
            if remaining_s <= 0.0:
                generation = result.generation
                request_kind = result.request_kind
                self.invalidate()
                return AsyncFlexibleBridgeResult(
                    "failed",
                    generation,
                    failure_reason="setup_template_timeout",
                    request_kind=request_kind,
                )
            time.sleep(min(float(poll_interval_s), remaining_s))

    def invalidate(self) -> None:
        with self._lock:
            self._generation += 1
            future = self._future
            self._future = None
            self._request_snapshot_timestamp_s = None
            self._request_kind = None
            self._freshness_required = True
        if future is not None:
            future.cancel()

    def close(self) -> None:
        self.invalidate()
        self._executor.shutdown(wait=False, cancel_futures=True)


def require_flexible_queue(result: FlexibleBridgeSearchResult) -> PrecomputedBridgeQueue:
    if result.queue is None:
        reasons = list(result.rejected_reason_counts) or ["no_command_safe_candidate"]
        raise BridgeGenerationError(reasons)
    return result.queue
