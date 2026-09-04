"""Geometric and dynamic metrics for a sampled cubic Bezier bridge."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .bezier_bridge import CubicBezierBridge


@dataclass(frozen=True)
class BridgeMetrics:
    length_mm: float
    max_velocity_mm_s: float
    max_acceleration_mm_s2: float
    max_curvature_per_mm: float
    max_jerk_mm_s3: float
    integrated_squared_jerk: float
    backtracking_ratio: float
    workspace_satisfied: bool
    finite: bool


def bridge_partial_length_mm(
    bridge: CubicBezierBridge,
    *,
    u_start: float = 0.0,
    u_end: float = 1.0,
    sample_hz: float = 60.0,
) -> float:
    """Approximate arc length over a normalized sub-interval."""

    start = float(u_start)
    end = float(u_end)
    if not (0.0 <= start <= end <= 1.0):
        raise ValueError("Bezier arc interval must satisfy 0 <= start <= end <= 1")
    if not np.isfinite(sample_hz) or sample_hz <= 0.0:
        raise ValueError("sample_hz must be finite and positive")
    if start == end:
        return 0.0
    interval_duration = (end - start) * bridge.duration_s
    count = max(2, int(np.ceil(interval_duration * sample_hz)) + 1)
    position = bridge.position(np.linspace(start, end, count))
    return float(np.sum(np.linalg.norm(np.diff(position, axis=0), axis=1)))


def evaluate_bridge(
    bridge: CubicBezierBridge,
    *,
    sample_hz: float,
    curvature_epsilon: float,
    workspace_min_mm: np.ndarray,
    workspace_max_mm: np.ndarray,
    workspace_min_limit_enabled: np.ndarray | None = None,
) -> tuple[BridgeMetrics, dict[str, np.ndarray]]:
    minimum_enabled = (
        np.ones(3, dtype=np.bool_)
        if workspace_min_limit_enabled is None
        else np.asarray(workspace_min_limit_enabled, dtype=np.bool_)
    )
    if minimum_enabled.shape != (3,):
        raise ValueError("workspace_min_limit_enabled must contain XYZ")
    sample = bridge.sample(sample_hz)
    position = sample["position_mm"]
    velocity = sample["velocity_mm_s"]
    acceleration = sample["acceleration_mm_s2"]
    jerk = sample["jerk_mm_s3"]
    speed = np.linalg.norm(velocity, axis=1)
    acceleration_magnitude = np.linalg.norm(acceleration, axis=1)
    jerk_magnitude = np.linalg.norm(jerk, axis=1)
    cross = np.linalg.norm(np.cross(velocity, acceleration), axis=1)
    curvature = cross / np.maximum(speed**3, curvature_epsilon)
    increments = np.linalg.norm(np.diff(position, axis=0), axis=1)
    length = float(np.sum(increments))
    chord = float(np.linalg.norm(bridge.p3 - bridge.p0))
    backtracking = length / max(chord, 1e-9)
    minimum = np.asarray(workspace_min_mm)
    maximum = np.asarray(workspace_max_mm)
    workspace = bool(
        np.all(
            position[:, minimum_enabled]
            >= minimum[None, minimum_enabled]
        )
        and np.all(position <= maximum[None, :])
    )
    arrays = [position, velocity, acceleration, jerk, curvature]
    finite = all(np.all(np.isfinite(array)) for array in arrays)
    trapezoid = getattr(np, "trapezoid", np.trapz)
    integrated_jerk = float(trapezoid(jerk_magnitude**2, x=sample["time_s"]))
    sample["speed_mm_s"] = speed
    sample["acceleration_magnitude_mm_s2"] = acceleration_magnitude
    sample["jerk_magnitude_mm_s3"] = jerk_magnitude
    sample["curvature_per_mm"] = curvature
    return (
        BridgeMetrics(
            length_mm=length,
            max_velocity_mm_s=float(np.max(speed)),
            max_acceleration_mm_s2=float(np.max(acceleration_magnitude)),
            max_curvature_per_mm=float(np.max(curvature)),
            max_jerk_mm_s3=float(np.max(jerk_magnitude)),
            integrated_squared_jerk=integrated_jerk,
            backtracking_ratio=backtracking,
            workspace_satisfied=workspace,
            finite=finite,
        ),
        sample,
    )


def feasibility_reasons(metrics: BridgeMetrics, limits: dict[str, float]) -> list[str]:
    reasons: list[str] = []
    if not metrics.finite:
        reasons.append("non_finite")
    if not metrics.workspace_satisfied:
        reasons.append("workspace_violation")
    checks = (
        (metrics.max_velocity_mm_s, limits["velocity_limit_mm_s"], "velocity_limit"),
        (
            metrics.max_acceleration_mm_s2,
            limits["acceleration_limit_mm_s2"],
            "acceleration_limit",
        ),
        (
            metrics.max_curvature_per_mm,
            limits["curvature_limit_per_mm"],
            "curvature_limit",
        ),
        (metrics.max_jerk_mm_s3, limits["jerk_limit_mm_s3"], "jerk_limit"),
        (
            metrics.integrated_squared_jerk,
            limits["integrated_squared_jerk_limit"],
            "integrated_squared_jerk_limit",
        ),
        (metrics.backtracking_ratio, limits["backtracking_ratio_limit"], "backtracking_limit"),
    )
    for actual, threshold, label in checks:
        if actual > threshold:
            reasons.append(label)
    return reasons
