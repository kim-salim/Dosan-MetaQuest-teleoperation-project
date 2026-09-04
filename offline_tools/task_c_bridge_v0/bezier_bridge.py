"""Velocity-constrained cubic Bezier geometry."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


CUBIC_BEZIER_FIXED = "cubic_bezier_fixed"
CUBIC_BEZIER_TANGENT_REGULARIZED_V1 = (
    "cubic_bezier_tangent_regularized_v1"
)


@dataclass(frozen=True)
class TangentRegularizationDiagnostics:
    """Audit values for the opt-in low-speed endpoint regularization.

    The regularizer preserves both endpoint tangent directions.  It only
    increases a tangent magnitude when the velocity-matched control handle is
    shorter than a configured fraction of the endpoint chord.
    """

    chord_length_mm: float
    minimum_handle_mm: float
    source_raw_handle_mm: float
    successor_raw_handle_mm: float
    source_applied_handle_mm: float
    successor_applied_handle_mm: float
    source_speed_adjustment_mm_s: float
    successor_speed_adjustment_mm_s: float

    @property
    def maximum_speed_adjustment_mm_s(self) -> float:
        return max(
            self.source_speed_adjustment_mm_s,
            self.successor_speed_adjustment_mm_s,
        )


@dataclass(frozen=True)
class CubicBezierBridge:
    p0: np.ndarray
    p1: np.ndarray
    p2: np.ndarray
    p3: np.ndarray
    duration_s: float

    def __post_init__(self) -> None:
        if self.duration_s <= 0.0:
            raise ValueError("Bezier duration must be positive")
        for name in ("p0", "p1", "p2", "p3"):
            value = np.asarray(getattr(self, name), dtype=np.float64)
            if value.shape != (3,) or not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must be a finite XYZ vector")
            object.__setattr__(self, name, value)

    def position(self, u: np.ndarray | float) -> np.ndarray:
        u = np.asarray(u, dtype=np.float64)
        one = 1.0 - u
        return (
            one[..., None] ** 3 * self.p0
            + 3.0 * one[..., None] ** 2 * u[..., None] * self.p1
            + 3.0 * one[..., None] * u[..., None] ** 2 * self.p2
            + u[..., None] ** 3 * self.p3
        )

    def velocity(self, u: np.ndarray | float) -> np.ndarray:
        u = np.asarray(u, dtype=np.float64)
        one = 1.0 - u
        derivative_u = 3.0 * (
            one[..., None] ** 2 * (self.p1 - self.p0)
            + 2.0 * one[..., None] * u[..., None] * (self.p2 - self.p1)
            + u[..., None] ** 2 * (self.p3 - self.p2)
        )
        return derivative_u / self.duration_s

    def acceleration(self, u: np.ndarray | float) -> np.ndarray:
        u = np.asarray(u, dtype=np.float64)
        one = 1.0 - u
        derivative_uu = 6.0 * (
            one[..., None] * (self.p2 - 2.0 * self.p1 + self.p0)
            + u[..., None] * (self.p3 - 2.0 * self.p2 + self.p1)
        )
        return derivative_uu / self.duration_s**2

    def jerk(self, u: np.ndarray | float) -> np.ndarray:
        u = np.asarray(u, dtype=np.float64)
        constant = 6.0 * (self.p3 - 3.0 * self.p2 + 3.0 * self.p1 - self.p0)
        return np.broadcast_to(constant / self.duration_s**3, u.shape + (3,)).copy()

    def sample(self, sample_hz: float = 60.0) -> dict[str, np.ndarray]:
        count = max(2, int(np.ceil(self.duration_s * sample_hz)) + 1)
        u = np.linspace(0.0, 1.0, count)
        return {
            "u": u,
            "time_s": u * self.duration_s,
            "position_mm": self.position(u),
            "velocity_mm_s": self.velocity(u),
            "acceleration_mm_s2": self.acceleration(u),
            "jerk_mm_s3": self.jerk(u),
        }


def build_velocity_matched_bezier(
    p_a: np.ndarray,
    v_a: np.ndarray,
    p_b: np.ndarray,
    v_b: np.ndarray,
    duration: float,
) -> CubicBezierBridge:
    p0 = np.asarray(p_a, dtype=np.float64)
    p3 = np.asarray(p_b, dtype=np.float64)
    v0 = np.asarray(v_a, dtype=np.float64)
    v1 = np.asarray(v_b, dtype=np.float64)
    return CubicBezierBridge(
        p0=p0,
        p1=p0 + duration * v0 / 3.0,
        p2=p3 - duration * v1 / 3.0,
        p3=p3,
        duration_s=float(duration),
    )


def build_tangent_regularized_bezier(
    p_a: np.ndarray,
    v_a: np.ndarray,
    p_b: np.ndarray,
    v_b: np.ndarray,
    duration: float,
    *,
    minimum_handle_chord_ratio: float,
    direction_epsilon_mm_s: float = 1.0e-6,
) -> tuple[CubicBezierBridge, TangentRegularizationDiagnostics]:
    """Build an opt-in cubic with bounded low-speed tangent degeneration.

    This is deliberately not the default generator.  Existing manifests keep
    exact velocity-matched control points.  A regularized manifest may use
    this helper after an offline hard-filter pass, and the runtime reconstructs
    the same rule from the actual/acknowledged source state.

    The caller remains responsible for enforcing an endpoint speed-adjustment
    limit and all geometric/dynamic hard filters.
    """

    duration_s = float(duration)
    ratio = float(minimum_handle_chord_ratio)
    epsilon = float(direction_epsilon_mm_s)
    if not np.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("Bezier duration must be positive")
    if not np.isfinite(ratio) or not 0.0 < ratio <= 0.25:
        raise ValueError(
            "minimum_handle_chord_ratio must be in (0, 0.25]"
        )
    if not np.isfinite(epsilon) or epsilon <= 0.0:
        raise ValueError("direction_epsilon_mm_s must be positive")

    p0 = np.asarray(p_a, dtype=np.float64)
    p3 = np.asarray(p_b, dtype=np.float64)
    v0 = np.asarray(v_a, dtype=np.float64)
    v1 = np.asarray(v_b, dtype=np.float64)
    for name, value in (("p_a", p0), ("p_b", p3), ("v_a", v0), ("v_b", v1)):
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError(f"{name} must be a finite XYZ vector")

    source_speed = float(np.linalg.norm(v0))
    successor_speed = float(np.linalg.norm(v1))
    if source_speed <= epsilon:
        raise ValueError("source_direction_undefined")
    if successor_speed <= epsilon:
        raise ValueError("successor_direction_undefined")

    chord_length = float(np.linalg.norm(p3 - p0))
    minimum_handle = ratio * chord_length
    source_raw_handle = duration_s * source_speed / 3.0
    successor_raw_handle = duration_s * successor_speed / 3.0
    source_handle = max(source_raw_handle, minimum_handle)
    successor_handle = max(successor_raw_handle, minimum_handle)

    bridge = CubicBezierBridge(
        p0=p0,
        p1=p0 + (v0 / source_speed) * source_handle,
        p2=p3 - (v1 / successor_speed) * successor_handle,
        p3=p3,
        duration_s=duration_s,
    )
    diagnostics = TangentRegularizationDiagnostics(
        chord_length_mm=chord_length,
        minimum_handle_mm=minimum_handle,
        source_raw_handle_mm=source_raw_handle,
        successor_raw_handle_mm=successor_raw_handle,
        source_applied_handle_mm=source_handle,
        successor_applied_handle_mm=successor_handle,
        source_speed_adjustment_mm_s=max(
            0.0,
            3.0 * source_handle / duration_s - source_speed,
        ),
        successor_speed_adjustment_mm_s=max(
            0.0,
            3.0 * successor_handle / duration_s - successor_speed,
        ),
    )
    return bridge, diagnostics
