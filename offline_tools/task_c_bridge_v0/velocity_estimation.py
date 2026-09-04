"""Robust offline and causal-runtime Cartesian velocity estimation."""

from __future__ import annotations

from collections import deque

import numpy as np


def _window_bounds(index: int, length: int, window_frames: int) -> tuple[int, int]:
    if window_frames < 3:
        raise ValueError("velocity_window_frames must be at least 3")
    half = window_frames // 2
    start = max(0, index - half)
    stop = min(length, index + half + 1)
    if stop - start < 3:
        if start == 0:
            stop = min(length, 3)
        else:
            start = max(0, length - 3)
    return start, stop


def estimate_local_velocity(
    xyz_mm: np.ndarray,
    timestamp_s: np.ndarray,
    index: int,
    *,
    window_frames: int = 15,
    method: str = "linear_regression",
    velocity_epsilon: float = 1e-9,
) -> np.ndarray:
    """Estimate ``[vx, vy, vz]`` without a single-frame difference."""

    xyz = np.asarray(xyz_mm, dtype=np.float64)
    timestamps = np.asarray(timestamp_s, dtype=np.float64)
    start, stop = _window_bounds(index, len(xyz), window_frames)
    return estimate_velocity_over_window(
        xyz[start:stop],
        timestamps[start:stop],
        method=method,
        velocity_epsilon=velocity_epsilon,
    )


def estimate_velocity_over_window(
    xyz_mm: np.ndarray,
    timestamp_s: np.ndarray,
    *,
    method: str = "linear_regression",
    velocity_epsilon: float = 1e-9,
) -> np.ndarray:
    """Estimate one velocity vector from an explicit timestamped window.

    This function is shared by centered demonstration estimates, causal TCP
    history, and ACT action-chunk intent estimates. It deliberately requires
    at least three samples so no caller silently falls back to a one-frame
    difference.
    """

    points = np.asarray(xyz_mm, dtype=np.float64)
    times = np.asarray(timestamp_s, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("xyz_mm must have shape [samples, 3]")
    if times.ndim != 1 or len(times) != len(points):
        raise ValueError("timestamp_s must have shape [samples]")
    if len(points) < 3:
        raise ValueError("velocity estimation requires at least three samples")
    if not np.all(np.isfinite(points)) or not np.all(np.isfinite(times)):
        raise ValueError("velocity window contains non-finite values")
    if np.any(np.diff(times) <= velocity_epsilon):
        raise ValueError("velocity window contains invalid timestamps")

    if method == "linear_regression":
        centered = times - float(np.mean(times))
        denominator = float(np.dot(centered, centered))
        if denominator <= velocity_epsilon:
            raise ValueError("velocity window has insufficient time extent")
        return (centered[:, None] * points).sum(axis=0) / denominator

    if method == "central_difference_mean":
        dt = np.diff(times)
        return np.mean(np.diff(points, axis=0) / dt[:, None], axis=0)

    raise ValueError(f"unsupported velocity smoothing method: {method}")


def robust_representative_velocity(
    velocities_mm_s: np.ndarray,
    *,
    method: str = "component_median",
    trim_fraction: float = 0.1,
) -> np.ndarray:
    """Return a robust representative 3-D velocity across demonstrations."""

    values = np.asarray(velocities_mm_s, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3 or len(values) == 0:
        raise ValueError("velocities_mm_s must have shape [samples, 3]")
    if not np.all(np.isfinite(values)):
        raise ValueError("representative velocity input contains non-finite values")
    if method == "component_median":
        return np.median(values, axis=0)
    if method == "trimmed_component_mean":
        if not 0.0 <= trim_fraction < 0.5:
            raise ValueError("trim_fraction must satisfy 0 <= value < 0.5")
        ordered = np.sort(values, axis=0)
        trim = int(np.floor(len(ordered) * trim_fraction))
        retained = ordered[trim : len(ordered) - trim] if trim else ordered
        return np.mean(retained, axis=0)
    raise ValueError(f"unsupported representative velocity method: {method}")


class RuntimeVelocityHistory:
    """Bounded causal TCP history used for the physical A/Bridge velocity.

    The latest ``window_frames`` samples are fit as one window. This is
    intentionally different from reading ACT-A's predicted targets: measured
    TCP velocity is the boundary condition that the robot actually reaches.
    """

    def __init__(self, max_samples: int = 120) -> None:
        if max_samples < 3:
            raise ValueError("max_samples must be at least 3")
        self._samples: deque[tuple[float, np.ndarray]] = deque(maxlen=max_samples)

    def clear(self) -> None:
        self._samples.clear()

    def add(self, timestamp_s: float, position_mm: np.ndarray) -> None:
        timestamp = float(timestamp_s)
        position = np.asarray(position_mm, dtype=np.float64)
        if not np.isfinite(timestamp) or position.shape != (3,) or not np.all(
            np.isfinite(position)
        ):
            raise ValueError("runtime TCP sample must be finite timestamped XYZ")
        if self._samples and timestamp < self._samples[-1][0]:
            raise ValueError("runtime TCP timestamps must be monotonic")
        if self._samples and timestamp == self._samples[-1][0]:
            self._samples[-1] = (timestamp, position.copy())
            return
        self._samples.append((timestamp, position.copy()))

    def __len__(self) -> int:
        return len(self._samples)

    def arrays(self, window_frames: int | None = None) -> tuple[np.ndarray, np.ndarray]:
        if window_frames is not None and window_frames < 3:
            raise ValueError("window_frames must be at least 3")
        values = list(self._samples)
        if window_frames is not None:
            values = values[-window_frames:]
        if not values:
            return np.empty((0, 3), dtype=np.float64), np.empty(0, dtype=np.float64)
        timestamps = np.asarray([item[0] for item in values], dtype=np.float64)
        positions = np.stack([item[1] for item in values], axis=0)
        return positions, timestamps

    def estimate(
        self,
        *,
        window_frames: int,
        method: str = "linear_regression",
        velocity_epsilon: float = 1e-9,
    ) -> np.ndarray:
        positions, timestamps = self.arrays(window_frames)
        return estimate_velocity_over_window(
            positions,
            timestamps,
            method=method,
            velocity_epsilon=velocity_epsilon,
        )
