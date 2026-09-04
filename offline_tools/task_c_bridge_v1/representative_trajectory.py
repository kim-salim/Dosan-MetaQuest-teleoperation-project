"""Semantic/arc-length standardization for one representative ACT path.

The output stays in the Doosan base frame and physical units.  "Standardize"
here means aligning the same grasp-to-release semantic interval and resampling
it by normalized Cartesian arc length; it never z-scores coordinates that will
later be used by the robot planner.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

import numpy as np

from offline_tools.task_c_bridge_v0.trajectory_states import Trajectory
from offline_tools.task_c_bridge_v0.velocity_estimation import (
    estimate_local_velocity,
)


Role = Literal["a_exit", "b_entry"]


@dataclass(frozen=True)
class TransportWindowConfig:
    start_offset_frames: int = 30
    end_offset_frames: int = -30
    minimum_transport_clearance_mm: float = 50.0
    minimum_z_mm: float | None = None
    phase_points: int = 101
    velocity_window_frames: int = 15
    velocity_method: str = "linear_regression"
    velocity_epsilon: float = 1e-9
    minimum_episodes: int = 5
    floor_quantile: float = 1.0
    transition_ordinal: int = 0

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "TransportWindowConfig":
        return cls(**value)

    def __post_init__(self) -> None:
        if self.start_offset_frames < 0:
            raise ValueError("start_offset_frames must be non-negative")
        if self.end_offset_frames > 0:
            raise ValueError("end_offset_frames must be non-positive")
        if self.minimum_transport_clearance_mm < 0.0:
            raise ValueError("minimum transport clearance must be non-negative")
        if self.minimum_z_mm is not None and not np.isfinite(self.minimum_z_mm):
            raise ValueError("minimum_z_mm must be finite when set")
        if self.phase_points < 3:
            raise ValueError("phase_points must be at least three")
        if self.velocity_window_frames < 3:
            raise ValueError("velocity window must be at least three frames")
        if self.velocity_method not in {
            "linear_regression",
            "central_difference_mean",
        }:
            raise ValueError("unsupported velocity_method")
        if self.minimum_episodes < 1:
            raise ValueError("minimum_episodes must be positive")
        if not 0.0 <= self.floor_quantile <= 1.0:
            raise ValueError("floor_quantile must be in [0, 1]")
        if isinstance(self.transition_ordinal, bool) or self.transition_ordinal < 0:
            raise ValueError("transition_ordinal must be a non-negative integer")


@dataclass(frozen=True)
class StandardizedEpisode:
    episode: int
    source_start_index: int
    source_end_index: int
    close_index: int
    open_index: int
    minimum_bridge_z_mm: float
    phase: np.ndarray
    xyz_mm: np.ndarray
    velocity_mm_s: np.ndarray
    acceleration_mm_s2: np.ndarray
    prefix_length_mm: np.ndarray
    suffix_length_mm: np.ndarray

    def __post_init__(self) -> None:
        phase = np.asarray(self.phase, dtype=np.float64)
        arrays = (
            np.asarray(self.xyz_mm, dtype=np.float64),
            np.asarray(self.velocity_mm_s, dtype=np.float64),
            np.asarray(self.acceleration_mm_s2, dtype=np.float64),
        )
        prefix = np.asarray(self.prefix_length_mm, dtype=np.float64)
        suffix = np.asarray(self.suffix_length_mm, dtype=np.float64)
        if phase.ndim != 1 or len(phase) < 3:
            raise ValueError("standardized episode phase must be one-dimensional")
        if not np.allclose(phase[[0, -1]], [0.0, 1.0], atol=1e-12):
            raise ValueError("standardized episode phase must cover [0, 1]")
        if np.any(np.diff(phase) <= 0.0):
            raise ValueError("standardized episode phase must be increasing")
        for array in arrays:
            if array.shape != (len(phase), 3) or not np.all(np.isfinite(array)):
                raise ValueError("standardized XYZ arrays must be finite [phase, 3]")
        for array in (prefix, suffix):
            if array.shape != phase.shape or not np.all(np.isfinite(array)):
                raise ValueError("standardized path lengths must match phase")
            if np.any(array < 0.0):
                raise ValueError("standardized path lengths must be non-negative")
        object.__setattr__(self, "phase", phase.copy())
        object.__setattr__(self, "xyz_mm", arrays[0].copy())
        object.__setattr__(self, "velocity_mm_s", arrays[1].copy())
        object.__setattr__(self, "acceleration_mm_s2", arrays[2].copy())
        object.__setattr__(self, "prefix_length_mm", prefix.copy())
        object.__setattr__(self, "suffix_length_mm", suffix.copy())


@dataclass(frozen=True)
class RepresentativeTrajectory:
    role: Role
    dataset: str
    phase: np.ndarray
    xyz_mm: np.ndarray
    velocity_mm_s: np.ndarray
    acceleration_mm_s2: np.ndarray
    tangent: np.ndarray
    covariance_mm2: np.ndarray
    mad_mm: np.ndarray
    prefix_length_mm: np.ndarray
    suffix_length_mm: np.ndarray
    minimum_bridge_z_mm: float
    episodes: tuple[StandardizedEpisode, ...]

    def __post_init__(self) -> None:
        if self.role not in {"a_exit", "b_entry"}:
            raise ValueError("representative trajectory role is invalid")
        phase = np.asarray(self.phase, dtype=np.float64)
        count = len(phase)
        shapes = {
            "xyz_mm": (count, 3),
            "velocity_mm_s": (count, 3),
            "acceleration_mm_s2": (count, 3),
            "tangent": (count, 3),
            "covariance_mm2": (count, 3, 3),
            "mad_mm": (count, 3),
        }
        for name, shape in shapes.items():
            value = np.asarray(getattr(self, name), dtype=np.float64)
            if value.shape != shape or not np.all(np.isfinite(value)):
                raise ValueError(f"representative {name} must be finite {shape}")
            object.__setattr__(self, name, value.copy())
        for name in ("prefix_length_mm", "suffix_length_mm"):
            value = np.asarray(getattr(self, name), dtype=np.float64)
            if value.shape != (count,) or not np.all(np.isfinite(value)):
                raise ValueError(f"representative {name} must match phase")
            object.__setattr__(self, name, value.copy())
        if not self.episodes:
            raise ValueError("representative trajectory requires source episodes")
        tangent_norms = np.linalg.norm(self.tangent, axis=1)
        if np.any(tangent_norms <= 1e-9):
            raise ValueError("representative trajectory tangent must be non-zero")
        object.__setattr__(self, "phase", phase.copy())

    @property
    def episode_count(self) -> int:
        return len(self.episodes)

    def _interpolate(self, values: np.ndarray, phase: float) -> np.ndarray | float:
        value = float(phase)
        if not 0.0 <= value <= 1.0:
            raise ValueError("representative phase must be in [0, 1]")
        array = np.asarray(values, dtype=np.float64)
        if array.ndim == 1:
            return float(np.interp(value, self.phase, array))
        flattened = array.reshape(len(self.phase), -1)
        result = np.asarray(
            [
                np.interp(value, self.phase, flattened[:, index])
                for index in range(flattened.shape[1])
            ],
            dtype=np.float64,
        )
        return result.reshape(array.shape[1:])

    def position(self, phase: float) -> np.ndarray:
        return np.asarray(self._interpolate(self.xyz_mm, phase), dtype=np.float64)

    def velocity(self, phase: float) -> np.ndarray:
        return np.asarray(
            self._interpolate(self.velocity_mm_s, phase), dtype=np.float64
        )

    def acceleration(self, phase: float) -> np.ndarray:
        return np.asarray(
            self._interpolate(self.acceleration_mm_s2, phase), dtype=np.float64
        )

    def unit_tangent(self, phase: float) -> np.ndarray:
        value = np.asarray(self._interpolate(self.tangent, phase), dtype=np.float64)
        norm = float(np.linalg.norm(value))
        if norm <= 1e-9:
            raise ValueError("interpolated representative tangent is zero")
        return value / norm

    def prefix_length(self, phase: float) -> float:
        return float(self._interpolate(self.prefix_length_mm, phase))

    def suffix_length(self, phase: float) -> float:
        return float(self._interpolate(self.suffix_length_mm, phase))

    def episode_values(
        self, phase: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        positions = np.stack(
            [
                np.asarray(episode.xyz_mm[
                    int(np.argmin(np.abs(episode.phase - phase)))
                ])
                for episode in self.episodes
            ],
            axis=0,
        )
        velocities = np.stack(
            [
                np.asarray(episode.velocity_mm_s[
                    int(np.argmin(np.abs(episode.phase - phase)))
                ])
                for episode in self.episodes
            ],
            axis=0,
        )
        accelerations = np.stack(
            [
                np.asarray(episode.acceleration_mm_s2[
                    int(np.argmin(np.abs(episode.phase - phase)))
                ])
                for episode in self.episodes
            ],
            axis=0,
        )
        return positions, velocities, accelerations

    def support_record(self, phase: float, radius_mm: float) -> dict[str, Any]:
        positions, _, _ = self.episode_values(phase)
        center = self.position(phase)
        distances = np.linalg.norm(positions - center, axis=1)
        return {
            "radius_mm": float(radius_mm),
            "episodes_within_radius": int(np.sum(distances <= radius_mm)),
            "episode_count": len(distances),
            "fraction_within_radius": float(np.mean(distances <= radius_mm)),
            "distance_p50_mm": float(np.percentile(distances, 50)),
            "distance_p90_mm": float(np.percentile(distances, 90)),
            "distance_p95_mm": float(np.percentile(distances, 95)),
            "distance_max_mm": float(np.max(distances)),
        }


def _transition_span(
    trajectory: Trajectory, transition_ordinal: int = 0
) -> tuple[int, int]:
    closed = trajectory.gripper_closed
    close_indices = np.flatnonzero((~closed[:-1]) & closed[1:]) + 1
    open_indices = np.flatnonzero(closed[:-1] & (~closed[1:])) + 1
    spans = []
    for close in close_indices:
        following = open_indices[open_indices > close]
        if len(following):
            spans.append((int(close), int(following[0])))
    if not spans:
        raise ValueError(
            f"episode {trajectory.episode} has no close-to-open transport span"
        )
    if transition_ordinal >= len(spans):
        raise ValueError(
            f"episode {trajectory.episode} lacks close-to-open transport "
            f"span ordinal {transition_ordinal}"
        )
    return spans[transition_ordinal]


def _longest_contiguous(indices: np.ndarray) -> np.ndarray:
    if len(indices) == 0:
        return indices
    split_points = np.flatnonzero(np.diff(indices) != 1) + 1
    groups = np.split(indices, split_points)
    return max(groups, key=len)


def _full_path_lengths(trajectory: Trajectory) -> np.ndarray:
    increments = np.linalg.norm(np.diff(trajectory.xyz_mm, axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(increments)))


def _local_derivative(
    values: np.ndarray,
    timestamps: np.ndarray,
    index: int,
    window_frames: int,
    method: str,
    epsilon: float,
) -> np.ndarray:
    return estimate_local_velocity(
        values,
        timestamps,
        index,
        window_frames=window_frames,
        method=method,
        velocity_epsilon=epsilon,
    )


def standardize_episode(
    trajectory: Trajectory,
    config: TransportWindowConfig,
) -> StandardizedEpisode:
    close_index, open_index = _transition_span(
        trajectory, config.transition_ordinal
    )
    first = close_index + config.start_offset_frames
    last = open_index + config.end_offset_frames
    if first >= last:
        raise ValueError(
            f"episode {trajectory.episode} transport window is empty"
        )
    event_height = max(
        float(trajectory.xyz_mm[close_index, 2]),
        float(trajectory.xyz_mm[open_index, 2]),
    )
    floor = event_height + config.minimum_transport_clearance_mm
    required_z = floor
    if config.minimum_z_mm is not None:
        required_z = max(required_z, float(config.minimum_z_mm))
    index = np.arange(len(trajectory.xyz_mm))
    mask = (
        (index >= first)
        & (index <= last)
        & trajectory.gripper_closed
        & (trajectory.xyz_mm[:, 2] >= required_z)
    )
    eligible = _longest_contiguous(np.flatnonzero(mask))
    if len(eligible) < 3:
        raise ValueError(
            f"episode {trajectory.episode} has fewer than three eligible transport frames"
        )

    source_xyz = trajectory.xyz_mm[eligible]
    increments = np.linalg.norm(np.diff(source_xyz, axis=0), axis=1)
    arc = np.concatenate(([0.0], np.cumsum(increments)))
    keep = np.concatenate(([True], np.diff(arc) > config.velocity_epsilon))
    if np.sum(keep) < 3 or arc[-1] <= config.velocity_epsilon:
        raise ValueError(f"episode {trajectory.episode} transport arc is degenerate")
    eligible = eligible[keep]
    source_xyz = source_xyz[keep]
    arc = arc[keep]
    source_phase = arc / arc[-1]

    velocities = np.stack(
        [
            _local_derivative(
                trajectory.xyz_mm,
                trajectory.timestamp_s,
                int(item),
                config.velocity_window_frames,
                config.velocity_method,
                config.velocity_epsilon,
            )
            for item in eligible
        ],
        axis=0,
    )
    # Estimate acceleration from the already smoothed full-episode velocity
    # samples.  This remains a diagnostic/tie-break metric, never a command.
    full_velocity = np.stack(
        [
            _local_derivative(
                trajectory.xyz_mm,
                trajectory.timestamp_s,
                int(item),
                config.velocity_window_frames,
                config.velocity_method,
                config.velocity_epsilon,
            )
            for item in range(len(trajectory.xyz_mm))
        ],
        axis=0,
    )
    accelerations = np.stack(
        [
            _local_derivative(
                full_velocity,
                trajectory.timestamp_s,
                int(item),
                config.velocity_window_frames,
                config.velocity_method,
                config.velocity_epsilon,
            )
            for item in eligible
        ],
        axis=0,
    )
    full_length = _full_path_lengths(trajectory)
    prefix = full_length[eligible]
    suffix = full_length[-1] - prefix
    target_phase = np.linspace(0.0, 1.0, config.phase_points)

    def interpolate(values: np.ndarray) -> np.ndarray:
        value = np.asarray(values, dtype=np.float64)
        if value.ndim == 1:
            return np.interp(target_phase, source_phase, value)
        return np.stack(
            [
                np.interp(target_phase, source_phase, value[:, axis])
                for axis in range(value.shape[1])
            ],
            axis=1,
        )

    return StandardizedEpisode(
        episode=trajectory.episode,
        source_start_index=int(eligible[0]),
        source_end_index=int(eligible[-1]),
        close_index=close_index,
        open_index=open_index,
        minimum_bridge_z_mm=floor,
        phase=target_phase,
        xyz_mm=interpolate(source_xyz),
        velocity_mm_s=interpolate(velocities),
        acceleration_mm_s2=interpolate(accelerations),
        prefix_length_mm=interpolate(prefix),
        suffix_length_mm=interpolate(suffix),
    )


def build_representative_trajectory(
    trajectories: list[Trajectory],
    *,
    role: Role,
    config: TransportWindowConfig,
) -> RepresentativeTrajectory:
    episodes: list[StandardizedEpisode] = []
    failures: list[str] = []
    for trajectory in trajectories:
        try:
            episodes.append(standardize_episode(trajectory, config))
        except ValueError as error:
            failures.append(str(error))
    if len(episodes) < config.minimum_episodes:
        raise ValueError(
            f"only {len(episodes)} standardized episodes remain; "
            f"minimum is {config.minimum_episodes}; failures={failures}"
        )
    phase = episodes[0].phase
    positions = np.stack([episode.xyz_mm for episode in episodes], axis=0)
    velocities = np.stack(
        [episode.velocity_mm_s for episode in episodes], axis=0
    )
    accelerations = np.stack(
        [episode.acceleration_mm_s2 for episode in episodes], axis=0
    )
    prefix = np.stack(
        [episode.prefix_length_mm for episode in episodes], axis=0
    )
    suffix = np.stack(
        [episode.suffix_length_mm for episode in episodes], axis=0
    )
    representative_xyz = np.median(positions, axis=0)
    representative_velocity = np.median(velocities, axis=0)
    representative_acceleration = np.median(accelerations, axis=0)
    covariance = (
        np.zeros((len(phase), 3, 3), dtype=np.float64)
        if len(episodes) == 1
        else np.stack(
            [
                np.cov(positions[:, index, :], rowvar=False)
                for index in range(len(phase))
            ],
            axis=0,
        )
    )
    mad = np.median(
        np.abs(positions - representative_xyz[None, :, :]), axis=0
    )
    tangent = representative_velocity.copy()
    gradient = np.gradient(representative_xyz, phase, axis=0)
    for index in range(len(tangent)):
        norm = float(np.linalg.norm(tangent[index]))
        if norm <= config.velocity_epsilon:
            tangent[index] = gradient[index]
            norm = float(np.linalg.norm(tangent[index]))
        if norm <= config.velocity_epsilon:
            raise ValueError(f"representative tangent is zero at phase {phase[index]}")
        tangent[index] /= norm
    floors = np.asarray(
        [episode.minimum_bridge_z_mm for episode in episodes], dtype=np.float64
    )
    floor = float(np.quantile(floors, config.floor_quantile))
    return RepresentativeTrajectory(
        role=role,
        dataset=trajectories[0].dataset,
        phase=phase,
        xyz_mm=representative_xyz,
        velocity_mm_s=representative_velocity,
        acceleration_mm_s2=representative_acceleration,
        tangent=tangent,
        covariance_mm2=covariance,
        mad_mm=mad,
        prefix_length_mm=np.median(prefix, axis=0),
        suffix_length_mm=np.median(suffix, axis=0),
        minimum_bridge_z_mm=floor,
        episodes=tuple(episodes),
    )
