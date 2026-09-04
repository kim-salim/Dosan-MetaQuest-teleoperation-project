"""Lightweight actual-support phase tracking for the Task-C ACT-A exit.

The phase in this module is the normalized Cartesian arc-length phase of one
semantic segment.  It is deliberately independent from ACT-B admission:
source phase answers only *when ACT-A may be interrupted*.  All arrays are
loaded before the control loop and each update is a vectorized nearest-support
query (global once, then a bounded local query).
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path
from typing import Any

import numpy as np


class SourceTriggerMode(str, Enum):
    """Backward-compatible ACT-A trigger selection."""

    MEDIAN_SPHERE = "median_sphere"
    PHASE_SUPPORT = "phase_support"


def clipped_phase_window(nominal_phase: float, half_width: float) -> tuple[float, float]:
    """Return the inclusive source phase window clipped to ``[0, 1]``."""

    nominal = float(nominal_phase)
    width = float(half_width)
    if not np.isfinite(nominal) or not 0.0 <= nominal <= 1.0:
        raise ValueError("nominal phase must be finite and in [0, 1]")
    if not np.isfinite(width) or width < 0.0 or width > 1.0:
        raise ValueError("phase half-width must be finite and in [0, 1]")
    return max(0.0, nominal - width), min(1.0, nominal + width)


@dataclass(frozen=True)
class SourcePhaseSupportConfig:
    nominal_phase: float
    phase_half_width: float = 0.05
    prearm_extra_phase: float = 0.02
    persistence_ticks: int = 3
    local_search_radius_indices: int = 5
    backward_tolerance: float = 0.02
    deadline_extra_phase: float = 0.0
    latch_phase_ready_until_deadline: bool = False
    allow_geometry_only_prearm: bool = False

    def __post_init__(self) -> None:
        clipped_phase_window(self.nominal_phase, self.phase_half_width)
        for name in ("prearm_extra_phase", "backward_tolerance", "deadline_extra_phase"):
            value = float(getattr(self, name))
            if not np.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be finite and in [0, 1]")
        for name in ("persistence_ticks", "local_search_radius_indices"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        if not isinstance(self.latch_phase_ready_until_deadline, bool):
            raise ValueError("latch_phase_ready_until_deadline must be bool")
        if not isinstance(self.allow_geometry_only_prearm, bool):
            raise ValueError("allow_geometry_only_prearm must be bool")

    @property
    def phase_window(self) -> tuple[float, float]:
        return clipped_phase_window(self.nominal_phase, self.phase_half_width)

    @property
    def prearm_phase_low(self) -> float:
        low, _ = self.phase_window
        return max(0.0, low - self.prearm_extra_phase)

    @property
    def deadline_phase(self) -> float:
        _, high = self.phase_window
        return min(1.0, high + self.deadline_extra_phase)


@dataclass(frozen=True)
class SourcePhaseSupportBank:
    """In-memory real-episode support for one semantic segment."""

    artifact_path: str
    segment: str
    phase: np.ndarray
    episode_ids: np.ndarray
    episode_xyz_mm: np.ndarray
    median_xyz_mm: np.ndarray
    support_distance_threshold_mm: float
    support_threshold_source: str
    support_loo_quantile: float

    def __post_init__(self) -> None:
        phase = np.asarray(self.phase, dtype=np.float64)
        episode_ids = np.asarray(self.episode_ids, dtype=np.int64)
        episode_xyz = np.asarray(self.episode_xyz_mm, dtype=np.float64)
        median_xyz = np.asarray(self.median_xyz_mm, dtype=np.float64)
        if phase.ndim != 1 or phase.size < 2:
            raise ValueError("semantic support phase must be one-dimensional")
        if np.any(~np.isfinite(phase)) or np.any(np.diff(phase) <= 0.0):
            raise ValueError("semantic support phase must be finite and increasing")
        if float(phase[0]) < 0.0 or float(phase[-1]) > 1.0:
            raise ValueError("semantic support phase must lie in [0, 1]")
        expected = (episode_ids.size, phase.size, 3)
        if episode_xyz.shape != expected or median_xyz.shape != (phase.size, 3):
            raise ValueError(
                "semantic support XYZ shape mismatch: "
                f"episodes={episode_xyz.shape}, median={median_xyz.shape}, expected={expected}"
            )
        if episode_ids.size < 2:
            raise ValueError("phase_support requires at least two actual episodes")
        if np.any(~np.isfinite(episode_xyz)) or np.any(~np.isfinite(median_xyz)):
            raise ValueError("semantic support XYZ arrays must be finite")
        threshold = float(self.support_distance_threshold_mm)
        if not np.isfinite(threshold) or threshold <= 0.0:
            raise ValueError("support distance threshold must be finite and positive")
        quantile = float(self.support_loo_quantile)
        if not np.isfinite(quantile) or not 0.0 < quantile <= 1.0:
            raise ValueError("support LOO quantile must be in (0, 1]")
        object.__setattr__(self, "phase", phase.copy())
        object.__setattr__(self, "episode_ids", episode_ids.copy())
        object.__setattr__(self, "episode_xyz_mm", episode_xyz.copy())
        object.__setattr__(self, "median_xyz_mm", median_xyz.copy())

    @classmethod
    def load(
        cls,
        artifact_path: str | Path,
        *,
        segment: str,
        phase_window: tuple[float, float],
        support_distance_threshold_mm: float | None,
        support_loo_quantile: float,
    ) -> "SourcePhaseSupportBank":
        path = Path(artifact_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"source semantic support artifact not found: {path}")
        segment_name = str(segment).strip()
        if not segment_name:
            raise ValueError("source semantic segment must not be empty")
        prefix = f"{segment_name}_"
        required = {
            "phase": prefix + "phase",
            "episode_ids": prefix + "episode_ids",
            "episode_xyz_mm": prefix + "episode_xyz_mm",
            "median_xyz_mm": prefix + "median_xyz_mm",
        }
        with np.load(path, allow_pickle=False) as archive:
            missing = [key for key in required.values() if key not in archive.files]
            if missing:
                raise ValueError(
                    f"semantic support artifact lacks {segment_name}: {','.join(missing)}"
                )
            values = {name: np.array(archive[key], copy=True) for name, key in required.items()}

        threshold_source = "configured"
        threshold = support_distance_threshold_mm
        if threshold is None:
            threshold = cls._calibrate_leave_one_out_threshold(
                values["phase"],
                values["episode_xyz_mm"],
                phase_window=phase_window,
                quantile=support_loo_quantile,
            )
            threshold_source = f"leave_one_out_q{float(support_loo_quantile):.3f}"
        return cls(
            artifact_path=str(path),
            segment=segment_name,
            phase=values["phase"],
            episode_ids=values["episode_ids"],
            episode_xyz_mm=values["episode_xyz_mm"],
            median_xyz_mm=values["median_xyz_mm"],
            support_distance_threshold_mm=float(threshold),
            support_threshold_source=threshold_source,
            support_loo_quantile=float(support_loo_quantile),
        )

    @staticmethod
    def _calibrate_leave_one_out_threshold(
        phase: np.ndarray,
        episode_xyz_mm: np.ndarray,
        *,
        phase_window: tuple[float, float],
        quantile: float,
    ) -> float:
        phase_values = np.asarray(phase, dtype=np.float64)
        positions = np.asarray(episode_xyz_mm, dtype=np.float64)
        low, high = phase_window
        indices = np.flatnonzero((phase_values >= low) & (phase_values <= high))
        if indices.size == 0:
            raise ValueError("source phase window has no support samples")
        if positions.ndim != 3 or positions.shape[0] < 2:
            raise ValueError("leave-one-out support calibration requires two episodes")
        selected = positions[:, indices, :]  # E x P x XYZ
        deltas = selected[:, None, :, :] - selected[None, :, :, :]
        distances = np.linalg.norm(deltas, axis=3)  # E x E x P
        episode_count = positions.shape[0]
        distances[np.arange(episode_count), np.arange(episode_count), :] = np.inf
        nearest = np.min(distances, axis=1)
        value = float(np.quantile(nearest, float(quantile)))
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError("leave-one-out support threshold is not positive")
        return value

    def record(self) -> dict[str, Any]:
        return {
            "artifact_path": self.artifact_path,
            "segment": self.segment,
            "episode_count": int(self.episode_ids.size),
            "phase_points": int(self.phase.size),
            "support_distance_threshold_mm": self.support_distance_threshold_mm,
            "support_threshold_source": self.support_threshold_source,
            "support_loo_quantile": self.support_loo_quantile,
        }


@dataclass(frozen=True)
class SourcePhaseStatus:
    estimated_phase: float
    phase_index: int
    phase_window_low: float
    phase_window_high: float
    distance_to_component_median_mm: float
    distance_to_nearest_actual_support_mm: float
    nearest_actual_support_episode: int
    support_distance_threshold_mm: float
    phase_ready: bool
    support_ready: bool
    semantic_ready: bool
    persistence_ticks: int
    persistence_required: int
    prearmed: bool
    just_prearmed: bool
    commit_ready: bool
    deadline_phase: float
    deadline_exceeded: bool
    search_mode: str
    search_low_index: int
    search_high_index: int
    tracker_latency_ms: float
    waiting_reasons: tuple[str, ...]
    raw_phase_ready: bool = False
    phase_window_latched: bool = False

    def record(self) -> dict[str, Any]:
        return asdict(self)


class SourcePhaseSupportTracker:
    """Causal phase/support tracker with bounded post-lock search cost."""

    def __init__(
        self,
        bank: SourcePhaseSupportBank,
        config: SourcePhaseSupportConfig,
    ) -> None:
        self.bank = bank
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._previous_phase_index: int | None = None
        self._persistence_ticks = 0
        self._prearmed = False
        self._phase_window_latched = False
        self._semantic_updates = 0
        self._last_status: SourcePhaseStatus | None = None

    @property
    def last_status(self) -> SourcePhaseStatus | None:
        return self._last_status

    def _search_indices(self) -> tuple[np.ndarray, str]:
        phase = self.bank.phase
        previous = self._previous_phase_index
        if previous is None:
            return np.arange(phase.size, dtype=np.int64), "global"
        radius = self.config.local_search_radius_indices
        low = max(0, previous - radius)
        high = min(phase.size, previous + radius + 1)
        indices = np.arange(low, high, dtype=np.int64)
        minimum_phase = float(phase[previous]) - self.config.backward_tolerance
        allowed = indices[phase[indices] >= minimum_phase - 1e-12]
        if allowed.size == 0:
            allowed = np.array([previous], dtype=np.int64)
        return allowed, "local"

    def update(
        self,
        *,
        tcp_position_mm: np.ndarray,
        tcp_velocity_mm_s: np.ndarray | None = None,
        open_before_close_observed: bool,
        gripper_closed: bool,
        semantic_ready: bool | None = None,
    ) -> SourcePhaseStatus:
        del tcp_velocity_mm_s  # Reserved for an optional future phase feature.
        started = time.perf_counter()
        position = np.asarray(tcp_position_mm, dtype=np.float64)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("source phase tracker requires finite TCP XYZ")
        semantic_ok = bool(
            open_before_close_observed and gripper_closed
            if semantic_ready is None
            else semantic_ready
        )
        if semantic_ok:
            self._semantic_updates += 1

        indices, search_mode = self._search_indices()
        support = self.bank.episode_xyz_mm[:, indices, :]
        distances = np.linalg.norm(support - position[None, None, :], axis=2)
        flattened = int(np.argmin(distances))
        episode_offset, phase_offset = np.unravel_index(flattened, distances.shape)
        phase_index = int(indices[phase_offset])
        estimated_phase = float(self.bank.phase[phase_index])
        nearest_distance = float(distances[episode_offset, phase_offset])
        nearest_episode = int(self.bank.episode_ids[episode_offset])
        median_distance = float(
            np.linalg.norm(position - self.bank.median_xyz_mm[phase_index])
        )
        # Do not lock the segment phase while the semantic predecessor has
        # not completed. Otherwise an approach pose could seed the local search
        # in the wrong part of the transport segment.
        if semantic_ok:
            self._previous_phase_index = phase_index

        low, high = self.config.phase_window
        deadline_phase = self.config.deadline_phase
        raw_phase_ready = bool(
            low - 1e-12 <= estimated_phase <= high + 1e-12
        )
        support_ready = bool(
            nearest_distance <= self.bank.support_distance_threshold_mm
        )
        if not (support_ready and semantic_ok):
            self._phase_window_latched = False
        elif (
            self.config.latch_phase_ready_until_deadline
            and raw_phase_ready
        ):
            self._phase_window_latched = True
        phase_ready = bool(
            raw_phase_ready
            or (
                self.config.latch_phase_ready_until_deadline
                and self._phase_window_latched
                and estimated_phase <= deadline_phase + 1e-12
            )
        )
        prearm_ready = bool(
            (
                semantic_ok
                or (
                    self.config.allow_geometry_only_prearm and support_ready
                )
            )
            and self.config.prearm_phase_low - 1e-12
            <= estimated_phase
            <= high + 1e-12
        )
        just_prearmed = False
        if prearm_ready and not self._prearmed:
            self._prearmed = True
            just_prearmed = True

        stable = bool(phase_ready and support_ready and semantic_ok)
        self._persistence_ticks = self._persistence_ticks + 1 if stable else 0
        commit_ready = bool(
            self._persistence_ticks >= self.config.persistence_ticks
        )
        deadline_exceeded = bool(
            not commit_ready
            and self._semantic_updates >= self.config.persistence_ticks
            and estimated_phase > deadline_phase + 1e-12
        )

        reasons: list[str] = []
        if not phase_ready:
            reasons.append("source_phase_outside_primary_window")
        if not support_ready:
            reasons.append("source_actual_support_ood")
        if not semantic_ok:
            reasons.append("source_semantic_not_ready")
        if self._persistence_ticks < self.config.persistence_ticks:
            reasons.append("source_commit_not_persistent")
        if deadline_exceeded:
            reasons.append("source_phase_deadline_exceeded")
        status = SourcePhaseStatus(
            estimated_phase=estimated_phase,
            phase_index=phase_index,
            phase_window_low=low,
            phase_window_high=high,
            distance_to_component_median_mm=median_distance,
            distance_to_nearest_actual_support_mm=nearest_distance,
            nearest_actual_support_episode=nearest_episode,
            support_distance_threshold_mm=(
                self.bank.support_distance_threshold_mm
            ),
            phase_ready=phase_ready,
            support_ready=support_ready,
            semantic_ready=semantic_ok,
            persistence_ticks=self._persistence_ticks,
            persistence_required=self.config.persistence_ticks,
            prearmed=self._prearmed,
            just_prearmed=just_prearmed,
            commit_ready=commit_ready,
            deadline_phase=self.config.deadline_phase,
            deadline_exceeded=deadline_exceeded,
            search_mode=search_mode,
            search_low_index=int(indices[0]),
            search_high_index=int(indices[-1]),
            tracker_latency_ms=(time.perf_counter() - started) * 1000.0,
            waiting_reasons=tuple(reasons),
            raw_phase_ready=raw_phase_ready,
            phase_window_latched=self._phase_window_latched,
        )
        self._last_status = status
        return status
