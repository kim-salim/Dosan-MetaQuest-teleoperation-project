"""Data structures for position-only, offline Task-C composition."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal

import numpy as np


Role = Literal["a_cut", "b_entry"]


@dataclass(frozen=True)
class SemanticState:
    """Only states that a Cartesian bridge cannot change belong here.

    ``None`` means that the dataset did not record the state.  Unknown values
    are never presented as verified; the compatibility report lists them as
    unchecked.
    """

    gripper_closed: bool
    holding: bool | None = None
    contact_mode: str | None = None
    completed_subgoals: tuple[str, ...] = ()
    object_state: str | None = None
    entry_preconditions: tuple[str, ...] = ()


@dataclass
class Trajectory:
    dataset: str
    episode: int
    xyz_mm: np.ndarray
    timestamp_s: np.ndarray
    gripper_closed: np.ndarray
    frame_index: np.ndarray
    orientation_payload: np.ndarray | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.xyz_mm = np.asarray(self.xyz_mm, dtype=np.float64)
        self.timestamp_s = np.asarray(self.timestamp_s, dtype=np.float64)
        self.gripper_closed = np.asarray(self.gripper_closed, dtype=np.bool_)
        self.frame_index = np.asarray(self.frame_index, dtype=np.int64)
        if self.xyz_mm.ndim != 2 or self.xyz_mm.shape[1] != 3:
            raise ValueError("xyz_mm must have shape [frames, 3]")
        lengths = {
            len(self.xyz_mm),
            len(self.timestamp_s),
            len(self.gripper_closed),
            len(self.frame_index),
        }
        if len(lengths) != 1:
            raise ValueError("trajectory arrays must have the same length")
        if len(self.xyz_mm) < 2:
            raise ValueError("trajectory must contain at least two frames")
        if not np.all(np.isfinite(self.xyz_mm)):
            raise ValueError("trajectory contains non-finite XYZ")
        if np.any(np.diff(self.timestamp_s) <= 0.0):
            raise ValueError("timestamps must be strictly increasing per episode")
        if self.orientation_payload is not None:
            # Preserve a private copy.  V0 never reads this field for ranking.
            self.orientation_payload = np.array(self.orientation_payload, copy=True)
            if len(self.orientation_payload) != len(self.xyz_mm):
                raise ValueError(
                    "orientation_payload must have the same frame count as XYZ"
                )

    @property
    def cumulative_length_mm(self) -> np.ndarray:
        increments = np.linalg.norm(np.diff(self.xyz_mm, axis=0), axis=1)
        return np.concatenate(([0.0], np.cumsum(increments)))

    @property
    def total_length_mm(self) -> float:
        return float(self.cumulative_length_mm[-1])


@dataclass(frozen=True)
class CandidatePoint:
    role: Role
    trajectory: Trajectory
    index: int
    semantic_label: str
    semantic_state: SemanticState
    velocity_mm_s: np.ndarray
    velocity_method: str
    provenance: str = "gripper_semantic_boundary"
    minimum_bridge_z_mm: float | None = None
    holding_assumption: str = "UNKNOWN"
    semantic_phase: float | None = None
    velocity_sample_count: int = 1
    velocity_source_episodes: tuple[int, ...] = ()

    def __post_init__(self) -> None:
        if self.index < 0 or self.index >= len(self.trajectory.xyz_mm):
            raise ValueError("candidate index is outside its trajectory")
        velocity = np.asarray(self.velocity_mm_s, dtype=np.float64)
        if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
            raise ValueError("candidate velocity_mm_s must be a finite XYZ vector")
        object.__setattr__(self, "velocity_mm_s", velocity.copy())
        if self.semantic_phase is not None and not 0.0 <= self.semantic_phase <= 1.0:
            raise ValueError("semantic_phase must be in [0, 1]")
        if self.velocity_sample_count < 1:
            raise ValueError("velocity_sample_count must be positive")
        if not self.velocity_source_episodes:
            object.__setattr__(
                self,
                "velocity_source_episodes",
                (self.trajectory.episode,),
            )

    @property
    def position_mm(self) -> np.ndarray:
        return self.trajectory.xyz_mm[self.index]

    @property
    def frame(self) -> int:
        return int(self.trajectory.frame_index[self.index])

    @property
    def timestamp_s(self) -> float:
        return float(self.trajectory.timestamp_s[self.index])
