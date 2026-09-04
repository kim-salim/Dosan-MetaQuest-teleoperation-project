"""Pure state tracking for the representative 40/20 mm live boundary.

The tracker never publishes commands and never changes a policy queue.  It
only turns timestamped TCP positions and velocities into pre-arm/commit
status.  The live strategy owns the explicit semantic gate and the atomic
transition after final Bridge validation.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


def _finite_xyz(value: Any, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite XYZ")
    return result.copy()


@dataclass(frozen=True)
class RepresentativeBoundaryContract:
    """Immutable live contract emitted by the V1 offline planner."""

    center_position_mm: np.ndarray
    representative_velocity_mm_s: np.ndarray
    representative_tangent: np.ndarray
    prearm_radius_mm: float = 40.0
    commit_radius_mm: float = 20.0
    direction_cosine_minimum: float = 0.7
    approach_frames: int = 3
    commit_stable_frames: int = 3
    approach_epsilon_mm: float = 0.05
    boundary_id: str = "representative_a_exit"

    def __post_init__(self) -> None:
        center = _finite_xyz(self.center_position_mm, "boundary center")
        velocity = _finite_xyz(
            self.representative_velocity_mm_s,
            "representative boundary velocity",
        )
        tangent = _finite_xyz(
            self.representative_tangent,
            "representative boundary tangent",
        )
        tangent_norm = float(np.linalg.norm(tangent))
        if tangent_norm <= 1e-9:
            raise ValueError("representative boundary tangent must be non-zero")
        tangent /= tangent_norm
        if not np.isfinite(self.prearm_radius_mm) or not np.isfinite(
            self.commit_radius_mm
        ):
            raise ValueError("boundary radii must be finite")
        if self.commit_radius_mm <= 0.0:
            raise ValueError("commit radius must be positive")
        if self.prearm_radius_mm <= self.commit_radius_mm:
            raise ValueError("pre-arm radius must exceed commit radius")
        if not -1.0 <= self.direction_cosine_minimum <= 1.0:
            raise ValueError("direction cosine minimum must be in [-1, 1]")
        for name, value in (
            ("approach_frames", self.approach_frames),
            ("commit_stable_frames", self.commit_stable_frames),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        if (
            not np.isfinite(self.approach_epsilon_mm)
            or self.approach_epsilon_mm < 0.0
        ):
            raise ValueError("approach epsilon must be finite and non-negative")
        if not self.boundary_id:
            raise ValueError("boundary_id must not be empty")
        object.__setattr__(self, "center_position_mm", center)
        object.__setattr__(self, "representative_velocity_mm_s", velocity)
        object.__setattr__(self, "representative_tangent", tangent)

    @classmethod
    def from_manifest(cls, manifest: dict[str, Any]) -> "RepresentativeBoundaryContract":
        value = dict(manifest["representative_boundary"])
        if value.get("enabled") is not True:
            raise ValueError("representative boundary manifest is not enabled")
        if value.get("role") != "a_exit":
            raise ValueError("representative boundary must describe an A exit")
        return cls(
            center_position_mm=value["center_position_mm"],
            representative_velocity_mm_s=value[
                "representative_velocity_mm_s"
            ],
            representative_tangent=value["representative_tangent"],
            prearm_radius_mm=float(value["prearm_radius_mm"]),
            commit_radius_mm=float(value["commit_radius_mm"]),
            direction_cosine_minimum=float(value["direction_cosine_minimum"]),
            approach_frames=int(value["approach_frames"]),
            commit_stable_frames=int(value["commit_stable_frames"]),
            approach_epsilon_mm=float(value.get("approach_epsilon_mm", 0.05)),
            boundary_id=str(value.get("boundary_id", "representative_a_exit")),
        )

    def to_record(self) -> dict[str, Any]:
        return {
            "enabled": True,
            "role": "a_exit",
            "boundary_id": self.boundary_id,
            "center_position_mm": self.center_position_mm.tolist(),
            "representative_velocity_mm_s": (
                self.representative_velocity_mm_s.tolist()
            ),
            "representative_tangent": self.representative_tangent.tolist(),
            "prearm_radius_mm": self.prearm_radius_mm,
            "commit_radius_mm": self.commit_radius_mm,
            "direction_cosine_minimum": self.direction_cosine_minimum,
            "approach_frames": self.approach_frames,
            "commit_stable_frames": self.commit_stable_frames,
            "approach_epsilon_mm": self.approach_epsilon_mm,
        }


@dataclass(frozen=True)
class RepresentativeBoundaryStatus:
    distance_mm: float
    minimum_distance_mm: float
    direction_cosine: float | None
    direction_valid: bool
    approach_frames: int
    commit_stable_frames: int
    prearmed: bool
    just_prearmed: bool
    commit_ready: bool
    waiting_reasons: tuple[str, ...]


class RepresentativeBoundaryTracker:
    """Latch an outer read-only pre-arm and debounce the inner commit zone."""

    def __init__(self, contract: RepresentativeBoundaryContract) -> None:
        self.contract = contract
        self.reset()

    def reset(self) -> None:
        self._last_distance_mm: float | None = None
        self._minimum_distance_mm = float("inf")
        self._approach_frames = 0
        self._commit_stable_frames = 0
        self._prearmed = False

    @property
    def prearmed(self) -> bool:
        return self._prearmed

    @property
    def minimum_distance_mm(self) -> float:
        return self._minimum_distance_mm

    def update(
        self,
        *,
        tcp_position_mm: np.ndarray,
        tcp_velocity_mm_s: np.ndarray | None,
        open_before_close_observed: bool,
        gripper_closed: bool,
        semantic_ready: bool | None = None,
    ) -> RepresentativeBoundaryStatus:
        semantic_ok = (
            open_before_close_observed and gripper_closed
            if semantic_ready is None
            else bool(semantic_ready)
        )
        position = _finite_xyz(tcp_position_mm, "boundary TCP position")
        distance = float(
            np.linalg.norm(position - self.contract.center_position_mm)
        )
        self._minimum_distance_mm = min(self._minimum_distance_mm, distance)

        direction_cosine: float | None = None
        if tcp_velocity_mm_s is not None:
            velocity = _finite_xyz(tcp_velocity_mm_s, "boundary TCP velocity")
            speed = float(np.linalg.norm(velocity))
            if speed > 1e-9:
                direction_cosine = float(
                    np.dot(velocity / speed, self.contract.representative_tangent)
                )
        direction_valid = bool(
            direction_cosine is not None
            and direction_cosine >= self.contract.direction_cosine_minimum
        )

        approaching = bool(
            self._last_distance_mm is not None
            and distance
            <= self._last_distance_mm - self.contract.approach_epsilon_mm
        )
        self._last_distance_mm = distance
        if (
            semantic_ok
            and distance <= self.contract.prearm_radius_mm
            and direction_valid
            and approaching
        ):
            self._approach_frames += 1
        elif not self._prearmed:
            self._approach_frames = 0

        just_prearmed = False
        if (
            not self._prearmed
            and self._approach_frames >= self.contract.approach_frames
        ):
            self._prearmed = True
            just_prearmed = True

        inside_commit = bool(
            self._prearmed
            and semantic_ok
            and direction_valid
            and distance <= self.contract.commit_radius_mm
        )
        if inside_commit:
            self._commit_stable_frames += 1
        else:
            self._commit_stable_frames = 0
        commit_ready = bool(
            self._commit_stable_frames >= self.contract.commit_stable_frames
        )

        reasons: list[str] = []
        if semantic_ready is None:
            if not open_before_close_observed:
                reasons.append("open_before_close_not_observed")
            if not gripper_closed:
                reasons.append("gripper_not_closed")
        elif not semantic_ok:
            reasons.append("source_semantic_not_ready")
        if distance > self.contract.prearm_radius_mm:
            reasons.append("outside_prearm_radius")
        if not direction_valid:
            reasons.append("boundary_direction_not_aligned")
        if not self._prearmed:
            reasons.append("boundary_prearm_not_latched")
        if distance > self.contract.commit_radius_mm:
            reasons.append("outside_commit_radius")
        if self._commit_stable_frames < self.contract.commit_stable_frames:
            reasons.append("boundary_commit_not_stable")
        return RepresentativeBoundaryStatus(
            distance_mm=distance,
            minimum_distance_mm=self._minimum_distance_mm,
            direction_cosine=direction_cosine,
            direction_valid=direction_valid,
            approach_frames=self._approach_frames,
            commit_stable_frames=self._commit_stable_frames,
            prearmed=self._prearmed,
            just_prearmed=just_prearmed,
            commit_ready=commit_ready,
            waiting_reasons=tuple(reasons),
        )
