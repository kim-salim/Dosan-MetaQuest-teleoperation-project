"""Command-free automatic source-phase supervisor for multi-stage Task-C V2.

The supervisor answers only one question: *may the current frozen ACT visit be
interrupted now?*  It owns no ROS entity, policy inference, Bridge generation,
or command publication.  The live strategy calls :meth:`update` once per
control observation and, after a positive decision, routes the transition to
the existing V2 Bridge/handoff implementation.

The gripper event latch prevents an open-gripper contact skill (for example
opening a drawer) from phase-locking while the robot is still in its initial
open-gripper approach.  It is deliberately a discrete semantic event, not a
grasp-stability or force-success classifier.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from enum import Enum
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .models import HandoffBoundary
from .source_phase import (
    SourcePhaseStatus,
    SourcePhaseSupportBank,
    SourcePhaseSupportConfig,
    SourcePhaseSupportTracker,
)


class GripperEventMode(str, Enum):
    """Minimal observable event needed before phase tracking gains authority."""

    STATE_MATCH = "state_match"
    OPEN_THEN_CLOSED = "open_then_closed"
    CLOSED_THEN_OPEN = "closed_then_open"
    NONE = "none"
@dataclass(frozen=True)
class ZeroCostExecutionTail:
    """Data-derived physical cut after a symbolic operator has completed.

    Dijkstra state and cost are committed at the semantic exit. ACT-A may
    continue without another symbolic operator charge until it enters the
    commit collar. The tail never applies another operator effect.
    """

    source_operator: str
    semantic_exit_segment: str
    semantic_exit_phase: float
    tracking_segment: str
    tracking_start_phase: float
    collar_phase_low: float
    collar_phase_high: float
    commit_phase_low: float
    commit_phase_high: float
    nominal_phase: float
    deadline_phase: float
    phase_half_width: float
    prearm_extra_phase: float
    phase_points: int
    episode_count: int
    derivation_method: str
    same_segment_as_semantic_exit: bool
    collar_progress_mm_low: float
    collar_progress_mm_high: float
    support_residual_p90_limit_mm: float
    zero_symbolic_cost: bool = True
    fixed_z_minimum_used: bool = False
    latch_commit_window_until_deadline: bool = False

    def __post_init__(self) -> None:
        for name in (
            "source_operator",
            "semantic_exit_segment",
            "tracking_segment",
            "derivation_method",
        ):
            if not str(getattr(self, name)).strip():
                raise ValueError(f"execution_tail {name} must not be empty")
        for name in (
            "semantic_exit_phase",
            "tracking_start_phase",
            "collar_phase_low",
            "collar_phase_high",
            "commit_phase_low",
            "commit_phase_high",
            "nominal_phase",
            "deadline_phase",
            "phase_half_width",
            "prearm_extra_phase",
        ):
            value = float(getattr(self, name))
            if not np.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"execution_tail {name} must be in [0, 1]")
        if not (
            self.tracking_start_phase
            <= self.collar_phase_low
            <= self.commit_phase_low
            <= self.nominal_phase
            <= self.commit_phase_high
            <= self.collar_phase_high
            <= self.deadline_phase
        ):
            raise ValueError("execution_tail phase ordering is invalid")
        expected_half_width = 0.5 * (
            self.commit_phase_high - self.commit_phase_low
        )
        if expected_half_width <= 0.0 or not np.isclose(
            self.phase_half_width, expected_half_width, atol=1.0e-12
        ):
            raise ValueError("execution_tail phase_half_width is inconsistent")
        expected_prearm = self.commit_phase_low - self.tracking_start_phase
        if not np.isclose(
            self.prearm_extra_phase, expected_prearm, atol=1.0e-12
        ):
            raise ValueError("execution_tail prearm_extra_phase is inconsistent")
        if self.phase_points < 2 or self.episode_count < 2:
            raise ValueError("execution_tail requires phase and episode support")
        for name in (
            "collar_progress_mm_low",
            "collar_progress_mm_high",
            "support_residual_p90_limit_mm",
        ):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"execution_tail {name} must be non-negative")
        if self.collar_progress_mm_high < self.collar_progress_mm_low:
            raise ValueError("execution_tail progress must not run backward")
        if self.zero_symbolic_cost is not True:
            raise ValueError("execution_tail must preserve zero symbolic cost")
        if self.fixed_z_minimum_used is not False:
            raise ValueError("execution_tail must not introduce a fixed Z minimum")
        if not isinstance(self.latch_commit_window_until_deadline, bool):
            raise ValueError(
                "execution_tail latch_commit_window_until_deadline must be bool"
            )

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "ZeroCostExecutionTail":
        return cls(**dict(value))

    def to_record(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class StagePhaseSupervisorConfig:
    support_artifact_path: Path
    phase_half_width: float = 0.05
    prearm_extra_phase: float = 0.02
    persistence_ticks: int = 3
    local_search_radius_indices: int = 5
    backward_tolerance: float = 0.02
    deadline_extra_phase: float = 0.0
    support_distance_threshold_mm: float | None = None
    support_loo_quantile: float = 0.95
    gripper_event_mode: GripperEventMode = GripperEventMode.STATE_MATCH
    rearm_closed_then_open_at_prearm: bool = False
    semantic_event_phase_half_width: float = 0.05
    semantic_event_prearm_extra_phase: float = 0.02
    execution_tail: ZeroCostExecutionTail | None = None

    def __post_init__(self) -> None:
        path = Path(self.support_artifact_path).expanduser().resolve()
        object.__setattr__(self, "support_artifact_path", path)
        object.__setattr__(
            self,
            "gripper_event_mode",
            GripperEventMode(self.gripper_event_mode),
        )
        if self.execution_tail is not None and not isinstance(
            self.execution_tail, ZeroCostExecutionTail
        ):
            object.__setattr__(
                self,
                "execution_tail",
                ZeroCostExecutionTail.from_mapping(self.execution_tail),
            )
        if self.support_distance_threshold_mm is not None and (
            not np.isfinite(self.support_distance_threshold_mm)
            or self.support_distance_threshold_mm <= 0.0
        ):
            raise ValueError(
                "support_distance_threshold_mm must be positive or null"
            )
        if (
            not np.isfinite(self.support_loo_quantile)
            or not 0.0 < self.support_loo_quantile <= 1.0
        ):
            raise ValueError("support_loo_quantile must be in (0, 1]")
        if not isinstance(self.rearm_closed_then_open_at_prearm, bool):
            raise ValueError(
                "rearm_closed_then_open_at_prearm must be bool"
            )
        if not 0.0 < self.semantic_event_phase_half_width <= 0.5:
            raise ValueError(
                "semantic_event_phase_half_width must be in (0, 0.5]"
            )
        if not 0.0 <= self.semantic_event_prearm_extra_phase <= 0.5:
            raise ValueError(
                "semantic_event_prearm_extra_phase must be in [0, 0.5]"
            )
        if (
            self.rearm_closed_then_open_at_prearm
            and self.gripper_event_mode
            is not GripperEventMode.CLOSED_THEN_OPEN
        ):
            raise ValueError(
                "prearm event rearming is available only for closed_then_open"
            )

    @classmethod
    def from_mapping(
        cls,
        value: Mapping[str, Any],
        *,
        base_dir: Path,
    ) -> "StagePhaseSupervisorConfig":
        artifact = str(value.get("support_artifact", "")).strip()
        if not artifact:
            raise ValueError("phase_supervisor requires support_artifact")
        path = Path(artifact).expanduser()
        if not path.is_absolute():
            path = base_dir / path
        raw_execution_tail = value.get("execution_tail")
        return cls(
            support_artifact_path=path,
            phase_half_width=float(value.get("phase_half_width", 0.05)),
            prearm_extra_phase=float(value.get("prearm_extra_phase", 0.02)),
            persistence_ticks=int(value.get("persistence_ticks", 3)),
            local_search_radius_indices=int(
                value.get("local_search_radius_indices", 5)
            ),
            backward_tolerance=float(value.get("backward_tolerance", 0.02)),
            deadline_extra_phase=float(value.get("deadline_extra_phase", 0.0)),
            support_distance_threshold_mm=(
                None
                if value.get("support_distance_threshold_mm") is None
                else float(value["support_distance_threshold_mm"])
            ),
            support_loo_quantile=float(value.get("support_loo_quantile", 0.95)),
            gripper_event_mode=GripperEventMode(
                value.get("gripper_event", GripperEventMode.STATE_MATCH.value)
            ),
            rearm_closed_then_open_at_prearm=bool(
                value.get("rearm_closed_then_open_at_prearm", False)
            ),
            semantic_event_phase_half_width=float(
                value.get("semantic_event_phase_half_width", 0.05)
            ),
            semantic_event_prearm_extra_phase=float(
                value.get("semantic_event_prearm_extra_phase", 0.02)
            ),
            execution_tail=(
                None
                if raw_execution_tail is None
                else ZeroCostExecutionTail.from_mapping(
                    dict(raw_execution_tail)
                )
            ),
        )

    def to_record(self) -> dict[str, Any]:
        value = asdict(self)
        value["support_artifact_path"] = str(self.support_artifact_path)
        value["gripper_event_mode"] = self.gripper_event_mode.value
        value["execution_tail"] = (
            None
            if self.execution_tail is None
            else self.execution_tail.to_record()
        )
        return value


class StagePhaseSupervisor:
    """One preloaded, causal phase/support tracker for one policy visit."""

    def __init__(
        self,
        boundary: HandoffBoundary,
        config: StagePhaseSupervisorConfig,
    ) -> None:
        self.boundary = boundary
        self.config = config
        execution_tail = config.execution_tail
        if (
            execution_tail is not None
            and execution_tail.tracking_segment != boundary.segment
        ):
            raise ValueError(
                "execution_tail tracking source differs from handoff manifest; "
                "refusing to reuse the symbolic exit as a runtime Bridge source"
            )
        if (
            execution_tail is not None
            and boundary.phase > execution_tail.nominal_phase + 1.0e-12
        ):
            raise ValueError(
                "handoff manifest reference phase is after the execution-tail "
                "nominal phase; runtime must join the reference causally forward"
            )
        self._separate_semantic_event_gate = bool(
            execution_tail is not None
            and config.rearm_closed_then_open_at_prearm
            and config.gripper_event_mode
            is GripperEventMode.CLOSED_THEN_OPEN
            and execution_tail.semantic_exit_segment
            != execution_tail.tracking_segment
        )
        if self._separate_semantic_event_gate and boundary.semantic.gripper_state != "open":
            raise ValueError(
                "a cross-segment closed_then_open event requires an open source boundary"
            )
        tracking_segment = (
            boundary.segment
            if execution_tail is None
            else execution_tail.tracking_segment
        )
        nominal_phase = (
            boundary.phase
            if execution_tail is None
            else execution_tail.nominal_phase
        )
        phase_half_width = (
            config.phase_half_width
            if execution_tail is None
            else execution_tail.phase_half_width
        )
        prearm_extra_phase = (
            config.prearm_extra_phase
            if execution_tail is None
            else execution_tail.prearm_extra_phase
        )
        deadline_extra_phase = (
            config.deadline_extra_phase
            if execution_tail is None
            else max(
                0.0,
                execution_tail.deadline_phase
                - execution_tail.commit_phase_high,
            )
        )
        phase_config = SourcePhaseSupportConfig(
            nominal_phase=nominal_phase,
            phase_half_width=phase_half_width,
            prearm_extra_phase=prearm_extra_phase,
            persistence_ticks=config.persistence_ticks,
            local_search_radius_indices=config.local_search_radius_indices,
            backward_tolerance=config.backward_tolerance,
            deadline_extra_phase=deadline_extra_phase,
            latch_phase_ready_until_deadline=(
                execution_tail is not None
                and execution_tail.latch_commit_window_until_deadline
            ),
            allow_geometry_only_prearm=(
                config.rearm_closed_then_open_at_prearm
                and not self._separate_semantic_event_gate
            ),
        )
        bank = SourcePhaseSupportBank.load(
            config.support_artifact_path,
            segment=tracking_segment,
            phase_window=phase_config.phase_window,
            support_distance_threshold_mm=(
                config.support_distance_threshold_mm
            ),
            support_loo_quantile=config.support_loo_quantile,
        )
        self.tracker = SourcePhaseSupportTracker(bank, phase_config)
        self.semantic_event_tracker: SourcePhaseSupportTracker | None = None
        if self._separate_semantic_event_gate:
            assert execution_tail is not None
            semantic_phase_config = SourcePhaseSupportConfig(
                nominal_phase=execution_tail.semantic_exit_phase,
                phase_half_width=config.semantic_event_phase_half_width,
                prearm_extra_phase=config.semantic_event_prearm_extra_phase,
                persistence_ticks=config.persistence_ticks,
                local_search_radius_indices=config.local_search_radius_indices,
                backward_tolerance=config.backward_tolerance,
                allow_geometry_only_prearm=True,
            )
            semantic_bank = SourcePhaseSupportBank.load(
                config.support_artifact_path,
                segment=execution_tail.semantic_exit_segment,
                phase_window=semantic_phase_config.phase_window,
                support_distance_threshold_mm=(
                    config.support_distance_threshold_mm
                ),
                support_loo_quantile=config.support_loo_quantile,
            )
            self.semantic_event_tracker = SourcePhaseSupportTracker(
                semantic_bank, semantic_phase_config
            )
        self.reset()

    def reset(self) -> None:
        self.tracker.reset()
        if self.semantic_event_tracker is not None:
            self.semantic_event_tracker.reset()
        self._seen_open = False
        self._seen_closed = False
        self._prearm_closed_ticks = 0
        self._semantic_armed = not (
            self.config.rearm_closed_then_open_at_prearm
            and self.config.gripper_event_mode
            is GripperEventMode.CLOSED_THEN_OPEN
        )
        self._semantic_event_complete = self.semantic_event_tracker is None

    def _semantic_ready(self, *, gripper_closed: bool) -> bool:
        if not self._semantic_armed:
            # T1 may briefly reopen and regrasp near the start of S2. Do not
            # retain that event; only a post-prearm open may complete O1.
            return False
        if gripper_closed:
            self._seen_closed = True
        else:
            self._seen_open = True
        mode = self.config.gripper_event_mode
        expected_closed = self.boundary.semantic.gripper_state == "closed"
        state_matches = bool(gripper_closed == expected_closed)
        if mode is GripperEventMode.NONE:
            return True
        if mode is GripperEventMode.STATE_MATCH:
            return state_matches
        if mode is GripperEventMode.OPEN_THEN_CLOSED:
            return bool(self._seen_open and gripper_closed and state_matches)
        if mode is GripperEventMode.CLOSED_THEN_OPEN:
            return bool(self._seen_closed and not gripper_closed and state_matches)
        raise AssertionError(mode)

    def _maybe_rearm_closed_then_open(
        self,
        *,
        status: SourcePhaseStatus,
        gripper_closed: bool,
        tracker: SourcePhaseSupportTracker | None = None,
    ) -> bool:
        """Arm a release event only inside its supported prearm geometry."""

        if self._semantic_armed:
            return False
        phase_tracker = self.tracker if tracker is None else tracker
        low = phase_tracker.config.prearm_phase_low
        high = status.phase_window_high
        geometry_prearm_ready = bool(
            status.support_ready
            and low - 1.0e-12
            <= status.estimated_phase
            <= high + 1.0e-12
        )
        if not (geometry_prearm_ready and gripper_closed):
            self._prearm_closed_ticks = 0
            return False
        self._prearm_closed_ticks += 1
        if self._prearm_closed_ticks < self.config.persistence_ticks:
            return False
        self._semantic_armed = True
        self._seen_closed = True
        return True

    def update(
        self,
        *,
        tcp_position_mm: np.ndarray,
        tcp_velocity_mm_s: np.ndarray | None,
        gripper_closed: bool,
    ) -> SourcePhaseStatus:
        semantic_ready = self._semantic_ready(gripper_closed=gripper_closed)
        event_tracker = self.semantic_event_tracker
        if event_tracker is not None and not self._semantic_event_complete:
            semantic_event_status = event_tracker.update(
                tcp_position_mm=tcp_position_mm,
                tcp_velocity_mm_s=tcp_velocity_mm_s,
                open_before_close_observed=self._seen_open,
                gripper_closed=gripper_closed,
                semantic_ready=semantic_ready,
            )
            self._maybe_rearm_closed_then_open(
                status=semantic_event_status,
                gripper_closed=gripper_closed,
                tracker=event_tracker,
            )
            # The live gripper bit changes only after the JRT driver completes
            # the command. Latch that supported O1 sample here; the independent
            # S3 tracker still requires the configured multi-tick persistence.
            if (
                semantic_event_status.phase_ready
                and semantic_event_status.support_ready
                and semantic_ready
            ):
                self._semantic_event_complete = True
        if event_tracker is not None:
            semantic_ready = bool(
                self._semantic_event_complete and semantic_ready
            )
        status = self.tracker.update(
            tcp_position_mm=tcp_position_mm,
            tcp_velocity_mm_s=tcp_velocity_mm_s,
            open_before_close_observed=self._seen_open,
            gripper_closed=gripper_closed,
            semantic_ready=semantic_ready,
        )
        if event_tracker is None:
            self._maybe_rearm_closed_then_open(
                status=status,
                gripper_closed=gripper_closed,
            )
        tail = self.config.execution_tail
        if (
            tail is not None
            and semantic_ready
            and status.estimated_phase > tail.deadline_phase + 1.0e-12
        ):
            status = replace(
                status,
                deadline_exceeded=True,
                waiting_reasons=tuple(
                    dict.fromkeys(
                        (*status.waiting_reasons, "execution_tail_deadline_exceeded")
                    )
                ),
            )
            self.tracker._last_status = status
        return status

    def execution_collar_bounds(self) -> tuple[float, float] | None:
        """Return the data-derived prearm collar for this policy visit."""

        tail = self.config.execution_tail
        if tail is None:
            return None
        return (tail.collar_phase_low, tail.collar_phase_high)

    def execution_collar_ready(self, status: SourcePhaseStatus) -> bool:
        """Gate actual/ACK Bridge work without changing phase estimation."""

        bounds = self.execution_collar_bounds()
        if bounds is None:
            return bool(status.prearmed)
        low, high = bounds
        return bool(
            status.prearmed
            and low - 1.0e-12 <= status.estimated_phase
            <= high + 1.0e-12
        )

    def adaptive_rebase_ready(self, status: SourcePhaseStatus) -> bool:
        """Allow rebases while the effective phase gate retains authority.

        Without an execution-tail latch, phase_ready is true only in the
        original commit window. A latched T2 tail may remain ready until its
        explicit deadline, but only while semantic and actual-support checks
        continue to pass.
        """

        return bool(
            status.phase_ready
            and status.support_ready
            and status.semantic_ready
            and not status.deadline_exceeded
        )


    def record(self) -> dict[str, Any]:
        return {
            "task": self.boundary.task,
            "segment": self.boundary.segment,
            "nominal_phase": self.boundary.phase,
            "tracking_segment": self.tracker.bank.segment,
            "tracking_nominal_phase": self.tracker.config.nominal_phase,
            "semantic_exit_segment": (
                self.boundary.segment
                if self.config.execution_tail is None
                else self.config.execution_tail.semantic_exit_segment
            ),
            "semantic_exit_phase": (
                self.boundary.phase
                if self.config.execution_tail is None
                else self.config.execution_tail.semantic_exit_phase
            ),
            "runtime_source_reference_segment": self.boundary.segment,
            "runtime_source_reference_phase": self.boundary.phase,
            "execution_tail": (
                None
                if self.config.execution_tail is None
                else self.config.execution_tail.to_record()
            ),
            "expected_gripper_state": self.boundary.semantic.gripper_state,
            "config": self.config.to_record(),
            "support": self.tracker.bank.record(),
            "seen_open": self._seen_open,
            "seen_closed": self._seen_closed,
            "prearm_closed_ticks": self._prearm_closed_ticks,
            "semantic_armed": self._semantic_armed,
            "separate_semantic_event_gate": self._separate_semantic_event_gate,
            "semantic_event_complete": self._semantic_event_complete,
            "semantic_event_support": (
                None
                if self.semantic_event_tracker is None
                else self.semantic_event_tracker.bank.record()
            ),
            "semantic_event_last_status": (
                None
                if self.semantic_event_tracker is None
                or self.semantic_event_tracker.last_status is None
                else self.semantic_event_tracker.last_status.record()
            ),
            "last_status": (
                None
                if self.tracker.last_status is None
                else self.tracker.last_status.record()
            ),
        }
