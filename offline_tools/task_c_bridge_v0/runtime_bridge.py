"""Pure-Python runtime replanning against offline B entry templates."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Iterable

import numpy as np

from .bezier_bridge import CubicBezierBridge, build_velocity_matched_bezier
from .bridge_metrics import BridgeMetrics, evaluate_bridge, feasibility_reasons
from .bridge_optimizer import NoFeasibleBridgeError
from .semantic_candidates import semantic_state_compatibility
from .trajectory_states import CandidatePoint, SemanticState


ObstacleCallback = Callable[[np.ndarray], bool | tuple[bool, str]]


@dataclass(frozen=True)
class RuntimeBEntry:
    entry_id: str
    dataset: str
    episode: int
    frame: int
    semantic_label: str
    semantic_state: SemanticState
    position_mm: np.ndarray
    demonstration_velocity_mm_s: np.ndarray
    b_retained_length_mm: float
    minimum_bridge_z_mm: float | None = None
    velocity_method: str = "demonstration_local"
    velocity_sample_count: int = 1
    source_episodes: tuple[int, ...] = ()

    def __post_init__(self) -> None:
        position = np.asarray(self.position_mm, dtype=np.float64)
        velocity = np.asarray(self.demonstration_velocity_mm_s, dtype=np.float64)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("runtime B entry position must be finite XYZ")
        if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
            raise ValueError("runtime B entry velocity must be finite XYZ")
        if self.b_retained_length_mm < 0.0:
            raise ValueError("B retained length must be non-negative")
        object.__setattr__(self, "position_mm", position.copy())
        object.__setattr__(self, "demonstration_velocity_mm_s", velocity.copy())
        if not self.source_episodes:
            object.__setattr__(self, "source_episodes", (self.episode,))

    @classmethod
    def from_candidate(cls, candidate: CandidatePoint) -> "RuntimeBEntry":
        if candidate.role != "b_entry":
            raise ValueError("runtime B entry requires a b_entry candidate")
        retained = (
            candidate.trajectory.total_length_mm
            - candidate.trajectory.cumulative_length_mm[candidate.index]
        )
        return cls(
            entry_id=(
                f"{candidate.trajectory.dataset}:"
                f"ep{candidate.trajectory.episode}:f{candidate.frame}:"
                f"{candidate.semantic_label}"
            ),
            dataset=candidate.trajectory.dataset,
            episode=candidate.trajectory.episode,
            frame=candidate.frame,
            semantic_label=candidate.semantic_label,
            semantic_state=candidate.semantic_state,
            position_mm=candidate.position_mm,
            demonstration_velocity_mm_s=candidate.velocity_mm_s,
            b_retained_length_mm=float(retained),
            minimum_bridge_z_mm=candidate.minimum_bridge_z_mm,
            velocity_method=candidate.velocity_method,
            velocity_sample_count=candidate.velocity_sample_count,
            source_episodes=candidate.velocity_source_episodes,
        )

    @classmethod
    def from_record(cls, value: dict[str, Any]) -> "RuntimeBEntry":
        """Rehydrate one safety-marked entry from the offline manifest."""

        if value.get("robot_executable") is not False:
            raise ValueError("runtime manifest entry must not claim executable")
        if value.get("dry_run_only") is not True:
            raise ValueError("runtime manifest entry must remain dry-run-only")
        state_value = dict(value["semantic_state"])
        state = SemanticState(
            gripper_closed=bool(state_value["gripper_closed"]),
            holding=state_value.get("holding"),
            contact_mode=state_value.get("contact_mode"),
            completed_subgoals=tuple(state_value.get("completed_subgoals", ())),
            object_state=state_value.get("object_state"),
            entry_preconditions=tuple(state_value.get("entry_preconditions", ())),
        )
        return cls(
            entry_id=str(value["entry_id"]),
            dataset=str(value["dataset"]),
            episode=int(value["episode"]),
            frame=int(value["frame"]),
            semantic_label=str(value["semantic_label"]),
            semantic_state=state,
            position_mm=np.asarray(value["position_mm"], dtype=np.float64),
            demonstration_velocity_mm_s=np.asarray(
                value["demonstration_velocity_mm_s"], dtype=np.float64
            ),
            b_retained_length_mm=float(value["B_retained_length_mm"]),
            minimum_bridge_z_mm=(
                None
                if value.get("minimum_bridge_z_mm") is None
                else float(value["minimum_bridge_z_mm"])
            ),
            velocity_method=str(value.get("velocity_method", "manifest")),
            velocity_sample_count=int(value.get("velocity_sample_count", 1)),
            source_episodes=tuple(
                int(item) for item in value.get("velocity_source_episodes", ())
            ),
        )


@dataclass(frozen=True)
class RuntimeBridgeCandidate:
    entry: RuntimeBEntry
    bridge: CubicBezierBridge
    metrics: BridgeMetrics
    a_retained_length_mm: float
    committed_bridge_prefix_length_mm: float
    total_c_estimate_mm: float
    terminal_velocity_mm_s: np.ndarray
    terminal_velocity_source: str
    semantic_unchecked: tuple[str, ...]
    feasible: bool
    failure_reasons: tuple[str, ...]

    def __post_init__(self) -> None:
        velocity = np.asarray(self.terminal_velocity_mm_s, dtype=np.float64)
        if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
            raise ValueError("runtime terminal velocity must be finite XYZ")
        object.__setattr__(self, "terminal_velocity_mm_s", velocity.copy())
        lengths = (
            self.a_retained_length_mm,
            self.committed_bridge_prefix_length_mm,
            self.total_c_estimate_mm,
        )
        if any(not np.isfinite(value) or value < 0.0 for value in lengths):
            raise ValueError("runtime candidate path lengths must be non-negative")

    @property
    def smoothness_key(self) -> tuple[float, float, float]:
        return (
            self.metrics.max_acceleration_mm_s2,
            self.metrics.max_curvature_per_mm,
            self.metrics.integrated_squared_jerk,
        )


@dataclass(frozen=True)
class RuntimePlanningResult:
    selected: RuntimeBridgeCandidate
    top_k: tuple[RuntimeBridgeCandidate, ...]
    candidates_evaluated: int
    semantic_rejected_entries: int
    failure_counts: dict[str, int]


def runtime_entries_from_manifest(
    manifest: dict[str, Any],
) -> tuple[RuntimeBEntry, ...]:
    safety = dict(manifest.get("safety", {}))
    if safety.get("robot_executable") is not False:
        raise ValueError("runtime manifest safety must say robot_executable=false")
    if safety.get("dry_run_only") is not True:
        raise ValueError("runtime manifest safety must say dry_run_only=true")
    entries = tuple(
        RuntimeBEntry.from_record(value) for value in manifest.get("b_entries", ())
    )
    if not entries:
        raise ValueError("runtime manifest contains no B entries")
    return entries


def _duration_candidates(config: dict[str, Any]) -> np.ndarray:
    start = float(config["bridge_duration_min_s"])
    stop = float(config["bridge_duration_max_s"])
    step = float(config["bridge_duration_step_s"])
    if start <= 0.0 or stop < start or step <= 0.0:
        raise ValueError("invalid runtime duration search bounds")
    count = int(np.floor((stop - start) / step + 1e-9)) + 1
    return start + np.arange(count, dtype=np.float64) * step


def _select_runtime_candidate(
    candidates: list[RuntimeBridgeCandidate],
    near_shortest_delta: float,
) -> RuntimeBridgeCandidate:
    if not candidates:
        raise NoFeasibleBridgeError("no feasible runtime bridge")
    shortest = min(candidate.total_c_estimate_mm for candidate in candidates)
    near = [
        candidate
        for candidate in candidates
        if candidate.total_c_estimate_mm
        <= shortest * (1.0 + near_shortest_delta)
    ]
    return min(
        near,
        key=lambda candidate: (
            candidate.smoothness_key,
            candidate.total_c_estimate_mm,
        ),
    )


class RuntimeBridgePlanner:
    """Enumerate all B entries and durations from a live measured A state."""

    def __init__(
        self,
        *,
        semantic_config: dict[str, Any],
        feasibility_config: dict[str, Any],
        sampling_config: dict[str, Any],
        selection_config: dict[str, Any],
        duration_config: dict[str, Any],
        workspace_min_mm: np.ndarray,
        workspace_max_mm: np.ndarray,
        obstacle_callback: ObstacleCallback | None = None,
    ) -> None:
        self.semantic_config = dict(semantic_config)
        self.feasibility_config = dict(feasibility_config)
        self.sampling_config = dict(sampling_config)
        self.selection_config = dict(selection_config)
        self.duration_config = dict(duration_config)
        self.workspace_min_mm = np.asarray(workspace_min_mm, dtype=np.float64)
        self.workspace_max_mm = np.asarray(workspace_max_mm, dtype=np.float64)
        if (
            self.workspace_min_mm.shape != (3,)
            or self.workspace_max_mm.shape != (3,)
            or np.any(self.workspace_min_mm >= self.workspace_max_mm)
        ):
            raise ValueError("runtime workspace bounds must be ordered XYZ")
        self.obstacle_callback = obstacle_callback

    @classmethod
    def from_manifest(
        cls,
        manifest: dict[str, Any],
        *,
        duration_search: str = "initial_duration_search",
        obstacle_callback: ObstacleCallback | None = None,
    ) -> "RuntimeBridgePlanner":
        planners = dict(manifest["planners"])
        workspace = dict(manifest["workspace"])
        if duration_search not in planners:
            raise KeyError(f"manifest has no duration search: {duration_search}")
        return cls(
            semantic_config=planners["semantic_compatibility"],
            feasibility_config=planners["feasibility"],
            sampling_config=planners["sampling"],
            selection_config=planners["selection"],
            duration_config=planners[duration_search],
            workspace_min_mm=np.asarray(workspace["minimum_mm"], dtype=np.float64),
            workspace_max_mm=np.asarray(workspace["maximum_mm"], dtype=np.float64),
            obstacle_callback=obstacle_callback,
        )

    def plan(
        self,
        *,
        position_a_mm: np.ndarray,
        velocity_a_mm_s: np.ndarray,
        semantic_state_a: SemanticState,
        a_retained_length_mm: float,
        b_entries: Iterable[RuntimeBEntry],
        committed_bridge_prefix_length_mm: float = 0.0,
        terminal_velocity_override_mm_s: np.ndarray | None = None,
        terminal_position_hint_mm: np.ndarray | None = None,
        terminal_position_tolerance_mm: float | None = None,
        terminal_position_override_mm: np.ndarray | None = None,
        terminal_velocity_source: str = "offline_demonstration",
        duration_preference: str = "near_shortest_smoothest",
    ) -> RuntimePlanningResult:
        position_a = np.asarray(position_a_mm, dtype=np.float64)
        velocity_a = np.asarray(velocity_a_mm_s, dtype=np.float64)
        if (
            position_a.shape != (3,)
            or velocity_a.shape != (3,)
            or not np.all(np.isfinite(position_a))
            or not np.all(np.isfinite(velocity_a))
        ):
            raise ValueError("runtime A position and velocity must be finite XYZ")
        prefix_lengths = (
            float(a_retained_length_mm),
            float(committed_bridge_prefix_length_mm),
        )
        if any(not np.isfinite(value) or value < 0.0 for value in prefix_lengths):
            raise ValueError("retained/committed prefix lengths must be non-negative")
        override = (
            None
            if terminal_velocity_override_mm_s is None
            else np.asarray(terminal_velocity_override_mm_s, dtype=np.float64)
        )
        if override is not None and (
            override.shape != (3,) or not np.all(np.isfinite(override))
        ):
            raise ValueError("terminal velocity override must be finite XYZ")
        position_hint = (
            None
            if terminal_position_hint_mm is None
            else np.asarray(terminal_position_hint_mm, dtype=np.float64)
        )
        if (position_hint is None) != (terminal_position_tolerance_mm is None):
            raise ValueError("terminal position hint and tolerance must be paired")
        position_tolerance = None
        if position_hint is not None:
            if position_hint.shape != (3,) or not np.all(np.isfinite(position_hint)):
                raise ValueError("terminal position hint must be finite XYZ")
            position_tolerance = float(terminal_position_tolerance_mm)
            if not np.isfinite(position_tolerance) or position_tolerance <= 0.0:
                raise ValueError("terminal position tolerance must be positive")
        position_override = (
            None
            if terminal_position_override_mm is None
            else np.asarray(terminal_position_override_mm, dtype=np.float64)
        )
        if position_override is not None and (
            position_override.shape != (3,)
            or not np.all(np.isfinite(position_override))
        ):
            raise ValueError("terminal position override must be finite XYZ")
        if duration_preference not in {
            "near_shortest_smoothest",
            "shortest_feasible",
        }:
            raise ValueError(
                "duration_preference must be "
                "near_shortest_smoothest or shortest_feasible"
            )

        feasible: list[RuntimeBridgeCandidate] = []
        evaluated = 0
        semantic_rejected = 0
        failures_by_reason: dict[str, int] = {}
        shared_geometry = position_override is not None and override is not None
        geometry_by_duration: dict[
            float,
            tuple[CubicBezierBridge, BridgeMetrics, dict[str, np.ndarray], tuple[str, ...]],
        ] = {}
        for entry in b_entries:
            compatible, semantic_failures, unchecked = semantic_state_compatibility(
                semantic_state_a,
                entry.semantic_state,
                self.semantic_config,
            )
            if not compatible:
                semantic_rejected += 1
                for reason in semantic_failures:
                    failures_by_reason[reason] = failures_by_reason.get(reason, 0) + 1
                continue
            if position_hint is not None and float(
                np.linalg.norm(entry.position_mm - position_hint)
            ) > float(position_tolerance):
                reason = "act_b_entry_position_inconsistent"
                failures_by_reason[reason] = (
                    failures_by_reason.get(reason, 0) + 1
                )
                continue
            velocity_b = (
                entry.demonstration_velocity_mm_s if override is None else override
            )
            terminal_position = (
                entry.position_mm if position_override is None else position_override
            )
            for duration in _duration_candidates(self.duration_config):
                evaluated += 1
                duration_s = float(duration)
                cached = geometry_by_duration.get(duration_s)
                if cached is None or not shared_geometry:
                    bridge = build_velocity_matched_bezier(
                        position_a,
                        velocity_a,
                        terminal_position,
                        velocity_b,
                        duration_s,
                    )
                    metrics, sample = evaluate_bridge(
                        bridge,
                        sample_hz=float(self.sampling_config["bridge_sample_hz"]),
                        curvature_epsilon=float(
                            self.sampling_config["curvature_epsilon"]
                        ),
                        workspace_min_mm=self.workspace_min_mm,
                        workspace_max_mm=self.workspace_max_mm,
                    )
                    failures = feasibility_reasons(
                        metrics, self.feasibility_config
                    )
                    axis_velocity_limit = self.feasibility_config.get(
                        "axis_velocity_limit_mm_s"
                    )
                    if axis_velocity_limit is not None:
                        axis_velocity_limit = float(axis_velocity_limit)
                        if (
                            not np.isfinite(axis_velocity_limit)
                            or axis_velocity_limit <= 0.0
                        ):
                            raise ValueError(
                                "axis_velocity_limit_mm_s must be finite and positive"
                            )
                        if (
                            float(np.max(np.abs(sample["velocity_mm_s"])))
                            > axis_velocity_limit
                        ):
                            failures.append("axis_velocity_limit")
                    if self.obstacle_callback is not None:
                        obstacle_result = self.obstacle_callback(
                            sample["position_mm"]
                        )
                        if isinstance(obstacle_result, tuple):
                            obstacle_ok, obstacle_reason = obstacle_result
                        else:
                            obstacle_ok, obstacle_reason = (
                                bool(obstacle_result),
                                "obstacle_callback",
                            )
                        if not obstacle_ok:
                            failures.append(str(obstacle_reason))
                    if shared_geometry:
                        geometry_by_duration[duration_s] = (
                            bridge,
                            metrics,
                            sample,
                            tuple(failures),
                        )
                else:
                    bridge, metrics, sample, cached_failures = cached
                    failures = list(cached_failures)
                if (
                    entry.minimum_bridge_z_mm is not None
                    and self.feasibility_config.get(
                        "enforce_payload_transport_floor", False
                    )
                ):
                    tolerance = float(
                        self.feasibility_config.get(
                            "transport_floor_tolerance_mm", 1e-6
                        )
                    )
                    if (
                        float(np.min(sample["position_mm"][:, 2]))
                        < entry.minimum_bridge_z_mm - tolerance
                    ):
                        failures.append("payload_clearance_violation")
                candidate = RuntimeBridgeCandidate(
                    entry=entry,
                    bridge=bridge,
                    metrics=metrics,
                    a_retained_length_mm=float(a_retained_length_mm),
                    committed_bridge_prefix_length_mm=float(
                        committed_bridge_prefix_length_mm
                    ),
                    total_c_estimate_mm=(
                        float(a_retained_length_mm)
                        + float(committed_bridge_prefix_length_mm)
                        + metrics.length_mm
                        + entry.b_retained_length_mm
                    ),
                    terminal_velocity_mm_s=velocity_b,
                    terminal_velocity_source=terminal_velocity_source,
                    semantic_unchecked=tuple(unchecked),
                    feasible=not failures,
                    failure_reasons=tuple(failures),
                )
                for reason in failures:
                    failures_by_reason[reason] = failures_by_reason.get(reason, 0) + 1
                if candidate.feasible:
                    feasible.append(candidate)

        if not feasible:
            raise NoFeasibleBridgeError(
                "no feasible runtime bridge",
                failure_counts=failures_by_reason,
                candidates_evaluated=evaluated,
                semantic_rejected_entries=semantic_rejected,
            )
        if duration_preference == "shortest_feasible":
            selected = min(
                feasible,
                key=lambda candidate: (
                    candidate.bridge.duration_s,
                    candidate.total_c_estimate_mm,
                    candidate.smoothness_key,
                ),
            )
        else:
            selected = _select_runtime_candidate(
                feasible,
                float(self.selection_config["near_shortest_delta"]),
            )
        top_k = tuple(
            sorted(
                feasible,
                key=lambda candidate: (
                    candidate.total_c_estimate_mm,
                    candidate.smoothness_key,
                ),
            )[: int(self.selection_config["top_k"])]
        )
        return RuntimePlanningResult(
            selected=selected,
            top_k=top_k,
            candidates_evaluated=evaluated,
            semantic_rejected_entries=semantic_rejected,
            failure_counts=failures_by_reason,
        )
