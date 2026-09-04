"""Fail-closed A -> Cartesian Bridge -> B runtime coordinator.

This is a command-proposal state machine only. Every proposal is position-only,
dry_run_only=True, and robot_executable=False. It does not import ROS, the
command MUX, ServoL, or robot adapters.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable

import numpy as np

from .bridge_optimizer import NoFeasibleBridgeError
from .bridge_metrics import bridge_partial_length_mm
from .runtime_bridge import (
    RuntimeBEntry,
    RuntimeBridgeCandidate,
    RuntimeBridgePlanner,
    RuntimePlanningResult,
)
from .runtime_policy import (
    AsyncPolicySession,
    PolicyChunk,
    PolicyChunkAssessment,
    PolicyInferenceError,
    assess_policy_chunk,
)
from .trajectory_states import SemanticState
from .velocity_estimation import RuntimeVelocityHistory


class RuntimePhase(str, Enum):
    NEW = "NEW"
    MODELS_WARMED = "MODELS_WARMED"
    A_STARTING = "A_STARTING"
    A_RUNNING = "A_RUNNING"
    BRIDGE_RUNNING = "BRIDGE_RUNNING"
    B_PRIMING = "B_PRIMING"
    B_READY = "B_READY"
    B_RUNNING = "B_RUNNING"
    FAILED_HOLD = "FAILED_HOLD"


@dataclass(frozen=True)
class RuntimeObservation:
    timestamp_s: float
    tcp_position_mm: np.ndarray
    semantic_state: SemanticState
    policy_input: Any = None
    acknowledged_command_position_mm: np.ndarray | None = None
    acknowledged_command_velocity_mm_s: np.ndarray | None = None

    def __post_init__(self) -> None:
        position = np.asarray(self.tcp_position_mm, dtype=np.float64)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("runtime observation TCP must be finite XYZ")
        if not np.isfinite(self.timestamp_s):
            raise ValueError("runtime observation timestamp must be finite")
        object.__setattr__(self, "tcp_position_mm", position.copy())
        command_position = self.acknowledged_command_position_mm
        command_velocity = self.acknowledged_command_velocity_mm_s
        if (command_position is None) != (command_velocity is None):
            raise ValueError(
                "acknowledged command position and velocity must be paired"
            )
        if command_position is not None:
            command_position = np.asarray(command_position, dtype=np.float64)
            command_velocity = np.asarray(command_velocity, dtype=np.float64)
            if (
                command_position.shape != (3,)
                or command_velocity.shape != (3,)
                or not np.all(np.isfinite(command_position))
                or not np.all(np.isfinite(command_velocity))
            ):
                raise ValueError(
                    "acknowledged command state must be finite XYZ position/velocity"
                )
            object.__setattr__(
                self,
                "acknowledged_command_position_mm",
                command_position.copy(),
            )
            object.__setattr__(
                self,
                "acknowledged_command_velocity_mm_s",
                command_velocity.copy(),
            )


@dataclass(frozen=True)
class RuntimeCommandProposal:
    timestamp_s: float
    source: str
    xyz_mm: np.ndarray | None
    gripper_closed: bool | None
    full_policy_action: np.ndarray | None = None
    phase: str = ""
    failure_reasons: tuple[str, ...] = ()
    orientation_status: str = "pending"
    orientation_bridge_generated: bool = False
    robot_executable: bool = False
    dry_run_only: bool = True

    def __post_init__(self) -> None:
        if self.xyz_mm is not None:
            xyz = np.asarray(self.xyz_mm, dtype=np.float64)
            if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
                raise ValueError("proposal XYZ must be finite")
            object.__setattr__(self, "xyz_mm", xyz.copy())
        if self.full_policy_action is not None:
            action = np.asarray(self.full_policy_action, dtype=np.float64)
            if action.ndim != 1 or len(action) < 3 or not np.all(np.isfinite(action)):
                raise ValueError("full policy action must be a finite vector")
            object.__setattr__(self, "full_policy_action", action.copy())
        if self.robot_executable or not self.dry_run_only:
            raise ValueError("Task-C V0 proposals must remain dry-run-only")


@dataclass(frozen=True)
class RuntimeEvent:
    timestamp_s: float
    phase: str
    event: str
    details: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RuntimeHandoffConfig:
    control_hz: float = 30.0
    warmup_inferences: int = 2
    velocity_history_capacity: int = 120
    velocity_window_frames: int = 15
    velocity_smoothing_method: str = "linear_regression"
    velocity_epsilon: float = 1e-9
    b_prime_lead_s: float = 0.6
    a_chunk_velocity_window_steps: int = 15
    b_final_refresh_enabled: bool = True
    b_tail_seed_failure_fallback_enabled: bool = True
    b_final_refresh_lead_s: float = 0.25
    b_endpoint_stop_before_final_refresh: bool = False
    b_moving_overlap_primary_enabled: bool = False
    b_endpoint_settle_position_tolerance_mm: float = 3.0
    b_endpoint_settle_velocity_tolerance_mm_s: float = 15.0
    b_endpoint_settle_min_hold_s: float = 0.1
    b_endpoint_settle_timeout_s: float = 3.0
    b_overlap_search_max_skip_steps: int = 0
    b_stopped_endpoint_direct_handoff_enabled: bool = False
    b_stopped_endpoint_position_bridge_enabled: bool = False
    b_stopped_endpoint_direct_position_limit_mm: float = 6.67
    b_inference_timeout_s: float = 1.0
    b_chunk_max_observation_age_s: float = 1.0
    b_chunk_velocity_window_steps: int = 15
    b_chunk_velocity_method: str = "linear_regression"
    b_first_action_position_jump_limit_mm: float = 75.0
    b_predicted_velocity_limit_mm_s: float = 300.0
    b_entry_position_tolerance_mm: float = 75.0
    b_prime_endpoint_distance_mm: float | None = None
    bridge_progress_max_step_s: float | None = None
    handoff_position_tolerance_mm: float = 10.0
    handoff_velocity_tolerance_mm_s: float = 75.0
    handoff_first_target_tolerance_mm: float = 75.0

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "RuntimeHandoffConfig":
        allowed = set(cls.__dataclass_fields__)
        unknown = set(value) - allowed
        if unknown:
            raise ValueError(f"unknown runtime handoff config fields: {sorted(unknown)}")
        return cls(**value)

    def __post_init__(self) -> None:
        positive = (
            self.control_hz,
            self.velocity_epsilon,
            self.b_prime_lead_s,
            self.b_final_refresh_lead_s,
            self.b_endpoint_settle_position_tolerance_mm,
            self.b_endpoint_settle_velocity_tolerance_mm_s,
            self.b_endpoint_settle_min_hold_s,
            self.b_endpoint_settle_timeout_s,
            self.b_stopped_endpoint_direct_position_limit_mm,
            self.b_inference_timeout_s,
            self.b_chunk_max_observation_age_s,
            self.b_first_action_position_jump_limit_mm,
            self.b_predicted_velocity_limit_mm_s,
            self.b_entry_position_tolerance_mm,
            self.handoff_position_tolerance_mm,
            self.handoff_velocity_tolerance_mm_s,
            self.handoff_first_target_tolerance_mm,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError("runtime time/limit parameters must be positive")
        if (
            self.b_prime_endpoint_distance_mm is not None
            and self.b_prime_endpoint_distance_mm <= 0.0
        ):
            raise ValueError("b_prime_endpoint_distance_mm must be positive when set")
        if (
            self.bridge_progress_max_step_s is not None
            and self.bridge_progress_max_step_s <= 0.0
        ):
            raise ValueError("bridge_progress_max_step_s must be positive when set")
        if self.warmup_inferences < 0:
            raise ValueError("warmup_inferences must be non-negative")
        if not isinstance(self.b_tail_seed_failure_fallback_enabled, bool):
            raise ValueError("b_tail_seed_failure_fallback_enabled must be boolean")
        if not isinstance(self.b_endpoint_stop_before_final_refresh, bool):
            raise ValueError(
                "b_endpoint_stop_before_final_refresh must be boolean"
            )
        if not isinstance(self.b_moving_overlap_primary_enabled, bool):
            raise ValueError(
                "b_moving_overlap_primary_enabled must be boolean"
            )
        if not isinstance(self.b_stopped_endpoint_direct_handoff_enabled, bool):
            raise ValueError(
                "b_stopped_endpoint_direct_handoff_enabled must be boolean"
            )
        if not isinstance(self.b_stopped_endpoint_position_bridge_enabled, bool):
            raise ValueError(
                "b_stopped_endpoint_position_bridge_enabled must be boolean"
            )
        if (
            not isinstance(self.b_overlap_search_max_skip_steps, int)
            or isinstance(self.b_overlap_search_max_skip_steps, bool)
            or self.b_overlap_search_max_skip_steps < 0
        ):
            raise ValueError(
                "b_overlap_search_max_skip_steps must be a non-negative integer"
            )
        if self.velocity_history_capacity < 3 or self.velocity_window_frames < 3:
            raise ValueError("runtime velocity history/window must be at least 3")
        if (
            self.a_chunk_velocity_window_steps < 2
            or self.b_chunk_velocity_window_steps < 2
        ):
            raise ValueError("policy chunk velocity windows require at least 2 actions")


class TaskCRealtimeCoordinator:
    """Coordinate one transition while keeping policy sessions isolated."""

    def __init__(
        self,
        *,
        policy_a: AsyncPolicySession,
        policy_b: AsyncPolicySession,
        initial_planner: RuntimeBridgePlanner,
        tail_planner: RuntimeBridgePlanner,
        b_entries: list[RuntimeBEntry],
        config: RuntimeHandoffConfig,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if policy_a is policy_b:
            raise ValueError("ACT-A and ACT-B must use separate policy sessions")
        if not b_entries:
            raise ValueError("at least one runtime B entry is required")
        self.policy_a = policy_a
        self.policy_b = policy_b
        self.initial_planner = initial_planner
        self.tail_planner = tail_planner
        self.b_entries = tuple(b_entries)
        self.config = config
        self._clock = clock
        self.phase = RuntimePhase.NEW
        self.history = RuntimeVelocityHistory(config.velocity_history_capacity)
        self.events: list[RuntimeEvent] = []
        self.current_plan: RuntimeBridgeCandidate | None = None
        self.bridge_started_s: float | None = None
        self.bridge_elapsed_s = 0.0
        self._bridge_last_observation_s: float | None = None
        self.a_retained_length_mm = 0.0
        self.committed_bridge_prefix_length_mm = 0.0
        self.a_measured_velocity_mm_s: np.ndarray | None = None
        self.a_policy_intent_velocity_mm_s: np.ndarray | None = None
        self.b_generation: int | None = None
        self.b_generation_role: str | None = None
        self.b_prime_started_s: float | None = None
        self.b_prime_position_mm: np.ndarray | None = None
        self.b_tail_seed_attempted = False
        self.b_final_refresh_count = 0
        self.b_endpoint_hold_started_s: float | None = None
        self.b_action_start_index = 0
        self.b_stopped_endpoint_direct_handoff = False
        self.b_stopped_endpoint_position_bridge_handoff = False
        self.b_chunk: PolicyChunk | None = None
        self.b_assessment: PolicyChunkAssessment | None = None
        self.failure_reasons: list[str] = []
        self._a_generation: int | None = None

    def _event(self, timestamp_s: float, event: str, **details: Any) -> None:
        self.events.append(
            RuntimeEvent(
                timestamp_s=float(timestamp_s),
                phase=self.phase.value,
                event=event,
                details=details,
            )
        )

    def warmup_models(
        self,
        *,
        observation_a: Any,
        observation_b: Any,
        timestamp_s: float,
    ) -> dict[str, list[float]]:
        if self.phase is not RuntimePhase.NEW:
            raise RuntimeError("models can only be warmed from NEW")
        latencies = {
            "ACT-A": self.policy_a.warmup(
                observation_a, inferences=self.config.warmup_inferences
            ),
            "ACT-B": self.policy_b.warmup(
                observation_b, inferences=self.config.warmup_inferences
            ),
        }
        self.phase = RuntimePhase.MODELS_WARMED
        self._event(
            timestamp_s,
            "models_resident_and_warmed_outputs_discarded",
            warmup_inferences=self.config.warmup_inferences,
        )
        return latencies

    def adopt_warmed_models(self, *, timestamp_s: float) -> None:
        """Start a new coordinator trial without re-running resident warmup.

        Shadow latency experiments create one fail-closed coordinator per
        transition trial while keeping the two heavyweight ACT sessions GPU
        resident.  This method only accepts sessions whose warmup has already
        completed and never installs a warmup output into either queue.
        """

        if self.phase is not RuntimePhase.NEW:
            raise RuntimeError("resident models can only be adopted from NEW")
        if not self.policy_a.warmed or not self.policy_b.warmed:
            raise RuntimeError("both policy sessions must already be warmed")
        self.policy_a.deactivate_and_clear()
        self.policy_b.deactivate_and_clear()
        self.phase = RuntimePhase.MODELS_WARMED
        self._event(
            timestamp_s,
            "resident_warmed_models_adopted",
            warmup_outputs_reused=False,
            queues_cleared=True,
        )

    def start_a(self, observation: RuntimeObservation) -> int:
        if self.phase is not RuntimePhase.MODELS_WARMED:
            raise RuntimeError("ACT-A can start only after both models are warmed")
        self.history.clear()
        self.history.add(observation.timestamp_s, observation.tcp_position_mm)
        self._a_generation = self.policy_a.prime(
            observation.policy_input,
            observation_timestamp_s=observation.timestamp_s,
        )
        self.phase = RuntimePhase.A_STARTING
        self._event(
            observation.timestamp_s,
            "act_a_fresh_generation_requested",
            generation=self._a_generation,
        )
        return self._a_generation

    def request_cut(
        self,
        observation: RuntimeObservation,
        *,
        semantic_cut_valid: bool,
        a_retained_length_mm: float,
        a_policy_intent_velocity_mm_s: np.ndarray | None = None,
    ) -> RuntimeBridgeCandidate:
        if self.phase is not RuntimePhase.A_RUNNING:
            raise RuntimeError("A cut requires an active fresh ACT-A generation")
        if not semantic_cut_valid:
            raise ValueError("runtime A cut semantic condition is false")
        self.history.add(observation.timestamp_s, observation.tcp_position_mm)
        retained_length = float(a_retained_length_mm)
        if not np.isfinite(retained_length) or retained_length < 0.0:
            raise ValueError("A retained length must be finite and non-negative")

        measured_velocity = self.history.estimate(
            window_frames=self.config.velocity_window_frames,
            method=self.config.velocity_smoothing_method,
            velocity_epsilon=self.config.velocity_epsilon,
        )
        intent: np.ndarray | None = None
        intent_source = "unavailable"
        if a_policy_intent_velocity_mm_s is not None:
            intent = np.asarray(a_policy_intent_velocity_mm_s, dtype=np.float64)
            if intent.shape != (3,) or not np.all(np.isfinite(intent)):
                raise ValueError("ACT-A intent velocity must be finite XYZ")
            intent_source = "caller_supplied_act_a_intent"
        else:
            active_a_chunk = self.policy_a.active_remaining_chunk()
            if active_a_chunk is not None and len(active_a_chunk.actions) >= 2:
                intent_assessment = assess_policy_chunk(
                    active_a_chunk,
                    observation.tcp_position_mm,
                    velocity_window_steps=self.config.a_chunk_velocity_window_steps,
                    velocity_method=self.config.velocity_smoothing_method,
                    velocity_epsilon=self.config.velocity_epsilon,
                    first_position_jump_limit_mm=1e12,
                    predicted_velocity_limit_mm_s=1e12,
                )
                intent = intent_assessment.intended_velocity_mm_s
                intent_source = "active_act_a_postprocessed_chunk"
        tracking_velocity_delta = (
            None
            if intent is None
            else float(np.linalg.norm(measured_velocity - intent))
        )
        if observation.acknowledged_command_position_mm is None:
            bridge_start_position = observation.tcp_position_mm
            bridge_start_position_source = "actual_tcp_fallback"
        else:
            bridge_start_position = observation.acknowledged_command_position_mm
            bridge_start_position_source = "acknowledged_commanded_posx"
        self.a_measured_velocity_mm_s = measured_velocity
        self.a_policy_intent_velocity_mm_s = None if intent is None else intent.copy()
        self.a_retained_length_mm = retained_length
        self.committed_bridge_prefix_length_mm = 0.0
        self.b_generation = None
        self.b_generation_role = None
        self.b_prime_started_s = None
        self.b_prime_position_mm = None
        self.b_tail_seed_attempted = False
        self.b_final_refresh_count = 0
        self.b_endpoint_hold_started_s = None
        self.b_action_start_index = 0
        self.b_stopped_endpoint_direct_handoff = False
        self.b_stopped_endpoint_position_bridge_handoff = False
        self.b_chunk = None
        self.b_assessment = None

        invalidated_generation = self.policy_a.deactivate_and_clear()
        self.policy_b.deactivate_and_clear()
        self._event(
            observation.timestamp_s,
            "act_a_queue_deactivated",
            invalidated_generation=invalidated_generation,
            velocity_source="measured_tcp_history",
            measured_velocity_mm_s=measured_velocity.tolist(),
            act_a_intent_velocity_mm_s=None if intent is None else intent.tolist(),
            act_a_intent_velocity_source=intent_source,
            measured_vs_intent_delta_mm_s=tracking_velocity_delta,
        )

        planning_started_s = self._clock()
        endpoint_stop = (
            self.config.b_final_refresh_enabled
            and self.config.b_endpoint_stop_before_final_refresh
        )
        try:
            result = self.initial_planner.plan(
                position_a_mm=bridge_start_position,
                velocity_a_mm_s=measured_velocity,
                semantic_state_a=observation.semantic_state,
                a_retained_length_mm=self.a_retained_length_mm,
                b_entries=self.b_entries,
                terminal_velocity_override_mm_s=(
                    np.zeros(3, dtype=np.float64) if endpoint_stop else None
                ),
                terminal_velocity_source=(
                    "planned_endpoint_stop"
                    if endpoint_stop
                    else "offline_representative_demonstration"
                ),
            )
        except (ValueError, NoFeasibleBridgeError) as error:
            planning_completed_s = self._clock()
            self._event(
                observation.timestamp_s,
                "initial_bridge_planning_failed",
                planning_started_timestamp_s=planning_started_s,
                planning_completed_timestamp_s=planning_completed_s,
                planning_latency_s=planning_completed_s - planning_started_s,
                failure_reason="no_feasible_initial_bridge",
            )
            self._fail(observation.timestamp_s, "no_feasible_initial_bridge")
            raise NoFeasibleBridgeError(
                "A cut committed but no feasible initial bridge exists"
            ) from error
        self.current_plan = result.selected
        planning_completed_s = self._clock()
        self.bridge_started_s = observation.timestamp_s
        self.bridge_elapsed_s = 0.0
        self._bridge_last_observation_s = observation.timestamp_s
        self.phase = RuntimePhase.BRIDGE_RUNNING
        self._event(
            observation.timestamp_s,
            "initial_bridge_planned",
            entry_id=result.selected.entry.entry_id,
            duration_s=result.selected.bridge.duration_s,
            bridge_start_position_mm=bridge_start_position.tolist(),
            bridge_start_position_source=bridge_start_position_source,
            bridge_start_velocity_source="measured_tcp_history",
            actual_to_bridge_start_position_error_mm=float(
                np.linalg.norm(
                    observation.tcp_position_mm - bridge_start_position
                )
            ),
            terminal_velocity_source=result.selected.terminal_velocity_source,
            candidates_evaluated=result.candidates_evaluated,
            planning_started_timestamp_s=planning_started_s,
            planning_completed_timestamp_s=planning_completed_s,
            planning_latency_s=planning_completed_s - planning_started_s,
        )
        return result.selected

    def step(self, observation: RuntimeObservation) -> RuntimeCommandProposal:
        self.history.add(observation.timestamp_s, observation.tcp_position_mm)
        if self.phase in {RuntimePhase.A_STARTING, RuntimePhase.A_RUNNING}:
            return self._step_a(observation)
        if self.phase in {
            RuntimePhase.BRIDGE_RUNNING,
            RuntimePhase.B_PRIMING,
            RuntimePhase.B_READY,
        }:
            return self._step_bridge(observation)
        if self.phase is RuntimePhase.B_RUNNING:
            return self._step_b(observation)
        if self.phase is RuntimePhase.FAILED_HOLD:
            return self._failed_proposal(observation.timestamp_s)
        return RuntimeCommandProposal(
            timestamp_s=observation.timestamp_s,
            source="NONE",
            xyz_mm=None,
            gripper_closed=None,
            phase=self.phase.value,
        )

    def _step_a(self, observation: RuntimeObservation) -> RuntimeCommandProposal:
        try:
            chunk = self.policy_a.poll()
        except PolicyInferenceError:
            return self._fail(observation.timestamp_s, "act_a_inference_failure")
        if chunk is not None:
            self.policy_a.activate(chunk.generation)
            self.phase = RuntimePhase.A_RUNNING
            self._event(
                observation.timestamp_s,
                "act_a_queue_ready",
                generation=chunk.generation,
                inference_latency_s=chunk.inference_latency_s,
            )
        action = self.policy_a.pop_action()
        return RuntimeCommandProposal(
            timestamp_s=observation.timestamp_s,
            source="ACT-A" if action is not None else "ACT-A_WAITING",
            xyz_mm=None if action is None else action[:3],
            full_policy_action=action,
            gripper_closed=observation.semantic_state.gripper_closed,
            phase=self.phase.value,
        )

    def _step_bridge(self, observation: RuntimeObservation) -> RuntimeCommandProposal:
        assert self.current_plan is not None and self.bridge_started_s is not None
        committed_elapsed_s = self.bridge_elapsed_s
        elapsed = self._advance_bridge_progress(observation.timestamp_s)
        proposal_step_s = max(0.0, elapsed - committed_elapsed_s)
        remaining = max(0.0, self.current_plan.bridge.duration_s - elapsed)
        endpoint_stop = (
            self.config.b_final_refresh_enabled
            and self.config.b_endpoint_stop_before_final_refresh
            and self.current_plan.terminal_velocity_source
            == "planned_endpoint_stop"
        )

        if (
            self.config.b_final_refresh_enabled
            and not endpoint_stop
            and self.b_final_refresh_count == 0
            and remaining <= self.config.b_final_refresh_lead_s
            and self.b_generation_role != "execution_refresh"
        ):
            if observation.policy_input is None:
                return self._fail(
                    observation.timestamp_s, "b_final_refresh_input_missing"
                )
            self.b_generation = self.policy_b.prime(
                observation.policy_input,
                observation_timestamp_s=observation.timestamp_s,
            )
            self.b_generation_role = "execution_refresh"
            self.b_prime_started_s = observation.timestamp_s
            self.b_prime_position_mm = observation.tcp_position_mm.copy()
            self.b_chunk = None
            self.b_assessment = None
            self.b_final_refresh_count += 1
            self.phase = RuntimePhase.B_PRIMING
            self._event(
                observation.timestamp_s,
                "act_b_final_near_entry_refresh_requested",
                generation=self.b_generation,
                generation_role=self.b_generation_role,
                remaining_bridge_s=remaining,
            )

        endpoint_distance = float(
            np.linalg.norm(
                observation.tcp_position_mm - self.current_plan.bridge.p3
            )
        )
        endpoint_distance_ready = (
            self.config.b_prime_endpoint_distance_mm is None
            or endpoint_distance <= self.config.b_prime_endpoint_distance_mm
        )
        if (
            (not endpoint_stop or self.config.b_moving_overlap_primary_enabled)
            and self.b_generation is None
            and remaining <= self.config.b_prime_lead_s
            and endpoint_distance_ready
            and not (
                self.config.b_final_refresh_enabled
                and self.b_tail_seed_attempted
            )
        ):
            if observation.policy_input is None:
                return self._fail(observation.timestamp_s, "b_policy_input_missing")
            self.b_generation = self.policy_b.prime(
                observation.policy_input,
                observation_timestamp_s=observation.timestamp_s,
            )
            self.b_generation_role = (
                "tail_seed" if self.config.b_final_refresh_enabled else "execution"
            )
            if self.b_generation_role == "tail_seed":
                self.b_tail_seed_attempted = True
            self.b_prime_started_s = observation.timestamp_s
            self.b_prime_position_mm = observation.tcp_position_mm.copy()
            self.phase = RuntimePhase.B_PRIMING
            self._event(
                observation.timestamp_s,
                "act_b_fresh_generation_requested_during_bridge",
                generation=self.b_generation,
                generation_role=self.b_generation_role,
                remaining_bridge_s=remaining,
                endpoint_distance_mm=endpoint_distance,
                moving_overlap_primary=(
                    endpoint_stop
                    and self.config.b_moving_overlap_primary_enabled
                ),
                endpoint_stop_fallback_available=(
                    endpoint_stop
                    and self.current_plan.terminal_velocity_source
                    == "planned_endpoint_stop"
                ),
            )

        if (
            endpoint_stop
            and elapsed >= self.current_plan.bridge.duration_s
            and self.b_final_refresh_count == 0
        ):
            if self.b_endpoint_hold_started_s is None:
                self.b_endpoint_hold_started_s = observation.timestamp_s
                self.phase = RuntimePhase.BRIDGE_RUNNING
                self._event(
                    observation.timestamp_s,
                    "bridge_endpoint_stop_hold_started",
                    endpoint_mm=self.current_plan.bridge.p3.tolist(),
                    planned_terminal_velocity_mm_s=(
                        self.current_plan.bridge.velocity(1.0).tolist()
                    ),
                )
            hold_elapsed_s = (
                observation.timestamp_s - self.b_endpoint_hold_started_s
            )
            actual_position_error_mm = float(
                np.linalg.norm(
                    observation.tcp_position_mm - self.current_plan.bridge.p3
                )
            )
            acknowledged_position_error_mm = float("inf")
            if observation.acknowledged_command_position_mm is not None:
                acknowledged_position_error_mm = float(
                    np.linalg.norm(
                        observation.acknowledged_command_position_mm
                        - self.current_plan.bridge.p3
                    )
                )
            try:
                measured_speed_mm_s = float(
                    np.linalg.norm(
                        self.history.estimate(
                            window_frames=self.config.velocity_window_frames,
                            method=self.config.velocity_smoothing_method,
                            velocity_epsilon=self.config.velocity_epsilon,
                        )
                    )
                )
            except ValueError:
                measured_speed_mm_s = float("inf")
            settled = (
                hold_elapsed_s >= self.config.b_endpoint_settle_min_hold_s
                and actual_position_error_mm
                <= self.config.b_endpoint_settle_position_tolerance_mm
                and acknowledged_position_error_mm
                <= self.config.b_endpoint_settle_position_tolerance_mm
                and measured_speed_mm_s
                <= self.config.b_endpoint_settle_velocity_tolerance_mm_s
            )
            if not settled:
                if hold_elapsed_s > self.config.b_endpoint_settle_timeout_s:
                    return self._fail(
                        observation.timestamp_s,
                        "bridge_endpoint_settle_timeout",
                    )
                return RuntimeCommandProposal(
                    timestamp_s=observation.timestamp_s,
                    source="BEZIER_BRIDGE",
                    xyz_mm=self.current_plan.bridge.p3,
                    gripper_closed=observation.semantic_state.gripper_closed,
                    phase=self.phase.value,
                )
            # A moving-overlap inference normally completes before the stopped
            # endpoint. If it is still pending, do not overwrite its generation;
            # poll it below from the acknowledged stopped state. A rejected
            # planning-only seed is then cleared and the next tick requests the
            # ordinary endpoint execution refresh.
            if self.b_generation is None:
                if observation.policy_input is None:
                    return self._fail(
                        observation.timestamp_s,
                        "b_endpoint_refresh_input_missing",
                    )
                self.b_generation = self.policy_b.prime(
                    observation.policy_input,
                    observation_timestamp_s=observation.timestamp_s,
                )
                self.b_generation_role = "execution_refresh"
                self.b_prime_started_s = observation.timestamp_s
                self.b_prime_position_mm = observation.tcp_position_mm.copy()
                self.b_chunk = None
                self.b_assessment = None
                self.b_final_refresh_count += 1
                self.phase = RuntimePhase.B_PRIMING
                self._event(
                    observation.timestamp_s,
                    "act_b_endpoint_settled_refresh_requested",
                    generation=self.b_generation,
                    generation_role=self.b_generation_role,
                    endpoint_hold_elapsed_s=hold_elapsed_s,
                    actual_position_error_mm=actual_position_error_mm,
                    acknowledged_position_error_mm=(
                        acknowledged_position_error_mm
                    ),
                    measured_speed_mm_s=measured_speed_mm_s,
                    splice_velocity_mm_s=[0.0, 0.0, 0.0],
                )
                return RuntimeCommandProposal(
                    timestamp_s=observation.timestamp_s,
                    source="BEZIER_BRIDGE",
                    xyz_mm=self.current_plan.bridge.p3,
                    gripper_closed=observation.semantic_state.gripper_closed,
                    phase=self.phase.value,
                )

        if self.b_generation is not None and self.b_chunk is None:
            chunk: PolicyChunk | None = None
            if (
                self.b_prime_started_s is not None
                and observation.timestamp_s - self.b_prime_started_s
                > self.config.b_inference_timeout_s
            ):
                if not self._retain_bridge_after_advisory_tail_seed_failure(
                    observation.timestamp_s,
                    stage="act_b_inference_timeout",
                    failure_reasons=["act_b_inference_timeout"],
                ):
                    return self._fail(
                        observation.timestamp_s,
                        "act_b_inference_timeout",
                    )
            else:
                try:
                    chunk = self.policy_b.poll()
                except PolicyInferenceError:
                    if not self._retain_bridge_after_advisory_tail_seed_failure(
                        observation.timestamp_s,
                        stage="act_b_inference_failure",
                        failure_reasons=["act_b_inference_failure"],
                    ):
                        return self._fail(
                            observation.timestamp_s,
                            "act_b_inference_failure",
                        )
            if chunk is not None:
                has_acknowledged_splice = (
                    observation.acknowledged_command_position_mm is not None
                )
                proposal = self._accept_b_chunk_and_replan(
                    observation,
                    chunk,
                    committed_elapsed_s=(
                        committed_elapsed_s
                        if has_acknowledged_splice
                        else elapsed
                    ),
                    proposal_step_s=(
                        proposal_step_s if has_acknowledged_splice else 0.0
                    ),
                )
                if proposal is not None:
                    return proposal

        if (
            endpoint_stop
            and self.current_plan.terminal_velocity_source
            == "planned_endpoint_stop"
            and elapsed >= self.current_plan.bridge.duration_s
        ):
            if (
                self.phase is RuntimePhase.B_READY
                and self.b_stopped_endpoint_direct_handoff
            ):
                return self._handoff_to_b(observation)
            return RuntimeCommandProposal(
                timestamp_s=observation.timestamp_s,
                source="BEZIER_BRIDGE",
                xyz_mm=self.current_plan.bridge.p3,
                gripper_closed=observation.semantic_state.gripper_closed,
                phase=self.phase.value,
            )

        assert self.current_plan is not None and self.bridge_started_s is not None
        elapsed = self.bridge_elapsed_s
        if elapsed >= self.current_plan.bridge.duration_s:
            return self._handoff_to_b(observation)

        u = min(1.0, elapsed / self.current_plan.bridge.duration_s)
        return RuntimeCommandProposal(
            timestamp_s=observation.timestamp_s,
            source="BEZIER_BRIDGE",
            xyz_mm=self.current_plan.bridge.position(u),
            gripper_closed=observation.semantic_state.gripper_closed,
            phase=self.phase.value,
        )

    def _retain_bridge_after_advisory_tail_seed_failure(
        self,
        timestamp_s: float,
        *,
        stage: str,
        failure_reasons: list[str],
    ) -> bool:
        """Discard a rejected planning-only seed and require endpoint refresh.

        A ``tail_seed`` is never executable when final refresh is enabled. If
        its mid-Bridge observation is outside ACT-B's demonstrated phase, a
        dynamically infeasible seed must not replace the already validated
        Bridge. The existing Bridge remains active until a fresh
        ``execution_refresh`` is requested near its endpoint. A failure of
        that executable refresh still follows the normal fail-closed path.
        """

        if not (
            self.config.b_final_refresh_enabled
            and self.config.b_tail_seed_failure_fallback_enabled
            and self.b_generation_role == "tail_seed"
            and self.b_final_refresh_count == 0
        ):
            return False
        invalidated_generation = self.policy_b.deactivate_and_clear()
        rejected_generation = self.b_generation
        self.b_generation = None
        self.b_generation_role = None
        self.b_prime_started_s = None
        self.b_prime_position_mm = None
        self.b_chunk = None
        self.b_assessment = None
        self.b_action_start_index = 0
        self.b_stopped_endpoint_direct_handoff = False
        self.b_stopped_endpoint_position_bridge_handoff = False
        self.phase = RuntimePhase.BRIDGE_RUNNING
        self._event(
            timestamp_s,
            "act_b_tail_seed_rejected_existing_bridge_retained",
            rejected_generation=rejected_generation,
            invalidated_generation=invalidated_generation,
            rejected_generation_role="tail_seed",
            stage=stage,
            failure_reasons=list(failure_reasons),
            existing_bridge_retained=True,
            execution_refresh_still_required=True,
            endpoint_stop_fallback_retained=(
                self.current_plan is not None
                and self.current_plan.terminal_velocity_source
                == "planned_endpoint_stop"
            ),
            safety_limits_relaxed=False,
        )
        return True

    def _arm_stopped_endpoint_direct_handoff(
        self,
        observation: RuntimeObservation,
        *,
        chunk: PolicyChunk,
        assessment: PolicyChunkAssessment,
        splice_position: np.ndarray,
        splice_velocity: np.ndarray,
        measured_velocity: np.ndarray,
        planner_failure_counts: dict[str, int],
        planning_started_s: float,
        planning_completed_s: float,
    ) -> bool:
        """Allow a V1-only direct B entry after a fully settled Bridge stop.

        A zero-start-velocity cubic can have a geometric curvature singularity
        even when the first fresh ACT-B target already fits in one downstream
        ramp tick. This fallback never relaxes a limit: it is armed only after
        every tail candidate failed for smoothness-only reasons, and the live
        layer must still validate the complete 6D chunk, per-axis ramp,
        workspace, orientation, and tracking before publishing the first B
        command.
        """

        if not self.config.b_stopped_endpoint_direct_handoff_enabled:
            return False

        assert self.current_plan is not None
        allowed_planner_failures = {
            "acceleration_limit",
            "backtracking_limit",
            "boundary_acceleration_jump_limit",
            "curvature_limit",
            "integrated_squared_jerk_limit",
            "jerk_limit",
        }
        planner_failures = set(planner_failure_counts)
        endpoint = self.current_plan.bridge.p3
        target_delta = chunk.first_xyz_mm - splice_position
        target_max_axis_step = float(np.max(np.abs(target_delta)))
        actual_position_error = float(
            np.linalg.norm(observation.tcp_position_mm - endpoint)
        )
        acknowledged_position_error = float(
            np.linalg.norm(splice_position - endpoint)
        )
        measured_speed = float(np.linalg.norm(measured_velocity))
        commanded_speed = float(np.linalg.norm(splice_velocity))
        terminal_speed = float(
            np.linalg.norm(self.current_plan.bridge.velocity(1.0))
        )
        reasons: list[str] = []
        if not (
            self.config.b_final_refresh_enabled
            and self.config.b_endpoint_stop_before_final_refresh
            and self.current_plan.terminal_velocity_source
            == "planned_endpoint_stop"
        ):
            reasons.append("not_a_stopped_endpoint_refresh")
        if self.b_generation_role != "execution_refresh":
            reasons.append("not_execution_refresh")
        if self.b_final_refresh_count < 1 or self.b_endpoint_hold_started_s is None:
            reasons.append("endpoint_settle_not_confirmed")
        if observation.semantic_state.gripper_closed is not True:
            reasons.append("gripper_not_closed")
        if observation.semantic_state.holding is False:
            reasons.append("payload_not_held")
        if not planner_failures:
            reasons.append("planner_failure_counts_missing")
        elif planner_failures - allowed_planner_failures:
            reasons.append("non_smoothness_planner_failure")
        if actual_position_error > self.config.b_endpoint_settle_position_tolerance_mm:
            reasons.append("actual_endpoint_position")
        if (
            acknowledged_position_error
            > self.config.b_endpoint_settle_position_tolerance_mm
        ):
            reasons.append("acknowledged_endpoint_position")
        if measured_speed > self.config.b_endpoint_settle_velocity_tolerance_mm_s:
            reasons.append("actual_endpoint_speed")
        zero_speed_tolerance = max(1.0e-6, self.config.velocity_epsilon)
        if commanded_speed > zero_speed_tolerance or terminal_speed > zero_speed_tolerance:
            reasons.append("commanded_endpoint_not_stopped")
        if (
            target_max_axis_step
            > self.config.b_stopped_endpoint_direct_position_limit_mm
        ):
            reasons.append("first_target_exceeds_direct_position_limit")

        if reasons:
            self._event(
                observation.timestamp_s,
                "act_b_stopped_endpoint_direct_handoff_rejected",
                generation=chunk.generation,
                generation_role=self.b_generation_role,
                failure_reasons=reasons,
                planner_failure_counts=dict(planner_failure_counts),
                first_target_delta_mm=target_delta.tolist(),
                first_target_jump_mm=float(np.linalg.norm(target_delta)),
                first_target_max_axis_step_mm=target_max_axis_step,
                direct_position_metric="max_abs_cartesian_axis",
                direct_position_limit_mm=(
                    self.config.b_stopped_endpoint_direct_position_limit_mm
                ),
                actual_endpoint_position_error_mm=actual_position_error,
                acknowledged_endpoint_position_error_mm=(
                    acknowledged_position_error
                ),
                measured_endpoint_speed_mm_s=measured_speed,
                commanded_endpoint_speed_mm_s=commanded_speed,
                planned_terminal_speed_mm_s=terminal_speed,
                safety_limits_relaxed=False,
            )
            return False

        self.b_chunk = chunk
        self.b_assessment = assessment
        self.b_action_start_index = 0
        self.b_stopped_endpoint_direct_handoff = True
        self.b_stopped_endpoint_position_bridge_handoff = False
        self.phase = RuntimePhase.B_READY
        self._event(
            observation.timestamp_s,
            "act_b_stopped_endpoint_direct_handoff_ready",
            generation=chunk.generation,
            generation_role=self.b_generation_role,
            first_target_xyz_mm=chunk.first_xyz_mm.tolist(),
            first_target_delta_mm=target_delta.tolist(),
            first_target_jump_mm=float(np.linalg.norm(target_delta)),
            first_target_max_axis_step_mm=target_max_axis_step,
            direct_position_metric="max_abs_cartesian_axis",
            direct_position_limit_mm=(
                self.config.b_stopped_endpoint_direct_position_limit_mm
            ),
            measured_endpoint_speed_mm_s=measured_speed,
            commanded_endpoint_speed_mm_s=commanded_speed,
            planned_terminal_speed_mm_s=terminal_speed,
            planner_failure_counts=dict(planner_failure_counts),
            planning_started_timestamp_s=planning_started_s,
            planning_completed_timestamp_s=planning_completed_s,
            planning_latency_s=planning_completed_s - planning_started_s,
            full_pose_live_validation_required=True,
            safety_limits_relaxed=False,
        )
        return True

    def _plan_stopped_endpoint_position_bridge(
        self,
        observation: RuntimeObservation,
        *,
        chunk: PolicyChunk,
        assessment: PolicyChunkAssessment,
        splice_position: np.ndarray,
        splice_velocity: np.ndarray,
        measured_velocity: np.ndarray,
        committed_bridge_prefix_length_mm: float,
        planning_started_s: float,
    ) -> RuntimePlanningResult | None:
        """Bridge a settled B boundary to the first fresh ACT-B target.

        This V1-only fallback is attempted after the velocity-matched tail and
        one-tick direct handoff both fail. It preserves the already verified B
        boundary stop, plans zero-to-zero motion to the first postprocessed
        ACT-B target, and selects the shortest duration that passes the same
        workspace, payload-floor, velocity, acceleration, jerk, curvature, and
        backtracking checks as every other runtime Bridge.
        """

        if not self.config.b_stopped_endpoint_position_bridge_enabled:
            return None
        if self.b_generation_role != "execution_refresh":
            return None

        assert self.current_plan is not None
        endpoint = self.current_plan.bridge.p3
        actual_position_error = float(
            np.linalg.norm(observation.tcp_position_mm - endpoint)
        )
        acknowledged_position_error = float(
            np.linalg.norm(splice_position - endpoint)
        )
        measured_speed = float(np.linalg.norm(measured_velocity))
        commanded_speed = float(np.linalg.norm(splice_velocity))
        terminal_speed = float(
            np.linalg.norm(self.current_plan.bridge.velocity(1.0))
        )
        zero_speed_tolerance = max(1.0e-6, self.config.velocity_epsilon)
        reasons: list[str] = []
        if not (
            self.config.b_final_refresh_enabled
            and self.config.b_endpoint_stop_before_final_refresh
            and self.current_plan.terminal_velocity_source
            == "planned_endpoint_stop"
        ):
            reasons.append("not_a_stopped_endpoint_refresh")
        if self.b_final_refresh_count < 1 or self.b_endpoint_hold_started_s is None:
            reasons.append("endpoint_settle_not_confirmed")
        if observation.semantic_state.gripper_closed is not True:
            reasons.append("gripper_not_closed")
        if observation.semantic_state.holding is False:
            reasons.append("payload_not_held")
        if actual_position_error > self.config.b_endpoint_settle_position_tolerance_mm:
            reasons.append("actual_endpoint_position")
        if (
            acknowledged_position_error
            > self.config.b_endpoint_settle_position_tolerance_mm
        ):
            reasons.append("acknowledged_endpoint_position")
        if measured_speed > self.config.b_endpoint_settle_velocity_tolerance_mm_s:
            reasons.append("actual_endpoint_speed")
        if commanded_speed > zero_speed_tolerance or terminal_speed > zero_speed_tolerance:
            reasons.append("commanded_endpoint_not_stopped")
        if reasons:
            self._event(
                observation.timestamp_s,
                "act_b_stopped_endpoint_position_bridge_rejected",
                generation=chunk.generation,
                generation_role=self.b_generation_role,
                failure_reasons=reasons,
                actual_endpoint_position_error_mm=actual_position_error,
                acknowledged_endpoint_position_error_mm=(
                    acknowledged_position_error
                ),
                measured_endpoint_speed_mm_s=measured_speed,
                commanded_endpoint_speed_mm_s=commanded_speed,
                planned_terminal_speed_mm_s=terminal_speed,
                safety_limits_relaxed=False,
            )
            return None

        target_delta = chunk.first_xyz_mm - splice_position
        try:
            result = self.tail_planner.plan(
                position_a_mm=splice_position,
                velocity_a_mm_s=np.zeros(3, dtype=np.float64),
                semantic_state_a=observation.semantic_state,
                a_retained_length_mm=self.a_retained_length_mm,
                b_entries=self.b_entries,
                committed_bridge_prefix_length_mm=(
                    committed_bridge_prefix_length_mm
                ),
                terminal_velocity_override_mm_s=np.zeros(3, dtype=np.float64),
                terminal_position_hint_mm=chunk.first_xyz_mm,
                terminal_position_tolerance_mm=(
                    self.config.b_entry_position_tolerance_mm
                ),
                terminal_position_override_mm=chunk.first_xyz_mm,
                terminal_velocity_source="stopped_endpoint_position_bridge",
                duration_preference="shortest_feasible",
            )
        except (ValueError, NoFeasibleBridgeError) as error:
            self._event(
                observation.timestamp_s,
                "act_b_stopped_endpoint_position_bridge_rejected",
                generation=chunk.generation,
                generation_role=self.b_generation_role,
                failure_reasons=["no_feasible_stopped_endpoint_position_bridge"],
                planner_error=str(error),
                candidates_evaluated=getattr(error, "candidates_evaluated", 0),
                planner_failure_counts=dict(
                    getattr(error, "failure_counts", {})
                ),
                first_target_delta_mm=target_delta.tolist(),
                first_target_jump_mm=float(np.linalg.norm(target_delta)),
                safety_limits_relaxed=False,
            )
            return None

        selected = result.selected
        self.b_chunk = chunk
        self.b_assessment = assessment
        self.b_action_start_index = 0
        self.b_stopped_endpoint_direct_handoff = False
        self.b_stopped_endpoint_position_bridge_handoff = True
        self._event(
            observation.timestamp_s,
            "act_b_stopped_endpoint_position_bridge_ready",
            generation=chunk.generation,
            generation_role=self.b_generation_role,
            first_target_xyz_mm=chunk.first_xyz_mm.tolist(),
            first_target_delta_mm=target_delta.tolist(),
            first_target_jump_mm=float(np.linalg.norm(target_delta)),
            duration_s=selected.bridge.duration_s,
            duration_preference="shortest_feasible",
            planned_start_velocity_mm_s=selected.bridge.velocity(0.0).tolist(),
            planned_terminal_velocity_mm_s=selected.bridge.velocity(1.0).tolist(),
            max_velocity_mm_s=selected.metrics.max_velocity_mm_s,
            max_acceleration_mm_s2=selected.metrics.max_acceleration_mm_s2,
            max_jerk_mm_s3=selected.metrics.max_jerk_mm_s3,
            max_curvature_per_mm=selected.metrics.max_curvature_per_mm,
            planning_started_timestamp_s=planning_started_s,
            planning_completed_timestamp_s=self._clock(),
            full_pose_live_validation_required=True,
            all_runtime_bridge_limits_rechecked=True,
            safety_limits_relaxed=False,
        )
        return result

    def _accept_b_chunk_and_replan(
        self,
        observation: RuntimeObservation,
        chunk: PolicyChunk,
        *,
        committed_elapsed_s: float,
        proposal_step_s: float,
    ) -> RuntimeCommandProposal | None:
        assert self.b_prime_position_mm is not None
        planning_started_s = self._clock()
        moving_overlap_primary = bool(
            self.config.b_moving_overlap_primary_enabled
            and self.b_generation_role == "tail_seed"
            and self.current_plan is not None
            and self.current_plan.terminal_velocity_source
            == "planned_endpoint_stop"
        )
        full_chunk = chunk
        self.b_chunk = chunk
        self.b_action_start_index = 0
        assessment = assess_policy_chunk(
            chunk,
            self.b_prime_position_mm,
            velocity_window_steps=self.config.b_chunk_velocity_window_steps,
            velocity_method=self.config.b_chunk_velocity_method,
            velocity_epsilon=self.config.velocity_epsilon,
            first_position_jump_limit_mm=self.config.b_first_action_position_jump_limit_mm,
            predicted_velocity_limit_mm_s=self.config.b_predicted_velocity_limit_mm_s,
        )
        self.b_assessment = assessment
        if not assessment.valid:
            planning_completed_s = self._clock()
            self._event(
                observation.timestamp_s,
                "bridge_tail_planning_failed",
                generation=chunk.generation,
                generation_role=self.b_generation_role,
                stage="act_b_chunk_assessment",
                planning_started_timestamp_s=planning_started_s,
                planning_completed_timestamp_s=planning_completed_s,
                planning_latency_s=planning_completed_s - planning_started_s,
                failure_reasons=list(assessment.failure_reasons),
                first_position_jump_mm=assessment.first_position_jump_mm,
                max_predicted_velocity_mm_s=(
                    assessment.max_predicted_velocity_mm_s
                ),
                intended_velocity_mm_s=assessment.intended_velocity_mm_s.tolist(),
                velocity_window_steps=assessment.velocity_window_steps,
                velocity_method=assessment.velocity_method,
                velocity_sample_source="consecutive_postprocessed_act_b_targets",
            )
            if self._retain_bridge_after_advisory_tail_seed_failure(
                observation.timestamp_s,
                stage="act_b_chunk_assessment",
                failure_reasons=list(assessment.failure_reasons),
            ):
                return None
            return self._fail(observation.timestamp_s, *assessment.failure_reasons)
        splice_position: np.ndarray | None = None
        splice_velocity: np.ndarray | None = None
        measured_velocity: np.ndarray | None = None
        splice_position_source = "unavailable"
        splice_velocity_source = "unavailable"
        try:
            assert self.current_plan is not None and self.bridge_started_s is not None
            committed_u = min(
                1.0,
                committed_elapsed_s / self.current_plan.bridge.duration_s,
            )
            committed_prefix = (
                self.committed_bridge_prefix_length_mm
                + bridge_partial_length_mm(
                    self.current_plan.bridge,
                    u_end=committed_u,
                    sample_hz=float(
                        self.tail_planner.sampling_config["bridge_sample_hz"]
                    ),
                )
            )
            measured_velocity = self.history.estimate(
                window_frames=self.config.velocity_window_frames,
                method=self.config.velocity_smoothing_method,
                velocity_epsilon=self.config.velocity_epsilon,
            )
            if observation.acknowledged_command_position_mm is None:
                splice_position = observation.tcp_position_mm
                splice_velocity = measured_velocity
                splice_position_source = "actual_tcp_fallback"
                splice_velocity_source = "measured_tcp_history_fallback"
            else:
                assert observation.acknowledged_command_velocity_mm_s is not None
                splice_position = observation.acknowledged_command_position_mm
                splice_velocity = observation.acknowledged_command_velocity_mm_s
                splice_position_source = "acknowledged_commanded_posx"
                splice_velocity_source = "acknowledged_bridge_tangent"
            result = self.tail_planner.plan(
                position_a_mm=splice_position,
                velocity_a_mm_s=splice_velocity,
                semantic_state_a=observation.semantic_state,
                a_retained_length_mm=self.a_retained_length_mm,
                b_entries=self.b_entries,
                committed_bridge_prefix_length_mm=committed_prefix,
                terminal_velocity_override_mm_s=assessment.intended_velocity_mm_s,
                terminal_position_hint_mm=chunk.first_xyz_mm,
                terminal_position_tolerance_mm=(
                    self.config.b_entry_position_tolerance_mm
                ),
                terminal_position_override_mm=chunk.first_xyz_mm,
                terminal_velocity_source="fresh_act_b_postprocessed_chunk",
            )
        except (ValueError, NoFeasibleBridgeError) as first_error:
            attempts: list[dict[str, Any]] = [
                {
                    "action_start_index": 0,
                    "planner_error": str(first_error),
                    "candidates_evaluated": getattr(
                        first_error, "candidates_evaluated", 0
                    ),
                    "planner_failure_counts": dict(
                        getattr(first_error, "failure_counts", {})
                    ),
                }
            ]
            result = None
            last_error: ValueError | NoFeasibleBridgeError = first_error
            maximum_skip = min(
                self.config.b_overlap_search_max_skip_steps,
                len(full_chunk.actions) - 2,
            )
            if splice_position is not None and splice_velocity is not None:
                for action_start_index in range(1, maximum_skip + 1):
                    candidate_chunk = full_chunk.suffix(action_start_index)
                    candidate_assessment = assess_policy_chunk(
                        candidate_chunk,
                        self.b_prime_position_mm,
                        velocity_window_steps=(
                            self.config.b_chunk_velocity_window_steps
                        ),
                        velocity_method=self.config.b_chunk_velocity_method,
                        velocity_epsilon=self.config.velocity_epsilon,
                        first_position_jump_limit_mm=(
                            self.config.b_first_action_position_jump_limit_mm
                        ),
                        predicted_velocity_limit_mm_s=(
                            self.config.b_predicted_velocity_limit_mm_s
                        ),
                    )
                    if not candidate_assessment.valid:
                        attempts.append(
                            {
                                "action_start_index": action_start_index,
                                "assessment_failure_reasons": list(
                                    candidate_assessment.failure_reasons
                                ),
                            }
                        )
                        continue
                    try:
                        candidate_result = self.tail_planner.plan(
                            position_a_mm=splice_position,
                            velocity_a_mm_s=splice_velocity,
                            semantic_state_a=observation.semantic_state,
                            a_retained_length_mm=self.a_retained_length_mm,
                            b_entries=self.b_entries,
                            committed_bridge_prefix_length_mm=committed_prefix,
                            terminal_velocity_override_mm_s=(
                                candidate_assessment.intended_velocity_mm_s
                            ),
                            terminal_position_hint_mm=(
                                candidate_chunk.first_xyz_mm
                            ),
                            terminal_position_tolerance_mm=(
                                self.config.b_entry_position_tolerance_mm
                            ),
                            terminal_position_override_mm=(
                                candidate_chunk.first_xyz_mm
                            ),
                            terminal_velocity_source=(
                                "fresh_act_b_postprocessed_chunk"
                            ),
                        )
                    except (ValueError, NoFeasibleBridgeError) as retry_error:
                        last_error = retry_error
                        attempts.append(
                            {
                                "action_start_index": action_start_index,
                                "planner_error": str(retry_error),
                                "candidates_evaluated": getattr(
                                    retry_error, "candidates_evaluated", 0
                                ),
                                "planner_failure_counts": dict(
                                    getattr(retry_error, "failure_counts", {})
                                ),
                            }
                        )
                        continue
                    result = candidate_result
                    chunk = candidate_chunk
                    assessment = candidate_assessment
                    self.b_chunk = candidate_chunk
                    self.b_assessment = candidate_assessment
                    self.b_action_start_index = action_start_index
                    break
            if result is None:
                planning_completed_s = self._clock()
                self._event(
                    observation.timestamp_s,
                    "bridge_tail_planning_failed",
                    generation=full_chunk.generation,
                    generation_role=self.b_generation_role,
                    stage="velocity_matched_tail_overlap_search",
                    planning_started_timestamp_s=planning_started_s,
                    planning_completed_timestamp_s=planning_completed_s,
                    planning_latency_s=planning_completed_s - planning_started_s,
                    failure_reasons=["no_feasible_act_b_velocity_matched_tail"],
                    planner_error=str(last_error),
                    overlap_attempts=attempts,
                    overlap_max_skip_steps=(
                        self.config.b_overlap_search_max_skip_steps
                    ),
                    b_prime_position_mm=self.b_prime_position_mm.tolist(),
                    act_b_first_xyz_mm=full_chunk.first_xyz_mm.tolist(),
                    splice_position_mm=(
                        None
                        if splice_position is None
                        else splice_position.tolist()
                    ),
                    splice_velocity_mm_s=(
                        None
                        if splice_velocity is None
                        else splice_velocity.tolist()
                    ),
                    splice_position_source=splice_position_source,
                    splice_velocity_source=splice_velocity_source,
                )
                if (
                    splice_position is not None
                    and splice_velocity is not None
                    and measured_velocity is not None
                    and self._arm_stopped_endpoint_direct_handoff(
                        observation,
                        chunk=full_chunk,
                        assessment=assessment,
                        splice_position=splice_position,
                        splice_velocity=splice_velocity,
                        measured_velocity=measured_velocity,
                        planner_failure_counts=dict(
                            attempts[0].get("planner_failure_counts", {})
                        ),
                        planning_started_s=planning_started_s,
                        planning_completed_s=planning_completed_s,
                    )
                ):
                    return None
                position_bridge_result = None
                if (
                    splice_position is not None
                    and splice_velocity is not None
                    and measured_velocity is not None
                ):
                    position_bridge_result = (
                        self._plan_stopped_endpoint_position_bridge(
                            observation,
                            chunk=full_chunk,
                            assessment=assessment,
                            splice_position=splice_position,
                            splice_velocity=splice_velocity,
                            measured_velocity=measured_velocity,
                            committed_bridge_prefix_length_mm=committed_prefix,
                            planning_started_s=planning_started_s,
                        )
                    )
                if position_bridge_result is None:
                    if self._retain_bridge_after_advisory_tail_seed_failure(
                        observation.timestamp_s,
                        stage="velocity_matched_tail_overlap_search",
                        failure_reasons=[
                            "no_feasible_act_b_velocity_matched_tail"
                        ],
                    ):
                        return None
                    return self._fail(
                        observation.timestamp_s,
                        "no_feasible_act_b_velocity_matched_tail",
                    )
                result = position_bridge_result
                chunk = full_chunk
                assert self.b_assessment is not None
                assessment = self.b_assessment
        planning_completed_s = self._clock()
        initial_progress_s = min(
            max(0.0, float(proposal_step_s)),
            1.0 / self.config.control_hz,
            result.selected.bridge.duration_s,
        )
        first_outgoing_u = initial_progress_s / result.selected.bridge.duration_s
        first_outgoing_position = result.selected.bridge.position(first_outgoing_u)
        self.committed_bridge_prefix_length_mm = committed_prefix
        self.current_plan = result.selected
        self.bridge_started_s = observation.timestamp_s - initial_progress_s
        self.bridge_elapsed_s = initial_progress_s
        self._bridge_last_observation_s = observation.timestamp_s
        self.phase = RuntimePhase.B_READY
        self._event(
            observation.timestamp_s,
            "bridge_tail_replanned_from_fresh_act_b",
            generation=chunk.generation,
            generation_role=self.b_generation_role,
            b_action_start_index=self.b_action_start_index,
            b_skipped_prefix_duration_s=(
                self.b_action_start_index / chunk.action_hz
            ),
            entry_id=result.selected.entry.entry_id,
            duration_s=result.selected.bridge.duration_s,
            b_intended_velocity_mm_s=assessment.intended_velocity_mm_s.tolist(),
            first_position_jump_mm=assessment.first_position_jump_mm,
            bridge_endpoint_equals_act_b_first_target=True,
            splice_position_source=splice_position_source,
            splice_velocity_source=splice_velocity_source,
            splice_position_mm=splice_position.tolist(),
            splice_velocity_mm_s=splice_velocity.tolist(),
            actual_to_splice_position_error_mm=float(
                np.linalg.norm(observation.tcp_position_mm - splice_position)
            ),
            initial_tail_progress_s=initial_progress_s,
            first_outgoing_position_mm=first_outgoing_position.tolist(),
            first_outgoing_axis_step_mm=np.abs(
                first_outgoing_position - splice_position
            ).tolist(),
            atomic_splice_at_acknowledged_tick=True,
            moving_overlap_primary=moving_overlap_primary,
            stopped_endpoint_position_bridge=(
                self.b_stopped_endpoint_position_bridge_handoff
            ),
            handoff_mode=(
                "stopped_endpoint_position_bridge"
                if self.b_stopped_endpoint_position_bridge_handoff
                else "velocity_matched_tail"
            ),
            planned_terminal_velocity_mm_s=(
                result.selected.bridge.velocity(1.0).tolist()
            ),
            endpoint_stop_fallback_consumed=(
                moving_overlap_primary
                or self.b_stopped_endpoint_position_bridge_handoff
            ),
            committed_bridge_prefix_length_mm=committed_prefix,
            total_c_estimate_mm=result.selected.total_c_estimate_mm,
            planning_started_timestamp_s=planning_started_s,
            planning_completed_timestamp_s=planning_completed_s,
            planning_latency_s=planning_completed_s - planning_started_s,
        )
        return None

    def _advance_bridge_progress(self, timestamp_s: float) -> float:
        """Advance path time without catching up multiple ticks in one command."""

        assert self.bridge_started_s is not None
        previous = self._bridge_last_observation_s
        if previous is None:
            previous = self.bridge_started_s
        delta_s = max(0.0, float(timestamp_s) - float(previous))
        maximum = self.config.bridge_progress_max_step_s
        if maximum is not None:
            delta_s = min(delta_s, maximum)
        self.bridge_elapsed_s += delta_s
        self._bridge_last_observation_s = max(float(previous), float(timestamp_s))
        return self.bridge_elapsed_s

    def _handoff_to_b(self, observation: RuntimeObservation) -> RuntimeCommandProposal:
        reasons: list[str] = []
        if self.b_chunk is None or self.b_assessment is None or self.b_generation is None:
            reasons.append("act_b_fresh_queue_not_ready")
        else:
            if self.b_chunk.generation != self.b_generation:
                reasons.append("act_b_generation_mismatch")
            expected_role = (
                "execution_refresh"
                if self.config.b_final_refresh_enabled
                else "execution"
            )
            if self.b_generation_role != expected_role:
                reasons.append("act_b_execution_generation_role_mismatch")
            if (
                self.b_chunk.observation_age_s(observation.timestamp_s)
                > self.config.b_chunk_max_observation_age_s
            ):
                reasons.append("act_b_chunk_stale")
            assert self.current_plan is not None
            direct_handoff = self.b_stopped_endpoint_direct_handoff
            position_bridge_handoff = (
                self.b_stopped_endpoint_position_bridge_handoff
            )
            endpoint_error = float(
                np.linalg.norm(observation.tcp_position_mm - self.current_plan.bridge.p3)
            )
            endpoint_position_limit = (
                self.config.b_endpoint_settle_position_tolerance_mm
                if direct_handoff
                else self.config.handoff_position_tolerance_mm
            )
            if endpoint_error > endpoint_position_limit:
                reasons.append("bridge_endpoint_position_tracking_error")
            try:
                measured_velocity = self.history.estimate(
                    window_frames=self.config.velocity_window_frames,
                    method=self.config.velocity_smoothing_method,
                    velocity_epsilon=self.config.velocity_epsilon,
                )
                if direct_handoff or position_bridge_handoff:
                    if (
                        float(np.linalg.norm(measured_velocity))
                        > self.config.b_endpoint_settle_velocity_tolerance_mm_s
                    ):
                        reasons.append("bridge_endpoint_velocity_tracking_error")
                else:
                    velocity_error = float(
                        np.linalg.norm(
                            measured_velocity
                            - self.b_assessment.intended_velocity_mm_s
                        )
                    )
                    if velocity_error > self.config.handoff_velocity_tolerance_mm_s:
                        reasons.append("bridge_endpoint_velocity_tracking_error")
            except ValueError:
                reasons.append("bridge_endpoint_velocity_unavailable")
            first_target_error = float(
                np.linalg.norm(self.b_chunk.first_xyz_mm - self.current_plan.bridge.p3)
            )
            first_target_max_axis_step = float(
                np.max(
                    np.abs(
                        self.b_chunk.first_xyz_mm
                        - self.current_plan.bridge.p3
                    )
                )
            )
            if direct_handoff:
                if not (
                    self.config.b_stopped_endpoint_direct_handoff_enabled
                    and self.config.b_endpoint_stop_before_final_refresh
                    and self.current_plan.terminal_velocity_source
                    == "planned_endpoint_stop"
                    and self.b_generation_role == "execution_refresh"
                ):
                    reasons.append("stopped_endpoint_direct_handoff_contract")
                if (
                    first_target_max_axis_step
                    > self.config.b_stopped_endpoint_direct_position_limit_mm
                ):
                    reasons.append("act_b_first_target_exceeds_direct_limit")
                zero_speed_tolerance = max(1.0e-6, self.config.velocity_epsilon)
                if (
                    float(
                        np.linalg.norm(self.current_plan.bridge.velocity(1.0))
                    )
                    > zero_speed_tolerance
                ):
                    reasons.append("bridge_terminal_velocity_not_stopped")
                if observation.acknowledged_command_position_mm is None:
                    reasons.append("stopped_endpoint_acknowledgement_missing")
                elif (
                    float(
                        np.linalg.norm(
                            observation.acknowledged_command_position_mm
                            - self.current_plan.bridge.p3
                        )
                    )
                    > self.config.b_endpoint_settle_position_tolerance_mm
                ):
                    reasons.append("stopped_endpoint_acknowledgement_error")
            elif position_bridge_handoff:
                if not (
                    self.config.b_stopped_endpoint_position_bridge_enabled
                    and self.config.b_endpoint_stop_before_final_refresh
                    and self.current_plan.terminal_velocity_source
                    == "stopped_endpoint_position_bridge"
                    and self.b_generation_role == "execution_refresh"
                ):
                    reasons.append(
                        "stopped_endpoint_position_bridge_handoff_contract"
                    )
                if first_target_error > 1.0e-6:
                    reasons.append(
                        "act_b_first_target_not_at_position_bridge_endpoint"
                    )
                zero_speed_tolerance = max(
                    1.0e-6,
                    self.config.velocity_epsilon,
                )
                if (
                    float(
                        np.linalg.norm(self.current_plan.bridge.velocity(1.0))
                    )
                    > zero_speed_tolerance
                ):
                    reasons.append("position_bridge_terminal_velocity_not_stopped")
                if observation.acknowledged_command_position_mm is None:
                    reasons.append("position_bridge_acknowledgement_missing")
                elif (
                    float(
                        np.linalg.norm(
                            observation.acknowledged_command_position_mm
                            - self.current_plan.bridge.p3
                        )
                    )
                    > self.config.b_endpoint_settle_position_tolerance_mm
                ):
                    reasons.append("position_bridge_acknowledgement_error")
            else:
                if first_target_error > self.config.handoff_first_target_tolerance_mm:
                    reasons.append("act_b_first_target_not_at_bridge_endpoint")
                if not np.allclose(
                    self.current_plan.bridge.velocity(1.0),
                    self.b_assessment.intended_velocity_mm_s,
                    atol=1e-8,
                    rtol=1e-8,
                ):
                    reasons.append("bridge_terminal_velocity_internal_mismatch")
        if reasons:
            return self._fail(observation.timestamp_s, *reasons)

        assert self.b_generation is not None
        self.policy_b.activate(self.b_generation)
        for _ in range(self.b_action_start_index):
            if self.policy_b.pop_action() is None:
                return self._fail(
                    observation.timestamp_s,
                    "act_b_overlap_prefix_discard_failed",
                )
        self.phase = RuntimePhase.B_RUNNING
        self._event(
            observation.timestamp_s,
            "atomic_bridge_to_act_b_handoff",
            handoff_mode=(
                "stopped_endpoint_direct"
                if self.b_stopped_endpoint_direct_handoff
                else (
                    "stopped_endpoint_position_bridge"
                    if self.b_stopped_endpoint_position_bridge_handoff
                    else "velocity_matched_tail"
                )
            ),
            generation=self.b_generation,
            generation_role=self.b_generation_role,
            b_action_start_index=self.b_action_start_index,
            b_skipped_prefix_duration_s=(
                self.b_action_start_index / self.b_chunk.action_hz
            ),
            a_queue_size=self.policy_a.queue_size,
            b_queue_size=self.policy_b.queue_size,
            full_pose_live_validation_required=(
                self.b_stopped_endpoint_direct_handoff
                or self.b_stopped_endpoint_position_bridge_handoff
            ),
            safety_limits_relaxed=False,
        )
        return self._step_b(observation)

    def _step_b(self, observation: RuntimeObservation) -> RuntimeCommandProposal:
        action = self.policy_b.pop_action()
        if action is None:
            return self._fail(observation.timestamp_s, "act_b_queue_empty")
        return RuntimeCommandProposal(
            timestamp_s=observation.timestamp_s,
            source="ACT-B",
            xyz_mm=action[:3],
            full_policy_action=action,
            gripper_closed=observation.semantic_state.gripper_closed,
            phase=self.phase.value,
        )

    def _fail(self, timestamp_s: float, *reasons: str) -> RuntimeCommandProposal:
        for reason in reasons:
            if reason not in self.failure_reasons:
                self.failure_reasons.append(reason)
        self.policy_a.deactivate_and_clear()
        self.policy_b.deactivate_and_clear()
        self.phase = RuntimePhase.FAILED_HOLD
        self._event(
            timestamp_s,
            "fail_closed_no_command",
            failure_reasons=list(self.failure_reasons),
        )
        return self._failed_proposal(timestamp_s)

    def _failed_proposal(self, timestamp_s: float) -> RuntimeCommandProposal:
        return RuntimeCommandProposal(
            timestamp_s=timestamp_s,
            source="FAIL_CLOSED_HOLD_REQUIRED",
            xyz_mm=None,
            gripper_closed=None,
            phase=self.phase.value,
            failure_reasons=tuple(self.failure_reasons),
        )
