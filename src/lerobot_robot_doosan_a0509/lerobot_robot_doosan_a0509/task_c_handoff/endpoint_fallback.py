"""Generic non-blocking endpoint fallback for multi-stage Task-C V2.

The moving handoff window remains the primary path.  If it expires, this
coordinator keeps returning the prevalidated Bridge endpoint while a *fresh*
successor request is prepared asynchronously.  A successor can take over only
after the same prefix/crossfade admission used by the moving V2 path passes.

This module owns no ROS objects and performs no blocking waits.  The caller is
responsible for sending every returned 7D action through the existing
LeRobot -> MUX -> safety -> ServoL path and for supplying acknowledged state.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any, Callable

import numpy as np

from offline_tools.task_c_bridge_v0.live_transition import pose_delta_metrics
from offline_tools.task_c_bridge_v0.runtime_policy import PolicyChunk

from .async_successor import AsyncSuccessorController, SuccessorResult
from .compatibility import HandoffCompatibilityEvaluator, PrefixAdmission
from .models import EpisodeHandoffManifest
from .soft_handoff import SoftHandoffPlan, build_soft_handoff


class EndpointFallbackState(str, Enum):
    HOLDING_ENDPOINT = "HOLDING_ENDPOINT"
    ENDPOINT_SETTLED = "ENDPOINT_SETTLED"
    B_SHADOW_PENDING = "B_SHADOW_PENDING"
    B_PREFIX_ADMISSION = "B_PREFIX_ADMISSION"
    SOFT_HANDOFF = "SOFT_HANDOFF"
    RUN_B = "RUN_B"
    FAILED_HOLD = "FAILED_HOLD"


@dataclass(frozen=True)
class EndpointFallbackConfig:
    control_hz: float
    settle_position_tolerance_mm: float
    settle_velocity_tolerance_mm_s: float
    settle_min_hold_s: float
    settle_timeout_s: float
    total_timeout_s: float
    request_retry_interval_steps: int
    crossfade_steps: int

    def __post_init__(self) -> None:
        positive = (
            self.control_hz,
            self.settle_position_tolerance_mm,
            self.settle_velocity_tolerance_mm_s,
            self.settle_min_hold_s,
            self.settle_timeout_s,
            self.total_timeout_s,
        )
        if any(not np.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("endpoint fallback timing/limits must be positive")
        if self.settle_timeout_s > self.total_timeout_s:
            raise ValueError("settle_timeout_s cannot exceed total_timeout_s")
        for name in ("request_retry_interval_steps", "crossfade_steps"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 1:
                raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True)
class EndpointFallbackEvent:
    timestamp_s: float
    state: str
    event: str
    details: dict[str, Any]


@dataclass(frozen=True)
class EndpointFallbackCommand:
    action: np.ndarray | None
    source: str
    state: EndpointFallbackState
    crossfade_weight: float | None = None
    successor_generation: int | None = None
    transition_to_b_after_commit: bool = False
    failure_reason: str | None = None

    def __post_init__(self) -> None:
        if self.action is None:
            return
        action = np.asarray(self.action, dtype=np.float64)
        if action.shape != (7,) or not np.all(np.isfinite(action)):
            raise ValueError("endpoint fallback command must be finite 7D")
        object.__setattr__(self, "action", action.copy())


class MultiEndpointFallbackCoordinator:
    """Hold a Bridge endpoint while preparing a fresh successor non-blockingly."""

    def __init__(
        self,
        *,
        endpoint_action: np.ndarray,
        manifest: EpisodeHandoffManifest,
        config: EndpointFallbackConfig,
        successor: AsyncSuccessorController,
        evaluator: HandoffCompatibilityEvaluator,
        started_s: float,
        moving_failure_reason: str,
        event_callback: Callable[[EndpointFallbackEvent], None] | None = None,
    ) -> None:
        endpoint = np.asarray(endpoint_action, dtype=np.float64)
        if endpoint.shape != (7,) or not np.all(np.isfinite(endpoint)):
            raise ValueError("endpoint_action must be finite shape (7,)")
        endpoint = endpoint.copy()
        endpoint[6] = 1.0 if endpoint[6] >= 0.5 else 0.0
        if successor.inflight_generation is not None:
            raise ValueError("moving-window successor generation was not invalidated")
        self.endpoint_action = endpoint
        self.manifest = manifest
        self.config = config
        self.successor = successor
        self.evaluator = evaluator
        self.started_s = float(started_s)
        self.moving_failure_reason = str(moving_failure_reason)
        self.event_callback = event_callback
        self.state = EndpointFallbackState.HOLDING_ENDPOINT
        self.events: list[EndpointFallbackEvent] = []
        self._settled_since_s: float | None = None
        self._tick_index = 0
        self._last_request_tick: int | None = None
        self._request_count = 0
        self._rejection_count = 0
        self._last_admission: PrefixAdmission | None = None
        self._ready_chunk: PolicyChunk | None = None
        self._soft_plan: SoftHandoffPlan | None = None
        self._soft_index = 0
        self._failure_reason: str | None = None
        self._takeover_timestamp_s: float | None = None
        self._last_settle_metrics: dict[str, Any] = {}
        self._event(
            self.started_s,
            "endpoint_hold_started",
            endpoint_action=self.endpoint_action.tolist(),
            moving_failure_reason=self.moving_failure_reason,
            stale_moving_generation_invalidated=True,
        )

    @property
    def last_admission(self) -> PrefixAdmission | None:
        return self._last_admission

    @property
    def failure_reason(self) -> str | None:
        return self._failure_reason

    def _event(self, timestamp_s: float, event: str, **details: Any) -> None:
        value = EndpointFallbackEvent(
            timestamp_s=float(timestamp_s),
            state=self.state.value,
            event=event,
            details=details,
        )
        self.events.append(value)
        if self.event_callback is not None:
            self.event_callback(value)

    def _set_state(
        self,
        state: EndpointFallbackState,
        timestamp_s: float,
        *,
        reason: str,
    ) -> None:
        previous = self.state
        self.state = state
        self._event(
            timestamp_s,
            "state_transition",
            previous=previous.value,
            current=state.value,
            reason=reason,
        )

    def _hold_command(self) -> EndpointFallbackCommand:
        return EndpointFallbackCommand(
            self.endpoint_action,
            "ENDPOINT_HOLD_V2",
            self.state,
            successor_generation=self.successor.inflight_generation,
        )

    def _fail(self, timestamp_s: float, reason: str) -> EndpointFallbackCommand:
        self.successor.invalidate()
        self._failure_reason = str(reason)
        self._set_state(
            EndpointFallbackState.FAILED_HOLD,
            timestamp_s,
            reason=reason,
        )
        self._event(
            timestamp_s,
            "endpoint_fallback_fail_closed",
            failure_reason=reason,
            settle_metrics=dict(self._last_settle_metrics),
        )
        return EndpointFallbackCommand(
            None,
            "FAIL_CLOSED_HOLD_REQUIRED",
            self.state,
            failure_reason=reason,
        )

    def _settle_metrics(
        self,
        *,
        actual_pose_mm_deg: np.ndarray,
        acknowledged_pose_mm_deg: np.ndarray,
        actual_velocity_mm_s: np.ndarray,
    ) -> dict[str, Any]:
        actual = np.asarray(actual_pose_mm_deg, dtype=np.float64)
        acknowledged = np.asarray(acknowledged_pose_mm_deg, dtype=np.float64)
        velocity = np.asarray(actual_velocity_mm_s, dtype=np.float64)
        if actual.shape != (6,) or not np.all(np.isfinite(actual)):
            raise ValueError("actual endpoint pose must be finite 6D")
        if acknowledged.shape != (6,) or not np.all(np.isfinite(acknowledged)):
            raise ValueError("acknowledged endpoint pose must be finite 6D")
        if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
            raise ValueError("actual endpoint velocity must be finite XYZ")
        actual_position, actual_rotation = pose_delta_metrics(
            actual,
            self.endpoint_action[:6],
        )
        acknowledged_position, acknowledged_rotation = pose_delta_metrics(
            acknowledged,
            self.endpoint_action[:6],
        )
        speed = float(np.linalg.norm(velocity))
        within = bool(
            actual_position <= self.config.settle_position_tolerance_mm
            and acknowledged_position <= self.config.settle_position_tolerance_mm
            and speed <= self.config.settle_velocity_tolerance_mm_s
        )
        return {
            "actual_position_error_mm": float(actual_position),
            "actual_rotation_error_deg": float(actual_rotation),
            "acknowledged_position_error_mm": float(acknowledged_position),
            "acknowledged_rotation_error_deg": float(acknowledged_rotation),
            "actual_speed_mm_s": speed,
            "within_settle_limits": within,
        }

    def tick(
        self,
        *,
        timestamp_s: float,
        actual_pose_mm_deg: np.ndarray,
        acknowledged_pose_mm_deg: np.ndarray,
        actual_velocity_mm_s: np.ndarray,
        command_history_mm_deg: np.ndarray,
        actual_semantic: Any,
        policy_input: Any,
        observation_timestamp_s: float,
    ) -> EndpointFallbackCommand:
        """Advance one fixed-rate decision without waiting for inference."""

        now_s = float(timestamp_s)
        self._tick_index += 1
        if self.state is EndpointFallbackState.SOFT_HANDOFF:
            return self._tick_soft_handoff(now_s)
        if self.state is EndpointFallbackState.RUN_B:
            return EndpointFallbackCommand(None, "ACT-B-OWNED", self.state)
        if self.state is EndpointFallbackState.FAILED_HOLD:
            return EndpointFallbackCommand(
                None,
                "FAIL_CLOSED_HOLD_REQUIRED",
                self.state,
                failure_reason=self._failure_reason,
            )
        if now_s - self.started_s > self.config.total_timeout_s:
            return self._fail(now_s, "endpoint_fallback_total_timeout")

        metrics = self._settle_metrics(
            actual_pose_mm_deg=actual_pose_mm_deg,
            acknowledged_pose_mm_deg=acknowledged_pose_mm_deg,
            actual_velocity_mm_s=actual_velocity_mm_s,
        )
        self._last_settle_metrics = metrics
        if not metrics["within_settle_limits"]:
            if self._settled_since_s is not None:
                invalidated = self.successor.invalidate()
                self._event(
                    now_s,
                    "endpoint_settle_lost",
                    invalidated_generation=invalidated,
                    **metrics,
                )
            self._settled_since_s = None
            if now_s - self.started_s > self.config.settle_timeout_s:
                return self._fail(now_s, "bridge_endpoint_settle_timeout")
            if self.state is not EndpointFallbackState.HOLDING_ENDPOINT:
                self._set_state(
                    EndpointFallbackState.HOLDING_ENDPOINT,
                    now_s,
                    reason="endpoint_not_settled",
                )
            return self._hold_command()

        if self._settled_since_s is None:
            self._settled_since_s = now_s
            self._event(now_s, "endpoint_settle_candidate", **metrics)
        settled_elapsed_s = now_s - self._settled_since_s
        if settled_elapsed_s < self.config.settle_min_hold_s:
            return self._hold_command()
        if self.state is EndpointFallbackState.HOLDING_ENDPOINT:
            self._set_state(
                EndpointFallbackState.ENDPOINT_SETTLED,
                now_s,
                reason="continuous_endpoint_settle_confirmed",
            )
            self._event(
                now_s,
                "endpoint_settled",
                continuous_hold_s=settled_elapsed_s,
                **metrics,
            )

        outer_ready, outer_reasons, outer_metrics = self.evaluator.outer_ready(
            manifest=self.manifest,
            actual_semantic=actual_semantic,
            current_pose_mm_deg=np.asarray(actual_pose_mm_deg, dtype=np.float64),
        )
        retry_ready = bool(
            self._last_request_tick is None
            or self._tick_index - self._last_request_tick
            >= self.config.request_retry_interval_steps
        )
        if (
            outer_ready
            and retry_ready
            and self.successor.inflight_generation is None
        ):
            generation = self.successor.request(
                policy_input,
                observation_timestamp_s=observation_timestamp_s,
            )
            if generation is not None:
                self._last_request_tick = self._tick_index
                self._request_count += 1
                self._set_state(
                    EndpointFallbackState.B_SHADOW_PENDING,
                    now_s,
                    reason="fresh_endpoint_observation_ready",
                )
                self._event(
                    now_s,
                    "endpoint_act_b_shadow_requested",
                    generation=generation,
                    request_count=self._request_count,
                    observation_timestamp_s=observation_timestamp_s,
                    **outer_metrics,
                )
        elif not outer_ready:
            self._event(
                now_s,
                "endpoint_outer_not_ready",
                failure_reasons=list(outer_reasons),
                **outer_metrics,
            )

        result = self.successor.poll(now_s=now_s)
        if result.status in {"idle", "pending"}:
            return self._hold_command()
        self._event_successor_result(now_s, result)
        if result.chunk is None:
            self._last_request_tick = self._tick_index
            if self.state is not EndpointFallbackState.ENDPOINT_SETTLED:
                self._set_state(
                    EndpointFallbackState.ENDPOINT_SETTLED,
                    now_s,
                    reason="fresh_successor_result_unavailable",
                )
            return self._hold_command()

        chunk = result.chunk
        history = np.asarray(command_history_mm_deg, dtype=np.float64)
        if history.shape != (2, 6) or not np.all(np.isfinite(history)):
            self.successor.reject_ready()
            self._last_request_tick = self._tick_index
            self._rejection_count += 1
            self._set_state(
                EndpointFallbackState.ENDPOINT_SETTLED,
                now_s,
                reason="acknowledged_command_history_unavailable",
            )
            self._event(
                now_s,
                "endpoint_command_history_unavailable",
                command_history_shape=list(history.shape),
            )
            return self._hold_command()

        self._set_state(
            EndpointFallbackState.B_PREFIX_ADMISSION,
            now_s,
            reason="fresh_endpoint_successor_chunk_ready",
        )
        bridge_actions = np.repeat(
            self.endpoint_action[None, :],
            self.config.crossfade_steps,
            axis=0,
        )
        plan = build_soft_handoff(
            bridge_actions,
            chunk.actions,
            steps=self.config.crossfade_steps,
            held_gripper_target=float(self.endpoint_action[6]),
        )
        continuation_index = self.config.crossfade_steps
        post_crossfade_action = (
            chunk.actions[continuation_index]
            if len(chunk.actions) > continuation_index
            else None
        )
        admission = self.evaluator.evaluate_prefix(
            chunk,
            bridge_reference_action=self.endpoint_action,
            bridge_velocity_mm_s=np.zeros(3, dtype=np.float64),
            expected_semantic=self.manifest.successor.semantic,
            soft_handoff_plan=plan,
            command_history_mm_deg=history,
            post_crossfade_action=post_crossfade_action,
        )
        self._last_admission = admission
        self._event(
            now_s,
            "endpoint_act_b_prefix_admission",
            generation=chunk.generation,
            compatibility="PASS" if admission.valid else "FAIL",
            admission=admission.record(),
        )
        if not admission.valid:
            self.successor.reject_ready()
            self._last_request_tick = self._tick_index
            self._rejection_count += 1
            self._set_state(
                EndpointFallbackState.ENDPOINT_SETTLED,
                now_s,
                reason="endpoint_prefix_incompatible",
            )
            return self._hold_command()

        self._ready_chunk = chunk
        self._soft_plan = plan
        self._soft_index = 0
        self._set_state(
            EndpointFallbackState.SOFT_HANDOFF,
            now_s,
            reason="fresh_endpoint_B_prefix_compatible",
        )
        self._event(
            now_s,
            "endpoint_soft_handoff_started",
            generation=chunk.generation,
            crossfade_steps=self.config.crossfade_steps,
        )
        return self._tick_soft_handoff(now_s)

    def _event_successor_result(
        self,
        timestamp_s: float,
        result: SuccessorResult,
    ) -> None:
        details: dict[str, Any] = {
            "status": result.status,
            "generation": result.generation,
            "stale": result.stale,
            "failure_reason": result.failure_reason,
        }
        if result.chunk is not None:
            chunk = result.chunk
            details.update(
                {
                    "request_timestamp_s": chunk.request_timestamp_s,
                    "observation_timestamp_s": chunk.observation_timestamp_s,
                    "completion_timestamp_s": chunk.completed_timestamp_s,
                    "total_preparation_latency_ms": (
                        chunk.request_to_completion_latency_s * 1000.0
                    ),
                }
            )
        if result.backend_timing is not None:
            details["backend_stage_timing"] = result.backend_timing.record_ms()
        self._event(timestamp_s, "endpoint_act_b_shadow_result", **details)

    def _tick_soft_handoff(self, timestamp_s: float) -> EndpointFallbackCommand:
        assert self._soft_plan is not None
        assert self._ready_chunk is not None
        index = self._soft_index
        action = self._soft_plan.actions[index]
        weight = float(self._soft_plan.weights[index])
        self._soft_index += 1
        final = self._soft_index >= len(self._soft_plan.actions)
        if final:
            self.successor.activate_ready(
                self._ready_chunk,
                skip_actions=self._soft_plan.b_actions_consumed,
            )
            self._takeover_timestamp_s = float(timestamp_s)
            self._set_state(
                EndpointFallbackState.RUN_B,
                timestamp_s,
                reason="endpoint_crossfade_complete",
            )
            self._event(
                timestamp_s,
                "endpoint_act_b_full_takeover",
                generation=self._ready_chunk.generation,
                crossfade_steps=self._soft_plan.b_actions_consumed,
            )
        return EndpointFallbackCommand(
            action,
            "ENDPOINT_SOFT_HANDOFF_V2",
            EndpointFallbackState.SOFT_HANDOFF,
            crossfade_weight=weight,
            successor_generation=self._ready_chunk.generation,
            transition_to_b_after_commit=final,
        )

    def record(self) -> dict[str, Any]:
        return {
            "state": self.state.value,
            "moving_failure_reason": self.moving_failure_reason,
            "endpoint_action": self.endpoint_action.tolist(),
            "started_s": self.started_s,
            "takeover_timestamp_s": self._takeover_timestamp_s,
            "request_count": self._request_count,
            "rejection_count": self._rejection_count,
            "failure_reason": self._failure_reason,
            "settle_metrics": dict(self._last_settle_metrics),
            "last_admission": (
                None
                if self._last_admission is None
                else self._last_admission.record()
            ),
            "config": asdict(self.config),
        }
