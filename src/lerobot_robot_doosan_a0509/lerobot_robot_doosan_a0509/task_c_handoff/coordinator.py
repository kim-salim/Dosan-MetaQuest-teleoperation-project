"""Pure non-blocking Task-C V2 Bridge-to-successor coordinator."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

from offline_tools.task_c_bridge_v0.runtime_policy import PolicyChunk

from .async_successor import AsyncSuccessorController, SuccessorResult
from .bridge_runtime import PrecomputedBridgeQueue
from .compatibility import HandoffCompatibilityEvaluator, PrefixAdmission
from .models import (
    BoundarySemanticState,
    EpisodeHandoffManifest,
    HandoffV2Config,
    HandoffV2State,
)
from .soft_handoff import SoftHandoffPlan, build_soft_handoff


@dataclass(frozen=True)
class V2CoordinatorEvent:
    timestamp_s: float
    state: str
    event: str
    details: dict[str, Any]


@dataclass(frozen=True)
class HandoffCommand:
    action: np.ndarray | None
    source: str
    state: HandoffV2State
    bridge_index: int | None = None
    bridge_progress: float | None = None
    handoff_window_progress: float | None = None
    crossfade_weight: float | None = None
    successor_generation: int | None = None
    transition_to_b_after_commit: bool = False
    endpoint_fallback_required: bool = False
    failure_reason: str | None = None

    def __post_init__(self) -> None:
        if self.action is None:
            return
        action = np.asarray(self.action, dtype=np.float64)
        if action.shape != (7,) or not np.all(np.isfinite(action)):
            raise ValueError("handoff command action must be finite 7D")
        object.__setattr__(self, "action", action.copy())


class TaskCHandoffV2Coordinator:
    """Consume precomputed targets while ACT-B inference proceeds in shadow."""

    def __init__(
        self,
        *,
        manifest: EpisodeHandoffManifest,
        config: HandoffV2Config,
        successor: AsyncSuccessorController,
        evaluator: HandoffCompatibilityEvaluator,
        event_callback: Callable[[V2CoordinatorEvent], None] | None = None,
    ) -> None:
        if not config.enabled:
            raise ValueError("Task-C V2 coordinator requires enabled=true")
        if config.semantic_authority is not manifest.semantic_authority:
            raise ValueError("V2 manifest/config semantic_authority mismatch")
        if evaluator.semantic_authority is not config.semantic_authority:
            raise ValueError("V2 evaluator/config semantic_authority mismatch")
        self.manifest = manifest
        self.config = config
        self.successor = successor
        self.evaluator = evaluator
        self.event_callback = event_callback
        self.state = HandoffV2State.WAITING
        self.bridge: PrecomputedBridgeQueue | None = None
        self.bridge_index = 0
        self._soft_plan: SoftHandoffPlan | None = None
        self._soft_index = 0
        self._ready_chunk: PolicyChunk | None = None
        self._ready_splice_index = 0
        self._last_request_bridge_index: int | None = None
        self._last_admission: PrefixAdmission | None = None
        self._fallback_reason: str | None = None
        self._takeover_timestamp_s: float | None = None
        self.events: list[V2CoordinatorEvent] = []

    @property
    def last_admission(self) -> PrefixAdmission | None:
        return self._last_admission

    @property
    def fallback_reason(self) -> str | None:
        return self._fallback_reason

    def _event(self, timestamp_s: float, event: str, **details: Any) -> None:
        value = V2CoordinatorEvent(
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
        state: HandoffV2State,
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

    def start_bridge(
        self,
        bridge: PrecomputedBridgeQueue,
        *,
        timestamp_s: float,
    ) -> None:
        if self.state not in {
            HandoffV2State.WAITING,
            HandoffV2State.RUN_A,
            HandoffV2State.A_EXIT_COMMIT,
            HandoffV2State.PREPARE_BRIDGE,
        }:
            raise RuntimeError(f"cannot start V2 Bridge from {self.state.value}")
        self.successor.invalidate()
        self.bridge = bridge
        self.bridge_index = 0
        self._soft_plan = None
        self._soft_index = 0
        self._ready_chunk = None
        self._ready_splice_index = 0
        self._last_request_bridge_index = None
        self._last_admission = None
        self._fallback_reason = None
        self._set_state(
            HandoffV2State.RUN_BRIDGE,
            timestamp_s,
            reason="precomputed_bridge_ready",
        )
        self._event(timestamp_s, "bridge_queue_started", **bridge.record())

    def mark_policies_loaded(self, *, timestamp_s: float) -> None:
        self._set_state(
            HandoffV2State.LOAD_POLICIES,
            timestamp_s,
            reason="frozen_ACT_A_and_ACT_B_loaded",
        )
        self._set_state(
            HandoffV2State.WAITING,
            timestamp_s,
            reason="waiting_for_external_live_gate",
        )

    def mark_run_a(self, *, timestamp_s: float) -> None:
        self._set_state(HandoffV2State.RUN_A, timestamp_s, reason="ACT_A_active")

    def mark_exit_commit(self, *, timestamp_s: float) -> None:
        self._set_state(
            HandoffV2State.A_EXIT_COMMIT,
            timestamp_s,
            reason="selected_source_exit_committed",
        )

    def mark_prepare_bridge(self, *, timestamp_s: float) -> None:
        self._set_state(
            HandoffV2State.PREPARE_BRIDGE,
            timestamp_s,
            reason="actual_state_snapshot_captured",
        )

    def tick(
        self,
        *,
        timestamp_s: float,
        actual_pose_mm_deg: np.ndarray,
        command_history_mm_deg: np.ndarray,
        actual_semantic: BoundarySemanticState,
        policy_input: Any,
        observation_timestamp_s: float,
    ) -> HandoffCommand:
        if self.state is HandoffV2State.SOFT_HANDOFF:
            return self._tick_soft_handoff(timestamp_s)
        if self.state is HandoffV2State.RUN_B:
            return HandoffCommand(None, "ACT-B-OWNED", self.state)
        if self.state is HandoffV2State.ENDPOINT_FALLBACK:
            return HandoffCommand(
                None,
                "ENDPOINT_FALLBACK",
                self.state,
                endpoint_fallback_required=True,
                failure_reason=self._fallback_reason,
            )
        if self.state is HandoffV2State.FAILED_HOLD:
            return HandoffCommand(
                None,
                "FAIL_CLOSED_HOLD_REQUIRED",
                self.state,
                failure_reason=self._fallback_reason,
            )
        if self.bridge is None or self.state not in {
            HandoffV2State.RUN_BRIDGE,
            HandoffV2State.HANDOFF_WINDOW,
            HandoffV2State.B_SHADOW_PENDING,
            HandoffV2State.B_PREFIX_ADMISSION,
        }:
            raise RuntimeError(f"V2 tick is invalid in state {self.state.value}")

        if self.bridge_index >= self.bridge.steps:
            return self._enter_fallback(timestamp_s, "handoff_window_expired")

        if self.bridge.in_handoff_window(self.bridge_index):
            if self.state is HandoffV2State.RUN_BRIDGE:
                self._set_state(
                    HandoffV2State.HANDOFF_WINDOW,
                    timestamp_s,
                    reason="bridge_window_index_reached",
                )
                self._event(
                    timestamp_s,
                    "handoff_window_entered",
                    bridge_index=self.bridge_index,
                    window_start_index=self.bridge.handoff_window_start_index,
                    remaining_steps=self.bridge.remaining_steps(self.bridge_index),
                )
            command = self._handoff_window_tick(
                timestamp_s=timestamp_s,
                actual_pose_mm_deg=actual_pose_mm_deg,
                command_history_mm_deg=command_history_mm_deg,
                actual_semantic=actual_semantic,
                policy_input=policy_input,
                observation_timestamp_s=observation_timestamp_s,
            )
            if command is not None:
                return command

        return self._pop_bridge_command(timestamp_s)

    def _handoff_window_tick(
        self,
        *,
        timestamp_s: float,
        actual_pose_mm_deg: np.ndarray,
        command_history_mm_deg: np.ndarray,
        actual_semantic: BoundarySemanticState,
        policy_input: Any,
        observation_timestamp_s: float,
    ) -> HandoffCommand | None:
        assert self.bridge is not None
        outer_ready, outer_reasons, outer_metrics = self.evaluator.outer_ready(
            manifest=self.manifest,
            actual_semantic=actual_semantic,
            current_pose_mm_deg=actual_pose_mm_deg,
        )
        remaining = self.bridge.remaining_steps(self.bridge_index)
        enough_crossfade = remaining >= self.config.crossfade_steps
        retry_ready = bool(
            self._last_request_bridge_index is None
            or self.bridge_index - self._last_request_bridge_index
            >= self.config.request_retry_interval_steps
        )
        if (
            outer_ready
            and enough_crossfade
            and retry_ready
            and self.successor.inflight_generation is None
        ):
            generation = self.successor.request(
                policy_input,
                observation_timestamp_s=observation_timestamp_s,
            )
            if generation is not None:
                self._last_request_bridge_index = self.bridge_index
                self._set_state(
                    HandoffV2State.B_SHADOW_PENDING,
                    timestamp_s,
                    reason="outer_compatibility_ready",
                )
                self._event(
                    timestamp_s,
                    "act_b_shadow_requested",
                    generation=generation,
                    bridge_index=self.bridge_index,
                    observation_timestamp_s=observation_timestamp_s,
                    **outer_metrics,
                )
        elif not outer_ready and self.state is HandoffV2State.HANDOFF_WINDOW:
            self._event(
                timestamp_s,
                "act_b_outer_not_ready",
                failure_reasons=list(outer_reasons),
                bridge_index=self.bridge_index,
                **outer_metrics,
            )

        result = self.successor.poll(now_s=timestamp_s)
        if result.status == "pending" or result.status == "idle":
            return None
        self._event_successor_result(timestamp_s, result)
        if result.chunk is None:
            return None

        chunk = result.chunk
        self._set_state(
            HandoffV2State.B_PREFIX_ADMISSION,
            timestamp_s,
            reason="fresh_successor_chunk_ready",
        )
        remaining = self.bridge.remaining_steps(self.bridge_index)
        if remaining < self.config.crossfade_steps:
            self.successor.reject_ready()
            self._set_state(
                HandoffV2State.HANDOFF_WINDOW,
                timestamp_s,
                reason="insufficient_crossfade_tail",
            )
            return None

        history = np.asarray(command_history_mm_deg, dtype=np.float64)
        if (
            history.shape != (2, 6)
            or not np.all(np.isfinite(history))
        ):
            self.successor.reject_ready()
            self._set_state(
                HandoffV2State.HANDOFF_WINDOW,
                timestamp_s,
                reason="acknowledged_command_history_unavailable",
            )
            self._event(
                timestamp_s,
                "act_b_command_history_unavailable",
                command_history_shape=list(history.shape),
            )
            return None

        candidates: list[tuple[float, int, SoftHandoffPlan, PrefixAdmission]] = []
        rejected_admissions: list[PrefixAdmission] = []
        indices = self.evaluator.splice_selector.candidate_indices(
            chunk,
            prefix_steps=max(
                self.config.b_prefix_steps,
                self.config.crossfade_steps + 1,
            ),
        )
        for splice_index in indices:
            successor_actions = chunk.actions[splice_index:]
            if len(successor_actions) < self.config.crossfade_steps:
                continue
            prospective = build_soft_handoff(
                self.bridge.actions[self.bridge_index :],
                successor_actions,
                steps=self.config.crossfade_steps,
                held_gripper_target=float(
                    self.bridge.actions[self.bridge_index, 6]
                ),
            )
            continuation_index = splice_index + self.config.crossfade_steps
            post_crossfade_action = (
                chunk.actions[continuation_index]
                if len(chunk.actions) > continuation_index
                else None
            )
            evaluated = self.evaluator.evaluate_prefix(
                chunk,
                bridge_reference_action=self.bridge.actions[self.bridge_index],
                bridge_velocity_mm_s=self.bridge.velocity_mm_s[self.bridge_index],
                expected_semantic=self.manifest.successor.semantic,
                soft_handoff_plan=prospective,
                command_history_mm_deg=command_history_mm_deg,
                post_crossfade_action=post_crossfade_action,
                splice_index=splice_index,
            )
            selection_score = None
            if not evaluated.valid:
                self._event(
                    timestamp_s,
                    "act_b_splice_candidate",
                    generation=chunk.generation,
                    bridge_index=self.bridge_index,
                    splice_index=splice_index,
                    compatibility="FAIL",
                    selection_score=None,
                    admission=evaluated.record(),
                )
                rejected_admissions.append(evaluated)
                continue
            dynamics = evaluated.dynamics
            # Ranking only; all hard command/semantic checks have already
            # passed. This lets a fresh ACT-B chunk select another inference
            # region without turning the score into a safety decision.
            selection_score = (
                dynamics.first_xyz_delta_mm
                + dynamics.first_rotation_delta_deg
                + 0.01 * dynamics.bridge_prefix_velocity_mismatch_mm_s
            )
            self._event(
                timestamp_s,
                "act_b_splice_candidate",
                generation=chunk.generation,
                bridge_index=self.bridge_index,
                splice_index=splice_index,
                compatibility="PASS",
                selection_score=selection_score,
                admission=evaluated.record(),
            )
            candidates.append(
                (selection_score, splice_index, prospective, evaluated)
            )

        if candidates:
            selected_score, selected_splice, prospective_plan, admission = min(
                candidates,
                key=lambda value: (value[0], value[1]),
            )
        else:
            selected_score = None
            selected_splice = 0
            prospective_plan = None
            admission = (
                rejected_admissions[0]
                if rejected_admissions
                else None
            )
        self._last_admission = admission
        self._event(
            timestamp_s,
            "act_b_prefix_admission",
            generation=chunk.generation,
            bridge_index=self.bridge_index,
            splice_candidates=list(indices),
            selected_splice_index=(
                selected_splice if prospective_plan is not None else None
            ),
            selected_splice_score=selected_score,
            compatibility=(
                "PASS"
                if admission is not None and admission.valid
                else "FAIL"
            ),
            admission=None if admission is None else admission.record(),
        )
        if admission is None or not admission.valid or prospective_plan is None:
            self.successor.reject_ready()
            self._set_state(
                HandoffV2State.HANDOFF_WINDOW,
                timestamp_s,
                reason="prefix_incompatible",
            )
            return None
        if not self.config.enable_soft_handoff:
            self.successor.reject_ready()
            return self._enter_fallback(timestamp_s, "soft_handoff_disabled")

        self._ready_chunk = chunk
        self._ready_splice_index = selected_splice
        self._soft_plan = prospective_plan
        self._soft_index = 0
        self._set_state(
            HandoffV2State.SOFT_HANDOFF,
            timestamp_s,
            reason="B_prefix_compatible",
        )
        self._event(
            timestamp_s,
            "soft_handoff_started",
            generation=chunk.generation,
            splice_index=selected_splice,
            bridge_index=self.bridge_index,
            crossfade_steps=self.config.crossfade_steps,
            bridge_progress=self.bridge_index / float(self.bridge.steps),
        )
        return self._tick_soft_handoff(timestamp_s)

    def _event_successor_result(
        self,
        timestamp_s: float,
        result: SuccessorResult,
    ) -> None:
        chunk = result.chunk
        details: dict[str, Any] = {
            "status": result.status,
            "generation": result.generation,
            "stale": result.stale,
            "failure_reason": result.failure_reason,
        }
        if chunk is not None:
            details.update(
                {
                    "request_timestamp_s": chunk.request_timestamp_s,
                    "observation_timestamp_s": chunk.observation_timestamp_s,
                    "completion_timestamp_s": chunk.completed_timestamp_s,
                    "snapshot_latency_ms": (
                        chunk.request_snapshot_latency_s * 1000.0
                    ),
                    "gpu_wait_latency_ms": chunk.gpu_wait_latency_s * 1000.0,
                    "backend_total_latency_ms": (
                        chunk.backend_inference_latency_s * 1000.0
                    ),
                    "total_preparation_latency_ms": (
                        chunk.request_to_completion_latency_s * 1000.0
                    ),
                }
            )
        if result.backend_timing is not None:
            details["backend_stage_timing"] = result.backend_timing.record_ms()
        self._event(timestamp_s, "act_b_shadow_result", **details)

    def _pop_bridge_command(self, timestamp_s: float) -> HandoffCommand:
        assert self.bridge is not None
        if self.bridge_index >= self.bridge.steps:
            return self._enter_fallback(timestamp_s, "bridge_queue_exhausted")
        index = self.bridge_index
        action = self.bridge.actions[index]
        self.bridge_index += 1
        window_progress = None
        if index >= self.bridge.handoff_window_start_index:
            denominator = max(
                1,
                self.bridge.steps - self.bridge.handoff_window_start_index,
            )
            window_progress = (
                index - self.bridge.handoff_window_start_index + 1
            ) / float(denominator)
        return HandoffCommand(
            action,
            "BEZIER_BRIDGE_V2",
            self.state,
            bridge_index=index,
            bridge_progress=(index + 1) / float(self.bridge.steps),
            handoff_window_progress=window_progress,
            successor_generation=self.successor.inflight_generation,
        )

    def _tick_soft_handoff(self, timestamp_s: float) -> HandoffCommand:
        assert self.bridge is not None
        assert self._soft_plan is not None
        assert self._ready_chunk is not None
        index = self._soft_index
        action = self._soft_plan.actions[index]
        weight = float(self._soft_plan.weights[index])
        bridge_index = self.bridge_index
        self._soft_index += 1
        self.bridge_index += 1
        final = self._soft_index >= len(self._soft_plan.actions)
        if final:
            self.successor.activate_ready(
                self._ready_chunk,
                skip_actions=(
                    self._ready_splice_index
                    + self._soft_plan.b_actions_consumed
                ),
            )
            self._takeover_timestamp_s = timestamp_s
            self._set_state(
                HandoffV2State.RUN_B,
                timestamp_s,
                reason="crossfade_complete",
            )
            self._event(
                timestamp_s,
                "act_b_full_takeover",
                generation=self._ready_chunk.generation,
                splice_index=self._ready_splice_index,
                crossfade_steps=self._soft_plan.b_actions_consumed,
                bridge_progress=self.bridge_index / float(self.bridge.steps),
            )
        return HandoffCommand(
            action,
            "SOFT_HANDOFF_V2",
            HandoffV2State.SOFT_HANDOFF,
            bridge_index=bridge_index,
            bridge_progress=self.bridge_index / float(self.bridge.steps),
            crossfade_weight=weight,
            successor_generation=self._ready_chunk.generation,
            transition_to_b_after_commit=final,
        )

    def _enter_fallback(self, timestamp_s: float, reason: str) -> HandoffCommand:
        self.successor.invalidate()
        self._fallback_reason = reason
        if not self.config.enable_endpoint_fallback:
            self.state = HandoffV2State.FAILED_HOLD
            self._event(
                timestamp_s,
                "v2_fail_closed",
                failure_reason=reason,
                endpoint_fallback_enabled=False,
            )
            return HandoffCommand(
                None,
                "FAIL_CLOSED_HOLD_REQUIRED",
                self.state,
                failure_reason=reason,
            )
        self._set_state(
            HandoffV2State.ENDPOINT_FALLBACK,
            timestamp_s,
            reason=reason,
        )
        self._event(
            timestamp_s,
            "endpoint_v1_fallback_requested",
            failure_reason=reason,
            bridge_steps_executed=self.bridge_index,
        )
        return HandoffCommand(
            None,
            "ENDPOINT_FALLBACK",
            self.state,
            endpoint_fallback_required=True,
            failure_reason=reason,
        )

    def mark_complete(self, *, timestamp_s: float) -> None:
        if self.state is not HandoffV2State.RUN_B:
            raise RuntimeError(
                f"cannot complete V2 Task-C from {self.state.value}"
            )
        self._set_state(
            HandoffV2State.COMPLETE,
            timestamp_s,
            reason="ACT_B_semantic_completion_confirmed",
        )

    def mark_endpoint_takeover(
        self,
        *,
        timestamp_s: float,
        generation: int,
        crossfade_steps: int,
    ) -> None:
        """Record a generic Multi-V2 endpoint fallback takeover.

        The endpoint fallback owns its fresh inference and crossfade, but the
        main V2 coordinator remains the authoritative lifecycle state exposed
        to recording, summaries, and fail-closed handling.
        """

        if self.state is not HandoffV2State.ENDPOINT_FALLBACK:
            raise RuntimeError(
                "endpoint takeover requires ENDPOINT_FALLBACK state, got "
                f"{self.state.value}"
            )
        if generation < 1 or crossfade_steps < 1:
            raise ValueError("endpoint takeover metadata must be positive")
        self._takeover_timestamp_s = float(timestamp_s)
        self._set_state(
            HandoffV2State.RUN_B,
            timestamp_s,
            reason="multi_endpoint_crossfade_complete",
        )
        self._event(
            timestamp_s,
            "act_b_full_takeover",
            generation=int(generation),
            crossfade_steps=int(crossfade_steps),
            bridge_progress=1.0,
            endpoint_fallback=True,
        )

    def fail_closed(self, *, timestamp_s: float, reason: str) -> None:
        self.successor.invalidate()
        self._fallback_reason = reason
        self.state = HandoffV2State.FAILED_HOLD
        self._event(timestamp_s, "v2_fail_closed", failure_reason=reason)
