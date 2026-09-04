"""Opt-in multi-stage composition built from the existing Task-C V2 edge.

The reviewed ``task_c_live_v2`` implementation remains the sole owner of one
Bridge -> asynchronous successor -> prefix admission -> soft-crossfade edge.
This module adds only an episode-level stage sequencer and a resident policy
registry so a plan such as T4 -> T2 -> T4 can execute two independent V2
edges without reloading either checkpoint.

Each stage declares either external-planner or preloaded phase-supervisor exit
authority. External service callbacks only update a locked mailbox; the
automatic supervisor performs one bounded NumPy support query per observation.
STRICT Bridge generation, queue invalidation, and command dispatch retain the
existing control-thread path. FLEXIBLE candidate generation runs in the
existing bounded Bridge worker; CUDA completion is never awaited there.
"""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
from lerobot.rollout.configs import RolloutStrategyConfig
from lerobot.rollout.strategies.core import RolloutStrategy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from offline_tools.cross_task_handoff.authority import (
    SemanticAuthority,
    parse_semantic_authority,
)
from offline_tools.task_c_bridge_v0.lerobot_act_backend import (
    LeRobotACTBackend,
    ResidentLeRobotACTBackend,
)
from offline_tools.task_c_bridge_v0.runtime_policy import AsyncPolicySession

from lerobot_robot_doosan_a0509.act_async_rollout import get_act_gpu_arbiter
from lerobot_robot_doosan_a0509.runtime_scheduling import (
    preserve_current_thread_affinity,
)
from lerobot_robot_doosan_a0509.task_c_handoff.async_successor import (
    AsyncSuccessorController,
    TimedACTBackend,
)
from lerobot_robot_doosan_a0509.task_c_handoff.bridge_runtime import (
    BridgeGenerationError,
)
from lerobot_robot_doosan_a0509.task_c_handoff.compatibility import (
    HandoffCompatibilityEvaluator,
    HandoffSpliceSelector,
)
from lerobot_robot_doosan_a0509.task_c_handoff.coordinator import (
    HandoffCommand,
    TaskCHandoffV2Coordinator,
)
from lerobot_robot_doosan_a0509.task_c_handoff.endpoint_fallback import (
    EndpointFallbackConfig,
    EndpointFallbackEvent,
    EndpointFallbackState,
    MultiEndpointFallbackCoordinator,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    BridgeAdmissionMode,
    HandoffV2State,
)
from lerobot_robot_doosan_a0509.task_c_handoff.multi_stage import (
    MultiStageCoordinator,
    MultiStageEvent,
    MultiStagePlan,
    MultiStageRuntimeState,
    MultiTransitionSpec,
)
from lerobot_robot_doosan_a0509.task_c_handoff.policy_registry import (
    PolicyRegistry,
)
from lerobot_robot_doosan_a0509.task_c_handoff.stage_supervisor import (
    StagePhaseSupervisor,
)
from lerobot_robot_doosan_a0509.task_c_handoff.source_queue_splice import (
    SourceActionQueueSnapshot,
    source_snapshot_from_async_session,
)
from lerobot_robot_doosan_a0509.task_c_live_rollout import (
    LivePhase,
    _LiveCapture,
    _PendingLiveCommand,
)
from lerobot_robot_doosan_a0509.task_c_live_v2_rollout import (
    TaskCLiveV2Strategy,
    TaskCLiveV2StrategyConfig,
)
from lerobot_robot_doosan_a0509.task_c_shadow_rollout import _same_checkpoint


@RolloutStrategyConfig.register_subclass("task_c_multi_live_v2")
@dataclass
class TaskCMultiLiveV2StrategyConfig(TaskCLiveV2StrategyConfig):
    """Configuration for an ordered sequence of existing V2 handoff edges."""

    multi_stage_plan: str = ""
    request_next_service: str = "/control/task_c/request_next_stage"
    complete_service: str = "/control/task_c/complete_episode"
    status_service: str = "/control/task_c/multi_stage_status"
    multi_bridge_retry_interval_s: float = 0.25

    def __post_init__(self) -> None:
        if not self.multi_stage_plan:
            raise ValueError("task_c_multi_live_v2 requires multi_stage_plan")
        plan = MultiStagePlan.load(self.multi_stage_plan)
        if plan.runtime_command_profile_id is not None:
            if (
                self.runtime_command_profile_id
                and self.runtime_command_profile_id
                != plan.runtime_command_profile_id
            ):
                raise ValueError(
                    "runtime command profile differs from multi-stage plan"
                )
            self.runtime_command_profile_id = (
                plan.runtime_command_profile_id
            )
        first_edge = plan.transitions[0]
        first_successor = plan.policy(plan.stages[1].policy_id)

        if self.handoff_episode_manifest:
            configured = Path(self.handoff_episode_manifest).expanduser().resolve()
            if configured != first_edge.handoff_manifest_path:
                raise ValueError(
                    "handoff_episode_manifest must equal the first multi-stage edge"
                )
        else:
            self.handoff_episode_manifest = str(first_edge.handoff_manifest_path)

        if self.checkpoint_b:
            if not _same_checkpoint(self.checkpoint_b, str(first_successor.checkpoint)):
                raise ValueError(
                    "checkpoint_b must equal the first multi-stage successor"
                )
        else:
            self.checkpoint_b = str(first_successor.checkpoint)

        authorities = {
            edge.handoff_manifest.semantic_authority for edge in plan.transitions
        }
        if authorities != {SemanticAuthority.EXTERNAL_PLANNER}:
            raise ValueError(
                "multi-stage V2 delegates stage semantics to the external planner; "
                "every edge manifest requires semantic_authority=external_planner"
            )
        if self.semantic_authority:
            if (
                parse_semantic_authority(self.semantic_authority)
                is not SemanticAuthority.EXTERNAL_PLANNER
            ):
                raise ValueError(
                    "task_c_multi_live_v2 requires semantic_authority=external_planner"
                )
        else:
            self.semantic_authority = SemanticAuthority.EXTERNAL_PLANNER.value

        super().__post_init__()
        final_stage = plan.stages[-1]
        if final_stage.exit_authority == "policy_inference_end":
            assert final_stage.inference_end_steps is not None
            if final_stage.inference_end_steps > self.b_execution_steps:
                raise ValueError(
                    "final inference_end_steps must not exceed "
                    "b_execution_steps watchdog"
                )
            inference_duration_s = (
                final_stage.inference_end_steps / self.downstream_control_hz
            )
            if inference_duration_s >= self.b_execution_timeout_s:
                raise ValueError(
                    "final inference horizon must finish before "
                    "b_execution_timeout_s"
                )
        for name, value in (
            ("request_next_service", self.request_next_service),
            ("complete_service", self.complete_service),
            ("status_service", self.status_service),
        ):
            if not value.startswith("/"):
                raise ValueError(f"{name} must be an absolute ROS service name")
        if (
            not np.isfinite(self.multi_bridge_retry_interval_s)
            or self.multi_bridge_retry_interval_s <= 0.0
        ):
            raise ValueError("multi_bridge_retry_interval_s must be positive")


class TaskCMultiLiveV2Strategy(TaskCLiveV2Strategy):
    """Repeat the unchanged V2 handoff edge across an ordered policy plan."""

    def __init__(self, config: TaskCMultiLiveV2StrategyConfig) -> None:
        super().__init__(config)
        self.multi_config = config
        self._multi_plan = MultiStagePlan.load(config.multi_stage_plan)
        self._multi_coordinator = MultiStageCoordinator(
            self._multi_plan,
            event_callback=self._on_multi_stage_event,
        )
        self._policy_registry: PolicyRegistry | None = None
        self._multi_ready_pub: Any = None
        self._multi_services: list[Any] = []
        self._multi_active_edge: MultiTransitionSpec | None = None
        self._multi_started = False
        self._multi_next_bridge_retry_s = 0.0
        self._multi_last_bridge_rejection: str | None = None
        self._multi_configured_edge_id: str | None = None
        self._multi_phase_supervisors: dict[str, StagePhaseSupervisor] = {}
        self._multi_latest_phase_status: dict[str, Any] = {}
        self._multi_phase_latency_ms: dict[str, list[float]] = {}
        self._multi_endpoint_fallback: MultiEndpointFallbackCoordinator | None = None
        self._multi_endpoint_fallback_records: list[dict[str, Any]] = []

    def _uses_legacy_endpoint_fallback(self) -> bool:
        """Multi V2 binds fallback to each active edge, never the V1 manifest."""

        return False

    def setup(self, ctx: Any) -> None:
        """Load each unique policy once; all synchronous warmup stays in setup."""

        super().setup(ctx)
        if (
            self._v2_runtime_config is not None
            and self._v2_runtime_config.bridge_admission_mode
            is BridgeAdmissionMode.FLEXIBLE_LEVEL2
        ):
            for edge in self._multi_plan.transitions:
                self._cache_nominal_flexible_bridge_template(
                    edge.handoff_manifest,
                    startup_phase="multi_setup_before_live",
                )
            self._event(
                "multi_nominal_bridge_templates_cached",
                transition_count=len(self._multi_plan.transitions),
                cached_handoff_ids=sorted(
                    self._flexible_bridge_nominal_template_cache
                ),
                before_live_authorization=True,
            )
        assert self._session_b is not None
        assert self._handoff_config is not None
        initial_policy = self._multi_plan.initial_policy
        loaded_initial = str(ctx.runtime.cfg.policy.pretrained_path)
        if not _same_checkpoint(loaded_initial, str(initial_policy.checkpoint)):
            raise ValueError(
                "rollout context checkpoint differs from the first multi-stage policy"
            )

        first_successor_id = self._multi_plan.stages[1].policy_id
        first_successor = self._multi_plan.policy(first_successor_id)
        backend_checkpoint = str(self._session_b.backend.checkpoint)
        if not _same_checkpoint(backend_checkpoint, str(first_successor.checkpoint)):
            raise ValueError(
                "resident first-successor session differs from the multi-stage plan"
            )

        registry = PolicyRegistry()
        resident_backend = TimedACTBackend(
            ResidentLeRobotACTBackend(
                ctx.policy.policy,
                ctx.policy.preprocessor,
                ctx.policy.postprocessor,
                device=str(ctx.runtime.cfg.device),
                checkpoint=initial_policy.checkpoint,
            )
        )
        resident_session = AsyncPolicySession(
            initial_policy.policy_id,
            resident_backend,
            action_hz=float(ctx.runtime.cfg.fps),
            inference_lock=get_act_gpu_arbiter(),
        )
        registry.register(
            policy_id=initial_policy.policy_id,
            checkpoint=str(initial_policy.checkpoint),
            session=resident_session,
            reuse_context_policy=True,
        )
        registry.register(
            policy_id=first_successor.policy_id,
            checkpoint=str(first_successor.checkpoint),
            session=self._session_b,
            reuse_context_policy=False,
        )

        context_contract = {
            key: tuple(value.shape)
            for key, value in ctx.policy.policy.config.input_features.items()
        }
        additional_sessions: list[AsyncPolicySession] = []
        for policy in self._multi_plan.policies:
            if policy.policy_id in {
                initial_policy.policy_id,
                first_successor.policy_id,
            }:
                continue
            backend = LeRobotACTBackend(
                policy.checkpoint,
                device=str(ctx.runtime.cfg.device),
            )
            contract = {
                key: tuple(value.shape)
                for key, value in backend.config.input_features.items()
            }
            if contract != context_contract:
                raise ValueError(
                    f"policy {policy.policy_id} observation contract differs"
                )
            session = AsyncPolicySession(
                policy.policy_id,
                TimedACTBackend(backend),
                action_hz=float(ctx.runtime.cfg.fps),
                inference_lock=get_act_gpu_arbiter(),
            )
            registry.register(
                policy_id=policy.policy_id,
                checkpoint=str(policy.checkpoint),
                session=session,
                reuse_context_policy=False,
            )
            additional_sessions.append(session)

        if additional_sessions:
            warm_observation = self._capture(ctx)[0].policy_input
            with preserve_current_thread_affinity():
                for session in additional_sessions:
                    session.warmup(
                        warm_observation,
                        inferences=self._handoff_config.warmup_inferences,
                    )

        self._policy_registry = registry
        self._multi_configured_edge_id = self._multi_plan.transitions[0].transition_id
        for index, stage in enumerate(self._multi_plan.stages[:-1]):
            if stage.exit_authority != "phase_supervisor":
                continue
            edge = self._multi_plan.transitions[index]
            if stage.phase_supervisor is None:
                raise RuntimeError("phase-supervised stage lacks validated config")
            supervisor = StagePhaseSupervisor(
                edge.handoff_manifest.source,
                stage.phase_supervisor,
            )
            self._multi_phase_supervisors[stage.stage_id] = supervisor
            self._multi_phase_latency_ms[stage.stage_id] = []
            self._event(
                "multi_phase_supervisor_loaded",
                stage_id=stage.stage_id,
                policy_id=stage.policy_id,
                transition_id=edge.transition_id,
                supervisor=supervisor.record(),
                control_thread_role="bounded_vectorized_update_only",
                bridge_generation_deferred_until_commit=(
                    self._v2_runtime_config.bridge_admission_mode
                    is BridgeAdmissionMode.STRICT_LEVEL2
                ),
                flexible_bridge_generation_starts_at_prearm=(
                    self._v2_runtime_config.bridge_admission_mode
                    is BridgeAdmissionMode.FLEXIBLE_LEVEL2
                ),
            )
        node = self._raw_robot._node
        multi_ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._multi_ready_pub = node.create_publisher(
            String,
            "/control/task_c/multi_ready",
            multi_ready_qos,
        )
        self._multi_services = [
            node.create_service(
                Trigger,
                self.multi_config.request_next_service,
                self._on_request_next_service,
            ),
            node.create_service(
                Trigger,
                self.multi_config.complete_service,
                self._on_complete_service,
            ),
            node.create_service(
                Trigger,
                self.multi_config.status_service,
                self._on_status_service,
            ),
        ]
        self._event(
            "multi_stage_v2_ready",
            composition_id=self._multi_plan.composition_id,
            plan_path=str(self._multi_plan.source_path),
            policies=[
                {
                    "policy_id": item.policy_id,
                    "checkpoint": str(item.checkpoint),
                    "reuse_context_policy": item.reuse_context_policy,
                }
                for item in self._multi_plan.policies
            ],
            stages=[asdict(item) for item in self._multi_plan.stages],
            transitions=[
                {
                    "transition_id": item.transition_id,
                    "source_stage": item.source_stage,
                    "successor_stage": item.successor_stage,
                    "handoff_id": item.handoff_manifest.handoff_id,
                    "handoff_manifest": str(item.handoff_manifest_path),
                }
                for item in self._multi_plan.transitions
            ],
            unique_policy_count=len(self._multi_plan.policies),
            policy_visit_count=len(self._multi_plan.stages),
            policy_loading="setup_only",
            cuda_inference="existing_process_wide_arbiter_serialized",
            runtime_semantic_authority="external_planner",
            runtime_command_profile_id=(
                self._multi_plan.runtime_command_profile_id
            ),
            endpoint_fallback_enabled=self.multi_config.enable_endpoint_fallback,
            endpoint_fallback_mode=(
                "edge_endpoint_hold_fresh_successor"
                if self.multi_config.enable_endpoint_fallback
                else "disabled_fail_closed_baseline"
            ),
        )
        self._multi_ready_pub.publish(
            String(
                data=json.dumps(
                    {
                        "schema": "task_c_multi_ready.v1",
                        "timestamp_s": time.monotonic(),
                        "event": "multi_stage_v2_ready",
                        "composition_id": self._multi_plan.composition_id,
                        "plan_path": str(self._multi_plan.source_path),
                        "runtime_command_profile_id": (
                            self._multi_plan.runtime_command_profile_id
                        ),
                    },
                    sort_keys=True,
                )
            )
        )

    def _on_multi_endpoint_fallback_event(
        self,
        event: EndpointFallbackEvent,
    ) -> None:
        edge = self._multi_active_edge
        self._event(
            f"multi_{event.event}",
            endpoint_fallback_state=event.state,
            transition_id=None if edge is None else edge.transition_id,
            source_stage=None if edge is None else edge.source_stage,
            successor_stage=None if edge is None else edge.successor_stage,
            **dict(event.details),
        )

    def _on_multi_stage_event(self, event: MultiStageEvent) -> None:
        self._event(
            f"multi_{event.event}",
            composition_id=self._multi_plan.composition_id,
            multi_state=event.state,
            stage_index=event.stage_index,
            stage_id=event.stage_id,
            policy_id=event.policy_id,
            visit_id=event.visit_id,
            transition_id=event.transition_id,
            **dict(event.details or {}),
        )

    def _on_request_next_service(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        stage = self._multi_coordinator.current_stage
        if stage.exit_authority != "external_planner":
            response.success = False
            response.message = (
                f"stage {stage.stage_id} exit is owned by "
                f"{stage.exit_authority}"
            )
            return response
        accepted, message = self._multi_coordinator.try_request_next()
        response.success = bool(accepted)
        response.message = str(message)
        return response

    def _on_complete_service(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        stage = self._multi_coordinator.current_stage
        if stage.exit_authority != "external_complete":
            response.success = False
            response.message = (
                f"stage {stage.stage_id} completion is owned by "
                f"{stage.exit_authority}"
            )
            return response
        accepted, message = self._multi_coordinator.try_complete()
        response.success = bool(accepted)
        response.message = str(message)
        return response

    def _on_status_service(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        response.success = True
        response.message = str(self._multi_coordinator.record())
        return response

    def _reset_current_phase_supervisor(self) -> None:
        stage = self._multi_coordinator.current_stage
        self._multi_latest_phase_status.pop(stage.stage_id, None)
        supervisor = self._multi_phase_supervisors.get(stage.stage_id)
        if supervisor is not None:
            supervisor.reset()
            self._event(
                "multi_phase_supervisor_reset",
                stage_id=stage.stage_id,
                policy_id=stage.policy_id,
                transition_id=(
                    None
                    if self._multi_plan.transition_after(
                        self._multi_coordinator.stage_index
                    )
                    is None
                    else self._multi_plan.transition_after(
                        self._multi_coordinator.stage_index
                    ).transition_id
                ),
            )

    def _update_automatic_phase_supervisor(
        self,
        capture: _LiveCapture,
        state: np.ndarray | None = None,
    ) -> bool:
        """Request the next stage after phase/support/gripper persistence.

        This method performs one bounded vectorized support query. It never
        generates a Bridge, accesses a file, requests CUDA inference, or waits
        for another thread.
        """

        coordinator_state = self._multi_coordinator.state
        if coordinator_state not in {
            MultiStageRuntimeState.RUN_STAGE,
            MultiStageRuntimeState.TRANSITION_REQUESTED,
        }:
            return False
        transition_pending = (
            coordinator_state
            is MultiStageRuntimeState.TRANSITION_REQUESTED
        )
        stage = self._multi_coordinator.current_stage
        if stage.exit_authority != "phase_supervisor":
            return False
        supervisor = self._multi_phase_supervisors.get(stage.stage_id)
        if supervisor is None:
            raise RuntimeError(
                f"stage {stage.stage_id} phase supervisor was not preloaded"
            )
        velocity = self._estimate_actual_boundary_velocity()
        status = supervisor.update(
            tcp_position_mm=capture.tcp_position_mm,
            tcp_velocity_mm_s=velocity,
            gripper_closed=capture.semantic_state.gripper_closed,
        )
        self._multi_latest_phase_status[stage.stage_id] = status
        collar_bounds_fn = getattr(
            supervisor, "execution_collar_bounds", None
        )
        collar_ready_fn = getattr(
            supervisor, "execution_collar_ready", None
        )
        rebase_ready_fn = getattr(
            supervisor, "adaptive_rebase_ready", None
        )
        execution_collar_bounds = (
            collar_bounds_fn() if callable(collar_bounds_fn) else None
        )
        execution_collar_ready = (
            bool(collar_ready_fn(status))
            if callable(collar_ready_fn)
            else bool(getattr(status, "prearmed", False))
        )
        adaptive_rebase_ready = (
            bool(rebase_ready_fn(status))
            if callable(rebase_ready_fn)
            else bool(getattr(status, "phase_ready", status.commit_ready))
        )
        flexible_worker = getattr(self, "_flexible_bridge_worker", None)
        flexible_work_pending = bool(
            (
                flexible_worker is not None
                and getattr(flexible_worker, "inflight_generation", None)
                is not None
            )
            or getattr(self, "_flexible_bridge_ready", None) is not None
        )
        flexible_prearm_poll_ready = bool(
            execution_collar_ready
            or adaptive_rebase_ready
            or flexible_work_pending
        )
        self._multi_phase_latency_ms[stage.stage_id].append(
            status.tracker_latency_ms
        )
        edge = self._multi_plan.transition_after(
            self._multi_coordinator.stage_index
        )
        if edge is None:
            raise RuntimeError("phase-supervised final stage has no transition")
        tracker = getattr(supervisor, "tracker", None)
        tracker_config = getattr(tracker, "config", None)
        tracker_bank = getattr(tracker, "bank", None)
        tracking_nominal_phase = float(
            getattr(
                tracker_config,
                "nominal_phase",
                edge.handoff_manifest.source.phase,
            )
        )
        tracking_segment = str(
            getattr(
                tracker_bank,
                "segment",
                getattr(
                    edge.handoff_manifest.source, "segment", "unknown"
                ),
            )
        )
        execution_tail = getattr(
            stage.phase_supervisor, "execution_tail", None
        )
        manifest_source_segment = str(
            getattr(
                edge.handoff_manifest.source,
                "segment",
                tracking_segment,
            )
        )
        self._event(
            "multi_phase_supervisor_status",
            stage_id=stage.stage_id,
            policy_id=stage.policy_id,
            transition_id=edge.transition_id,
            **status.record(),
            semantic_a_nominal_phase=(
                edge.handoff_manifest.source.phase
                if execution_tail is None
                else execution_tail.semantic_exit_phase
            ),
            semantic_a_exit_segment=(
                manifest_source_segment
                if execution_tail is None
                else execution_tail.semantic_exit_segment
            ),
            a_nominal_phase=tracking_nominal_phase,
            a_tracking_segment=tracking_segment,
            runtime_bridge_source_segment=manifest_source_segment,
            runtime_bridge_source_phase=(
                edge.handoff_manifest.source.phase
            ),
            zero_cost_execution_tail=(
                None
                if execution_tail is None
                else execution_tail.to_record()
            ),
            expected_gripper_state=(
                edge.handoff_manifest.source.semantic.gripper_state
            ),
            gripper_event=stage.phase_supervisor.gripper_event_mode.value,
            bridge_feasible=None,
            transition_requested=transition_pending,
            execution_collar_ready=execution_collar_ready,
            execution_collar_phase_low=(
                None
                if execution_collar_bounds is None
                else execution_collar_bounds[0]
            ),
            execution_collar_phase_high=(
                None
                if execution_collar_bounds is None
                else execution_collar_bounds[1]
            ),
            adaptive_rebase_ready=adaptive_rebase_ready,
            flexible_bridge_work_pending=flexible_work_pending,
            flexible_bridge_prearm_poll_ready=flexible_prearm_poll_ready,
        )
        runtime_config = getattr(self, "_v2_runtime_config", None)
        if (
            not transition_pending
            and flexible_prearm_poll_ready
            and not status.commit_ready
            and velocity is not None
            and state is not None
            and runtime_config is not None
            and runtime_config.bridge_admission_mode
            is BridgeAdmissionMode.FLEXIBLE_LEVEL2
        ):
            if self._multi_configured_edge_id != edge.transition_id:
                self._configure_edge_runtime(edge)
            self._try_prepare_flexible_bridge(
                capture,
                state,
                allow_commit=False,
            )
            self._event(
                "multi_flexible_bridge_prearm_polled",
                stage_id=stage.stage_id,
                policy_id=stage.policy_id,
                transition_id=edge.transition_id,
                source_policy_queue_invalidated=False,
                execution_collar_ready=execution_collar_ready,
                adaptive_rebase_ready=adaptive_rebase_ready,
                flexible_bridge_work_pending=flexible_work_pending,
                flexible_bridge_prearm_poll_ready=(
                    flexible_prearm_poll_ready
                ),
                execution_collar_phase_low=(
                    None
                    if execution_collar_bounds is None
                    else execution_collar_bounds[0]
                ),
                execution_collar_phase_high=(
                    None
                    if execution_collar_bounds is None
                    else execution_collar_bounds[1]
                ),
            )
        if status.deadline_exceeded:
            raise RuntimeError(
                f"stage {stage.stage_id} phase deadline exceeded before "
                "a feasible Bridge commit"
            )
        if transition_pending:
            return False
        if not status.commit_ready:
            return False
        accepted, message = self._multi_coordinator.try_request_next(
            expected_stage_id=stage.stage_id
        )
        if not accepted:
            raise RuntimeError(
                f"automatic transition request rejected: {message}"
            )
        self._event(
            "multi_phase_transition_requested",
            stage_id=stage.stage_id,
            policy_id=stage.policy_id,
            transition_id=edge.transition_id,
            phase_status=status.record(),
            source_policy_queue_invalidated=False,
            bridge_generation_pending=True,
        )
        return True

    def _assert_task_a_start(self, state: np.ndarray) -> None:
        super()._assert_task_a_start(state)
        if self._multi_started:
            raise RuntimeError("multi-stage episode cannot be started twice")
        assert self._policy_registry is not None
        self._policy_registry.begin_visit(
            self._multi_plan.initial_policy.policy_id,
            invalidate_queue=False,
        )
        self._multi_coordinator.start()
        self._reset_current_phase_supervisor()
        self._multi_started = True

    def _a_exit_commit_ready(
        self,
        ctx: Any,
        capture: _LiveCapture,
        state: np.ndarray,
        cut_status: Any,
    ) -> bool:
        """Route either external or automatic stage authority to existing V2."""

        del ctx, cut_status
        if self._multi_coordinator.state in {
            MultiStageRuntimeState.RUN_STAGE,
            MultiStageRuntimeState.TRANSITION_REQUESTED,
        }:
            self._update_automatic_phase_supervisor(capture, state)
        if (
            self._multi_coordinator.state
            is not MultiStageRuntimeState.TRANSITION_REQUESTED
        ):
            return False
        edge = self._multi_coordinator.requested_transition
        if edge is None or edge is not self._multi_plan.transitions[0]:
            raise RuntimeError("initial multi-stage transition order mismatch")
        return self._try_prepare_requested_bridge(capture, state)

    def _multi_adaptive_rebase_ready(self) -> bool:
        """Latch a requested cut while support/semantics remain valid.

        Before a transition request, the data-derived commit window retains
        authority. Once that window has produced a request, ACT-A may consume
        its bounded predicted suffix until the Bridge is ready; it is not
        forced back through a phase gate that has already been satisfied.
        The existing semantic/support/deadline checks remain fail-closed.
        """

        stage = self._multi_coordinator.current_stage
        if stage.exit_authority != "phase_supervisor":
            return True
        status = self._multi_latest_phase_status.get(stage.stage_id)
        if status is None:
            return False
        supervisor = self._multi_phase_supervisors.get(stage.stage_id)
        if supervisor is None:
            return False
        if (
            getattr(self._multi_coordinator, "state", None)
            is MultiStageRuntimeState.TRANSITION_REQUESTED
        ):
            return bool(
                not getattr(status, "deadline_exceeded", False)
                and getattr(status, "support_ready", True)
                and getattr(status, "semantic_ready", True)
            )
        ready_fn = getattr(supervisor, "adaptive_rebase_ready", None)
        if callable(ready_fn):
            return bool(ready_fn(status))
        return bool(getattr(status, "phase_ready", status.commit_ready))

    def _source_command_sequence(self) -> int:
        if (
            self._multi_coordinator.stage_index == 0
            and self.phase is LivePhase.ACT_A
        ):
            return super()._source_command_sequence()
        return int(self._b_steps_sent)

    def _source_action_queue_snapshot(
        self,
        capture: _LiveCapture,
        *,
        maximum_steps: int,
    ) -> SourceActionQueueSnapshot | None:
        """Use RTC for stage zero and the active resident session thereafter."""

        if (
            self._multi_coordinator.stage_index == 0
            and self.phase is LivePhase.ACT_A
        ):
            return super()._source_action_queue_snapshot(
                capture,
                maximum_steps=maximum_steps,
            )
        registry = self._policy_registry
        if registry is None:
            return None
        active_policy_id = registry.active_policy_id
        if active_policy_id is None:
            return None
        session = registry.session(active_policy_id)
        session_snapshot = session.active_queue_snapshot(
            maximum_steps=maximum_steps,
        )
        return source_snapshot_from_async_session(
            session_snapshot,
            command_sequence=self._source_command_sequence(),
            timestamp_s=capture.capture_completed_s,
            maximum_steps=maximum_steps,
        )

    def _try_prepare_requested_bridge(
        self,
        capture: _LiveCapture,
        state: np.ndarray,
    ) -> bool:
        now_s = time.monotonic()
        runtime_config = getattr(self, "_v2_runtime_config", None)
        flexible = (
            runtime_config is not None
            and runtime_config.bridge_admission_mode
            is BridgeAdmissionMode.FLEXIBLE_LEVEL2
        )
        # Flexible worker polling is constant-time and must run every control
        # tick. Only new adaptive-rebase submissions are rate-limited inside
        # the base strategy; the legacy synchronous generator keeps this gate.
        if not flexible and now_s < self._multi_next_bridge_retry_s:
            return False
        if not flexible:
            self._multi_next_bridge_retry_s = (
                now_s + self.multi_config.multi_bridge_retry_interval_s
            )
        try:
            if flexible:
                adaptive_rebase_ready = self._multi_adaptive_rebase_ready()
                self._prepared_bridge_commit = (
                    self._try_prepare_flexible_bridge(
                        capture,
                        state,
                        allow_commit=adaptive_rebase_ready,
                    )
                )
                if self._prepared_bridge_commit is None:
                    return False
            else:
                self._prepared_bridge_commit = self._prepare_bridge_commit(
                    capture,
                    state,
                )
        except BridgeGenerationError as exc:
            reason = f"{type(exc).__name__}: {exc}"
            self._prepared_bridge_commit = None
            if reason != self._multi_last_bridge_rejection:
                self._multi_last_bridge_rejection = reason
                self._event(
                    "multi_requested_bridge_rejected",
                    reason=reason,
                    command_queue_invalidated=False,
                    retry_interval_s=(
                        self.multi_config.multi_bridge_retry_interval_s
                    ),
                )
            return False
        except RuntimeError as exc:
            if str(exc) != "V2 A-exit velocity is unavailable":
                raise
            self._prepared_bridge_commit = None
            return False
        self._multi_last_bridge_rejection = None
        self._event(
            "multi_requested_bridge_ready",
            bridge=self._prepared_bridge_commit.bridge.record(),
            command_queue_invalidated=False,
        )
        return True

    def _begin_bridge(
        self,
        ctx: Any,
        capture: _LiveCapture,
        state: np.ndarray,
    ) -> None:
        edge = self._multi_coordinator.begin_requested_transition()
        if edge is not self._multi_plan.transitions[0]:
            raise RuntimeError("initial multi-stage edge changed before commit")
        self._multi_active_edge = edge
        super()._begin_bridge(ctx, capture, state)
        assert self._policy_registry is not None
        self._policy_registry.deactivate_active(invalidate_queue=True)
        self._event(
            "multi_source_policy_invalidated",
            transition_id=edge.transition_id,
            source_stage=edge.source_stage,
            successor_stage=edge.successor_stage,
        )

    def _configure_edge_runtime(self, edge: MultiTransitionSpec) -> AsyncPolicySession:
        assert self._policy_registry is not None
        assert self._v2_runtime_config is not None
        manifest = edge.handoff_manifest
        if (
            manifest.bridge_admission_mode
            is not self._v2_runtime_config.bridge_admission_mode
        ):
            raise RuntimeError(
                "multi-stage edge Bridge admission mode differs from runtime"
            )
        if manifest.semantic_authority is not SemanticAuthority.EXTERNAL_PLANNER:
            raise RuntimeError("multi-stage edge lost external planner authority")
        target_policy_id = self._multi_plan.policy(
            self._multi_plan.stages[self._multi_coordinator.stage_index + 1].policy_id
        ).policy_id
        target_session = self._policy_registry.session(target_policy_id)
        successor = AsyncSuccessorController(
            target_session,
            max_result_age_s=self.multi_config.max_b_result_age_sec,
        )
        evaluator = HandoffCompatibilityEvaluator(
            self._v2_runtime_config.compatibility,
            prefix_steps=self.multi_config.b_prefix_steps,
            action_hz=self._downstream_contract.control_hz,
            semantic_authority=SemanticAuthority.EXTERNAL_PLANNER,
            splice_selector=HandoffSpliceSelector(
                max_splice_index=(
                    self._v2_runtime_config.adaptive_b_max_splice_index
                ),
                max_candidates=(
                    self._v2_runtime_config.adaptive_b_max_candidates
                ),
            ),
        )
        self._v2_manifest = manifest
        self._semantic_authority = SemanticAuthority.EXTERNAL_PLANNER
        self._v2_coordinator = TaskCHandoffV2Coordinator(
            manifest=manifest,
            config=self._v2_runtime_config,
            successor=successor,
            evaluator=evaluator,
            event_callback=self._on_v2_coordinator_event,
        )
        timestamp_s = time.monotonic()
        self._v2_coordinator.mark_policies_loaded(timestamp_s=timestamp_s)
        self._v2_coordinator.mark_run_a(timestamp_s=timestamp_s)
        self._source_phase_status = None
        self._prepared_bridge_commit = None
        self._reset_flexible_bridge_runtime()
        if self._multi_endpoint_fallback is not None:
            raise RuntimeError("previous endpoint fallback was not finalized")
        self._event(
            "multi_edge_runtime_bound",
            transition_id=edge.transition_id,
            source_stage=edge.source_stage,
            successor_stage=edge.successor_stage,
            target_policy_id=target_policy_id,
            target_session_generation=target_session.generation,
            successor_phase=manifest.successor.phase,
            successor_phase_role="bridge_reference_and_metadata_only",
            successor_reference_phase_window=(
                manifest.successor_reference_phase_window
            ),
            successor_reference_selection_method=(
                manifest.successor_reference_selection_method
            ),
            successor_interior_path_margin_mm=(
                manifest.successor_interior_path_margin_mm
            ),
            semantic_local_total_length_mm=(
                manifest.semantic_local_total_length_mm
            ),
            successor_runtime_phase_gate=False,
        )
        self._multi_configured_edge_id = edge.transition_id
        return target_session

    def _reset_source_policy_runtime_after_invalidation(self) -> None:
        self._b_refresh_generation = None
        self._b_refresh_requested_s = None
        self._b_refresh_queue_consumed_at_request = 0
        self._b_refresh_hold_cycles = 0

    def _start_later_requested_edge(
        self,
        ctx: Any,
        capture: _LiveCapture,
        state: np.ndarray,
    ) -> bool:
        edge = self._multi_coordinator.requested_transition
        if edge is None:
            raise RuntimeError("transition request lacks edge metadata")
        runtime_config = getattr(self, "_v2_runtime_config", None)
        flexible = (
            runtime_config is not None
            and runtime_config.bridge_admission_mode
            is BridgeAdmissionMode.FLEXIBLE_LEVEL2
        )
        if not flexible and time.monotonic() < self._multi_next_bridge_retry_s:
            return False
        assert self._policy_registry is not None
        if self._multi_configured_edge_id != edge.transition_id:
            target_session = self._configure_edge_runtime(edge)
        else:
            successor_stage = self._multi_plan.stages[
                self._multi_coordinator.stage_index + 1
            ]
            target_session = self._policy_registry.session(
                successor_stage.policy_id
            )
        if not self._try_prepare_requested_bridge(capture, state):
            return False

        committed = self._multi_coordinator.begin_requested_transition()
        if committed.transition_id != edge.transition_id:
            raise RuntimeError("multi-stage transition changed during Bridge commit")
        self._multi_active_edge = committed
        assert self._policy_registry is not None
        self._policy_registry.deactivate_active(invalidate_queue=True)
        self._reset_source_policy_runtime_after_invalidation()
        self._session_b = target_session
        TaskCLiveV2Strategy._begin_bridge(self, ctx, capture, state)
        self._event(
            "multi_source_policy_invalidated",
            transition_id=edge.transition_id,
            source_stage=edge.source_stage,
            successor_stage=edge.successor_stage,
        )
        return True

    def _reset_successor_stage_runtime(self, *, started_s: float) -> None:
        self._b_steps_sent = 0
        self._b_started_s = float(started_s)
        self._b_refresh_generation = None
        self._b_refresh_requested_s = None
        self._b_refresh_queue_consumed_at_request = 0
        self._b_refresh_count = 0
        self._b_refresh_hold_cycles = 0
        self._b_direct_handoff_validated_generation = None
        self._b_release_candidate_steps = 0
        self._b_release_authorized = False
        self._b_release_authorized_s = None
        self._b_release_command_sent_s = None
        self._b_release_confirmed = False
        self._b_release_open_observation_frames = 0
        self._b_release_pose_mm = None
        self._b_completion_stable_frames = 0
        self._transition_started_s = None

    def _start_endpoint_v1_fallback(
        self,
        ctx: Any,
        capture: _LiveCapture,
        state: np.ndarray,
        command: HandoffCommand,
    ) -> None:
        """Run the edge-scoped Multi-V2 endpoint fallback, not legacy V1."""

        if not self.multi_config.enable_endpoint_fallback:
            raise RuntimeError("multi endpoint fallback is disabled")
        assert self._v2_coordinator is not None
        assert self._v2_coordinator.bridge is not None
        if self._multi_endpoint_fallback is None:
            endpoint_action = self._v2_coordinator.bridge.actions[-1]
            settle_timeout_s = min(
                self.live_config.b_endpoint_settle_timeout_s,
                self.live_config.transition_timeout_s,
            )
            self._multi_endpoint_fallback = MultiEndpointFallbackCoordinator(
                endpoint_action=endpoint_action,
                manifest=self._v2_coordinator.manifest,
                config=EndpointFallbackConfig(
                    control_hz=self._downstream_contract.control_hz,
                    settle_position_tolerance_mm=(
                        self.live_config.b_endpoint_settle_position_tolerance_mm
                    ),
                    settle_velocity_tolerance_mm_s=(
                        self.live_config.b_endpoint_settle_velocity_tolerance_mm_s
                    ),
                    settle_min_hold_s=(
                        self.live_config.b_endpoint_settle_min_hold_s
                    ),
                    settle_timeout_s=settle_timeout_s,
                    total_timeout_s=self.live_config.transition_timeout_s,
                    request_retry_interval_steps=(
                        self.multi_config.b_request_retry_interval_steps
                    ),
                    crossfade_steps=self.multi_config.crossfade_steps,
                ),
                successor=self._v2_coordinator.successor,
                evaluator=self._v2_coordinator.evaluator,
                started_s=capture.capture_completed_s,
                moving_failure_reason=(
                    command.failure_reason
                    or self._v2_coordinator.fallback_reason
                    or "moving_handoff_window_failed"
                ),
                event_callback=self._on_multi_endpoint_fallback_event,
            )
            self._v2_endpoint_fallback_used = True
            # Match the reviewed V1 behavior: the bounded endpoint fallback has
            # its own transition watchdog after the moving Bridge is exhausted.
            self._transition_started_s = capture.capture_completed_s
            self._event(
                "multi_endpoint_fallback_started",
                transition_id=(
                    None
                    if self._multi_active_edge is None
                    else self._multi_active_edge.transition_id
                ),
                endpoint_action=np.asarray(endpoint_action).tolist(),
                moving_failure_reason=command.failure_reason,
                control_path="existing_ACK_MUX_safety_ServoL",
                inference_wait_on_control_thread=False,
            )
        self._step_multi_endpoint_fallback(ctx, capture, state)

    def _step_multi_endpoint_fallback(
        self,
        ctx: Any,
        capture: _LiveCapture,
        state: np.ndarray,
    ) -> None:
        runtime = self._multi_endpoint_fallback
        if runtime is None:
            raise RuntimeError("multi endpoint fallback runtime is missing")
        acknowledged = self._last_acknowledged_bridge_pose
        if acknowledged is None:
            raise RuntimeError("multi endpoint fallback lacks acknowledged pose")
        measured_velocity = self._estimate_actual_boundary_velocity()
        if measured_velocity is None:
            raise RuntimeError("multi endpoint fallback actual velocity is unavailable")
        command = runtime.tick(
            timestamp_s=capture.capture_completed_s,
            actual_pose_mm_deg=state[6:12],
            acknowledged_pose_mm_deg=acknowledged,
            actual_velocity_mm_s=measured_velocity,
            command_history_mm_deg=self._v2_command_history_array(),
            actual_semantic=self._actual_boundary_semantic(state),
            policy_input=capture.policy_input,
            observation_timestamp_s=capture.capture_completed_s,
        )
        if command.failure_reason is not None and command.action is None:
            raise RuntimeError(
                f"Multi V2 endpoint fallback failed: {command.failure_reason}"
            )
        if command.action is None:
            return
        if command.transition_to_b_after_commit:
            generation = command.successor_generation
            if generation is None:
                raise RuntimeError("endpoint takeover lacks successor generation")
            assert self._v2_coordinator is not None
            self._v2_coordinator.mark_endpoint_takeover(
                timestamp_s=capture.capture_completed_s,
                generation=generation,
                crossfade_steps=self.multi_config.crossfade_steps,
            )
        meta_state = (
            HandoffV2State.SOFT_HANDOFF
            if command.state is EndpointFallbackState.SOFT_HANDOFF
            else HandoffV2State.ENDPOINT_FALLBACK
        )
        self._dispatch_v2_command(
            ctx,
            HandoffCommand(
                action=command.action,
                source=command.source,
                state=meta_state,
                bridge_index=self._v2_coordinator.bridge.steps - 1,
                bridge_progress=1.0,
                crossfade_weight=command.crossfade_weight,
                successor_generation=command.successor_generation,
                transition_to_b_after_commit=(
                    command.transition_to_b_after_commit
                ),
            ),
            actual_pose=state[6:12],
        )

    def _commit_live_command(
        self,
        ctx: Any,
        pending: _PendingLiveCommand,
        *,
        actual_pose: np.ndarray,
    ) -> None:
        takeover = bool(pending.transition_to_act_b and self._v2_runtime_active)
        edge = self._multi_active_edge if takeover else None
        endpoint_fallback_record = (
            self._multi_endpoint_fallback.record()
            if takeover and self._multi_endpoint_fallback is not None
            else None
        )
        super()._commit_live_command(ctx, pending, actual_pose=actual_pose)
        if not takeover:
            return
        if edge is None or self._session_b is None:
            raise RuntimeError("multi-stage takeover lacks edge/session metadata")
        generation = self._session_b.active_generation
        if generation is None:
            raise RuntimeError("multi-stage takeover lacks active policy generation")
        assert self._policy_registry is not None
        successor_stage = self._multi_plan.stages[
            self._multi_coordinator.stage_index + 1
        ]
        registry_visit = self._policy_registry.begin_visit(
            successor_stage.policy_id,
            invalidate_queue=False,
        )
        stage = self._multi_coordinator.complete_takeover(
            transition_id=edge.transition_id,
            policy_generation=generation,
        )
        started_s = time.monotonic()
        self._reset_successor_stage_runtime(started_s=started_s)
        self._reset_current_phase_supervisor()
        self._multi_active_edge = None
        self._multi_next_bridge_retry_s = 0.0
        self._event(
            "multi_policy_takeover_committed",
            transition_id=edge.transition_id,
            stage_id=stage.stage_id,
            policy_id=stage.policy_id,
            registry_visit=registry_visit,
            generation=generation,
            actual_pose_mm_deg=np.asarray(actual_pose, dtype=np.float64).tolist(),
            endpoint_fallback_used=endpoint_fallback_record is not None,
        )
        if endpoint_fallback_record is not None:
            self._multi_endpoint_fallback_records.append(
                {
                    "transition_id": edge.transition_id,
                    **endpoint_fallback_record,
                }
            )
            self._multi_endpoint_fallback = None

    def _maybe_complete_terminal_inference(self) -> bool:
        if self._multi_coordinator.state is not MultiStageRuntimeState.RUN_STAGE:
            return False
        stage = self._multi_coordinator.current_stage
        if stage.exit_authority != "policy_inference_end":
            return False
        if stage.inference_end_steps is None:
            raise RuntimeError("policy_inference_end lacks inference_end_steps")
        if self._b_steps_sent < stage.inference_end_steps:
            return False
        accepted, message = self._multi_coordinator.try_complete(
            expected_stage_id=stage.stage_id
        )
        if not accepted:
            raise RuntimeError(
                f"terminal inference completion rejected: {message}"
            )
        self._event(
            "multi_terminal_inference_end",
            stage_id=stage.stage_id,
            policy_id=stage.policy_id,
            configured_inference_end_steps=stage.inference_end_steps,
            b_steps_sent=self._b_steps_sent,
            policy_done_token_available=False,
            interpretation="configured_finite_inference_horizon_not_semantic_success",
        )
        return True

    def _complete_multi_episode_from_control_thread(self) -> None:
        assert self._policy_registry is not None
        stage = self._multi_coordinator.current_stage
        generation = self._policy_registry.deactivate_active(
            invalidate_queue=True
        )
        if (
            self._v2_coordinator is not None
            and self._v2_coordinator.state is HandoffV2State.RUN_B
        ):
            self._v2_coordinator.mark_complete(timestamp_s=time.monotonic())
        self.phase = LivePhase.COMPLETE
        self._pending_command = None
        self._event(
            "multi_episode_control_complete",
            invalidated_generation=generation,
            final_stage=stage.stage_id,
            completion_authority=stage.exit_authority,
        )

    def _step_b(
        self,
        ctx: Any,
        capture: _LiveCapture,
        state: np.ndarray,
    ) -> None:
        if self._multi_coordinator.state in {
            MultiStageRuntimeState.RUN_STAGE,
            MultiStageRuntimeState.TRANSITION_REQUESTED,
        }:
            self._update_automatic_phase_supervisor(capture, state)
            if (
                self._multi_coordinator.state
                is MultiStageRuntimeState.RUN_STAGE
            ):
                self._maybe_complete_terminal_inference()
        multi_state = self._multi_coordinator.state
        if multi_state is MultiStageRuntimeState.COMPLETE:
            self._complete_multi_episode_from_control_thread()
            return
        if multi_state is MultiStageRuntimeState.FAILED_HOLD:
            raise RuntimeError("multi-stage coordinator requested fail-closed hold")
        if multi_state is MultiStageRuntimeState.TRANSITION_REQUESTED:
            if self._start_later_requested_edge(ctx, capture, state):
                return
        super()._step_b(ctx, capture, state)

    def _fail(self, reason: str) -> None:
        if self._multi_endpoint_fallback is not None:
            edge = self._multi_active_edge
            self._multi_endpoint_fallback_records.append(
                {
                    "transition_id": (
                        None if edge is None else edge.transition_id
                    ),
                    **self._multi_endpoint_fallback.record(),
                }
            )
            self._multi_endpoint_fallback = None
        self._multi_coordinator.fail_closed(reason)
        if self._policy_registry is not None:
            for policy in self._multi_plan.policies:
                try:
                    self._policy_registry.session(
                        policy.policy_id
                    ).deactivate_and_clear()
                except Exception:
                    pass
        super()._fail(reason)

    def _episode_summary(self, recording_stats: Any) -> dict[str, Any]:
        summary = super()._episode_summary(recording_stats)
        summary["multi_stage"] = {
            "plan_schema": self._multi_plan.schema_version,
            "composition_id": self._multi_plan.composition_id,
            "plan_path": str(self._multi_plan.source_path),
            "coordinator": self._multi_coordinator.record(),
            "policy_registry": (
                None
                if self._policy_registry is None
                else dict(self._policy_registry.record())
            ),
            "events": [asdict(event) for event in self._multi_coordinator.events],
            "stage_exit_authority": {
                stage.stage_id: stage.exit_authority
                for stage in self._multi_plan.stages
            },
            "phase_supervisors": {
                stage_id: {
                    **supervisor.record(),
                    "latency_ms": (
                        {}
                        if not self._multi_phase_latency_ms.get(stage_id)
                        else {
                            "samples": len(self._multi_phase_latency_ms[stage_id]),
                            "p50": float(
                                np.percentile(
                                    self._multi_phase_latency_ms[stage_id], 50
                                )
                            ),
                            "p95": float(
                                np.percentile(
                                    self._multi_phase_latency_ms[stage_id], 95
                                )
                            ),
                            "p99": float(
                                np.percentile(
                                    self._multi_phase_latency_ms[stage_id], 99
                                )
                            ),
                            "max": float(
                                np.max(self._multi_phase_latency_ms[stage_id])
                            ),
                        }
                    ),
                }
                for stage_id, supervisor in self._multi_phase_supervisors.items()
            },
            "semantic_transition_authority": (
                "per_stage_external_or_phase_supervisor"
            ),
            "dynamic_takeover_authority": "fresh_successor_prefix_admission",
            "endpoint_fallback": {
                "enabled": self.multi_config.enable_endpoint_fallback,
                "mode": "edge_endpoint_hold_fresh_successor",
                "records": list(self._multi_endpoint_fallback_records),
            },
        }
        return summary

    def teardown(self, ctx: Any) -> None:
        errors: list[BaseException] = []
        try:
            super().teardown(ctx)
        except BaseException as exc:
            errors.append(exc)
        if self._policy_registry is not None:
            try:
                self._policy_registry.close()
            except BaseException as exc:
                errors.append(exc)
        if errors:
            detail = "; ".join(
                f"{type(error).__name__}: {error}" for error in errors
            )
            raise RuntimeError(
                f"Task-C multi-stage V2 teardown failed: {detail}"
            ) from errors[0]


_MULTI_V2_FACTORY_INSTALLED = False


def install_task_c_multi_live_v2_strategy() -> bool:
    """Install an additive factory wrapper without modifying V0/V1/V2."""

    global _MULTI_V2_FACTORY_INSTALLED
    if _MULTI_V2_FACTORY_INSTALLED:
        return False
    import lerobot.rollout as rollout_package
    import lerobot.rollout.strategies as strategies_package
    from lerobot.rollout.strategies import factory

    original = factory.create_strategy

    def create_strategy(config: RolloutStrategyConfig) -> RolloutStrategy:
        if config.type == "task_c_multi_live_v2":
            if not isinstance(config, TaskCMultiLiveV2StrategyConfig):
                raise TypeError("task_c_multi_live_v2 config registration mismatch")
            return TaskCMultiLiveV2Strategy(config)
        return original(config)

    factory.create_strategy = create_strategy
    strategies_package.create_strategy = create_strategy
    rollout_package.create_strategy = create_strategy
    _MULTI_V2_FACTORY_INSTALLED = True
    return True
