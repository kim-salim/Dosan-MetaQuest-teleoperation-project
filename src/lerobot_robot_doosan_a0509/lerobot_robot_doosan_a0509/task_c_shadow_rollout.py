"""Read-only live-observation shadow strategy for Task-C latency trials.

This module never dispatches a policy action. During the virtual bridge it
feeds perfect-tracking Cartesian samples to the ROS-independent coordinator,
while both ACT policies continue to see immutable snapshots captured from the
real rollout observation pipeline. The split is explicit in every JSONL trial.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
from lerobot.rollout.configs import RolloutStrategyConfig
from lerobot.rollout.strategies.core import RolloutStrategy
from lerobot.utils.feature_utils import build_dataset_frame
from lerobot.utils.robot_utils import precise_sleep

from offline_tools.task_c_bridge_v0.lerobot_act_backend import (
    LeRobotACTBackend,
    ResidentLeRobotACTBackend,
)
from offline_tools.task_c_bridge_v0.runtime_bridge import (
    RuntimeBridgePlanner,
    runtime_entries_from_manifest,
)
from offline_tools.task_c_bridge_v0.runtime_orchestrator import (
    RuntimeHandoffConfig,
    RuntimeObservation,
    RuntimePhase,
    TaskCRealtimeCoordinator,
)
from offline_tools.task_c_bridge_v0.runtime_policy import (
    AsyncPolicySession,
    PolicyChunk,
)
from offline_tools.task_c_bridge_v0.shadow_latency import (
    ShadowJsonlRecorder,
    policy_chunk_timing_record,
)
from offline_tools.task_c_bridge_v0.trajectory_states import SemanticState


logger = logging.getLogger(__name__)


@RolloutStrategyConfig.register_subclass("task_c_shadow")
@dataclass
class TaskCShadowStrategyConfig(RolloutStrategyConfig):
    """CLI configuration for the command-free Task-C shadow runner."""

    runtime_manifest: str = ""
    checkpoint_b: str = ""
    jsonl_path: str = ""
    summary_path: str = ""
    warmup_inferences: int = 2
    stale_after_s: float = 1.5
    trial_interval_s: float = 0.5
    trial_timeout_s: float = 15.0
    closed_stable_frames: int = 3
    max_trials: int = 0

    def __post_init__(self) -> None:
        if self.warmup_inferences < 0:
            raise ValueError("warmup_inferences must be non-negative")
        if self.stale_after_s <= 0.0 or self.trial_timeout_s <= 0.0:
            raise ValueError("stale_after_s and trial_timeout_s must be positive")
        if self.trial_interval_s < 0.0:
            raise ValueError("trial_interval_s must be non-negative")
        if self.closed_stable_frames < 1 or self.max_trials < 0:
            raise ValueError("invalid shadow trial count configuration")


@dataclass(frozen=True)
class _LiveCapture:
    capture_started_s: float
    capture_completed_s: float
    processor_completed_s: float
    policy_input_ready_s: float
    policy_input: dict[str, Any]
    tcp_position_mm: np.ndarray
    semantic_state: SemanticState
    source_ages_s: dict[str, float | None]

    @property
    def capture_duration_ms(self) -> float:
        return (self.capture_completed_s - self.capture_started_s) * 1000.0

    @property
    def processor_duration_ms(self) -> float:
        return (self.processor_completed_s - self.capture_completed_s) * 1000.0

    @property
    def policy_input_build_ms(self) -> float:
        return (self.policy_input_ready_s - self.processor_completed_s) * 1000.0


@dataclass
class _ActiveTrial:
    trial_id: int
    coordinator: TaskCRealtimeCoordinator
    started_s: float
    a_request_capture: _LiveCapture
    a_chunk: PolicyChunk | None = None
    cut_capture: _LiveCapture | None = None
    b_request_capture: _LiveCapture | None = None
    b_deadline_s: float | None = None
    b_deadline_budget_s: float | None = None
    initial_plan_record: dict[str, Any] | None = None


def _resolved(path: str | Path) -> Path:
    return Path(path).expanduser().resolve()


def _same_checkpoint(left: str | Path, right: str | Path) -> bool:
    return _resolved(left) == _resolved(right)


def _policy_input_from_processed(
    processed: dict[str, Any],
    *,
    hw_features: dict[str, Any],
    expected_features: dict[str, Any],
) -> dict[str, Any]:
    """Mirror RTC frame conversion but keep tensors unbatched for our backend."""

    import torch

    frame = build_dataset_frame(hw_features, processed, prefix="observation")
    missing = [name for name in expected_features if name not in frame]
    if missing:
        raise KeyError(f"live observation is missing policy features: {missing}")
    result: dict[str, Any] = {}
    for name, feature in expected_features.items():
        value = frame[name]
        if isinstance(value, torch.Tensor):
            tensor = value.detach().clone()
        else:
            array = np.array(value, copy=True)
            tensor = torch.from_numpy(array)
        expected_shape = tuple(feature.shape)
        if "image" in name:
            if tensor.dtype == torch.uint8:
                tensor = tensor.to(dtype=torch.float32) / 255.0
            else:
                tensor = tensor.to(dtype=torch.float32)
            if tuple(tensor.shape) != expected_shape:
                if (
                    tensor.ndim == 3
                    and tuple(tensor.shape[:2]) == expected_shape[1:]
                    and int(tensor.shape[2]) == expected_shape[0]
                ):
                    tensor = tensor.permute(2, 0, 1).contiguous()
        else:
            tensor = tensor.to(dtype=torch.float32)
        if tuple(tensor.shape) != expected_shape:
            raise ValueError(
                f"{name} live shape {tuple(tensor.shape)} != {expected_shape}"
            )
        if not torch.isfinite(tensor).all():
            raise ValueError(f"{name} live snapshot contains non-finite values")
        result[name] = tensor
    return result


def _state_and_semantics(policy_input: dict[str, Any]) -> tuple[np.ndarray, SemanticState]:
    state_value = policy_input["observation.state"]
    if hasattr(state_value, "detach"):
        state = state_value.detach().cpu().numpy()
    else:
        state = np.asarray(state_value)
    state = np.asarray(state, dtype=np.float64)
    if state.shape != (13,) or not np.all(np.isfinite(state)):
        raise ValueError("live ACT state must be finite shape (13,)")
    gripper_closed = bool(round(float(state[12])))
    semantic = SemanticState(
        gripper_closed=gripper_closed,
        holding=True if gripper_closed else False,
        contact_mode="free_transport_assumed" if gripper_closed else "NOT_OBSERVED",
        completed_subgoals=("grasp_complete",) if gripper_closed else (),
        object_state="A_OBJECT_HELD_ASSUMED" if gripper_closed else "NOT_OBSERVED",
    )
    return state[6:9].copy(), semantic


def _event_details(coordinator: TaskCRealtimeCoordinator, name: str) -> dict[str, Any] | None:
    for event in reversed(coordinator.events):
        if event.event == name:
            return dict(event.details)
    return None


def _candidate_record(candidate: Any) -> dict[str, Any]:
    bridge = candidate.bridge
    metrics = candidate.metrics
    return {
        "entry_id": candidate.entry.entry_id,
        "entry_dataset": candidate.entry.dataset,
        "entry_episode": candidate.entry.episode,
        "entry_frame": candidate.entry.frame,
        "duration_s": bridge.duration_s,
        "P0_mm": bridge.p0.tolist(),
        "P1_mm": bridge.p1.tolist(),
        "P2_mm": bridge.p2.tolist(),
        "P3_mm": bridge.p3.tolist(),
        "A_retained_length_mm": candidate.a_retained_length_mm,
        "committed_bridge_prefix_length_mm": (
            candidate.committed_bridge_prefix_length_mm
        ),
        "bridge_length_mm": metrics.length_mm,
        "B_retained_length_mm": candidate.entry.b_retained_length_mm,
        "total_C_estimate_mm": candidate.total_c_estimate_mm,
        "max_velocity_mm_s": metrics.max_velocity_mm_s,
        "max_acceleration_mm_s2": metrics.max_acceleration_mm_s2,
        "max_curvature_per_mm": metrics.max_curvature_per_mm,
        "max_jerk_mm_s3": metrics.max_jerk_mm_s3,
        "integrated_squared_jerk": metrics.integrated_squared_jerk,
        "terminal_velocity_mm_s": candidate.terminal_velocity_mm_s.tolist(),
        "terminal_velocity_source": candidate.terminal_velocity_source,
        "semantic_unchecked": list(candidate.semantic_unchecked),
        "feasible": candidate.feasible,
        "failure_reasons": list(candidate.failure_reasons),
    }


class TaskCShadowStrategy(RolloutStrategy):
    """Drive the existing coordinator without ever calling ``send_action``."""

    def __init__(self, config: TaskCShadowStrategyConfig) -> None:
        super().__init__(config)
        self.shadow_config = config
        self._recorder: ShadowJsonlRecorder | None = None
        self._manifest: dict[str, Any] | None = None
        self._entries = ()
        self._initial_planner: RuntimeBridgePlanner | None = None
        self._tail_planner: RuntimeBridgePlanner | None = None
        self._handoff_config: RuntimeHandoffConfig | None = None
        self._session_a: AsyncPolicySession | None = None
        self._session_b: AsyncPolicySession | None = None
        self._active: _ActiveTrial | None = None
        self._trial_count = 0
        self._capture_successes = 0
        self._capture_failures = 0
        self._semantic_wait_frames = 0
        self._closed_stable_count = 0
        self._last_trial_finished_s = float("-inf")
        self._last_heartbeat_s = float("-inf")
        self._last_actual_position_mm: np.ndarray | None = None
        self._a_retained_length_mm = 0.0
        self._command_attempts = 0
        self._command_count_baseline: dict[str, int] = {}
        self._raw_robot: Any = None
        self._send_action_had_instance_value = False
        self._original_instance_send_action: Any = None
        self._setup_complete = False

    def _validate_safety_mode(self, ctx: Any) -> None:
        cfg = ctx.runtime.cfg
        robot_cfg = cfg.robot
        if getattr(robot_cfg, "mode", None) != "policy_shadow":
            raise ValueError(
                "Task-C shadow strategy requires --robot.mode=policy_shadow"
            )
        if cfg.return_to_initial_position:
            raise ValueError(
                "Task-C shadow requires --return_to_initial_position=false"
            )
        if cfg.display_data:
            logger.warning("display_data is enabled; no action is passed to visualization")

    def _install_command_guard(self, robot: Any) -> None:
        self._send_action_had_instance_value = "send_action" in robot.__dict__
        self._original_instance_send_action = robot.__dict__.get("send_action")

        def forbidden_send_action(*_args: Any, **_kwargs: Any) -> Any:
            self._command_attempts += 1
            raise RuntimeError("Task-C policy_shadow forbids every send_action call")

        robot.send_action = forbidden_send_action

    def _remove_command_guard(self) -> None:
        if self._raw_robot is None:
            return
        if self._send_action_had_instance_value:
            self._raw_robot.send_action = self._original_instance_send_action
        elif "send_action" in self._raw_robot.__dict__:
            del self._raw_robot.__dict__["send_action"]

    def _capture(self, ctx: Any) -> _LiveCapture:
        started = time.monotonic()
        raw = ctx.hardware.robot_wrapper.get_observation()
        captured = time.monotonic()
        processed = ctx.processors.robot_observation_processor(raw)
        processor_completed = time.monotonic()
        assert self._session_a is not None
        expected = self._session_a.backend.config.input_features
        policy_input = _policy_input_from_processed(
            processed,
            hw_features=ctx.data.hw_features,
            expected_features=expected,
        )
        ready = time.monotonic()
        tcp_position, semantic = _state_and_semantics(policy_input)
        source_ages: dict[str, float | None] = {}
        cache = getattr(self._raw_robot, "cache", None)
        for key in ("joint_positions", "actual_tcp_position", "gripper_commanded_state"):
            sample = None if cache is None else cache.sample(key)
            source_ages[key] = None if sample is None else sample.age(captured)
        return _LiveCapture(
            capture_started_s=started,
            capture_completed_s=captured,
            processor_completed_s=processor_completed,
            policy_input_ready_s=ready,
            policy_input=policy_input,
            tcp_position_mm=tcp_position,
            semantic_state=semantic,
            source_ages_s=source_ages,
        )

    def setup(self, ctx: Any) -> None:
        self._validate_safety_mode(ctx)
        config = self.shadow_config
        if not config.runtime_manifest or not config.jsonl_path:
            raise ValueError("runtime_manifest and jsonl_path are required")
        summary_path = config.summary_path or str(
            _resolved(config.jsonl_path).with_suffix(".summary.json")
        )
        self._recorder = ShadowJsonlRecorder(config.jsonl_path, summary_path)
        self._raw_robot = ctx.hardware.robot_wrapper.inner
        self._install_command_guard(self._raw_robot)

        # Establish the dynamic read-only proof before any heavyweight model
        # or manifest setup can fail. Teardown runs even when setup raises, so
        # a late baseline would turn all-zero counts into a false violation.
        self._command_count_baseline = {
            "live_publish_count": int(getattr(self._raw_robot, "live_publish_count", 0)),
            "hold_publish_count": int(getattr(self._raw_robot, "hold_publish_count", 0)),
            "debug_publish_count": int(getattr(self._raw_robot, "debug_publish_count", 0)),
        }
        publishers_absent = all(
            getattr(self._raw_robot, name, None) is None
            for name in ("_target_pub", "_gripper_pub", "_debug_pub", "_policy_ready_pub")
        )
        if not publishers_absent:
            raise RuntimeError("policy_shadow unexpectedly created a command publisher")

        manifest_path = _resolved(config.runtime_manifest)
        self._manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        safety = dict(self._manifest.get("safety", {}))
        if safety.get("robot_executable") is not False:
            raise ValueError("manifest must explicitly say robot_executable=false")
        if safety.get("dry_run_only") is not True:
            raise ValueError("manifest must explicitly say dry_run_only=true")
        if safety.get("publish_robot_commands") is not False:
            raise ValueError("manifest must explicitly disable robot commands")

        checkpoints = dict(self._manifest.get("policy_checkpoints", {}))
        checkpoint_a = str(ctx.runtime.cfg.policy.pretrained_path)
        manifest_a = checkpoints.get("ACT-A")
        if manifest_a and not _same_checkpoint(checkpoint_a, manifest_a):
            raise ValueError(
                f"loaded ACT-A {checkpoint_a} differs from manifest {manifest_a}"
            )
        checkpoint_b = config.checkpoint_b or str(checkpoints.get("ACT-B", ""))
        if not checkpoint_b:
            raise ValueError("ACT-B checkpoint is absent from config and manifest")

        backend_a = ResidentLeRobotACTBackend(
            ctx.policy.policy,
            ctx.policy.preprocessor,
            ctx.policy.postprocessor,
            device=str(ctx.runtime.cfg.device),
            checkpoint=checkpoint_a,
        )
        backend_b = LeRobotACTBackend(
            checkpoint_b,
            device=str(ctx.runtime.cfg.device),
        )
        a_contract = {
            key: tuple(value.shape)
            for key, value in backend_a.config.input_features.items()
        }
        b_contract = {
            key: tuple(value.shape)
            for key, value in backend_b.config.input_features.items()
        }
        if a_contract != b_contract:
            raise ValueError(f"ACT-A/B observation contracts differ: {a_contract} != {b_contract}")

        gpu_arbiter = threading.Lock()
        action_hz = float(ctx.runtime.cfg.fps)
        self._session_a = AsyncPolicySession(
            "ACT-A", backend_a, action_hz=action_hz, inference_lock=gpu_arbiter
        )
        self._session_b = AsyncPolicySession(
            "ACT-B", backend_b, action_hz=action_hz, inference_lock=gpu_arbiter
        )
        self._entries = runtime_entries_from_manifest(self._manifest)
        self._initial_planner = RuntimeBridgePlanner.from_manifest(self._manifest)
        self._tail_planner = RuntimeBridgePlanner.from_manifest(
            self._manifest, duration_search="tail_duration_search"
        )
        self._handoff_config = RuntimeHandoffConfig.from_dict(
            dict(self._manifest["runtime_handoff"])
        )

        initial_capture = self._capture(ctx)
        warm_a = self._session_a.warmup(
            initial_capture.policy_input, inferences=config.warmup_inferences
        )
        warm_b = self._session_b.warmup(
            initial_capture.policy_input, inferences=config.warmup_inferences
        )
        if self._session_a.ready_chunk is not None or self._session_b.ready_chunk is not None:
            raise RuntimeError("warmup output leaked into a policy queue")

        cuda_memory = None
        if str(ctx.runtime.cfg.device).startswith("cuda"):
            import torch

            device = torch.device(ctx.runtime.cfg.device)
            cuda_memory = {
                "device_name": torch.cuda.get_device_name(device),
                "allocated_bytes": int(torch.cuda.memory_allocated(device)),
                "reserved_bytes": int(torch.cuda.memory_reserved(device)),
                "both_models_resident": True,
            }
        self._recorder.append(
            {
                "record_type": "session_start",
                "mode": "live_observation_read_only_shadow",
                "runtime_manifest": str(manifest_path),
                "checkpoint_a": str(backend_a.checkpoint),
                "checkpoint_b": str(backend_b.checkpoint),
                "action_hz": action_hz,
                "warmup_inferences_per_policy": config.warmup_inferences,
                "warmup_latency_ms": {
                    "ACT-A": [value * 1000.0 for value in warm_a],
                    "ACT-B": [value * 1000.0 for value in warm_b],
                },
                "warmup_outputs_discarded": True,
                "shared_gpu_arbiter_only": True,
                "gpu_wait_definition": (
                    "worker_wait_to_acquire_shared_inference_lock; "
                    "CUDA execution and device queue time remain inside backend_inference_ms"
                ),
                "separate_policy_sessions": True,
                "separate_action_queues": True,
                "observation_source": "live_rollout_snapshot",
                "bridge_tracking_source": "perfect_tracking_virtual_shadow",
                "command_guard_installed": True,
                "command_publishers_created": False,
                "cuda_memory": cuda_memory,
                "semantic_assumptions": {
                    "holding": "gripper_closed_implies_true",
                    "contact_mode": "free_transport_assumed_when_closed",
                    "object_state": "A_OBJECT_HELD_ASSUMED",
                },
                "orientation_status": "pending",
                "robot_executable": False,
                "dry_run_only": True,
            }
        )
        self._setup_complete = True
        logger.info(
            "Task-C shadow ready: both ACT models resident, publishers absent, output=%s",
            self._recorder.jsonl_path,
        )

    def _new_coordinator(self, timestamp_s: float) -> TaskCRealtimeCoordinator:
        assert self._session_a is not None and self._session_b is not None
        assert self._initial_planner is not None and self._tail_planner is not None
        assert self._handoff_config is not None
        coordinator = TaskCRealtimeCoordinator(
            policy_a=self._session_a,
            policy_b=self._session_b,
            initial_planner=self._initial_planner,
            tail_planner=self._tail_planner,
            b_entries=list(self._entries),
            config=self._handoff_config,
        )
        coordinator.adopt_warmed_models(timestamp_s=timestamp_s)
        return coordinator

    def _runtime_observation(
        self,
        capture: _LiveCapture,
        *,
        position_mm: np.ndarray | None = None,
    ) -> RuntimeObservation:
        return RuntimeObservation(
            timestamp_s=capture.capture_completed_s,
            tcp_position_mm=(
                capture.tcp_position_mm if position_mm is None else position_mm
            ),
            semantic_state=capture.semantic_state,
            policy_input=capture.policy_input,
        )

    def _begin_trial(self, capture: _LiveCapture) -> None:
        coordinator = self._new_coordinator(capture.capture_completed_s)
        coordinator.start_a(self._runtime_observation(capture))
        self._active = _ActiveTrial(
            trial_id=self._trial_count + 1,
            coordinator=coordinator,
            started_s=capture.capture_completed_s,
            a_request_capture=capture,
        )

    def _cut_is_ready(self, active: _ActiveTrial) -> bool:
        assert self._handoff_config is not None
        return (
            self._closed_stable_count >= self.shadow_config.closed_stable_frames
            and len(active.coordinator.history)
            >= self._handoff_config.velocity_window_frames
        )

    def _virtual_position(self, coordinator: TaskCRealtimeCoordinator, now_s: float) -> np.ndarray:
        assert coordinator.current_plan is not None
        assert coordinator.bridge_started_s is not None
        duration = coordinator.current_plan.bridge.duration_s
        u = min(1.0, max(0.0, (now_s - coordinator.bridge_started_s) / duration))
        return coordinator.current_plan.bridge.position(u)

    def _register_b_request(self, active: _ActiveTrial, capture: _LiveCapture) -> None:
        details = _event_details(
            active.coordinator,
            "act_b_fresh_generation_requested_during_bridge",
        )
        if details is None:
            details = _event_details(
                active.coordinator,
                "act_b_final_near_entry_refresh_requested",
            )
        budget = None if details is None else float(details["remaining_bridge_s"])
        active.b_request_capture = capture
        active.b_deadline_budget_s = budget
        active.b_deadline_s = (
            None if budget is None else capture.capture_completed_s + budget
        )

    def _step_active(self, capture: _LiveCapture) -> None:
        active = self._active
        assert active is not None
        coordinator = active.coordinator
        if capture.capture_completed_s - active.started_s > self.shadow_config.trial_timeout_s:
            self._finish_trial("trial_timeout", capture.capture_completed_s)
            return

        previous_b_generation = coordinator.b_generation
        if coordinator.phase in {
            RuntimePhase.BRIDGE_RUNNING,
            RuntimePhase.B_PRIMING,
            RuntimePhase.B_READY,
        }:
            virtual_position = self._virtual_position(
                coordinator, capture.capture_completed_s
            )
            coordinator.step(
                self._runtime_observation(capture, position_mm=virtual_position)
            )
        else:
            coordinator.step(self._runtime_observation(capture))

        if active.a_chunk is None and coordinator.phase is RuntimePhase.A_RUNNING:
            active.a_chunk = self._session_a.ready_chunk

        if coordinator.phase is RuntimePhase.A_RUNNING and self._cut_is_ready(active):
            active.cut_capture = capture
            try:
                selected = coordinator.request_cut(
                    self._runtime_observation(capture),
                    semantic_cut_valid=True,
                    a_retained_length_mm=self._a_retained_length_mm,
                )
                active.initial_plan_record = _candidate_record(selected)
            except Exception as exc:
                logger.warning("Task-C shadow initial planning failed: %s", exc)
                self._finish_trial(
                    "initial_planning_failed", capture.capture_completed_s
                )
                return

        if coordinator.b_generation != previous_b_generation:
            self._register_b_request(active, capture)

        if coordinator.phase is RuntimePhase.B_READY:
            self._finish_trial("ready", time.monotonic())
        elif coordinator.phase is RuntimePhase.FAILED_HOLD:
            self._finish_trial("failed_hold", time.monotonic())

    @staticmethod
    def _capture_record(capture: _LiveCapture | None) -> dict[str, Any] | None:
        if capture is None:
            return None
        return {
            "capture_started_s": capture.capture_started_s,
            "capture_completed_s": capture.capture_completed_s,
            "processor_completed_s": capture.processor_completed_s,
            "policy_input_ready_s": capture.policy_input_ready_s,
            "capture_duration_ms": capture.capture_duration_ms,
            "observation_processor_ms": capture.processor_duration_ms,
            "policy_input_build_ms": capture.policy_input_build_ms,
            "tcp_position_mm": capture.tcp_position_mm.tolist(),
            "gripper_closed": capture.semantic_state.gripper_closed,
            "source_ages_s": capture.source_ages_s,
        }

    def _finish_trial(self, status: str, ready_s: float) -> None:
        active = self._active
        if active is None:
            return
        coordinator = active.coordinator
        b_chunk = coordinator.b_chunk
        b_capture = active.b_request_capture
        tail_event = _event_details(coordinator, "bridge_tail_replanned_from_fresh_act_b")
        if tail_event is None:
            tail_event = _event_details(coordinator, "bridge_tail_planning_failed")
        initial_event = _event_details(coordinator, "initial_bridge_planned")
        if initial_event is None:
            initial_event = _event_details(coordinator, "initial_bridge_planning_failed")

        b_observation_timestamp_s = (
            b_chunk.observation_timestamp_s
            if b_chunk is not None
            else (
                None
                if b_capture is None
                else b_capture.capture_completed_s
            )
        )
        observation_age_s = (
            None
            if b_observation_timestamp_s is None
            else max(0.0, ready_s - b_observation_timestamp_s)
        )
        deadline_missed = (
            None if active.b_deadline_s is None else ready_s > active.b_deadline_s
        )
        deadline_slack_ms = (
            None
            if active.b_deadline_s is None
            else (active.b_deadline_s - ready_s) * 1000.0
        )
        capture_to_ready_ms = (
            None
            if b_observation_timestamp_s is None
            else (ready_s - b_observation_timestamp_s) * 1000.0
        )
        stale = (
            None
            if observation_age_s is None
            else observation_age_s > self.shadow_config.stale_after_s
        )
        a_timing = (
            None
            if active.a_chunk is None
            else policy_chunk_timing_record(active.a_chunk)
        )
        b_timing = None if b_chunk is None else policy_chunk_timing_record(b_chunk)
        capture_for_flat_metrics = b_capture or active.cut_capture or active.a_request_capture
        selected = coordinator.current_plan
        assessment = coordinator.b_assessment
        tail_started_s = (
            None
            if tail_event is None
            else float(tail_event["planning_started_timestamp_s"])
        )
        tail_completed_s = (
            None
            if tail_event is None
            else float(tail_event["planning_completed_timestamp_s"])
        )
        record = {
            "record_type": "transition_trial",
            "trial_id": active.trial_id,
            "status": status,
            "terminal_phase": coordinator.phase.value,
            "trial_started_s": active.started_s,
            "ready_timestamp_s": ready_s,
            "capture_duration_ms": capture_for_flat_metrics.capture_duration_ms,
            "policy_input_build_ms": capture_for_flat_metrics.policy_input_build_ms,
            "act_a_gpu_wait_ms": None if a_timing is None else a_timing["gpu_wait_ms"],
            "act_a_backend_inference_ms": (
                None if a_timing is None else a_timing["backend_inference_ms"]
            ),
            "act_b_gpu_wait_ms": None if b_timing is None else b_timing["gpu_wait_ms"],
            "act_b_backend_inference_ms": (
                None if b_timing is None else b_timing["backend_inference_ms"]
            ),
            "act_b_capture_to_gpu_acquired_ms": (
                None
                if b_timing is None
                else b_timing["capture_to_gpu_acquired_ms"]
            ),
            "act_b_inference_to_tail_planning_ms": (
                None
                if b_chunk is None or tail_started_s is None
                else (tail_started_s - b_chunk.completed_timestamp_s) * 1000.0
            ),
            "tail_planning_ms": (
                None if tail_event is None else float(tail_event["planning_latency_s"]) * 1000.0
            ),
            "tail_planning_to_ready_ms": (
                None
                if tail_completed_s is None
                else (ready_s - tail_completed_s) * 1000.0
            ),
            "capture_to_ready_ms": capture_to_ready_ms,
            "observation_age_at_ready_ms": (
                None if observation_age_s is None else observation_age_s * 1000.0
            ),
            "stale_after_s": self.shadow_config.stale_after_s,
            "stale": stale,
            "deadline_budget_s": active.b_deadline_budget_s,
            "deadline_timestamp_s": active.b_deadline_s,
            "deadline_missed": deadline_missed,
            "deadline_slack_ms": deadline_slack_ms,
            "capture": {
                "ACT-A": self._capture_record(active.a_request_capture),
                "A-cut": self._capture_record(active.cut_capture),
                "ACT-B": self._capture_record(b_capture),
            },
            "policy_timing": {"ACT-A": a_timing, "ACT-B": b_timing},
            "planning_timing": {
                "initial": initial_event,
                "tail": tail_event,
            },
            "initial_plan": active.initial_plan_record,
            "selected_tail": None if selected is None else _candidate_record(selected),
            "act_b_assessment": (
                None
                if assessment is None
                else {
                    "valid": assessment.valid,
                    "failure_reasons": list(assessment.failure_reasons),
                    "intended_velocity_mm_s": assessment.intended_velocity_mm_s.tolist(),
                    "first_position_jump_mm": assessment.first_position_jump_mm,
                    "max_predicted_velocity_mm_s": assessment.max_predicted_velocity_mm_s,
                }
            ),
            "coordinator_failure_reasons": list(coordinator.failure_reasons),
            "coordinator_events": [asdict(event) for event in coordinator.events],
            "observation_source": "live_rollout_snapshot",
            "planning_state_before_cut": "live_measured_tcp_history",
            "planning_state_during_bridge": "perfect_tracking_virtual_shadow",
            "act_b_velocity_source": "fresh_postprocessed_act_b_action_chunk",
            "single_frame_velocity_difference_used": False,
            "semantic_assumption_active": True,
            "orientation_status": "pending",
            "orientation_bridge_generated": False,
            "ik_status": "NOT_CHECKED",
            "collision_status": "NOT_CHECKED_WITH_PAYLOAD",
            "robot_executable": False,
            "dry_run_only": True,
            "robot_commands_published": False,
        }
        assert self._recorder is not None
        self._recorder.append(record)
        self._trial_count += 1
        self._last_trial_finished_s = ready_s
        if self._session_a is not None:
            self._session_a.deactivate_and_clear()
        if self._session_b is not None:
            self._session_b.deactivate_and_clear()
        self._active = None
        logger.info(
            "Task-C shadow trial %d status=%s stale=%s deadline_missed=%s",
            active.trial_id,
            status,
            stale,
            deadline_missed,
        )

    def _heartbeat(self, capture: _LiveCapture) -> None:
        now = capture.capture_completed_s
        if now - self._last_heartbeat_s < 1.0:
            return
        self._last_heartbeat_s = now
        assert self._recorder is not None
        self._recorder.append(
            {
                "record_type": "heartbeat",
                "timestamp_s": now,
                "active_trial": None if self._active is None else self._active.trial_id,
                "closed_stable_frames": self._closed_stable_count,
                "semantic_trial_eligible": (
                    self._closed_stable_count
                    >= self.shadow_config.closed_stable_frames
                ),
                "capture_duration_ms": capture.capture_duration_ms,
                "policy_input_build_ms": capture.policy_input_build_ms,
                "source_ages_s": capture.source_ages_s,
                "robot_commands_published": False,
            }
        )

    def run(self, ctx: Any) -> None:
        if not self._setup_complete:
            raise RuntimeError("Task-C shadow setup did not complete")
        cfg = ctx.runtime.cfg
        interval_s = 1.0 / float(cfg.fps)
        started_s = time.monotonic()
        while not ctx.runtime.shutdown_event.is_set():
            loop_started = time.monotonic()
            if cfg.duration > 0.0 and loop_started - started_s >= cfg.duration:
                break
            if (
                self.shadow_config.max_trials > 0
                and self._trial_count >= self.shadow_config.max_trials
            ):
                break
            try:
                capture = self._capture(ctx)
            except Exception as exc:
                self._capture_failures += 1
                assert self._recorder is not None
                self._recorder.append(
                    {
                        "record_type": "capture_failure",
                        "timestamp_s": time.monotonic(),
                        "error_type": type(exc).__name__,
                        "error": str(exc),
                        "source_stale_or_unavailable": True,
                        "robot_commands_published": False,
                    }
                )
                logger.warning("Task-C shadow capture failed: %s", exc)
                precise_sleep(interval_s)
                continue
            self._capture_successes += 1

            if self._last_actual_position_mm is not None:
                self._a_retained_length_mm += float(
                    np.linalg.norm(
                        capture.tcp_position_mm - self._last_actual_position_mm
                    )
                )
            self._last_actual_position_mm = capture.tcp_position_mm.copy()
            if capture.semantic_state.gripper_closed:
                self._closed_stable_count += 1
            else:
                self._closed_stable_count = 0
                self._semantic_wait_frames += 1

            if self._active is None:
                eligible = (
                    self._closed_stable_count
                    >= self.shadow_config.closed_stable_frames
                )
                interval_elapsed = (
                    capture.capture_completed_s - self._last_trial_finished_s
                    >= self.shadow_config.trial_interval_s
                )
                if eligible and interval_elapsed:
                    self._begin_trial(capture)
            else:
                self._step_active(capture)
            self._heartbeat(capture)

            elapsed = time.monotonic() - loop_started
            if elapsed < interval_s:
                precise_sleep(interval_s - elapsed)
            else:
                logger.warning(
                    "Task-C shadow loop missed %.1f Hz period: %.2f ms",
                    cfg.fps,
                    elapsed * 1000.0,
                )

    def teardown(self, ctx: Any) -> None:
        if self._active is not None:
            self._finish_trial("shutdown_before_ready", time.monotonic())
        if self._session_a is not None:
            self._session_a.close()
        if self._session_b is not None:
            self._session_b.close()

        final_counts = {
            "live_publish_count": int(getattr(self._raw_robot, "live_publish_count", 0)),
            "hold_publish_count": int(getattr(self._raw_robot, "hold_publish_count", 0)),
            "debug_publish_count": int(getattr(self._raw_robot, "debug_publish_count", 0)),
        }
        counts_unchanged = final_counts == self._command_count_baseline
        publishers_absent = all(
            getattr(self._raw_robot, name, None) is None
            for name in ("_target_pub", "_gripper_pub", "_debug_pub", "_policy_ready_pub")
        )
        extra = {
            "capture_success_count": self._capture_successes,
            "capture_failure_count": self._capture_failures,
            "source_capture_failure_rate": (
                None
                if self._capture_successes + self._capture_failures == 0
                else self._capture_failures
                / (self._capture_successes + self._capture_failures)
            ),
            "semantic_wait_frames": self._semantic_wait_frames,
            "session_stats": {
                "ACT-A": None if self._session_a is None else asdict(self._session_a.stats()),
                "ACT-B": None if self._session_b is None else asdict(self._session_b.stats()),
            },
            "read_only_proof": {
                "command_attempts_blocked": self._command_attempts,
                "publish_counts_before": self._command_count_baseline,
                "publish_counts_after": final_counts,
                "publish_counts_unchanged": counts_unchanged,
                "command_publishers_absent": publishers_absent,
            },
        }
        try:
            if self._recorder is not None:
                self._recorder.append(
                    {
                        "record_type": "session_safety_check",
                        **extra["read_only_proof"],
                        "robot_commands_published": not counts_unchanged,
                        "robot_executable": False,
                        "dry_run_only": True,
                    }
                )
                summary = self._recorder.close(extra_summary=extra)
                logger.info(
                    "Task-C shadow summary: trials=%d stale_rate=%s deadline_miss_rate=%s",
                    summary["trial_count"],
                    summary["stale_rate"],
                    summary["deadline_miss_rate"],
                )
        finally:
            try:
                robot = ctx.hardware.robot_wrapper.inner
                if robot.is_connected:
                    robot.disconnect()
                teleop = ctx.hardware.teleop
                if teleop is not None and teleop.is_connected:
                    teleop.disconnect()
            finally:
                self._remove_command_guard()
        if not counts_unchanged or not publishers_absent or self._command_attempts:
            logger.error("Task-C shadow read-only invariant was violated")


_FACTORY_INSTALLED = False


def install_task_c_shadow_strategy() -> bool:
    """Patch only the local rollout factory to dispatch our registered type."""

    global _FACTORY_INSTALLED
    if _FACTORY_INSTALLED:
        return False
    import lerobot.rollout as rollout_package
    import lerobot.rollout.strategies as strategies_package
    from lerobot.rollout.strategies import factory

    original = factory.create_strategy

    def create_strategy(config: RolloutStrategyConfig) -> RolloutStrategy:
        if config.type == "task_c_shadow":
            if not isinstance(config, TaskCShadowStrategyConfig):
                raise TypeError("task_c_shadow config registration mismatch")
            return TaskCShadowStrategy(config)
        return original(config)

    factory.create_strategy = create_strategy
    strategies_package.create_strategy = create_strategy
    rollout_package.create_strategy = create_strategy
    _FACTORY_INSTALLED = True
    return True
