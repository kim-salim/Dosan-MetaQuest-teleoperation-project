"""Opt-in non-blocking Task-C handoff V2 live strategy.

V0/V1 remain untouched and selectable through ``task_c_live``.  This module
subclasses that reviewed command path so policy proposals still traverse the
A0509 robot adapter, MUX, guard, and ServoL streamer.  It replaces only the
A-exit-to-B-takeover coordinator when ``strategy.type=task_c_live_v2``.
"""

from __future__ import annotations

from collections import deque

import json
import logging
import os
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np
from lerobot.rollout.configs import RolloutStrategyConfig
from lerobot.rollout.strategies.core import RolloutStrategy
from std_msgs.msg import Float64MultiArray, String

from offline_tools.cross_task_handoff.authority import (
    SemanticAuthority,
    parse_semantic_authority,
)
from offline_tools.task_c_bridge_v0.live_transition import (
    OrientationBridge,
    full_action,
    pose_delta_metrics,
)
from offline_tools.task_c_bridge_v0.runtime_orchestrator import (
    RuntimeObservation,
    RuntimePhase,
)
from offline_tools.task_c_bridge_v1.runtime_boundary import (
    RepresentativeBoundaryContract,
    RepresentativeBoundaryTracker,
)

from lerobot_robot_doosan_a0509.task_c_handoff.async_successor import (
    AsyncSuccessorController,
    TimedACTBackend,
)
from lerobot_robot_doosan_a0509.task_c_handoff.bridge_runtime import (
    BridgeGenerationError,
    BridgeRuntimeLimits,
    BridgeRuntimeSnapshot,
    PrecomputedBridgeQueue,
    instantiate_bridge_queue,
)
from lerobot_robot_doosan_a0509.task_c_handoff.compatibility import (
    HandoffCompatibilityEvaluator,
    HandoffSpliceSelector,
)
from lerobot_robot_doosan_a0509.task_c_handoff.coordinator import (
    HandoffCommand,
    TaskCHandoffV2Coordinator,
    V2CoordinatorEvent,
)
from lerobot_robot_doosan_a0509.task_c_handoff.handoff_trace import (
    ControlTimingMonitor,
    NonBlockingHandoffTrace,
    json_default,
)
from lerobot_robot_doosan_a0509.task_c_handoff.flexible_bridge import (
    AsyncFlexibleBridgePlanner,
    FlexibleBridgeSearchConfig,
    FlexibleBridgeSearchResult,
    FlexibleBridgeTemplate,
    nominal_medoid_bridge_snapshot,
    validate_flexible_entry_rebase,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    BoundarySemanticState,
    BridgeAdmissionMode,
    EpisodeHandoffManifest,
    HandoffCompatibilityConfig,
    HandoffMode,
    HandoffV2Config,
    HandoffV2State,
)
from lerobot_robot_doosan_a0509.task_c_handoff.recording import (
    AsyncTaskCLeRobotRecorder,
)
from lerobot_robot_doosan_a0509.task_c_handoff.runtime_command_profile import (
    get_runtime_command_profile,
)
from lerobot_robot_doosan_a0509.task_c_handoff.source_phase import (
    SourcePhaseStatus,
    SourcePhaseSupportBank,
    SourcePhaseSupportConfig,
    SourcePhaseSupportTracker,
    SourceTriggerMode,
)
from lerobot_robot_doosan_a0509.task_c_handoff.source_queue_splice import (
    PredictiveSourceSplice,
    SourceActionQueueSnapshot,
    predictive_lookahead_steps,
    project_predictive_source_splice,
    snapshot_rtc_source_queue,
)
from lerobot_robot_doosan_a0509.task_c_shadow_rollout import _same_checkpoint
from lerobot_robot_doosan_a0509.task_c_live_rollout import (
    LivePhase,
    TaskCLiveStrategy,
    TaskCLiveStrategyConfig,
    _LiveCapture,
    _PendingLiveCommand,
)
from lerobot_robot_doosan_a0509.runtime_scheduling import parse_cpu_set


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class _PreparedBridgeCommit:
    bridge: PrecomputedBridgeQueue
    acknowledged_pose_mm_deg: np.ndarray
    acknowledged_receive_count: int
    measured_velocity_mm_s: np.ndarray
    actual_semantic: BoundarySemanticState
    actual_to_ack_position_error_mm: float
    actual_to_ack_orientation_error_deg: float
    capture_timestamp_s: float


@dataclass(frozen=True)
class _BridgeSnapshotContext:
    snapshot: BridgeRuntimeSnapshot
    acknowledged_receive_count: int
    actual_semantic: BoundarySemanticState
    actual_to_ack_position_error_mm: float
    actual_to_ack_orientation_error_deg: float


@RolloutStrategyConfig.register_subclass("task_c_live_v2")
@dataclass
class TaskCLiveV2StrategyConfig(TaskCLiveStrategyConfig):
    # V2 delegates the complete post-takeover behavior to fresh ACT-B rolling
    # inference. V0/V1 keep the inherited legacy mode unless explicitly changed.
    b_completion_mode: str = "successor_owned"
    handoff_mode: str = HandoffMode.ASYNC_WINDOW_V2.value
    handoff_episode_manifest: str = ""
    bridge_admission_mode: str = BridgeAdmissionMode.STRICT_LEVEL2.value
    semantic_authority: str = ""
    v2_trace_jsonl_path: str = ""
    v2_trace_csv_path: str = ""
    # Empty preserves standalone historical V2 runs. Planner-compiled Multi-V2
    # plans bind this identifier and validate every linked command limit.
    runtime_command_profile_id: str = ""
    handoff_window_steps: int = 24
    b_prefix_steps: int = 15
    crossfade_steps: int = 15
    max_b_result_age_sec: float = 0.30
    max_inflight_b_requests: int = 1
    b_request_retry_interval_steps: int = 3
    adaptive_b_max_splice_index: int = 0
    adaptive_b_max_candidates: int = 1
    enable_soft_handoff: bool = True
    enable_endpoint_fallback: bool = True
    max_first_xyz_axis_delta_mm: float | None = None
    max_first_rotation_delta_deg: float | None = None
    max_prefix_velocity_mm_s: float | None = None
    max_prefix_acceleration_mm_s2: float | None = None
    # Shared proposal-command acceleration admission bound for runtime Bridge
    # generation, cached-entry revalidation, and the inherited endpoint
    # fallback planners. This is not a Doosan hardware acceleration rating.
    bridge_acceleration_limit_mm_s2: float = 4000.0
    # Optional runtime overrides preserve historical manifests and STRICT
    # replay while allowing the reviewed FLEXIBLE_LEVEL2 command profile to
    # be selected explicitly by its launcher.
    bridge_jerk_limit_mm_s3: float | None = None
    bridge_integrated_squared_jerk_limit: float | None = None
    # Provisional command-space admission bound selected from command-free
    # T2->T3 shadow traces. This is not a Doosan hardware acceleration rating.
    max_crossfade_command_acceleration_mm_s2: float | None = 4000.0
    max_bridge_prefix_velocity_mismatch_mm_s: float | None = None
    max_crossfade_xyz_axis_step_mm: float | None = None
    max_crossfade_rotation_step_deg: float | None = None
    record_task_c_dataset: bool = False
    recording_queue_size: int = 96
    source_trigger_mode: str = SourceTriggerMode.MEDIAN_SPHERE.value
    source_phase_artifact_path: str = ""
    source_phase_half_width: float = 0.05
    source_phase_prearm_extra: float = 0.02
    source_phase_persistence_ticks: int = 3
    source_phase_local_search_radius_indices: int = 5
    source_phase_backward_tolerance: float = 0.02
    source_phase_deadline_extra: float = 0.0
    source_support_distance_threshold_mm: float | None = None
    source_support_loo_quantile: float = 0.95
    successor_runtime_phase_gate: bool = False
    flexible_bridge_max_result_age_sec: float = 0.30
    flexible_bridge_max_candidates: int = 64
    flexible_bridge_max_search_time_s: float = 0.50
    flexible_bridge_worker_backend: str = "process"
    flexible_bridge_cpu_set: str = "13"
    flexible_bridge_worker_nice: int = 10
    flexible_bridge_rebase_retry_interval_s: float = 0.10
    flexible_bridge_template_max_attempts: int = 3
    flexible_bridge_template_setup_timeout_s: float = 2.0
    # Predict the near-future ACT-A queue point reached while the isolated
    # adaptive-entry worker runs. The queue remains authoritative until the
    # intended splice sequence, then latest actual/ACK hard admission decides.
    flexible_predictive_source_splice: bool = True
    flexible_source_splice_nominal_latency_s: float = 0.12
    flexible_source_splice_margin_steps: int = 2
    flexible_source_splice_min_lookahead_steps: int = 4
    flexible_source_splice_max_lookahead_steps: int = 8
    flexible_source_splice_queue_snapshot_steps: int = 12
    flexible_source_splice_late_tolerance_steps: int = 1
    flexible_source_splice_velocity_window_steps: int = 5
    flexible_tangent_handle_chord_ratios: tuple[float, ...] = (
        0.0,
        0.05,
        0.10,
        0.20,
        0.25,
    )
    flexible_duration_multipliers: tuple[float, ...] = (1.0, 1.25, 1.50)
    flexible_enable_settle_connector: bool = True
    flexible_enable_alignment_connector: bool = True
    flexible_enable_lift_transport_connector: bool = True
    flexible_enable_generic_safe_connector: bool = False
    allow_intermediate_release: bool = False

    def __post_init__(self) -> None:
        super().__post_init__()
        admission_mode = BridgeAdmissionMode(self.bridge_admission_mode)
        if self.runtime_command_profile_id:
            profile = get_runtime_command_profile(
                self.runtime_command_profile_id
            )
            exact_values = {
                "downstream_control_hz": (
                    self.downstream_control_hz,
                    profile.control_hz,
                ),
                "downstream_linear_ramp_mm_per_tick": (
                    self.downstream_linear_ramp_mm_per_tick,
                    profile.linear_ramp_mm_per_tick,
                ),
                "downstream_orientation_ramp_deg_per_tick": (
                    self.downstream_orientation_ramp_deg_per_tick,
                    profile.orientation_ramp_deg_per_tick,
                ),
                "bridge_acceleration_limit_mm_s2": (
                    self.bridge_acceleration_limit_mm_s2,
                    profile.acceleration_limit_mm_s2,
                ),
                "bridge_jerk_limit_mm_s3": (
                    self.bridge_jerk_limit_mm_s3,
                    profile.jerk_limit_mm_s3,
                ),
                "bridge_integrated_squared_jerk_limit": (
                    self.bridge_integrated_squared_jerk_limit,
                    profile.integrated_squared_jerk_limit,
                ),
                "max_crossfade_command_acceleration_mm_s2": (
                    self.max_crossfade_command_acceleration_mm_s2,
                    profile.acceleration_limit_mm_s2,
                ),
            }
            mismatches = [
                name
                for name, (actual, expected) in exact_values.items()
                if actual is None
                or not np.isclose(
                    float(actual),
                    float(expected),
                    rtol=0.0,
                    atol=1.0e-9,
                )
            ]
            if self.bridge_ack_mode != profile.bridge_ack_mode:
                mismatches.append("bridge_ack_mode")
            if self.bridge_max_ack_lag_steps != profile.max_ack_lag_steps:
                mismatches.append("bridge_max_ack_lag_steps")
            if (
                self.max_prefix_velocity_mm_s is not None
                and not np.isclose(
                    self.max_prefix_velocity_mm_s,
                    profile.cartesian_velocity_limit_mm_s,
                    rtol=0.0,
                    atol=1.0e-9,
                )
            ):
                mismatches.append("max_prefix_velocity_mm_s")
            if mismatches:
                raise ValueError(
                    "runtime command profile mismatch: "
                    + ",".join(mismatches)
                )
        if self.handoff_mode != HandoffMode.ASYNC_WINDOW_V2.value:
            raise ValueError("task_c_live_v2 requires handoff_mode=async_window_v2")
        if not self.handoff_episode_manifest:
            raise ValueError("task_c_live_v2 requires handoff_episode_manifest")
        if not self.v2_trace_jsonl_path:
            raise ValueError("task_c_live_v2 requires v2_trace_jsonl_path")
        if self.semantic_authority:
            parse_semantic_authority(self.semantic_authority)
        if not isinstance(self.record_task_c_dataset, bool):
            raise ValueError("record_task_c_dataset must be boolean")
        if (
            not isinstance(self.recording_queue_size, int)
            or isinstance(self.recording_queue_size, bool)
            or self.recording_queue_size < 1
        ):
            raise ValueError("recording_queue_size must be a positive integer")
        try:
            source_mode = SourceTriggerMode(self.source_trigger_mode)
        except ValueError as exc:
            choices = ",".join(mode.value for mode in SourceTriggerMode)
            raise ValueError(f"source_trigger_mode must be one of: {choices}") from exc
        if source_mode is SourceTriggerMode.PHASE_SUPPORT and not (
            self.source_phase_artifact_path
        ):
            raise ValueError(
                "phase_support requires source_phase_artifact_path"
            )
        if (
            admission_mode is BridgeAdmissionMode.FLEXIBLE_LEVEL2
            and source_mode is SourceTriggerMode.MEDIAN_SPHERE
            and not getattr(self, "multi_stage_plan", "")
        ):
            raise ValueError(
                "single-edge FLEXIBLE_LEVEL2 requires phase_support source collar"
            )
        if not isinstance(self.successor_runtime_phase_gate, bool):
            raise ValueError("successor_runtime_phase_gate must be boolean")
        if self.successor_runtime_phase_gate:
            raise ValueError(
                "Task-C V2 forbids successor runtime phase authority; "
                "ACT-B takeover is fresh-output based"
            )
        if self.allow_intermediate_release:
            raise ValueError(
                "Task-C FLEXIBLE bridge runtime does not allow intermediate release"
            )
        if (
            not np.isfinite(self.bridge_acceleration_limit_mm_s2)
            or self.bridge_acceleration_limit_mm_s2 <= 0.0
        ):
            raise ValueError(
                "bridge_acceleration_limit_mm_s2 must be finite and positive"
            )
        for name, value in (
            ("bridge_jerk_limit_mm_s3", self.bridge_jerk_limit_mm_s3),
            (
                "bridge_integrated_squared_jerk_limit",
                self.bridge_integrated_squared_jerk_limit,
            ),
        ):
            if value is not None and (
                not np.isfinite(value) or value <= 0.0
            ):
                raise ValueError(f"{name} must be finite and positive or null")
        if (
            self.source_support_distance_threshold_mm is not None
            and (
                not np.isfinite(self.source_support_distance_threshold_mm)
                or self.source_support_distance_threshold_mm <= 0.0
            )
        ):
            raise ValueError(
                "source_support_distance_threshold_mm must be positive or null"
            )
        if (
            not np.isfinite(self.source_support_loo_quantile)
            or not 0.0 < self.source_support_loo_quantile <= 1.0
        ):
            raise ValueError("source_support_loo_quantile must be in (0, 1]")
        SourcePhaseSupportConfig(
            nominal_phase=0.5,
            phase_half_width=self.source_phase_half_width,
            prearm_extra_phase=self.source_phase_prearm_extra,
            persistence_ticks=self.source_phase_persistence_ticks,
            local_search_radius_indices=(
                self.source_phase_local_search_radius_indices
            ),
            backward_tolerance=self.source_phase_backward_tolerance,
            deadline_extra_phase=self.source_phase_deadline_extra,
        )
        HandoffV2Config(
            enabled=True,
            handoff_window_steps=self.handoff_window_steps,
            b_prefix_steps=self.b_prefix_steps,
            crossfade_steps=self.crossfade_steps,
            max_b_result_age_sec=self.max_b_result_age_sec,
            max_inflight_b_requests=self.max_inflight_b_requests,
            request_retry_interval_steps=self.b_request_retry_interval_steps,
            bridge_admission_mode=admission_mode,
            adaptive_b_max_splice_index=self.adaptive_b_max_splice_index,
            adaptive_b_max_candidates=self.adaptive_b_max_candidates,
            enable_soft_handoff=self.enable_soft_handoff,
            enable_endpoint_fallback=self.enable_endpoint_fallback,
            semantic_authority=(self.semantic_authority or SemanticAuthority.RUNTIME_GUARDED.value),
            control_hz=self.downstream_control_hz,
        )
        if admission_mode is BridgeAdmissionMode.FLEXIBLE_LEVEL2:
            if not isinstance(self.flexible_predictive_source_splice, bool):
                raise ValueError(
                    "flexible_predictive_source_splice must be boolean"
                )
            if (
                not np.isfinite(self.flexible_source_splice_nominal_latency_s)
                or self.flexible_source_splice_nominal_latency_s < 0.0
            ):
                raise ValueError(
                    "flexible_source_splice_nominal_latency_s must be "
                    "finite and non-negative"
                )
            for name, value, minimum in (
                ("flexible_source_splice_margin_steps", self.flexible_source_splice_margin_steps, 0),
                ("flexible_source_splice_min_lookahead_steps", self.flexible_source_splice_min_lookahead_steps, 1),
                ("flexible_source_splice_max_lookahead_steps", self.flexible_source_splice_max_lookahead_steps, 1),
                ("flexible_source_splice_queue_snapshot_steps", self.flexible_source_splice_queue_snapshot_steps, 2),
                ("flexible_source_splice_late_tolerance_steps", self.flexible_source_splice_late_tolerance_steps, 0),
                ("flexible_source_splice_velocity_window_steps", self.flexible_source_splice_velocity_window_steps, 3),
            ):
                if (
                    not isinstance(value, int)
                    or isinstance(value, bool)
                    or value < minimum
                ):
                    raise ValueError(f"{name} must be an integer >= {minimum}")
            if (
                self.flexible_source_splice_max_lookahead_steps
                < self.flexible_source_splice_min_lookahead_steps
            ):
                raise ValueError(
                    "source splice maximum lookahead must be >= minimum"
                )
            if (
                self.flexible_source_splice_queue_snapshot_steps
                <= self.flexible_source_splice_max_lookahead_steps
            ):
                raise ValueError(
                    "source splice queue snapshot must leave at least one "
                    "action after maximum lookahead"
                )
            if self.flexible_bridge_worker_backend not in {"thread", "process"}:
                raise ValueError(
                    "flexible_bridge_worker_backend must be thread or process"
                )
            bridge_cpus = parse_cpu_set(self.flexible_bridge_cpu_set)
            if (
                self.flexible_bridge_worker_backend == "process"
                and len(bridge_cpus) != 1
            ):
                raise ValueError(
                    "live Flexible Bridge process must use exactly one isolated CPU"
                )
            if (
                not isinstance(self.flexible_bridge_worker_nice, int)
                or not 0 <= self.flexible_bridge_worker_nice <= 19
            ):
                raise ValueError("flexible_bridge_worker_nice must be in [0, 19]")
            if (
                not np.isfinite(self.flexible_bridge_rebase_retry_interval_s)
                or self.flexible_bridge_rebase_retry_interval_s <= 0.0
            ):
                raise ValueError(
                    "flexible_bridge_rebase_retry_interval_s must be positive"
                )
            if (
                not isinstance(self.flexible_bridge_template_max_attempts, int)
                or isinstance(self.flexible_bridge_template_max_attempts, bool)
                or self.flexible_bridge_template_max_attempts < 1
            ):
                raise ValueError(
                    "flexible_bridge_template_max_attempts must be positive"
                )
            if (
                not np.isfinite(self.flexible_bridge_template_setup_timeout_s)
                or self.flexible_bridge_template_setup_timeout_s <= 0.0
            ):
                raise ValueError(
                    "flexible_bridge_template_setup_timeout_s must be positive"
                )
            if self.flexible_bridge_worker_backend == "process":
                for name in (
                    "LEROBOT_A0509_ROS_CPU_SET",
                    "LEROBOT_A0509_MAIN_CPU_SET",
                    "LEROBOT_A0509_ACT_INFERENCE_CPU_SET",
                ):
                    configured = os.environ.get(name, "").strip()
                    if configured and bridge_cpus.intersection(
                        parse_cpu_set(configured)
                    ):
                        raise ValueError(
                            "Flexible Bridge CPU set must be disjoint from "
                            f"{name}: bridge={sorted(bridge_cpus)} "
                            f"other={sorted(parse_cpu_set(configured))}"
                        )
            FlexibleBridgeSearchConfig(
                tangent_handle_chord_ratios=tuple(
                    self.flexible_tangent_handle_chord_ratios
                ),
                duration_multipliers=tuple(
                    self.flexible_duration_multipliers
                ),
                enable_settle_connector=self.flexible_enable_settle_connector,
                enable_alignment_connector=(
                    self.flexible_enable_alignment_connector
                ),
                enable_lift_transport_connector=(
                    self.flexible_enable_lift_transport_connector
                ),
                enable_generic_safe_connector=(
                    self.flexible_enable_generic_safe_connector
                ),
                allow_intermediate_release=self.allow_intermediate_release,
                max_candidates=self.flexible_bridge_max_candidates,
                max_search_time_s=self.flexible_bridge_max_search_time_s,
            )


class TaskCLiveV2Strategy(TaskCLiveStrategy):
    """Existing ACT-A/B command path with a precomputed async handoff window."""

    def __init__(self, config: TaskCLiveV2StrategyConfig) -> None:
        super().__init__(config)
        self.v2_config = config
        self._semantic_authority = parse_semantic_authority(
            config.semantic_authority or SemanticAuthority.RUNTIME_GUARDED.value
        )
        self._v2_manifest: EpisodeHandoffManifest | None = None
        self._v2_runtime_config: HandoffV2Config | None = None
        self._v2_coordinator: TaskCHandoffV2Coordinator | None = None
        self._v2_runtime_active = False
        self._v2_command_meta: HandoffCommand | None = None
        self._v2_trace: NonBlockingHandoffTrace | None = None
        self._v2_pretrace_events: list[dict[str, Any]] = []
        self._v2_recorder: AsyncTaskCLeRobotRecorder | None = None
        self._v2_latest_processed: dict[str, Any] | None = None
        self._v2_latest_raw: dict[str, Any] | None = None
        self._v2_latest_capture: _LiveCapture | None = None
        self._v2_control_timing = ControlTimingMonitor(
            control_hz=config.downstream_control_hz
        )
        self._v2_last_capture_start_s: float | None = None
        self._v2_last_control_tick_ms: float | None = None
        self._v2_safe_pose: np.ndarray | None = None
        self._v2_safe_pose_timestamp_s: float | None = None
        self._v2_selected_pose: np.ndarray | None = None
        self._v2_selected_pose_timestamp_s: float | None = None
        self._v2_audit_subscriptions: list[Any] = []
        self._v2_endpoint_fallback_used = False
        self._v2_full_takeover_timestamp_s: float | None = None
        self._source_phase_bank: SourcePhaseSupportBank | None = None
        self._source_phase_tracker: SourcePhaseSupportTracker | None = None
        self._source_phase_status: SourcePhaseStatus | None = None
        self._source_phase_latency_ms: list[float] = []
        self._source_phase_commit_latched = False
        self._prepared_bridge_commit: _PreparedBridgeCommit | None = None
        self._source_bridge_last_failure: str | None = None
        self._v2_acknowledged_command_history: deque[np.ndarray] = deque(maxlen=2)
        self._v2_last_acknowledged_receive_count: int | None = None
        self._flexible_bridge_worker: AsyncFlexibleBridgePlanner | None = None
        self._flexible_bridge_search_config: FlexibleBridgeSearchConfig | None = None
        self._flexible_bridge_nominal_template_cache: dict[
            str, FlexibleBridgeTemplate
        ] = {}
        self._flexible_bridge_template: FlexibleBridgeTemplate | None = None
        self._flexible_bridge_template_requested = False
        self._flexible_bridge_template_request_attempts = 0
        self._flexible_bridge_template_retry_exhausted_logged = False
        self._flexible_bridge_next_template_request_s = 0.0
        self._flexible_bridge_ready: FlexibleBridgeSearchResult | None = None
        self._flexible_bridge_next_rebase_request_s = 0.0
        self._flexible_bridge_inflight_splice: PredictiveSourceSplice | None = None
        self._flexible_bridge_inflight_generation: int | None = None
        self._flexible_bridge_inflight_requested_s: float | None = None
        self._flexible_bridge_ready_splice: PredictiveSourceSplice | None = None
        self._flexible_bridge_ready_generation: int | None = None
        self._flexible_bridge_result_wall_latencies_s: deque[float] = deque(
            maxlen=32
        )

    def _event(self, event: str, **details: Any) -> None:
        record = {
            "schema": "task_c_live_v2",
            "record_type": "event",
            "timestamp_s": time.monotonic(),
            "state": (
                self._v2_coordinator.state.value
                if self._v2_coordinator is not None
                else self.phase.value
            ),
            "phase": self.phase.value,
            "event": event,
            "handoff_id": (
                None if self._v2_manifest is None else self._v2_manifest.handoff_id
            ),
            **details,
        }
        if self._v2_trace is None:
            self._v2_pretrace_events.append(record)
        else:
            self._v2_trace.emit(record)
        message = String(
            data=json.dumps(record, sort_keys=True, default=json_default)
        )
        if self._event_pub is not None:
            self._event_pub.publish(message)
        if (
            event == "strategy_ready_external_gate_required"
            and self._ready_pub is not None
        ):
            self._ready_pub.publish(message)
        if self._phase_pub is not None:
            self._phase_pub.publish(String(data=str(record["state"])))

    def _uses_legacy_endpoint_fallback(self) -> bool:
        """Whether endpoint fallback is backed by the single-edge V1 manifest."""

        return True

    def setup(self, ctx: Any) -> None:
        config = self.v2_config
        self._v2_manifest = EpisodeHandoffManifest.load(
            config.handoff_episode_manifest
        )
        admission_mode = BridgeAdmissionMode(config.bridge_admission_mode)
        if self._v2_manifest.bridge_admission_mode is not admission_mode:
            raise ValueError(
                "configured bridge_admission_mode differs from episode manifest"
            )
        manifest_authority = self._v2_manifest.semantic_authority
        if config.semantic_authority:
            configured_authority = parse_semantic_authority(config.semantic_authority)
            if configured_authority is not manifest_authority:
                raise ValueError(
                    "configured semantic_authority differs from episode manifest"
                )
        self._semantic_authority = manifest_authority
        if (
            self._v2_manifest.bridge_duration_s
            >= config.transition_timeout_s
        ):
            raise ValueError(
                "V2 Bridge duration must be below transition_timeout_s"
            )
        source_semantic = self._v2_manifest.source.semantic
        successor_semantic = self._v2_manifest.successor.semantic
        compatible, reasons = HandoffCompatibilityEvaluator.semantic_compatibility(
            source_semantic,
            successor_semantic,
        )
        if not compatible and self._semantic_authority.runtime_semantic_checks_enforced:
            raise ValueError(
                "selected V2 handoff violates hard semantic compatibility: "
                + ",".join(reasons)
            )
        if (
            self._semantic_authority.runtime_semantic_checks_enforced
            and source_semantic.contact_mode not in {
                "free_transport",
                "free_transport_assumed",
            }
        ):
            raise ValueError("initial V2 runtime supports free_transport only")

        source_mode = SourceTriggerMode(config.source_trigger_mode)
        source_phase_config: SourcePhaseSupportConfig | None = None
        preloaded_source_bank: SourcePhaseSupportBank | None = None
        if source_mode is SourceTriggerMode.PHASE_SUPPORT:
            source_phase_config = SourcePhaseSupportConfig(
                nominal_phase=self._v2_manifest.source.phase,
                phase_half_width=config.source_phase_half_width,
                prearm_extra_phase=config.source_phase_prearm_extra,
                persistence_ticks=config.source_phase_persistence_ticks,
                local_search_radius_indices=(
                    config.source_phase_local_search_radius_indices
                ),
                backward_tolerance=config.source_phase_backward_tolerance,
                deadline_extra_phase=config.source_phase_deadline_extra,
            )
            preloaded_source_bank = SourcePhaseSupportBank.load(
                config.source_phase_artifact_path,
                segment=self._v2_manifest.source.segment,
                phase_window=source_phase_config.phase_window,
                support_distance_threshold_mm=(
                    config.source_support_distance_threshold_mm
                ),
                support_loo_quantile=config.source_support_loo_quantile,
            )

        super().setup(ctx)
        if config.enable_endpoint_fallback and self._uses_legacy_endpoint_fallback():
            legacy_checkpoints = dict(
                self._manifest.get("policy_checkpoints", {})
            )
            legacy_b = str(legacy_checkpoints.get("ACT-B", ""))
            if not legacy_b:
                raise ValueError(
                    "endpoint V1 fallback requires manifest ACT-B checkpoint"
                )
            if not _same_checkpoint(config.checkpoint_b, legacy_b):
                raise ValueError(
                    "V2 ACT-B differs from endpoint V1 fallback checkpoint"
                )
        if self._event_file is not None:
            # V2 events are written only by NonBlockingHandoffTrace.
            self._event_file.close()
            self._event_file = None
        self._v2_trace = NonBlockingHandoffTrace(
            config.v2_trace_jsonl_path,
            csv_path=(config.v2_trace_csv_path or None),
        )
        for record in self._v2_pretrace_events:
            self._v2_trace.emit(record)
        self._v2_pretrace_events.clear()

        assert self._session_b is not None
        self._session_b.backend = TimedACTBackend(self._session_b.backend)
        assert self._handoff_config is not None
        assert self._coordinator is not None
        initial_planner = self._coordinator.initial_planner
        # Historical manifests retain their original limits for reproducible
        # artifacts. A V2 launcher can explicitly override the reviewed
        # command-space limits before any live Bridge candidate is generated.
        for planner in (
            self._coordinator.initial_planner,
            self._coordinator.tail_planner,
        ):
            planner.feasibility_config["acceleration_limit_mm_s2"] = (
                config.bridge_acceleration_limit_mm_s2
            )
            if "boundary_acceleration_jump_limit_mm_s2" in (
                planner.feasibility_config
            ):
                planner.feasibility_config[
                    "boundary_acceleration_jump_limit_mm_s2"
                ] = config.bridge_acceleration_limit_mm_s2
            if config.bridge_jerk_limit_mm_s3 is not None:
                planner.feasibility_config["jerk_limit_mm_s3"] = (
                    config.bridge_jerk_limit_mm_s3
                )
            if config.bridge_integrated_squared_jerk_limit is not None:
                planner.feasibility_config[
                    "integrated_squared_jerk_limit"
                ] = config.bridge_integrated_squared_jerk_limit
        feasibility = initial_planner.feasibility_config
        compatibility = HandoffCompatibilityConfig(
            max_first_xyz_axis_delta_mm=(
                config.max_first_xyz_axis_delta_mm
                if config.max_first_xyz_axis_delta_mm is not None
                else self._handoff_config.b_first_action_position_jump_limit_mm
            ),
            max_first_rotation_delta_deg=(
                config.max_first_rotation_delta_deg
                if config.max_first_rotation_delta_deg is not None
                else (
                    self._downstream_contract.orientation_ramp_deg_per_tick
                    * config.crossfade_steps
                )
            ),
            max_prefix_velocity_mm_s=(
                config.max_prefix_velocity_mm_s
                if config.max_prefix_velocity_mm_s is not None
                else self._handoff_config.b_predicted_velocity_limit_mm_s
            ),
            max_prefix_acceleration_mm_s2=config.max_prefix_acceleration_mm_s2,
            max_bridge_prefix_velocity_mismatch_mm_s=(
                config.max_bridge_prefix_velocity_mismatch_mm_s
                if config.max_bridge_prefix_velocity_mismatch_mm_s is not None
                else self._handoff_config.handoff_velocity_tolerance_mm_s
            ),
            max_crossfade_xyz_axis_step_mm=(
                config.max_crossfade_xyz_axis_step_mm
                if config.max_crossfade_xyz_axis_step_mm is not None
                else self._downstream_contract.linear_ramp_mm_per_tick
            ),
            max_crossfade_rotation_step_deg=(
                config.max_crossfade_rotation_step_deg
                if config.max_crossfade_rotation_step_deg is not None
                else self._downstream_contract.orientation_ramp_deg_per_tick
            ),
            acknowledged_command_span_steps=(
                1 + config.bridge_max_ack_lag_steps
            ),
            max_acknowledged_xyz_axis_span_mm=(
                self._downstream_contract.linear_ramp_mm_per_tick
            ),
            max_acknowledged_rotation_span_deg=(
                self._downstream_contract.orientation_ramp_deg_per_tick
            ),
            max_crossfade_command_acceleration_mm_s2=(
                config.max_crossfade_command_acceleration_mm_s2
            ),
        )
        compatibility.require_live_thresholds()
        if config.runtime_command_profile_id:
            profile = get_runtime_command_profile(
                config.runtime_command_profile_id
            )
            resolved_profile_values = {
                "max_prefix_velocity_mm_s": (
                    compatibility.max_prefix_velocity_mm_s,
                    profile.cartesian_velocity_limit_mm_s,
                ),
                "axis_velocity_limit_mm_s": (
                    feasibility["axis_velocity_limit_mm_s"],
                    profile.axis_velocity_limit_mm_s,
                ),
                "velocity_limit_mm_s": (
                    feasibility["velocity_limit_mm_s"],
                    profile.cartesian_velocity_limit_mm_s,
                ),
            }
            mismatches = [
                name
                for name, (actual, expected) in resolved_profile_values.items()
                if actual is None
                or not np.isclose(
                    float(actual),
                    float(expected),
                    rtol=0.0,
                    atol=1.0e-9,
                )
            ]
            if mismatches:
                raise ValueError(
                    "resolved runtime command profile mismatch: "
                    + ",".join(mismatches)
                )
        self._v2_runtime_config = HandoffV2Config(
            enabled=True,
            handoff_window_steps=config.handoff_window_steps,
            b_prefix_steps=config.b_prefix_steps,
            crossfade_steps=config.crossfade_steps,
            max_b_result_age_sec=config.max_b_result_age_sec,
            max_inflight_b_requests=config.max_inflight_b_requests,
            request_retry_interval_steps=config.b_request_retry_interval_steps,
            bridge_admission_mode=admission_mode,
            adaptive_b_max_splice_index=config.adaptive_b_max_splice_index,
            adaptive_b_max_candidates=config.adaptive_b_max_candidates,
            enable_soft_handoff=config.enable_soft_handoff,
            enable_endpoint_fallback=config.enable_endpoint_fallback,
            semantic_authority=self._semantic_authority,
            control_hz=self._downstream_contract.control_hz,
            compatibility=compatibility,
        )
        if admission_mode is BridgeAdmissionMode.FLEXIBLE_LEVEL2:
            self._flexible_bridge_search_config = FlexibleBridgeSearchConfig(
                tangent_handle_chord_ratios=tuple(
                    config.flexible_tangent_handle_chord_ratios
                ),
                duration_multipliers=tuple(
                    config.flexible_duration_multipliers
                ),
                enable_settle_connector=(
                    config.flexible_enable_settle_connector
                ),
                enable_alignment_connector=(
                    config.flexible_enable_alignment_connector
                ),
                enable_lift_transport_connector=(
                    config.flexible_enable_lift_transport_connector
                ),
                enable_generic_safe_connector=(
                    config.flexible_enable_generic_safe_connector
                ),
                allow_intermediate_release=config.allow_intermediate_release,
                max_candidates=config.flexible_bridge_max_candidates,
                max_search_time_s=config.flexible_bridge_max_search_time_s,
            )
            self._flexible_bridge_worker = AsyncFlexibleBridgePlanner(
                max_result_age_s=config.flexible_bridge_max_result_age_sec,
                backend=config.flexible_bridge_worker_backend,
                cpu_set=tuple(sorted(parse_cpu_set(config.flexible_bridge_cpu_set))),
                nice_value=config.flexible_bridge_worker_nice,
            )
            worker_info = self._flexible_bridge_worker.start()
            if (
                config.flexible_bridge_worker_backend == "process"
                and worker_info["cpu_set"]
                != sorted(parse_cpu_set(config.flexible_bridge_cpu_set))
            ):
                raise RuntimeError(
                    "Flexible Bridge worker affinity verification failed"
                )
            self._event(
                "flexible_bridge_worker_started",
                worker=worker_info,
                startup_phase="setup_before_live",
                control_thread_blocking=False,
                heavy_search="cached_once_per_edge",
                fresh_work="adaptive_entry_single_candidate",
            )
            self._cache_nominal_flexible_bridge_template(
                self._v2_manifest,
                startup_phase="base_setup_before_live",
            )
        successor = AsyncSuccessorController(
            self._session_b,
            max_result_age_s=config.max_b_result_age_sec,
        )
        evaluator = HandoffCompatibilityEvaluator(
            compatibility,
            prefix_steps=config.b_prefix_steps,
            action_hz=self._downstream_contract.control_hz,
            semantic_authority=self._semantic_authority,
            splice_selector=HandoffSpliceSelector(
                max_splice_index=config.adaptive_b_max_splice_index,
                max_candidates=config.adaptive_b_max_candidates,
            ),
        )
        self._v2_coordinator = TaskCHandoffV2Coordinator(
            manifest=self._v2_manifest,
            config=self._v2_runtime_config,
            successor=successor,
            evaluator=evaluator,
            event_callback=self._on_v2_coordinator_event,
        )
        self._v2_coordinator.mark_policies_loaded(timestamp_s=time.monotonic())

        source_velocity = self._v2_manifest.source.nominal_velocity_mm_s
        speed = float(np.linalg.norm(source_velocity))
        if speed <= 1e-9:
            raise ValueError("selected V2 source exit requires non-zero nominal velocity")
        boundary = self._v2_manifest.source
        if source_mode is SourceTriggerMode.MEDIAN_SPHERE:
            self._representative_boundary_contract = RepresentativeBoundaryContract(
                center_position_mm=boundary.nominal_position_mm,
                representative_velocity_mm_s=source_velocity,
                representative_tangent=source_velocity / speed,
                prearm_radius_mm=boundary.prearm_radius_mm,
                commit_radius_mm=boundary.commit_radius_mm,
                direction_cosine_minimum=boundary.direction_cosine_minimum,
                approach_frames=boundary.approach_frames,
                commit_stable_frames=boundary.commit_stable_frames,
                boundary_id=f"v2:{self._v2_manifest.handoff_id}:source_exit",
            )
            self._representative_boundary_tracker = RepresentativeBoundaryTracker(
                self._representative_boundary_contract
            )
        else:
            assert source_phase_config is not None
            assert preloaded_source_bank is not None
            self._source_phase_bank = preloaded_source_bank
            self._source_phase_tracker = SourcePhaseSupportTracker(
                preloaded_source_bank,
                source_phase_config,
            )
            # Disable only the inherited 40/20 mm authority. The median remains
            # available inside the support bank as a diagnostic metric.
            self._representative_boundary_contract = None
            self._representative_boundary_tracker = None

        self._v2_audit_subscriptions = [
            self._raw_robot._node.create_subscription(
                Float64MultiArray,
                self._downstream_contract.safe_posx_topic,
                self._on_v2_safe_pose,
                50,
            ),
            self._raw_robot._node.create_subscription(
                Float64MultiArray,
                self._downstream_contract.selected_target_topic,
                self._on_v2_selected_pose,
                50,
            ),
        ]
        if config.record_task_c_dataset:
            if ctx.data.dataset is None:
                raise ValueError(
                    "record_task_c_dataset=true requires rollout --dataset.* config"
                )
            task_description = (
                ctx.runtime.cfg.dataset.single_task or ctx.runtime.cfg.task
            )
            self._v2_recorder = AsyncTaskCLeRobotRecorder(
                ctx.data.dataset,
                features=ctx.data.dataset_features,
                ordered_action_keys=ctx.data.ordered_action_keys,
                task_description=task_description,
                queue_size=config.recording_queue_size,
            )
        self._event(
            "task_c_handoff_v2_ready",
            handoff_mode=config.handoff_mode,
            semantic_authority=self._semantic_authority.value,
            semantic_checks_enforced_by_runtime=(
                self._semantic_authority.runtime_semantic_checks_enforced
            ),
            setup_semantic_diagnostics=list(reasons),
            policy_gripper_contract=(
                "guarded_latch"
                if self._semantic_authority.runtime_semantic_checks_enforced
                else "policy_delegated"
            ),
            bridge_ack_mode=config.bridge_ack_mode,
            bridge_max_ack_lag_steps=config.bridge_max_ack_lag_steps,
            runtime_command_profile=(
                None
                if not config.runtime_command_profile_id
                else get_runtime_command_profile(
                    config.runtime_command_profile_id
                ).to_record()
            ),
            episode_manifest=self._v2_manifest.to_record(),
            resolved_compatibility=asdict(compatibility),
            bridge_acceleration_limit_mm_s2=(
                config.bridge_acceleration_limit_mm_s2
            ),
            crossfade_command_acceleration_limit_mm_s2=(
                config.max_crossfade_command_acceleration_mm_s2
            ),
            fixed_rate_hz=self._downstream_contract.control_hz,
            command_path=(
                "policy->/control/lerobot/target_posx->MUX->guard->"
                "ServoL->/vr/commanded_posx"
            ),
            actual_robot_executed=False,
            ik_checked=self._v2_manifest.ik_checked,
            collision_checked=self._v2_manifest.collision_checked,
            source_trigger_mode=config.source_trigger_mode,
            source_phase_support=(
                None
                if self._source_phase_bank is None
                else self._source_phase_bank.record()
            ),
            successor_runtime_phase_gate=False,
            successor_phase_role="metadata_only",
        )

    def _on_v2_coordinator_event(self, event: V2CoordinatorEvent) -> None:
        self._event(
            event.event,
            coordinator_timestamp_s=event.timestamp_s,
            coordinator_state=event.state,
            **event.details,
        )

    def _on_v2_safe_pose(self, message: Float64MultiArray) -> None:
        self._cache_v2_audit_pose("safe", message.data)

    def _on_v2_selected_pose(self, message: Float64MultiArray) -> None:
        self._cache_v2_audit_pose("selected", message.data)

    def _cache_v2_audit_pose(self, kind: str, values: Any) -> None:
        pose = np.asarray(values, dtype=np.float64)
        if pose.shape != (6,) or not np.all(np.isfinite(pose)):
            return
        timestamp_s = time.monotonic()
        if kind == "safe":
            self._v2_safe_pose = pose.copy()
            self._v2_safe_pose_timestamp_s = timestamp_s
        else:
            self._v2_selected_pose = pose.copy()
            self._v2_selected_pose_timestamp_s = timestamp_s
        if self._v2_trace is not None:
            self._v2_trace.emit(
                {
                    "schema": "task_c_live_v2",
                    "record_type": f"downstream_{kind}",
                    "timestamp_s": timestamp_s,
                    "state": (
                        self._v2_coordinator.state.value
                        if self._v2_coordinator is not None
                        else self.phase.value
                    ),
                    f"{kind}_pose_mm_deg": pose.tolist(),
                    "handoff_id": (
                        None
                        if self._v2_manifest is None
                        else self._v2_manifest.handoff_id
                    ),
                }
            )

    def _capture(
        self,
        ctx: Any,
    ) -> tuple[_LiveCapture, np.ndarray, dict[str, Any], dict[str, Any]]:
        now = time.perf_counter()
        if (
            self.phase is not LivePhase.WAITING_FOR_LIVE
            and self._v2_last_capture_start_s is not None
        ):
            duration_s = now - self._v2_last_capture_start_s
            self._v2_control_timing.add(duration_s)
            self._v2_last_control_tick_ms = duration_s * 1000.0
        self._v2_last_capture_start_s = now
        capture, state, raw, processed = super()._capture(ctx)
        self._v2_latest_capture = capture
        self._v2_latest_raw = raw
        self._v2_latest_processed = processed
        if self._v2_recorder is not None:
            self._v2_recorder.raise_if_failed()
        return capture, state, raw, processed

    def _read_only_representative_preplan(
        self,
        capture: _LiveCapture,
        measured_velocity_mm_s: np.ndarray,
    ) -> None:
        """V2 prearm is observation-only; no corridor search runs at 30 Hz."""

        self._representative_preplan_attempts += 1
        flexible = bool(
            self._v2_runtime_config is not None
            and self._v2_runtime_config.bridge_admission_mode
            is BridgeAdmissionMode.FLEXIBLE_LEVEL2
        )
        self._representative_preplan_succeeded = True
        self._event(
            "v2_source_boundary_prearmed",
            read_only=True,
            bridge_generation_deferred_until_commit=not flexible,
            flexible_worker_prearm=flexible,
            measured_velocity_mm_s=measured_velocity_mm_s.tolist(),
            capture_timestamp_s=capture.capture_completed_s,
        )

    def _actual_boundary_semantic(self, state: np.ndarray) -> BoundarySemanticState:
        assert self._v2_manifest is not None
        closed = bool(float(state[12]) >= 0.5)
        source = self._v2_manifest.source.semantic
        return BoundarySemanticState(
            gripper_state="closed" if closed else "open",
            held_object=source.held_object if closed else "none",
            contact_mode=source.contact_mode,
            semantic_state=source.semantic_state,
            entry_preconditions=source.entry_preconditions,
        )

    def _task_a_gripper_target(
        self,
        raw_target: float,
        *,
        open_seen: bool,
    ) -> tuple[float, bool]:
        if self._semantic_authority.runtime_semantic_checks_enforced:
            return super()._task_a_gripper_target(raw_target, open_seen=open_seen)
        target = float(raw_target)
        if not np.isfinite(target):
            raise ValueError("ACT-A gripper target must be finite")
        return target, False

    def _prepare_b_action(
        self,
        action: np.ndarray,
        state: np.ndarray,
    ) -> np.ndarray:
        if self._semantic_authority.runtime_semantic_checks_enforced:
            return super()._prepare_b_action(action, state)
        del state
        values = np.asarray(action, dtype=np.float64).copy()
        if values.shape != (7,) or not np.all(np.isfinite(values)):
            raise ValueError("ACT-B action must be finite shape (7,)")
        return values

    def _send_b_refresh_hold(
        self,
        ctx: Any,
        *,
        actual_pose: np.ndarray,
    ) -> None:
        if self._semantic_authority.runtime_semantic_checks_enforced:
            super()._send_b_refresh_hold(ctx, actual_pose=actual_pose)
            return
        if self._last_command_action is None:
            raise RuntimeError("ACT-B refresh hold lacks a prior command")
        self._send_array(
            ctx,
            self._last_command_action.copy(),
            "ACT-B-REFRESH-HOLD",
            actual_pose=actual_pose,
            enforce_stream_ramp=False,
        )
        self._b_refresh_hold_cycles += 1
        if self._b_refresh_hold_cycles == 1:
            self._event(
                "act_b_refresh_queue_hold_started",
                generation=self._b_refresh_generation,
                b_steps_sent=self._b_steps_sent,
            )

    def _cache_nominal_flexible_bridge_template(
        self,
        manifest: EpisodeHandoffManifest,
        *,
        startup_phase: str,
    ) -> FlexibleBridgeTemplate:
        """Build and cache one edge template before external Live authority."""

        cached = self._flexible_bridge_nominal_template_cache.get(
            manifest.handoff_id
        )
        if cached is not None:
            return cached
        worker = self._flexible_bridge_worker
        search_config = self._flexible_bridge_search_config
        if worker is None or search_config is None:
            raise RuntimeError("FLEXIBLE_LEVEL2 Bridge worker is unavailable")
        if self._v2_runtime_config is None:
            raise RuntimeError("FLEXIBLE_LEVEL2 runtime config is unavailable")

        last_failure = "template_not_requested"
        for attempt in range(
            1, self.v2_config.flexible_bridge_template_max_attempts + 1
        ):
            snapshot = nominal_medoid_bridge_snapshot(
                manifest,
                timestamp_s=time.monotonic(),
            )
            generation = worker.request_template(
                manifest,
                snapshot,
                self._v2_runtime_config,
                self._bridge_runtime_limits(),
                search_config,
                live_mode=True,
            )
            self._event(
                "flexible_bridge_nominal_template_requested",
                template_handoff_id=manifest.handoff_id,
                attempt=attempt,
                max_attempts=(
                    self.v2_config.flexible_bridge_template_max_attempts
                ),
                generation=generation,
                source_support_episode=manifest.source.support_episode,
                source_support_frame=manifest.source.support_frame,
                source_reference_selection_method=(
                    manifest.source_reference_selection_method
                ),
                startup_phase=startup_phase,
                before_live_authorization=True,
                control_thread_active=False,
            )
            if generation is None:
                last_failure = "bridge_worker_busy_during_setup_cache"
                result = None
            else:
                result = worker.wait_for_result(
                    timeout_s=(
                        self.v2_config.flexible_bridge_template_setup_timeout_s
                    )
                )
                last_failure = str(result.failure_reason or result.status)
            search = None if result is None else result.search
            self._event(
                "flexible_bridge_nominal_template_result",
                template_handoff_id=manifest.handoff_id,
                attempt=attempt,
                status=None if result is None else result.status,
                generation=None if result is None else result.generation,
                failure_reason=last_failure,
                candidates_evaluated=(
                    None if search is None else search.candidates_evaluated
                ),
                candidates_hard_passed=(
                    None if search is None else search.candidates_hard_passed
                ),
                rejected_reason_counts=(
                    {} if search is None else dict(search.rejected_reason_counts)
                ),
            )
            if result is None or result.status != "ready":
                continue
            assert search is not None and search.queue is not None
            template = FlexibleBridgeTemplate(
                handoff_id=manifest.handoff_id,
                search=search,
                prepared_from_snapshot_timestamp_s=(
                    search.queue.source_snapshot.timestamp_s
                ),
            )
            self._flexible_bridge_nominal_template_cache[
                manifest.handoff_id
            ] = template
            if (
                self._v2_manifest is not None
                and self._v2_manifest.handoff_id == manifest.handoff_id
            ):
                self._flexible_bridge_template = template
                self._flexible_bridge_template_requested = True
            self._event(
                "flexible_bridge_nominal_template_cached",
                template_handoff_id=manifest.handoff_id,
                attempt=attempt,
                cached_steps=template.queue.steps,
                cached_generator=template.queue.candidate_generator_type,
                heavy_search_latency_ms=search.search_latency_s * 1000.0,
                before_live_authorization=True,
                runtime_rebase_required=True,
            )
            return template

        self._event(
            "flexible_bridge_nominal_template_exhausted",
            template_handoff_id=manifest.handoff_id,
            attempts=self.v2_config.flexible_bridge_template_max_attempts,
            failure_reason=last_failure,
            fail_closed=True,
        )
        raise RuntimeError(
            "nominal medoid Bridge template failed after bounded setup "
            f"attempts: handoff={manifest.handoff_id} reason={last_failure}"
        )


    def _reset_flexible_bridge_runtime(self) -> None:
        """Invalidate an edge generation and discard its cached/rebase state."""

        handoff_id = (
            None
            if self._v2_manifest is None
            else self._v2_manifest.handoff_id
        )
        self._flexible_bridge_template = (
            None
            if handoff_id is None
            else self._flexible_bridge_nominal_template_cache.get(handoff_id)
        )
        self._flexible_bridge_template_requested = (
            self._flexible_bridge_template is not None
        )
        self._flexible_bridge_template_request_attempts = 0
        self._flexible_bridge_template_retry_exhausted_logged = False
        self._flexible_bridge_next_template_request_s = 0.0
        self._flexible_bridge_ready = None
        self._flexible_bridge_next_rebase_request_s = 0.0
        self._flexible_bridge_inflight_splice = None
        self._flexible_bridge_inflight_generation = None
        self._flexible_bridge_inflight_requested_s = None
        self._flexible_bridge_ready_splice = None
        self._flexible_bridge_ready_generation = None
        if self._flexible_bridge_worker is not None:
            self._flexible_bridge_worker.invalidate()

    def _assert_task_a_start(self, state: np.ndarray) -> None:
        """Enter RUN_A only after the existing external-Live start gate passes."""

        super()._assert_task_a_start(state)
        if self._source_phase_tracker is not None:
            self._source_phase_tracker.reset()
        self._source_phase_status = None
        self._source_phase_latency_ms.clear()
        self._source_phase_commit_latched = False
        self._prepared_bridge_commit = None
        self._source_bridge_last_failure = None
        self._reset_flexible_bridge_runtime()
        assert self._v2_coordinator is not None
        self._v2_coordinator.mark_run_a(timestamp_s=time.monotonic())

    def _bridge_runtime_limits(self) -> BridgeRuntimeLimits:
        assert self._coordinator is not None
        planner = self._coordinator.initial_planner
        feasibility = planner.feasibility_config
        return BridgeRuntimeLimits(
            workspace_min_mm=planner.workspace_min_mm,
            workspace_max_mm=planner.workspace_max_mm,
            velocity_limit_mm_s=float(feasibility["velocity_limit_mm_s"]),
            axis_velocity_limit_mm_s=float(
                feasibility["axis_velocity_limit_mm_s"]
            ),
            acceleration_limit_mm_s2=float(
                self.v2_config.bridge_acceleration_limit_mm_s2
            ),
            curvature_limit_per_mm=float(
                feasibility["curvature_limit_per_mm"]
            ),
            jerk_limit_mm_s3=float(feasibility["jerk_limit_mm_s3"]),
            integrated_squared_jerk_limit=float(
                feasibility["integrated_squared_jerk_limit"]
            ),
            backtracking_ratio_limit=float(
                feasibility["backtracking_ratio_limit"]
            ),
            linear_ramp_mm_per_tick=(
                self._downstream_contract.linear_ramp_mm_per_tick
            ),
            orientation_ramp_deg_per_tick=(
                self._downstream_contract.orientation_ramp_deg_per_tick
            ),
            sample_hz=max(
                60.0,
                float(self._coordinator.initial_planner.sampling_config["bridge_sample_hz"]),
            ),
            curvature_epsilon=float(
                self._coordinator.initial_planner.sampling_config[
                    "curvature_epsilon"
                ]
            ),
            ack_pipeline_max_lag_steps=(
                self.v2_config.bridge_max_ack_lag_steps
                if self.v2_config.bridge_ack_mode == "bounded_pipeline"
                else 0
            ),
            workspace_min_limit_enabled=(
                self._downstream_contract.workspace_min_limit_enabled
            ),
        )

    def _bridge_snapshot_context(
        self,
        capture: _LiveCapture,
        state: np.ndarray,
    ) -> _BridgeSnapshotContext:
        """Atomically validate actual/ACK/semantic state for Bridge planning."""

        assert self._v2_manifest is not None
        measured_velocity = self._estimate_actual_boundary_velocity()
        if measured_velocity is None:
            raise RuntimeError("V2 A-exit velocity is unavailable")
        cache = getattr(self._raw_robot, "cache", None)
        if cache is None:
            raise RuntimeError("V2 Bridge lacks commanded_posx cache")
        commanded = cache.require(
            "commanded_posx",
            max_age_sec=self.live_config.downstream_command_max_age_s,
            now=time.monotonic(),
        )
        acknowledged_pose = np.asarray(commanded.value, dtype=np.float64)
        if acknowledged_pose.shape != (6,) or not np.all(
            np.isfinite(acknowledged_pose)
        ):
            raise RuntimeError("V2 acknowledged pose must be finite 6D")
        position_error, orientation_error = pose_delta_metrics(
            state[6:12],
            acknowledged_pose,
        )
        if position_error > self.live_config.actual_tracking_position_tolerance_mm:
            raise RuntimeError("V2 A-exit actual/ack position tracking error")
        if (
            orientation_error
            > self.live_config.actual_tracking_orientation_tolerance_deg
        ):
            raise RuntimeError("V2 A-exit actual/ack orientation tracking error")
        actual_semantic = self._actual_boundary_semantic(state)
        semantic_ok, semantic_reasons = (
            HandoffCompatibilityEvaluator.semantic_compatibility(
                actual_semantic,
                self._v2_manifest.source.semantic,
            )
        )
        if not semantic_ok and self._semantic_authority.runtime_semantic_checks_enforced:
            raise RuntimeError(
                "V2 source semantic state changed at commit: "
                + ",".join(semantic_reasons)
            )
        self._event(
            "source_semantic_commit_assessment",
            semantic_authority=self._semantic_authority.value,
            semantic_checks_enforced_by_runtime=(
                self._semantic_authority.runtime_semantic_checks_enforced
            ),
            semantic_compatible=semantic_ok,
            semantic_diagnostics=list(semantic_reasons),
            actual_gripper_target=1.0 if float(state[12]) >= 0.5 else 0.0,
        )
        snapshot = BridgeRuntimeSnapshot(
            timestamp_s=capture.capture_completed_s,
            actual_pose_mm_deg=state[6:12],
            acknowledged_pose_mm_deg=acknowledged_pose,
            actual_velocity_mm_s=measured_velocity,
            gripper_target=1.0 if float(state[12]) >= 0.5 else 0.0,
        )
        return _BridgeSnapshotContext(
            snapshot=snapshot,
            acknowledged_receive_count=int(commanded.receive_count),
            actual_semantic=actual_semantic,
            actual_to_ack_position_error_mm=position_error,
            actual_to_ack_orientation_error_deg=orientation_error,
        )

    @staticmethod
    def _prepared_from_context(
        bridge: PrecomputedBridgeQueue,
        context: _BridgeSnapshotContext,
        *,
        capture_timestamp_s: float,
    ) -> _PreparedBridgeCommit:
        snapshot = context.snapshot
        return _PreparedBridgeCommit(
            bridge=bridge,
            acknowledged_pose_mm_deg=(
                snapshot.acknowledged_pose_mm_deg.copy()
            ),
            acknowledged_receive_count=context.acknowledged_receive_count,
            measured_velocity_mm_s=snapshot.actual_velocity_mm_s.copy(),
            actual_semantic=context.actual_semantic,
            actual_to_ack_position_error_mm=(
                context.actual_to_ack_position_error_mm
            ),
            actual_to_ack_orientation_error_deg=(
                context.actual_to_ack_orientation_error_deg
            ),
            capture_timestamp_s=capture_timestamp_s,
        )

    def _prepare_bridge_commit(
        self,
        capture: _LiveCapture,
        state: np.ndarray,
    ) -> _PreparedBridgeCommit:
        """Generate the unchanged STRICT bridge before A invalidation."""

        assert self._v2_manifest is not None
        assert self._v2_runtime_config is not None
        if (
            self._v2_runtime_config.bridge_admission_mode
            is BridgeAdmissionMode.FLEXIBLE_LEVEL2
        ):
            raise BridgeGenerationError(["flexible_bridge_worker_required"])
        context = self._bridge_snapshot_context(capture, state)
        snapshot = context.snapshot
        bridge = instantiate_bridge_queue(
            self._v2_manifest,
            snapshot,
            self._v2_runtime_config,
            self._bridge_runtime_limits(),
            clock=time.perf_counter,
        )
        return self._prepared_from_context(
            bridge,
            context,
            capture_timestamp_s=capture.capture_completed_s,
        )

    def _source_command_sequence(self) -> int:
        """Number of source-policy commands already sent in this stage."""

        return int(self._commands_by_phase[LivePhase.ACT_A.value])

    def _source_action_queue_snapshot(
        self,
        capture: _LiveCapture,
        *,
        maximum_steps: int,
    ) -> SourceActionQueueSnapshot | None:
        """Read the initial LeRobot RTC queue without waiting for inference."""

        assert self._v2_manifest is not None
        return snapshot_rtc_source_queue(
            engine=self._engine,
            interpolator=self._interpolator,
            policy_id=self._v2_manifest.source.task,
            command_sequence=self._source_command_sequence(),
            timestamp_s=capture.capture_completed_s,
            action_hz=self._downstream_contract.control_hz,
            maximum_steps=maximum_steps,
        )

    def _build_predictive_source_splice(
        self,
        capture: _LiveCapture,
        context: _BridgeSnapshotContext,
    ) -> PredictiveSourceSplice | None:
        """Project only a small prepared ACT suffix to a future splice point."""

        config = self.v2_config
        queue_snapshot = self._source_action_queue_snapshot(
            capture,
            maximum_steps=config.flexible_source_splice_queue_snapshot_steps,
        )
        if queue_snapshot is None:
            self._event(
                "predictive_source_splice_unavailable",
                reason="source_action_queue_unavailable",
                act_a_queue_invalidated=False,
            )
            return None
        requested_lookahead = predictive_lookahead_steps(
            nominal_latency_s=(
                config.flexible_source_splice_nominal_latency_s
            ),
            observed_latencies_s=tuple(
                self._flexible_bridge_result_wall_latencies_s
            ),
            action_hz=self._downstream_contract.control_hz,
            margin_steps=config.flexible_source_splice_margin_steps,
            minimum_steps=config.flexible_source_splice_min_lookahead_steps,
            maximum_steps=config.flexible_source_splice_max_lookahead_steps,
        )
        # Keep one queued action after the splice. It lets commit-time lineage
        # validation distinguish a still-active queue from an exhausted one.
        lookahead = min(requested_lookahead, len(queue_snapshot.actions) - 1)
        if lookahead < config.flexible_source_splice_min_lookahead_steps:
            self._event(
                "predictive_source_splice_unavailable",
                reason="insufficient_source_queue_horizon",
                requested_lookahead_steps=requested_lookahead,
                available_steps=len(queue_snapshot.actions),
                act_a_queue_invalidated=False,
            )
            return None
        splice = project_predictive_source_splice(
            queue_snapshot,
            context.snapshot,
            lookahead_steps=lookahead,
            linear_ramp_mm_per_tick=(
                self._downstream_contract.linear_ramp_mm_per_tick
            ),
            orientation_ramp_deg_per_tick=(
                self._downstream_contract.orientation_ramp_deg_per_tick
            ),
            velocity_window_steps=(
                config.flexible_source_splice_velocity_window_steps
            ),
        )
        self._event(
            "predictive_source_splice_built",
            splice=splice.record(),
            requested_lookahead_steps=requested_lookahead,
            source_policy_continues=True,
            act_a_queue_invalidated=False,
            planning_authority="prepared_act_queue_only",
            commit_authority="latest_actual_ack_hard_admission",
        )
        return splice

    def _predictive_source_lineage_matches(
        self,
        capture: _LiveCapture,
        splice: PredictiveSourceSplice,
    ) -> tuple[bool, SourceActionQueueSnapshot | None]:
        current = self._source_action_queue_snapshot(capture, maximum_steps=1)
        return (
            current is not None and splice.queue_snapshot.same_lineage(current),
            current,
        )

    def _try_prepare_flexible_bridge(
        self,
        capture: _LiveCapture,
        state: np.ndarray,
        *,
        allow_commit: bool,
    ) -> _PreparedBridgeCommit | None:
        """Poll cached-template/rebase work without blocking the 30 Hz loop."""

        assert self._v2_manifest is not None
        assert self._v2_runtime_config is not None
        worker = self._flexible_bridge_worker
        search_config = self._flexible_bridge_search_config
        if worker is None or search_config is None:
            raise RuntimeError("FLEXIBLE_LEVEL2 Bridge worker is unavailable")

        predictive_enabled = bool(
            self.v2_config.flexible_predictive_source_splice
        )
        result = worker.poll(now_s=capture.capture_completed_s)
        result_splice: PredictiveSourceSplice | None = None
        result_wall_latency_s: float | None = None
        if (
            result.status not in {"idle", "pending"}
            and result.request_kind == "adaptive_rebase"
            and result.generation == self._flexible_bridge_inflight_generation
        ):
            result_splice = self._flexible_bridge_inflight_splice
            if self._flexible_bridge_inflight_requested_s is not None:
                result_wall_latency_s = max(
                    0.0,
                    capture.capture_completed_s
                    - self._flexible_bridge_inflight_requested_s,
                )
                self._flexible_bridge_result_wall_latencies_s.append(
                    result_wall_latency_s
                )
            self._flexible_bridge_inflight_splice = None
            self._flexible_bridge_inflight_generation = None
            self._flexible_bridge_inflight_requested_s = None
        if result.status not in {"idle", "pending"}:
            search_summary = None
            if result.search is not None:
                search_summary = {
                    "valid": result.search.valid,
                    "candidates_evaluated": result.search.candidates_evaluated,
                    "candidates_hard_passed": (
                        result.search.candidates_hard_passed
                    ),
                    "search_latency_ms": result.search.search_latency_s * 1000.0,
                    "selected": (
                        None
                        if result.search.selected is None
                        else result.search.selected.record()
                    ),
                    "rejected_reason_counts": dict(
                        result.search.rejected_reason_counts
                    ),
                    "timed_out": result.search.timed_out,
                    "candidate_diagnostic_count": len(
                        result.search.candidate_diagnostics
                    ),
                }
            self._event(
                "flexible_bridge_worker_result",
                status=result.status,
                generation=result.generation,
                request_kind=result.request_kind,
                stale=result.stale,
                failure_reason=result.failure_reason,
                freshness_applied=(result.request_kind != "template_search"),
                request_to_poll_latency_ms=(
                    None
                    if result_wall_latency_s is None
                    else result_wall_latency_s * 1000.0
                ),
                predictive_source_splice=(
                    None if result_splice is None else result_splice.record()
                ),
                search=search_summary,
            )
        if result.status == "ready":
            assert result.search is not None
            if result.request_kind == "template_search":
                cached_queue = result.search.queue
                assert cached_queue is not None
                self._flexible_bridge_template = FlexibleBridgeTemplate(
                    handoff_id=self._v2_manifest.handoff_id,
                    search=result.search,
                    prepared_from_snapshot_timestamp_s=(
                        cached_queue.source_snapshot.timestamp_s
                    ),
                )
                self._flexible_bridge_ready = None
                self._flexible_bridge_ready_splice = None
                self._flexible_bridge_ready_generation = None
                self._source_bridge_last_failure = None
                self._event(
                    "flexible_bridge_template_cached",
                    prepared_from_snapshot_timestamp_s=(
                        self._flexible_bridge_template.prepared_from_snapshot_timestamp_s
                    ),
                    heavy_search_latency_ms=(
                        result.search.search_latency_s * 1000.0
                    ),
                    cached_steps=cached_queue.steps,
                    cached_generator=(
                        cached_queue.candidate_generator_type
                    ),
                )
            else:
                self._flexible_bridge_ready = result.search
                self._flexible_bridge_ready_splice = result_splice
                self._flexible_bridge_ready_generation = result.generation
        elif result.status in {"failed", "rejected"}:
            self._flexible_bridge_ready = None
            self._flexible_bridge_ready_splice = None
            self._flexible_bridge_ready_generation = None
            self._source_bridge_last_failure = (
                result.failure_reason or result.status
            )
            if result.request_kind == "template_search":
                self._flexible_bridge_template_requested = False
                self._flexible_bridge_next_template_request_s = (
                    capture.capture_completed_s
                    + self.v2_config.flexible_bridge_rebase_retry_interval_s
                )
                exhausted = (
                    self._flexible_bridge_template_request_attempts
                    >= self.v2_config.flexible_bridge_template_max_attempts
                )
                self._flexible_bridge_template_retry_exhausted_logged = (
                    exhausted
                )
                self._event(
                    "flexible_bridge_template_retry_state",
                    attempts=(
                        self._flexible_bridge_template_request_attempts
                    ),
                    max_attempts=(
                        self.v2_config.flexible_bridge_template_max_attempts
                    ),
                    retry_scheduled=not exhausted,
                    exhausted=exhausted,
                    act_a_queue_invalidated=False,
                )
            elif result.request_kind == "adaptive_rebase":
                self._flexible_bridge_next_rebase_request_s = (
                    capture.capture_completed_s
                    + self.v2_config.flexible_bridge_rebase_retry_interval_s
                )

        context = self._bridge_snapshot_context(capture, state)
        ready = self._flexible_bridge_ready
        if ready is not None and ready.queue is not None:
            ready_splice = self._flexible_bridge_ready_splice
            current_sequence = self._source_command_sequence()
            discard_reason: str | None = None
            discard_details: dict[str, Any] = {}
            if predictive_enabled:
                if ready_splice is None:
                    discard_reason = "missing_predictive_source_splice"
                elif current_sequence < ready_splice.target_command_sequence:
                    # The worker finished early. ACT-A continues consuming the
                    # exact lineage used to predict this future boundary.
                    return None
                elif not allow_commit:
                    discard_reason = "source_not_commit_ready_at_predicted_sequence"
                elif current_sequence > (
                    ready_splice.target_command_sequence
                    + self.v2_config.flexible_source_splice_late_tolerance_steps
                ):
                    discard_reason = "predicted_source_sequence_missed"
                elif (
                    capture.capture_completed_s
                    - ready_splice.queue_snapshot.timestamp_s
                    > self.v2_config.flexible_bridge_max_result_age_sec
                ):
                    discard_reason = "predictive_source_snapshot_stale_at_commit"
                else:
                    lineage_matches, current_queue = (
                        self._predictive_source_lineage_matches(
                            capture,
                            ready_splice,
                        )
                    )
                    if not lineage_matches:
                        discard_reason = "source_queue_lineage_changed"
                        discard_details["expected_queue"] = (
                            ready_splice.queue_snapshot.record()
                        )
                        discard_details["current_queue"] = (
                            None if current_queue is None else current_queue.record()
                        )

            if discard_reason is None and allow_commit:
                valid, reasons, metrics = validate_flexible_entry_rebase(
                    ready.queue,
                    context.snapshot,
                    self._bridge_runtime_limits(),
                )
                prediction_metrics: dict[str, float | int | None] = {}
                bridge_for_commit = ready.queue
                if ready_splice is not None:
                    predicted_position_error, predicted_orientation_error = (
                        pose_delta_metrics(
                            context.snapshot.acknowledged_pose_mm_deg,
                            ready_splice.predicted_snapshot.acknowledged_pose_mm_deg,
                        )
                    )
                    prediction_metrics = {
                        "predicted_to_commit_position_error_mm": (
                            predicted_position_error
                        ),
                        "predicted_to_commit_orientation_error_deg": (
                            predicted_orientation_error
                        ),
                        "predicted_to_commit_velocity_error_mm_s": float(
                            np.linalg.norm(
                                context.snapshot.actual_velocity_mm_s
                                - ready_splice.predicted_snapshot.actual_velocity_mm_s
                            )
                        ),
                        "predicted_target_command_sequence": (
                            ready_splice.target_command_sequence
                        ),
                        "actual_commit_command_sequence": current_sequence,
                    }
                self._event(
                    "flexible_bridge_commit_revalidation",
                    valid=valid,
                    failure_reasons=list(reasons),
                    predictive_source_splice=(
                        None
                        if ready_splice is None
                        else ready_splice.record()
                    ),
                    **prediction_metrics,
                    **metrics,
                )
                if valid:
                    if ready_splice is not None:
                        diagnostics = dict(
                            ready.queue.candidate_diagnostics or {}
                        )
                        diagnostics["predictive_source_splice"] = (
                            ready_splice.record()
                        )
                        diagnostics["commit_prediction_metrics"] = (
                            prediction_metrics
                        )
                        # Geometry was generated from the predicted future
                        # boundary. Audit authority is replaced with the actual
                        # commit snapshot that passed hard admission.
                        bridge_for_commit = replace(
                            ready.queue,
                            source_snapshot=context.snapshot,
                            candidate_diagnostics=diagnostics,
                        )
                    self._flexible_bridge_ready = None
                    self._flexible_bridge_ready_splice = None
                    self._flexible_bridge_ready_generation = None
                    self._source_bridge_last_failure = None
                    return self._prepared_from_context(
                        bridge_for_commit,
                        context,
                        capture_timestamp_s=capture.capture_completed_s,
                    )
                discard_reason = ",".join(reasons)

            if discard_reason is not None:
                self._event(
                    "predictive_source_splice_discarded",
                    reason=discard_reason,
                    allow_commit=allow_commit,
                    current_command_sequence=current_sequence,
                    predicted_source_splice=(
                        None
                        if ready_splice is None
                        else ready_splice.record()
                    ),
                    act_a_queue_invalidated=False,
                    **discard_details,
                )
                self._flexible_bridge_ready = None
                self._flexible_bridge_ready_splice = None
                self._flexible_bridge_ready_generation = None
                self._source_bridge_last_failure = discard_reason
                # Replan from a new future suffix immediately on the next
                # bounded poll; never invalidate or stop the source queue.
                self._flexible_bridge_next_rebase_request_s = (
                    capture.capture_completed_s
                )

        if worker.inflight_generation is not None:
            return None

        if (
            self._flexible_bridge_template is None
            and not self._flexible_bridge_template_requested
            and self._flexible_bridge_template_request_attempts
            < self.v2_config.flexible_bridge_template_max_attempts
            and capture.capture_completed_s
            >= self._flexible_bridge_next_template_request_s
        ):
            # Prearm may intentionally begin before the source semantic event
            # (for example, T1 is still closed immediately before O1). The
            # worker may use that early pose/velocity for command-free
            # geometry preparation, but a Bridge must hold the reviewed source
            # semantic gripper state. Otherwise an early CLOSED snapshot can
            # leak into the first post-O1 Bridge commands.
            template_snapshot = replace(
                context.snapshot,
                gripper_target=(
                    self._v2_manifest.source.semantic.gripper_target
                ),
            )
            generation = worker.request_template(
                self._v2_manifest,
                template_snapshot,
                self._v2_runtime_config,
                self._bridge_runtime_limits(),
                search_config,
                live_mode=True,
            )
            if generation is not None:
                self._flexible_bridge_template_requested = True
                self._flexible_bridge_template_request_attempts += 1
            self._event(
                "flexible_bridge_template_requested",
                generation=generation,
                attempt=self._flexible_bridge_template_request_attempts,
                max_attempts=(
                    self.v2_config.flexible_bridge_template_max_attempts
                ),
                snapshot_timestamp_s=context.snapshot.timestamp_s,
                freshness_applied=False,
                bounded_retry=True,
                act_a_queue_invalidated=False,
            )
            return None

        if (
            (allow_commit or predictive_enabled)
            and self._flexible_bridge_template is not None
            and self._flexible_bridge_ready is None
            and capture.capture_completed_s
            >= self._flexible_bridge_next_rebase_request_s
        ):
            predictive_splice = None
            planning_snapshot = replace(
                context.snapshot,
                gripper_target=(
                    self._v2_manifest.source.semantic.gripper_target
                ),
            )
            if predictive_enabled:
                predictive_splice = self._build_predictive_source_splice(
                    capture,
                    context,
                )
                if predictive_splice is None:
                    return None
                planning_snapshot = replace(
                    predictive_splice.predicted_snapshot,
                    gripper_target=(
                        self._v2_manifest.source.semantic.gripper_target
                    ),
                )
                predictive_splice = replace(
                    predictive_splice,
                    predicted_snapshot=planning_snapshot,
                )
            generation = worker.request_rebase(
                self._flexible_bridge_template,
                self._v2_manifest,
                planning_snapshot,
                self._v2_runtime_config,
                self._bridge_runtime_limits(),
                search_config,
            )
            if generation is not None:
                self._flexible_bridge_inflight_splice = predictive_splice
                self._flexible_bridge_inflight_generation = generation
                self._flexible_bridge_inflight_requested_s = (
                    capture.capture_completed_s
                )
                self._flexible_bridge_next_rebase_request_s = (
                    capture.capture_completed_s
                    + self.v2_config.flexible_bridge_rebase_retry_interval_s
                )
            self._event(
                "flexible_bridge_adaptive_rebase_requested",
                generation=generation,
                snapshot_timestamp_s=planning_snapshot.timestamp_s,
                current_actual_ack_snapshot_timestamp_s=(
                    context.snapshot.timestamp_s
                ),
                cached_template_timestamp_s=(
                    self._flexible_bridge_template.prepared_from_snapshot_timestamp_s
                ),
                freshness_applied=True,
                candidates=search_config.adaptive_entry_max_candidates,
                join_strategy=(
                    "actual_ack_projection_to_bounded_future_reference_join"
                ),
                source_splice_mode=(
                    "remaining_act_queue_predictive"
                    if predictive_splice is not None
                    else "current_snapshot_legacy"
                ),
                predictive_source_splice=(
                    None
                    if predictive_splice is None
                    else predictive_splice.record()
                ),
                source_policy_continues=(predictive_splice is not None),
                act_a_queue_invalidated=False,
            )
        return None

    def _representative_source_semantic_override(self, cut_status: Any) -> bool | None:
        if not self._semantic_authority.runtime_semantic_checks_enforced:
            return True
        return super()._representative_source_semantic_override(cut_status)

    def _a_exit_commit_ready(
        self,
        ctx: Any,
        capture: _LiveCapture,
        state: np.ndarray,
        cut_status: Any,
    ) -> bool:
        if (
            SourceTriggerMode(self.v2_config.source_trigger_mode)
            is SourceTriggerMode.MEDIAN_SPHERE
        ):
            return super()._a_exit_commit_ready(
                ctx,
                capture,
                state,
                cut_status,
            )

        assert self._source_phase_tracker is not None
        boundary_velocity = self._estimate_actual_boundary_velocity()
        phase_status = self._source_phase_tracker.update(
            tcp_position_mm=capture.tcp_position_mm,
            tcp_velocity_mm_s=boundary_velocity,
            open_before_close_observed=cut_status.open_seen,
            gripper_closed=capture.semantic_state.gripper_closed,
            semantic_ready=(
                bool(cut_status.ready)
                if self._semantic_authority.runtime_semantic_checks_enforced
                else True
            ),
        )
        self._source_phase_status = phase_status
        self._source_phase_latency_ms.append(phase_status.tracker_latency_ms)
        self._event(
            "source_phase_support_status",
            **phase_status.record(),
            a_nominal_phase=self._v2_manifest.source.phase,
            bridge_feasible=None,
            commit=False,
        )
        if phase_status.just_prearmed:
            self._event(
                "source_phase_prearmed",
                phase_prearm_low=(
                    self._source_phase_tracker.config.prearm_phase_low
                ),
                phase_status=phase_status.record(),
                old_median_sphere_authority=False,
            )
            if boundary_velocity is not None:
                self._read_only_representative_preplan(
                    capture,
                    boundary_velocity,
                )
        if phase_status.deadline_exceeded:
            self._event(
                "source_phase_deadline_fail_closed",
                phase_status=phase_status.record(),
                last_bridge_failure=self._source_bridge_last_failure,
            )
            raise RuntimeError(
                "source phase deadline exceeded before a feasible Bridge commit"
            )
        if phase_status.commit_ready:
            self._source_phase_commit_latched = True
        commit_authorized = bool(
            phase_status.commit_ready
            or (
                self._source_phase_commit_latched
                and phase_status.support_ready
                and phase_status.semantic_ready
                and not phase_status.deadline_exceeded
            )
        )
        if not commit_authorized:
            self._prepared_bridge_commit = None
            if (
                phase_status.prearmed
                and getattr(self, "_v2_runtime_config", None) is not None
                and getattr(self, "_v2_runtime_config").bridge_admission_mode
                is BridgeAdmissionMode.FLEXIBLE_LEVEL2
            ):
                self._try_prepare_flexible_bridge(
                    capture,
                    state,
                    allow_commit=False,
                )
            return False

        try:
            if (
                getattr(self, "_v2_runtime_config", None) is not None
                and getattr(self, "_v2_runtime_config").bridge_admission_mode
                is BridgeAdmissionMode.FLEXIBLE_LEVEL2
            ):
                prepared = self._try_prepare_flexible_bridge(
                    capture,
                    state,
                    allow_commit=True,
                )
                if prepared is None:
                    self._event(
                        "source_phase_flexible_bridge_pending",
                        phase_status=phase_status.record(),
                        bridge_feasible=None,
                        commit=False,
                        act_a_queue_invalidated=False,
                    )
                    return False
            else:
                prepared = self._prepare_bridge_commit(capture, state)
        except BridgeGenerationError as exc:
            self._prepared_bridge_commit = None
            self._source_bridge_last_failure = f"{type(exc).__name__}: {exc}"
            self._event(
                "source_phase_bridge_rejected",
                phase_status=phase_status.record(),
                bridge_feasible=False,
                commit=False,
                reason=self._source_bridge_last_failure,
                act_a_queue_invalidated=False,
            )
            return False

        self._prepared_bridge_commit = prepared
        self._source_bridge_last_failure = None
        self._event(
            "source_phase_commit_ready",
            phase_status=phase_status.record(),
            bridge_feasible=True,
            bridge=prepared.bridge.record(),
            commit=True,
            act_a_queue_invalidated=False,
        )
        return True

    def _v2_command_history_array(self) -> np.ndarray:
        if not self._v2_acknowledged_command_history:
            return np.empty((0, 6), dtype=np.float64)
        return np.stack(tuple(self._v2_acknowledged_command_history), axis=0)

    def _bounded_bridge_ack_pipeline_enabled(self) -> bool:
        # Endpoint fallback intentionally returns to the reviewed V1
        # stop-and-wait behavior. The one-step pipeline is scoped only to the
        # prevalidated V2 queue and its soft-handoff commands.
        return bool(
            self._v2_runtime_active
            and super()._bounded_bridge_ack_pipeline_enabled()
        )

    def _bridge_streamer_acknowledged(self) -> bool:
        acknowledged = super()._bridge_streamer_acknowledged()
        if not acknowledged or not self._v2_runtime_active:
            return acknowledged
        count = self._last_acknowledged_bridge_receive_count
        pose = self._last_acknowledged_bridge_pose
        if count is None or pose is None:
            raise RuntimeError("V2 acknowledgement lacks command sample metadata")
        if (
            self._v2_last_acknowledged_receive_count is None
            or count > self._v2_last_acknowledged_receive_count
        ):
            self._v2_acknowledged_command_history.append(pose.copy())
            self._v2_last_acknowledged_receive_count = count
        return True

    def _begin_bridge(
        self,
        ctx: Any,
        capture: _LiveCapture,
        state: np.ndarray,
    ) -> None:
        assert self._v2_manifest is not None
        assert self._v2_runtime_config is not None
        assert self._v2_coordinator is not None
        prepared = self._prepared_bridge_commit
        if prepared is None:
            # Legacy median-sphere mode reaches this point without phase preplan.
            # It still validates the complete queue before invalidating ACT-A.
            prepared = self._prepare_bridge_commit(capture, state)
        elif prepared.capture_timestamp_s != capture.capture_completed_s:
            raise RuntimeError("prepared V2 Bridge snapshot became stale before commit")

        bridge = prepared.bridge
        acknowledged_pose = prepared.acknowledged_pose_mm_deg
        measured_velocity = prepared.measured_velocity_mm_s
        actual_semantic = prepared.actual_semantic

        # From this point onward the complete actual-state Bridge is feasible.
        # Only now may the active ACT-A generation/queue be invalidated.
        self._v2_coordinator.mark_exit_commit(
            timestamp_s=capture.capture_completed_s
        )
        self._v2_coordinator.mark_prepare_bridge(
            timestamp_s=capture.capture_completed_s
        )
        self._engine.pause()
        self._engine.reset()
        self._prepared_bridge_commit = None

        self.phase = LivePhase.BRIDGE
        self._pending_command = None
        self._last_bridge_commanded_receive_count = None
        self._reset_bridge_ack_tracking(
            acknowledged_pose,
            prepared.acknowledged_receive_count,
        )
        self._v2_acknowledged_command_history.clear()
        self._v2_acknowledged_command_history.append(acknowledged_pose.copy())
        self._v2_last_acknowledged_receive_count = prepared.acknowledged_receive_count
        self._transition_started_s = capture.capture_completed_s
        self._v2_runtime_active = True
        self._v2_coordinator.start_bridge(
            bridge,
            timestamp_s=capture.capture_completed_s,
        )
        command = self._v2_coordinator.tick(
            timestamp_s=capture.capture_completed_s,
            actual_pose_mm_deg=state[6:12],
            command_history_mm_deg=self._v2_command_history_array(),
            actual_semantic=actual_semantic,
            policy_input=capture.policy_input,
            observation_timestamp_s=capture.capture_completed_s,
        )
        self._dispatch_v2_command(ctx, command, actual_pose=state[6:12])
        self._event(
            "act_a_to_precomputed_bridge_v2",
            actual_exit_pose_mm_deg=state[6:12].tolist(),
            acknowledged_exit_pose_mm_deg=acknowledged_pose.tolist(),
            actual_exit_velocity_mm_s=measured_velocity.tolist(),
            actual_to_ack_position_error_mm=(
                prepared.actual_to_ack_position_error_mm
            ),
            actual_to_ack_orientation_error_deg=(
                prepared.actual_to_ack_orientation_error_deg
            ),
            bridge=bridge.record(),
            source_phase=self._v2_manifest.source.phase,
            estimated_a_phase=(
                None
                if self._source_phase_status is None
                else self._source_phase_status.estimated_phase
            ),
            successor_phase=self._v2_manifest.successor.phase,
            successor_phase_role="metadata_only",
            successor_reference_phase_window=(
                self._v2_manifest.successor_reference_phase_window
            ),
            successor_interior_path_margin_mm=(
                self._v2_manifest.successor_interior_path_margin_mm
            ),
            semantic_local_total_length_mm=(
                self._v2_manifest.semantic_local_total_length_mm
            ),
            successor_runtime_phase_gate=False,
            bridge_feasible_before_a_invalidation=True,
        )

    def _step_bridge(
        self,
        ctx: Any,
        capture: _LiveCapture,
        state: np.ndarray,
    ) -> None:
        if not self._v2_runtime_active:
            super()._step_bridge(ctx, capture, state)
            return
        assert self._v2_coordinator is not None
        command = self._v2_coordinator.tick(
            timestamp_s=capture.capture_completed_s,
            actual_pose_mm_deg=state[6:12],
            command_history_mm_deg=self._v2_command_history_array(),
            actual_semantic=self._actual_boundary_semantic(state),
            policy_input=capture.policy_input,
            observation_timestamp_s=capture.capture_completed_s,
        )
        if command.endpoint_fallback_required:
            self._start_endpoint_v1_fallback(ctx, capture, state, command)
            return
        if command.failure_reason is not None and command.action is None:
            raise RuntimeError(f"V2 handoff failed: {command.failure_reason}")
        self._dispatch_v2_command(ctx, command, actual_pose=state[6:12])

    def _dispatch_v2_command(
        self,
        ctx: Any,
        command: HandoffCommand,
        *,
        actual_pose: np.ndarray,
    ) -> None:
        if command.action is None:
            return
        self._v2_command_meta = command
        self._submit_live_command(
            ctx,
            command.action,
            "BEZIER_BRIDGE",
            actual_pose=actual_pose,
            enforce_stream_ramp=True,
            transition_to_act_b=command.transition_to_b_after_commit,
        )

    def _commit_live_command(
        self,
        ctx: Any,
        pending: _PendingLiveCommand,
        *,
        actual_pose: np.ndarray,
    ) -> None:
        if pending.transition_to_act_b and self._v2_runtime_active:
            safe_pending = replace(pending, transition_to_act_b=False)
            super()._commit_live_command(
                ctx,
                safe_pending,
                actual_pose=actual_pose,
            )
            self.phase = LivePhase.ACT_B
            self._b_started_s = time.monotonic()
            self._v2_runtime_active = False
            self._v2_full_takeover_timestamp_s = self._b_started_s
            self._event(
                "soft_handoff_to_act_b_committed",
                generation=(
                    None
                    if self._session_b is None
                    else self._session_b.active_generation
                ),
                actual_pose_mm_deg=np.asarray(actual_pose).tolist(),
                crossfade_steps=self.v2_config.crossfade_steps,
            )
            self._v2_command_meta = None
            return
        super()._commit_live_command(ctx, pending, actual_pose=actual_pose)
        if self._pending_command is None:
            self._v2_command_meta = None

    def _start_endpoint_v1_fallback(
        self,
        ctx: Any,
        capture: _LiveCapture,
        state: np.ndarray,
        command: HandoffCommand,
    ) -> None:
        """Re-enter the existing V1 endpoint-stop/fresh-B coordinator safely."""

        assert self._coordinator is not None
        assert self._handoff_config is not None
        cache = getattr(self._raw_robot, "cache", None)
        if cache is None:
            raise RuntimeError("V1 fallback lacks commanded_posx cache")
        sample = cache.require(
            "commanded_posx",
            max_age_sec=self.live_config.downstream_command_max_age_s,
            now=time.monotonic(),
        )
        acknowledged = np.asarray(sample.value, dtype=np.float64)
        measured_velocity = self._estimate_actual_boundary_velocity()
        if measured_velocity is None:
            raise RuntimeError("V1 fallback actual velocity is unavailable")
        self._coordinator.history.clear()
        positions, timestamps = self._actual_history.arrays(
            self._handoff_config.velocity_window_frames
        )
        for position, timestamp in zip(positions, timestamps, strict=True):
            self._coordinator.history.add(float(timestamp), position)
        self._coordinator.phase = RuntimePhase.A_RUNNING
        fallback_observation = RuntimeObservation(
            timestamp_s=capture.capture_completed_s,
            tcp_position_mm=capture.tcp_position_mm,
            semantic_state=capture.semantic_state,
            policy_input=capture.policy_input,
            acknowledged_command_position_mm=acknowledged[:3],
            acknowledged_command_velocity_mm_s=measured_velocity,
        )
        plan = self._coordinator.request_cut(
            fallback_observation,
            semantic_cut_valid=True,
            a_retained_length_mm=self._a_path_length_mm,
        )
        self._orientation_bridge = OrientationBridge(
            acknowledged[3:6],
            acknowledged[3:6],
            duration_s=plan.bridge.duration_s,
        )
        self._assess_bridge_passthrough(
            plan.bridge,
            acknowledged[3:6],
            acknowledged[3:6],
        )
        self._v2_runtime_active = False
        self._v2_endpoint_fallback_used = True
        self._pending_command = None
        self._last_bridge_commanded_receive_count = None
        self._reset_bridge_ack_tracking(
            acknowledged,
            int(sample.receive_count),
        )
        self._transition_started_s = capture.capture_completed_s
        initial_hold = full_action(
            plan.bridge.p0,
            acknowledged[3:6],
            gripper_target=1.0 if float(state[12]) >= 0.5 else 0.0,
        )
        # The fallback command still traverses the exact existing V1 send path.
        self._send_array(
            ctx,
            initial_hold,
            "BEZIER_BRIDGE",
            actual_pose=state[6:12],
        )
        self._event(
            "endpoint_v1_fallback_started",
            v2_failure_reason=command.failure_reason,
            fallback_bridge_duration_s=plan.bridge.duration_s,
            fallback_bridge_endpoint_mm=plan.bridge.p3.tolist(),
            fallback_uses_existing_coordinator=True,
        )

    def _send_array(
        self,
        ctx: Any,
        action: np.ndarray,
        source: str,
        *,
        actual_pose: np.ndarray,
        enforce_stream_ramp: bool = True,
    ) -> None:
        super()._send_array(
            ctx,
            action,
            source,
            actual_pose=actual_pose,
            enforce_stream_ramp=enforce_stream_ramp,
        )
        values = np.asarray(action, dtype=np.float64)
        meta = self._v2_command_meta
        meta_matches = bool(
            meta is not None
            and meta.action is not None
            and np.allclose(meta.action, values, atol=1e-12, rtol=0.0)
        )
        teacher_stage = (
            meta.source
            if meta_matches
            else (
                "V2_TRACKING_HOLD"
                if self._v2_runtime_active and self.phase is LivePhase.BRIDGE
                else source
            )
        )
        self._record_v2_primary_action(values)
        self._trace_v2_command(
            values,
            actual_pose=np.asarray(actual_pose, dtype=np.float64),
            teacher_stage=teacher_stage,
            meta=meta if meta_matches else None,
        )

    def _send_next_a_action(self, ctx: Any, **kwargs: Any) -> dict[str, float] | None:
        result = super()._send_next_a_action(ctx, **kwargs)
        if result is None:
            return None
        values = np.asarray(
            [result[key] for key in ctx.data.ordered_action_keys],
            dtype=np.float64,
        )
        self._record_v2_primary_action(values)
        self._trace_v2_command(
            values,
            actual_pose=np.asarray(kwargs["actual_pose"], dtype=np.float64),
            teacher_stage="ACT-A",
            meta=None,
        )
        return result

    def _record_v2_primary_action(self, action: np.ndarray) -> None:
        if self._v2_recorder is None:
            return
        if self._v2_latest_processed is None:
            raise RuntimeError("Task-C recording lacks the latest observation")
        snapshot_latency_ms = self._v2_recorder.enqueue(
            self._v2_latest_processed,
            action,
        )
        if self._v2_trace is not None:
            self._v2_trace.emit(
                {
                    "schema": "task_c_live_v2",
                    "record_type": "recording_enqueue",
                    "timestamp_s": time.monotonic(),
                    "state": self.phase.value,
                    "handoff_id": self._v2_manifest.handoff_id,
                    "record_snapshot_latency_ms": snapshot_latency_ms,
                }
            )

    def _trace_v2_command(
        self,
        action: np.ndarray,
        *,
        actual_pose: np.ndarray,
        teacher_stage: str,
        meta: HandoffCommand | None,
    ) -> None:
        if self._v2_trace is None or self._v2_manifest is None:
            return
        commanded = None
        cache = getattr(self._raw_robot, "cache", None)
        if cache is not None:
            sample = cache.sample("commanded_posx")
            if sample is not None:
                commanded = np.asarray(sample.value, dtype=np.float64).tolist()
        self._v2_trace.emit(
            {
                "schema": "task_c_live_v2",
                "record_type": "control_command",
                "timestamp_s": time.monotonic(),
                "state": (
                    self._v2_coordinator.state.value
                    if self._v2_coordinator is not None
                    else self.phase.value
                ),
                "teacher_stage": teacher_stage,
                "handoff_id": self._v2_manifest.handoff_id,
                "source_phase": self._v2_manifest.source.phase,
                "estimated_a_phase": (
                    None
                    if self._source_phase_status is None
                    else self._source_phase_status.estimated_phase
                ),
                "distance_to_a_component_median_mm": (
                    None
                    if self._source_phase_status is None
                    else (
                        self._source_phase_status
                        .distance_to_component_median_mm
                    )
                ),
                "distance_to_a_nearest_actual_support_mm": (
                    None
                    if self._source_phase_status is None
                    else (
                        self._source_phase_status
                        .distance_to_nearest_actual_support_mm
                    )
                ),
                "nearest_a_support_episode": (
                    None
                    if self._source_phase_status is None
                    else (
                        self._source_phase_status
                        .nearest_actual_support_episode
                    )
                ),
                "successor_phase": self._v2_manifest.successor.phase,
                "successor_phase_role": "metadata_only",
                "successor_reference_phase_window": (
                    self._v2_manifest.successor_reference_phase_window
                ),
                "successor_interior_path_margin_mm": (
                    self._v2_manifest.successor_interior_path_margin_mm
                ),
                "semantic_local_total_length_mm": (
                    self._v2_manifest.semantic_local_total_length_mm
                ),
                "successor_runtime_phase_gate": False,
                "action": action.tolist(),
                "actual_pose_mm_deg": actual_pose.tolist(),
                "policy_proposal_pose_mm_deg": action[:6].tolist(),
                "safe_pose_mm_deg": (
                    None
                    if self._v2_safe_pose is None
                    else self._v2_safe_pose.tolist()
                ),
                "safe_pose_timestamp_s": self._v2_safe_pose_timestamp_s,
                "commanded_pose_mm_deg": commanded,
                "bridge_index": None if meta is None else meta.bridge_index,
                "bridge_progress": None if meta is None else meta.bridge_progress,
                "handoff_window_progress": (
                    None if meta is None else meta.handoff_window_progress
                ),
                "crossfade_weight": (
                    None if meta is None else meta.crossfade_weight
                ),
                "successor_generation": (
                    None if meta is None else meta.successor_generation
                ),
                "control_tick_ms": self._v2_last_control_tick_ms,
            }
        )

    def _step_b(
        self,
        ctx: Any,
        capture: _LiveCapture,
        state: np.ndarray,
    ) -> None:
        super()._step_b(ctx, capture, state)
        if (
            self.phase is LivePhase.COMPLETE
            and self._v2_coordinator is not None
            and self._v2_coordinator.state is HandoffV2State.RUN_B
        ):
            self._v2_coordinator.mark_complete(
                timestamp_s=capture.capture_completed_s
            )

    def _fail(self, reason: str) -> None:
        if self._v2_coordinator is not None:
            self._v2_coordinator.fail_closed(
                timestamp_s=time.monotonic(),
                reason=reason,
            )
        super()._fail(reason)

    def run(self, ctx: Any) -> None:
        assert self._v2_coordinator is not None
        super().run(ctx)

    def _episode_summary(self, recording_stats: Any) -> dict[str, Any]:
        manifest = self._v2_manifest
        coordinator = self._v2_coordinator
        bridge = None if coordinator is None else coordinator.bridge

        def latest(event_name: str) -> V2CoordinatorEvent | None:
            if coordinator is None:
                return None
            return next(
                (
                    event
                    for event in reversed(coordinator.events)
                    if event.event == event_name
                ),
                None,
            )

        request = latest("act_b_shadow_requested")
        result = latest("act_b_shadow_result")
        crossfade = latest("soft_handoff_started")
        takeover = latest("act_b_full_takeover")
        timing = self._v2_control_timing.summary()
        phase_latency = np.asarray(
            self._source_phase_latency_ms,
            dtype=np.float64,
        )
        phase_timing = {
            "samples": int(phase_latency.size),
            "p50_ms": (
                None
                if phase_latency.size == 0
                else float(np.percentile(phase_latency, 50))
            ),
            "p95_ms": (
                None
                if phase_latency.size == 0
                else float(np.percentile(phase_latency, 95))
            ),
            "p99_ms": (
                None
                if phase_latency.size == 0
                else float(np.percentile(phase_latency, 99))
            ),
            "max_ms": (
                None
                if phase_latency.size == 0
                else float(np.max(phase_latency))
            ),
        }
        source_splice_latency = np.asarray(
            self._flexible_bridge_result_wall_latencies_s,
            dtype=np.float64,
        )
        source_splice_timing = {
            "samples": int(source_splice_latency.size),
            "p50_ms": (
                None
                if source_splice_latency.size == 0
                else float(np.percentile(source_splice_latency, 50) * 1000.0)
            ),
            "p95_ms": (
                None
                if source_splice_latency.size == 0
                else float(np.percentile(source_splice_latency, 95) * 1000.0)
            ),
            "p99_ms": (
                None
                if source_splice_latency.size == 0
                else float(np.percentile(source_splice_latency, 99) * 1000.0)
            ),
            "max_ms": (
                None
                if source_splice_latency.size == 0
                else float(np.max(source_splice_latency) * 1000.0)
            ),
        }
        return {
            "handoff_id": None if manifest is None else manifest.handoff_id,
            "semantic_authority": self._semantic_authority.value,
            "semantic_checks_enforced_by_runtime": (
                self._semantic_authority.runtime_semantic_checks_enforced
            ),
            "source_phase": None if manifest is None else manifest.source.phase,
            "successor_phase": (
                None if manifest is None else manifest.successor.phase
            ),
            "successor_phase_role": "metadata_only",
            "successor_runtime_phase_gate": False,
            "successor_reference": {
                "selection_method": (
                    None
                    if manifest is None
                    else manifest.successor_reference_selection_method
                ),
                "phase_window": (
                    None
                    if manifest is None
                    else manifest.successor_reference_phase_window
                ),
                "interior_path_margin_mm": (
                    None
                    if manifest is None
                    else manifest.successor_interior_path_margin_mm
                ),
                "semantic_local_total_length_mm": (
                    None
                    if manifest is None
                    else manifest.semantic_local_total_length_mm
                ),
            },
            "source_trigger": {
                "mode": self.v2_config.source_trigger_mode,
                "last_status": (
                    None
                    if self._source_phase_status is None
                    else self._source_phase_status.record()
                ),
                "support_bank": (
                    None
                    if self._source_phase_bank is None
                    else self._source_phase_bank.record()
                ),
                "phase_tracker_timing": phase_timing,
                "last_bridge_failure": self._source_bridge_last_failure,
                "bridge_feasible_before_a_invalidation": bridge is not None,
            },
            "predictive_source_splice": {
                "enabled": self.v2_config.flexible_predictive_source_splice,
                "nominal_worker_latency_s": (
                    self.v2_config.flexible_source_splice_nominal_latency_s
                ),
                "lookahead_steps": [
                    self.v2_config.flexible_source_splice_min_lookahead_steps,
                    self.v2_config.flexible_source_splice_max_lookahead_steps,
                ],
                "margin_steps": (
                    self.v2_config.flexible_source_splice_margin_steps
                ),
                "late_tolerance_steps": (
                    self.v2_config.flexible_source_splice_late_tolerance_steps
                ),
                "worker_result_timing": source_splice_timing,
                "remaining_inflight": (
                    self._flexible_bridge_inflight_generation is not None
                ),
            },
            "actual_A_exit_pose_mm_deg": (
                None
                if bridge is None
                else bridge.source_snapshot.actual_pose_mm_deg.tolist()
            ),
            "actual_A_exit_velocity_mm_s": (
                None
                if bridge is None
                else bridge.source_snapshot.actual_velocity_mm_s.tolist()
            ),
            "bridge_duration_s": (
                None if bridge is None else bridge.bridge.duration_s
            ),
            "bridge_executed_steps": (
                0 if coordinator is None else coordinator.bridge_index
            ),
            "handoff_window_start_step": (
                None if bridge is None else bridge.handoff_window_start_index
            ),
            "B_inference": {
                "request_timestamp_s": (
                    None if request is None else request.timestamp_s
                ),
                "result_timestamp_s": (
                    None if result is None else result.timestamp_s
                ),
                "result": None if result is None else result.details,
            },
            "B_prefix": (
                None
                if coordinator is None or coordinator.last_admission is None
                else coordinator.last_admission.record()
            ),
            "crossfade": {
                "started": crossfade is not None,
                "start_timestamp_s": (
                    None if crossfade is None else crossfade.timestamp_s
                ),
                "steps": self.v2_config.crossfade_steps,
                "takeover_timestamp_s": (
                    None if takeover is None else takeover.timestamp_s
                ),
                "takeover_bridge_progress": (
                    None
                    if takeover is None
                    else takeover.details.get("bridge_progress")
                ),
            },
            "terminal_phase": self.phase.value,
            "terminal_v2_state": (
                None if coordinator is None else coordinator.state.value
            ),
            "B_takeover_success": self._v2_full_takeover_timestamp_s is not None,
            "B_completion": {
                "mode": self.v2_config.b_completion_mode,
                "release_confirmed": self._b_release_confirmed,
                "automatic_completion_enabled": (
                    self.v2_config.b_completion_mode
                    != "successor_owned"
                ),
                "completion_authority": (
                    "external_operator_or_planner"
                    if self.v2_config.b_completion_mode == "successor_owned"
                    else "legacy_release_and_settle_heuristic"
                ),
            },
            "fallback_used": self._v2_endpoint_fallback_used,
            "failure_reasons": list(self._failure_reasons),
            "control_timing": asdict(timing),
            "recording": (
                None if recording_stats is None else asdict(recording_stats)
            ),
            "commands_by_phase": dict(self._commands_by_phase),
            "ik_checked": None if manifest is None else manifest.ik_checked,
            "collision_checked": (
                None if manifest is None else manifest.collision_checked
            ),
        }

    def teardown(self, ctx: Any) -> None:
        recording_stats = None
        errors: list[BaseException] = []
        if self._flexible_bridge_worker is not None:
            try:
                self._flexible_bridge_worker.close()
            except BaseException as exc:
                errors.append(exc)
            finally:
                self._flexible_bridge_worker = None
        if self._v2_recorder is not None:
            try:
                recording_stats = self._v2_recorder.close(
                    save=self.phase is LivePhase.COMPLETE
                )
            except BaseException as exc:
                errors.append(exc)
        try:
            super().teardown(ctx)
        except BaseException as exc:
            errors.append(exc)
        if self._v2_trace is not None:
            try:
                self._v2_trace.close(
                    summary=self._episode_summary(recording_stats)
                )
            except BaseException as exc:
                errors.append(exc)
            finally:
                self._v2_trace = None
        if errors:
            detail = "; ".join(
                f"{type(error).__name__}: {error}" for error in errors
            )
            raise RuntimeError(f"Task-C V2 teardown failed: {detail}") from errors[0]


_V2_FACTORY_INSTALLED = False


def install_task_c_live_v2_strategy() -> bool:
    global _V2_FACTORY_INSTALLED
    if _V2_FACTORY_INSTALLED:
        return False
    import lerobot.rollout as rollout_package
    import lerobot.rollout.strategies as strategies_package
    from lerobot.rollout.strategies import factory

    original = factory.create_strategy

    def create_strategy(config: RolloutStrategyConfig) -> RolloutStrategy:
        if config.type == "task_c_live_v2":
            if not isinstance(config, TaskCLiveV2StrategyConfig):
                raise TypeError("task_c_live_v2 config registration mismatch")
            return TaskCLiveV2Strategy(config)
        return original(config)

    factory.create_strategy = create_strategy
    strategies_package.create_strategy = create_strategy
    rollout_package.create_strategy = create_strategy
    _V2_FACTORY_INSTALLED = True
    return True
