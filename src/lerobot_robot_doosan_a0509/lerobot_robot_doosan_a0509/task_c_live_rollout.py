"""Bounded command-producing A -> Cartesian Bridge -> B rollout strategy.

The strategy reuses LeRobot's continuously refreshed ACT-A RTC/FIFO engine,
pauses and clears it at one semantic cut, executes a full-pose Cartesian
bridge through the existing A0509 robot adapter/MUX path, then consumes only a
fresh resident ACT-B chunk.  External MUX/Live ownership is deliberately not
armed here; ``scripts/a0509_task_c_live_trial_gate.py`` owns that authority and
forces Live OFF on completion, failure, or operator interruption.
"""

from __future__ import annotations

import copy
import json
import logging
import time
from collections import deque
from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path
from typing import Any

import numpy as np
from lerobot.rollout.configs import RolloutStrategyConfig
from lerobot.rollout.strategies.core import RolloutStrategy
from lerobot.utils.constants import OBS_STR
from lerobot.utils.feature_utils import build_dataset_frame
from lerobot.utils.robot_utils import precise_sleep
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from offline_tools.task_c_bridge_v0.bridge_optimizer import NoFeasibleBridgeError
from offline_tools.task_c_bridge_v0.lerobot_act_backend import LeRobotACTBackend
from offline_tools.task_c_bridge_v0.live_transition import (
    BridgeStreamAssessment,
    CutTriggerConfig,
    DownstreamControlContract,
    LiveCutTrigger,
    OrientationBridge,
    assess_bridge_stream_contract,
    assess_policy_pose_chunk,
    full_action,
    pose_delta_metrics,
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
    PolicyInferenceError,
)
from offline_tools.task_c_bridge_v0.trajectory_states import SemanticState
from offline_tools.task_c_bridge_v0.velocity_estimation import RuntimeVelocityHistory
from offline_tools.task_c_bridge_v1.runtime_boundary import (
    RepresentativeBoundaryContract,
    RepresentativeBoundaryStatus,
    RepresentativeBoundaryTracker,
)

from quest_a0509_teleop.doosan_orientation import (
    doosan_zyz_deg_to_quaternion,
    quaternion_slerp,
    quaternion_to_doosan_zyz_deg,
)
from lerobot_robot_doosan_a0509.act_async_rollout import (
    get_act_gpu_arbiter,
    policy_live_queue_ready,
)
from lerobot_robot_doosan_a0509.runtime_scheduling import (
    preserve_current_thread_affinity,
)
from lerobot_robot_doosan_a0509.doosan_a0509_ros import ACTION_KEYS
from lerobot_robot_doosan_a0509.task_c_shadow_rollout import (
    _LiveCapture,
    _policy_input_from_processed,
    _same_checkpoint,
)


logger = logging.getLogger(__name__)


class LivePhase(str, Enum):
    WAITING_FOR_LIVE = "WAITING_FOR_LIVE"
    ACT_A = "ACT_A"
    BRIDGE = "BRIDGE"
    ACT_B = "ACT_B"
    COMPLETE = "COMPLETE"
    FAILED_HOLD = "FAILED_HOLD"


@dataclass
class _PendingLiveCommand:
    action: np.ndarray
    source: str
    enforce_stream_ramp: bool
    transition_to_act_b: bool
    queued_s: float
    wait_cycles: int
    max_candidate_position_error_mm: float
    max_candidate_orientation_error_deg: float


@dataclass(frozen=True)
class _BridgeSentCommand:
    sequence: int
    pose_mm_deg: np.ndarray
    receive_count_before_send: int


@RolloutStrategyConfig.register_subclass("task_c_live")
@dataclass
class TaskCLiveStrategyConfig(RolloutStrategyConfig):
    runtime_manifest: str = ""
    checkpoint_b: str = ""
    event_jsonl_path: str = ""
    acknowledge_uncertified_manifest: bool = False
    # V0 remains the default. This must be explicitly enabled together with a
    # V1 representative-boundary manifest.
    representative_boundary_enabled: bool = False
    cut_open_stable_frames: int = 3
    cut_closed_stable_frames: int = 15
    cut_post_close_delay_s: float = 1.0
    cut_transport_z_min_mm: float = 400.0
    cut_timeout_s: float = 25.0
    a_gripper_close_latch_threshold: float = 0.7
    transition_timeout_s: float = 12.0
    # Watchdogs only. Successful completion is semantic, never a fixed tick count.
    b_execution_steps: int = 900
    b_execution_timeout_s: float = 30.0
    b_refresh_queue_threshold: int = 20
    b_refresh_overlap_steps: int = 15
    b_refresh_timeout_s: float = 1.0
    b_refresh_max_observation_age_s: float = 0.5
    b_release_open_threshold: float = 0.3
    b_release_stable_steps: int = 3
    b_release_driver_timeout_s: float = 5.0
    b_release_observation_stable_frames: int = 3
    # Legacy absolute demonstration envelopes are retained only for explicit
    # baseline reproduction. They are disabled by default so ACT-B's fresh
    # policy output owns release timing and post-release motion.
    b_release_position_gate_enabled: bool = False
    b_release_workspace_min_xyz_mm: tuple[float, float, float] = (
        389.0,
        -228.0,
        280.0,
    )
    b_release_workspace_max_xyz_mm: tuple[float, float, float] = (
        486.0,
        -125.0,
        322.0,
    )
    # ``legacy_release_settle`` preserves the reviewed V0/V1 automatic
    # completion contract. ``successor_owned`` never interprets OPEN or a
    # transient stop as task completion: ACT-B keeps rolling until an external
    # operator/planner ends the run. Safety time/step watchdogs remain active.
    b_completion_mode: str = "legacy_release_settle"
    b_completion_position_gate_enabled: bool = False
    b_completion_workspace_min_xyz_mm: tuple[float, float, float] = (
        350.0,
        -70.0,
        430.0,
    )
    b_completion_workspace_max_xyz_mm: tuple[float, float, float] = (
        446.0,
        45.0,
        496.0,
    )
    b_completion_stable_frames: int = 15
    b_completion_velocity_tolerance_mm_s: float = 15.0
    b_final_refresh_lead_s: float = 0.5
    b_endpoint_stop_before_final_refresh: bool = True
    b_moving_overlap_primary_enabled: bool = False
    b_endpoint_settle_position_tolerance_mm: float = 3.0
    b_endpoint_settle_velocity_tolerance_mm_s: float = 15.0
    b_endpoint_settle_min_hold_s: float = 0.1
    b_endpoint_settle_timeout_s: float = 3.0
    b_overlap_search_max_skip_steps: int = 15
    b_stopped_endpoint_direct_handoff_enabled: bool = False
    b_stopped_endpoint_position_bridge_enabled: bool = False
    b_tail_duration_min_s: float = 0.2
    b_tail_duration_max_s: float = 2.5
    b_tail_duration_step_s: float = 0.1
    start_position_center_mm: tuple[float, float, float] = (
        427.3434661865234,
        0.565598671634992,
        458.2581522623698,
    )
    start_position_radius_mm: float = 35.0
    start_orientation_center_zyz_deg: tuple[float, float, float] = (
        0.10490300878882408,
        149.98989868164062,
        0.12113751471042633,
    )
    start_orientation_radius_deg: float = 6.0
    first_action_position_limit_mm: float = 50.0
    first_action_orientation_limit_deg: float = 10.0
    b_first_orientation_jump_limit_deg: float = 20.0
    b_predicted_angular_velocity_limit_deg_s: float = 180.0
    handoff_orientation_tolerance_deg: float = 5.0
    actual_tracking_position_tolerance_mm: float = 50.0
    actual_tracking_orientation_tolerance_deg: float = 10.0
    actual_tracking_position_admission_margin_mm: float = 0.0
    actual_tracking_orientation_admission_margin_deg: float = 1.0
    downstream_control_hz: float = 30.0
    downstream_linear_ramp_mm_per_tick: float = 6.67
    downstream_orientation_ramp_deg_per_tick: float = 1.0
    downstream_servol_time_s: float = 0.1
    downstream_servol_use_auto_velocity_acceleration: bool = True
    downstream_lerobot_timeout_s: float = 0.3
    downstream_command_max_age_s: float = 0.2
    downstream_ack_position_tolerance_mm: float = 0.05
    downstream_ack_orientation_tolerance_deg: float = 0.02
    # ``stop_and_wait`` preserves the reviewed V0/V1 behavior. The bounded
    # pipeline permits one command of decision-time ACK lag while retaining a
    # fresh downstream-command check and the existing ServoL ramp contract.
    bridge_ack_mode: str = "stop_and_wait"
    bridge_max_ack_lag_steps: int = 0
    downstream_lerobot_target_topic: str = "/control/lerobot/target_posx"
    downstream_selected_target_topic: str = "/vr/target_posx"
    downstream_safe_posx_topic: str = "/vr/safe_posx"
    downstream_commanded_posx_topic: str = "/vr/commanded_posx"
    downstream_workspace_min_xyz_mm: tuple[float, float, float] = (
        50.0,
        -350.0,
        0.0,
    )
    downstream_workspace_min_limit_enabled: tuple[bool, bool, bool] = (
        True,
        True,
        False,
    )
    downstream_workspace_max_xyz_mm: tuple[float, float, float] = (
        650.0,
        350.0,
        600.0,
    )
    downstream_orientation_limit_deg: float = 90.0

    def __post_init__(self) -> None:
        if not isinstance(self.representative_boundary_enabled, bool):
            raise ValueError("representative_boundary_enabled must be boolean")
        if self.cut_timeout_s <= 0.0 or self.transition_timeout_s <= 0.0:
            raise ValueError("Task-C live timeouts must be positive")
        if (
            not isinstance(self.b_execution_steps, int)
            or isinstance(self.b_execution_steps, bool)
            or self.b_execution_steps < 1
            or self.b_execution_steps > 2000
        ):
            raise ValueError("b_execution_steps watchdog must be in [1, 2000]")
        if (
            not isinstance(self.b_refresh_queue_threshold, int)
            or isinstance(self.b_refresh_queue_threshold, bool)
            or not 2 <= self.b_refresh_queue_threshold < 100
        ):
            raise ValueError("b_refresh_queue_threshold must be in [2, 99]")
        if (
            not isinstance(self.b_refresh_overlap_steps, int)
            or isinstance(self.b_refresh_overlap_steps, bool)
            or not 0 <= self.b_refresh_overlap_steps
            <= self.b_refresh_queue_threshold
        ):
            raise ValueError(
                "b_refresh_overlap_steps must be within the refresh threshold"
            )
        for name, value in (
            ("b_release_stable_steps", self.b_release_stable_steps),
            (
                "b_release_observation_stable_frames",
                self.b_release_observation_stable_frames,
            ),
            ("b_completion_stable_frames", self.b_completion_stable_frames),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        if not 0.0 < self.b_release_open_threshold < 1.0:
            raise ValueError("b_release_open_threshold must be in (0, 1)")
        if not 0.0 <= self.a_gripper_close_latch_threshold <= 1.0:
            raise ValueError("a_gripper_close_latch_threshold must be in [0, 1]")
        limits = (
            self.b_execution_timeout_s,
            self.b_refresh_timeout_s,
            self.b_refresh_max_observation_age_s,
            self.b_release_driver_timeout_s,
            self.b_completion_velocity_tolerance_mm_s,
            self.first_action_position_limit_mm,
            self.first_action_orientation_limit_deg,
            self.start_position_radius_mm,
            self.start_orientation_radius_deg,
            self.b_first_orientation_jump_limit_deg,
            self.b_predicted_angular_velocity_limit_deg_s,
            self.handoff_orientation_tolerance_deg,
            self.b_final_refresh_lead_s,
            self.b_endpoint_settle_position_tolerance_mm,
            self.b_endpoint_settle_velocity_tolerance_mm_s,
            self.b_endpoint_settle_min_hold_s,
            self.b_endpoint_settle_timeout_s,
            self.b_tail_duration_min_s,
            self.b_tail_duration_max_s,
            self.b_tail_duration_step_s,
            self.actual_tracking_position_tolerance_mm,
            self.actual_tracking_orientation_tolerance_deg,
            self.downstream_command_max_age_s,
            self.downstream_ack_position_tolerance_mm,
            self.downstream_ack_orientation_tolerance_deg,
        )
        if any(value <= 0.0 for value in limits):
            raise ValueError("Task-C live pose limits must be positive")
        if self.bridge_ack_mode not in {"stop_and_wait", "bounded_pipeline"}:
            raise ValueError(
                "bridge_ack_mode must be stop_and_wait or bounded_pipeline"
            )
        if (
            not isinstance(self.bridge_max_ack_lag_steps, int)
            or isinstance(self.bridge_max_ack_lag_steps, bool)
            or self.bridge_max_ack_lag_steps not in {0, 1}
        ):
            raise ValueError("bridge_max_ack_lag_steps must be 0 or 1")
        if (
            self.bridge_ack_mode == "bounded_pipeline"
            and self.bridge_max_ack_lag_steps != 1
        ):
            raise ValueError(
                "bounded_pipeline requires bridge_max_ack_lag_steps=1"
            )
        for name, value in (
            (
                "actual_tracking_position_admission_margin_mm",
                self.actual_tracking_position_admission_margin_mm,
            ),
            (
                "actual_tracking_orientation_admission_margin_deg",
                self.actual_tracking_orientation_admission_margin_deg,
            ),
        ):
            if value < 0.0:
                raise ValueError(f"{name} must be non-negative")
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
        for name, value in (
            (
                "b_release_position_gate_enabled",
                self.b_release_position_gate_enabled,
            ),
            (
                "b_completion_position_gate_enabled",
                self.b_completion_position_gate_enabled,
            ),
        ):
            if not isinstance(value, bool):
                raise ValueError(f"{name} must be boolean")
        if self.b_completion_mode not in {
            "legacy_release_settle",
            "successor_owned",
        }:
            raise ValueError(
                "b_completion_mode must be legacy_release_settle or "
                "successor_owned"
            )
        if (
            self.b_completion_mode == "successor_owned"
            and self.b_completion_position_gate_enabled
        ):
            raise ValueError(
                "successor_owned completion cannot use a completion position gate"
            )
        if (
            not isinstance(self.b_overlap_search_max_skip_steps, int)
            or isinstance(self.b_overlap_search_max_skip_steps, bool)
            or self.b_overlap_search_max_skip_steps < 0
        ):
            raise ValueError(
                "b_overlap_search_max_skip_steps must be a non-negative integer"
            )
        if self.b_tail_duration_max_s < self.b_tail_duration_min_s:
            raise ValueError("B tail duration bounds must be ordered")
        if (
            self.actual_tracking_position_admission_margin_mm
            >= self.actual_tracking_position_tolerance_mm
        ):
            raise ValueError(
                "position admission margin must be below tracking tolerance"
            )
        if (
            self.actual_tracking_orientation_admission_margin_deg
            >= self.actual_tracking_orientation_tolerance_deg
        ):
            raise ValueError(
                "orientation admission margin must be below tracking tolerance"
            )
        for name, minimum, maximum in (
            (
                "b_release_workspace",
                self.b_release_workspace_min_xyz_mm,
                self.b_release_workspace_max_xyz_mm,
            ),
            (
                "b_completion_workspace",
                self.b_completion_workspace_min_xyz_mm,
                self.b_completion_workspace_max_xyz_mm,
            ),
        ):
            lower = np.asarray(minimum, dtype=np.float64)
            upper = np.asarray(maximum, dtype=np.float64)
            if lower.shape != (3,) or upper.shape != (3,):
                raise ValueError(f"{name} bounds must contain XYZ")
            if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
                raise ValueError(f"{name} bounds must be finite")
            if np.any(lower >= upper):
                raise ValueError(f"{name} bounds must be strictly ordered")
        if len(self.start_position_center_mm) != 3:
            raise ValueError("start_position_center_mm must contain XYZ")
        if len(self.start_orientation_center_zyz_deg) != 3:
            raise ValueError("start_orientation_center_zyz_deg must contain ZYZ")
        DownstreamControlContract(
            control_hz=self.downstream_control_hz,
            linear_ramp_mm_per_tick=self.downstream_linear_ramp_mm_per_tick,
            orientation_ramp_deg_per_tick=(
                self.downstream_orientation_ramp_deg_per_tick
            ),
            servol_time_s=self.downstream_servol_time_s,
            servol_use_auto_velocity_acceleration=(
                self.downstream_servol_use_auto_velocity_acceleration
            ),
            lerobot_timeout_s=self.downstream_lerobot_timeout_s,
            lerobot_target_topic=self.downstream_lerobot_target_topic,
            selected_target_topic=self.downstream_selected_target_topic,
            safe_posx_topic=self.downstream_safe_posx_topic,
            commanded_posx_topic=self.downstream_commanded_posx_topic,
            workspace_min_xyz_mm=self.downstream_workspace_min_xyz_mm,
            workspace_min_limit_enabled=(
                self.downstream_workspace_min_limit_enabled
            ),
            workspace_max_xyz_mm=self.downstream_workspace_max_xyz_mm,
            orientation_limit_deg=self.downstream_orientation_limit_deg,
        )


def _resolved(path: str | Path) -> Path:
    return Path(path).expanduser().resolve()


def _full_state(policy_input: dict[str, Any]) -> np.ndarray:
    value = policy_input["observation.state"]
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    state = np.asarray(value, dtype=np.float64)
    if state.shape != (13,) or not np.all(np.isfinite(state)):
        raise ValueError("live ACT state must be finite shape (13,)")
    return state.copy()


def _semantic(state: np.ndarray) -> SemanticState:
    closed = bool(round(float(state[12])))
    return SemanticState(
        gripper_closed=closed,
        holding=True if closed else False,
        contact_mode="free_transport_assumed" if closed else "NOT_OBSERVED",
        completed_subgoals=("grasp_complete",) if closed else (),
        object_state="A_OBJECT_HELD_ASSUMED" if closed else "NOT_OBSERVED",
    )


def _writable_engine_snapshot(value: Any) -> Any:
    """Own RTC inputs without exposing immutable camera buffers to PyTorch."""

    if isinstance(value, np.ndarray):
        return np.array(value, copy=True)
    try:
        import torch

        if isinstance(value, torch.Tensor):
            return value.detach().clone()
    except ImportError:
        pass
    if isinstance(value, dict):
        return {key: _writable_engine_snapshot(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_writable_engine_snapshot(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_writable_engine_snapshot(item) for item in value)
    try:
        return copy.deepcopy(value)
    except TypeError:
        return value


def _inside_xyz_bounds(
    xyz_mm: np.ndarray,
    minimum_mm: tuple[float, float, float],
    maximum_mm: tuple[float, float, float],
) -> bool:
    xyz = np.asarray(xyz_mm, dtype=np.float64)
    lower = np.asarray(minimum_mm, dtype=np.float64)
    upper = np.asarray(maximum_mm, dtype=np.float64)
    return bool(
        xyz.shape == (3,)
        and np.all(np.isfinite(xyz))
        and np.all(xyz >= lower)
        and np.all(xyz <= upper)
    )


def _blend_b_action_overlap(
    old_actions: np.ndarray,
    new_actions: np.ndarray,
    overlap_steps: int,
) -> np.ndarray:
    """Delay-aligned full-pose blend; keep old discrete gripper decisions."""

    old = np.asarray(old_actions, dtype=np.float64)
    new = np.asarray(new_actions, dtype=np.float64)
    if old.ndim != 2 or new.ndim != 2 or old.shape[1:] != new.shape[1:]:
        raise ValueError("B overlap chunks must have matching [steps, action_dim]")
    if old.shape[1] < 7 or not np.all(np.isfinite(old)) or not np.all(np.isfinite(new)):
        raise ValueError("B overlap chunks must contain finite full actions")
    if not isinstance(overlap_steps, int) or isinstance(overlap_steps, bool):
        raise ValueError("B overlap_steps must be an integer")
    steps = min(max(0, overlap_steps), len(old), len(new))
    if steps == 0:
        return new.copy()

    fractions = np.arange(1, steps + 1, dtype=np.float64) / float(steps + 1)
    weights = 3.0 * fractions**2 - 2.0 * fractions**3
    blended = old[:steps].copy()
    blended[:, :3] = (
        (1.0 - weights[:, None]) * old[:steps, :3]
        + weights[:, None] * new[:steps, :3]
    )
    orientation_reference: list[float] | None = None
    for index, weight in enumerate(weights):
        old_zyz = old[index, 3:6].tolist()
        new_zyz = new[index, 3:6].tolist()
        quaternion = quaternion_slerp(
            doosan_zyz_deg_to_quaternion(old_zyz),
            doosan_zyz_deg_to_quaternion(new_zyz),
            float(weight),
        )
        reference = old_zyz if orientation_reference is None else orientation_reference
        orientation_reference = quaternion_to_doosan_zyz_deg(quaternion, reference)
        blended[index, 3:6] = orientation_reference
    blended[:, 6:] = old[:steps, 6:]
    return np.concatenate((blended, new[steps:].copy()), axis=0)


class TaskCLiveStrategy(RolloutStrategy):
    """Execute one full Task-C trial when externally armed."""

    def __init__(self, config: TaskCLiveStrategyConfig) -> None:
        super().__init__(config)
        self.live_config = config
        self.phase = LivePhase.WAITING_FOR_LIVE
        self._raw_robot: Any = None
        self._event_file: Any = None
        self._event_pub: Any = None
        self._ready_pub: Any = None
        self._phase_pub: Any = None
        self._manifest: dict[str, Any] | None = None
        self._representative_boundary_contract: (
            RepresentativeBoundaryContract | None
        ) = None
        self._representative_boundary_tracker: (
            RepresentativeBoundaryTracker | None
        ) = None
        self._representative_boundary_status: (
            RepresentativeBoundaryStatus | None
        ) = None
        self._representative_preplan_attempts = 0
        self._representative_preplan_succeeded = False
        self._handoff_config: RuntimeHandoffConfig | None = None
        self._coordinator: TaskCRealtimeCoordinator | None = None
        self._session_a_stub: AsyncPolicySession | None = None
        self._session_b: AsyncPolicySession | None = None
        self._orientation_bridge: OrientationBridge | None = None
        self._cut_trigger = LiveCutTrigger(
            CutTriggerConfig(
                open_stable_frames=config.cut_open_stable_frames,
                closed_stable_frames=config.cut_closed_stable_frames,
                post_close_delay_s=config.cut_post_close_delay_s,
                transport_z_min_mm=config.cut_transport_z_min_mm,
            )
        )
        self._actual_history = RuntimeVelocityHistory(120)
        self._a_path_length_mm = 0.0
        self._last_actual_xyz: np.ndarray | None = None
        self._last_command_pose: np.ndarray | None = None
        self._last_command_action: np.ndarray | None = None
        self._pending_command: _PendingLiveCommand | None = None
        self._last_bridge_commanded_receive_count: int | None = None
        self._last_acknowledged_bridge_pose: np.ndarray | None = None
        self._last_acknowledged_bridge_receive_count: int | None = None
        self._bridge_ack_wait_cycles = 0
        self._bridge_ack_no_fresh_cycles = 0
        self._bridge_ack_pose_mismatch_cycles = 0
        self._bridge_ack_lag_hold_cycles = 0
        self._bridge_ack_pipeline_advance_cycles = 0
        self._bridge_ack_max_decision_lag_steps = 0
        self._bridge_ack_max_outstanding_after_send_steps = 0
        self._bridge_ack_sent_sequence = -1
        self._bridge_ack_acknowledged_sequence = -1
        self._bridge_ack_anchor_pose: np.ndarray | None = None
        self._bridge_ack_last_observed_receive_count: int | None = None
        self._bridge_ack_unresolved_mismatch = False
        self._bridge_ack_sent_history: deque[_BridgeSentCommand] = deque(
            maxlen=8
        )
        self._tracking_backpressure_events = 0
        self._tracking_backpressure_wait_cycles = 0
        self._tracking_hold_commands = 0
        self._tracking_backpressure_max_candidate_position_error_mm = 0.0
        self._tracking_backpressure_max_candidate_orientation_error_deg = 0.0
        self._live_started_s: float | None = None
        self._transition_started_s: float | None = None
        self._a_gripper_close_latched = False
        self._b_steps_sent = 0
        self._b_started_s: float | None = None
        self._b_refresh_generation: int | None = None
        self._b_refresh_requested_s: float | None = None
        self._b_refresh_queue_consumed_at_request = 0
        self._b_refresh_count = 0
        self._b_refresh_hold_cycles = 0
        self._b_direct_handoff_validated_generation: int | None = None
        self._b_release_candidate_steps = 0
        self._b_release_authorized = False
        self._b_release_authorized_s: float | None = None
        self._b_release_command_sent_s: float | None = None
        self._b_release_confirmed = False
        self._b_release_open_observation_frames = 0
        self._b_release_pose_mm: np.ndarray | None = None
        self._b_completion_stable_frames = 0
        self._commands_by_phase = {phase.value: 0 for phase in LivePhase}
        self._failure_reasons: list[str] = []
        self._downstream_contract = DownstreamControlContract(
            control_hz=config.downstream_control_hz,
            linear_ramp_mm_per_tick=config.downstream_linear_ramp_mm_per_tick,
            orientation_ramp_deg_per_tick=(
                config.downstream_orientation_ramp_deg_per_tick
            ),
            servol_time_s=config.downstream_servol_time_s,
            servol_use_auto_velocity_acceleration=(
                config.downstream_servol_use_auto_velocity_acceleration
            ),
            lerobot_timeout_s=config.downstream_lerobot_timeout_s,
            lerobot_target_topic=config.downstream_lerobot_target_topic,
            selected_target_topic=config.downstream_selected_target_topic,
            safe_posx_topic=config.downstream_safe_posx_topic,
            commanded_posx_topic=config.downstream_commanded_posx_topic,
            workspace_min_xyz_mm=config.downstream_workspace_min_xyz_mm,
            workspace_min_limit_enabled=(
                config.downstream_workspace_min_limit_enabled
            ),
            workspace_max_xyz_mm=config.downstream_workspace_max_xyz_mm,
            orientation_limit_deg=config.downstream_orientation_limit_deg,
        )
        self._live_orientation_anchor: np.ndarray | None = None
        self._effective_bridge_workspace_min_mm: np.ndarray | None = None
        self._effective_bridge_workspace_max_mm: np.ndarray | None = None

    def _event(self, event: str, **details: Any) -> None:
        record = {
            "schema": "task_c_live_v1",
            "timestamp_s": time.monotonic(),
            "phase": self.phase.value,
            "event": event,
            **details,
        }
        if self._event_file is not None:
            self._event_file.write(json.dumps(record, sort_keys=True) + "\n")
            self._event_file.flush()
        message = String(data=json.dumps(record, sort_keys=True))
        if self._event_pub is not None:
            self._event_pub.publish(message)
        if (
            event == "strategy_ready_external_gate_required"
            and self._ready_pub is not None
        ):
            self._ready_pub.publish(message)
        if self._phase_pub is not None:
            self._phase_pub.publish(String(data=self.phase.value))

    def _capture(
        self, ctx: Any
    ) -> tuple[_LiveCapture, np.ndarray, dict[str, Any], dict[str, Any]]:
        started = time.monotonic()
        raw = ctx.hardware.robot_wrapper.get_observation()
        captured = time.monotonic()
        processed = ctx.processors.robot_observation_processor(raw)
        processed_at = time.monotonic()
        if (
            self._engine is not None
            and self.phase in {LivePhase.WAITING_FOR_LIVE, LivePhase.ACT_A}
        ):
            self._engine.notify_observation(_writable_engine_snapshot(processed))
        assert self._session_b is not None
        policy_input = _policy_input_from_processed(
            processed,
            hw_features=ctx.data.hw_features,
            expected_features=self._session_b.backend.config.input_features,
        )
        ready = time.monotonic()
        state = _full_state(policy_input)
        cache = getattr(self._raw_robot, "cache", None)
        source_ages: dict[str, float | None] = {}
        for key in (
            "joint_positions",
            "actual_tcp_position",
            "gripper_commanded_state",
            "gripper_completed_command",
            "gripper_driver_busy",
            "gripper_last_command_ok",
            "commanded_posx",
        ):
            sample = None if cache is None else cache.sample(key)
            source_ages[key] = None if sample is None else sample.age(captured)
        capture = _LiveCapture(
            capture_started_s=started,
            capture_completed_s=captured,
            processor_completed_s=processed_at,
            policy_input_ready_s=ready,
            policy_input=policy_input,
            tcp_position_mm=state[6:9],
            semantic_state=_semantic(state),
            source_ages_s=source_ages,
        )
        return capture, state, raw, processed

    def _runtime_observation(
        self, capture: _LiveCapture, state: np.ndarray
    ) -> RuntimeObservation:
        del state
        command_position = None
        command_velocity = None
        if (
            self.phase is LivePhase.BRIDGE
            and self._last_acknowledged_bridge_pose is not None
            and self._coordinator is not None
            and self._coordinator.current_plan is not None
        ):
            bridge = self._coordinator.current_plan.bridge
            u = min(
                1.0,
                max(0.0, self._coordinator.bridge_elapsed_s / bridge.duration_s),
            )
            command_position = self._last_acknowledged_bridge_pose[:3]
            command_velocity = bridge.velocity(u)
        return RuntimeObservation(
            timestamp_s=capture.capture_completed_s,
            tcp_position_mm=capture.tcp_position_mm,
            semantic_state=capture.semantic_state,
            policy_input=capture.policy_input,
            acknowledged_command_position_mm=command_position,
            acknowledged_command_velocity_mm_s=command_velocity,
        )

    def _assert_downstream_passthrough(
        self,
        target_pose: np.ndarray,
        *,
        actual_pose: np.ndarray,
        source: str,
        enforce_stream_ramp: bool,
        streamer_command_pose: np.ndarray | None = None,
    ) -> None:
        """Reject guard clamps and, at transition boundaries, stream ramps."""

        target = np.asarray(target_pose, dtype=np.float64)
        actual = np.asarray(actual_pose, dtype=np.float64)
        if target.shape != (6,) or actual.shape != (6,):
            raise ValueError("downstream pose contract requires shape (6,)")
        if enforce_stream_ramp and streamer_command_pose is None:
            raise RuntimeError(
                f"{source} requires a fresh ServoL commanded_posx reference"
            )
        if streamer_command_pose is not None:
            reference = np.asarray(streamer_command_pose, dtype=np.float64)
            if reference.shape != (6,) or not np.all(np.isfinite(reference)):
                raise ValueError("streamer command pose must be finite shape (6,)")
        else:
            reference = (
                actual if self._last_command_pose is None else self._last_command_pose
            )
        axis_step = np.abs(target[:3] - reference[:3])
        _position_step, orientation_step = pose_delta_metrics(reference, target)
        contract = self._downstream_contract
        failures: list[str] = []
        if enforce_stream_ramp and np.any(
            axis_step > contract.linear_ramp_mm_per_tick + 1e-9
        ):
            failures.append(
                "linear_ramp="
                + ",".join(f"{value:.3f}" for value in axis_step)
            )
        if (
            enforce_stream_ramp
            and orientation_step > contract.orientation_ramp_deg_per_tick + 1e-9
        ):
            failures.append(f"orientation_ramp={orientation_step:.3f}")

        guard_minimum = np.asarray(contract.workspace_min_xyz_mm, dtype=np.float64)
        guard_maximum = np.asarray(contract.workspace_max_xyz_mm, dtype=np.float64)
        minimum_enabled = np.asarray(
            contract.workspace_min_limit_enabled, dtype=np.bool_
        )
        if (
            np.any(target[:3][minimum_enabled] < guard_minimum[minimum_enabled] - 1e-9)
            or np.any(target[:3] > guard_maximum + 1e-9)
        ):
            failures.append("workspace_clamp")
        anchor = (
            actual[3:6]
            if self._live_orientation_anchor is None
            else self._live_orientation_anchor
        )
        _unused, anchor_error = pose_delta_metrics(
            np.concatenate((target[:3], anchor)),
            target,
        )
        if anchor_error > contract.orientation_limit_deg + 1e-9:
            failures.append(f"orientation_clamp={anchor_error:.3f}")
        if failures:
            raise RuntimeError(
                f"{source} downstream pass-through contract failed: "
                + ";".join(failures)
            )

    def _assess_bridge_passthrough(
        self,
        bridge: Any,
        start_orientation: np.ndarray,
        target_orientation: np.ndarray,
    ) -> BridgeStreamAssessment:
        assessment = assess_bridge_stream_contract(
            bridge,
            start_orientation,
            target_orientation,
            contract=self._downstream_contract,
            orientation_anchor_zyz_deg=self._live_orientation_anchor,
        )
        if not assessment.valid:
            raise RuntimeError(
                "Bridge downstream pass-through assessment failed: "
                + ",".join(assessment.failure_reasons)
            )
        return assessment

    def _bounded_bridge_ack_pipeline_enabled(self) -> bool:
        return self.live_config.bridge_ack_mode == "bounded_pipeline"

    def _reset_bridge_ack_tracking(
        self,
        anchor_pose_mm_deg: np.ndarray | None = None,
        receive_count: int | None = None,
    ) -> None:
        if (anchor_pose_mm_deg is None) != (receive_count is None):
            raise ValueError(
                "Bridge ACK anchor pose and receive count must be paired"
            )
        anchor = None
        if anchor_pose_mm_deg is not None:
            anchor = np.asarray(anchor_pose_mm_deg, dtype=np.float64)
            if anchor.shape != (6,) or not np.all(np.isfinite(anchor)):
                raise ValueError("Bridge ACK anchor must be finite shape (6,)")
            anchor = anchor.copy()
        self._bridge_ack_wait_cycles = 0
        self._bridge_ack_no_fresh_cycles = 0
        self._bridge_ack_pose_mismatch_cycles = 0
        self._bridge_ack_lag_hold_cycles = 0
        self._bridge_ack_pipeline_advance_cycles = 0
        self._bridge_ack_max_decision_lag_steps = 0
        self._bridge_ack_max_outstanding_after_send_steps = 0
        self._bridge_ack_sent_sequence = -1
        self._bridge_ack_acknowledged_sequence = -1
        self._bridge_ack_anchor_pose = anchor
        self._bridge_ack_last_observed_receive_count = (
            None if receive_count is None else int(receive_count)
        )
        self._bridge_ack_unresolved_mismatch = False
        if hasattr(self, "_bridge_ack_sent_history"):
            self._bridge_ack_sent_history.clear()
        else:
            # Some pure lifecycle tests construct the strategy without
            # running the hardware-owning constructor.
            self._bridge_ack_sent_history = deque(maxlen=8)
        self._last_acknowledged_bridge_pose = (
            None if anchor is None else anchor.copy()
        )
        self._last_acknowledged_bridge_receive_count = (
            None if receive_count is None else int(receive_count)
        )

    def _register_bridge_sent_command(
        self,
        pose_mm_deg: np.ndarray,
        *,
        receive_count_before_send: int,
    ) -> None:
        if not self._bounded_bridge_ack_pipeline_enabled():
            return
        pose = np.asarray(pose_mm_deg, dtype=np.float64)
        if pose.shape != (6,) or not np.all(np.isfinite(pose)):
            raise ValueError("Bridge sent command must be finite shape (6,)")
        if self._bridge_ack_anchor_pose is None:
            raise RuntimeError("bounded Bridge ACK pipeline lacks an anchor")
        self._bridge_ack_sent_sequence += 1
        self._bridge_ack_sent_history.append(
            _BridgeSentCommand(
                sequence=self._bridge_ack_sent_sequence,
                pose_mm_deg=pose.copy(),
                receive_count_before_send=int(receive_count_before_send),
            )
        )
        lag = (
            self._bridge_ack_sent_sequence
            - self._bridge_ack_acknowledged_sequence
        )
        self._bridge_ack_max_outstanding_after_send_steps = max(
            self._bridge_ack_max_outstanding_after_send_steps,
            lag,
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
        values = np.asarray(action, dtype=np.float64)
        if values.shape != (7,) or not np.all(np.isfinite(values)):
            raise ValueError("Task-C live action must be finite shape (7,)")
        streamer_command_pose = None
        streamer_command_receive_count = None
        if enforce_stream_ramp:
            cache = getattr(self._raw_robot, "cache", None)
            if cache is None:
                raise RuntimeError("ServoL commanded_posx cache is unavailable")
            streamer_sample = cache.require(
                "commanded_posx",
                max_age_sec=self.live_config.downstream_command_max_age_s,
                now=time.monotonic(),
            )
            streamer_command_pose = np.asarray(streamer_sample.value, dtype=np.float64)
            streamer_command_receive_count = streamer_sample.receive_count
        self._assert_downstream_passthrough(
            values[:6],
            actual_pose=actual_pose,
            source=source,
            enforce_stream_ramp=enforce_stream_ramp,
            streamer_command_pose=streamer_command_pose,
        )
        payload = {
            key: float(values[index]) for index, key in enumerate(ACTION_KEYS)
        }
        ctx.hardware.robot_wrapper.send_action(payload)
        self._last_command_pose = values[:6].copy()
        self._last_command_action = values.copy()
        if source == "BEZIER_BRIDGE":
            if (
                streamer_command_pose is None
                or streamer_command_receive_count is None
            ):
                raise RuntimeError(
                    "Bridge command lacks downstream ACK baseline"
                )
            if (
                self._bounded_bridge_ack_pipeline_enabled()
                and self._bridge_ack_anchor_pose is None
            ):
                self._reset_bridge_ack_tracking(
                    streamer_command_pose,
                    int(streamer_command_receive_count),
                )
            self._register_bridge_sent_command(
                values[:6],
                receive_count_before_send=int(
                    streamer_command_receive_count
                ),
            )
            self._last_bridge_commanded_receive_count = (
                streamer_command_receive_count
            )
        self._commands_by_phase[self.phase.value] += 1
        logger.debug("Task-C live command source=%s pose=%s", source, values[:6])

    def _assert_tracking(self, actual_pose: np.ndarray) -> None:
        if self._last_command_pose is None:
            return
        position_error, orientation_error = pose_delta_metrics(
            actual_pose, self._last_command_pose
        )
        if position_error > self.live_config.actual_tracking_position_tolerance_mm:
            raise RuntimeError(
                f"actual position tracking error {position_error:.3f} mm"
            )
        if orientation_error > self.live_config.actual_tracking_orientation_tolerance_deg:
            raise RuntimeError(
                f"actual orientation tracking error {orientation_error:.3f} deg"
            )

    def _tracking_admission(
        self,
        actual_pose: np.ndarray,
        candidate_pose: np.ndarray,
    ) -> tuple[bool, float, float]:
        position_error, orientation_error = pose_delta_metrics(
            actual_pose,
            candidate_pose,
        )
        position_limit = (
            self.live_config.actual_tracking_position_tolerance_mm
            - self.live_config.actual_tracking_position_admission_margin_mm
        )
        orientation_limit = (
            self.live_config.actual_tracking_orientation_tolerance_deg
            - self.live_config.actual_tracking_orientation_admission_margin_deg
        )
        admitted = (
            position_error <= position_limit
            and orientation_error <= orientation_limit
        )
        return admitted, position_error, orientation_error

    def _commit_live_command(
        self,
        ctx: Any,
        pending: _PendingLiveCommand,
        *,
        actual_pose: np.ndarray,
    ) -> None:
        previous_phase = self.phase
        if pending.transition_to_act_b:
            self.phase = LivePhase.ACT_B
        try:
            self._send_array(
                ctx,
                pending.action,
                pending.source,
                actual_pose=actual_pose,
                enforce_stream_ramp=pending.enforce_stream_ramp,
            )
        except Exception:
            self.phase = previous_phase
            raise

        if pending.transition_to_act_b:
            endpoint_position_error, endpoint_orientation_error = pose_delta_metrics(
                actual_pose,
                pending.action[:6],
            )
            assert self._coordinator is not None
            if self._b_started_s is None:
                self._b_started_s = time.monotonic()
            self._event(
                "atomic_bridge_to_act_b",
                generation=self._coordinator.b_generation,
                generation_role=self._coordinator.b_generation_role,
                endpoint_position_error_mm=endpoint_position_error,
                endpoint_orientation_error_deg=endpoint_orientation_error,
            )

        if pending.source == "ACT-B":
            self._b_steps_sent += 1
            if (
                self._b_release_authorized
                and pending.action[6] < self.live_config.b_release_open_threshold
                and self._b_release_command_sent_s is None
            ):
                self._b_release_command_sent_s = time.monotonic()
                self._event(
                    "act_b_release_command_sent",
                    b_steps_sent=self._b_steps_sent,
                    active_generation=(
                        None
                        if self._session_b is None
                        else self._session_b.active_generation
                    ),
                    policy_target_mm_deg=pending.action[:6].tolist(),
                    gripper_target=float(pending.action[6]),
                )

    def _send_tracking_hold(
        self,
        ctx: Any,
        pending: _PendingLiveCommand,
        *,
        actual_pose: np.ndarray,
    ) -> None:
        if self._last_command_action is None:
            raise RuntimeError("tracking backpressure lacks a prior command")
        bridge_hold = self.phase is LivePhase.BRIDGE
        self._send_array(
            ctx,
            self._last_command_action.copy(),
            "BEZIER_BRIDGE" if bridge_hold else "ACT-B",
            actual_pose=actual_pose,
            enforce_stream_ramp=bridge_hold,
        )
        pending.wait_cycles += 1
        self._tracking_backpressure_wait_cycles += 1
        self._tracking_hold_commands += 1

    def _submit_live_command(
        self,
        ctx: Any,
        action: np.ndarray,
        source: str,
        *,
        actual_pose: np.ndarray,
        enforce_stream_ramp: bool,
        transition_to_act_b: bool = False,
    ) -> bool:
        if self._pending_command is not None:
            raise RuntimeError("cannot replace an uncommitted live command")
        values = np.asarray(action, dtype=np.float64)
        if values.shape != (7,) or not np.all(np.isfinite(values)):
            raise ValueError("Task-C pending action must be finite shape (7,)")
        admitted, position_error, orientation_error = self._tracking_admission(
            actual_pose,
            values[:6],
        )
        pending = _PendingLiveCommand(
            action=values.copy(),
            source=source,
            enforce_stream_ramp=enforce_stream_ramp,
            transition_to_act_b=transition_to_act_b,
            queued_s=time.monotonic(),
            wait_cycles=0,
            max_candidate_position_error_mm=position_error,
            max_candidate_orientation_error_deg=orientation_error,
        )
        if admitted:
            self._commit_live_command(ctx, pending, actual_pose=actual_pose)
            return True

        self._pending_command = pending
        self._tracking_backpressure_events += 1
        self._tracking_backpressure_max_candidate_position_error_mm = max(
            self._tracking_backpressure_max_candidate_position_error_mm,
            position_error,
        )
        self._tracking_backpressure_max_candidate_orientation_error_deg = max(
            self._tracking_backpressure_max_candidate_orientation_error_deg,
            orientation_error,
        )
        last_position_error = None
        last_orientation_error = None
        if self._last_command_pose is not None:
            last_position_error, last_orientation_error = pose_delta_metrics(
                actual_pose,
                self._last_command_pose,
            )
        self._event(
            "tracking_backpressure_started",
            pending_source=source,
            candidate_position_error_mm=position_error,
            candidate_orientation_error_deg=orientation_error,
            last_command_position_error_mm=last_position_error,
            last_command_orientation_error_deg=last_orientation_error,
            admission_position_limit_mm=(
                self.live_config.actual_tracking_position_tolerance_mm
                - self.live_config.actual_tracking_position_admission_margin_mm
            ),
            admission_orientation_limit_deg=(
                self.live_config.actual_tracking_orientation_tolerance_deg
                - self.live_config.actual_tracking_orientation_admission_margin_deg
            ),
        )
        self._send_tracking_hold(ctx, pending, actual_pose=actual_pose)
        return False

    def _flush_pending_command(
        self,
        ctx: Any,
        *,
        actual_pose: np.ndarray,
    ) -> bool:
        pending = self._pending_command
        if pending is None:
            return False
        admitted, position_error, orientation_error = self._tracking_admission(
            actual_pose,
            pending.action[:6],
        )
        pending.max_candidate_position_error_mm = max(
            pending.max_candidate_position_error_mm,
            position_error,
        )
        pending.max_candidate_orientation_error_deg = max(
            pending.max_candidate_orientation_error_deg,
            orientation_error,
        )
        self._tracking_backpressure_max_candidate_position_error_mm = max(
            self._tracking_backpressure_max_candidate_position_error_mm,
            position_error,
        )
        self._tracking_backpressure_max_candidate_orientation_error_deg = max(
            self._tracking_backpressure_max_candidate_orientation_error_deg,
            orientation_error,
        )
        if not admitted:
            self._send_tracking_hold(ctx, pending, actual_pose=actual_pose)
            return False

        self._commit_live_command(ctx, pending, actual_pose=actual_pose)
        self._pending_command = None
        self._event(
            "tracking_backpressure_released",
            pending_source=pending.source,
            wait_cycles=pending.wait_cycles,
            wait_duration_s=time.monotonic() - pending.queued_s,
            release_candidate_position_error_mm=position_error,
            release_candidate_orientation_error_deg=orientation_error,
            max_candidate_position_error_mm=(
                pending.max_candidate_position_error_mm
            ),
            max_candidate_orientation_error_deg=(
                pending.max_candidate_orientation_error_deg
            ),
        )
        return True

    def _assert_task_a_start(self, state: np.ndarray) -> None:
        center = np.asarray(
            (*self.live_config.start_position_center_mm,
             *self.live_config.start_orientation_center_zyz_deg),
            dtype=np.float64,
        )
        position_error, orientation_error = pose_delta_metrics(state[6:12], center)
        if position_error > self.live_config.start_position_radius_mm:
            raise RuntimeError(
                "Task-A start position is outside the demonstration envelope: "
                f"{position_error:.3f} mm"
            )
        if orientation_error > self.live_config.start_orientation_radius_deg:
            raise RuntimeError(
                "Task-A start orientation is outside the demonstration envelope: "
                f"{orientation_error:.3f} deg"
            )
        if bool(round(float(state[12]))):
            raise RuntimeError("Task-A live start requires gripper open")

    def _send_next_a_action(
        self,
        ctx: Any,
        *,
        processed_observation: dict[str, Any],
        raw_observation: dict[str, Any],
        actual_pose: np.ndarray,
        gripper_close_latch_armed: bool,
    ) -> dict[str, float] | None:
        interpolator = self._interpolator
        if interpolator.needs_new_action():
            obs_frame = build_dataset_frame(
                ctx.data.dataset_features,
                processed_observation,
                prefix=OBS_STR,
            )
            action_tensor = self._engine.get_action(obs_frame)
            if action_tensor is not None:
                interpolator.add(action_tensor.cpu())
        interpolated = interpolator.get()
        if interpolated is None:
            return None
        ordered_keys = ctx.data.ordered_action_keys
        if len(interpolated) != len(ordered_keys):
            raise ValueError("ACT-A action dimension differs from robot contract")
        action_dict = {
            key: interpolated[index].item()
            for index, key in enumerate(ordered_keys)
        }
        values = np.asarray(
            [action_dict[key] for key in ordered_keys],
            dtype=np.float64,
        )
        raw_gripper_target = float(values[6])
        outgoing_gripper_target, newly_latched = (
            self._task_a_gripper_target(
                raw_gripper_target,
                open_seen=gripper_close_latch_armed,
            )
        )
        values[6] = outgoing_gripper_target
        action_dict[ordered_keys[6]] = outgoing_gripper_target
        if newly_latched:
            self._event(
                "act_a_gripper_close_latched",
                raw_policy_gripper_target=raw_gripper_target,
                outgoing_gripper_target=outgoing_gripper_target,
                open_before_close_observed=gripper_close_latch_armed,
                policy_pose_mm_deg=values[:6].tolist(),
                latch_scope="remaining_act_a_only",
            )
        position_jump, orientation_jump = pose_delta_metrics(
            actual_pose,
            values[:6],
        )
        if position_jump > self.live_config.first_action_position_limit_mm:
            raise RuntimeError(
                f"ACT-A target position jump {position_jump:.3f} mm"
            )
        if orientation_jump > self.live_config.first_action_orientation_limit_deg:
            raise RuntimeError(
                f"ACT-A target orientation jump {orientation_jump:.3f} deg"
            )
        self._assert_downstream_passthrough(
            values[:6],
            actual_pose=actual_pose,
            source="ACT-A",
            enforce_stream_ramp=False,
        )
        robot_action = ctx.processors.robot_action_processor(
            (action_dict, raw_observation)
        )
        ctx.hardware.robot_wrapper.send_action(robot_action)
        self._last_command_pose = values[:6].copy()
        # Bridge tracking backpressure must be able to repeat the exact final
        # ACT-A command, including its discrete gripper state. Keeping only
        # the 6D pose leaves no safe 7D hold command at the A->Bridge boundary.
        self._last_command_action = values.copy()
        return action_dict

    def _task_a_gripper_target(
        self,
        raw_target: float,
        *,
        open_seen: bool,
    ) -> tuple[float, bool]:
        """Latch only Task-A's gripper after its first valid close request."""

        target = float(raw_target)
        if not np.isfinite(target):
            raise ValueError("ACT-A gripper target must be finite")
        if self._a_gripper_close_latched:
            return 1.0, False
        if (
            open_seen
            and target >= self.live_config.a_gripper_close_latch_threshold
        ):
            self._a_gripper_close_latched = True
            return 1.0, True
        return target, False

    def setup(self, ctx: Any) -> None:
        config = self.live_config
        if getattr(ctx.runtime.cfg.robot, "mode", None) != "policy_live":
            raise ValueError("Task-C live requires --robot.mode=policy_live")
        if ctx.runtime.cfg.return_to_initial_position:
            raise ValueError("Task-C live requires --return_to_initial_position=false")
        if not config.runtime_manifest or not config.event_jsonl_path:
            raise ValueError("runtime_manifest and event_jsonl_path are required")
        if not np.isclose(
            float(ctx.runtime.cfg.fps),
            self._downstream_contract.control_hz,
            atol=1e-9,
            rtol=0.0,
        ):
            raise ValueError(
                "Task-C policy FPS must match downstream control_hz exactly"
            )

        self._raw_robot = ctx.hardware.robot_wrapper.inner
        robot_config = self._raw_robot.config
        if (
            robot_config.lerobot_target_topic
            != self._downstream_contract.lerobot_target_topic
        ):
            raise ValueError("robot adapter LeRobot target topic differs from Task-C contract")
        if (
            robot_config.commanded_posx_topic
            != self._downstream_contract.commanded_posx_topic
        ):
            raise ValueError("robot adapter commanded_posx topic differs from Task-C contract")
        manifest_path = _resolved(config.runtime_manifest)
        self._manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        has_representative_boundary = "representative_boundary" in self._manifest
        if config.representative_boundary_enabled:
            if self._manifest.get("mode") != "representative_boundary_dry_run_design":
                raise ValueError(
                    "representative boundary mode requires a V1 representative manifest"
                )
            self._representative_boundary_contract = (
                RepresentativeBoundaryContract.from_manifest(self._manifest)
            )
            self._representative_boundary_tracker = RepresentativeBoundaryTracker(
                self._representative_boundary_contract
            )
            paths = dict(self._manifest.get("representative_paths", {}))
            if paths.get("single_representative_per_task") is not True:
                raise ValueError("V1 manifest must contain one representative per task")
            if len(self._manifest.get("b_entries", ())) != 1:
                raise ValueError("V1 manifest must contain exactly one B entry")
        elif has_representative_boundary:
            raise ValueError(
                "V1 representative manifest requires explicit "
                "--strategy.representative_boundary_enabled=true"
            )
        manifest_safety = dict(self._manifest.get("safety", {}))
        if (
            manifest_safety.get("robot_executable") is not False
            or manifest_safety.get("dry_run_only") is not True
        ):
            raise ValueError(
                "Task-C live currently accepts only the reviewed offline-only "
                "runtime manifest format"
            )
        if not config.acknowledge_uncertified_manifest:
            raise ValueError(
                "Task-C live requires explicit acknowledgement that the runtime "
                "manifest is not robot-certified"
            )
        checkpoints = dict(self._manifest.get("policy_checkpoints", {}))
        checkpoint_a = str(ctx.runtime.cfg.policy.pretrained_path)
        if checkpoints.get("ACT-A") and not _same_checkpoint(
            checkpoint_a, checkpoints["ACT-A"]
        ):
            raise ValueError("loaded ACT-A differs from Task-C manifest")
        checkpoint_b = config.checkpoint_b or str(checkpoints.get("ACT-B", ""))
        if not checkpoint_b:
            raise ValueError("ACT-B checkpoint is absent")

        output = _resolved(config.event_jsonl_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        self._event_file = output.open("a", encoding="utf-8")
        task_c_state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        task_c_ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._event_pub = self._raw_robot._node.create_publisher(
            String,
            "/control/task_c/event",
            task_c_state_qos,
        )
        self._ready_pub = self._raw_robot._node.create_publisher(
            String,
            "/control/task_c/ready",
            task_c_ready_qos,
        )
        self._phase_pub = self._raw_robot._node.create_publisher(
            String,
            "/control/task_c/phase",
            task_c_state_qos,
        )

        backend_b = LeRobotACTBackend(
            checkpoint_b,
            device=str(ctx.runtime.cfg.device),
        )
        a_contract = {
            key: tuple(value.shape)
            for key, value in ctx.policy.policy.config.input_features.items()
        }
        b_contract = {
            key: tuple(value.shape)
            for key, value in backend_b.config.input_features.items()
        }
        if a_contract != b_contract:
            raise ValueError("ACT-A/B observation contracts differ")
        self._session_b = AsyncPolicySession(
            "ACT-B",
            backend_b,
            action_hz=float(ctx.runtime.cfg.fps),
            inference_lock=get_act_gpu_arbiter(),
        )

        # The position coordinator still requires two isolated session objects,
        # but ACT-A command execution is owned exclusively by ctx.policy.inference.
        # This inert stub is never primed or consumed.
        self._session_a_stub = AsyncPolicySession(
            "ACT-A-EXTERNAL",
            backend_b,
            action_hz=float(ctx.runtime.cfg.fps),
            inference_lock=get_act_gpu_arbiter(),
        )
        entries = runtime_entries_from_manifest(self._manifest)
        initial_planner = RuntimeBridgePlanner.from_manifest(self._manifest)
        tail_planner = RuntimeBridgePlanner.from_manifest(
            self._manifest, duration_search="tail_duration_search"
        )
        tail_planner.duration_config.update(
            {
                "bridge_duration_min_s": self.live_config.b_tail_duration_min_s,
                "bridge_duration_max_s": self.live_config.b_tail_duration_max_s,
                "bridge_duration_step_s": self.live_config.b_tail_duration_step_s,
            }
        )
        for planner in (initial_planner, tail_planner):
            effective_minimum, effective_maximum = (
                self._downstream_contract.intersect_workspace(
                    planner.workspace_min_mm,
                    planner.workspace_max_mm,
                )
            )
            planner.workspace_min_mm = effective_minimum
            planner.workspace_max_mm = effective_maximum
            planner.feasibility_config["axis_velocity_limit_mm_s"] = (
                self._downstream_contract.conservative_velocity_limit_mm_s
            )
        self._effective_bridge_workspace_min_mm = initial_planner.workspace_min_mm.copy()
        self._effective_bridge_workspace_max_mm = initial_planner.workspace_max_mm.copy()
        base_handoff = RuntimeHandoffConfig.from_dict(
            dict(self._manifest["runtime_handoff"])
        )
        handoff_values = asdict(base_handoff)
        # Prime by remaining Cartesian distance, not wall-clock phase.  A paused
        # or lagging physical bridge must not consume its own inference deadline.
        handoff_values["b_prime_endpoint_distance_mm"] = 90.0
        # The V1 opt-in may splice a moving ACT-B-matched tail first. The
        # zero-terminal-velocity Bridge remains intact until that tail passes
        # every check, so it is still the endpoint fallback on rejection.
        handoff_values["b_final_refresh_lead_s"] = (
            self.live_config.b_final_refresh_lead_s
        )
        handoff_values["b_endpoint_stop_before_final_refresh"] = (
            self.live_config.b_endpoint_stop_before_final_refresh
        )
        handoff_values["b_moving_overlap_primary_enabled"] = (
            self.live_config.b_moving_overlap_primary_enabled
        )
        handoff_values["b_endpoint_settle_position_tolerance_mm"] = (
            self.live_config.b_endpoint_settle_position_tolerance_mm
        )
        handoff_values["b_endpoint_settle_velocity_tolerance_mm_s"] = (
            self.live_config.b_endpoint_settle_velocity_tolerance_mm_s
        )
        handoff_values["b_endpoint_settle_min_hold_s"] = (
            self.live_config.b_endpoint_settle_min_hold_s
        )
        handoff_values["b_endpoint_settle_timeout_s"] = (
            self.live_config.b_endpoint_settle_timeout_s
        )
        handoff_values["b_overlap_search_max_skip_steps"] = (
            self.live_config.b_overlap_search_max_skip_steps
        )
        handoff_values["b_stopped_endpoint_direct_handoff_enabled"] = (
            self.live_config.b_stopped_endpoint_direct_handoff_enabled
        )
        handoff_values["b_stopped_endpoint_position_bridge_enabled"] = (
            self.live_config.b_stopped_endpoint_position_bridge_enabled
        )
        handoff_values["b_stopped_endpoint_direct_position_limit_mm"] = (
            self._downstream_contract.linear_ramp_mm_per_tick
        )
        handoff_values["bridge_progress_max_step_s"] = (
            1.0 / self._downstream_contract.control_hz
        )
        handoff_values["b_chunk_max_observation_age_s"] = max(
            float(handoff_values["b_chunk_max_observation_age_s"]), 3.0
        )
        self._handoff_config = RuntimeHandoffConfig(**handoff_values)
        self._coordinator = TaskCRealtimeCoordinator(
            policy_a=self._session_a_stub,
            policy_b=self._session_b,
            initial_planner=initial_planner,
            tail_planner=tail_planner,
            b_entries=list(entries),
            config=self._handoff_config,
        )
        with preserve_current_thread_affinity():
            self._session_b.warmup(
                self._capture(ctx)[0].policy_input,
                inferences=self._handoff_config.warmup_inferences,
            )
        self._coordinator.phase = RuntimePhase.A_RUNNING

        self._init_engine(ctx)
        self._engine.resume()
        self._event(
            "strategy_ready_external_gate_required",
            manifest=str(manifest_path),
            checkpoint_a=checkpoint_a,
            checkpoint_b=str(backend_b.checkpoint),
            both_models_resident=True,
            mux_selected_by_strategy=False,
            live_enabled_by_strategy=False,
            uncertified_manifest_explicitly_acknowledged=True,
            representative_boundary_enabled=(
                self._representative_boundary_tracker is not None
            ),
            representative_boundary=(
                None
                if self._representative_boundary_contract is None
                else self._representative_boundary_contract.to_record()
            ),
            representative_boundary_prearm_effect="read_only_planning_only",
            downstream_contract=asdict(self._downstream_contract),
            effective_bridge_workspace_min_mm=(
                self._effective_bridge_workspace_min_mm.tolist()
            ),
            effective_bridge_workspace_max_mm=(
                self._effective_bridge_workspace_max_mm.tolist()
            ),
            effective_bridge_velocity_limit_mm_s=(
                initial_planner.feasibility_config["velocity_limit_mm_s"]
            ),
            effective_bridge_axis_velocity_limit_mm_s=(
                initial_planner.feasibility_config["axis_velocity_limit_mm_s"]
            ),
            actual_tracking_position_hard_tolerance_mm=(
                self.live_config.actual_tracking_position_tolerance_mm
            ),
            actual_tracking_orientation_hard_tolerance_deg=(
                self.live_config.actual_tracking_orientation_tolerance_deg
            ),
            actual_tracking_position_admission_limit_mm=(
                self.live_config.actual_tracking_position_tolerance_mm
                - self.live_config.actual_tracking_position_admission_margin_mm
            ),
            actual_tracking_orientation_admission_limit_deg=(
                self.live_config.actual_tracking_orientation_tolerance_deg
                - self.live_config.actual_tracking_orientation_admission_margin_deg
            ),
            bridge_ack_mode=self.live_config.bridge_ack_mode,
            bridge_max_ack_lag_steps=(
                self.live_config.bridge_max_ack_lag_steps
            ),
            effective_b_final_refresh_lead_s=(
                self._handoff_config.b_final_refresh_lead_s
            ),
            b_endpoint_stop_before_final_refresh=(
                self._handoff_config.b_endpoint_stop_before_final_refresh
            ),
            b_moving_overlap_primary_enabled=(
                self._handoff_config.b_moving_overlap_primary_enabled
            ),
            b_mid_bridge_tail_seed_enabled=(
                self._handoff_config.b_moving_overlap_primary_enabled
            ),
            b_endpoint_fresh_inference_after_settle=(
                self._handoff_config.b_endpoint_stop_before_final_refresh
                and not self._handoff_config.b_moving_overlap_primary_enabled
            ),
            b_moving_overlap_primary_mode=(
                "acknowledged_bridge_tail_seed_then_fresh_execution_refresh"
                if self._handoff_config.b_moving_overlap_primary_enabled
                else "disabled"
            ),
            b_endpoint_stop_fallback_enabled=(
                self._handoff_config.b_endpoint_stop_before_final_refresh
            ),
            b_endpoint_settle_position_tolerance_mm=(
                self._handoff_config.b_endpoint_settle_position_tolerance_mm
            ),
            b_endpoint_settle_velocity_tolerance_mm_s=(
                self._handoff_config.b_endpoint_settle_velocity_tolerance_mm_s
            ),
            b_overlap_search_max_skip_steps=(
                self._handoff_config.b_overlap_search_max_skip_steps
            ),
            b_stopped_endpoint_direct_handoff_enabled=(
                self._handoff_config.b_stopped_endpoint_direct_handoff_enabled
            ),
            b_stopped_endpoint_position_bridge_enabled=(
                self._handoff_config.b_stopped_endpoint_position_bridge_enabled
            ),
            b_stopped_endpoint_position_bridge_duration_preference=(
                "shortest_feasible"
            ),
            b_stopped_endpoint_direct_position_limit_mm=(
                self._handoff_config.b_stopped_endpoint_direct_position_limit_mm
            ),
            b_stopped_endpoint_direct_position_metric=(
                "max_abs_cartesian_axis"
            ),
            effective_b_tail_duration_search=dict(
                tail_planner.duration_config
            ),
            b_execution_mode="rolling_fresh_observation_inference",
            b_transition_mode=(
                "moving_overlap_primary_with_endpoint_stop_fallback"
                if self._handoff_config.b_moving_overlap_primary_enabled
                else (
                    "endpoint_boundary_fresh_act_b_with_alignment_bridge"
                    if self._handoff_config.b_stopped_endpoint_position_bridge_enabled
                    else "endpoint_stop_then_fresh_act_b"
                )
            ),
            b_refresh_queue_threshold=self.live_config.b_refresh_queue_threshold,
            b_refresh_overlap_steps=self.live_config.b_refresh_overlap_steps,
            b_execution_step_watchdog=self.live_config.b_execution_steps,
            b_execution_timeout_s=self.live_config.b_execution_timeout_s,
            b_semantic_completion={
                "mode": self.live_config.b_completion_mode,
                "completion_authority": (
                    "external_operator_or_planner"
                    if self.live_config.b_completion_mode == "successor_owned"
                    else "legacy_release_and_settle_heuristic"
                ),
                "release_open_threshold": (
                    self.live_config.b_release_open_threshold
                ),
                "release_stable_steps": self.live_config.b_release_stable_steps,
                "release_position_gate_enabled": (
                    self.live_config.b_release_position_gate_enabled
                ),
                "release_workspace_min_xyz_mm": (
                    list(self.live_config.b_release_workspace_min_xyz_mm)
                    if self.live_config.b_release_position_gate_enabled
                    else None
                ),
                "release_workspace_max_xyz_mm": (
                    list(self.live_config.b_release_workspace_max_xyz_mm)
                    if self.live_config.b_release_position_gate_enabled
                    else None
                ),
                "completion_position_gate_enabled": (
                    self.live_config.b_completion_position_gate_enabled
                ),
                "completion_workspace_min_xyz_mm": (
                    list(self.live_config.b_completion_workspace_min_xyz_mm)
                    if self.live_config.b_completion_position_gate_enabled
                    else None
                ),
                "completion_workspace_max_xyz_mm": (
                    list(self.live_config.b_completion_workspace_max_xyz_mm)
                    if self.live_config.b_completion_position_gate_enabled
                    else None
                ),
                "completion_stable_frames": (
                    self.live_config.b_completion_stable_frames
                ),
                "completion_velocity_tolerance_mm_s": (
                    self.live_config.b_completion_velocity_tolerance_mm_s
                ),
                "required_gripper_driver_evidence": [
                    "completed_command=open",
                    "driver_busy=false",
                    "last_command_ok=true",
                ],
            },
        )

    def _observe_actual(self, capture: _LiveCapture, state: np.ndarray) -> None:
        xyz = capture.tcp_position_mm
        self._actual_history.add(capture.capture_completed_s, xyz)
        if self._last_actual_xyz is not None:
            self._a_path_length_mm += float(np.linalg.norm(xyz - self._last_actual_xyz))
        self._last_actual_xyz = xyz.copy()
        if self.phase in {LivePhase.BRIDGE, LivePhase.ACT_B}:
            self._assert_tracking(state[6:12])

    def _estimate_actual_boundary_velocity(self) -> np.ndarray | None:
        """Return one causal TCP estimate, or None until three samples exist."""

        assert self._handoff_config is not None
        if len(self._actual_history) < 3:
            return None
        try:
            return self._actual_history.estimate(
                window_frames=self._handoff_config.velocity_window_frames,
                method=self._handoff_config.velocity_smoothing_method,
                velocity_epsilon=self._handoff_config.velocity_epsilon,
            )
        except ValueError:
            return None

    def _read_only_representative_preplan(
        self,
        capture: _LiveCapture,
        measured_velocity_mm_s: np.ndarray,
    ) -> None:
        """Probe Bridge feasibility at 40 mm without mutating command state."""

        assert self._coordinator is not None
        assert self._handoff_config is not None
        cache = getattr(self._raw_robot, "cache", None)
        if cache is None:
            self._event(
                "representative_boundary_preplan_failed",
                reason="commanded_posx_cache_unavailable",
            )
            return
        self._representative_preplan_attempts += 1
        commands_before = dict(self._commands_by_phase)
        pending_before = self._pending_command
        started_s = time.monotonic()
        try:
            streamer_sample = cache.require(
                "commanded_posx",
                max_age_sec=self.live_config.downstream_command_max_age_s,
                now=started_s,
            )
            streamer_pose = np.asarray(streamer_sample.value, dtype=np.float64)
            if streamer_pose.shape != (6,) or not np.all(np.isfinite(streamer_pose)):
                raise ValueError("ServoL commanded_posx must be finite shape (6,)")
            endpoint_stop = (
                self._handoff_config.b_final_refresh_enabled
                and self._handoff_config.b_endpoint_stop_before_final_refresh
            )
            result = self._coordinator.initial_planner.plan(
                position_a_mm=streamer_pose[:3],
                velocity_a_mm_s=measured_velocity_mm_s,
                semantic_state_a=capture.semantic_state,
                a_retained_length_mm=self._a_path_length_mm,
                b_entries=self._coordinator.b_entries,
                terminal_velocity_override_mm_s=(
                    np.zeros(3, dtype=np.float64) if endpoint_stop else None
                ),
                terminal_velocity_source=(
                    "planned_endpoint_stop"
                    if endpoint_stop
                    else "offline_representative_demonstration"
                ),
            )
            if (
                self._commands_by_phase != commands_before
                or self._pending_command is not pending_before
            ):
                raise RuntimeError("representative preplan mutated command state")
            self._representative_preplan_succeeded = True
            self._event(
                "representative_boundary_preplan_ready",
                read_only=True,
                policy_queue_mutated=False,
                command_count_delta=0,
                planning_latency_ms=(time.monotonic() - started_s) * 1000.0,
                candidates_evaluated=result.candidates_evaluated,
                entry_id=result.selected.entry.entry_id,
                duration_s=result.selected.bridge.duration_s,
                bridge_start_position_source="acknowledged_commanded_posx",
                bridge_start_velocity_source="causal_measured_tcp_history",
            )
        except (ValueError, NoFeasibleBridgeError, RuntimeError) as exc:
            if (
                self._commands_by_phase != commands_before
                or self._pending_command is not pending_before
            ):
                raise RuntimeError(
                    "representative preplan failure mutated command state"
                ) from exc
            self._representative_preplan_succeeded = False
            self._event(
                "representative_boundary_preplan_failed",
                read_only=True,
                policy_queue_mutated=False,
                command_count_delta=0,
                planning_latency_ms=(time.monotonic() - started_s) * 1000.0,
                reason=f"{type(exc).__name__}: {exc}",
            )

    def _begin_bridge(
        self,
        ctx: Any,
        capture: _LiveCapture,
        state: np.ndarray,
    ) -> None:
        assert self._coordinator is not None
        assert self._handoff_config is not None
        representative_mode = self._representative_boundary_tracker is not None
        if not representative_mode:
            # Preserve the reviewed V0 ordering exactly.
            self._engine.pause()
            self._engine.reset()

        positions, timestamps = self._actual_history.arrays(
            self._handoff_config.velocity_window_frames
        )
        self._coordinator.history.clear()
        for position, timestamp in zip(positions, timestamps, strict=True):
            self._coordinator.history.add(float(timestamp), position)
        measured_velocity = self._coordinator.history.estimate(
            window_frames=self._handoff_config.velocity_window_frames,
            method=self._handoff_config.velocity_smoothing_method,
            velocity_epsilon=self._handoff_config.velocity_epsilon,
        )
        cache = getattr(self._raw_robot, "cache", None)
        if cache is None:
            raise RuntimeError("ServoL commanded_posx cache is unavailable")
        streamer_sample = cache.require(
            "commanded_posx",
            max_age_sec=self.live_config.downstream_command_max_age_s,
            now=time.monotonic(),
        )
        streamer_pose = np.asarray(streamer_sample.value, dtype=np.float64)
        if streamer_pose.shape != (6,) or not np.all(np.isfinite(streamer_pose)):
            raise RuntimeError("ServoL commanded_posx must be finite shape (6,)")
        tracking_position_error, tracking_orientation_error = pose_delta_metrics(
            state[6:12],
            streamer_pose,
        )
        if (
            tracking_position_error
            > self.live_config.actual_tracking_position_tolerance_mm
        ):
            raise RuntimeError(
                "A-to-Bridge command tracking position error "
                f"{tracking_position_error:.3f} mm"
            )
        if (
            tracking_orientation_error
            > self.live_config.actual_tracking_orientation_tolerance_deg
        ):
            raise RuntimeError(
                "A-to-Bridge command tracking orientation error "
                f"{tracking_orientation_error:.3f} deg"
            )
        final_validation = None
        if representative_mode:
            endpoint_stop = (
                self._handoff_config.b_final_refresh_enabled
                and self._handoff_config.b_endpoint_stop_before_final_refresh
            )
            final_validation = self._coordinator.initial_planner.plan(
                position_a_mm=streamer_pose[:3],
                velocity_a_mm_s=measured_velocity,
                semantic_state_a=capture.semantic_state,
                a_retained_length_mm=self._a_path_length_mm,
                b_entries=self._coordinator.b_entries,
                terminal_velocity_override_mm_s=(
                    np.zeros(3, dtype=np.float64) if endpoint_stop else None
                ),
                terminal_velocity_source=(
                    "planned_endpoint_stop"
                    if endpoint_stop
                    else "offline_representative_demonstration"
                ),
            )
            # Only now may the ACT-A producer/queue be stopped. The final
            # feasibility decision above is pure and issued no command.
            self._engine.pause()
            self._engine.reset()
        cut_observation = RuntimeObservation(
            timestamp_s=capture.capture_completed_s,
            tcp_position_mm=capture.tcp_position_mm,
            semantic_state=capture.semantic_state,
            policy_input=capture.policy_input,
            acknowledged_command_position_mm=streamer_pose[:3],
            acknowledged_command_velocity_mm_s=measured_velocity,
        )
        self._coordinator.phase = RuntimePhase.A_RUNNING
        try:
            plan = self._coordinator.request_cut(
                cut_observation,
                semantic_cut_valid=True,
                a_retained_length_mm=self._a_path_length_mm,
            )
        except (ValueError, NoFeasibleBridgeError) as exc:
            raise RuntimeError(f"initial bridge planning failed: {exc}") from exc
        if final_validation is not None:
            expected = final_validation.selected
            if (
                not np.isclose(
                    expected.bridge.duration_s,
                    plan.bridge.duration_s,
                    atol=1e-12,
                    rtol=0.0,
                )
                or not np.allclose(
                    np.stack(
                        (
                            expected.bridge.p0,
                            expected.bridge.p1,
                            expected.bridge.p2,
                            expected.bridge.p3,
                        )
                    ),
                    np.stack(
                        (plan.bridge.p0, plan.bridge.p1, plan.bridge.p2, plan.bridge.p3)
                    ),
                    atol=1e-8,
                    rtol=0.0,
                )
            ):
                raise RuntimeError(
                    "representative final validation changed after ACT-A queue clear"
                )
        if not np.allclose(
            plan.bridge.p0,
            streamer_pose[:3],
            atol=1e-8,
            rtol=0.0,
        ):
            raise RuntimeError(
                "initial Bridge p0 differs from acknowledged commanded_posx"
            )

        self._orientation_bridge = OrientationBridge(
            streamer_pose[3:6],
            streamer_pose[3:6],
            duration_s=plan.bridge.duration_s,
        )
        stream_assessment = self._assess_bridge_passthrough(
            plan.bridge,
            streamer_pose[3:6],
            streamer_pose[3:6],
        )
        self.phase = LivePhase.BRIDGE
        self._pending_command = None
        self._last_bridge_commanded_receive_count = None
        self._reset_bridge_ack_tracking(
            streamer_pose,
            int(streamer_sample.receive_count),
        )
        self._transition_started_s = capture.capture_completed_s
        initial_hold = full_action(
            plan.bridge.p0,
            streamer_pose[3:6],
            gripper_target=1.0,
        )
        self._send_array(
            ctx,
            initial_hold,
            "BEZIER_BRIDGE",
            actual_pose=state[6:12],
        )
        self._event(
            "act_a_to_bridge",
            cut_pose_mm_deg=state[6:12].tolist(),
            acknowledged_command_pose_mm_deg=streamer_pose.tolist(),
            actual_to_acknowledged_position_error_mm=tracking_position_error,
            actual_to_acknowledged_orientation_error_deg=(
                tracking_orientation_error
            ),
            bridge_start_position_source="acknowledged_commanded_posx",
            bridge_start_velocity_source="measured_tcp_history",
            initial_bridge_hold_command_sent=True,
            measured_velocity_mm_s=self._coordinator.a_measured_velocity_mm_s.tolist(),
            initial_bridge_duration_s=plan.bridge.duration_s,
            initial_bridge_endpoint_mm=plan.bridge.p3.tolist(),
            initial_bridge_terminal_velocity_mm_s=(
                plan.bridge.velocity(1.0).tolist()
            ),
            initial_bridge_terminal_velocity_source=(
                plan.terminal_velocity_source
            ),
            downstream_stream_assessment=asdict(stream_assessment),
            a_commands=self._commands_by_phase[LivePhase.ACT_A.value],
            representative_boundary_enabled=representative_mode,
            representative_boundary_status=(
                None
                if self._representative_boundary_status is None
                else asdict(self._representative_boundary_status)
            ),
            representative_preplan_attempts=self._representative_preplan_attempts,
            representative_preplan_succeeded=(
                self._representative_preplan_succeeded
            ),
            final_bridge_validated_before_act_a_queue_clear=(
                final_validation is not None
            ),
        )

    def _step_bridge(self, ctx: Any, capture: _LiveCapture, state: np.ndarray) -> None:
        assert self._coordinator is not None
        previous_generation = self._coordinator.b_generation
        previous_plan = self._coordinator.current_plan
        previous_elapsed_s = self._coordinator.bridge_elapsed_s
        previous_acknowledged_pose = (
            None
            if self._last_acknowledged_bridge_pose is None
            else self._last_acknowledged_bridge_pose.copy()
        )
        previous_coordinator_event_count = len(self._coordinator.events)
        proposal = self._coordinator.step(self._runtime_observation(capture, state))
        for coordinator_event in self._coordinator.events[
            previous_coordinator_event_count:
        ]:
            if coordinator_event.event in {
                "act_b_tail_seed_rejected_existing_bridge_retained",
                "act_b_fresh_generation_requested_during_bridge",
                "act_b_final_near_entry_refresh_requested",
                "bridge_endpoint_stop_hold_started",
                "act_b_endpoint_settled_refresh_requested",
                "bridge_tail_planning_failed",
                "act_b_stopped_endpoint_direct_handoff_ready",
                "act_b_stopped_endpoint_direct_handoff_rejected",
                "act_b_stopped_endpoint_position_bridge_ready",
                "act_b_stopped_endpoint_position_bridge_rejected",
                "atomic_bridge_to_act_b_handoff",
            }:
                self._event(
                    coordinator_event.event,
                    coordinator_timestamp_s=coordinator_event.timestamp_s,
                    **dict(coordinator_event.details),
                )
        if self._coordinator.phase is RuntimePhase.FAILED_HOLD:
            assessment = self._coordinator.b_assessment
            if assessment is not None:
                tail_failure = next(
                    (
                        event
                        for event in reversed(self._coordinator.events)
                        if event.event == "bridge_tail_planning_failed"
                    ),
                    None,
                )
                self._event(
                    "act_b_position_rejected",
                    generation=self._coordinator.b_generation,
                    generation_role=self._coordinator.b_generation_role,
                    assessment={
                        "valid": assessment.valid,
                        "failure_reasons": list(assessment.failure_reasons),
                        "intended_velocity_mm_s": (
                            assessment.intended_velocity_mm_s.tolist()
                        ),
                        "first_position_jump_mm": (
                            assessment.first_position_jump_mm
                        ),
                        "max_predicted_velocity_mm_s": (
                            assessment.max_predicted_velocity_mm_s
                        ),
                        "velocity_window_steps": assessment.velocity_window_steps,
                        "velocity_method": assessment.velocity_method,
                    },
                    velocity_sample_source=(
                        "consecutive_postprocessed_act_b_targets"
                    ),
                    act_b_first_xyz_mm=(
                        None
                        if self._coordinator.b_chunk is None
                        else self._coordinator.b_chunk.first_xyz_mm.tolist()
                    ),
                    b_prime_position_mm=(
                        None
                        if self._coordinator.b_prime_position_mm is None
                        else self._coordinator.b_prime_position_mm.tolist()
                    ),
                    tail_planning_failure=(
                        None if tail_failure is None else dict(tail_failure.details)
                    ),
                )
            raise RuntimeError(
                "coordinator failed: " + ",".join(self._coordinator.failure_reasons)
            )

        if (
            self._coordinator.b_generation != previous_generation
            and self._coordinator.b_generation is not None
        ):
            self._event(
                "act_b_inference_requested",
                generation=self._coordinator.b_generation,
                generation_role=self._coordinator.b_generation_role,
            )

        if self._coordinator.current_plan is not previous_plan:
            chunk = self._coordinator.b_chunk
            plan = self._coordinator.current_plan
            if chunk is None or plan is None:
                raise RuntimeError("B-ready replan lacks chunk or bridge")
            if previous_plan is None or previous_acknowledged_pose is None:
                raise RuntimeError(
                    "B-ready replan lacks an acknowledged Bridge splice state"
                )
            previous_u = min(
                1.0,
                max(0.0, previous_elapsed_s / previous_plan.bridge.duration_s),
            )
            expected_splice_position = previous_acknowledged_pose[:3]
            expected_splice_velocity = previous_plan.bridge.velocity(previous_u)
            if not np.allclose(
                plan.bridge.p0,
                expected_splice_position,
                atol=1e-8,
                rtol=0.0,
            ):
                raise RuntimeError(
                    "ACT-B tail p0 differs from acknowledged commanded_posx"
                )
            if not np.allclose(
                plan.bridge.velocity(0.0),
                expected_splice_velocity,
                atol=1e-8,
                rtol=1e-8,
            ):
                raise RuntimeError(
                    "ACT-B tail v0 differs from the acknowledged Bridge tangent"
                )
            assessment = assess_policy_pose_chunk(
                chunk,
                state[6:12],
                window_steps=self._handoff_config.b_chunk_velocity_window_steps,
                first_position_jump_limit_mm=(
                    self._handoff_config.b_first_action_position_jump_limit_mm
                ),
                first_orientation_jump_limit_deg=(
                    self.live_config.b_first_orientation_jump_limit_deg
                ),
                predicted_velocity_limit_mm_s=(
                    self._handoff_config.b_predicted_velocity_limit_mm_s
                ),
                predicted_angular_velocity_limit_deg_s=(
                    self.live_config.b_predicted_angular_velocity_limit_deg_s
                ),
            )
            if not assessment.valid:
                self._event(
                    "act_b_full_pose_rejected",
                    generation=chunk.generation,
                    generation_role=self._coordinator.b_generation_role,
                    assessment=asdict(assessment),
                    velocity_sample_source=(
                        "consecutive_postprocessed_act_b_targets"
                    ),
                )
                raise RuntimeError(
                    "ACT-B full-pose assessment failed: "
                    + ",".join(assessment.failure_reasons)
                )
            position_assessment = self._coordinator.b_assessment
            if position_assessment is None:
                raise RuntimeError("ACT-B position assessment is unavailable")
            if np.any(
                np.abs(position_assessment.intended_velocity_mm_s)
                > self._downstream_contract.conservative_velocity_limit_mm_s
                + 1e-9
            ):
                raise RuntimeError(
                    "ACT-B terminal velocity exceeds ServoL axis capacity"
                )
            self._orientation_bridge = OrientationBridge(
                previous_acknowledged_pose[3:6],
                chunk.actions[0, 3:6],
                duration_s=plan.bridge.duration_s,
            )
            stream_assessment = self._assess_bridge_passthrough(
                plan.bridge,
                previous_acknowledged_pose[3:6],
                chunk.actions[0, 3:6],
            )
            self._event(
                "bridge_replanned_to_exact_act_b_pose",
                generation=chunk.generation,
                generation_role=self._coordinator.b_generation_role,
                b_action_start_index=(
                    self._coordinator.b_action_start_index
                ),
                b_skipped_prefix_duration_s=(
                    self._coordinator.b_action_start_index / chunk.action_hz
                ),
                bridge_endpoint_mm=plan.bridge.p3.tolist(),
                act_b_first_xyz_mm=chunk.first_xyz_mm.tolist(),
                act_b_first_orientation_deg=chunk.actions[0, 3:6].tolist(),
                full_pose_assessment=asdict(assessment),
                terminal_axis_velocity_mm_s=(
                    position_assessment.intended_velocity_mm_s.tolist()
                ),
                planned_terminal_axis_velocity_mm_s=(
                    plan.bridge.velocity(1.0).tolist()
                ),
                splice_position_mm=plan.bridge.p0.tolist(),
                splice_velocity_mm_s=plan.bridge.velocity(0.0).tolist(),
                actual_to_splice_position_error_mm=float(
                    np.linalg.norm(state[6:9] - plan.bridge.p0)
                ),
                splice_position_source="acknowledged_commanded_posx",
                splice_velocity_source="acknowledged_bridge_tangent",
                atomic_splice_at_acknowledged_tick=True,
                moving_overlap_primary=(
                    self._handoff_config.b_moving_overlap_primary_enabled
                    and self._coordinator.b_generation_role == "tail_seed"
                ),
                stopped_endpoint_position_bridge=(
                    self._coordinator.b_stopped_endpoint_position_bridge_handoff
                ),
                handoff_mode=(
                    "stopped_endpoint_position_bridge"
                    if self._coordinator.b_stopped_endpoint_position_bridge_handoff
                    else "velocity_matched_tail"
                ),
                endpoint_stop_fallback_consumed=(
                    (
                        self._handoff_config.b_moving_overlap_primary_enabled
                        and self._coordinator.b_generation_role == "tail_seed"
                    )
                    or self._coordinator.b_stopped_endpoint_position_bridge_handoff
                ),
                velocity_sample_source=(
                    "consecutive_postprocessed_act_b_targets"
                ),
                downstream_stream_assessment=asdict(stream_assessment),
            )

        if proposal.source == "BEZIER_BRIDGE":
            assert self._orientation_bridge is not None
            orientation = self._orientation_bridge.sample(
                self._coordinator.bridge_elapsed_s
            )
            action = full_action(
                proposal.xyz_mm,
                orientation,
                gripper_target=1.0,
            )
            self._submit_live_command(
                ctx,
                action,
                "BEZIER_BRIDGE",
                actual_pose=state[6:12],
                enforce_stream_ramp=True,
            )
            return

        if proposal.source == "ACT-B":
            chunk = self._coordinator.b_chunk
            if chunk is None:
                raise RuntimeError("atomic B handoff lacks a chunk")
            if (
                self._coordinator.b_stopped_endpoint_direct_handoff
                and self._b_direct_handoff_validated_generation
                != chunk.generation
            ):
                direct_assessment = self._assess_b_refresh_chunk(
                    chunk,
                    state[6:12],
                )
                reference_pose = (
                    state[6:12]
                    if self._last_command_pose is None
                    else self._last_command_pose
                )
                first_axis_step = np.abs(
                    chunk.actions[0, :3] - reference_pose[:3]
                )
                _unused, first_orientation_step = pose_delta_metrics(
                    reference_pose,
                    chunk.actions[0, :6],
                )
                self._event(
                    "act_b_stopped_endpoint_direct_full_pose_validated",
                    generation=chunk.generation,
                    generation_role=self._coordinator.b_generation_role,
                    full_pose_assessment=asdict(direct_assessment),
                    reference_command_pose_mm_deg=reference_pose.tolist(),
                    first_target_pose_mm_deg=chunk.actions[0, :6].tolist(),
                    first_axis_step_mm=first_axis_step.tolist(),
                    first_orientation_step_deg=first_orientation_step,
                    linear_ramp_limit_mm_per_tick=(
                        self._downstream_contract.linear_ramp_mm_per_tick
                    ),
                    orientation_ramp_limit_deg_per_tick=(
                        self._downstream_contract.orientation_ramp_deg_per_tick
                    ),
                    workspace_checked=True,
                    tracking_admission_checked=True,
                    safety_limits_relaxed=False,
                )
                self._b_direct_handoff_validated_generation = chunk.generation
            endpoint_position_error, endpoint_orientation_error = pose_delta_metrics(
                state[6:12], chunk.actions[0, :6]
            )
            if endpoint_position_error > self._handoff_config.handoff_position_tolerance_mm:
                raise RuntimeError("ACT-B handoff position tolerance exceeded")
            if endpoint_orientation_error > self.live_config.handoff_orientation_tolerance_deg:
                raise RuntimeError("ACT-B handoff orientation tolerance exceeded")
            self._submit_live_command(
                ctx,
                proposal.full_policy_action,
                "ACT-B",
                actual_pose=state[6:12],
                enforce_stream_ramp=True,
                transition_to_act_b=True,
            )

    def _bridge_streamer_acknowledged(self) -> bool:
        previous_count = self._last_bridge_commanded_receive_count
        cache = getattr(self._raw_robot, "cache", None)
        if cache is None:
            raise RuntimeError("ServoL commanded_posx cache is unavailable")
        sample = cache.require(
            "commanded_posx",
            max_age_sec=self.live_config.downstream_command_max_age_s,
            now=time.monotonic(),
        )
        commanded = np.asarray(sample.value, dtype=np.float64)
        if commanded.shape != (6,) or not np.all(np.isfinite(commanded)):
            raise RuntimeError("ServoL commanded_posx must be finite shape (6,)")
        if self._bounded_bridge_ack_pipeline_enabled():
            return self._bounded_bridge_streamer_can_advance(
                commanded,
                receive_count=int(sample.receive_count),
            )
        if previous_count is None:
            # Capture the command-space anchor before the first Bridge proposal.
            # This closes the race where an unusually fast tail-seed inference
            # could otherwise complete before any Bridge command was acknowledged.
            self._last_acknowledged_bridge_pose = commanded.copy()
            self._last_acknowledged_bridge_receive_count = int(sample.receive_count)
            return True
        if self._last_command_pose is None:
            raise RuntimeError("Bridge acknowledgement lacks the prior command pose")
        if sample.receive_count <= previous_count:
            self._bridge_ack_no_fresh_cycles += 1
            self._bridge_ack_wait_cycles += 1
            return False
        position_error, orientation_error = pose_delta_metrics(
            commanded,
            self._last_command_pose,
        )
        if (
            position_error
            > self.live_config.downstream_ack_position_tolerance_mm
            or orientation_error
            > self.live_config.downstream_ack_orientation_tolerance_deg
        ):
            self._bridge_ack_pose_mismatch_cycles += 1
            self._bridge_ack_wait_cycles += 1
            return False
        self._last_acknowledged_bridge_pose = commanded.copy()
        self._last_acknowledged_bridge_receive_count = int(sample.receive_count)
        return True

    def _bounded_bridge_streamer_can_advance(
        self,
        commanded_pose_mm_deg: np.ndarray,
        *,
        receive_count: int,
    ) -> bool:
        commanded = np.asarray(commanded_pose_mm_deg, dtype=np.float64)
        if commanded.shape != (6,) or not np.all(np.isfinite(commanded)):
            raise ValueError("bounded Bridge ACK must be finite shape (6,)")
        if self._bridge_ack_anchor_pose is None:
            self._reset_bridge_ack_tracking(commanded, receive_count)
            return True

        observed_count = self._bridge_ack_last_observed_receive_count
        new_sample = observed_count is None or receive_count > observed_count
        if new_sample:
            matched_command: _BridgeSentCommand | None = None
            for sent in reversed(self._bridge_ack_sent_history):
                if sent.sequence <= self._bridge_ack_acknowledged_sequence:
                    break
                if receive_count <= sent.receive_count_before_send:
                    continue
                position_error, orientation_error = pose_delta_metrics(
                    commanded,
                    sent.pose_mm_deg,
                )
                if (
                    position_error
                    <= self.live_config.downstream_ack_position_tolerance_mm
                    and orientation_error
                    <= self.live_config.downstream_ack_orientation_tolerance_deg
                ):
                    matched_command = sent
                    break

            matches_current_ack = False
            if (
                matched_command is None
                and self._last_acknowledged_bridge_pose is not None
            ):
                position_error, orientation_error = pose_delta_metrics(
                    commanded,
                    self._last_acknowledged_bridge_pose,
                )
                matches_current_ack = bool(
                    position_error
                    <= self.live_config.downstream_ack_position_tolerance_mm
                    and orientation_error
                    <= self.live_config.downstream_ack_orientation_tolerance_deg
                )

            self._bridge_ack_last_observed_receive_count = receive_count
            if matched_command is not None:
                self._bridge_ack_acknowledged_sequence = (
                    matched_command.sequence
                )
                self._last_acknowledged_bridge_pose = commanded.copy()
                self._last_acknowledged_bridge_receive_count = receive_count
                self._bridge_ack_unresolved_mismatch = False
            elif matches_current_ack:
                self._last_acknowledged_bridge_pose = commanded.copy()
                self._last_acknowledged_bridge_receive_count = receive_count
                self._bridge_ack_unresolved_mismatch = False
            else:
                self._bridge_ack_pose_mismatch_cycles += 1
                self._bridge_ack_wait_cycles += 1
                self._bridge_ack_unresolved_mismatch = True
                return False
        else:
            self._bridge_ack_no_fresh_cycles += 1

        if self._bridge_ack_unresolved_mismatch:
            self._bridge_ack_wait_cycles += 1
            return False

        lag = (
            self._bridge_ack_sent_sequence
            - self._bridge_ack_acknowledged_sequence
        )
        self._bridge_ack_max_decision_lag_steps = max(
            self._bridge_ack_max_decision_lag_steps,
            lag,
        )
        if lag > self.live_config.bridge_max_ack_lag_steps:
            self._bridge_ack_lag_hold_cycles += 1
            self._bridge_ack_wait_cycles += 1
            return False
        if lag > 0:
            self._bridge_ack_pipeline_advance_cycles += 1
        return True

    @staticmethod
    def _chunk_with_actions(chunk: PolicyChunk, actions: np.ndarray) -> PolicyChunk:
        return PolicyChunk(
            policy_id=chunk.policy_id,
            generation=chunk.generation,
            observation_timestamp_s=chunk.observation_timestamp_s,
            completed_timestamp_s=chunk.completed_timestamp_s,
            inference_latency_s=chunk.inference_latency_s,
            action_hz=chunk.action_hz,
            actions=actions,
            request_timestamp_s=chunk.request_timestamp_s,
            snapshot_completed_timestamp_s=chunk.snapshot_completed_timestamp_s,
            submitted_timestamp_s=chunk.submitted_timestamp_s,
            worker_started_timestamp_s=chunk.worker_started_timestamp_s,
            gpu_acquired_timestamp_s=chunk.gpu_acquired_timestamp_s,
        )

    def _request_b_refresh(self, capture: _LiveCapture) -> None:
        assert self._session_b is not None
        if self._b_refresh_generation is not None:
            return
        active_generation = self._session_b.active_generation
        if active_generation is None:
            raise RuntimeError("ACT-B refresh requires an active generation")
        queue_size = self._session_b.queue_size
        self._b_refresh_queue_consumed_at_request = (
            self._session_b.stats().queue_actions_consumed
        )
        self._b_refresh_requested_s = time.monotonic()
        self._b_refresh_generation = self._session_b.prime(
            capture.policy_input,
            observation_timestamp_s=capture.capture_completed_s,
            preserve_active=True,
        )
        self._event(
            "act_b_rolling_inference_requested",
            generation=self._b_refresh_generation,
            replaces_generation=active_generation,
            queue_size_at_request=queue_size,
            capture_timestamp_s=capture.capture_completed_s,
            policy_input_ready_timestamp_s=capture.policy_input_ready_s,
        )

    def _assess_b_refresh_chunk(
        self,
        chunk: PolicyChunk,
        actual_pose: np.ndarray,
    ) -> Any:
        assert self._handoff_config is not None
        assessment = assess_policy_pose_chunk(
            chunk,
            actual_pose,
            window_steps=len(chunk.actions),
            first_position_jump_limit_mm=(
                self.live_config.actual_tracking_position_tolerance_mm
                - self.live_config.actual_tracking_position_admission_margin_mm
            ),
            first_orientation_jump_limit_deg=(
                self.live_config.actual_tracking_orientation_tolerance_deg
                - self.live_config.actual_tracking_orientation_admission_margin_deg
            ),
            predicted_velocity_limit_mm_s=(
                self._handoff_config.b_predicted_velocity_limit_mm_s
            ),
            predicted_axis_velocity_limit_mm_s=(
                self._downstream_contract.conservative_velocity_limit_mm_s
            ),
            predicted_angular_velocity_limit_deg_s=(
                self._downstream_contract.angular_velocity_limit_deg_s
            ),
        )
        reasons = list(assessment.failure_reasons)
        reference = (
            np.asarray(actual_pose, dtype=np.float64)
            if self._last_command_pose is None
            else self._last_command_pose
        )
        first_axis_step = np.abs(chunk.actions[0, :3] - reference[:3])
        _unused, first_orientation_step = pose_delta_metrics(
            reference,
            chunk.actions[0, :6],
        )
        if np.any(
            first_axis_step
            > self._downstream_contract.linear_ramp_mm_per_tick + 1e-9
        ):
            reasons.append("refresh_first_target_linear_ramp")
        if (
            first_orientation_step
            > self._downstream_contract.orientation_ramp_deg_per_tick + 1e-9
        ):
            reasons.append("refresh_first_target_orientation_ramp")

        minimum = np.asarray(
            self._downstream_contract.workspace_min_xyz_mm,
            dtype=np.float64,
        )
        maximum = np.asarray(
            self._downstream_contract.workspace_max_xyz_mm,
            dtype=np.float64,
        )
        minimum_enabled = np.asarray(
            self._downstream_contract.workspace_min_limit_enabled,
            dtype=np.bool_,
        )
        xyz = chunk.actions[:, :3]
        if (
            np.any(xyz[:, minimum_enabled] < minimum[minimum_enabled] - 1e-9)
            or np.any(xyz > maximum + 1e-9)
        ):
            reasons.append("refresh_chunk_workspace_clamp")

        anchor = (
            np.asarray(actual_pose[3:6], dtype=np.float64)
            if self._live_orientation_anchor is None
            else self._live_orientation_anchor
        )
        maximum_anchor_error = 0.0
        for pose in chunk.actions[:, :6]:
            _unused, anchor_error = pose_delta_metrics(
                np.concatenate((pose[:3], anchor)),
                pose,
            )
            maximum_anchor_error = max(maximum_anchor_error, anchor_error)
        if (
            maximum_anchor_error
            > self._downstream_contract.orientation_limit_deg + 1e-9
        ):
            reasons.append("refresh_chunk_orientation_clamp")

        if reasons:
            self._event(
                "act_b_rolling_chunk_rejected",
                generation=chunk.generation,
                failure_reasons=reasons,
                assessment=asdict(assessment),
                first_axis_step_mm=first_axis_step.tolist(),
                first_orientation_step_deg=first_orientation_step,
                max_orientation_from_anchor_deg=maximum_anchor_error,
            )
            raise RuntimeError(
                "ACT-B rolling chunk failed: " + ",".join(dict.fromkeys(reasons))
            )
        return assessment

    def _poll_b_refresh(
        self,
        capture: _LiveCapture,
        state: np.ndarray,
    ) -> None:
        assert self._session_b is not None
        generation = self._b_refresh_generation
        if generation is None:
            return
        requested_s = self._b_refresh_requested_s
        if requested_s is None:
            raise RuntimeError("ACT-B refresh lacks a request timestamp")
        try:
            chunk = self._session_b.poll()
        except PolicyInferenceError as exc:
            raise RuntimeError("ACT-B rolling inference failed") from exc
        if chunk is None:
            if time.monotonic() - requested_s > self.live_config.b_refresh_timeout_s:
                raise TimeoutError("ACT-B rolling inference timeout")
            return
        if chunk.generation != generation:
            raise RuntimeError("ACT-B rolling inference generation mismatch")
        observation_age_s = chunk.observation_age_s(capture.capture_completed_s)
        if observation_age_s > self.live_config.b_refresh_max_observation_age_s:
            raise RuntimeError(
                f"ACT-B rolling chunk stale: {observation_age_s:.3f}s"
            )

        consumed_now = self._session_b.stats().queue_actions_consumed
        delay_steps = max(
            0,
            consumed_now - self._b_refresh_queue_consumed_at_request,
        )
        if delay_steps >= len(chunk.actions) - 1:
            raise RuntimeError("ACT-B rolling inference consumed its prediction horizon")
        new_actions = chunk.actions[delay_steps:].copy()
        old_chunk = self._session_b.active_remaining_chunk()
        if old_chunk is None:
            if self._last_command_action is None:
                raise RuntimeError("ACT-B rolling swap lacks an old command anchor")
            old_actions = self._last_command_action[None, :].copy()
        else:
            old_actions = old_chunk.actions
        overlap_steps = min(
            self.live_config.b_refresh_overlap_steps,
            len(old_actions),
            len(new_actions),
        )
        merged_actions = _blend_b_action_overlap(
            old_actions,
            new_actions,
            overlap_steps,
        )
        merged_chunk = self._chunk_with_actions(chunk, merged_actions)
        assessment = self._assess_b_refresh_chunk(
            merged_chunk,
            state[6:12],
        )
        prior_generation = self._session_b.active_generation
        self._session_b.activate(
            generation,
            actions=merged_actions,
        )
        self._b_refresh_count += 1
        self._event(
            "act_b_rolling_queue_swapped",
            generation=generation,
            replaced_generation=prior_generation,
            delay_compensation_steps=delay_steps,
            overlap_steps=overlap_steps,
            old_remaining_steps=len(old_actions),
            new_raw_steps=len(chunk.actions),
            active_queue_steps=self._session_b.queue_size,
            observation_age_s=observation_age_s,
            request_to_completion_ms=chunk.request_to_completion_latency_s * 1000.0,
            gpu_wait_ms=chunk.gpu_wait_latency_s * 1000.0,
            backend_inference_ms=chunk.backend_inference_latency_s * 1000.0,
            assessment=asdict(assessment),
        )
        self._b_refresh_generation = None
        self._b_refresh_requested_s = None
        self._b_refresh_queue_consumed_at_request = 0
        self._b_refresh_hold_cycles = 0

    def _send_b_refresh_hold(
        self,
        ctx: Any,
        *,
        actual_pose: np.ndarray,
    ) -> None:
        if self._last_command_action is None:
            raise RuntimeError("ACT-B refresh hold lacks a prior command")
        hold = self._last_command_action.copy()
        hold[6] = 0.0 if self._b_release_authorized else 1.0
        self._send_array(
            ctx,
            hold,
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

    def _prepare_b_action(
        self,
        action: np.ndarray,
        state: np.ndarray,
    ) -> np.ndarray:
        values = np.asarray(action, dtype=np.float64).copy()
        if values.shape != (7,) or not np.all(np.isfinite(values)):
            raise ValueError("ACT-B action must be finite shape (7,)")
        raw_gripper_target = float(values[6])
        if self._b_release_authorized:
            values[6] = 0.0
            return values

        if raw_gripper_target < self.live_config.b_release_open_threshold:
            self._b_release_candidate_steps += 1
        else:
            self._b_release_candidate_steps = 0
        values[6] = 1.0
        if self._b_release_candidate_steps < self.live_config.b_release_stable_steps:
            return values

        if self.live_config.b_release_position_gate_enabled:
            target_inside = _inside_xyz_bounds(
                values[:3],
                self.live_config.b_release_workspace_min_xyz_mm,
                self.live_config.b_release_workspace_max_xyz_mm,
            )
            actual_inside = _inside_xyz_bounds(
                state[6:9],
                self.live_config.b_release_workspace_min_xyz_mm,
                self.live_config.b_release_workspace_max_xyz_mm,
            )
            if not target_inside or not actual_inside:
                self._event(
                    "act_b_release_rejected_outside_demonstration_envelope",
                    raw_gripper_target=raw_gripper_target,
                    stable_open_steps=self._b_release_candidate_steps,
                    policy_target_xyz_mm=values[:3].tolist(),
                    actual_xyz_mm=state[6:9].tolist(),
                    target_inside=target_inside,
                    actual_inside=actual_inside,
                    position_gate_enabled=True,
                )
                raise RuntimeError(
                    "ACT-B stable release request outside demonstration envelope"
                )

        self._b_release_authorized = True
        self._b_release_authorized_s = time.monotonic()
        values[6] = 0.0
        self._event(
            "act_b_release_authorized",
            raw_gripper_target=raw_gripper_target,
            stable_open_steps=self._b_release_candidate_steps,
            authorization_basis="stable_fresh_act_b_open_request",
            position_gate_enabled=(
                self.live_config.b_release_position_gate_enabled
            ),
            policy_target_mm_deg=values[:6].tolist(),
            actual_pose_mm_deg=state[6:12].tolist(),
            active_generation=(
                None
                if self._session_b is None
                else self._session_b.active_generation
            ),
            authorization_precedes_open_command=True,
        )
        return values

    def _update_b_release_confirmation(
        self,
        capture: _LiveCapture,
        state: np.ndarray,
    ) -> None:
        if self._b_release_command_sent_s is None or self._b_release_confirmed:
            return
        if float(state[12]) < 0.5:
            self._b_release_open_observation_frames += 1
        else:
            self._b_release_open_observation_frames = 0

        cache = getattr(self._raw_robot, "cache", None)
        if cache is None:
            raise RuntimeError("ACT-B release confirmation lacks robot cache")
        completed = cache.sample("gripper_completed_command")
        busy = cache.sample("gripper_driver_busy")
        command_ok = cache.sample("gripper_last_command_ok")
        completed_after_command = bool(
            completed is not None
            and completed.value == "open"
            and completed.receive_time >= self._b_release_command_sent_s
        )
        driver_ready = bool(
            busy is not None
            and busy.receive_time >= self._b_release_command_sent_s
            and busy.value is False
            and command_ok is not None
            and command_ok.receive_time >= self._b_release_command_sent_s
            and command_ok.value is True
        )
        if (
            completed_after_command
            and driver_ready
            and self._b_release_open_observation_frames
            >= self.live_config.b_release_observation_stable_frames
        ):
            self._b_release_confirmed = True
            self._b_release_pose_mm = state[6:12].copy()
            self._event(
                "act_b_release_confirmed",
                completed_command_receive_timestamp_s=completed.receive_time,
                driver_busy=bool(busy.value),
                driver_last_command_ok=bool(command_ok.value),
                open_observation_stable_frames=(
                    self._b_release_open_observation_frames
                ),
                actual_release_pose_mm_deg=self._b_release_pose_mm.tolist(),
                seconds_from_command=(
                    capture.capture_completed_s - self._b_release_command_sent_s
                ),
            )
            if self.live_config.b_completion_mode == "successor_owned":
                self._event(
                    "act_b_release_verified_successor_continues",
                    automatic_completion_disabled=True,
                    completion_authority="external_operator_or_planner",
                    act_b_queue_preserved=True,
                    rolling_refresh_preserved=True,
                )
            return
        if (
            capture.capture_completed_s - self._b_release_command_sent_s
            > self.live_config.b_release_driver_timeout_s
        ):
            raise TimeoutError("ACT-B gripper open driver confirmation timeout")

    def _update_b_completion(
        self,
        state: np.ndarray,
    ) -> tuple[bool, float | None]:
        if not self._b_release_confirmed:
            self._b_completion_stable_frames = 0
            return False, None
        if float(state[12]) >= 0.5:
            raise RuntimeError("gripper re-closed after ACT-B release confirmation")

        if self.live_config.b_completion_mode == "successor_owned":
            # OPEN is evidence that the physical release occurred, not a
            # task-level done token. ACT has no such token, so keep consuming
            # and refreshing ACT-B until an external authority ends the run.
            self._b_completion_stable_frames = 0
            return False, None

        position_ready = True
        if self.live_config.b_completion_position_gate_enabled:
            position_ready = _inside_xyz_bounds(
                state[6:9],
                self.live_config.b_completion_workspace_min_xyz_mm,
                self.live_config.b_completion_workspace_max_xyz_mm,
            )
        speed_mm_s: float | None = None
        if position_ready:
            try:
                velocity = self._actual_history.estimate(
                    window_frames=15,
                    method="linear_regression",
                    velocity_epsilon=1e-9,
                )
                speed_mm_s = float(np.linalg.norm(velocity))
            except ValueError:
                speed_mm_s = None
        settled = bool(
            position_ready
            and speed_mm_s is not None
            and speed_mm_s
            <= self.live_config.b_completion_velocity_tolerance_mm_s
        )
        if settled:
            self._b_completion_stable_frames += 1
        else:
            self._b_completion_stable_frames = 0
        return (
            self._b_completion_stable_frames
            >= self.live_config.b_completion_stable_frames,
            speed_mm_s,
        )

    def _complete_b(
        self,
        ctx: Any,
        state: np.ndarray,
        *,
        speed_mm_s: float | None,
    ) -> None:
        if self._last_command_action is None:
            raise RuntimeError("ACT-B completion lacks a final command")
        final_hold = self._last_command_action.copy()
        final_hold[6] = 0.0
        self._send_array(
            ctx,
            final_hold,
            "ACT-B-FINAL-HOLD",
            actual_pose=state[6:12],
            enforce_stream_ramp=False,
        )
        if self._session_b is not None:
            self._session_b.deactivate_and_clear()
        self.phase = LivePhase.COMPLETE
        self._event(
            "act_b_task_complete",
            completion_mode=self.live_config.b_completion_mode,
            completion_reason=(
                "release_open_and_position_gated_settle"
                if self.live_config.b_completion_position_gate_enabled
                else "release_open_and_motion_settled"
            ),
            completion_position_gate_enabled=(
                self.live_config.b_completion_position_gate_enabled
            ),
            b_steps_sent=self._b_steps_sent,
            rolling_refresh_count=self._b_refresh_count,
            refresh_hold_cycles=self._b_refresh_hold_cycles,
            release_pose_mm_deg=(
                None
                if self._b_release_pose_mm is None
                else self._b_release_pose_mm.tolist()
            ),
            final_actual_pose_mm_deg=state[6:12].tolist(),
            final_speed_mm_s=speed_mm_s,
            completion_stable_frames=self._b_completion_stable_frames,
            gripper_open_latched=True,
            gripper_driver_open_confirmed=True,
            commands_by_phase=dict(self._commands_by_phase),
        )

    def _step_b(self, ctx: Any, capture: _LiveCapture, state: np.ndarray) -> None:
        assert self._session_b is not None
        if self._b_started_s is None:
            raise RuntimeError("ACT-B phase lacks an atomic handoff timestamp")
        b_elapsed_s = time.monotonic() - self._b_started_s
        if b_elapsed_s > self.live_config.b_execution_timeout_s:
            if self.live_config.b_completion_mode == "successor_owned":
                raise TimeoutError("ACT-B successor-owned safety timeout")
            raise TimeoutError("ACT-B semantic completion timeout")
        if self._b_steps_sent >= self.live_config.b_execution_steps:
            raise RuntimeError("ACT-B execution step watchdog reached")

        self._poll_b_refresh(capture, state)
        self._update_b_release_confirmation(capture, state)
        completed, speed_mm_s = self._update_b_completion(state)
        if completed:
            self._complete_b(ctx, state, speed_mm_s=speed_mm_s)
            return

        if (
            self._b_refresh_generation is None
            and self._session_b.queue_size
            <= self.live_config.b_refresh_queue_threshold
        ):
            self._request_b_refresh(capture)

        action = self._session_b.pop_action()
        if action is None:
            if self._b_refresh_generation is None:
                self._request_b_refresh(capture)
            self._send_b_refresh_hold(ctx, actual_pose=state[6:12])
            return

        outgoing = self._prepare_b_action(action, state)
        self._submit_live_command(
            ctx,
            outgoing,
            "ACT-B",
            actual_pose=state[6:12],
            enforce_stream_ramp=False,
        )

    def _fail(self, reason: str) -> None:
        if reason not in self._failure_reasons:
            self._failure_reasons.append(reason)
        self.phase = LivePhase.FAILED_HOLD
        self._pending_command = None
        try:
            if self._engine is not None:
                self._engine.pause()
                self._engine.reset()
            if self._session_b is not None:
                self._session_b.deactivate_and_clear()
        finally:
            self._event(
                "fail_closed_external_gate_must_disable",
                failure_reasons=list(self._failure_reasons),
                commands_by_phase=dict(self._commands_by_phase),
            )

    def _representative_source_semantic_override(
        self,
        _cut_status: Any,
    ) -> bool | None:
        """Return None to preserve the reviewed V0/V1 semantic contract."""
        return None

    def _a_exit_commit_ready(
        self,
        ctx: Any,
        capture: _LiveCapture,
        state: np.ndarray,
        cut_status: Any,
    ) -> bool:
        """Evaluate the ACT-A exit without changing the reviewed default contract.

        V0/V1 keep using the representative 40/20 mm tracker here.  V2 may
        override this single decision hook while reusing the same 30 Hz loop,
        command dispatch, acknowledgement, and fail-closed paths.
        """

        del ctx, state
        semantic_override = self._representative_source_semantic_override(cut_status)
        boundary_commit_ready = True
        if self._representative_boundary_tracker is not None:
            boundary_velocity = self._estimate_actual_boundary_velocity()
            boundary_status = self._representative_boundary_tracker.update(
                tcp_position_mm=capture.tcp_position_mm,
                tcp_velocity_mm_s=boundary_velocity,
                open_before_close_observed=cut_status.open_seen,
                gripper_closed=capture.semantic_state.gripper_closed,
                semantic_ready=semantic_override,
            )
            self._representative_boundary_status = boundary_status
            boundary_commit_ready = boundary_status.commit_ready
            if boundary_status.just_prearmed:
                self._event(
                    "representative_boundary_prearmed",
                    read_only=True,
                    boundary_status=asdict(boundary_status),
                )
                if boundary_velocity is not None:
                    self._read_only_representative_preplan(
                        capture,
                        boundary_velocity,
                    )
        semantic_ready = (
            cut_status.ready if semantic_override is None else semantic_override
        )
        return bool(semantic_ready and boundary_commit_ready)

    def run(self, ctx: Any) -> None:
        interval_s = 1.0 / float(ctx.runtime.cfg.fps)
        started_s = time.monotonic()
        while not ctx.runtime.shutdown_event.is_set():
            loop_started = time.monotonic()
            if self.phase in {LivePhase.COMPLETE, LivePhase.FAILED_HOLD}:
                break
            if ctx.runtime.cfg.duration > 0.0 and loop_started - started_s >= ctx.runtime.cfg.duration:
                self._fail("rollout_duration_timeout")
                break
            try:
                capture, state, raw, processed = self._capture(ctx)
                live_enabled = bool(
                    self._raw_robot.cache.require_present("live_state").value
                )
                if not live_enabled:
                    if self.phase is not LivePhase.WAITING_FOR_LIVE:
                        self._fail("live_disabled_during_transition")
                        break
                elif self.phase is LivePhase.WAITING_FOR_LIVE:
                    self._assert_task_a_start(state)
                    self.phase = LivePhase.ACT_A
                    self._live_started_s = capture.capture_completed_s
                    self._cut_trigger.reset()
                    if self._representative_boundary_tracker is not None:
                        self._representative_boundary_tracker.reset()
                    self._representative_boundary_status = None
                    self._representative_preplan_attempts = 0
                    self._representative_preplan_succeeded = False
                    self._actual_history.clear()
                    self._a_path_length_mm = 0.0
                    self._last_actual_xyz = None
                    self._last_command_pose = None
                    self._last_command_action = None
                    self._pending_command = None
                    self._last_bridge_commanded_receive_count = None
                    self._reset_bridge_ack_tracking()
                    self._a_gripper_close_latched = False
                    self._live_orientation_anchor = state[9:12].copy()
                    self._event("external_live_edge_accepted")

                self._observe_actual(capture, state)

                if self.phase is LivePhase.ACT_A:
                    if self._live_started_s is None:
                        raise RuntimeError("ACT-A phase lacks Live timestamp")
                    if (
                        capture.capture_completed_s - self._live_started_s
                        > self.live_config.cut_timeout_s
                    ):
                        raise RuntimeError("semantic cut timeout")
                    status = self._cut_trigger.update(
                        timestamp_s=capture.capture_completed_s,
                        tcp_z_mm=float(state[8]),
                        gripper_closed=capture.semantic_state.gripper_closed,
                    )
                    if self._a_exit_commit_ready(
                        ctx,
                        capture,
                        state,
                        status,
                    ):
                        self._begin_bridge(ctx, capture, state)
                    else:
                        if not policy_live_queue_ready():
                            # Adapter hold timer owns the physical hold until a
                            # fresh post-Live ACT-A chunk is ready.
                            pass
                        else:
                            action = self._send_next_a_action(
                                ctx,
                                processed_observation=processed,
                                raw_observation=raw,
                                actual_pose=state[6:12],
                                gripper_close_latch_armed=status.open_seen,
                            )
                            if action is not None:
                                self._commands_by_phase[LivePhase.ACT_A.value] += 1
                elif self.phase is LivePhase.BRIDGE:
                    if (
                        self._transition_started_s is not None
                        and capture.capture_completed_s - self._transition_started_s
                        > self.live_config.transition_timeout_s
                    ):
                        raise RuntimeError("Bridge/B transition timeout")
                    if self._pending_command is not None:
                        self._flush_pending_command(
                            ctx,
                            actual_pose=state[6:12],
                        )
                    elif self._bridge_streamer_acknowledged():
                        self._step_bridge(ctx, capture, state)
                elif self.phase is LivePhase.ACT_B:
                    if self._pending_command is not None:
                        self._flush_pending_command(
                            ctx,
                            actual_pose=state[6:12],
                        )
                    else:
                        self._step_b(ctx, capture, state)
            except Exception as exc:
                logger.exception("Task-C live failed")
                self._fail(f"{type(exc).__name__}: {exc}")
                break

            elapsed = time.monotonic() - loop_started
            if elapsed < interval_s:
                precise_sleep(interval_s - elapsed)

    def teardown(self, ctx: Any) -> None:
        if self._engine is not None:
            self._engine.stop()
        if self._session_b is not None:
            self._session_b.close()
        if self._session_a_stub is not None:
            self._session_a_stub.close()
        self._event(
            "strategy_teardown",
            terminal_phase=self.phase.value,
            b_completion_mode=self.live_config.b_completion_mode,
            failure_reasons=list(self._failure_reasons),
            commands_by_phase=dict(self._commands_by_phase),
            bridge_ack_mode=self.live_config.bridge_ack_mode,
            bridge_max_ack_lag_steps=(
                self.live_config.bridge_max_ack_lag_steps
            ),
            bridge_ack_wait_cycles=self._bridge_ack_wait_cycles,
            bridge_ack_no_fresh_cycles=self._bridge_ack_no_fresh_cycles,
            bridge_ack_pose_mismatch_cycles=(
                self._bridge_ack_pose_mismatch_cycles
            ),
            bridge_ack_lag_hold_cycles=self._bridge_ack_lag_hold_cycles,
            bridge_ack_pipeline_advance_cycles=(
                self._bridge_ack_pipeline_advance_cycles
            ),
            bridge_ack_max_decision_lag_steps=(
                self._bridge_ack_max_decision_lag_steps
            ),
            bridge_ack_max_outstanding_after_send_steps=(
                self._bridge_ack_max_outstanding_after_send_steps
            ),
            tracking_backpressure_events=self._tracking_backpressure_events,
            tracking_backpressure_wait_cycles=(
                self._tracking_backpressure_wait_cycles
            ),
            tracking_hold_commands=self._tracking_hold_commands,
            tracking_backpressure_max_candidate_position_error_mm=(
                self._tracking_backpressure_max_candidate_position_error_mm
            ),
            tracking_backpressure_max_candidate_orientation_error_deg=(
                self._tracking_backpressure_max_candidate_orientation_error_deg
            ),
        )
        if self._event_file is not None:
            self._event_file.close()
            self._event_file = None
        robot = ctx.hardware.robot_wrapper.inner
        if robot.is_connected:
            robot.disconnect()
        teleop = ctx.hardware.teleop
        if teleop is not None and teleop.is_connected:
            teleop.disconnect()


_FACTORY_INSTALLED = False


def install_task_c_live_strategy() -> bool:
    global _FACTORY_INSTALLED
    if _FACTORY_INSTALLED:
        return False
    import lerobot.rollout as rollout_package
    import lerobot.rollout.strategies as strategies_package
    from lerobot.rollout.strategies import factory

    original = factory.create_strategy

    def create_strategy(config: RolloutStrategyConfig) -> RolloutStrategy:
        if config.type == "task_c_live":
            if not isinstance(config, TaskCLiveStrategyConfig):
                raise TypeError("task_c_live config registration mismatch")
            return TaskCLiveStrategy(config)
        return original(config)

    factory.create_strategy = create_strategy
    strategies_package.create_strategy = create_strategy
    rollout_package.create_strategy = create_strategy
    _FACTORY_INSTALLED = True
    return True
