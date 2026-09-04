"""Pure pose-continuity contracts for the bounded Task-C live handoff.

This module has no ROS or robot imports.  It provides the pieces that must be
validated before a command-producing strategy may translate the existing
position-only coordinator proposals into full A0509 Cartesian actions.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from quest_a0509_teleop.doosan_orientation import (
    doosan_zyz_deg_to_quaternion,
    quaternion_angle_deg,
    quaternion_slerp,
    quaternion_to_doosan_zyz_deg,
)

from .bezier_bridge import CubicBezierBridge
from .runtime_policy import PolicyChunk


def _finite_vector(value: np.ndarray, length: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite shape ({length},)")
    return result.copy()


def pose_delta_metrics(
    actual_pose_mm_deg: np.ndarray,
    target_pose_mm_deg: np.ndarray,
) -> tuple[float, float]:
    """Return Cartesian position and physical orientation errors."""

    actual = _finite_vector(actual_pose_mm_deg, 6, "actual pose")
    target = _finite_vector(target_pose_mm_deg, 6, "target pose")
    position_error = float(np.linalg.norm(target[:3] - actual[:3]))
    orientation_error = quaternion_angle_deg(
        doosan_zyz_deg_to_quaternion(actual[3:6]),
        doosan_zyz_deg_to_quaternion(target[3:6]),
    )
    return position_error, float(orientation_error)


@dataclass(frozen=True)
class PolicyPoseAssessment:
    valid: bool
    failure_reasons: tuple[str, ...]
    first_position_jump_mm: float
    first_orientation_jump_deg: float
    max_predicted_velocity_mm_s: float
    max_predicted_axis_velocity_mm_s: float
    max_predicted_angular_velocity_deg_s: float
    window_steps: int


def assess_policy_pose_chunk(
    chunk: PolicyChunk,
    observation_pose_mm_deg: np.ndarray,
    *,
    window_steps: int,
    first_position_jump_limit_mm: float,
    first_orientation_jump_limit_deg: float,
    predicted_velocity_limit_mm_s: float,
    predicted_angular_velocity_limit_deg_s: float,
    predicted_axis_velocity_limit_mm_s: float | None = None,
) -> PolicyPoseAssessment:
    """Assess full-pose continuity of a postprocessed ACT chunk.

    The actual pose at capture is used only for explicit first-target jump
    checks. The Cartesian bridge owns that pose-to-target gap. Predicted linear
    and angular velocities are measured only between consecutive postprocessed
    policy targets. Euler-number differences are never used for orientation;
    every angular delta is the quaternion geodesic angle.
    """

    pose = _finite_vector(observation_pose_mm_deg, 6, "observation pose")
    limits = (
        first_position_jump_limit_mm,
        first_orientation_jump_limit_deg,
        predicted_velocity_limit_mm_s,
        predicted_angular_velocity_limit_deg_s,
    )
    if any(not np.isfinite(value) or value <= 0.0 for value in limits):
        raise ValueError("pose assessment limits must be finite and positive")
    if predicted_axis_velocity_limit_mm_s is not None and (
        not np.isfinite(predicted_axis_velocity_limit_mm_s)
        or predicted_axis_velocity_limit_mm_s <= 0.0
    ):
        raise ValueError("predicted axis velocity limit must be finite and positive")
    if window_steps < 2:
        raise ValueError("window_steps must be at least 2")
    if chunk.actions.shape[1] < 6:
        raise ValueError("full-pose assessment requires at least six action values")

    steps = min(int(window_steps), len(chunk.actions))
    if steps < 2:
        raise ValueError("policy chunk requires at least 2 actions for pose velocity")
    predicted = chunk.actions[:steps, :6]
    linear_velocities = np.diff(predicted[:, :3], axis=0) * chunk.action_hz
    max_linear = float(np.max(np.linalg.norm(linear_velocities, axis=1)))
    max_linear_axis = float(np.max(np.abs(linear_velocities)))
    orientations = predicted[:, 3:6]
    angular_rates = [
        quaternion_angle_deg(
            doosan_zyz_deg_to_quaternion(left),
            doosan_zyz_deg_to_quaternion(right),
        )
        * chunk.action_hz
        for left, right in zip(orientations, orientations[1:])
    ]
    first_position, first_orientation = pose_delta_metrics(pose, predicted[0])
    max_angular = float(max(angular_rates))

    reasons: list[str] = []
    if first_position > first_position_jump_limit_mm:
        reasons.append("policy_first_position_jump")
    if first_orientation > first_orientation_jump_limit_deg:
        reasons.append("policy_first_orientation_jump")
    if max_linear > predicted_velocity_limit_mm_s:
        reasons.append("policy_predicted_velocity_limit")
    if (
        predicted_axis_velocity_limit_mm_s is not None
        and max_linear_axis > predicted_axis_velocity_limit_mm_s
    ):
        reasons.append("policy_predicted_axis_velocity_limit")
    if max_angular > predicted_angular_velocity_limit_deg_s:
        reasons.append("policy_predicted_angular_velocity_limit")
    return PolicyPoseAssessment(
        valid=not reasons,
        failure_reasons=tuple(reasons),
        first_position_jump_mm=first_position,
        first_orientation_jump_deg=first_orientation,
        max_predicted_velocity_mm_s=max_linear,
        max_predicted_axis_velocity_mm_s=max_linear_axis,
        max_predicted_angular_velocity_deg_s=max_angular,
        window_steps=steps,
    )


class OrientationBridge:
    """Shortest-path quaternion bridge with zero endpoint angular rate."""

    def __init__(
        self,
        start_zyz_deg: np.ndarray,
        target_zyz_deg: np.ndarray,
        *,
        duration_s: float,
    ) -> None:
        self.start_zyz_deg = _finite_vector(start_zyz_deg, 3, "start orientation")
        self.target_zyz_deg = _finite_vector(target_zyz_deg, 3, "target orientation")
        self.duration_s = float(duration_s)
        if not np.isfinite(self.duration_s) or self.duration_s <= 0.0:
            raise ValueError("orientation bridge duration must be positive")
        self._start_quaternion = doosan_zyz_deg_to_quaternion(self.start_zyz_deg)
        self._target_quaternion = doosan_zyz_deg_to_quaternion(self.target_zyz_deg)
        self._last_zyz_deg = self.start_zyz_deg.tolist()

    @staticmethod
    def _quintic_smoothstep(value: float) -> float:
        u = min(1.0, max(0.0, float(value)))
        return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5

    def sample(self, elapsed_s: float) -> np.ndarray:
        u = min(1.0, max(0.0, float(elapsed_s) / self.duration_s))
        quaternion = quaternion_slerp(
            self._start_quaternion,
            self._target_quaternion,
            self._quintic_smoothstep(u),
        )
        self._last_zyz_deg = quaternion_to_doosan_zyz_deg(
            quaternion,
            self._last_zyz_deg,
        )
        return np.asarray(self._last_zyz_deg, dtype=np.float64)


@dataclass(frozen=True)
class DownstreamControlContract:
    """Command-space limits imposed after the Task-C policy publisher.

    A bridge is natural only if the safety guard and ServoL stream ramp pass
    every requested pose through unchanged.  These defaults mirror the
    accepted A0509 bringup configuration; the live runner additionally emits
    them in its readiness event so the external gate can verify the running
    ROS parameters before it enables Live output.
    """

    control_hz: float = 30.0
    linear_ramp_mm_per_tick: float = 6.67
    orientation_ramp_deg_per_tick: float = 1.0
    servol_time_s: float = 0.1
    servol_use_auto_velocity_acceleration: bool = True
    lerobot_timeout_s: float = 0.3
    lerobot_target_topic: str = "/control/lerobot/target_posx"
    selected_target_topic: str = "/vr/target_posx"
    safe_posx_topic: str = "/vr/safe_posx"
    commanded_posx_topic: str = "/vr/commanded_posx"
    workspace_min_xyz_mm: tuple[float, float, float] = (50.0, -350.0, 0.0)
    workspace_min_limit_enabled: tuple[bool, bool, bool] = (True, True, False)
    workspace_max_xyz_mm: tuple[float, float, float] = (650.0, 350.0, 600.0)
    orientation_limit_deg: float = 90.0

    def __post_init__(self) -> None:
        positive = (
            self.control_hz,
            self.linear_ramp_mm_per_tick,
            self.orientation_ramp_deg_per_tick,
            self.orientation_limit_deg,
            self.servol_time_s,
            self.lerobot_timeout_s,
        )
        if any(not np.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("downstream control rates and limits must be positive")
        topics = (
            self.lerobot_target_topic,
            self.selected_target_topic,
            self.safe_posx_topic,
            self.commanded_posx_topic,
        )
        if any(not value or not value.startswith("/") for value in topics):
            raise ValueError("downstream topics must be absolute ROS names")
        if (
            len(self.workspace_min_xyz_mm) != 3
            or len(self.workspace_min_limit_enabled) != 3
            or len(self.workspace_max_xyz_mm) != 3
        ):
            raise ValueError("downstream workspace contract must contain XYZ")
        minimum = np.asarray(self.workspace_min_xyz_mm, dtype=np.float64)
        maximum = np.asarray(self.workspace_max_xyz_mm, dtype=np.float64)
        if not np.all(np.isfinite(minimum)) or not np.all(np.isfinite(maximum)):
            raise ValueError("downstream workspace bounds must be finite")
        enabled = np.asarray(self.workspace_min_limit_enabled, dtype=np.bool_)
        if np.any(minimum[enabled] > maximum[enabled]):
            raise ValueError("enabled downstream workspace bounds are inverted")

    @property
    def conservative_velocity_limit_mm_s(self) -> float:
        """Vector speed that guarantees every Cartesian axis avoids ramping."""

        return float(self.linear_ramp_mm_per_tick * self.control_hz)

    @property
    def angular_velocity_limit_deg_s(self) -> float:
        return float(self.orientation_ramp_deg_per_tick * self.control_hz)

    def intersect_workspace(
        self,
        minimum_mm: np.ndarray,
        maximum_mm: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Intersect an offline workspace with the live clamp configuration."""

        minimum = _finite_vector(minimum_mm, 3, "offline workspace minimum")
        maximum = _finite_vector(maximum_mm, 3, "offline workspace maximum")
        guard_minimum = np.asarray(self.workspace_min_xyz_mm, dtype=np.float64)
        guard_maximum = np.asarray(self.workspace_max_xyz_mm, dtype=np.float64)
        enabled = np.asarray(self.workspace_min_limit_enabled, dtype=np.bool_)
        minimum[enabled] = np.maximum(minimum[enabled], guard_minimum[enabled])
        maximum = np.minimum(maximum, guard_maximum)
        if np.any(minimum >= maximum):
            raise ValueError("offline/live workspace intersection is empty")
        return minimum, maximum


@dataclass(frozen=True)
class BridgeStreamAssessment:
    valid: bool
    failure_reasons: tuple[str, ...]
    sample_count: int
    max_position_step_mm: float
    max_linear_axis_step_mm: float
    max_orientation_step_deg: float
    max_orientation_from_anchor_deg: float
    sampled_min_xyz_mm: tuple[float, float, float]
    sampled_max_xyz_mm: tuple[float, float, float]


def assess_bridge_stream_contract(
    bridge: CubicBezierBridge,
    start_orientation_zyz_deg: np.ndarray,
    target_orientation_zyz_deg: np.ndarray,
    *,
    contract: DownstreamControlContract,
    orientation_anchor_zyz_deg: np.ndarray | None = None,
) -> BridgeStreamAssessment:
    """Verify that the live guard/streamer would not alter a bridge request.

    This is a preflight assessment at the downstream 30 Hz command rate.  The
    live runner repeats the per-command checks using the actual elapsed-time
    proposal, which also catches scheduler jitter that advances farther than a
    nominal tick.
    """

    start_orientation = _finite_vector(
        start_orientation_zyz_deg, 3, "bridge start orientation"
    )
    target_orientation = _finite_vector(
        target_orientation_zyz_deg, 3, "bridge target orientation"
    )
    anchor_orientation = (
        start_orientation
        if orientation_anchor_zyz_deg is None
        else _finite_vector(
            orientation_anchor_zyz_deg, 3, "live orientation anchor"
        )
    )
    sample_count = max(2, int(np.ceil(bridge.duration_s * contract.control_hz)) + 1)
    elapsed = np.linspace(0.0, bridge.duration_s, sample_count)
    positions = bridge.position(elapsed / bridge.duration_s)
    orientation_bridge = OrientationBridge(
        start_orientation,
        target_orientation,
        duration_s=bridge.duration_s,
    )
    orientations = np.stack(
        [orientation_bridge.sample(float(value)) for value in elapsed],
        axis=0,
    )

    position_steps = np.diff(positions, axis=0)
    max_position_step = float(np.max(np.linalg.norm(position_steps, axis=1)))
    max_axis_step = float(np.max(np.abs(position_steps)))
    orientation_steps = [
        quaternion_angle_deg(
            doosan_zyz_deg_to_quaternion(left),
            doosan_zyz_deg_to_quaternion(right),
        )
        for left, right in zip(orientations[:-1], orientations[1:], strict=True)
    ]
    anchor_quaternion = doosan_zyz_deg_to_quaternion(anchor_orientation)
    anchor_deltas = [
        quaternion_angle_deg(
            anchor_quaternion,
            doosan_zyz_deg_to_quaternion(value),
        )
        for value in orientations
    ]
    max_orientation_step = float(max(orientation_steps))
    max_orientation_from_anchor = float(max(anchor_deltas))

    sampled_minimum = np.min(positions, axis=0)
    sampled_maximum = np.max(positions, axis=0)
    guard_minimum = np.asarray(contract.workspace_min_xyz_mm, dtype=np.float64)
    guard_maximum = np.asarray(contract.workspace_max_xyz_mm, dtype=np.float64)
    minimum_enabled = np.asarray(
        contract.workspace_min_limit_enabled, dtype=np.bool_
    )
    workspace_violation = bool(
        np.any(positions[:, minimum_enabled] < guard_minimum[minimum_enabled] - 1e-9)
        or np.any(positions > guard_maximum + 1e-9)
    )

    reasons: list[str] = []
    if workspace_violation:
        reasons.append("downstream_workspace_clamp")
    if max_axis_step > contract.linear_ramp_mm_per_tick + 1e-9:
        reasons.append("downstream_linear_ramp")
    if max_orientation_step > contract.orientation_ramp_deg_per_tick + 1e-9:
        reasons.append("downstream_orientation_ramp")
    if max_orientation_from_anchor > contract.orientation_limit_deg + 1e-9:
        reasons.append("downstream_orientation_clamp")
    return BridgeStreamAssessment(
        valid=not reasons,
        failure_reasons=tuple(reasons),
        sample_count=sample_count,
        max_position_step_mm=max_position_step,
        max_linear_axis_step_mm=max_axis_step,
        max_orientation_step_deg=max_orientation_step,
        max_orientation_from_anchor_deg=max_orientation_from_anchor,
        sampled_min_xyz_mm=tuple(float(value) for value in sampled_minimum),
        sampled_max_xyz_mm=tuple(float(value) for value in sampled_maximum),
    )


@dataclass(frozen=True)
class CutTriggerConfig:
    open_stable_frames: int = 3
    closed_stable_frames: int = 15
    post_close_delay_s: float = 1.0
    transport_z_min_mm: float = 400.0

    def __post_init__(self) -> None:
        if self.open_stable_frames < 1 or self.closed_stable_frames < 1:
            raise ValueError("cut trigger stable frame counts must be positive")
        if self.post_close_delay_s < 0.0:
            raise ValueError("post_close_delay_s must be non-negative")
        if not np.isfinite(self.transport_z_min_mm):
            raise ValueError("transport_z_min_mm must be finite")


@dataclass(frozen=True)
class CutTriggerStatus:
    ready: bool
    open_seen: bool
    open_stable_frames: int
    closed_stable_frames: int
    seconds_since_close: float | None
    tcp_z_mm: float
    waiting_reasons: tuple[str, ...]


class LiveCutTrigger:
    """Require an open->closed transport transition after Live starts."""

    def __init__(self, config: CutTriggerConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._open_stable = 0
        self._open_seen = False
        self._closed_stable = 0
        self._closed_started_s: float | None = None

    def update(
        self,
        *,
        timestamp_s: float,
        tcp_z_mm: float,
        gripper_closed: bool,
    ) -> CutTriggerStatus:
        timestamp = float(timestamp_s)
        z_value = float(tcp_z_mm)
        if not np.isfinite(timestamp) or not np.isfinite(z_value):
            raise ValueError("cut trigger timestamp and TCP Z must be finite")

        if not gripper_closed:
            self._open_stable += 1
            self._closed_stable = 0
            self._closed_started_s = None
            if self._open_stable >= self.config.open_stable_frames:
                self._open_seen = True
        else:
            self._open_stable = 0
            if self._open_seen:
                if self._closed_stable == 0:
                    self._closed_started_s = timestamp
                self._closed_stable += 1

        seconds_since_close = (
            None
            if self._closed_started_s is None
            else max(0.0, timestamp - self._closed_started_s)
        )
        reasons: list[str] = []
        if not self._open_seen:
            reasons.append("open_before_close_not_observed")
        if self._closed_stable < self.config.closed_stable_frames:
            reasons.append("gripper_close_not_stable")
        if (
            seconds_since_close is None
            or seconds_since_close < self.config.post_close_delay_s
        ):
            reasons.append("post_close_transport_delay")
        if z_value < self.config.transport_z_min_mm:
            reasons.append("transport_phase_z_not_reached")
        return CutTriggerStatus(
            ready=not reasons,
            open_seen=self._open_seen,
            open_stable_frames=self._open_stable,
            closed_stable_frames=self._closed_stable,
            seconds_since_close=seconds_since_close,
            tcp_z_mm=z_value,
            waiting_reasons=tuple(reasons),
        )


def full_action(
    xyz_mm: np.ndarray,
    orientation_zyz_deg: np.ndarray,
    *,
    gripper_target: float,
) -> np.ndarray:
    xyz = _finite_vector(xyz_mm, 3, "action XYZ")
    orientation = _finite_vector(orientation_zyz_deg, 3, "action orientation")
    gripper = float(gripper_target)
    if not np.isfinite(gripper) or not 0.0 <= gripper <= 1.0:
        raise ValueError("gripper_target must be finite in [0, 1]")
    return np.concatenate((xyz, orientation, np.array([gripper])))
