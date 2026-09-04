"""Bounded ACT-source queue projection for a future Bridge splice.

The ACT source keeps commanding while the isolated Flexible Bridge worker is
running.  This module takes a small immutable suffix of the already prepared
ACT queue and predicts the acknowledged ServoL pose at a near-future command
sequence.  The prediction is planning input only: the latest actual/ACK state
is still the authority for the final commit-time hard admission.

No inference, file I/O, search, or wait is performed here.  Queue snapshots
and projections are intentionally bounded to a handful of 7-D actions.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from offline_tools.task_c_bridge_v0.velocity_estimation import (
    estimate_velocity_over_window,
)
from quest_a0509_teleop.doosan_orientation import (
    step_doosan_zyz_toward_deg,
)

from .bridge_runtime import BridgeRuntimeSnapshot


@dataclass(frozen=True)
class SourceActionQueueSnapshot:
    """Immutable bounded suffix from one executable policy generation."""

    policy_id: str
    queue_kind: str
    generation: int
    revision_token: str
    cursor: int
    command_sequence: int
    timestamp_s: float
    action_hz: float
    actions: np.ndarray

    def __post_init__(self) -> None:
        actions = np.asarray(self.actions, dtype=np.float64)
        if not self.policy_id or not self.queue_kind or not self.revision_token:
            raise ValueError("source queue identity fields must not be empty")
        if self.generation < 0 or self.cursor < 0 or self.command_sequence < 0:
            raise ValueError("source queue counters must be non-negative")
        if not np.isfinite(self.timestamp_s):
            raise ValueError("source queue timestamp must be finite")
        if not np.isfinite(self.action_hz) or self.action_hz <= 0.0:
            raise ValueError("source queue action_hz must be positive")
        if actions.ndim != 2 or actions.shape[1] < 7:
            raise ValueError("source queue actions must have shape [N, >=7]")
        if not np.all(np.isfinite(actions)):
            raise ValueError("source queue actions must be finite")
        copied = actions.copy()
        copied.setflags(write=False)
        object.__setattr__(self, "actions", copied)

    @property
    def lineage(self) -> tuple[str, str, int, str]:
        return (
            self.policy_id,
            self.queue_kind,
            self.generation,
            self.revision_token,
        )

    def same_lineage(self, other: "SourceActionQueueSnapshot") -> bool:
        return self.lineage == other.lineage

    def record(self) -> dict[str, Any]:
        return {
            "policy_id": self.policy_id,
            "queue_kind": self.queue_kind,
            "generation": self.generation,
            "revision_token": self.revision_token,
            "cursor": self.cursor,
            "command_sequence": self.command_sequence,
            "timestamp_s": self.timestamp_s,
            "action_hz": self.action_hz,
            "available_steps": len(self.actions),
        }


@dataclass(frozen=True)
class PredictiveSourceSplice:
    """A future source boundary tied to one immutable queue lineage."""

    queue_snapshot: SourceActionQueueSnapshot
    lookahead_steps: int
    target_command_sequence: int
    predicted_snapshot: BridgeRuntimeSnapshot
    intended_timestamp_s: float
    projected_pose_path_mm_deg: np.ndarray

    def __post_init__(self) -> None:
        path = np.asarray(self.projected_pose_path_mm_deg, dtype=np.float64)
        if self.lookahead_steps < 1:
            raise ValueError("predictive source lookahead must be positive")
        if self.target_command_sequence != (
            self.queue_snapshot.command_sequence + self.lookahead_steps
        ):
            raise ValueError("predictive source target sequence is inconsistent")
        if path.shape != (self.lookahead_steps + 1, 6):
            raise ValueError("projected source path must include start plus each tick")
        if not np.all(np.isfinite(path)):
            raise ValueError("projected source path must be finite")
        copied = path.copy()
        copied.setflags(write=False)
        object.__setattr__(self, "projected_pose_path_mm_deg", copied)

    def record(self) -> dict[str, Any]:
        return {
            "queue": self.queue_snapshot.record(),
            "lookahead_steps": self.lookahead_steps,
            "target_command_sequence": self.target_command_sequence,
            "intended_timestamp_s": self.intended_timestamp_s,
            "predicted_pose_mm_deg": (
                self.predicted_snapshot.acknowledged_pose_mm_deg.tolist()
            ),
            "predicted_velocity_mm_s": (
                self.predicted_snapshot.actual_velocity_mm_s.tolist()
            ),
        }


def predictive_lookahead_steps(
    *,
    nominal_latency_s: float,
    observed_latencies_s: tuple[float, ...] | list[float],
    action_hz: float,
    margin_steps: int,
    minimum_steps: int,
    maximum_steps: int,
) -> int:
    """Choose a deterministic bounded horizon from nominal/recent p95 latency."""

    if not np.isfinite(nominal_latency_s) or nominal_latency_s < 0.0:
        raise ValueError("nominal latency must be finite and non-negative")
    if not np.isfinite(action_hz) or action_hz <= 0.0:
        raise ValueError("action_hz must be finite and positive")
    if margin_steps < 0 or minimum_steps < 1 or maximum_steps < minimum_steps:
        raise ValueError("invalid predictive source lookahead bounds")
    finite = np.asarray(observed_latencies_s, dtype=np.float64)
    finite = finite[np.isfinite(finite) & (finite >= 0.0)]
    observed_p95 = 0.0 if finite.size == 0 else float(np.percentile(finite, 95))
    latency = max(float(nominal_latency_s), observed_p95)
    requested = int(math.ceil(latency * float(action_hz))) + int(margin_steps)
    return max(minimum_steps, min(maximum_steps, requested))


def _as_cpu_numpy(value: Any) -> np.ndarray | None:
    """Copy a small tensor/array without ever synchronizing a CUDA tensor."""

    device = getattr(value, "device", None)
    if device is not None and getattr(device, "type", str(device)) != "cpu":
        return None
    detached = value.detach() if hasattr(value, "detach") else value
    array = detached.numpy() if hasattr(detached, "numpy") else np.asarray(detached)
    return np.asarray(array, dtype=np.float64).copy()


def _interpolated_source_suffix(
    interpolator: Any,
    queued_actions: np.ndarray,
    *,
    maximum_steps: int,
) -> np.ndarray:
    """Mirror LeRobot ActionInterpolator without mutating its state."""

    if maximum_steps < 1:
        raise ValueError("maximum_steps must be positive")
    output: list[np.ndarray] = []
    buffer = getattr(interpolator, "_buffer", ())
    index = int(getattr(interpolator, "_idx", 0))
    for value in tuple(buffer)[index:]:
        converted = _as_cpu_numpy(value)
        if converted is None:
            return np.empty((0, 7), dtype=np.float64)
        output.append(converted)
        if len(output) >= maximum_steps:
            return np.stack(output, axis=0)

    multiplier = int(getattr(interpolator, "multiplier", 1))
    if multiplier < 1:
        raise ValueError("interpolator multiplier must be positive")
    previous_value = getattr(interpolator, "_prev", None)
    previous = None if previous_value is None else _as_cpu_numpy(previous_value)
    if previous_value is not None and previous is None:
        return np.empty((0, 7), dtype=np.float64)

    for target in np.asarray(queued_actions, dtype=np.float64):
        if multiplier > 1 and previous is not None:
            for interpolation_index in range(1, multiplier + 1):
                fraction = interpolation_index / float(multiplier)
                output.append(previous + fraction * (target - previous))
                if len(output) >= maximum_steps:
                    return np.stack(output, axis=0)
        else:
            output.append(target.copy())
            if len(output) >= maximum_steps:
                return np.stack(output, axis=0)
        previous = target.copy()
    if not output:
        width = queued_actions.shape[1] if queued_actions.ndim == 2 else 7
        return np.empty((0, width), dtype=np.float64)
    return np.stack(output, axis=0)


def snapshot_rtc_source_queue(
    *,
    engine: Any,
    interpolator: Any,
    policy_id: str,
    command_sequence: int,
    timestamp_s: float,
    action_hz: float,
    maximum_steps: int,
) -> SourceActionQueueSnapshot | None:
    """Atomically copy a bounded suffix from LeRobot's processed RTC queue."""

    queue = getattr(engine, "action_queue", None)
    if queue is None or maximum_steps < 1:
        return None
    multiplier = max(1, int(getattr(interpolator, "multiplier", 1)))
    raw_steps = int(math.ceil(maximum_steps / multiplier)) + 1
    with queue.lock:
        queued = queue.queue
        cursor = int(queue.last_index)
        if queued is None:
            return None
        converted = _as_cpu_numpy(queued[cursor : cursor + raw_steps])
        if converted is None:
            return None
        generation = int(getattr(queue, "_a0509_live_generation", 0) or 0)
        revision_token = f"tensor:{id(queued)}"
    actions = _interpolated_source_suffix(
        interpolator,
        converted,
        maximum_steps=maximum_steps,
    )
    if actions.ndim != 2 or actions.shape[0] < 1:
        return None
    return SourceActionQueueSnapshot(
        policy_id=policy_id,
        queue_kind="lerobot_rtc",
        generation=generation,
        revision_token=revision_token,
        cursor=cursor,
        command_sequence=int(command_sequence),
        timestamp_s=float(timestamp_s),
        action_hz=float(action_hz),
        actions=actions,
    )


def source_snapshot_from_async_session(
    session_snapshot: Any,
    *,
    command_sequence: int,
    timestamp_s: float,
    maximum_steps: int,
) -> SourceActionQueueSnapshot | None:
    """Adapt an AsyncPolicySession queue snapshot to the shared contract."""

    if session_snapshot is None:
        return None
    actions = np.asarray(session_snapshot.actions, dtype=np.float64)[:maximum_steps]
    if len(actions) < 1:
        return None
    return SourceActionQueueSnapshot(
        policy_id=str(session_snapshot.policy_id),
        queue_kind="async_policy_session",
        generation=int(session_snapshot.generation),
        revision_token=f"chunk_generation:{int(session_snapshot.generation)}",
        cursor=int(session_snapshot.queue_index),
        command_sequence=int(command_sequence),
        timestamp_s=float(timestamp_s),
        action_hz=float(session_snapshot.action_hz),
        actions=actions,
    )


def project_predictive_source_splice(
    queue_snapshot: SourceActionQueueSnapshot,
    current_snapshot: BridgeRuntimeSnapshot,
    *,
    lookahead_steps: int,
    linear_ramp_mm_per_tick: float,
    orientation_ramp_deg_per_tick: float,
    velocity_window_steps: int,
) -> PredictiveSourceSplice:
    """Project a bounded queue prefix through the actual ServoL ramp contract."""

    if lookahead_steps < 1 or len(queue_snapshot.actions) < lookahead_steps:
        raise ValueError("source queue lacks the requested predictive horizon")
    if velocity_window_steps < 3:
        raise ValueError("velocity_window_steps must be at least three")
    if linear_ramp_mm_per_tick <= 0.0 or orientation_ramp_deg_per_tick <= 0.0:
        raise ValueError("source projection ramp limits must be positive")

    pose = current_snapshot.acknowledged_pose_mm_deg.copy()
    path = [pose.copy()]
    for action in queue_snapshot.actions[:lookahead_steps]:
        pose[:3] += np.clip(
            action[:3] - pose[:3],
            -linear_ramp_mm_per_tick,
            linear_ramp_mm_per_tick,
        )
        pose[3:6] = step_doosan_zyz_toward_deg(
            pose[3:6],
            action[3:6],
            orientation_ramp_deg_per_tick,
        )[0]
        path.append(pose.copy())
    projected_path = np.stack(path, axis=0)
    sample_count = min(len(projected_path), velocity_window_steps)
    if sample_count < 3:
        raise ValueError("predictive source velocity requires at least three poses")
    positions = projected_path[-sample_count:, :3]
    timestamps = np.arange(sample_count, dtype=np.float64) / queue_snapshot.action_hz
    velocity = estimate_velocity_over_window(positions, timestamps)
    predicted = BridgeRuntimeSnapshot(
        # Freshness measures the age of the observation/queue snapshot used to
        # make this prediction, not the future splice wall-clock timestamp.
        timestamp_s=queue_snapshot.timestamp_s,
        actual_pose_mm_deg=pose,
        acknowledged_pose_mm_deg=pose,
        actual_velocity_mm_s=velocity,
        gripper_target=current_snapshot.gripper_target,
    )
    return PredictiveSourceSplice(
        queue_snapshot=queue_snapshot,
        lookahead_steps=lookahead_steps,
        target_command_sequence=(
            queue_snapshot.command_sequence + lookahead_steps
        ),
        predicted_snapshot=predicted,
        intended_timestamp_s=(
            queue_snapshot.timestamp_s
            + lookahead_steps / queue_snapshot.action_hz
        ),
        projected_pose_path_mm_deg=projected_path,
    )
