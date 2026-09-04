"""Generation-isolated asynchronous policy sessions for Task-C dry runs.

The module owns no ROS objects and cannot publish robot commands. It keeps
model warmup, fresh inference, queue activation, and stale-result rejection
separate so ACT-A and ACT-B never share temporal state.
"""

from __future__ import annotations

import copy
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from typing import Any, Callable, Protocol

import numpy as np

from .velocity_estimation import estimate_velocity_over_window


class PolicyInferenceError(RuntimeError):
    """Raised when an asynchronous policy inference fails."""


class PolicyBackend(Protocol):
    """Minimal backend contract; implementations may wrap LeRobot or a fake."""

    def reset(self) -> None:
        ...

    def infer(self, observation: Any) -> np.ndarray:
        ...


@dataclass(frozen=True)
class PolicyChunk:
    policy_id: str
    generation: int
    observation_timestamp_s: float
    completed_timestamp_s: float
    inference_latency_s: float
    action_hz: float
    actions: np.ndarray
    request_timestamp_s: float | None = None
    snapshot_completed_timestamp_s: float | None = None
    submitted_timestamp_s: float | None = None
    worker_started_timestamp_s: float | None = None
    gpu_acquired_timestamp_s: float | None = None

    def __post_init__(self) -> None:
        actions = np.asarray(self.actions, dtype=np.float64)
        if actions.ndim != 2 or actions.shape[0] < 1 or actions.shape[1] < 3:
            raise ValueError("policy actions must have shape [steps, action_dim>=3]")
        if not np.all(np.isfinite(actions)):
            raise ValueError("policy chunk contains non-finite actions")
        if self.generation < 1:
            raise ValueError("policy generation must be positive")
        if self.action_hz <= 0.0 or not np.isfinite(self.action_hz):
            raise ValueError("action_hz must be finite and positive")
        if self.completed_timestamp_s < self.observation_timestamp_s:
            raise ValueError("chunk completion cannot precede its observation")
        # Older callers only provided the aggregate inference latency. Infer
        # a zero-wait timeline for those chunks while preserving the richer
        # timestamps produced by AsyncPolicySession.
        inferred_start = self.completed_timestamp_s - self.inference_latency_s
        request = (
            inferred_start
            if self.request_timestamp_s is None
            else float(self.request_timestamp_s)
        )
        snapshot = request if self.snapshot_completed_timestamp_s is None else float(
            self.snapshot_completed_timestamp_s
        )
        submitted = snapshot if self.submitted_timestamp_s is None else float(
            self.submitted_timestamp_s
        )
        worker = submitted if self.worker_started_timestamp_s is None else float(
            self.worker_started_timestamp_s
        )
        acquired = worker if self.gpu_acquired_timestamp_s is None else float(
            self.gpu_acquired_timestamp_s
        )
        timeline = (
            self.observation_timestamp_s,
            request,
            snapshot,
            submitted,
            worker,
            acquired,
            self.completed_timestamp_s,
        )
        if not all(np.isfinite(value) for value in timeline):
            raise ValueError("policy timing contains non-finite values")
        if any(right < left for left, right in zip(timeline, timeline[1:])):
            raise ValueError("policy timing must be monotonic")
        object.__setattr__(self, "request_timestamp_s", request)
        object.__setattr__(self, "snapshot_completed_timestamp_s", snapshot)
        object.__setattr__(self, "submitted_timestamp_s", submitted)
        object.__setattr__(self, "worker_started_timestamp_s", worker)
        object.__setattr__(self, "gpu_acquired_timestamp_s", acquired)
        object.__setattr__(self, "actions", actions.copy())

    @property
    def xyz_mm(self) -> np.ndarray:
        return self.actions[:, :3]

    @property
    def first_xyz_mm(self) -> np.ndarray:
        return self.xyz_mm[0]

    def observation_age_s(self, now_s: float) -> float:
        return max(0.0, float(now_s) - self.observation_timestamp_s)

    def suffix(self, start_index: int) -> "PolicyChunk":
        """Return an immutable action suffix with the original generation."""

        if not isinstance(start_index, int) or isinstance(start_index, bool):
            raise ValueError("policy suffix start_index must be an integer")
        if start_index < 0 or start_index >= len(self.actions):
            raise ValueError("policy suffix start_index is out of range")
        return PolicyChunk(
            policy_id=self.policy_id,
            generation=self.generation,
            observation_timestamp_s=self.observation_timestamp_s,
            completed_timestamp_s=self.completed_timestamp_s,
            inference_latency_s=self.inference_latency_s,
            action_hz=self.action_hz,
            actions=self.actions[start_index:],
            request_timestamp_s=self.request_timestamp_s,
            snapshot_completed_timestamp_s=self.snapshot_completed_timestamp_s,
            submitted_timestamp_s=self.submitted_timestamp_s,
            worker_started_timestamp_s=self.worker_started_timestamp_s,
            gpu_acquired_timestamp_s=self.gpu_acquired_timestamp_s,
        )

    @property
    def request_snapshot_latency_s(self) -> float:
        return self.snapshot_completed_timestamp_s - self.request_timestamp_s

    @property
    def submission_queue_latency_s(self) -> float:
        return self.worker_started_timestamp_s - self.submitted_timestamp_s

    @property
    def gpu_wait_latency_s(self) -> float:
        return self.gpu_acquired_timestamp_s - self.worker_started_timestamp_s

    @property
    def backend_inference_latency_s(self) -> float:
        return self.completed_timestamp_s - self.gpu_acquired_timestamp_s

    @property
    def request_to_completion_latency_s(self) -> float:
        return self.completed_timestamp_s - self.request_timestamp_s


@dataclass(frozen=True)
class PolicyChunkAssessment:
    valid: bool
    failure_reasons: tuple[str, ...]
    intended_velocity_mm_s: np.ndarray
    first_position_jump_mm: float
    max_predicted_velocity_mm_s: float
    velocity_window_steps: int
    velocity_method: str

    def __post_init__(self) -> None:
        velocity = np.asarray(self.intended_velocity_mm_s, dtype=np.float64)
        if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
            raise ValueError("intended policy velocity must be finite XYZ")
        object.__setattr__(self, "intended_velocity_mm_s", velocity.copy())


def assess_policy_chunk(
    chunk: PolicyChunk,
    observation_position_mm: np.ndarray,
    *,
    velocity_window_steps: int,
    velocity_method: str,
    velocity_epsilon: float,
    first_position_jump_limit_mm: float,
    predicted_velocity_limit_mm_s: float,
) -> PolicyChunkAssessment:
    """Extract B's intended initial velocity from postprocessed ACT targets.

    The capture-time observation is used only for the first-target jump. The
    bridge owns that spatial gap, so treating it as one ACT control tick would
    manufacture a large policy velocity while the bridge is still approaching
    B. Velocity is estimated only from consecutive postprocessed B targets.
    The fit is a multi-step regression, never a single action difference.
    """

    start = np.asarray(observation_position_mm, dtype=np.float64)
    if start.shape != (3,) or not np.all(np.isfinite(start)):
        raise ValueError("observation_position_mm must be finite XYZ")
    if velocity_window_steps < 2:
        raise ValueError("velocity_window_steps must be at least 2")
    steps = min(int(velocity_window_steps), len(chunk.xyz_mm))
    if steps < 2:
        raise ValueError("policy chunk requires at least 2 actions for velocity")
    positions = chunk.xyz_mm[:steps]
    timestamps = np.arange(len(positions), dtype=np.float64) / chunk.action_hz
    velocity = estimate_velocity_over_window(
        positions,
        timestamps,
        method=velocity_method,
        velocity_epsilon=velocity_epsilon,
    )
    discrete_velocity = np.diff(positions, axis=0) * chunk.action_hz
    max_predicted_velocity = float(
        np.max(np.linalg.norm(discrete_velocity, axis=1))
    )
    first_jump = float(np.linalg.norm(chunk.first_xyz_mm - start))
    reasons: list[str] = []
    if first_jump > first_position_jump_limit_mm:
        reasons.append("b_first_action_position_jump")
    if max_predicted_velocity > predicted_velocity_limit_mm_s:
        reasons.append("b_predicted_velocity_limit")
    return PolicyChunkAssessment(
        valid=not reasons,
        failure_reasons=tuple(reasons),
        intended_velocity_mm_s=velocity,
        first_position_jump_mm=first_jump,
        max_predicted_velocity_mm_s=max_predicted_velocity,
        velocity_window_steps=steps,
        velocity_method=velocity_method,
    )


def _snapshot_observation(value: Any) -> Any:
    """Own asynchronous inputs so camera/state buffers cannot mutate in-flight."""

    if isinstance(value, np.ndarray):
        return value.copy()
    try:
        import torch

        if isinstance(value, torch.Tensor):
            return value.detach().clone()
    except ImportError:
        pass
    if isinstance(value, dict):
        return {key: _snapshot_observation(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_snapshot_observation(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_snapshot_observation(item) for item in value)
    try:
        return copy.deepcopy(value)
    except TypeError:
        return value


@dataclass(frozen=True)
class PolicySessionStats:
    warmup_inferences: int
    submitted_inferences: int
    accepted_chunks: int
    stale_chunks_dropped: int
    queue_actions_consumed: int


@dataclass(frozen=True)
class ActivePolicyQueueSnapshot:
    """Atomic immutable view of a resident session's executable suffix."""

    policy_id: str
    generation: int
    queue_index: int
    queue_actions_consumed: int
    action_hz: float
    actions: np.ndarray

    def __post_init__(self) -> None:
        actions = np.asarray(self.actions, dtype=np.float64)
        if not self.policy_id:
            raise ValueError("policy queue snapshot requires policy_id")
        if self.generation < 1:
            raise ValueError("policy queue snapshot generation must be positive")
        if self.queue_index < 0 or self.queue_actions_consumed < 0:
            raise ValueError("policy queue snapshot counters must be non-negative")
        if not np.isfinite(self.action_hz) or self.action_hz <= 0.0:
            raise ValueError("policy queue snapshot action_hz must be positive")
        if actions.ndim != 2 or actions.shape[1] < 3:
            raise ValueError("policy queue snapshot actions must have shape [N, >=3]")
        if not np.all(np.isfinite(actions)):
            raise ValueError("policy queue snapshot actions must be finite")
        copied = actions.copy()
        copied.setflags(write=False)
        object.__setattr__(self, "actions", copied)


@dataclass(frozen=True)
class _InferenceResult:
    actions: np.ndarray
    observation_timestamp_s: float
    request_timestamp_s: float
    snapshot_completed_timestamp_s: float
    submitted_timestamp_s: float
    worker_started_timestamp_s: float
    gpu_acquired_timestamp_s: float
    completed_timestamp_s: float


class AsyncPolicySession:
    """One policy, one executor, one generation-isolated action queue."""

    def __init__(
        self,
        policy_id: str,
        backend: PolicyBackend,
        *,
        action_hz: float,
        inference_lock: threading.Lock | None = None,
        clock: Callable[[], float] = time.monotonic,
        snapshot_observation: Callable[[Any], Any] = _snapshot_observation,
    ) -> None:
        if not policy_id:
            raise ValueError("policy_id must not be empty")
        if action_hz <= 0.0:
            raise ValueError("action_hz must be positive")
        self.policy_id = policy_id
        self.backend = backend
        self.action_hz = float(action_hz)
        self._inference_lock = inference_lock or threading.Lock()
        self._clock = clock
        self._snapshot = snapshot_observation
        self._executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix=f"{policy_id}-inference",
        )
        self._lock = threading.Lock()
        self._generation = 0
        self._futures: dict[int, Future[_InferenceResult]] = {}
        self._ready_chunk: PolicyChunk | None = None
        self._active_chunk: PolicyChunk | None = None
        self._active_generation: int | None = None
        self._queue_index = 0
        self._closed = False
        self._warmed = False
        self._warmup_inferences = 0
        self._submitted_inferences = 0
        self._accepted_chunks = 0
        self._stale_chunks_dropped = 0
        self._queue_actions_consumed = 0

    @property
    def generation(self) -> int:
        with self._lock:
            return self._generation

    @property
    def warmed(self) -> bool:
        with self._lock:
            return self._warmed

    def active_remaining_chunk(self) -> PolicyChunk | None:
        """Return an immutable copy of the active, unconsumed action suffix."""

        with self._lock:
            chunk = self._active_chunk
            if (
                chunk is None
                or self._active_generation != chunk.generation
                or self._queue_index >= len(chunk.actions)
            ):
                return None
            actions = chunk.actions[self._queue_index :].copy()
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

    def active_queue_snapshot(
        self,
        *,
        maximum_steps: int,
    ) -> ActivePolicyQueueSnapshot | None:
        """Copy a bounded active suffix and its lineage under one lock.

        Unlike ``active_remaining_chunk``, this includes the exact queue cursor
        and cumulative consumption count needed by a future source splice.
        It never waits for inference and never activates a ready generation.
        """

        if (
            not isinstance(maximum_steps, int)
            or isinstance(maximum_steps, bool)
            or maximum_steps < 1
        ):
            raise ValueError("maximum_steps must be a positive integer")
        with self._lock:
            chunk = self._active_chunk
            if (
                chunk is None
                or self._active_generation != chunk.generation
                or self._queue_index >= len(chunk.actions)
            ):
                return None
            return ActivePolicyQueueSnapshot(
                policy_id=chunk.policy_id,
                generation=chunk.generation,
                queue_index=self._queue_index,
                queue_actions_consumed=self._queue_actions_consumed,
                action_hz=chunk.action_hz,
                actions=chunk.actions[
                    self._queue_index : self._queue_index + maximum_steps
                ],
            )

    @property
    def ready_chunk(self) -> PolicyChunk | None:
        with self._lock:
            return self._ready_chunk

    @property
    def active_generation(self) -> int | None:
        with self._lock:
            return self._active_generation

    @property
    def queue_size(self) -> int:
        with self._lock:
            if (
                self._active_chunk is None
                or self._active_generation != self._active_chunk.generation
            ):
                return 0
            return max(0, len(self._active_chunk.actions) - self._queue_index)

    def warmup(self, observation: Any, *, inferences: int) -> list[float]:
        """Run allocation/kernel warmup and discard every produced action."""

        if inferences < 0:
            raise ValueError("warmup inference count must be non-negative")
        sample = self._snapshot(observation)
        latencies: list[float] = []
        for _ in range(inferences):
            started = self._clock()
            with self._inference_lock:
                self.backend.reset()
                actions = np.asarray(self.backend.infer(sample))
            completed = self._clock()
            if actions.ndim != 2 or actions.shape[1] < 3:
                raise PolicyInferenceError("warmup produced an invalid action chunk")
            latencies.append(completed - started)
        with self._inference_lock:
            self.backend.reset()
        with self._lock:
            self._warmup_inferences += inferences
            self._warmed = True
            # Warmup output is deliberately never installed in _ready_chunk.
            self._ready_chunk = None
            self._active_chunk = None
            self._active_generation = None
            self._queue_index = 0
        return latencies

    def prime(
        self,
        observation: Any,
        *,
        observation_timestamp_s: float,
        preserve_active: bool = False,
    ) -> int:
        """Start a fresh inference generation from the latest observation.

        preserve_active stages a replacement generation while the current
        action suffix remains consumable. The replacement is never executable
        until an explicit activate call, so validation and an atomic queue swap
        can happen outside the inference worker.
        """

        with self._lock:
            if self._closed:
                raise RuntimeError(f"policy session {self.policy_id} is closed")
            if not isinstance(preserve_active, bool):
                raise ValueError("preserve_active must be boolean")
            self._generation += 1
            generation = self._generation
            self._ready_chunk = None
            if not preserve_active:
                self._active_chunk = None
                self._active_generation = None
                self._queue_index = 0
            self._submitted_inferences += 1
        request_timestamp_s = self._clock()
        sample = self._snapshot(observation)
        snapshot_completed_timestamp_s = self._clock()
        submitted_timestamp_s = self._clock()
        future = self._executor.submit(
            self._infer_job,
            sample,
            float(observation_timestamp_s),
            request_timestamp_s,
            snapshot_completed_timestamp_s,
            submitted_timestamp_s,
        )
        with self._lock:
            self._futures[generation] = future
        return generation

    def _infer_job(
        self,
        observation: Any,
        observation_timestamp_s: float,
        request_timestamp_s: float,
        snapshot_completed_timestamp_s: float,
        submitted_timestamp_s: float,
    ) -> _InferenceResult:
        worker_started = self._clock()
        with self._inference_lock:
            gpu_acquired = self._clock()
            self.backend.reset()
            actions = np.asarray(self.backend.infer(observation), dtype=np.float64)
        completed = self._clock()
        return _InferenceResult(
            actions=actions,
            observation_timestamp_s=observation_timestamp_s,
            request_timestamp_s=request_timestamp_s,
            snapshot_completed_timestamp_s=snapshot_completed_timestamp_s,
            submitted_timestamp_s=submitted_timestamp_s,
            worker_started_timestamp_s=worker_started,
            gpu_acquired_timestamp_s=gpu_acquired,
            completed_timestamp_s=completed,
        )

    def poll(self) -> PolicyChunk | None:
        """Accept only a completed result for the current generation."""

        completed_items: list[tuple[int, Future[_InferenceResult]]] = []
        with self._lock:
            for generation, future in list(self._futures.items()):
                if future.done():
                    completed_items.append((generation, future))
                    del self._futures[generation]
            current_generation = self._generation
        accepted: PolicyChunk | None = None
        for generation, future in sorted(completed_items):
            try:
                result = future.result()
            except Exception as exc:
                if generation == current_generation:
                    raise PolicyInferenceError(
                        f"{self.policy_id} generation {generation} inference failed"
                    ) from exc
                with self._lock:
                    self._stale_chunks_dropped += 1
                continue
            with self._lock:
                if generation != self._generation:
                    self._stale_chunks_dropped += 1
                    continue
                accepted = PolicyChunk(
                    policy_id=self.policy_id,
                    generation=generation,
                    observation_timestamp_s=result.observation_timestamp_s,
                    completed_timestamp_s=result.completed_timestamp_s,
                    inference_latency_s=(
                        result.completed_timestamp_s
                        - result.worker_started_timestamp_s
                    ),
                    action_hz=self.action_hz,
                    actions=result.actions,
                    request_timestamp_s=result.request_timestamp_s,
                    snapshot_completed_timestamp_s=(
                        result.snapshot_completed_timestamp_s
                    ),
                    submitted_timestamp_s=result.submitted_timestamp_s,
                    worker_started_timestamp_s=(
                        result.worker_started_timestamp_s
                    ),
                    gpu_acquired_timestamp_s=result.gpu_acquired_timestamp_s,
                )
                self._ready_chunk = accepted
                self._accepted_chunks += 1
        return accepted

    def wait_for_chunk(self, generation: int, timeout_s: float) -> PolicyChunk:
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            chunk = self.poll()
            if chunk is not None and chunk.generation == generation:
                return chunk
            time.sleep(0.001)
        raise TimeoutError(
            f"{self.policy_id} generation {generation} did not finish in {timeout_s:.3f}s"
        )

    def activate(self, generation: int, *, actions: np.ndarray | None = None) -> None:
        """Atomically make a validated ready generation executable.

        A caller may provide a delay-compensated or overlap-blended action
        array. Timing and generation metadata still come from the ready
        inference result.
        """

        with self._lock:
            if (
                self._ready_chunk is None
                or self._ready_chunk.generation != generation
                or generation != self._generation
            ):
                raise RuntimeError("cannot activate a missing or stale policy chunk")
            chunk = self._ready_chunk
            if actions is not None:
                chunk = PolicyChunk(
                    policy_id=chunk.policy_id,
                    generation=chunk.generation,
                    observation_timestamp_s=chunk.observation_timestamp_s,
                    completed_timestamp_s=chunk.completed_timestamp_s,
                    inference_latency_s=chunk.inference_latency_s,
                    action_hz=chunk.action_hz,
                    actions=np.asarray(actions, dtype=np.float64),
                    request_timestamp_s=chunk.request_timestamp_s,
                    snapshot_completed_timestamp_s=chunk.snapshot_completed_timestamp_s,
                    submitted_timestamp_s=chunk.submitted_timestamp_s,
                    worker_started_timestamp_s=chunk.worker_started_timestamp_s,
                    gpu_acquired_timestamp_s=chunk.gpu_acquired_timestamp_s,
                )
                self._ready_chunk = chunk
            self._active_chunk = chunk
            self._active_generation = generation
            self._queue_index = 0

    def pop_action(self) -> np.ndarray | None:
        with self._lock:
            if (
                self._active_chunk is None
                or self._active_generation != self._active_chunk.generation
                or self._queue_index >= len(self._active_chunk.actions)
            ):
                return None
            action = self._active_chunk.actions[self._queue_index].copy()
            self._queue_index += 1
            self._queue_actions_consumed += 1
            return action

    def deactivate_and_clear(self) -> int:
        """Invalidate queued and in-flight work without waiting for inference."""

        with self._lock:
            self._generation += 1
            self._ready_chunk = None
            self._active_chunk = None
            self._active_generation = None
            self._queue_index = 0
            return self._generation

    def stats(self) -> PolicySessionStats:
        with self._lock:
            return PolicySessionStats(
                warmup_inferences=self._warmup_inferences,
                submitted_inferences=self._submitted_inferences,
                accepted_chunks=self._accepted_chunks,
                stale_chunks_dropped=self._stale_chunks_dropped,
                queue_actions_consumed=self._queue_actions_consumed,
            )

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._generation += 1
            self._ready_chunk = None
            self._active_chunk = None
            self._active_generation = None
        self._executor.shutdown(wait=True, cancel_futures=False)

    def __enter__(self) -> "AsyncPolicySession":
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()
