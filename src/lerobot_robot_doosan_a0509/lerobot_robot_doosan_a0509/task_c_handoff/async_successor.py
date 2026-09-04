"""Non-blocking ACT-B request routing and optional detailed timing."""

from __future__ import annotations

import threading
import time
from dataclasses import asdict, dataclass
from typing import Any, Callable

import numpy as np

from offline_tools.task_c_bridge_v0.runtime_policy import (
    AsyncPolicySession,
    PolicyChunk,
    PolicyInferenceError,
)


@dataclass(frozen=True)
class BackendTiming:
    validation_latency_s: float
    preprocess_latency_s: float
    gpu_inference_latency_s: float
    postprocess_latency_s: float
    total_latency_s: float

    def record_ms(self) -> dict[str, float]:
        return {
            key.removesuffix("_s") + "_ms": float(value) * 1000.0
            for key, value in asdict(self).items()
        }


class TimedACTBackend:
    """Instrument one existing LeRobotACTBackend without reloading weights."""

    def __init__(self, backend: Any, *, clock: Callable[[], float] = time.perf_counter):
        self.backend = backend
        self.clock = clock
        self._lock = threading.Lock()
        self._last_timing: BackendTiming | None = None

    def __getattr__(self, name: str) -> Any:
        return getattr(self.backend, name)

    @property
    def last_timing(self) -> BackendTiming | None:
        with self._lock:
            return self._last_timing

    def reset(self) -> None:
        self.backend.reset()

    def infer(self, observation: Any) -> np.ndarray:
        """Mirror LeRobotACTBackend.infer with synchronized stage timing."""

        import torch

        started = self.clock()
        raw = self.backend._validate_raw_observation(observation)
        validated = self.clock()
        batch = self.backend.preprocessor(raw)
        if self.backend.device.type == "cuda":
            torch.cuda.synchronize(self.backend.device)
        preprocessed = self.clock()
        with torch.inference_mode():
            normalized = self.backend.policy.predict_action_chunk(batch)
        if self.backend.device.type == "cuda":
            torch.cuda.synchronize(self.backend.device)
        inferred = self.clock()
        physical = self.backend.postprocessor(normalized)
        if self.backend.device.type == "cuda":
            torch.cuda.synchronize(self.backend.device)
        actions = physical.detach().to(dtype=torch.float32, device="cpu").numpy()
        completed = self.clock()
        if actions.ndim != 3 or actions.shape[0] != 1:
            raise RuntimeError(
                f"ACT returned unexpected postprocessed shape {actions.shape}"
            )
        actions = actions[0]
        expected = (self.backend.action_steps, self.backend.action_dim)
        if actions.shape != expected:
            raise RuntimeError(f"ACT returned {actions.shape}, expected {expected}")
        if not np.all(np.isfinite(actions)):
            raise RuntimeError("ACT returned non-finite physical actions")
        timing = BackendTiming(
            validation_latency_s=validated - started,
            preprocess_latency_s=preprocessed - validated,
            gpu_inference_latency_s=inferred - preprocessed,
            postprocess_latency_s=completed - inferred,
            total_latency_s=completed - started,
        )
        with self._lock:
            self._last_timing = timing
        return actions.astype(np.float64, copy=False)


@dataclass(frozen=True)
class SuccessorResult:
    chunk: PolicyChunk | None
    status: str
    generation: int | None
    stale: bool
    failure_reason: str | None
    backend_timing: BackendTiming | None


class AsyncSuccessorController:
    """One in-flight B request; every control-facing method is non-blocking."""

    def __init__(
        self,
        session: AsyncPolicySession,
        *,
        max_result_age_s: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if max_result_age_s <= 0.0:
            raise ValueError("max_result_age_s must be positive")
        self.session = session
        self.max_result_age_s = float(max_result_age_s)
        self.clock = clock
        self._inflight_generation: int | None = None
        self._request_timestamp_s: float | None = None
        self._last_completed_generation: int | None = None

    @property
    def inflight_generation(self) -> int | None:
        return self._inflight_generation

    @property
    def request_timestamp_s(self) -> float | None:
        return self._request_timestamp_s

    def request(
        self,
        policy_input: Any,
        *,
        observation_timestamp_s: float,
    ) -> int | None:
        if self._inflight_generation is not None:
            return None
        generation = self.session.prime(
            policy_input,
            observation_timestamp_s=observation_timestamp_s,
            preserve_active=False,
        )
        self._inflight_generation = generation
        self._request_timestamp_s = self.clock()
        return generation

    def poll(self, *, now_s: float) -> SuccessorResult:
        generation = self._inflight_generation
        if generation is None:
            return SuccessorResult(None, "idle", None, False, None, None)
        try:
            chunk = self.session.poll()
        except PolicyInferenceError as exc:
            self._inflight_generation = None
            self._request_timestamp_s = None
            self.session.deactivate_and_clear()
            return SuccessorResult(
                None,
                "failed",
                generation,
                False,
                f"{type(exc).__name__}: {exc}",
                self._backend_timing(),
            )
        if chunk is None:
            return SuccessorResult(None, "pending", generation, False, None, None)
        self._inflight_generation = None
        self._request_timestamp_s = None
        self._last_completed_generation = chunk.generation
        if chunk.generation != generation:
            self.session.deactivate_and_clear()
            return SuccessorResult(
                None,
                "rejected",
                chunk.generation,
                True,
                "generation_mismatch",
                self._backend_timing(),
            )
        age = chunk.observation_age_s(now_s)
        if age > self.max_result_age_s:
            self.session.deactivate_and_clear()
            return SuccessorResult(
                None,
                "rejected",
                chunk.generation,
                True,
                f"stale_result_age_s={age:.6f}",
                self._backend_timing(),
            )
        return SuccessorResult(
            chunk,
            "ready",
            generation,
            False,
            None,
            self._backend_timing(),
        )

    def reject_ready(self) -> int:
        return self.session.deactivate_and_clear()

    def activate_ready(self, chunk: PolicyChunk, *, skip_actions: int) -> None:
        if skip_actions < 0 or skip_actions >= len(chunk.actions):
            raise ValueError("successor activation skip lies outside chunk")
        self.session.activate(
            chunk.generation,
            actions=chunk.actions[skip_actions:].copy(),
        )

    def invalidate(self) -> int:
        self._inflight_generation = None
        self._request_timestamp_s = None
        return self.session.deactivate_and_clear()

    def _backend_timing(self) -> BackendTiming | None:
        backend = self.session.backend
        timing = getattr(backend, "last_timing", None)
        return timing if isinstance(timing, BackendTiming) else None
