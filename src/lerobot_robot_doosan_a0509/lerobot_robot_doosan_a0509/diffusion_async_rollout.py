"""Diffusion-only asynchronous rollout support for the A0509.

This module deliberately does not modify :mod:`act_async_rollout`.  It keeps
the accepted ACT architecture (background inference, a fixed-rate consumer,
Live generation gating, and physical-space overlap blending), while adapting
the temporal and queue contracts to the A0509 Diffusion checkpoint:

* two consecutive 30 Hz observations are retained for ``n_obs_steps=2``;
* each runtime chunk is trimmed by its measured observation age;
* one Diffusion prior is retained for the duration of a Live generation;
* the three most recent predictions are aligned by intended execution tick and
  ensembled in Cartesian/quaternion space;
* stale chunks and inferences from a previous Live generation are discarded.
"""

from __future__ import annotations

import logging
import math
import os
import time
import traceback
from collections import deque
from contextlib import nullcontext
from dataclasses import dataclass
from functools import wraps
from pathlib import Path
from threading import Lock, Thread
from typing import Any
from weakref import WeakSet

import torch

from lerobot.policies.rtc import ActionQueue, LatencyTracker
from lerobot.policies.rtc.configuration_rtc import RTCConfig
from lerobot.policies.utils import prepare_observation_for_inference
from lerobot.rollout.inference.rtc import RTCInferenceEngine
from lerobot.utils.feature_utils import build_dataset_frame
from lerobot_robot_doosan_a0509.runtime_scheduling import pin_current_thread_from_env
from quest_a0509_teleop.doosan_orientation import (
    doosan_zyz_deg_to_quaternion,
    quaternion_slerp,
    quaternion_to_doosan_zyz_deg,
)

logger = logging.getLogger(__name__)

_POLICY_PATCH_MARKER = "_a0509_diffusion_async_compat"
_POLICY_WARMUP_MARKER = "_a0509_diffusion_async_warmed"
_INFERENCE_PIN_MARKER = "_a0509_diffusion_inference_thread_pinned"
_ENGINE_PATCH_MARKER = "_a0509_diffusion_rtc_engine"

_RTC_IDLE_SLEEP_S = 0.01
_RTC_ERROR_RETRY_DELAY_S = 0.5
_RTC_MAX_CONSECUTIVE_ERRORS = 10

_live_state_lock = Lock()
_live_gate_active = False
_live_enabled = False
_live_generation = 0
_live_queue_ready = False
_known_action_queues: WeakSet[DiffusionActionQueue] = WeakSet()


def _environment_nonnegative_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    value = default if raw is None else int(raw)
    if value < 0:
        raise ValueError(f"{name} must be non-negative, got {value}")
    return value


def _environment_positive_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    value = default if raw is None else float(raw)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be a positive finite value, got {value}")
    return value


def _environment_unit_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    value = default if raw is None else float(raw)
    if not math.isfinite(value) or not 0.0 <= value <= 1.0:
        raise ValueError(f"{name} must be finite and in [0, 1], got {value}")
    return value


class DiffusionLiveNoiseCache:
    """Keep the Diffusion prior continuous within one Live generation.

    A correlation of ``1.0`` reuses exactly the same prior for every replan.
    Values below one retain the requested fraction of the previous prior and
    add fresh Gaussian noise. A Live rising edge changes ``generation`` and
    therefore always starts from a new independent prior.
    """

    def __init__(self, correlation: float = 1.0) -> None:
        if not math.isfinite(correlation) or not 0.0 <= correlation <= 1.0:
            raise ValueError("noise correlation must be finite and in [0, 1]")
        self.correlation = float(correlation)
        self._generation: int | None = None
        self._noise: torch.Tensor | None = None

    def reset(self) -> None:
        self._generation = None
        self._noise = None

    def get(
        self,
        policy: Any,
        batch: dict[str, Any],
        *,
        generation: int,
    ) -> tuple[torch.Tensor, bool]:
        state = batch.get("observation.state")
        if not isinstance(state, torch.Tensor) or state.ndim < 1:
            raise ValueError("Diffusion batch is missing observation.state")
        parameter = next(policy.parameters())
        shape = (
            int(state.shape[0]),
            int(policy.config.horizon),
            int(policy.config.action_feature.shape[0]),
        )
        needs_new_generation = (
            self._noise is None
            or self._generation != int(generation)
            or tuple(self._noise.shape) != shape
            or self._noise.device != parameter.device
            or self._noise.dtype != parameter.dtype
        )
        if needs_new_generation:
            self._noise = torch.randn(
                shape,
                dtype=parameter.dtype,
                device=parameter.device,
            )
            self._generation = int(generation)
            regenerated = True
        elif self.correlation < 1.0:
            independent = torch.randn_like(self._noise)
            fresh_scale = math.sqrt(max(0.0, 1.0 - self.correlation**2))
            self._noise.mul_(self.correlation).add_(independent, alpha=fresh_scale)
            regenerated = False
        else:
            regenerated = False

        # Protect the cached prior if a scheduler implementation mutates its
        # input sample in-place.
        return self._noise.clone(), regenerated


@dataclass(frozen=True)
class TimeAlignedDiffusionChunk:
    """Raw prediction tagged with the first control tick it can execute on."""

    start_tick: int
    original: torch.Tensor
    processed: torch.Tensor


def _chunk_action_at_tick(
    chunk: TimeAlignedDiffusionChunk,
    tick: int,
) -> tuple[torch.Tensor, torch.Tensor] | None:
    offset = int(tick) - chunk.start_tick
    if offset < 0 or offset >= len(chunk.processed):
        return None
    return chunk.original[offset], chunk.processed[offset]


def ensemble_time_aligned_diffusion_chunks(
    new_original: torch.Tensor,
    new_processed: torch.Tensor,
    *,
    start_tick: int,
    history: list[TimeAlignedDiffusionChunk],
    weight_decay: float = 0.5,
) -> tuple[torch.Tensor, torch.Tensor, int, int]:
    """Ensemble predictions that refer to the same future control tick.

    The current prediction receives weight one. Successively older chunks
    receive ``weight_decay ** age``. Cartesian positions are averaged in
    physical space and Doosan ZYZ orientations are combined through
    quaternion SLERP. The gripper remains discrete and always comes from the
    newest prediction.
    """

    if new_original.ndim != 2 or new_processed.ndim != 2:
        raise ValueError("action chunks must have shape [steps, action_dim]")
    if new_original.shape != new_processed.shape:
        raise ValueError("normalized and physical Diffusion chunks must align")
    if new_processed.shape[1] < 7:
        raise ValueError("A0509 action chunks must contain at least seven values")
    if start_tick < 0:
        raise ValueError("start_tick must be non-negative")
    if not math.isfinite(weight_decay) or not 0.0 <= weight_decay <= 1.0:
        raise ValueError("ensemble weight_decay must be finite and in [0, 1]")

    original_out = new_original.clone()
    processed_out = new_processed.clone()
    ensemble_steps = 0
    max_contributors = 1
    previous_orientation: list[float] | None = None

    newest_first = list(reversed(history))
    for index in range(len(new_processed)):
        tick = int(start_tick) + index
        candidates: list[tuple[float, torch.Tensor, torch.Tensor]] = [
            (1.0, new_original[index], new_processed[index])
        ]
        for age, chunk in enumerate(newest_first, start=1):
            action = _chunk_action_at_tick(chunk, tick)
            if action is None:
                continue
            weight = weight_decay**age
            if weight > 0.0:
                candidates.append((weight, action[0], action[1]))

        if len(candidates) == 1:
            previous_orientation = [
                float(value) for value in new_processed[index, 3:6].tolist()
            ]
            continue

        ensemble_steps += 1
        max_contributors = max(max_contributors, len(candidates))
        weights = torch.as_tensor(
            [candidate[0] for candidate in candidates],
            dtype=new_processed.dtype,
            device=new_processed.device,
        )
        weights /= weights.sum()

        original_values = torch.stack(
            [
                candidate[1].to(
                    dtype=new_original.dtype,
                    device=new_original.device,
                )
                for candidate in candidates
            ]
        )
        processed_values = torch.stack(
            [
                candidate[2].to(
                    dtype=new_processed.dtype,
                    device=new_processed.device,
                )
                for candidate in candidates
            ]
        )
        original_weights = weights.to(
            dtype=new_original.dtype,
            device=new_original.device,
        )
        original_out[index, :6] = (
            original_weights.unsqueeze(1) * original_values[:, :6]
        ).sum(dim=0)
        processed_out[index, :3] = (
            weights.unsqueeze(1) * processed_values[:, :3]
        ).sum(dim=0)

        newest_zyz = [float(value) for value in candidates[0][2][3:6].tolist()]
        quaternion = doosan_zyz_deg_to_quaternion(newest_zyz)
        accumulated_weight = candidates[0][0]
        for weight, _original, processed in candidates[1:]:
            older_zyz = [float(value) for value in processed[3:6].tolist()]
            quaternion = quaternion_slerp(
                quaternion,
                doosan_zyz_deg_to_quaternion(older_zyz),
                weight / (accumulated_weight + weight),
            )
            accumulated_weight += weight
        reference = newest_zyz if previous_orientation is None else previous_orientation
        previous_orientation = quaternion_to_doosan_zyz_deg(quaternion, reference)
        processed_out[index, 3:6] = torch.as_tensor(
            previous_orientation,
            dtype=processed_out.dtype,
            device=processed_out.device,
        )

        # Never average the discrete gripper channel. Existing MUX thresholds
        # remain the sole open/close decision layer.
        original_out[index, 6:] = new_original[index, 6:]
        processed_out[index, 6:] = new_processed[index, 6:]

    return original_out, processed_out, ensemble_steps, max_contributors


def configure_diffusion_policy_live_queue(enabled: bool) -> None:
    """Enable the Live gate only for a commanding Diffusion rollout."""

    global _live_gate_active, _live_enabled, _live_queue_ready
    requested = bool(enabled)
    with _live_state_lock:
        _live_gate_active = requested
        if not requested:
            _live_enabled = False
            _live_queue_ready = False


def notify_diffusion_policy_live_state(enabled: bool) -> int:
    """Advance the queue generation on Live rising edges.

    A chunk generated before the rising edge must never become executable.
    Clearing every registered queue also wakes the background worker because
    the remaining queue size immediately falls below its inference threshold.
    """

    global _live_enabled, _live_generation, _live_queue_ready
    requested = bool(enabled)
    queues_to_clear: list[DiffusionActionQueue] = []
    with _live_state_lock:
        if requested and not _live_enabled:
            _live_generation += 1
            _live_queue_ready = False
            queues_to_clear = list(_known_action_queues)
            logger.info(
                "Diffusion Live rising edge: fresh queue generation=%d requested",
                _live_generation,
            )
        elif not requested:
            _live_queue_ready = False
        _live_enabled = requested
        generation = _live_generation

    for queue in queues_to_clear:
        queue.clear_for_generation(generation)
    return generation


def diffusion_policy_live_queue_ready() -> bool:
    with _live_state_lock:
        return _live_gate_active and _live_enabled and _live_queue_ready


def _policy_live_snapshot() -> tuple[bool, bool, int, bool]:
    with _live_state_lock:
        return _live_gate_active, _live_enabled, _live_generation, _live_queue_ready


def _mark_policy_queue_ready(generation: int) -> None:
    global _live_queue_ready
    with _live_state_lock:
        if _live_gate_active and _live_enabled and generation == _live_generation:
            if not _live_queue_ready:
                logger.info(
                    "Diffusion fresh action queue ready: generation=%d",
                    generation,
                )
            _live_queue_ready = True


def _mark_policy_queue_not_ready() -> None:
    global _live_queue_ready
    with _live_state_lock:
        _live_queue_ready = False


def _reset_diffusion_policy_live_state_for_test() -> None:
    global _live_gate_active, _live_enabled, _live_generation, _live_queue_ready
    with _live_state_lock:
        _live_gate_active = False
        _live_enabled = False
        _live_generation = 0
        _live_queue_ready = False
        _known_action_queues.clear()


def _smoothstep_weights(
    steps: int,
    *,
    dtype: torch.dtype,
    device: torch.device,
) -> torch.Tensor:
    if steps <= 0:
        return torch.empty((0, 1), dtype=dtype, device=device)
    fractions = torch.arange(1, steps + 1, dtype=dtype, device=device)
    fractions /= float(steps + 1)
    return (3.0 * fractions.square() - 2.0 * fractions.pow(3)).unsqueeze(1)


def blend_diffusion_processed_overlap(
    old_actions: torch.Tensor,
    new_actions: torch.Tensor,
    overlap_steps: int,
) -> torch.Tensor:
    """Blend an old physical tail into a complete new Diffusion chunk."""

    if old_actions.ndim != 2 or new_actions.ndim != 2:
        raise ValueError("action chunks must have shape [steps, action_dim]")
    if old_actions.shape[1] != new_actions.shape[1]:
        raise ValueError("old and new action dimensions must match")
    if new_actions.shape[1] < 7:
        raise ValueError("A0509 action chunks must contain at least seven values")

    steps = min(overlap_steps, len(old_actions), len(new_actions))
    if steps <= 0:
        return new_actions.clone()

    weights = _smoothstep_weights(
        steps,
        dtype=new_actions.dtype,
        device=new_actions.device,
    )
    old_prefix = old_actions[:steps].to(
        device=new_actions.device,
        dtype=new_actions.dtype,
    )
    blended = old_prefix.clone()
    blended[:, :3] = (
        (1.0 - weights) * old_prefix[:, :3]
        + weights * new_actions[:steps, :3]
    )

    previous_orientation: list[float] | None = None
    for index in range(steps):
        fraction = float(weights[index, 0].item())
        old_zyz = [float(value) for value in old_prefix[index, 3:6].tolist()]
        new_zyz = [float(value) for value in new_actions[index, 3:6].tolist()]
        quaternion = quaternion_slerp(
            doosan_zyz_deg_to_quaternion(old_zyz),
            doosan_zyz_deg_to_quaternion(new_zyz),
            fraction,
        )
        reference = old_zyz if previous_orientation is None else previous_orientation
        previous_orientation = quaternion_to_doosan_zyz_deg(quaternion, reference)
        blended[index, 3:6] = torch.as_tensor(
            previous_orientation,
            dtype=blended.dtype,
            device=blended.device,
        )

    # The gripper is a discrete state.  Never synthesize an intermediate
    # open/close command while Cartesian actions are crossing a chunk boundary.
    blended[:, 6:] = old_prefix[:, 6:]
    return torch.cat((blended, new_actions[steps:].clone()), dim=0)


def blend_diffusion_original_overlap(
    old_actions: torch.Tensor,
    new_actions: torch.Tensor,
    overlap_steps: int,
) -> torch.Tensor:
    """Keep the normalized queue length-aligned with the physical queue."""

    if old_actions.ndim != 2 or new_actions.ndim != 2:
        raise ValueError("action chunks must have shape [steps, action_dim]")
    if old_actions.shape[1] != new_actions.shape[1]:
        raise ValueError("old and new action dimensions must match")
    steps = min(overlap_steps, len(old_actions), len(new_actions))
    if steps <= 0:
        return new_actions.clone()
    weights = _smoothstep_weights(
        steps,
        dtype=new_actions.dtype,
        device=new_actions.device,
    )
    old_prefix = old_actions[:steps].to(
        device=new_actions.device,
        dtype=new_actions.dtype,
    )
    blended = (1.0 - weights) * old_prefix + weights * new_actions[:steps]
    if blended.shape[1] >= 7:
        blended[:, 6:] = old_prefix[:, 6:]
    return torch.cat((blended, new_actions[steps:].clone()), dim=0)


def merge_diffusion_action_chunks(
    old_original: torch.Tensor,
    old_processed: torch.Tensor,
    new_original: torch.Tensor,
    new_processed: torch.Tensor,
    *,
    overlap_steps: int,
    delay_steps: int = 0,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Replace an old tail with a delay-aligned latest Diffusion chunk.

    The new action at delay_steps is the first prediction whose intended
    execution time is not earlier than the merge time. Once that stale prefix
    is removed, both remaining prefixes describe the same future control ticks
    and may be blended safely.
    """

    if delay_steps < 0:
        raise ValueError("delay_steps must be non-negative")
    if len(new_original) != len(new_processed):
        raise ValueError("normalized and physical Diffusion chunks must align")

    aligned_original = new_original[delay_steps:].clone()
    aligned_processed = new_processed[delay_steps:].clone()
    return (
        blend_diffusion_original_overlap(
            old_original,
            aligned_original,
            overlap_steps,
        ),
        blend_diffusion_processed_overlap(
            old_processed,
            aligned_processed,
            overlap_steps,
        ),
    )


class DiffusionActionQueue(ActionQueue):
    """Latest-chunk queue with Live generation and stale-action guards."""

    def __init__(
        self,
        cfg: RTCConfig,
        *,
        overlap_steps: int = 3,
        expected_chunk_steps: int = 15,
        max_action_age_sec: float = 0.50,
        ensemble_history_size: int = 3,
        ensemble_weight_decay: float = 0.5,
    ) -> None:
        if cfg.enabled:
            raise ValueError("A0509 Diffusion requires FIFO mode: rtc.enabled=false")
        if overlap_steps < 0:
            raise ValueError("overlap_steps must be non-negative")
        if expected_chunk_steps <= 0:
            raise ValueError("expected_chunk_steps must be positive")
        if max_action_age_sec <= 0.0:
            raise ValueError("max_action_age_sec must be positive")
        if ensemble_history_size <= 0:
            raise ValueError("ensemble_history_size must be positive")
        if not 0.0 <= ensemble_weight_decay <= 1.0:
            raise ValueError("ensemble_weight_decay must be in [0, 1]")
        super().__init__(cfg)
        self.overlap_steps = overlap_steps
        self.expected_chunk_steps = expected_chunk_steps
        self.max_action_age_sec = max_action_age_sec
        self.ensemble_history_size = ensemble_history_size
        self.ensemble_weight_decay = ensemble_weight_decay
        self._control_tick = 0
        self._chunk_history: deque[TimeAlignedDiffusionChunk] = deque(
            maxlen=max(0, ensemble_history_size - 1)
        )
        self._queue_generation = 0
        self._inference_generation = 0
        self._active_source_time: float | None = None
        self._pending_source_time: float | None = None
        self._pending_inference_latency_sec: float | None = None
        self._last_inference_latency_sec: float | None = None
        self._empty_reported = False
        self._merge_count = 0
        self._underrun_count = 0
        self._stale_action_drop_count = 0
        self._stale_inference_drop_count = 0
        self._delay_expired_chunk_count = 0
        self._delay_trimmed_action_count = 0
        self._ensemble_step_count = 0
        self._max_ensemble_contributors = 1
        with _live_state_lock:
            _known_action_queues.add(self)

    def _clear_locked(self, generation: int | None = None) -> None:
        self.queue = None
        self.original_queue = None
        self.last_index = 0
        self._active_source_time = None
        self._pending_source_time = None
        self._pending_inference_latency_sec = None
        self._empty_reported = False
        self._control_tick = 0
        self._chunk_history.clear()
        if generation is not None:
            self._queue_generation = generation

    def clear(self) -> None:
        with self.lock:
            self._clear_locked()
        _mark_policy_queue_not_ready()

    def clear_for_generation(self, generation: int) -> None:
        with self.lock:
            self._clear_locked(generation)
        _mark_policy_queue_not_ready()
        logger.info("Diffusion queue cleared for Live generation=%d", generation)

    def get_action_index(self) -> int:
        _active, _live, generation, _ready = _policy_live_snapshot()
        with self.lock:
            self._inference_generation = generation
            return self.last_index

    def set_pending_chunk_context(
        self,
        *,
        source_observation_time: float,
        inference_latency_sec: float,
    ) -> None:
        with self.lock:
            self._pending_source_time = float(source_observation_time)
            self._pending_inference_latency_sec = float(inference_latency_sec)

    def get(self) -> torch.Tensor | None:
        live_gate_active, live_enabled, generation, _ready = _policy_live_snapshot()
        if live_gate_active and not live_enabled:
            return None

        mark_not_ready = False
        stale_age: float | None = None
        with self.lock:
            if live_gate_active and self._queue_generation != generation:
                self._clear_locked(generation)
                mark_not_ready = True

            now = time.monotonic()
            if (
                self.queue is not None
                and self._active_source_time is not None
                and now - self._active_source_time > self.max_action_age_sec
            ):
                stale_age = now - self._active_source_time
                self._stale_action_drop_count += 1
                self._clear_locked(generation if live_gate_active else None)
                mark_not_ready = True

            if self.queue is None or self.last_index >= len(self.queue):
                if not self._empty_reported:
                    self._underrun_count += 1
                    self._empty_reported = True
                    logger.warning(
                        "Diffusion action queue empty: underruns=%d",
                        self._underrun_count,
                    )
                mark_not_ready = mark_not_ready or live_gate_active
                action = None
            else:
                action = self.queue[self.last_index].clone()
                self.last_index += 1
            self._control_tick += 1

        if stale_age is not None:
            logger.error(
                "Dropped stale Diffusion action chunk: source_age_ms=%.1f limit_ms=%.1f",
                stale_age * 1000.0,
                self.max_action_age_sec * 1000.0,
            )
        if mark_not_ready:
            _mark_policy_queue_not_ready()
        return action

    def merge(
        self,
        original_actions: torch.Tensor,
        processed_actions: torch.Tensor,
        real_delay: int,
        action_index_before_inference: int | None = None,
    ) -> None:
        del action_index_before_inference
        if real_delay < 0:
            raise ValueError("real_delay must be non-negative")
        if original_actions.ndim != 2 or processed_actions.ndim != 2:
            raise ValueError("Diffusion chunks must have shape [steps, action_dim]")
        if len(original_actions) != self.expected_chunk_steps:
            raise ValueError(
                "unexpected normalized Diffusion chunk length: "
                f"{len(original_actions)} != {self.expected_chunk_steps}"
            )
        if len(processed_actions) != self.expected_chunk_steps:
            raise ValueError(
                "unexpected processed Diffusion chunk length: "
                f"{len(processed_actions)} != {self.expected_chunk_steps}"
            )

        live_gate_active, live_enabled, generation, _ready = _policy_live_snapshot()
        mark_ready = False
        stale_inference = False
        delay_expired = False
        old_tail_steps = 0
        aligned_steps = max(0, self.expected_chunk_steps - int(real_delay))
        overlap_used = 0
        ensemble_contributors = 1
        with self.lock:
            if (
                live_gate_active
                and live_enabled
                and self._inference_generation != generation
            ):
                self._stale_inference_drop_count += 1
                self._pending_source_time = None
                self._pending_inference_latency_sec = None
                stale_inference = True
            else:
                if live_gate_active and live_enabled and self._queue_generation != generation:
                    self._clear_locked(generation)

                if aligned_steps <= 0:
                    self._last_inference_latency_sec = self._pending_inference_latency_sec
                    self._pending_source_time = None
                    self._pending_inference_latency_sec = None
                    self._delay_expired_chunk_count += 1
                    delay_expired = True
                else:
                    old_tail_steps = (
                        0
                        if self.queue is None
                        else max(0, len(self.queue) - self.last_index)
                    )
                    aligned_original = original_actions[int(real_delay) :].clone()
                    aligned_processed = processed_actions[int(real_delay) :].clone()
                    start_tick = self._control_tick
                    (
                        self.original_queue,
                        self.queue,
                        ensemble_steps,
                        max_contributors,
                    ) = ensemble_time_aligned_diffusion_chunks(
                        aligned_original,
                        aligned_processed,
                        start_tick=start_tick,
                        history=list(self._chunk_history),
                        weight_decay=self.ensemble_weight_decay,
                    )
                    self._chunk_history.append(
                        TimeAlignedDiffusionChunk(
                            start_tick=start_tick,
                            original=aligned_original,
                            processed=aligned_processed,
                        )
                    )
                    overlap_used = ensemble_steps
                    ensemble_contributors = max_contributors
                    self._ensemble_step_count += ensemble_steps
                    self._max_ensemble_contributors = max(
                        self._max_ensemble_contributors,
                        max_contributors,
                    )

                    self.last_index = 0
                    self._active_source_time = (
                        time.monotonic()
                        if self._pending_source_time is None
                        else self._pending_source_time
                    )
                    self._last_inference_latency_sec = self._pending_inference_latency_sec
                    self._pending_source_time = None
                    self._pending_inference_latency_sec = None
                    self._delay_trimmed_action_count += int(real_delay)
                    self._merge_count += 1
                    self._empty_reported = False
                    mark_ready = live_gate_active and live_enabled

        if stale_inference:
            logger.warning(
                "Discarded stale Diffusion inference: inference_generation=%d live_generation=%d",
                self._inference_generation,
                generation,
            )
            return
        if delay_expired:
            logger.warning(
                "Discarded delay-expired Diffusion chunk: delay_steps=%d chunk_steps=%d",
                int(real_delay),
                self.expected_chunk_steps,
            )
            return
        if mark_ready:
            _mark_policy_queue_ready(generation)
        logger.info(
            "Diffusion latest chunk merged: ensemble_ticks=%d contributors=%d "
            "old_tail=%d delay_trim=%d aligned=%d queue=%d",
            overlap_used,
            ensemble_contributors,
            old_tail_steps,
            int(real_delay),
            aligned_steps,
            self.qsize(),
        )

    def metrics(self) -> dict[str, int | float | None]:
        with self.lock:
            return {
                "merge_count": self._merge_count,
                "underrun_count": self._underrun_count,
                "stale_action_drop_count": self._stale_action_drop_count,
                "stale_inference_drop_count": self._stale_inference_drop_count,
                "delay_expired_chunk_count": self._delay_expired_chunk_count,
                "delay_trimmed_action_count": self._delay_trimmed_action_count,
                "ensemble_step_count": self._ensemble_step_count,
                "max_ensemble_contributors": self._max_ensemble_contributors,
                "ensemble_history_entries": len(self._chunk_history),
                "remaining_actions": 0
                if self.queue is None
                else max(0, len(self.queue) - self.last_index),
                "last_inference_latency_ms": None
                if self._last_inference_latency_sec is None
                else self._last_inference_latency_sec * 1000.0,
            }


def stack_temporal_observation_batches(
    frames: list[dict[str, Any]],
    *,
    task: str,
) -> dict[str, Any]:
    """Stack single-frame ``[B,...]`` tensors into ``[B,T,...]`` tensors."""

    if not frames:
        raise ValueError("at least one prepared observation frame is required")
    tensor_keys = {
        key for key, value in frames[0].items() if isinstance(value, torch.Tensor)
    }
    if not tensor_keys:
        raise ValueError("prepared observation contains no tensors")
    for frame in frames[1:]:
        keys = {key for key, value in frame.items() if isinstance(value, torch.Tensor)}
        if keys != tensor_keys:
            raise ValueError("temporal observation tensor keys do not match")

    batch = {
        key: torch.stack([frame[key] for frame in frames], dim=1)
        for key in sorted(tensor_keys)
    }
    batch["task"] = [task]
    robot_type = frames[-1].get("robot_type")
    if robot_type is not None:
        batch["robot_type"] = robot_type
    return batch


class A0509DiffusionRTCInferenceEngine(RTCInferenceEngine):
    """RTC worker with a latest-only two-frame observation ring."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        policy = kwargs.get("policy") if "policy" in kwargs else args[0]
        super().__init__(*args, **kwargs)
        self._observation_history_steps = int(policy.config.n_obs_steps)
        if self._observation_history_steps != 2:
            raise ValueError(
                "A0509 Diffusion RTC requires n_obs_steps=2, got "
                f"{self._observation_history_steps}"
            )
        self._observation_history: deque[tuple[float, dict[str, Any]]] = deque(
            maxlen=self._observation_history_steps
        )
        self._max_observation_age_sec = _environment_positive_float(
            "LEROBOT_A0509_DIFFUSION_MAX_OBSERVATION_AGE_SEC",
            0.25,
        )
        self._overlap_steps = _environment_nonnegative_int(
            "LEROBOT_A0509_DIFFUSION_OVERLAP_STEPS",
            3,
        )
        self._max_action_age_sec = _environment_positive_float(
            "LEROBOT_A0509_DIFFUSION_MAX_ACTION_AGE_SEC",
            0.50,
        )
        self._noise_correlation = _environment_unit_float(
            "LEROBOT_A0509_DIFFUSION_NOISE_CORRELATION",
            1.0,
        )
        self._noise_cache = DiffusionLiveNoiseCache(self._noise_correlation)
        self._ensemble_history_size = _environment_nonnegative_int(
            "LEROBOT_A0509_DIFFUSION_ENSEMBLE_HISTORY_SIZE",
            3,
        )
        if self._ensemble_history_size <= 0:
            raise ValueError(
                "LEROBOT_A0509_DIFFUSION_ENSEMBLE_HISTORY_SIZE must be positive"
            )
        self._ensemble_weight_decay = _environment_unit_float(
            "LEROBOT_A0509_DIFFUSION_ENSEMBLE_WEIGHT_DECAY",
            0.5,
        )

    def reset(self) -> None:
        super().reset()
        self._noise_cache.reset()
        with self._obs_lock:
            self._observation_history.clear()

    def start(self) -> None:
        self._action_queue = DiffusionActionQueue(
            self._rtc_config,
            overlap_steps=self._overlap_steps,
            expected_chunk_steps=int(self._policy.config.n_action_steps),
            max_action_age_sec=self._max_action_age_sec,
            ensemble_history_size=self._ensemble_history_size,
            ensemble_weight_decay=self._ensemble_weight_decay,
        )
        with self._obs_lock:
            self._obs_holder = {"obs": None, "robot_type": self._robot.robot_type}
            self._observation_history.clear()
        self._shutdown_event.clear()
        self._rtc_thread = Thread(
            target=self._rtc_loop,
            daemon=True,
            name="A0509DiffusionInference",
        )
        self._rtc_thread.start()
        logger.info(
            "A0509 Diffusion inference thread started: observations=%d "
            "ensemble_history=%d ensemble_decay=%.3f noise_correlation=%.3f",
            self._observation_history_steps,
            self._ensemble_history_size,
            self._ensemble_weight_decay,
            self._noise_correlation,
        )

    def notify_observation(self, obs: dict[str, Any]) -> None:
        receive_time = time.monotonic()
        with self._obs_lock:
            snapshot = dict(obs)
            self._obs_holder["obs"] = snapshot
            self._observation_history.append((receive_time, snapshot))

    def _snapshot_observation_history(
        self,
    ) -> tuple[float, list[dict[str, Any]]] | None:
        with self._obs_lock:
            history = list(self._observation_history)
        if not history:
            return None
        while len(history) < self._observation_history_steps:
            history.insert(0, history[0])
        history = history[-self._observation_history_steps :]
        return history[-1][0], [observation for _stamp, observation in history]

    def _prepare_temporal_batch(
        self,
        observations: list[dict[str, Any]],
        policy_device: torch.device,
    ) -> dict[str, Any]:
        prepared_frames: list[dict[str, Any]] = []
        for observation in observations:
            frame = build_dataset_frame(
                self._hw_features,
                observation,
                prefix="observation",
            )
            prepared_frames.append(
                prepare_observation_for_inference(
                    frame,
                    policy_device,
                    self._task,
                    self._robot.robot_type,
                )
            )
        return stack_temporal_observation_batches(
            prepared_frames,
            task=self._task,
        )

    def _rtc_loop(self) -> None:
        try:
            latency_tracker = LatencyTracker()
            action_period_s = 1.0 / self._fps
            policy_device = torch.device(self._device)
            consecutive_errors = 0

            while not self._shutdown_event.is_set():
                if not self._policy_active.is_set():
                    time.sleep(_RTC_IDLE_SLEEP_S)
                    continue

                queue = self._action_queue
                snapshot = self._snapshot_observation_history()
                if queue is None or snapshot is None:
                    time.sleep(_RTC_IDLE_SLEEP_S)
                    continue
                if queue.qsize() > self._rtc_queue_threshold:
                    time.sleep(_RTC_IDLE_SLEEP_S)
                    continue

                source_observation_time, observations = snapshot
                observation_age = time.monotonic() - source_observation_time
                if observation_age > self._max_observation_age_sec:
                    logger.error(
                        "Diffusion observation stale before inference: age_ms=%.1f limit_ms=%.1f",
                        observation_age * 1000.0,
                        self._max_observation_age_sec * 1000.0,
                    )
                    time.sleep(_RTC_IDLE_SLEEP_S)
                    continue

                try:
                    started = time.perf_counter()
                    index_before = queue.get_action_index()
                    previous_latency = latency_tracker.max()
                    estimated_delay = (
                        math.ceil(previous_latency / action_period_s)
                        if previous_latency
                        else 0
                    )
                    temporal_batch = self._prepare_temporal_batch(
                        observations,
                        policy_device,
                    )
                    preprocessed = self._preprocessor(temporal_batch)
                    _active, _live, generation, _ready = _policy_live_snapshot()
                    noise, noise_regenerated = self._noise_cache.get(
                        self._policy,
                        preprocessed,
                        generation=generation,
                    )
                    if noise_regenerated:
                        logger.info(
                            "Diffusion prior initialized for Live generation=%d "
                            "correlation=%.3f",
                            generation,
                            self._noise_correlation,
                        )
                    actions = self._policy.predict_action_chunk(
                        preprocessed,
                        inference_delay=estimated_delay,
                        prev_chunk_left_over=None,
                        noise=noise,
                    )
                    original = actions.squeeze(0).clone()
                    processed = self._postprocessor(actions).squeeze(0)
                    latency_sec = time.perf_counter() - started
                    source_age_sec = time.monotonic() - source_observation_time
                    real_delay = math.ceil(source_age_sec / action_period_s)
                    latency_tracker.add(latency_sec)
                    consecutive_errors = 0

                    queue.set_pending_chunk_context(
                        source_observation_time=source_observation_time,
                        inference_latency_sec=latency_sec,
                    )
                    queue.merge(original, processed, real_delay, index_before)
                    logger.info(
                        "Diffusion async inference complete: latency_ms=%.3f "
                        "source_age_ms=%.3f delay_steps=%d queue=%d",
                        latency_sec * 1000.0,
                        source_age_sec * 1000.0,
                        real_delay,
                        queue.qsize(),
                    )
                except Exception as exc:
                    consecutive_errors += 1
                    logger.error(
                        "Diffusion RTC inference error (%d/%d): %s",
                        consecutive_errors,
                        _RTC_MAX_CONSECUTIVE_ERRORS,
                        exc,
                    )
                    logger.debug(traceback.format_exc())
                    if consecutive_errors >= _RTC_MAX_CONSECUTIVE_ERRORS:
                        raise
                    time.sleep(_RTC_ERROR_RETRY_DELAY_S)
        except Exception as exc:
            logger.error("Fatal Diffusion RTC error: %s", exc)
            logger.error(traceback.format_exc())
            self._rtc_error.set()
            self._compile_warmup_done.set()
            if self._global_shutdown_event is not None:
                self._global_shutdown_event.set()


def _synchronize_if_cuda(actions: Any) -> None:
    if isinstance(actions, torch.Tensor) and actions.is_cuda:
        torch.cuda.synchronize(actions.device)


def _run_diffusion_policy_once(
    policy: Any,
    original: Any,
    batch: dict[str, Any],
    noise: torch.Tensor | None,
) -> torch.Tensor:
    device = next(policy.parameters()).device
    use_amp = bool(getattr(policy.config, "use_amp", False)) and device.type == "cuda"
    amp_context: Any = (
        torch.autocast(device_type=device.type) if use_amp else nullcontext()
    )
    with torch.inference_mode(), amp_context:
        actions = original(policy, batch, noise=noise)
    _synchronize_if_cuda(actions)
    return actions


def install_diffusion_async_chunk_compat(
    warmup_inferences: int | None = None,
) -> bool:
    """Adapt ``DiffusionPolicy.predict_action_chunk`` to the RTC FIFO call."""

    from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy

    original = DiffusionPolicy.predict_action_chunk
    if getattr(original, _POLICY_PATCH_MARKER, False):
        return False
    if warmup_inferences is None:
        warmup_inferences = _environment_nonnegative_int(
            "LEROBOT_A0509_DIFFUSION_WARMUP_INFERENCES",
            2,
        )

    @wraps(original)
    def predict_action_chunk_compat(
        self: Any,
        batch: dict[str, Any],
        *,
        inference_delay: int = 0,
        prev_chunk_left_over: Any | None = None,
        noise: torch.Tensor | None = None,
    ) -> torch.Tensor:
        del prev_chunk_left_over
        if not getattr(self, _INFERENCE_PIN_MARKER, False):
            cpu_set = pin_current_thread_from_env(
                "LEROBOT_A0509_DIFFUSION_INFERENCE_CPU_SET"
            )
            if cpu_set is not None:
                logger.info("Diffusion inference thread pinned to CPUs %s", cpu_set)
            setattr(self, _INFERENCE_PIN_MARKER, True)

        if not getattr(self, _POLICY_WARMUP_MARKER, False):
            for index in range(warmup_inferences):
                started = time.perf_counter()
                _run_diffusion_policy_once(self, original, batch, noise)
                logger.info(
                    "Diffusion CUDA warmup %d/%d completed in %.3f ms",
                    index + 1,
                    warmup_inferences,
                    (time.perf_counter() - started) * 1000.0,
                )
            setattr(self, _POLICY_WARMUP_MARKER, True)

        started = time.perf_counter()
        actions = _run_diffusion_policy_once(self, original, batch, noise)
        latency_ms = (time.perf_counter() - started) * 1000.0
        expected_steps = int(self.config.n_action_steps)
        if (
            actions.ndim != 3
            or actions.shape[1] != expected_steps
            or actions.shape[2] != 7
        ):
            raise RuntimeError(
                "unexpected Diffusion action chunk shape: "
                f"{tuple(actions.shape)}; expected [batch, {expected_steps}, 7]"
            )
        logger.info(
            "Diffusion chunk ready: steps=%d latency_ms=%.3f prior_delay_steps=%d",
            int(actions.shape[1]),
            latency_ms,
            int(inference_delay),
        )
        return actions

    setattr(predict_action_chunk_compat, _POLICY_PATCH_MARKER, True)
    setattr(predict_action_chunk_compat, "_a0509_original", original)
    DiffusionPolicy.predict_action_chunk = predict_action_chunk_compat
    return True


def install_diffusion_rtc_engine() -> bool:
    """Route LeRobot's RTC factory to the Diffusion-specific worker."""

    from lerobot.rollout.inference import factory

    current = factory.RTCInferenceEngine
    if current is A0509DiffusionRTCInferenceEngine:
        return False
    if getattr(current, _ENGINE_PATCH_MARKER, False):
        return False
    setattr(A0509DiffusionRTCInferenceEngine, _ENGINE_PATCH_MARKER, True)
    factory.RTCInferenceEngine = A0509DiffusionRTCInferenceEngine
    return True


def _option_value(arguments: list[str], name: str) -> str | None:
    prefix = f"{name}="
    for index, argument in enumerate(arguments):
        if argument.startswith(prefix):
            return argument[len(prefix) :]
        if argument == name and index + 1 < len(arguments):
            return arguments[index + 1]
    return None


def require_diffusion_async_fifo_arguments(arguments: list[str]) -> None:
    """Enforce the measured A0509 Diffusion async baseline."""

    required = {
        "--inference.type": "rtc",
        "--inference.rtc.enabled": "false",
        "--inference.queue_threshold": "9",
        "--policy.n_action_steps": "15",
        "--robot.type": "doosan_a0509_diffusion_ros",
        "--fps": "30",
    }
    for name, expected in required.items():
        actual = _option_value(arguments, name)
        if actual is None or actual.strip().lower() != expected:
            raise ValueError(
                f"A0509 Diffusion async rollout requires {name}={expected}, got {actual!r}"
            )


def validate_diffusion_policy_config(config: Any) -> None:
    """Reject a checkpoint that does not match the measured runtime contract."""

    from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig

    if not isinstance(config, DiffusionConfig):
        raise ValueError(f"expected DiffusionConfig, got {type(config).__name__}")
    expected_scalars = {
        "n_obs_steps": 2,
        "horizon": 16,
        "n_action_steps": 8,
        "noise_scheduler_type": "DDIM",
        "num_inference_steps": 5,
        "use_amp": True,
        "compile_model": False,
    }
    for name, expected in expected_scalars.items():
        actual = getattr(config, name)
        if actual != expected:
            raise ValueError(
                f"unexpected Diffusion config {name}: {actual!r} != {expected!r}"
            )

    expected_cameras = {
        "observation.images.front",
        "observation.images.side",
        "observation.images.zed_rgb",
    }
    if set(config.image_features) != expected_cameras:
        raise ValueError(
            "unexpected Diffusion camera features: "
            f"{sorted(config.image_features)}"
        )
    for name, feature in config.image_features.items():
        if tuple(feature.shape) != (3, 480, 640):
            raise ValueError(
                f"unexpected Diffusion image shape for {name}: {tuple(feature.shape)}"
            )
    if tuple(config.robot_state_feature.shape) != (13,):
        raise ValueError(
            f"unexpected Diffusion state shape: {tuple(config.robot_state_feature.shape)}"
        )
    if tuple(config.action_feature.shape) != (7,):
        raise ValueError(
            f"unexpected Diffusion action shape: {tuple(config.action_feature.shape)}"
        )


def load_and_validate_diffusion_checkpoint(arguments: list[str]) -> Path:
    from lerobot.configs.policies import PreTrainedConfig

    raw_path = _option_value(arguments, "--policy.path")
    if raw_path is None:
        raise ValueError("A0509 Diffusion rollout requires --policy.path")
    checkpoint = Path(raw_path).expanduser().resolve()
    if not (checkpoint / "model.safetensors").is_file():
        raise ValueError(f"Diffusion checkpoint is missing model.safetensors: {checkpoint}")
    config = PreTrainedConfig.from_pretrained(checkpoint, local_files_only=True)
    validate_diffusion_policy_config(config)
    return checkpoint
