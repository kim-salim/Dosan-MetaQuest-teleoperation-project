"""ACT compatibility for LeRobot's asynchronous chunk worker.

LeRobot 0.6's RTC worker calls ``predict_action_chunk`` with RTC-specific
keyword arguments. ACT predicts chunks but does not implement prefix-guided
RTC, so its method does not accept those arguments. This module adapts ACT to
the worker's non-RTC FIFO mode. Inference runs in the background while the
rollout loop consumes the completed chunk at its fixed control frequency.
"""

from __future__ import annotations

import logging
import os
import time
from functools import wraps
from threading import Lock, RLock, get_native_id
from typing import Any
from weakref import WeakSet

import torch

from lerobot_robot_doosan_a0509.runtime_scheduling import pin_current_thread_from_env
from quest_a0509_teleop.doosan_orientation import (
    doosan_zyz_deg_to_quaternion,
    quaternion_slerp,
    quaternion_to_doosan_zyz_deg,
)

logger = logging.getLogger(__name__)

_PATCH_MARKER = "_a0509_act_async_compat"
_WARMUP_MARKER = "_a0509_act_async_warmed"
_INFERENCE_PIN_MARKER = "_a0509_act_inference_thread_pinned"
_QUEUE_PATCH_MARKER = "_a0509_overlap_queue"
_QUEUE_LIVE_GENERATION = "_a0509_live_generation"
_QUEUE_INFERENCE_GENERATION = "_a0509_inference_generation"

_live_state_lock = Lock()
_live_gate_active = False
_live_enabled = False
_live_generation = 0
_live_queue_ready = False
_known_action_queues: WeakSet[Any] = WeakSet()
_act_gpu_arbiter = RLock()
_act_inference_pin_log_lock = Lock()


def get_act_gpu_arbiter() -> RLock:
    """Return the process-wide ACT CUDA arbiter.

    Task-C keeps ACT-A in LeRobot's RTC worker and ACT-B in a separate
    resident session. Both paths call ``ACTPolicy.predict_action_chunk``;
    sharing this reentrant lock prevents concurrent CUDA inference while
    still allowing the ACT-B session to hold the same lock around reset,
    preprocess, inference, and postprocess.
    """

    return _act_gpu_arbiter


def _register_action_queue(queue: Any) -> None:
    """Track live ActionQueue instances without extending their lifetime."""

    with _live_state_lock:
        _known_action_queues.add(queue)


def _environment_nonnegative_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None:
        return default
    value = int(raw)
    if value < 0:
        raise ValueError(f"{name} must be non-negative, got {value}")
    return value


def configure_policy_live_queue(enabled: bool) -> None:
    """Enable Live gating only for a commanding policy_live robot."""

    global _live_gate_active, _live_enabled, _live_queue_ready
    requested = bool(enabled)
    with _live_state_lock:
        _live_gate_active = requested
        if not requested:
            _live_enabled = False
            _live_queue_ready = False


def notify_policy_live_state(enabled: bool) -> int:
    """Synchronize the ROS Live gate with the async action consumer.

    A rising edge starts a new generation. The action queue is cleared
    immediately and must be refilled from the latest observation before model
    actions may be consumed.
    """

    global _live_enabled, _live_generation, _live_queue_ready
    requested = bool(enabled)
    queues_to_clear: list[Any] = []
    with _live_state_lock:
        if requested and not _live_enabled:
            _live_generation += 1
            _live_queue_ready = False
            queues_to_clear = list(_known_action_queues)
            logger.info(
                "ACT Live rising edge: fresh queue generation=%d requested",
                _live_generation,
            )
        elif not requested:
            _live_queue_ready = False
        _live_enabled = requested
        generation = _live_generation

    # Do not wait for the rollout thread's next ActionQueue.get(). Clearing on
    # the ROS Live callback immediately drops the pre-Live chunk, lowers qsize
    # below the RTC threshold, and wakes fresh inference from the latest obs.
    for queue in queues_to_clear:
        with queue.lock:
            queue.queue = None
            queue.original_queue = None
            queue.last_index = 0
            setattr(queue, _QUEUE_LIVE_GENERATION, generation)
    if queues_to_clear:
        logger.info(
            "ACT Live generation=%d immediately cleared %d registered queue(s)",
            generation,
            len(queues_to_clear),
        )
    return generation


def policy_live_hold_required() -> bool:
    """Return True while the MUX should receive an actual-TCP hold target."""

    with _live_state_lock:
        return _live_gate_active and (not _live_enabled or not _live_queue_ready)


def policy_live_queue_ready() -> bool:
    """Return True only when a fresh chunk exists for the current Live edge."""

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
                    "ACT fresh action queue ready: generation=%d",
                    generation,
                )
            _live_queue_ready = True


def _reset_policy_live_state_for_test() -> None:
    global _live_gate_active, _live_enabled, _live_generation, _live_queue_ready
    with _live_state_lock:
        _live_enabled = False
        _live_gate_active = False
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


def _blend_processed_action_overlap(
    old_actions: torch.Tensor,
    new_actions: torch.Tensor,
    overlap_steps: int,
) -> torch.Tensor:
    """Blend absolute A0509 actions without interpolating gripper state."""

    if old_actions.ndim != 2 or new_actions.ndim != 2:
        raise ValueError("action chunks must have shape [steps, action_dim]")
    if old_actions.shape[1] != new_actions.shape[1]:
        raise ValueError("old and new action dimensions must match")
    if old_actions.shape[1] < 7:
        raise ValueError("A0509 action chunks must contain at least 7 values")

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
        blended_quaternion = quaternion_slerp(
            doosan_zyz_deg_to_quaternion(old_zyz),
            doosan_zyz_deg_to_quaternion(new_zyz),
            fraction,
        )
        reference = old_zyz if previous_orientation is None else previous_orientation
        previous_orientation = quaternion_to_doosan_zyz_deg(
            blended_quaternion,
            reference,
        )
        blended[index, 3:6] = torch.as_tensor(
            previous_orientation,
            dtype=blended.dtype,
            device=blended.device,
        )

    # Preserve the old discrete gripper decision throughout the transition.
    blended[:, 6:] = old_prefix[:, 6:]
    return torch.cat((blended, new_actions[steps:].clone()), dim=0)


def _blend_original_action_overlap(
    old_actions: torch.Tensor,
    new_actions: torch.Tensor,
    overlap_steps: int,
) -> torch.Tensor:
    """Keep RTC bookkeeping length-aligned in policy/normalized space."""

    steps = min(overlap_steps, len(old_actions), len(new_actions))
    if steps <= 0:
        return new_actions.clone()
    old_prefix = old_actions[:steps].to(
        device=new_actions.device,
        dtype=new_actions.dtype,
    )
    weights = _smoothstep_weights(
        steps,
        dtype=new_actions.dtype,
        device=new_actions.device,
    )
    blended = (1.0 - weights) * old_prefix + weights * new_actions[:steps]
    if blended.shape[1] >= 7:
        blended[:, 6:] = old_prefix[:, 6:]
    return torch.cat((blended, new_actions[steps:].clone()), dim=0)


def merge_fifo_action_chunks(
    old_original: torch.Tensor,
    old_processed: torch.Tensor,
    new_original: torch.Tensor,
    new_processed: torch.Tensor,
    *,
    consumed_during_inference: int,
    overlap_steps: int,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Delay-align and smoothly replace an ACT FIFO tail with a new chunk."""

    delay = max(
        0,
        min(
            int(consumed_during_inference),
            len(new_original),
            len(new_processed),
        ),
    )
    trimmed_original = new_original[delay:].clone()
    trimmed_processed = new_processed[delay:].clone()
    if len(trimmed_processed) == 0:
        return old_original.clone(), old_processed.clone()
    return (
        _blend_original_action_overlap(
            old_original,
            trimmed_original,
            overlap_steps,
        ),
        _blend_processed_action_overlap(
            old_processed,
            trimmed_processed,
            overlap_steps,
        ),
    )


def install_act_async_overlap_blending(overlap_steps: int | None = None) -> bool:
    """Patch LeRobot's non-RTC ActionQueue with live-aware smooth merging."""

    from lerobot.policies.rtc import ActionQueue

    if getattr(ActionQueue.get, _QUEUE_PATCH_MARKER, False):
        return False
    if overlap_steps is None:
        overlap_steps = _environment_nonnegative_int(
            "LEROBOT_A0509_ACT_OVERLAP_STEPS",
            15,
        )

    original_get = ActionQueue.get
    original_merge = ActionQueue.merge
    original_get_action_index = ActionQueue.get_action_index

    @wraps(original_get)
    def live_aware_get(self: Any):
        _register_action_queue(self)
        live_gate_active, live_enabled, generation, _ready = _policy_live_snapshot()
        if not live_gate_active:
            return original_get(self)
        if not live_enabled:
            return None
        with self.lock:
            queue_generation = getattr(self, _QUEUE_LIVE_GENERATION, None)
            if queue_generation != generation:
                self.queue = None
                self.original_queue = None
                self.last_index = 0
                setattr(self, _QUEUE_LIVE_GENERATION, generation)
                logger.info(
                    "ACT queue cleared on Live generation=%d; waiting for fresh inference",
                    generation,
                )
                return None
            if self.queue is None or self.last_index >= len(self.queue):
                return None
            action = self.queue[self.last_index]
            self.last_index += 1
            return action.clone()

    @wraps(original_get_action_index)
    def generation_aware_get_action_index(self: Any) -> int:
        """Remember which Live generation was active when inference began."""

        _register_action_queue(self)
        _active, _live, generation, _ready = _policy_live_snapshot()
        with self.lock:
            action_index = self.last_index
            setattr(self, _QUEUE_INFERENCE_GENERATION, generation)
            return action_index

    @wraps(original_merge)
    def overlap_merge(
        self: Any,
        original_actions: torch.Tensor,
        processed_actions: torch.Tensor,
        real_delay: int,
        action_index_before_inference: int | None = None,
    ):
        _register_action_queue(self)
        live_gate_active, live_enabled, generation, _ready = _policy_live_snapshot()
        logger.debug(
            "ACT queue merge entered: rtc_enabled=%s live_gate_active=%s "
            "live_enabled=%s generation=%d",
            bool(self.cfg.enabled),
            live_gate_active,
            live_enabled,
            generation,
        )
        if self.cfg.enabled:
            return original_merge(
                self,
                original_actions,
                processed_actions,
                real_delay,
                action_index_before_inference,
            )

        mark_ready = False
        with self.lock:
            queue_generation = getattr(self, _QUEUE_LIVE_GENERATION, None)
            inference_generation = getattr(self, _QUEUE_INFERENCE_GENERATION, generation)
            if live_gate_active and live_enabled and inference_generation != generation:
                self.queue = None
                self.original_queue = None
                self.last_index = 0
                setattr(self, _QUEUE_LIVE_GENERATION, generation)
                logger.info(
                    "Discarded stale ACT inference at Live generation=%d",
                    generation,
                )
                return None

            if live_gate_active and live_enabled and queue_generation != generation:
                self.queue = None
                self.original_queue = None
                self.last_index = 0
                setattr(self, _QUEUE_LIVE_GENERATION, generation)
                logger.info("Accepted first fresh ACT chunk for generation=%d", generation)
            if self.queue is None or self.original_queue is None:
                self.original_queue = original_actions.clone()
                self.queue = processed_actions.clone()
                self.last_index = 0
                mark_ready = live_gate_active and live_enabled
            else:
                before = (
                    self.last_index
                    if action_index_before_inference is None
                    else int(action_index_before_inference)
                )
                consumed = max(0, self.last_index - before)
                old_original = self.original_queue[self.last_index :].clone()
                old_processed = self.queue[self.last_index :].clone()
                self.original_queue, self.queue = merge_fifo_action_chunks(
                    old_original,
                    old_processed,
                    original_actions,
                    processed_actions,
                    consumed_during_inference=consumed,
                    overlap_steps=overlap_steps,
                )
                self.last_index = 0
                mark_ready = live_gate_active and live_enabled
                logger.info(
                    "ACT chunk overlap merged: overlap=%d old_tail=%d "
                    "consumed_during_inference=%d estimated_delay=%d queue=%d",
                    min(
                        overlap_steps,
                        len(old_processed),
                        max(0, len(processed_actions) - consumed),
                    ),
                    len(old_processed),
                    consumed,
                    int(real_delay),
                    len(self.queue),
                )

        if mark_ready:
            _mark_policy_queue_ready(generation)
        return None

    setattr(live_aware_get, _QUEUE_PATCH_MARKER, True)
    setattr(overlap_merge, _QUEUE_PATCH_MARKER, True)
    setattr(generation_aware_get_action_index, _QUEUE_PATCH_MARKER, True)
    ActionQueue.get = live_aware_get
    ActionQueue.merge = overlap_merge
    ActionQueue.get_action_index = generation_aware_get_action_index
    logger.info("Installed ACT async overlap blending: steps=%d", overlap_steps)
    return True


def _synchronize_if_cuda(actions: Any) -> None:
    if isinstance(actions, torch.Tensor) and actions.is_cuda:
        torch.cuda.synchronize(actions.device)


def install_act_async_chunk_compat(warmup_inferences: int | None = None) -> bool:
    """Adapt ACT's chunk predictor for the async FIFO worker.

    Two forward passes are discarded by default on the worker's first call so
    CUDA kernels and allocator state are warm before the first executable
    chunk enters the queue. Returns ``True`` when installed and ``False`` when
    an earlier call already installed the adapter.
    """

    from lerobot.policies.act.modeling_act import ACTPolicy

    install_act_async_overlap_blending()

    original = ACTPolicy.predict_action_chunk
    if getattr(original, _PATCH_MARKER, False):
        return False

    if warmup_inferences is None:
        warmup_inferences = _environment_nonnegative_int(
            "LEROBOT_A0509_ACT_WARMUP_INFERENCES", 2
        )
    if warmup_inferences < 0:
        raise ValueError("warmup_inferences must be non-negative")

    @wraps(original)
    def predict_action_chunk_compat(
        self: Any,
        batch: dict[str, Any],
        *,
        inference_delay: int = 0,
        prev_chunk_left_over: Any | None = None,
    ):
        # ACT has no prefix-guidance implementation. These arguments are
        # accepted only for RTCInferenceEngine's FIFO mode
        # (``--inference.rtc.enabled=false``).
        del prev_chunk_left_over

        # A policy is first invoked synchronously during setup warmup and later
        # from its dedicated executor. Pin every calling thread, while keeping
        # logging state per native thread rather than per policy object.
        cpu_set = pin_current_thread_from_env(
            "LEROBOT_A0509_ACT_INFERENCE_CPU_SET"
        )
        if cpu_set is not None:
            native_id = get_native_id()
            with _act_inference_pin_log_lock:
                prior = getattr(self, _INFERENCE_PIN_MARKER, frozenset())
                pinned_threads = (
                    set(prior)
                    if isinstance(prior, (set, frozenset, tuple))
                    else set()
                )
                first_pin_for_thread = native_id not in pinned_threads
                pinned_threads.add(native_id)
                setattr(self, _INFERENCE_PIN_MARKER, frozenset(pinned_threads))
            if first_pin_for_thread:
                logger.info("ACT inference thread pinned to CPUs %s", cpu_set)

        with _act_gpu_arbiter:
            if not getattr(self, _WARMUP_MARKER, False):
                for index in range(warmup_inferences):
                    started = time.perf_counter()
                    warmup_actions = original(self, batch)
                    _synchronize_if_cuda(warmup_actions)
                    logger.info(
                        "ACT CUDA warmup %d/%d completed in %.3f ms",
                        index + 1,
                        warmup_inferences,
                        (time.perf_counter() - started) * 1000.0,
                    )
                setattr(self, _WARMUP_MARKER, True)

            started = time.perf_counter()
            actions = original(self, batch)
            _synchronize_if_cuda(actions)
            latency_ms = (time.perf_counter() - started) * 1000.0
        action_steps = int(actions.shape[1]) if actions.ndim >= 2 else 0
        logger.info(
            "ACT async chunk ready: steps=%d latency_ms=%.3f prior_delay_steps=%d",
            action_steps,
            latency_ms,
            int(inference_delay),
        )
        return actions

    setattr(predict_action_chunk_compat, _PATCH_MARKER, True)
    setattr(predict_action_chunk_compat, "_a0509_original", original)
    ACTPolicy.predict_action_chunk = predict_action_chunk_compat
    return True


def require_async_fifo_arguments(arguments: list[str]) -> None:
    """Reject accidental prefix-guided RTC or a non-100 execution setting."""

    if _option_value(arguments, "--inference.type") != "rtc":
        raise ValueError("ACT async rollout requires --inference.type=rtc")

    rtc_enabled = _option_value(arguments, "--inference.rtc.enabled")
    if rtc_enabled is None or rtc_enabled.strip().lower() not in {
        "false",
        "0",
        "no",
        "off",
    }:
        raise ValueError(
            "ACT supports asynchronous FIFO prefetch, not prefix-guided RTC; "
            "pass --inference.rtc.enabled=false"
        )

    n_action_steps = _option_value(arguments, "--policy.n_action_steps")
    if n_action_steps != "100":
        raise ValueError(
            "this accepted baseline requires --policy.n_action_steps=100"
        )


def _option_value(arguments: list[str], name: str) -> str | None:
    prefix = f"{name}="
    for index, argument in enumerate(arguments):
        if argument.startswith(prefix):
            return argument[len(prefix) :]
        if argument == name and index + 1 < len(arguments):
            return arguments[index + 1]
    return None
