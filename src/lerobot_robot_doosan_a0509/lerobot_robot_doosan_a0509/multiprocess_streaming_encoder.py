"""Shared-memory process isolation for LeRobot streaming video encoding.

The stock LeRobot 0.6 encoder runs camera encoders and image statistics in
threads in the recording process.  This adapter keeps the public encoder API,
but moves that threaded encoder into one spawned worker process.  Frames cross
the process boundary through owned shared-memory ring slots; only compact slot
descriptors are sent through multiprocessing queues.
"""

from __future__ import annotations

import contextlib
import logging
import multiprocessing
import os
import queue
import time
import traceback
from collections import deque
from dataclasses import dataclass
from multiprocessing.shared_memory import SharedMemory
from pathlib import Path
from typing import Any

import numpy as np
from lerobot.configs import DepthEncoderConfig, RGBEncoderConfig
from lerobot.datasets.video_utils import StreamingVideoEncoder as ThreadStreamingVideoEncoder


logger = logging.getLogger(__name__)


def _parse_cpu_set(value: str) -> set[int]:
    cpus: set[int] = set()
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        if "-" in item:
            start_text, end_text = item.split("-", 1)
            start = int(start_text)
            end = int(end_text)
            if end < start:
                raise ValueError(f"invalid descending CPU range: {item}")
            cpus.update(range(start, end + 1))
        else:
            cpus.add(int(item))
    if not cpus:
        raise ValueError("encoder CPU set must contain at least one CPU")
    return cpus


def _close_attached_shared_memory(attached: dict[str, SharedMemory]) -> None:
    for shm in attached.values():
        with contextlib.suppress(Exception):
            shm.close()
    attached.clear()


def _worker_main(
    command_queue: Any,
    response_queue: Any,
    fps: int,
    rgb_encoder: RGBEncoderConfig | None,
    depth_encoder: DepthEncoderConfig | None,
    queue_maxsize: int,
    encoder_threads: int | None,
    cpu_set: str | None,
) -> None:
    """Run the stock threaded encoder behind a shared-memory command loop."""

    inner: ThreadStreamingVideoEncoder | None = None
    attached: dict[str, SharedMemory] = {}
    layouts: dict[str, tuple[tuple[int, ...], np.dtype[Any], int, int]] = {}
    active = False
    processed: dict[str, int] = {}

    try:
        if cpu_set:
            os.sched_setaffinity(0, _parse_cpu_set(cpu_set))

        inner = ThreadStreamingVideoEncoder(
            fps=fps,
            rgb_encoder=rgb_encoder,
            depth_encoder=depth_encoder,
            queue_maxsize=queue_maxsize,
            encoder_threads=encoder_threads,
        )
        response_queue.put(
            (
                "ready",
                {
                    "pid": os.getpid(),
                    "cpu_set": sorted(os.sched_getaffinity(0)),
                    "nice": os.getpriority(os.PRIO_PROCESS, 0),
                    "start_method": multiprocessing.get_start_method(),
                },
            )
        )

        while True:
            command = command_queue.get()
            operation = command[0]

            if operation == "start":
                if active:
                    inner.cancel_episode()
                _close_attached_shared_memory(attached)
                layouts.clear()
                _, video_keys, temp_dir, depth_video_keys = command
                inner.start_episode(
                    video_keys=list(video_keys),
                    temp_dir=Path(temp_dir),
                    depth_video_keys=list(depth_video_keys),
                )
                processed = {key: 0 for key in video_keys}
                active = True
                response_queue.put(("started",))
                continue

            if operation == "register":
                if not active:
                    raise RuntimeError("shared-memory registration without an active episode")
                _, video_key, shm_name, shape, dtype_text, slots = command
                if video_key in attached:
                    raise RuntimeError(f"duplicate shared-memory registration: {video_key}")
                dtype = np.dtype(dtype_text)
                shape_tuple = tuple(int(value) for value in shape)
                frame_nbytes = int(np.prod(shape_tuple, dtype=np.int64)) * dtype.itemsize
                attached[video_key] = SharedMemory(name=shm_name, create=False)
                layouts[video_key] = (shape_tuple, dtype, int(slots), frame_nbytes)
                continue

            if operation == "frame":
                if not active:
                    raise RuntimeError("frame received without an active episode")
                _, video_key, slot, sequence_id = command
                if video_key not in attached:
                    raise RuntimeError(f"frame received before registration: {video_key}")
                shape, dtype, slots, frame_nbytes = layouts[video_key]
                if not 0 <= int(slot) < slots:
                    raise RuntimeError(f"invalid shared-memory slot {slot} for {video_key}")
                frame = np.ndarray(
                    shape,
                    dtype=dtype,
                    buffer=attached[video_key].buf,
                    offset=int(slot) * frame_nbytes,
                )
                inner.feed_frame(video_key, frame)
                drops = dict(getattr(inner, "_dropped_frames", {}))
                if drops.get(video_key, 0):
                    raise RuntimeError(
                        f"inner encoder dropped {drops[video_key]} frame(s) for {video_key}"
                    )
                processed[video_key] += 1
                response_queue.put(("ack", video_key, int(slot), int(sequence_id)))
                continue

            if operation == "finish":
                if not active:
                    raise RuntimeError("finish requested without an active episode")
                inner_drops = dict(getattr(inner, "_dropped_frames", {}))
                results = inner.finish_episode()
                active = False
                _close_attached_shared_memory(attached)
                layouts.clear()
                response_queue.put(("finished", results, processed, inner_drops))
                continue

            if operation == "cancel":
                if active:
                    inner.cancel_episode()
                    active = False
                _close_attached_shared_memory(attached)
                layouts.clear()
                response_queue.put(("cancelled",))
                continue

            if operation == "close":
                if active:
                    inner.cancel_episode()
                    active = False
                _close_attached_shared_memory(attached)
                layouts.clear()
                inner.close()
                response_queue.put(("closed",))
                return

            raise RuntimeError(f"unknown encoder worker operation: {operation}")

    except BaseException as exc:
        with contextlib.suppress(Exception):
            response_queue.put(
                ("error", f"{type(exc).__name__}: {exc}", traceback.format_exc()),
                timeout=1.0,
            )
    finally:
        if inner is not None:
            with contextlib.suppress(Exception):
                if active:
                    inner.cancel_episode()
                inner.close()
        _close_attached_shared_memory(attached)


@dataclass
class _SharedRing:
    memory: SharedMemory
    shape: tuple[int, ...]
    dtype: np.dtype[Any]
    slots: int
    frame_nbytes: int
    free_slots: deque[int]
    in_flight: dict[int, int]
    peak_in_flight: int = 0

    def frame_view(self, slot: int) -> np.ndarray[Any, Any]:
        return np.ndarray(
            self.shape,
            dtype=self.dtype,
            buffer=self.memory.buf,
            offset=slot * self.frame_nbytes,
        )


class SharedMemoryStreamingVideoEncoder:
    """LeRobot-compatible encoder isolated in one spawned worker process."""

    def __init__(
        self,
        fps: int,
        rgb_encoder: RGBEncoderConfig | None = None,
        depth_encoder: DepthEncoderConfig | None = None,
        queue_maxsize: int = 30,
        encoder_threads: int | None = None,
        *,
        shared_memory_slots: int | None = None,
        startup_timeout_sec: float = 20.0,
        finish_timeout_sec: float = 140.0,
    ) -> None:
        if queue_maxsize <= 0:
            raise ValueError("queue_maxsize must be positive")
        configured_slots = os.environ.get("LEROBOT_ENCODER_SHM_SLOTS")
        if shared_memory_slots is None:
            shared_memory_slots = (
                int(configured_slots)
                if configured_slots is not None
                else max(32, queue_maxsize + 2)
            )
        if shared_memory_slots <= 0:
            raise ValueError("shared_memory_slots must be positive")

        self.fps = int(fps)
        self.queue_maxsize = int(queue_maxsize)
        self.shared_memory_slots = int(shared_memory_slots)
        self.startup_timeout_sec = float(startup_timeout_sec)
        self.finish_timeout_sec = float(finish_timeout_sec)
        self._closed = False
        self._episode_active = False
        self._worker_error: tuple[str, str] | None = None
        self._control_responses: deque[tuple[Any, ...]] = deque()
        self._rings: dict[str, _SharedRing] = {}
        self._video_keys: set[str] = set()
        self._submitted: dict[str, int] = {}
        self._next_sequence: dict[str, int] = {}
        self.last_episode_metrics: dict[str, Any] | None = None

        context = multiprocessing.get_context("spawn")
        queue_capacity = max(256, self.shared_memory_slots * 16)
        self._command_queue = context.Queue(maxsize=queue_capacity)
        self._response_queue = context.Queue(maxsize=queue_capacity)
        encoder_cpu_set = os.environ.get("LEROBOT_ENCODER_CPUSET")
        self._worker = context.Process(
            target=_worker_main,
            args=(
                self._command_queue,
                self._response_queue,
                self.fps,
                rgb_encoder,
                depth_encoder,
                self.queue_maxsize,
                encoder_threads,
                encoder_cpu_set,
            ),
            name="lerobot-a0509-encoder",
            daemon=True,
        )
        self._worker.start()
        ready = self._wait_for("ready", self.startup_timeout_sec)
        self.worker_info = dict(ready[1])
        logger.info(
            "Shared-memory encoder worker ready: pid=%s cpu_set=%s nice=%s slots_per_camera=%s",
            self.worker_info["pid"],
            self.worker_info["cpu_set"],
            self.worker_info["nice"],
            self.shared_memory_slots,
        )

    def start_episode(
        self,
        video_keys: list[str],
        temp_dir: Path,
        depth_video_keys: list[str] | None = None,
    ) -> None:
        self._require_open_worker()
        if self._episode_active:
            self.cancel_episode()
        self._cleanup_rings()
        self._video_keys = set(video_keys)
        self._submitted = {key: 0 for key in video_keys}
        self._next_sequence = {key: 0 for key in video_keys}
        self.last_episode_metrics = None
        self._send(
            (
                "start",
                tuple(video_keys),
                str(temp_dir),
                tuple(depth_video_keys or ()),
            )
        )
        self._wait_for("started", self.startup_timeout_sec)
        self._episode_active = True

    def feed_frame(self, video_key: str, image: np.ndarray) -> None:
        if not self._episode_active:
            raise RuntimeError("No active episode. Call start_episode() first.")
        if video_key not in self._video_keys:
            raise KeyError(f"unknown episode video key: {video_key}")
        self._drain_responses()
        self._raise_worker_error()

        array = np.asarray(image)
        ring = self._rings.get(video_key)
        if ring is None:
            ring = self._create_ring(video_key, array)
        elif array.shape != ring.shape or array.dtype != ring.dtype:
            raise RuntimeError(
                f"frame layout changed for {video_key}: "
                f"expected shape={ring.shape} dtype={ring.dtype}, "
                f"got shape={array.shape} dtype={array.dtype}"
            )

        if not ring.free_slots:
            self._drain_responses()
        if not ring.free_slots:
            raise RuntimeError(
                f"shared-memory encoder ring full for {video_key}; "
                "recording aborted instead of silently dropping a frame"
            )

        slot = ring.free_slots.popleft()
        sequence_id = self._next_sequence[video_key]
        np.copyto(ring.frame_view(slot), array, casting="no")
        ring.in_flight[slot] = sequence_id
        ring.peak_in_flight = max(ring.peak_in_flight, len(ring.in_flight))
        try:
            self._command_queue.put_nowait(("frame", video_key, slot, sequence_id))
        except queue.Full as exc:
            ring.in_flight.pop(slot, None)
            ring.free_slots.appendleft(slot)
            raise RuntimeError(
                "encoder command queue is full; recording aborted instead of dropping a frame"
            ) from exc
        self._next_sequence[video_key] += 1
        self._submitted[video_key] += 1

    def finish_episode(self) -> dict[str, tuple[Path, dict | None]]:
        if not self._episode_active:
            raise RuntimeError("No active episode to finish.")
        try:
            self._drain_responses()
            self._raise_worker_error()
            self._send(("finish",))
            message = self._wait_for("finished", self.finish_timeout_sec)
            _, results, processed, inner_drops = message
            self._drain_responses()
            self._raise_worker_error()

            in_flight = {
                key: len(ring.in_flight) for key, ring in self._rings.items() if ring.in_flight
            }
            if in_flight:
                raise RuntimeError(f"encoder finished with unacknowledged slots: {in_flight}")
            if dict(processed) != self._submitted:
                raise RuntimeError(
                    f"encoder frame count mismatch: submitted={self._submitted} processed={processed}"
                )
            drops = {key: int(value) for key, value in dict(inner_drops).items() if value}
            if drops:
                raise RuntimeError(f"inner encoder dropped frames: {drops}")

            self.last_episode_metrics = {
                "submitted": dict(self._submitted),
                "processed": dict(processed),
                "peak_in_flight": {
                    key: ring.peak_in_flight for key, ring in self._rings.items()
                },
                "drops": drops,
                "worker": dict(self.worker_info),
            }
            logger.info("Shared-memory encoder episode metrics: %s", self.last_episode_metrics)
            return dict(results)
        except Exception:
            if self._worker.is_alive():
                self._terminate_worker()
            raise
        finally:
            self._episode_active = False
            self._cleanup_rings()

    def cancel_episode(self) -> None:
        if not self._episode_active:
            return
        try:
            if self._worker.is_alive() and self._worker_error is None:
                self._send(("cancel",))
                self._wait_for("cancelled", 10.0)
        except Exception:
            self._terminate_worker()
        finally:
            self._episode_active = False
            self._cleanup_rings()

    def close(self) -> None:
        if self._closed:
            return
        try:
            if self._episode_active:
                self.cancel_episode()
            if self._worker.is_alive() and self._worker_error is None:
                self._send(("close",))
                with contextlib.suppress(Exception):
                    self._wait_for("closed", 10.0)
            self._worker.join(timeout=5.0)
            if self._worker.is_alive():
                self._terminate_worker()
        finally:
            self._cleanup_rings()
            self._close_queues()
            self._closed = True

    def _create_ring(self, video_key: str, image: np.ndarray) -> _SharedRing:
        shape = tuple(int(value) for value in image.shape)
        dtype = image.dtype
        frame_nbytes = int(image.nbytes)
        if frame_nbytes <= 0:
            raise ValueError(f"empty frame for {video_key}")
        memory = SharedMemory(create=True, size=frame_nbytes * self.shared_memory_slots)
        ring = _SharedRing(
            memory=memory,
            shape=shape,
            dtype=dtype,
            slots=self.shared_memory_slots,
            frame_nbytes=frame_nbytes,
            free_slots=deque(range(self.shared_memory_slots)),
            in_flight={},
        )
        self._rings[video_key] = ring
        try:
            self._send(
                (
                    "register",
                    video_key,
                    memory.name,
                    shape,
                    dtype.str,
                    self.shared_memory_slots,
                )
            )
        except Exception:
            self._rings.pop(video_key, None)
            with contextlib.suppress(Exception):
                memory.close()
            with contextlib.suppress(FileNotFoundError):
                memory.unlink()
            raise
        return ring

    def _drain_responses(self) -> None:
        while True:
            try:
                message = self._response_queue.get_nowait()
            except queue.Empty:
                return
            kind = message[0]
            if kind == "ack":
                _, video_key, slot, sequence_id = message
                ring = self._rings.get(video_key)
                if ring is None:
                    self._worker_error = (
                        f"ack for unknown ring {video_key}",
                        repr(message),
                    )
                    continue
                expected = ring.in_flight.pop(int(slot), None)
                if expected != int(sequence_id):
                    self._worker_error = (
                        f"slot acknowledgement mismatch for {video_key}",
                        f"slot={slot} expected={expected} received={sequence_id}",
                    )
                    continue
                ring.free_slots.append(int(slot))
            elif kind == "error":
                self._worker_error = (str(message[1]), str(message[2]))
            else:
                self._control_responses.append(message)

    def _wait_for(self, expected_kind: str, timeout_sec: float) -> tuple[Any, ...]:
        deadline = time.monotonic() + timeout_sec
        while True:
            self._drain_responses()
            self._raise_worker_error()
            for index, message in enumerate(self._control_responses):
                if message[0] == expected_kind:
                    del self._control_responses[index]
                    return message
            if not self._worker.is_alive():
                raise RuntimeError(
                    f"encoder worker exited with code {self._worker.exitcode} "
                    f"while waiting for {expected_kind}"
                )
            if time.monotonic() >= deadline:
                raise TimeoutError(f"timed out waiting for encoder worker {expected_kind}")
            time.sleep(0.005)

    def _send(self, command: tuple[Any, ...]) -> None:
        self._require_open_worker()
        try:
            self._command_queue.put(command, timeout=1.0)
        except queue.Full as exc:
            raise RuntimeError(f"encoder command queue full for {command[0]}") from exc

    def _require_open_worker(self) -> None:
        if self._closed:
            raise RuntimeError("encoder is closed")
        self._drain_responses()
        self._raise_worker_error()
        if not self._worker.is_alive():
            raise RuntimeError(f"encoder worker is not alive (exitcode={self._worker.exitcode})")

    def _raise_worker_error(self) -> None:
        if self._worker_error is None:
            return
        message, remote_traceback = self._worker_error
        raise RuntimeError(f"encoder worker failed: {message}\n{remote_traceback}")

    def _cleanup_rings(self) -> None:
        for ring in self._rings.values():
            with contextlib.suppress(Exception):
                ring.memory.close()
            with contextlib.suppress(FileNotFoundError):
                ring.memory.unlink()
        self._rings.clear()

    def _terminate_worker(self) -> None:
        if self._worker.is_alive():
            logger.error("Terminating unresponsive encoder worker pid=%s", self._worker.pid)
            self._worker.terminate()
            self._worker.join(timeout=5.0)

    def _close_queues(self) -> None:
        for managed_queue in (self._command_queue, self._response_queue):
            with contextlib.suppress(Exception):
                managed_queue.close()
            with contextlib.suppress(Exception):
                managed_queue.join_thread()

    def __del__(self) -> None:
        with contextlib.suppress(Exception):
            self.close()


def install_shared_memory_streaming_encoder() -> type[Any]:
    """Patch LeRobot's dataset factory for this recording process only."""

    from lerobot.datasets import lerobot_dataset

    previous = lerobot_dataset.StreamingVideoEncoder
    if previous is SharedMemoryStreamingVideoEncoder:
        return previous
    lerobot_dataset.StreamingVideoEncoder = SharedMemoryStreamingVideoEncoder
    logger.info("Installed A0509 shared-memory streaming encoder for this process")
    return previous
