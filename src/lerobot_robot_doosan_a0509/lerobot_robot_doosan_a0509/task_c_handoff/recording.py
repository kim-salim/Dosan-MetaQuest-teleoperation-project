"""Background LeRobot episode writer for autonomous Task-C demonstrations."""

from __future__ import annotations

import copy
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np


EXPECTED_STATE_NAMES = (
    "joint_1_rad",
    "joint_2_rad",
    "joint_3_rad",
    "joint_4_rad",
    "joint_5_rad",
    "joint_6_rad",
    "tcp_x_mm",
    "tcp_y_mm",
    "tcp_z_mm",
    "tcp_o1_deg",
    "tcp_o2_deg",
    "tcp_o3_deg",
    "gripper_commanded_state",
)
EXPECTED_ACTION_NAMES = (
    "target_x_mm",
    "target_y_mm",
    "target_z_mm",
    "target_o1_deg",
    "target_o2_deg",
    "target_o3_deg",
    "gripper_target",
)
EXPECTED_IMAGE_SHAPES = {
    "observation.images.front": (480, 640, 3),
    "observation.images.side": (480, 640, 3),
    "observation.images.zed_rgb": (376, 672, 3),
}


def _validate_dataset_contract(
    features: dict[str, dict],
    ordered_action_keys: tuple[str, ...],
) -> None:
    required = {
        "observation.state",
        "action",
        *EXPECTED_IMAGE_SHAPES,
    }
    missing = sorted(required.difference(features))
    if missing:
        raise ValueError(f"Task-C dataset features are missing: {missing}")
    state = features["observation.state"]
    action = features["action"]
    if tuple(state.get("shape", ())) != (13,) or tuple(
        state.get("names", ())
    ) != EXPECTED_STATE_NAMES:
        raise ValueError("Task-C observation.state must match the A0509 13D contract")
    if tuple(action.get("shape", ())) != (7,) or tuple(
        action.get("names", ())
    ) != EXPECTED_ACTION_NAMES:
        raise ValueError("Task-C action must match the A0509 7D target contract")
    if ordered_action_keys != EXPECTED_ACTION_NAMES:
        raise ValueError("Task-C ordered action keys differ from the A0509 contract")
    for key, expected_shape in EXPECTED_IMAGE_SHAPES.items():
        feature = features[key]
        if feature.get("dtype") not in {"image", "video"}:
            raise ValueError(f"{key} must be an image or video feature")
        if tuple(feature.get("shape", ())) != expected_shape:
            raise ValueError(f"{key} shape differs from the A0509 camera contract")


class RecordingBackpressureError(RuntimeError):
    """Fail closed instead of silently dropping a primary demonstration frame."""


def _owned_snapshot(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return np.array(value, copy=True)
    try:
        import torch

        if isinstance(value, torch.Tensor):
            return value.detach().clone()
    except ImportError:
        pass
    if isinstance(value, dict):
        return {key: _owned_snapshot(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_owned_snapshot(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_owned_snapshot(item) for item in value)
    try:
        return copy.deepcopy(value)
    except TypeError:
        return value


@dataclass(frozen=True)
class RecordingStats:
    frames_submitted: int
    frames_written: int
    maximum_queue_depth: int
    snapshot_latency_max_ms: float
    saved: bool


@dataclass(frozen=True)
class _Frame:
    observation: dict[str, Any]
    action: dict[str, float]
    task: str


@dataclass(frozen=True)
class _Finish:
    save: bool


class AsyncTaskCLeRobotRecorder:
    """Keep dataset add/save/video/file operations off the 30 Hz thread."""

    def __init__(
        self,
        dataset: Any,
        *,
        features: dict[str, dict],
        ordered_action_keys: list[str],
        task_description: str,
        queue_size: int = 96,
    ) -> None:
        if dataset is None:
            raise ValueError("Task-C recording requires a rollout dataset")
        if queue_size < 1:
            raise ValueError("recording queue_size must be positive")
        if not task_description:
            raise ValueError("Task-C task description must not be empty")
        ordered_keys = tuple(ordered_action_keys)
        _validate_dataset_contract(features, ordered_keys)
        self.dataset = dataset
        self.features = features
        self.ordered_action_keys = ordered_keys
        self.task_description = task_description
        self._queue: queue.Queue[_Frame | _Finish] = queue.Queue(maxsize=queue_size)
        self._error: BaseException | None = None
        self._closed = False
        self._frames_submitted = 0
        self._frames_written = 0
        self._maximum_queue_depth = 0
        self._snapshot_latency_max_ms = 0.0
        self._saved = False
        self._thread = threading.Thread(
            target=self._writer_main,
            name="task-c-v2-lerobot-writer",
            daemon=True,
        )
        self._thread.start()

    def enqueue(
        self,
        observation: dict[str, Any],
        action_values: np.ndarray,
    ) -> float:
        self.raise_if_failed()
        if self._closed:
            raise RuntimeError("Task-C recorder is closed")
        values = np.asarray(action_values, dtype=np.float64)
        if values.shape != (7,) or not np.all(np.isfinite(values)):
            raise ValueError("Task-C recorded action must be finite 7D")
        if len(self.ordered_action_keys) != 7:
            raise ValueError("Task-C dataset action contract must contain 7 names")
        started = time.perf_counter()
        owned_observation = _owned_snapshot(observation)
        latency_ms = (time.perf_counter() - started) * 1000.0
        frame = _Frame(
            observation=owned_observation,
            action={
                key: float(values[index])
                for index, key in enumerate(self.ordered_action_keys)
            },
            task=self.task_description,
        )
        try:
            self._queue.put_nowait(frame)
        except queue.Full as exc:
            raise RecordingBackpressureError(
                "Task-C recorder queue is full; no frame was dropped"
            ) from exc
        self._frames_submitted += 1
        self._maximum_queue_depth = max(
            self._maximum_queue_depth,
            self._queue.qsize(),
        )
        self._snapshot_latency_max_ms = max(
            self._snapshot_latency_max_ms,
            latency_ms,
        )
        return latency_ms

    def raise_if_failed(self) -> None:
        if self._error is not None:
            raise RuntimeError("Task-C background recorder failed") from self._error

    def close(self, *, save: bool) -> RecordingStats:
        if self._closed:
            return self.stats()
        self._closed = True
        if self._error is not None:
            self._thread.join()
            self.raise_if_failed()
        # Control has stopped before close(), so a bounded wait is no longer in
        # the 30 Hz command path. Every submitted frame is drained in order.
        self._queue.put(_Finish(bool(save)))
        self._thread.join()
        self.raise_if_failed()
        return self.stats()

    def stats(self) -> RecordingStats:
        return RecordingStats(
            frames_submitted=self._frames_submitted,
            frames_written=self._frames_written,
            maximum_queue_depth=self._maximum_queue_depth,
            snapshot_latency_max_ms=self._snapshot_latency_max_ms,
            saved=self._saved,
        )

    def _writer_main(self) -> None:
        try:
            from lerobot.utils.constants import ACTION, OBS_STR
            from lerobot.utils.feature_utils import build_dataset_frame

            while True:
                item = self._queue.get()
                if isinstance(item, _Finish):
                    if item.save and self._frames_written > 0:
                        self.dataset.save_episode()
                        self._saved = True
                    elif self.dataset.has_pending_frames():
                        self.dataset.clear_episode_buffer()
                    self.dataset.finalize()
                    return
                observation_frame = build_dataset_frame(
                    self.features,
                    item.observation,
                    prefix=OBS_STR,
                )
                action_frame = build_dataset_frame(
                    self.features,
                    item.action,
                    prefix=ACTION,
                )
                self.dataset.add_frame(
                    {
                        **observation_frame,
                        **action_frame,
                        "task": item.task,
                    }
                )
                self._frames_written += 1
        except BaseException as exc:
            self._error = exc
