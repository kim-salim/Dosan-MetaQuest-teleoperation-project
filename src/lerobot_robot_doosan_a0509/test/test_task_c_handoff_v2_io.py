from __future__ import annotations

import csv
import json
import time
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("lerobot")

from lerobot_robot_doosan_a0509.task_c_handoff.handoff_trace import (
    ControlTimingMonitor,
    NonBlockingHandoffTrace,
)
from lerobot_robot_doosan_a0509.task_c_handoff.recording import (
    EXPECTED_ACTION_NAMES,
    EXPECTED_IMAGE_SHAPES,
    EXPECTED_STATE_NAMES,
    AsyncTaskCLeRobotRecorder,
)


ACTION_NAMES = list(EXPECTED_ACTION_NAMES)
STATE_NAMES = list(EXPECTED_STATE_NAMES)


def _features() -> dict[str, dict]:
    return {
        "observation.state": {
            "dtype": "float32",
            "shape": (13,),
            "names": STATE_NAMES,
        },
        "observation.images.front": {
            "dtype": "video",
            "shape": EXPECTED_IMAGE_SHAPES["observation.images.front"],
            "names": None,
        },
        "observation.images.side": {
            "dtype": "video",
            "shape": EXPECTED_IMAGE_SHAPES["observation.images.side"],
            "names": None,
        },
        "observation.images.zed_rgb": {
            "dtype": "video",
            "shape": EXPECTED_IMAGE_SHAPES["observation.images.zed_rgb"],
            "names": None,
        },
        "action": {
            "dtype": "float32",
            "shape": (7,),
            "names": ACTION_NAMES,
        },
    }


def _observation() -> dict[str, object]:
    return {
        **{name: float(index) for index, name in enumerate(STATE_NAMES)},
        "front": np.full((2, 2, 3), 1, dtype=np.uint8),
        "side": np.full((2, 2, 3), 2, dtype=np.uint8),
        "zed_rgb": np.full((2, 2, 3), 3, dtype=np.uint8),
    }


class _FakeDataset:
    def __init__(self, *, fail_add: bool = False) -> None:
        self.fail_add = fail_add
        self.frames: list[dict] = []
        self.saved = False
        self.finalized = False
        self.cleared = False

    def add_frame(self, frame: dict) -> None:
        if self.fail_add:
            raise RuntimeError("fake add failure")
        self.frames.append(frame)

    def save_episode(self) -> None:
        self.saved = True

    def has_pending_frames(self) -> bool:
        return bool(self.frames)

    def clear_episode_buffer(self) -> None:
        self.cleared = True
        self.frames.clear()

    def finalize(self) -> None:
        self.finalized = True


def test_background_lerobot_recorder_owns_inputs_and_saves_one_episode():
    dataset = _FakeDataset()
    recorder = AsyncTaskCLeRobotRecorder(
        dataset,
        features=_features(),
        ordered_action_keys=ACTION_NAMES,
        task_description="A Bridge B",
        queue_size=4,
    )
    observation = _observation()
    recorder.enqueue(observation, np.arange(7, dtype=np.float64))
    observation["front"].fill(99)
    observation[STATE_NAMES[0]] = 99.0
    stats = recorder.close(save=True)

    assert stats.frames_submitted == 1
    assert stats.frames_written == 1
    assert stats.saved
    assert dataset.saved
    assert dataset.finalized
    assert dataset.frames[0]["task"] == "A Bridge B"
    assert dataset.frames[0]["observation.state"][0] == 0.0
    assert np.all(dataset.frames[0]["observation.images.front"] == 1)
    np.testing.assert_allclose(dataset.frames[0]["action"], np.arange(7))


def test_background_recorder_failure_is_reported_without_close_deadlock():
    recorder = AsyncTaskCLeRobotRecorder(
        _FakeDataset(fail_add=True),
        features=_features(),
        ordered_action_keys=ACTION_NAMES,
        task_description="A Bridge B",
        queue_size=2,
    )
    recorder.enqueue(_observation(), np.zeros(7))
    deadline = time.monotonic() + 1.0
    while recorder._error is None and time.monotonic() < deadline:
        time.sleep(0.002)
    started = time.monotonic()
    with pytest.raises(RuntimeError, match="background recorder failed"):
        recorder.close(save=False)
    assert time.monotonic() - started < 0.1


def test_nonblocking_trace_serializes_numpy_and_writes_machine_readable_csv(
    tmp_path: Path,
):
    trace_path = tmp_path / "trace.jsonl"
    csv_path = tmp_path / "trace.csv"
    trace = NonBlockingHandoffTrace(trace_path, csv_path=csv_path)
    trace.emit(
        {
            "record_type": "control_command",
            "timestamp_s": np.float64(1.25),
            "state": "RUN_BRIDGE",
            "teacher_stage": "BEZIER_BRIDGE_V2",
            "handoff_id": "h_test",
            "action": np.arange(7, dtype=np.float64),
            "actual_pose_mm_deg": np.arange(6, dtype=np.float64),
        }
    )
    trace.close(summary={"terminal_v2_state": "COMPLETE"})

    records = [json.loads(line) for line in trace_path.read_text().splitlines()]
    assert records[0]["action"] == list(range(7))
    assert records[-1]["record_type"] == "episode_summary"
    with csv_path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]["action_x_mm"] == "0.0"
    assert rows[0]["action_gripper"] == "6.0"

    with pytest.raises(FileExistsError):
        NonBlockingHandoffTrace(trace_path, csv_path=csv_path)


def test_control_timing_summary_reports_percentiles_and_deadline_misses():
    monitor = ControlTimingMonitor(control_hz=30.0)
    for duration_s in (0.010, 0.020, 0.040, 0.050):
        monitor.add(duration_s)
    summary = monitor.summary()
    assert summary.samples == 4
    assert summary.p50_ms == pytest.approx(30.0)
    assert summary.p95_ms > summary.p50_ms
    assert summary.max_ms == pytest.approx(50.0)
    assert summary.deadline_miss_count == 2
