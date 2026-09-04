"""Background JSONL/CSV trace writer and control timing summary for V2."""

from __future__ import annotations

import csv
import json
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


def json_default(value: Any) -> Any:
    """Serialize numerical trace details on the writer thread."""

    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(
        f"trace value is not JSON serializable: {type(value).__name__}"
    )


@dataclass(frozen=True)
class ControlTimingSummary:
    samples: int
    p50_ms: float
    p95_ms: float
    p99_ms: float
    max_ms: float
    deadline_miss_count: int


class ControlTimingMonitor:
    def __init__(self, *, control_hz: float) -> None:
        if control_hz <= 0.0:
            raise ValueError("control_hz must be positive")
        self.period_s = 1.0 / float(control_hz)
        self._durations_s: list[float] = []
        self._deadline_misses = 0

    def add(self, duration_s: float) -> None:
        value = float(duration_s)
        if not np.isfinite(value) or value < 0.0:
            raise ValueError("control tick duration must be finite and non-negative")
        self._durations_s.append(value)
        if value > self.period_s:
            self._deadline_misses += 1

    def summary(self) -> ControlTimingSummary:
        if not self._durations_s:
            return ControlTimingSummary(0, 0.0, 0.0, 0.0, 0.0, 0)
        values = np.asarray(self._durations_s, dtype=np.float64) * 1000.0
        return ControlTimingSummary(
            samples=len(values),
            p50_ms=float(np.percentile(values, 50)),
            p95_ms=float(np.percentile(values, 95)),
            p99_ms=float(np.percentile(values, 99)),
            max_ms=float(np.max(values)),
            deadline_miss_count=self._deadline_misses,
        )


_SENTINEL = object()


class NonBlockingHandoffTrace:
    """Serialize and write trace records only on a background thread."""

    CSV_FIELDS = (
        "timestamp_s",
        "record_type",
        "state",
        "teacher_stage",
        "handoff_id",
        "bridge_index",
        "bridge_progress",
        "handoff_window_progress",
        "crossfade_weight",
        "successor_generation",
        "control_tick_ms",
        "action_x_mm",
        "action_y_mm",
        "action_z_mm",
        "action_o1_deg",
        "action_o2_deg",
        "action_o3_deg",
        "action_gripper",
        "actual_x_mm",
        "actual_y_mm",
        "actual_z_mm",
        "commanded_x_mm",
        "commanded_y_mm",
        "commanded_z_mm",
        "event",
        "failure_reason",
    )

    def __init__(
        self,
        jsonl_path: str | Path,
        *,
        csv_path: str | Path | None = None,
    ) -> None:
        self.jsonl_path = Path(jsonl_path).expanduser().resolve()
        self.csv_path = (
            self.jsonl_path.with_suffix(".csv")
            if csv_path is None
            else Path(csv_path).expanduser().resolve()
        )
        self.jsonl_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        if self.jsonl_path.exists() or self.csv_path.exists():
            raise FileExistsError(
                "refusing to overwrite V2 trace: "
                f"{self.jsonl_path} or {self.csv_path}"
            )
        self._queue: queue.SimpleQueue[dict[str, Any] | object] = queue.SimpleQueue()
        self._closed = False
        self._error: BaseException | None = None
        self._thread = threading.Thread(
            target=self._writer_main,
            name="task-c-v2-trace-writer",
            daemon=True,
        )
        self._thread.start()

    def emit(self, record: dict[str, Any]) -> None:
        if self._error is not None:
            raise RuntimeError("V2 trace writer failed") from self._error
        if self._closed:
            raise RuntimeError("V2 trace is closed")
        # Keep control-thread work to one shallow copy and an unbounded queue
        # insertion. JSON serialization and all filesystem work stay in writer.
        self._queue.put(dict(record))

    def close(self, *, summary: dict[str, Any] | None = None) -> None:
        if self._closed:
            return
        if summary is not None:
            self.emit(
                {
                    "schema": "task_c_handoff_v2.trace",
                    "record_type": "episode_summary",
                    "timestamp_s": time.monotonic(),
                    **summary,
                }
            )
        self._closed = True
        self._queue.put(_SENTINEL)
        self._thread.join()
        if self._error is not None:
            raise RuntimeError("V2 trace writer failed") from self._error

    def _writer_main(self) -> None:
        try:
            self._write_records()
        except BaseException as exc:
            self._error = exc

    def _write_records(self) -> None:
        with self.jsonl_path.open("x", encoding="utf-8") as jsonl_file, self.csv_path.open(
            "x", encoding="utf-8", newline=""
        ) as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=self.CSV_FIELDS)
            writer.writeheader()
            while True:
                item = self._queue.get()
                if item is _SENTINEL:
                    break
                assert isinstance(item, dict)
                jsonl_file.write(json.dumps(item, sort_keys=True, default=json_default) + "\n")
                writer.writerow(self._csv_record(item))
            jsonl_file.flush()
            csv_file.flush()

    @classmethod
    def _csv_record(cls, record: dict[str, Any]) -> dict[str, Any]:
        result = {name: record.get(name) for name in cls.CSV_FIELDS}
        action = record.get("action")
        if isinstance(action, (list, tuple, np.ndarray)) and len(action) >= 7:
            for name, value in zip(
                (
                    "action_x_mm",
                    "action_y_mm",
                    "action_z_mm",
                    "action_o1_deg",
                    "action_o2_deg",
                    "action_o3_deg",
                    "action_gripper",
                ),
                action[:7],
                strict=True,
            ):
                result[name] = value
        actual = record.get("actual_pose_mm_deg")
        if isinstance(actual, (list, tuple, np.ndarray)) and len(actual) >= 3:
            result.update(
                {
                    "actual_x_mm": actual[0],
                    "actual_y_mm": actual[1],
                    "actual_z_mm": actual[2],
                }
            )
        commanded = record.get("commanded_pose_mm_deg")
        if isinstance(commanded, (list, tuple, np.ndarray)) and len(commanded) >= 3:
            result.update(
                {
                    "commanded_x_mm": commanded[0],
                    "commanded_y_mm": commanded[1],
                    "commanded_z_mm": commanded[2],
                }
            )
        return result
