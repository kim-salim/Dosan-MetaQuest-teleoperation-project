"""Append-only latency telemetry for command-free Task-C shadow trials."""

from __future__ import annotations

import json
import math
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from .runtime_policy import PolicyChunk


SCHEMA_VERSION = "task_c_shadow_latency_v1"


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _json_ready(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_ready(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def policy_chunk_timing_record(chunk: PolicyChunk) -> dict[str, Any]:
    """Render one generation's capture-to-completion timing without actions."""

    return {
        "policy_id": chunk.policy_id,
        "generation": chunk.generation,
        "action_chunk_shape": list(chunk.actions.shape),
        "observation_timestamp_s": chunk.observation_timestamp_s,
        "request_timestamp_s": chunk.request_timestamp_s,
        "snapshot_completed_timestamp_s": chunk.snapshot_completed_timestamp_s,
        "submitted_timestamp_s": chunk.submitted_timestamp_s,
        "worker_started_timestamp_s": chunk.worker_started_timestamp_s,
        "gpu_acquired_timestamp_s": chunk.gpu_acquired_timestamp_s,
        "inference_completed_timestamp_s": chunk.completed_timestamp_s,
        "request_snapshot_ms": chunk.request_snapshot_latency_s * 1000.0,
        "submission_queue_ms": chunk.submission_queue_latency_s * 1000.0,
        "gpu_wait_ms": chunk.gpu_wait_latency_s * 1000.0,
        "gpu_arbiter_wait_ms": chunk.gpu_wait_latency_s * 1000.0,
        "backend_inference_ms": chunk.backend_inference_latency_s * 1000.0,
        "worker_total_ms": chunk.inference_latency_s * 1000.0,
        "request_to_completion_ms": (
            chunk.request_to_completion_latency_s * 1000.0
        ),
        "capture_to_gpu_acquired_ms": (
            chunk.gpu_acquired_timestamp_s - chunk.observation_timestamp_s
        )
        * 1000.0,
        "capture_to_inference_completed_ms": (
            chunk.completed_timestamp_s - chunk.observation_timestamp_s
        )
        * 1000.0,
    }


def _distribution(values: Iterable[float]) -> dict[str, Any]:
    samples = np.asarray(list(values), dtype=np.float64)
    samples = samples[np.isfinite(samples)]
    if len(samples) == 0:
        return {"count": 0}
    return {
        "count": int(len(samples)),
        "mean": float(np.mean(samples)),
        "minimum": float(np.min(samples)),
        "p50": float(np.percentile(samples, 50)),
        "p90": float(np.percentile(samples, 90)),
        "p95": float(np.percentile(samples, 95)),
        "p99": float(np.percentile(samples, 99)),
        "maximum": float(np.max(samples)),
        "p99_sample_count_recommended": 100,
        "p99_sample_count_sufficient": bool(len(samples) >= 100),
    }


def summarize_shadow_trials(records: Iterable[dict[str, Any]]) -> dict[str, Any]:
    trials = [record for record in records if record.get("record_type") == "transition_trial"]
    statuses = Counter(str(record.get("status", "unknown")) for record in trials)
    latency_fields = (
        "capture_duration_ms",
        "policy_input_build_ms",
        "act_a_gpu_wait_ms",
        "act_a_backend_inference_ms",
        "act_b_gpu_wait_ms",
        "act_b_backend_inference_ms",
        "act_b_capture_to_gpu_acquired_ms",
        "act_b_inference_to_tail_planning_ms",
        "tail_planning_ms",
        "tail_planning_to_ready_ms",
        "capture_to_ready_ms",
        "observation_age_at_ready_ms",
        "deadline_slack_ms",
    )
    distributions = {
        name: _distribution(
            float(record[name])
            for record in trials
            if record.get(name) is not None
        )
        for name in latency_fields
    }
    deadline_records = [
        record for record in trials if record.get("deadline_missed") is not None
    ]
    stale_records = [record for record in trials if record.get("stale") is not None]
    return {
        "schema_version": SCHEMA_VERSION,
        "record_type": "session_summary",
        "created_utc": utc_now_iso(),
        "trial_count": len(trials),
        "status_counts": dict(sorted(statuses.items())),
        "ready_count": statuses.get("ready", 0),
        "failed_count": sum(
            count for status, count in statuses.items() if status != "ready"
        ),
        "deadline_evaluable_count": len(deadline_records),
        "deadline_miss_count": sum(
            bool(record["deadline_missed"]) for record in deadline_records
        ),
        "deadline_miss_rate": (
            None
            if not deadline_records
            else sum(bool(record["deadline_missed"]) for record in deadline_records)
            / len(deadline_records)
        ),
        "stale_evaluable_count": len(stale_records),
        "stale_count": sum(bool(record["stale"]) for record in stale_records),
        "stale_rate": (
            None
            if not stale_records
            else sum(bool(record["stale"]) for record in stale_records)
            / len(stale_records)
        ),
        "latency_ms": distributions,
        "safety": {
            "robot_commands_published": False,
            "robot_executable": False,
            "dry_run_only": True,
            "orientation_status": "pending",
        },
    }


class ShadowJsonlRecorder:
    """Write each terminal trial immediately and create a summary on close."""

    def __init__(self, jsonl_path: str | Path, summary_path: str | Path) -> None:
        self.jsonl_path = Path(jsonl_path).expanduser().resolve()
        self.summary_path = Path(summary_path).expanduser().resolve()
        self.jsonl_path.parent.mkdir(parents=True, exist_ok=True)
        self.summary_path.parent.mkdir(parents=True, exist_ok=True)
        # Exclusive creation avoids silently mixing runs or overwriting a
        # previous safety/latency record.
        self._stream = self.jsonl_path.open("x", encoding="utf-8")
        self._trials: list[dict[str, Any]] = []
        self._closed = False

    def append(self, record: dict[str, Any]) -> None:
        if self._closed:
            raise RuntimeError("shadow recorder is closed")
        rendered = {
            "schema_version": SCHEMA_VERSION,
            "written_utc": utc_now_iso(),
            **_json_ready(record),
        }
        self._stream.write(json.dumps(rendered, sort_keys=True) + "\n")
        self._stream.flush()
        if rendered.get("record_type") == "transition_trial":
            self._trials.append(rendered)

    def close(self, *, extra_summary: dict[str, Any] | None = None) -> dict[str, Any]:
        if self._closed:
            return summarize_shadow_trials(self._trials)
        summary = summarize_shadow_trials(self._trials)
        if extra_summary:
            summary.update(_json_ready(extra_summary))
        self.append(summary)
        self._stream.close()
        self._closed = True
        with self.summary_path.open("x", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)
            stream.write("\n")
        return summary
