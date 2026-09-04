"""Tests for Task-C shadow timing and append-only telemetry."""

from __future__ import annotations

import json
import tempfile
import threading
import time
import unittest
from pathlib import Path

import numpy as np

from .runtime_policy import AsyncPolicySession
from .shadow_latency import (
    ShadowJsonlRecorder,
    policy_chunk_timing_record,
    summarize_shadow_trials,
)


class _Backend:
    def reset(self) -> None:
        return None

    def infer(self, _observation: object) -> np.ndarray:
        actions = np.zeros((100, 7), dtype=np.float64)
        actions[:, 0] = np.arange(100, dtype=np.float64)
        return actions


class ShadowPolicyTimingTest(unittest.TestCase):
    def test_gpu_arbiter_wait_is_separate_from_backend_inference(self) -> None:
        arbiter = threading.Lock()
        arbiter.acquire()
        session = AsyncPolicySession(
            "ACT-B", _Backend(), action_hz=30.0, inference_lock=arbiter
        )
        try:
            capture = time.monotonic()
            generation = session.prime({}, observation_timestamp_s=capture)
            time.sleep(0.03)
            arbiter.release()
            chunk = session.wait_for_chunk(generation, 1.0)
            self.assertGreaterEqual(chunk.gpu_wait_latency_s, 0.02)
            self.assertGreaterEqual(chunk.backend_inference_latency_s, 0.0)
            self.assertAlmostEqual(
                chunk.inference_latency_s,
                chunk.gpu_wait_latency_s + chunk.backend_inference_latency_s,
                delta=0.005,
            )
            timing = policy_chunk_timing_record(chunk)
            self.assertGreaterEqual(timing["gpu_wait_ms"], 20.0)
            self.assertGreaterEqual(
                timing["capture_to_inference_completed_ms"],
                timing["gpu_wait_ms"],
            )
        finally:
            if arbiter.locked():
                arbiter.release()
            session.close()

    def test_snapshot_timeline_is_monotonic(self) -> None:
        session = AsyncPolicySession("ACT-A", _Backend(), action_hz=30.0)
        try:
            capture = time.monotonic()
            generation = session.prime(
                {"owned": np.arange(10)}, observation_timestamp_s=capture
            )
            chunk = session.wait_for_chunk(generation, 1.0)
            timeline = [
                chunk.observation_timestamp_s,
                chunk.request_timestamp_s,
                chunk.snapshot_completed_timestamp_s,
                chunk.submitted_timestamp_s,
                chunk.worker_started_timestamp_s,
                chunk.gpu_acquired_timestamp_s,
                chunk.completed_timestamp_s,
            ]
            self.assertEqual(timeline, sorted(timeline))
        finally:
            session.close()


class ShadowTelemetryTest(unittest.TestCase):
    @staticmethod
    def _trial(index: int) -> dict[str, object]:
        return {
            "record_type": "transition_trial",
            "status": "ready" if index % 4 else "failed_hold",
            "capture_duration_ms": float(index),
            "policy_input_build_ms": float(index + 1),
            "act_a_gpu_wait_ms": 0.0,
            "act_a_backend_inference_ms": 10.0,
            "act_b_gpu_wait_ms": float(index),
            "act_b_backend_inference_ms": 20.0,
            "tail_planning_ms": 2.0,
            "capture_to_ready_ms": float(100 + index),
            "observation_age_at_ready_ms": float(100 + index),
            "deadline_slack_ms": -1.0 if index % 5 == 0 else 10.0,
            "deadline_missed": index % 5 == 0,
            "stale": index % 10 == 0,
        }

    def test_summary_reports_p99_stale_and_deadline_rates(self) -> None:
        summary = summarize_shadow_trials(self._trial(index) for index in range(100))
        self.assertEqual(summary["trial_count"], 100)
        self.assertEqual(summary["deadline_miss_count"], 20)
        self.assertAlmostEqual(summary["deadline_miss_rate"], 0.2)
        self.assertEqual(summary["stale_count"], 10)
        self.assertAlmostEqual(summary["stale_rate"], 0.1)
        distribution = summary["latency_ms"]["act_b_gpu_wait_ms"]
        self.assertAlmostEqual(distribution["p99"], 98.01)
        self.assertTrue(distribution["p99_sample_count_sufficient"])

    def test_jsonl_is_append_only_and_summary_is_written(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            jsonl = root / "latency.jsonl"
            summary_path = root / "summary.json"
            recorder = ShadowJsonlRecorder(jsonl, summary_path)
            recorder.append({"record_type": "session_start"})
            recorder.append(self._trial(1))
            summary = recorder.close(extra_summary={"capture_failure_count": 0})
            lines = [json.loads(line) for line in jsonl.read_text().splitlines()]
            self.assertEqual([line["record_type"] for line in lines], [
                "session_start",
                "transition_trial",
                "session_summary",
            ])
            self.assertEqual(summary["trial_count"], 1)
            self.assertEqual(json.loads(summary_path.read_text())["trial_count"], 1)
            with self.assertRaises(FileExistsError):
                ShadowJsonlRecorder(jsonl, root / "other.json")


if __name__ == "__main__":
    unittest.main()
