from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from offline_tools.task_c_bridge_v0.runtime_policy import (
    AsyncPolicySession,
    PolicyChunk,
)
from lerobot_robot_doosan_a0509.task_c_handoff.async_successor import (
    AsyncSuccessorController,
)


class _ImmediateBackend:
    def reset(self) -> None:
        pass

    def infer(self, _observation):
        return np.tile(
            np.array([100.0, 0.0, 400.0, 0.0, 150.0, 0.0, 1.0]),
            (100, 1),
        )


def test_async_successor_rejects_observation_stale_result():
    session = AsyncPolicySession("B", _ImmediateBackend(), action_hz=30.0)
    controller = AsyncSuccessorController(session, max_result_age_s=0.05)
    try:
        now = time.monotonic()
        generation = controller.request(
            {"state": np.zeros(13)},
            observation_timestamp_s=now - 1.0,
        )
        assert generation is not None
        result = None
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            candidate = controller.poll(now_s=time.monotonic())
            if candidate.status != "pending":
                result = candidate
                break
            time.sleep(0.001)
        assert result is not None
        assert result.status == "rejected"
        assert result.stale
        assert result.failure_reason.startswith("stale_result_age_s=")
        assert session.active_generation is None
        assert session.queue_size == 0
    finally:
        session.close()


class _MismatchedSession:
    def __init__(self) -> None:
        self.backend = object()
        self.cleared = 0
        self.requested = False

    def prime(self, _value, *, observation_timestamp_s, preserve_active):
        assert preserve_active is False
        self.requested = True
        self.observation_timestamp_s = observation_timestamp_s
        return 7

    def poll(self):
        if not self.requested:
            return None
        self.requested = False
        now = time.monotonic()
        return PolicyChunk(
            policy_id="B",
            generation=8,
            observation_timestamp_s=self.observation_timestamp_s,
            completed_timestamp_s=now,
            inference_latency_s=0.0,
            action_hz=30.0,
            actions=np.zeros((100, 7)),
        )

    def deactivate_and_clear(self):
        self.cleared += 1
        return self.cleared


def test_async_successor_generation_mismatch_is_invalidated():
    session = _MismatchedSession()
    controller = AsyncSuccessorController(session, max_result_age_s=1.0)
    now = time.monotonic()
    assert controller.request({}, observation_timestamp_s=now) == 7
    result = controller.poll(now_s=time.monotonic())
    assert result.status == "rejected"
    assert result.stale
    assert result.failure_reason == "generation_mismatch"
    assert result.chunk is None
    assert session.cleared == 1


def test_v2_candidate_wrapper_never_enables_live_or_selects_lerobot():
    project_root = Path(__file__).resolve().parents[3]
    script = (
        project_root / "scripts" / "run_task_c_async_handoff_v2_candidate.sh"
    ).read_text(encoding="utf-8")
    assert "{data: true}" not in script
    assert "/control/select_lerobot" not in script
    assert "{data: false}" in script
    assert "/control/select_disabled" in script
    assert "does not enable Live" in script


def test_v2_candidate_wrapper_uses_reviewed_command_acceleration_default():
    project_root = Path(__file__).resolve().parents[3]
    script = (
        project_root / "scripts" / "run_task_c_async_handoff_v2_candidate.sh"
    ).read_text(encoding="utf-8")
    assert "COMMAND_ACCELERATION_LIMIT_MM_S2:-4000" in script
    assert (
        "MAX_CROSSFADE_COMMAND_ACCELERATION_MM_S2:-${command_acceleration_limit_mm_s2}"
        in script
    )
