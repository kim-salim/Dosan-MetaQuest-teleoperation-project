from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("lerobot")

from offline_tools.cross_task_handoff.authority import SemanticAuthority
from lerobot_robot_doosan_a0509.task_c_live_v2_rollout import (
    TaskCLiveV2Strategy,
)


def _external_strategy() -> TaskCLiveV2Strategy:
    strategy = object.__new__(TaskCLiveV2Strategy)
    strategy._semantic_authority = SemanticAuthority.EXTERNAL_PLANNER
    return strategy


def test_external_planner_delegates_a_and_b_gripper_sequences_to_policy():
    strategy = _external_strategy()
    for target in (0.05, 0.95, 0.15, 0.85):
        outgoing, latched = strategy._task_a_gripper_target(
            target,
            open_seen=True,
        )
        assert outgoing == target
        assert not latched

    state = np.zeros(13, dtype=np.float64)
    for target in (0.0, 1.0, 0.0, 1.0):
        action = np.array([1, 2, 3, 4, 5, 6, target], dtype=np.float64)
        np.testing.assert_allclose(strategy._prepare_b_action(action, state), action)


def test_external_planner_refresh_hold_repeats_exact_last_7d_action():
    strategy = _external_strategy()
    expected = np.array([1, 2, 3, 4, 5, 6, 0.42], dtype=np.float64)
    strategy._last_command_action = expected.copy()
    strategy._b_refresh_hold_cycles = 0
    strategy._b_refresh_generation = 7
    strategy._b_steps_sent = 9
    sent: list[np.ndarray] = []
    events: list[str] = []
    strategy._send_array = lambda _ctx, action, _source, **_kwargs: sent.append(
        np.asarray(action, dtype=np.float64).copy()
    )
    strategy._event = lambda event, **_details: events.append(event)

    strategy._send_b_refresh_hold(object(), actual_pose=np.zeros(6))

    assert strategy._b_refresh_hold_cycles == 1
    np.testing.assert_allclose(sent, [expected])
    assert events == ["act_b_refresh_queue_hold_started"]
