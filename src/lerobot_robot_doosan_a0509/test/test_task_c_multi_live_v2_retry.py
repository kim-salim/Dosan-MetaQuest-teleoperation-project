from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

pytest.importorskip("lerobot")

from lerobot_robot_doosan_a0509.task_c_multi_live_v2_rollout import (
    TaskCMultiLiveV2Strategy,
)


def _strategy() -> TaskCMultiLiveV2Strategy:
    strategy = object.__new__(TaskCMultiLiveV2Strategy)
    strategy._multi_next_bridge_retry_s = 0.0
    strategy._multi_last_bridge_rejection = None
    strategy._prepared_bridge_commit = None
    strategy.multi_config = SimpleNamespace(multi_bridge_retry_interval_s=0.25)
    strategy._event = lambda *_args, **_kwargs: None
    return strategy


def test_missing_causal_velocity_waits_instead_of_failing_closed():
    strategy = _strategy()
    strategy._prepare_bridge_commit = lambda *_args: (_ for _ in ()).throw(
        RuntimeError("V2 A-exit velocity is unavailable")
    )
    assert not strategy._try_prepare_requested_bridge(None, None)
    assert strategy._prepared_bridge_commit is None
    assert strategy._multi_next_bridge_retry_s > time.monotonic()


def test_other_runtime_bridge_prerequisite_failure_is_not_suppressed():
    strategy = _strategy()
    strategy._prepare_bridge_commit = lambda *_args: (_ for _ in ()).throw(
        RuntimeError("V2 A-exit actual/ack position tracking error")
    )
    with pytest.raises(RuntimeError, match="tracking error"):
        strategy._try_prepare_requested_bridge(None, None)


def test_later_edge_does_not_rebind_runtime_during_retry_backoff():
    strategy = _strategy()
    strategy._multi_next_bridge_retry_s = time.monotonic() + 10.0
    strategy._multi_coordinator = SimpleNamespace(requested_transition=object())
    configured: list[object] = []
    strategy._configure_edge_runtime = lambda edge: configured.append(edge)
    assert not strategy._start_later_requested_edge(None, None, None)
    assert configured == []
