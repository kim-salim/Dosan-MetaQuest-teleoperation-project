from __future__ import annotations

from types import SimpleNamespace

import pytest

pytest.importorskip("lerobot")

from lerobot_robot_doosan_a0509.task_c_handoff.bridge_runtime import (
    BridgeGenerationError,
)
from lerobot_robot_doosan_a0509.task_c_multi_live_v2_rollout import (
    TaskCMultiLiveV2Strategy,
)


def test_infeasible_bridge_is_retried_without_invalidating_source_queue():
    strategy = object.__new__(TaskCMultiLiveV2Strategy)
    strategy._multi_next_bridge_retry_s = 0.0
    strategy._multi_last_bridge_rejection = None
    strategy._prepared_bridge_commit = None
    strategy.multi_config = SimpleNamespace(multi_bridge_retry_interval_s=0.25)
    events: list[tuple[str, dict]] = []
    strategy._event = lambda name, **details: events.append((name, details))
    strategy._prepare_bridge_commit = lambda *_args: (_ for _ in ()).throw(
        BridgeGenerationError(["workspace_violation"])
    )

    assert not strategy._try_prepare_requested_bridge(None, None)
    assert strategy._prepared_bridge_commit is None
    assert events[0][0] == "multi_requested_bridge_rejected"
    assert events[0][1]["command_queue_invalidated"] is False
    assert "workspace_violation" in events[0][1]["reason"]
