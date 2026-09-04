from __future__ import annotations

import json
from argparse import Namespace
from pathlib import Path

import pytest
from std_msgs.msg import String

from scripts.a0509_multi_stage_live_trial_gate import (
    MOTION_AUTHORIZATION,
    MultiStageLiveTrialGate,
    _assert_ready_plan,
    run,
)


def test_ready_event_must_match_exact_compiled_plan(tmp_path):
    plan = (tmp_path / "plan.json").resolve()
    plan.write_text("{}\n", encoding="utf-8")
    event = {
        "plan_path": str(plan),
        "composition_id": "web_dijkstra_test",
    }
    _assert_ready_plan(
        event,
        expected_plan=plan,
        expected_composition_id="web_dijkstra_test",
    )

    with pytest.raises(RuntimeError, match="plan mismatch"):
        _assert_ready_plan(
            {**event, "plan_path": str(tmp_path / "different.json")},
            expected_plan=plan,
            expected_composition_id="web_dijkstra_test",
        )
    with pytest.raises(RuntimeError, match="composition mismatch"):
        _assert_ready_plan(
            {**event, "composition_id": "other"},
            expected_plan=plan,
            expected_composition_id="web_dijkstra_test",
        )


def test_dedicated_multi_ready_state_binds_exact_plan():
    gate = object.__new__(MultiStageLiveTrialGate)
    gate.multi_ready_event = None
    ready = {
        "event": "multi_stage_v2_ready",
        "composition_id": "web_dijkstra_test",
        "plan_path": "/tmp/plan.json",
    }

    gate._on_multi_ready(String(data=json.dumps(ready)))
    assert gate.multi_ready_event == ready

    gate.multi_ready_event = None
    gate._on_multi_ready(
        String(
            data=json.dumps(
                {
                    "event": "multi_stage_v2_ready",
                    "composition_id": "",
                    "plan_path": "/tmp/plan.json",
                }
            )
        )
    )
    assert gate.multi_ready_event is None


def _args(plan: Path, **updates) -> Namespace:
    value = {
        "execute": True,
        "motion_authorization": MOTION_AUTHORIZATION,
        "supervisor_pid": None,
        "multi_stage_plan": str(plan),
        "expected_composition_id": "web_dijkstra_test",
        "startup_timeout_sec": 1.0,
        "episode_timeout_sec": 1.0,
        "target_max_age_sec": 0.3,
        "state_max_age_sec": 0.5,
        "commanded_max_age_sec": 0.3,
        "position_limit_mm": 50.0,
        "orientation_limit_deg": 10.0,
        "report_json": "",
    }
    value.update(updates)
    return Namespace(**value)


def test_execute_requires_exact_phrase_and_live_supervisor_before_ros_init(tmp_path):
    plan = tmp_path / "plan.json"
    plan.write_text("{}\n", encoding="utf-8")

    with pytest.raises(SystemExit, match="motion-authorization"):
        run(_args(plan, motion_authorization="wrong"))
    with pytest.raises(SystemExit, match="supervisor-pid"):
        run(_args(plan))
