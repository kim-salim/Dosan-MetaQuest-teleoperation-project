from __future__ import annotations

import json
from collections import deque
from dataclasses import asdict
from types import SimpleNamespace

from rcl_interfaces.msg import ParameterType, ParameterValue
from std_msgs.msg import String

from offline_tools.task_c_bridge_v0.live_transition import (
    DownstreamControlContract,
)
from scripts.a0509_task_c_live_trial_gate import (
    TaskCLiveTrialGate,
    contract_mismatches,
    gripper_open_is_violation,
    parameter_response_values,
)


def _running_parameters(*, dry_run: bool) -> dict[str, object]:
    contract = DownstreamControlContract()
    return {
        "guard.publish_rate_hz": contract.control_hz,
        "guard.target_posx_topic": contract.selected_target_topic,
        "guard.safe_posx_topic": contract.safe_posx_topic,
        "guard.workspace_min_xyz_mm": list(contract.workspace_min_xyz_mm),
        "guard.workspace_min_limit_enabled": list(
            contract.workspace_min_limit_enabled
        ),
        "guard.workspace_max_xyz_mm": list(contract.workspace_max_xyz_mm),
        "guard.enable_orientation_limits": True,
        "guard.max_orientation_delta_deg": [contract.orientation_limit_deg] * 3,
        "streamer.dry_run": dry_run,
        "streamer.publish_rate_hz": contract.control_hz,
        "streamer.safe_posx_topic": contract.safe_posx_topic,
        "streamer.commanded_posx_topic": contract.commanded_posx_topic,
        "streamer.servol_time_sec": contract.servol_time_s,
        "streamer.servol_use_auto_velocity_acceleration": True,
        "streamer.stream_ramp_linear_mm_per_tick": (
            contract.linear_ramp_mm_per_tick
        ),
        "streamer.stream_ramp_rot_deg_per_tick": (
            contract.orientation_ramp_deg_per_tick
        ),
        "streamer.require_live_enable": True,
        "streamer.require_prepare_before_live": True,
        "streamer.require_robot_ready": True,
        "mux.lerobot_timeout_sec": contract.lerobot_timeout_s,
        "mux.lerobot_target_topic": contract.lerobot_target_topic,
        "mux.selected_target_topic": contract.selected_target_topic,
    }


def test_contract_comparison_permits_dry_preflight_but_not_dry_execution():
    ready = {"downstream_contract": asdict(DownstreamControlContract())}
    running = _running_parameters(dry_run=True)
    assert contract_mismatches(ready, running, execute=False) == []
    mismatches = contract_mismatches(ready, running, execute=True)
    assert len(mismatches) == 1
    assert "dry_run" in mismatches[0]
    running["streamer.dry_run"] = False
    assert contract_mismatches(ready, running, execute=True) == []


def test_contract_comparison_detects_ramp_or_workspace_drift():
    ready = {"downstream_contract": asdict(DownstreamControlContract())}
    running = _running_parameters(dry_run=False)
    running["streamer.stream_ramp_linear_mm_per_tick"] = 7.0
    running["guard.workspace_max_xyz_mm"] = [650.0, 350.0, 650.0]
    mismatches = contract_mismatches(ready, running, execute=True)
    assert len(mismatches) == 2
    assert any("stream_ramp_linear" in value for value in mismatches)
    assert any("workspace_max" in value for value in mismatches)


def test_contract_comparison_detects_topic_wiring_drift():
    ready = {"downstream_contract": asdict(DownstreamControlContract())}
    running = _running_parameters(dry_run=False)
    running["streamer.commanded_posx_topic"] = "/unexpected/commanded"
    mismatches = contract_mismatches(ready, running, execute=True)
    assert len(mismatches) == 1
    assert "commanded_posx_topic" in mismatches[0]


def test_jazzy_get_parameters_response_is_normalized():
    response = SimpleNamespace(
        values=[
            ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=30.0,
            ),
            ParameterValue(
                type=ParameterType.PARAMETER_STRING,
                string_value="/vr/safe_posx",
            ),
        ]
    )
    assert parameter_response_values(["rate", "topic"], response) == {
        "rate": 30.0,
        "topic": "/vr/safe_posx",
    }


def test_recent_pose_match_uses_physical_orientation_and_age():
    now = 10.0
    history = deque(
        [(now - 0.01, (300.0, 0.0, 400.0, 179.0, 45.0, -179.0))],
        maxlen=16,
    )
    wrapped = (300.0, 0.0, 400.0, 539.0, 45.0, -539.0)
    assert TaskCLiveTrialGate._matches_recent(wrapped, history, now)
    exact = (300.0, 0.0, 400.0, 179.0, 45.0, -179.0)
    assert TaskCLiveTrialGate._matches_recent(exact, history, now)
    assert not TaskCLiveTrialGate._matches_recent(exact, history, now + 1.0)


def test_dedicated_ready_state_is_independent_of_event_history():
    gate = object.__new__(TaskCLiveTrialGate)
    gate.ready_event = None
    ready = {
        "event": "strategy_ready_external_gate_required",
        "downstream_contract": asdict(DownstreamControlContract()),
    }
    wire_ready = json.loads(json.dumps(ready))

    gate._on_task_ready(String(data=json.dumps(ready)))
    assert gate.ready_event == wire_ready

    gate.ready_event = None
    gate._on_task_ready(
        String(
            data=json.dumps(
                {
                    "event": "unrelated_event",
                    "downstream_contract": ready["downstream_contract"],
                }
            )
        )
    )
    assert gate.ready_event is None
    gate._on_task_ready(String(data="{not-json"))
    assert gate.ready_event is None


def test_gripper_open_gate_is_phase_and_release_authorization_aware():
    assert gripper_open_is_violation(
        "BRIDGE", 0.0, release_authorized=False
    )
    assert gripper_open_is_violation(
        "ACT_B", 0.0, release_authorized=False
    )
    assert not gripper_open_is_violation(
        "ACT_B", 0.0, release_authorized=True
    )
    assert not gripper_open_is_violation(
        "ACT_B", 1.0, release_authorized=False
    )
