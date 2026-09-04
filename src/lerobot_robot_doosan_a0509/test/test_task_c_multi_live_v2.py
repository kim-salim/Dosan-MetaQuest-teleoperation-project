from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("lerobot")

from lerobot_robot_doosan_a0509.task_c_handoff.multi_stage import (
    MultiStageRuntimeState,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import BridgeAdmissionMode
from lerobot_robot_doosan_a0509.task_c_multi_live_v2_entrypoint import (
    require_task_c_multi_live_v2_arguments,
)
from lerobot_robot_doosan_a0509.task_c_multi_live_v2_rollout import (
    TaskCMultiLiveV2Strategy,
    TaskCMultiLiveV2StrategyConfig,
)


def _handoff(path: Path, source: str, successor: str, handoff_id: str) -> None:
    semantic = {
        "gripper_state": "closed",
        "held_object": "blue_block",
        "contact_mode": "free_transport_assumed",
    }
    value = {
        "schema_version": "a0509.task_c_handoff_episode.v2",
        "composition_id": "t4_t2_t4",
        "handoff_id": handoff_id,
        "source": {
            "task": source,
            "segment": "planner_selected",
            "phase": 0.5,
            "support_episode": 1,
            "support_frame": 10,
            "nominal_pose_mm_deg": [100, 0, 400, 0, 150, 0],
            "nominal_velocity_mm_s": [5, 0, 0],
            "support_radius_mm": 1000,
            "semantic": semantic,
        },
        "successor": {
            "task": successor,
            "segment": "planner_selected",
            "phase": 0.5,
            "support_episode": 2,
            "support_frame": 20,
            "nominal_pose_mm_deg": [150, 0, 410, 0, 150, 0],
            "nominal_velocity_mm_s": [2, 0, 0],
            "support_radius_mm": 1000,
            "semantic": semantic,
        },
        "bridge": {
            "duration_s": 4.0,
            "nominal_length_mm": 50,
            "transport_floor_mm": 300,
        },
        "generation": {"method": "manual_reviewed"},
        "validation": {
            "hard_filter_passed": True,
            "semantic_authority": "external_planner",
            "semantic_diagnostics": [],
            "robot_executable": False,
            "dry_run_only": True,
            "ik_checked": False,
            "collision_checked": False,
        },
    }
    path.write_text(json.dumps(value), encoding="utf-8")


def _plan(tmp_path: Path) -> Path:
    first = tmp_path / "t4_to_t2.json"
    second = tmp_path / "t2_to_t4.json"
    _handoff(first, "T4", "T2", "edge_t4_t2")
    _handoff(second, "T2", "T4", "edge_t2_t4")
    value = {
        "schema_version": "a0509.task_c_multi_stage_plan.v1",
        "composition_id": "open_pick_place",
        "policies": {
            "T4": {
                "checkpoint": "/models/t4/pretrained_model",
                "reuse_context_policy": True,
            },
            "T2": {
                "checkpoint": "/models/t2/pretrained_model",
                "reuse_context_policy": False,
            },
        },
        "stages": [
            {
                "stage_id": "open_drawer",
                "policy_id": "T4",
                "exit_authority": "external_planner",
            },
            {
                "stage_id": "pick_floor_block",
                "policy_id": "T2",
                "exit_authority": "external_planner",
            },
            {
                "stage_id": "place_in_drawer",
                "policy_id": "T4",
                "exit_authority": "external_complete",
                "terminal": True,
            },
        ],
        "transitions": [
            {
                "transition_id": "t4_to_t2",
                "source_stage": "open_drawer",
                "successor_stage": "pick_floor_block",
                "handoff_manifest": first.name,
            },
            {
                "transition_id": "t2_to_t4",
                "source_stage": "pick_floor_block",
                "successor_stage": "place_in_drawer",
                "handoff_manifest": second.name,
            },
        ],
    }
    path = tmp_path / "plan.json"
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


def test_config_derives_first_edge_and_first_successor(tmp_path: Path):
    plan_path = _plan(tmp_path)
    config = TaskCMultiLiveV2StrategyConfig(
        multi_stage_plan=str(plan_path),
        v2_trace_jsonl_path=str(tmp_path / "trace.jsonl"),
        enable_endpoint_fallback=False,
    )
    assert config.handoff_episode_manifest == str(
        (tmp_path / "t4_to_t2.json").resolve()
    )
    assert config.checkpoint_b == str(
        Path("/models/t2/pretrained_model").resolve()
    )
    assert config.semantic_authority == "external_planner"


def test_config_accepts_edge_scoped_endpoint_fallback(tmp_path: Path):
    config = TaskCMultiLiveV2StrategyConfig(
        multi_stage_plan=str(_plan(tmp_path)),
        v2_trace_jsonl_path=str(tmp_path / "trace.jsonl"),
        enable_endpoint_fallback=True,
    )
    assert config.enable_endpoint_fallback is True
    strategy = object.__new__(TaskCMultiLiveV2Strategy)
    assert strategy._uses_legacy_endpoint_fallback() is False


class _PlannerMailbox:
    def __init__(self, state: MultiStageRuntimeState) -> None:
        self.state = state
        self.requested_transition = object()
        self.current_stage = type(
            "Stage",
            (),
            {"exit_authority": "external_planner"},
        )()


def test_initial_act_is_not_cut_without_external_transition_request():
    strategy = object.__new__(TaskCMultiLiveV2Strategy)
    strategy._multi_coordinator = _PlannerMailbox(MultiStageRuntimeState.RUN_STAGE)
    strategy._multi_plan = type("Plan", (), {"transitions": (object(),)})()
    assert not strategy._a_exit_commit_ready(None, None, None, None)


def test_entrypoint_requires_explicit_endpoint_fallback_mode():
    arguments = [
        "--strategy.type=task_c_multi_live_v2",
        "--strategy.handoff_mode=async_window_v2",
        "--strategy.acknowledge_uncertified_manifest=true",
        "--strategy.enable_endpoint_fallback=true",
        "--strategy.runtime_manifest=/tmp/runtime.json",
        "--strategy.multi_stage_plan=/tmp/plan.json",
        "--strategy.event_jsonl_path=/tmp/events.jsonl",
        "--strategy.v2_trace_jsonl_path=/tmp/trace.jsonl",
        "--robot.mode=policy_live",
        "--return_to_initial_position=false",
        "--inference.type=rtc",
        "--inference.rtc.enabled=false",
        "--policy.n_action_steps=100",
    ]
    require_task_c_multi_live_v2_arguments(arguments)
    baseline = [
        value.replace("enable_endpoint_fallback=true", "enable_endpoint_fallback=false")
        for value in arguments
    ]
    require_task_c_multi_live_v2_arguments(baseline)
    missing = [
        value for value in arguments
        if not value.startswith("--strategy.enable_endpoint_fallback=")
    ]
    with pytest.raises(ValueError, match="explicit boolean"):
        require_task_c_multi_live_v2_arguments(missing)


class _TerminalMailbox:
    def __init__(self) -> None:
        self.state = MultiStageRuntimeState.RUN_STAGE
        self.current_stage = type(
            "Stage",
            (),
            {
                "stage_id": "final_t4",
                "policy_id": "T4",
                "exit_authority": "policy_inference_end",
                "inference_end_steps": 1800,
            },
        )()

    def try_complete(self, *, expected_stage_id=None):
        assert expected_stage_id == "final_t4"
        self.state = MultiStageRuntimeState.COMPLETE
        return True, "final_t4"


def test_terminal_policy_inference_horizon_completes_at_configured_step():
    strategy = object.__new__(TaskCMultiLiveV2Strategy)
    strategy._multi_coordinator = _TerminalMailbox()
    strategy._b_steps_sent = 1799
    events = []
    strategy._event = lambda event, **details: events.append((event, details))

    assert not strategy._maybe_complete_terminal_inference()
    assert strategy._multi_coordinator.state is MultiStageRuntimeState.RUN_STAGE

    strategy._b_steps_sent = 1800
    assert strategy._maybe_complete_terminal_inference()
    assert strategy._multi_coordinator.state is MultiStageRuntimeState.COMPLETE
    assert events[-1][0] == "multi_terminal_inference_end"
    assert events[-1][1]["policy_done_token_available"] is False


def test_phase_supervisor_only_requests_transition_through_mailbox():
    stage = type(
        "Stage",
        (),
        {
            "stage_id": "open_drawer",
            "policy_id": "T4",
            "exit_authority": "phase_supervisor",
            "phase_supervisor": type(
                "SupervisorConfig",
                (),
                {
                    "gripper_event_mode": type(
                        "Mode", (), {"value": "closed_then_open"}
                    )()
                },
            )(),
        },
    )()
    edge = type(
        "Edge",
        (),
        {
            "transition_id": "t4_to_t2",
            "handoff_manifest": type(
                "Manifest",
                (),
                {
                    "source": type(
                        "Boundary",
                        (),
                        {
                            "phase": 1.0,
                            "semantic": type(
                                "Semantic",
                                (),
                                {"gripper_state": "open"},
                            )(),
                        },
                    )()
                },
            )(),
        },
    )()

    class Mailbox:
        state = MultiStageRuntimeState.RUN_STAGE
        current_stage = stage
        stage_index = 0

        def try_request_next(self, *, expected_stage_id=None):
            assert expected_stage_id == "open_drawer"
            self.state = MultiStageRuntimeState.TRANSITION_REQUESTED
            return True, "t4_to_t2"

    status = type(
        "Status",
        (),
        {
            "tracker_latency_ms": 0.03,
            "deadline_exceeded": False,
            "commit_ready": True,
            "record": lambda self: {
                "commit_ready": True,
                "tracker_latency_ms": self.tracker_latency_ms,
            },
        },
    )()

    class Supervisor:
        def update(self, **kwargs):
            assert kwargs["tcp_position_mm"].shape == (3,)
            assert kwargs["gripper_closed"] is False
            return status

    strategy = object.__new__(TaskCMultiLiveV2Strategy)
    strategy._multi_coordinator = Mailbox()
    strategy._multi_phase_supervisors = {"open_drawer": Supervisor()}
    strategy._multi_latest_phase_status = {}
    strategy._multi_phase_latency_ms = {"open_drawer": []}
    strategy._multi_plan = type(
        "Plan",
        (),
        {"transition_after": lambda self, index: edge},
    )()
    strategy._estimate_actual_boundary_velocity = lambda: np.zeros(3)
    events = []
    strategy._event = lambda event, **details: events.append((event, details))
    capture = type(
        "Capture",
        (),
        {
            "tcp_position_mm": np.array([1.0, 2.0, 3.0]),
            "semantic_state": type(
                "SemanticState", (), {"gripper_closed": False}
            )(),
        },
    )()

    assert strategy._update_automatic_phase_supervisor(capture)
    assert (
        strategy._multi_coordinator.state
        is MultiStageRuntimeState.TRANSITION_REQUESTED
    )
    assert strategy._multi_phase_latency_ms["open_drawer"] == [0.03]
    assert events[-1][0] == "multi_phase_transition_requested"
    assert events[-1][1]["source_policy_queue_invalidated"] is False


def test_flexible_phase_prearm_polls_worker_before_transition_request():
    stage = type(
        "Stage",
        (),
        {
            "stage_id": "acquire",
            "policy_id": "T7",
            "exit_authority": "phase_supervisor",
            "phase_supervisor": type(
                "SupervisorConfig",
                (),
                {
                    "gripper_event_mode": type(
                        "Mode", (), {"value": "open_then_closed"}
                    )()
                },
            )(),
        },
    )()
    edge = type(
        "Edge",
        (),
        {
            "transition_id": "t7_to_t3",
            "handoff_manifest": type(
                "Manifest",
                (),
                {
                    "source": type(
                        "Boundary",
                        (),
                        {
                            "phase": 1.0,
                            "semantic": type(
                                "Semantic", (), {"gripper_state": "closed"}
                            )(),
                        },
                    )()
                },
            )(),
        },
    )()

    class Mailbox:
        state = MultiStageRuntimeState.RUN_STAGE
        current_stage = stage
        stage_index = 0

        def try_request_next(self, **_kwargs):
            raise AssertionError("prearm must not request a transition")

    status = type(
        "Status",
        (),
        {
            "tracker_latency_ms": 0.02,
            "deadline_exceeded": False,
            "prearmed": True,
            "commit_ready": False,
            "phase_ready": False,
            "estimated_phase": 0.08,
            "record": lambda self: {
                "prearmed": True,
                "commit_ready": False,
            },
        },
    )()

    class Supervisor:
        phase = 0.08

        def update(self, **_kwargs):
            status.estimated_phase = self.phase
            return status

        def execution_collar_bounds(self):
            return (0.09, 0.21)

        def execution_collar_ready(self, current):
            return bool(
                current.prearmed
                and 0.09 <= current.estimated_phase <= 0.21
            )

    supervisor = Supervisor()
    strategy = object.__new__(TaskCMultiLiveV2Strategy)
    strategy._multi_coordinator = Mailbox()
    strategy._multi_phase_supervisors = {"acquire": supervisor}
    strategy._multi_latest_phase_status = {}
    strategy._multi_phase_latency_ms = {"acquire": []}
    strategy._multi_plan = type(
        "Plan", (), {"transition_after": lambda self, index: edge}
    )()
    strategy._estimate_actual_boundary_velocity = lambda: np.zeros(3)
    strategy._v2_runtime_config = type(
        "RuntimeConfig",
        (), {"bridge_admission_mode": BridgeAdmissionMode.FLEXIBLE_LEVEL2},
    )()
    strategy._multi_configured_edge_id = edge.transition_id
    polls = []
    strategy._try_prepare_flexible_bridge = (
        lambda capture, state, *, allow_commit: polls.append(
            (capture, state, allow_commit)
        )
    )
    strategy._event = lambda *_args, **_kwargs: None
    capture = type(
        "Capture",
        (),
        {
            "tcp_position_mm": np.array([1.0, 2.0, 3.0]),
            "semantic_state": type(
                "SemanticState", (), {"gripper_closed": True}
            )(),
        },
    )()
    robot_state = np.zeros(13, dtype=np.float64)

    assert not strategy._update_automatic_phase_supervisor(capture, robot_state)
    assert polls == []
    supervisor.phase = 0.09
    assert not strategy._update_automatic_phase_supervisor(capture, robot_state)
    assert polls == [(capture, robot_state, False)]
    assert strategy._multi_coordinator.state is MultiStageRuntimeState.RUN_STAGE

    # Once submitted, worker polling must remain non-blocking and continue even
    # if the next phase estimate has already crossed the narrow source collar.
    strategy._flexible_bridge_worker = type(
        "Worker", (), {"inflight_generation": 7}
    )()
    supervisor.phase = 0.22
    assert not strategy._update_automatic_phase_supervisor(capture, robot_state)
    assert polls[-1] == (capture, robot_state, False)
    assert len(polls) == 2


def test_multi_adaptive_rebase_uses_window_then_latches_requested_cut():
    stage = type(
        "Stage",
        (),
        {
            "stage_id": "acquire",
            "exit_authority": "phase_supervisor",
        },
    )()
    status = type(
        "Status",
        (),
        {
            "estimated_phase": 0.15,
            "phase_ready": False,
            "commit_ready": False,
            "support_ready": True,
            "semantic_ready": True,
            "deadline_exceeded": False,
        },
    )()

    class Supervisor:
        def adaptive_rebase_ready(self, current):
            return bool(
                current.phase_ready
                and 0.16 <= current.estimated_phase <= 0.21
            )

    strategy = object.__new__(TaskCMultiLiveV2Strategy)
    coordinator = type(
        "Coordinator",
        (),
        {
            "current_stage": stage,
            "state": MultiStageRuntimeState.RUN_STAGE,
        },
    )()
    strategy._multi_coordinator = coordinator
    strategy._multi_phase_supervisors = {"acquire": Supervisor()}
    strategy._multi_latest_phase_status = {"acquire": status}

    assert not strategy._multi_adaptive_rebase_ready()
    for phase in (0.16, 0.18, 0.21):
        status.estimated_phase = phase
        status.phase_ready = True
        assert strategy._multi_adaptive_rebase_ready()
    status.estimated_phase = 0.22
    assert not strategy._multi_adaptive_rebase_ready()
    coordinator.state = MultiStageRuntimeState.TRANSITION_REQUESTED
    assert strategy._multi_adaptive_rebase_ready()
    status.support_ready = False
    assert not strategy._multi_adaptive_rebase_ready()
    status.support_ready = True
    status.semantic_ready = False
    assert not strategy._multi_adaptive_rebase_ready()
    status.semantic_ready = True
    status.deadline_exceeded = True
    assert not strategy._multi_adaptive_rebase_ready()

def test_pending_execution_tail_deadline_is_still_fail_closed():
    stage = type(
        "Stage",
        (),
        {
            "stage_id": "acquire",
            "policy_id": "T7",
            "exit_authority": "phase_supervisor",
            "phase_supervisor": type(
                "SupervisorConfig",
                (),
                {
                    "gripper_event_mode": type(
                        "Mode", (), {"value": "open_then_closed"}
                    )(),
                    "execution_tail": None,
                },
            )(),
        },
    )()
    edge = type(
        "Edge",
        (),
        {
            "transition_id": "t7_to_t3",
            "handoff_manifest": type(
                "Manifest",
                (),
                {
                    "source": type(
                        "Boundary",
                        (),
                        {
                            "phase": 1.0,
                            "segment": "S1",
                            "semantic": type(
                                "Semantic", (), {"gripper_state": "closed"}
                            )(),
                        },
                    )()
                },
            )(),
        },
    )()

    class Mailbox:
        state = MultiStageRuntimeState.TRANSITION_REQUESTED
        current_stage = stage
        stage_index = 0

    status = type(
        "Status",
        (),
        {
            "tracker_latency_ms": 0.02,
            "deadline_exceeded": True,
            "prearmed": True,
            "commit_ready": True,
            "record": lambda self: {
                "deadline_exceeded": True,
                "commit_ready": True,
            },
        },
    )()

    class Supervisor:
        def update(self, **_kwargs):
            return status

    strategy = object.__new__(TaskCMultiLiveV2Strategy)
    strategy._multi_coordinator = Mailbox()
    strategy._multi_phase_supervisors = {"acquire": Supervisor()}
    strategy._multi_latest_phase_status = {}
    strategy._multi_phase_latency_ms = {"acquire": []}
    strategy._multi_plan = type(
        "Plan", (), {"transition_after": lambda self, index: edge}
    )()
    strategy._estimate_actual_boundary_velocity = lambda: np.zeros(3)
    strategy._event = lambda *_args, **_kwargs: None
    capture = type(
        "Capture",
        (),
        {
            "tcp_position_mm": np.array([1.0, 2.0, 3.0]),
            "semantic_state": type(
                "SemanticState", (), {"gripper_closed": True}
            )(),
        },
    )()

    with pytest.raises(RuntimeError, match="phase deadline exceeded"):
        strategy._update_automatic_phase_supervisor(
            capture,
            np.zeros(13, dtype=np.float64),
        )
