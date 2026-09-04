from __future__ import annotations

import json
from pathlib import Path

import pytest

pytest.importorskip("lerobot")

from lerobot_robot_doosan_a0509.task_c_handoff.multi_stage import (
    MultiStageCoordinator,
    MultiStagePlan,
    MultiStageRuntimeState,
)
from lerobot_robot_doosan_a0509.task_c_handoff.policy_registry import (
    PolicyRegistry,
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
            "segment": "S1",
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
            "segment": "S2",
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


def _plan(tmp_path: Path) -> MultiStagePlan:
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
    return MultiStagePlan.load(path)


def test_t4_t2_t4_plan_reuses_two_unique_policies(tmp_path: Path):
    plan = _plan(tmp_path)
    assert [stage.policy_id for stage in plan.stages] == ["T4", "T2", "T4"]
    assert len(plan.policies) == 2
    assert [edge.handoff_manifest.handoff_id for edge in plan.transitions] == [
        "edge_t4_t2",
        "edge_t2_t4",
    ]


def test_multi_stage_coordinator_runs_two_edges_then_completes(tmp_path: Path):
    coordinator = MultiStageCoordinator(_plan(tmp_path), clock=lambda: 10.0)
    coordinator.start()
    assert coordinator.current_stage.policy_id == "T4"

    accepted, transition_id = coordinator.try_request_next(
        expected_stage_id="open_drawer"
    )
    assert accepted and transition_id == "t4_to_t2"
    first = coordinator.begin_requested_transition()
    assert first.transition_id == "t4_to_t2"
    stage = coordinator.complete_takeover(
        transition_id="t4_to_t2", policy_generation=3
    )
    assert stage.policy_id == "T2"

    accepted, transition_id = coordinator.try_request_next(
        expected_stage_id="pick_floor_block"
    )
    assert accepted and transition_id == "t2_to_t4"
    coordinator.begin_requested_transition()
    stage = coordinator.complete_takeover(
        transition_id="t2_to_t4", policy_generation=7
    )
    assert stage.policy_id == "T4"
    assert coordinator.visit_id == 2

    accepted, message = coordinator.try_request_next()
    assert not accepted and "complete" in message
    accepted, stage_id = coordinator.try_complete(
        expected_stage_id="place_in_drawer"
    )
    assert accepted and stage_id == "place_in_drawer"
    assert coordinator.state is MultiStageRuntimeState.COMPLETE


class _FakeSession:
    def __init__(self) -> None:
        self.generation = 0
        self.active_generation = None
        self.queue_size = 0
        self.closed = False

    def deactivate_and_clear(self) -> int:
        self.generation += 1
        self.active_generation = None
        self.queue_size = 0
        return self.generation

    def close(self) -> None:
        self.closed = True


def test_policy_registry_revisit_invalidates_old_t4_queue():
    registry = PolicyRegistry()
    t4 = _FakeSession()
    t2 = _FakeSession()
    registry.register(
        policy_id="T4",
        checkpoint="t4",
        session=t4,
        reuse_context_policy=True,
    )
    registry.register(
        policy_id="T2",
        checkpoint="t2",
        session=t2,
        reuse_context_policy=False,
    )

    assert registry.begin_visit("T4", invalidate_queue=False) == 1
    t4.active_generation = 11
    t4.queue_size = 50
    registry.deactivate_active(invalidate_queue=True)
    assert t4.generation == 1
    assert t4.queue_size == 0

    registry.begin_visit("T2", invalidate_queue=True)
    registry.deactivate_active(invalidate_queue=True)
    assert registry.begin_visit("T4", invalidate_queue=True) == 2
    assert t4.generation == 2
    assert t4.active_generation is None
    assert registry.record()["policies"]["T4"]["visit_count"] == 2


def test_plan_rejects_transition_task_policy_mismatch(tmp_path: Path):
    plan = _plan(tmp_path)
    path = plan.source_path
    value = json.loads(path.read_text(encoding="utf-8"))
    value["stages"][1]["policy_id"] = "T4"
    path.write_text(json.dumps(value), encoding="utf-8")
    with pytest.raises(ValueError, match="source task/policy mismatch|successor task/policy mismatch"):
        MultiStagePlan.load(path)

def test_plan_accepts_phase_supervisor_and_terminal_inference_end(tmp_path: Path):
    plan = _plan(tmp_path)
    path = plan.source_path
    value = json.loads(path.read_text(encoding="utf-8"))
    for index, event in ((0, "closed_then_open"), (1, "open_then_closed")):
        value["stages"][index]["exit_authority"] = "phase_supervisor"
        value["stages"][index]["phase_supervisor"] = {
            "support_artifact": f"support_{index}.npz",
            "gripper_event": event,
            "phase_half_width": 0.05,
            "persistence_ticks": 3,
        }
    value["stages"][2]["exit_authority"] = "policy_inference_end"
    value["stages"][2]["inference_end_steps"] = 1800
    path.write_text(json.dumps(value), encoding="utf-8")

    loaded = MultiStagePlan.load(path)

    assert loaded.stages[0].exit_authority == "phase_supervisor"
    assert loaded.stages[0].phase_supervisor is not None
    assert loaded.stages[0].phase_supervisor.support_artifact_path == (
        tmp_path / "support_0.npz"
    ).resolve()
    assert loaded.stages[-1].exit_authority == "policy_inference_end"
    assert loaded.stages[-1].inference_end_steps == 1800


def test_policy_inference_end_requires_positive_finite_horizon(tmp_path: Path):
    plan = _plan(tmp_path)
    path = plan.source_path
    value = json.loads(path.read_text(encoding="utf-8"))
    value["stages"][-1]["exit_authority"] = "policy_inference_end"
    value["stages"][-1]["inference_end_steps"] = 0
    path.write_text(json.dumps(value), encoding="utf-8")

    with pytest.raises(ValueError, match="inference_end_steps"):
        MultiStagePlan.load(path)
