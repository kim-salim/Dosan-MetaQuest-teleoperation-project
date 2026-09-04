from __future__ import annotations

import json
from pathlib import Path

import pytest

from lerobot_robot_doosan_a0509.interior_policy.catalog import (
    load_operator_catalog,
)
from lerobot_robot_doosan_a0509.interior_policy.episode_control import (
    EpisodePlanRequest,
    compile_episode_decision,
    plan_level2_episode,
)
from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (
    EdgeRuntimeRegistry,
)
from lerobot_robot_doosan_a0509.interior_policy.web_runtime_manifest import (
    build_web_runtime_support_manifest,
)
from lerobot_robot_doosan_a0509.task_c_handoff.multi_stage import MultiStagePlan
from offline_tools.task_c_bridge_v0.runtime_bridge import (
    RuntimeBridgePlanner,
    runtime_entries_from_manifest,
)
from offline_tools.task_c_bridge_v1.runtime_boundary import (
    RepresentativeBoundaryContract,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
CATALOG_PATH = (
    REPOSITORY_ROOT
    / "config/interior_policy/a0509_t1_t8_operator_catalog_spatial_v2.json"
)
REGISTRY_PATH = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_spatial_floor_level2_2026-09-02"
    / "edge_registry_level2_spatial_v4.json"
)
RUNTIME_TEMPLATE_PATH = (
    REPOSITORY_ROOT
    / "docs/artifacts/task_c_t2_t3_floor_to_floor_v2_2026-08-25"
    / "v2_fail_closed_support_runtime_manifest_t2_t3.json"
)


@pytest.fixture(scope="module")
def catalog():
    return load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        config_path=CATALOG_PATH,
        audit_raw_frames=False,
    )


@pytest.fixture(scope="module")
def registry():
    return EdgeRuntimeRegistry.load(REGISTRY_PATH)


def target_request(catalog, **updates) -> EpisodePlanRequest:
    value = {
        "initial_state": {
            "drawer": "closed",
            "white_container": "closed",
            "blue_block_location": "left_floor",
            "holding": "none",
            "gripper": "open",
            "contact_mode": "free_space",
            "support_blue_block_location": "unknown",
            "stack": "unknown",
        },
        "goal": {
            "drawer": "closed",
            "blue_block_location": "drawer",
            "holding": "none",
            "gripper": "open",
        },
        "enabled_operator_ids": [item.id for item in catalog.operators],
        "policy_switch_penalty": 0.25,
        "same_policy_continuation_penalty": 0.0,
        "final_inference_steps": 1800,
        "max_plans": 3,
    }
    value.update(updates)
    return EpisodePlanRequest.from_mapping(value, catalog=catalog)


def test_authoritative_level2_target_plan_is_t4_t2_t4(catalog, registry):
    decision = plan_level2_episode(catalog, registry, target_request(catalog))

    assert decision.runtime_compilable
    assert decision.runtime_blockers == ()
    assert decision.selected_plan is not None
    assert decision.selected_plan.operator_ids == (
        "T4.open_drawer",
        "T2.acquire_from_floor",
        "T4.deliver_to_drawer",
        "T4.close_drawer",
    )
    assert decision.selected_plan.policy_sequence == ("T4", "T2", "T4")
    assert decision.selected_plan.total_cost == pytest.approx(4.5)
    assert len(decision.search.plans) == 3


def test_compiled_plan_and_setup_manifest_parse_in_existing_v2(
    catalog,
    registry,
    tmp_path,
):
    decision = plan_level2_episode(catalog, registry, target_request(catalog))
    plan_path = compile_episode_decision(
        decision,
        catalog=catalog,
        edge_registry=registry,
        output_path=tmp_path / "multi_stage_plan.json",
        composition_id="pytest_web_dijkstra_t4_t2_t4",
    )
    manifest_path = build_web_runtime_support_manifest(
        multi_stage_plan_path=plan_path,
        template_path=RUNTIME_TEMPLATE_PATH,
        output_path=tmp_path / "base_runtime_support_manifest.json",
    )

    plan = MultiStagePlan.load(plan_path)
    assert [stage.policy_id for stage in plan.stages] == ["T4", "T2", "T4"]
    assert len(plan.transitions) == 2
    assert plan.stages[-1].terminal
    assert plan.stages[-1].exit_authority == "policy_inference_end"

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    entries = runtime_entries_from_manifest(manifest)
    planner = RuntimeBridgePlanner.from_manifest(manifest)
    boundary = RepresentativeBoundaryContract.from_manifest(manifest)
    assert len(entries) == 1
    assert planner is not None
    assert boundary.prearm_radius_mm == pytest.approx(40.0)
    assert boundary.commit_radius_mm == pytest.approx(20.0)
    assert planner.feasibility_config["acceleration_limit_mm_s2"] == pytest.approx(
        4000.0
    )
    assert planner.feasibility_config["axis_velocity_limit_mm_s"] == pytest.approx(
        225.0
    )
    assert planner.feasibility_config["jerk_limit_mm_s3"] == pytest.approx(
        4000.0
    )
    assert planner.feasibility_config[
        "integrated_squared_jerk_limit"
    ] == pytest.approx(10_000_000.0)
    assert planner.feasibility_config[
        "boundary_acceleration_jump_limit_mm_s2"
    ] == pytest.approx(4000.0)
    assert manifest["safety"]["live_authority"] == "external_gate_only"
    assert manifest["safety"]["publish_robot_commands"] is False
    assert manifest["web_generation"]["physical_validation_performed"] is False


def test_goal_already_satisfied_is_never_armed(catalog, registry):
    request = target_request(
        catalog,
        goal={"drawer": "closed", "blue_block_location": "left_floor"},
    )
    decision = plan_level2_episode(catalog, registry, request)
    assert decision.execution_kind == "no_op"
    assert decision.runtime_compilable is False
    assert decision.runtime_blockers == ("GOAL_ALREADY_SATISFIED",)


def test_left_floor_to_right_floor_is_not_noop_and_is_level2_compilable(
    catalog, registry
):
    initial = {
        "drawer": "closed",
        "white_container": "closed",
        "blue_block_location": "left_floor",
        "holding": "none",
        "gripper": "open",
        "contact_mode": "free_space",
        "support_blue_block_location": "unknown",
        "stack": "unknown",
    }
    request = target_request(
        catalog,
        initial_state=initial,
        goal={
            "blue_block_location": "right_floor",
            "holding": "none",
            "gripper": "open",
        },
        enabled_operator_ids=[
            "T2.acquire_from_floor",
            "T3.deliver_to_floor",
        ],
    )
    decision = plan_level2_episode(catalog, registry, request)
    assert decision.execution_kind == "multi_v2"
    assert decision.runtime_compilable
    assert decision.selected_plan is not None
    assert decision.selected_plan.operator_ids == (
        "T2.acquire_from_floor",
        "T3.deliver_to_floor",
    )


def test_single_policy_symbolic_plan_is_not_misrouted_to_multi_v2(
    catalog,
    registry,
):
    request = target_request(
        catalog,
        goal={"drawer": "open"},
        enabled_operator_ids=["T4.open_drawer"],
    )
    decision = plan_level2_episode(catalog, registry, request)
    assert decision.selected_plan is not None
    assert decision.selected_plan.operator_ids == ("T4.open_drawer",)
    assert decision.execution_kind == "single_policy"
    assert decision.runtime_compilable is False


def test_missing_level2_edge_returns_no_plan(catalog, registry):
    request = target_request(
        catalog,
        enabled_operator_ids=[
            "T4.open_drawer",
            "T2.acquire_from_floor",
            "T4.deliver_to_drawer",
            "T4.close_drawer",
        ],
    )
    # The controlled path is known to be admitted by the reviewed registry.
    decision = plan_level2_episode(catalog, registry, request)
    assert decision.runtime_compilable

    no_edge_registry = EdgeRuntimeRegistry(
        source_path=registry.source_path,
        edges=tuple(
            edge
            for edge in registry.edges
            if edge.key != ("T4.open_drawer", "T2.acquire_from_floor")
        ),
    )
    rejected = plan_level2_episode(catalog, no_edge_registry, request)
    assert rejected.selected_plan is None
    assert rejected.execution_kind == "no_plan"
    assert rejected.runtime_blockers == ("LEVEL2_NO_PLAN",)


@pytest.mark.parametrize(
    "updates,match",
    [
        ({"goal": {}}, "goal must be a non-empty"),
        ({"enabled_operator_ids": ["T99.fake"]}, "unknown enabled operators"),
        ({"max_plans": 0}, "max_plans"),
        ({"final_inference_steps": 0}, "final_inference_steps"),
        ({"final_inference_steps": 18001}, "final_inference_steps"),
        ({"policy_switch_penalty": float("nan")}, "must be finite"),
        ({"same_policy_continuation_penalty": float("inf")}, "must be finite"),
    ],
)
def test_invalid_web_requests_fail_before_planning(catalog, updates, match):
    with pytest.raises(ValueError, match=match):
        target_request(catalog, **updates)
