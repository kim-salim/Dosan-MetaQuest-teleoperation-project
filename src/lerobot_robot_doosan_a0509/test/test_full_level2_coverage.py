from __future__ import annotations

import json
from pathlib import Path

import pytest

from lerobot_robot_doosan_a0509.interior_policy.catalog import (
    load_operator_catalog,
)
from lerobot_robot_doosan_a0509.interior_policy.contracts import (
    WorldState,
    apply_operator_effects,
)
from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (
    EdgeRuntimeRegistry,
    compile_plan_to_multi_v2,
    uniform_cost_search_level2,
)
from lerobot_robot_doosan_a0509.interior_policy.planner import (
    CostBreakdown,
    Plan,
    PlanStep,
    operator_applicability,
    uniform_cost_search,
)
from lerobot_robot_doosan_a0509.interior_policy.v2_adapter import (
    SymbolicTransitionValidator,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
LEGACY_CATALOG_PATH = (
    REPOSITORY_ROOT
    / "config/interior_policy/a0509_t1_t6_operator_catalog_v1.json"
)
ARTIFACT_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_full_level2_coverage_2026-08-29"
)
INVENTORY_PATH = ARTIFACT_ROOT / "level1_edge_inventory.json"
SUMMARY_PATH = ARTIFACT_ROOT / "edge_coverage_summary.json"
REGISTRY_PATH = ARTIFACT_ROOT / "edge_registry_all_level2.json"
RECOVERY_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_borderline_curvature_recovery_2026-08-29"
)
RECOVERY_SUMMARY_PATH = RECOVERY_ROOT / "recovery_summary.json"
EXTENDED_REGISTRY_PATH = (
    RECOVERY_ROOT / "edge_registry_all_level2_extended.json"
)


def _load(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def catalog():
    return load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        config_path=LEGACY_CATALOG_PATH,
    )


@pytest.fixture(scope="module")
def inventory():
    return _load(INVENTORY_PATH)


@pytest.fixture(scope="module")
def summary():
    return _load(SUMMARY_PATH)


def _plan_for_edge(edge, catalog) -> Plan:
    source = catalog.by_id[edge["source_operator"]]
    successor = catalog.by_id[edge["successor_operator"]]
    state_before = WorldState(**dict(edge["witness_state_before"]))
    assert operator_applicability(state_before, source).valid
    state_after_source = apply_operator_effects(state_before, source)
    assert state_after_source.to_record() == edge["witness_state_after_source"]
    assert operator_applicability(state_after_source, successor).valid
    state_after_successor = apply_operator_effects(state_after_source, successor)
    source_cost = CostBreakdown(
        base_cost=source.base_cost,
        policy_switch_penalty=0.0,
        same_policy_continuation_penalty=0.0,
        transition_cost=0.0,
    )
    successor_cost = CostBreakdown(
        base_cost=successor.base_cost,
        policy_switch_penalty=catalog.target.cost_config.policy_switch_penalty,
        same_policy_continuation_penalty=0.0,
        transition_cost=0.0,
    )
    steps = (
        PlanStep(source, state_before, state_after_source, source_cost),
        PlanStep(
            successor,
            state_after_source,
            state_after_successor,
            successor_cost,
        ),
    )
    return Plan(
        steps=steps,
        total_cost=sum(step.cost.total for step in steps),
        final_state=state_after_successor,
    )


def test_inventory_exhausts_declared_direct_symbolic_edges(catalog, inventory):
    assert inventory["operator_count"] == 21
    assert inventory["edge_count"] == 128
    assert inventory["context_independent_edge_count"] == 36
    assert inventory["context_dependent_edge_count"] == 92
    assert inventory["robot_commands_published"] == 0
    assert inventory["physical_validation_performed"] is False
    keys = [
        (edge["source_operator"], edge["successor_operator"])
        for edge in inventory["edges"]
    ]
    assert len(keys) == len(set(keys))
    validator = SymbolicTransitionValidator()
    for edge in inventory["edges"]:
        source = catalog.by_id[edge["source_operator"]]
        successor = catalog.by_id[edge["successor_operator"]]
        state_after = WorldState(**dict(edge["witness_state_after_source"]))
        validation = validator.validate(source, successor, state_after)
        assert validation.valid, (edge["edge_id"], validation.reasons)
        assert validation.transition_type in {
            "empty_gripper_free_space_reposition",
            "held_blue_block_free_transport",
        }


def test_full_coverage_partition_is_complete(summary):
    assert summary["counts"] == {
        "level1_direct_edges": 128,
        "context_independent_edges": 36,
        "context_dependent_edges": 92,
        "bridge_feasible_edges": 84,
        "level2_edges": 80,
        "no_bridge_candidate_edges": 44,
        "shadow_failed_edges": 4,
        "unshadowed_edges": 0,
    }
    assert summary["robot_commands_published"] == 0
    assert summary["live_enabled"] is False
    assert summary["mux_selected"] is False
    assert summary["physical_validation_performed"] is False


def test_all_level2_edges_load_and_compile_command_free(
    catalog,
    inventory,
    summary,
):
    registry = EdgeRuntimeRegistry.load(REGISTRY_PATH)
    passing = {
        (edge["source_operator"], edge["successor_operator"])
        for edge in summary["edges"]
        if edge["classification"] == "LEVEL_2_RUNTIME_CONTRACT_COMPATIBLE"
    }
    assert len(registry.edges) == 80
    assert set(registry.by_key) == passing
    inventory_by_key = {
        (edge["source_operator"], edge["successor_operator"]): edge
        for edge in inventory["edges"]
    }
    for index, spec in enumerate(registry.edges):
        assert spec.handoff_manifest_path.is_file()
        assert spec.policy_shadow_path.is_file()
        plan = _plan_for_edge(inventory_by_key[spec.key], catalog)
        compiled = compile_plan_to_multi_v2(
            plan,
            catalog,
            registry,
            composition_id=f"full_level2_edge_{index:03d}",
        )
        assert len(compiled.edge_specs) == 1
        assert compiled.edge_specs[0].key == spec.key
        assert (
            compiled.mapping["planner_provenance"]
            ["physical_validation_performed"]
            is False
        )


def test_shadow_failures_are_not_registered(summary):
    failed = {
        (edge["source_operator"], edge["successor_operator"])
        for edge in summary["edges"]
        if edge["classification"] == "BRIDGE_FEASIBLE_SHADOW_FAILED"
    }
    assert failed == {
        ("T3.deliver_to_floor", "T2.acquire_from_floor"),
        ("T3.deliver_to_floor", "T5.open_drawer"),
        ("T6.acquire_moving_block", "T2.deliver_to_black_table"),
        ("T7.acquire_from_drawer_top", "T2.deliver_to_black_table"),
    }
    registry = EdgeRuntimeRegistry.load(REGISTRY_PATH)
    assert failed.isdisjoint(registry.by_key)


def test_drawer_to_floor_is_symbolic_but_not_falsely_promoted(
    catalog,
    summary,
):
    key = ("T5.acquire_from_drawer", "T3.deliver_to_floor")
    record = next(
        edge
        for edge in summary["edges"]
        if (edge["source_operator"], edge["successor_operator"]) == key
    )
    assert record["context_independent"] is True
    assert record["candidate_total"] == 900
    assert record["candidate_feasible"] == 0
    assert record["classification"] == "LEVEL_1_NO_BRIDGE_CANDIDATE"
    registry = EdgeRuntimeRegistry.load(REGISTRY_PATH)
    assert key not in registry.by_key

    initial = WorldState(
        drawer="open",
        white_container="unknown",
        blue_block_location="drawer",
        holding="none",
        gripper="open",
        contact_mode="free_space",
    )
    result = uniform_cost_search(
        initial,
        {
            "blue_block_location": "floor",
            "holding": "none",
            "gripper": "open",
        },
        catalog.operators,
        cost_config=catalog.target.cost_config,
    )
    assert result.best_plan.operator_ids == key


def test_level2_search_routes_around_unregistered_direct_edge(catalog):
    registry = EdgeRuntimeRegistry.load(REGISTRY_PATH)
    initial = WorldState(
        drawer="open",
        white_container="unknown",
        blue_block_location="drawer",
        holding="none",
        gripper="open",
        contact_mode="free_space",
    )
    result = uniform_cost_search_level2(
        initial,
        {
            "blue_block_location": "floor",
            "holding": "none",
            "gripper": "open",
        },
        catalog.operators,
        registry,
        cost_config=catalog.target.cost_config,
    )
    assert result.best_plan is not None
    assert result.best_plan.operator_ids == (
        "T5.acquire_from_drawer",
        "T5.deliver_to_black_table",
        "T3.acquire_from_black_table",
        "T3.deliver_to_floor",
    )
    assert result.best_plan.policy_sequence == ("T5", "T3")
    assert result.best_plan.total_cost == pytest.approx(4.25)
    compiled = compile_plan_to_multi_v2(
        result.best_plan,
        catalog,
        registry,
        composition_id="level2_drawer_to_floor_via_black_table",
    )
    assert [spec.key for spec in compiled.edge_specs] == [
        (
            "T5.deliver_to_black_table",
            "T3.acquire_from_black_table",
        )
    ]


def test_level2_search_preserves_controlled_t4_t2_t4_plan(catalog):
    registry = EdgeRuntimeRegistry.load(REGISTRY_PATH)
    result = uniform_cost_search_level2(
        catalog.target.initial_state,
        catalog.target.goal,
        catalog.controlled_operators,
        registry,
        cost_config=catalog.target.cost_config,
    )
    assert result.best_plan is not None
    assert result.best_plan.operator_ids == (
        "T4.open_drawer",
        "T2.acquire_from_floor",
        "T4.deliver_to_drawer",
        "T4.close_drawer",
    )
    assert result.best_plan.policy_sequence == ("T4", "T2", "T4")


def test_borderline_curvature_recovery_registry_is_strict_and_additive():
    summary = _load(RECOVERY_SUMMARY_PATH)
    assert summary["counts"] == {
        "baseline_zero_candidate_edges": 44,
        "bridge_recovered_edges": 10,
        "extended_level2_edges": 86,
        "level2_recovered_edges": 6,
        "not_recovered_edges": 34,
        "recovered_bridge_shadow_failed_edges": 4,
        "recovered_bridge_unshadowed_edges": 0,
    }
    assert summary["curvature_limit_per_mm_unchanged"] == 0.25
    assert summary["minimum_tangent_handle_chord_ratio"] == 0.04
    assert summary["maximum_endpoint_speed_adjustment_mm_s"] == 12.0
    assert summary["robot_commands_published"] == 0
    assert summary["physical_validation_performed"] is False

    baseline = EdgeRuntimeRegistry.load(REGISTRY_PATH)
    extended = EdgeRuntimeRegistry.load(EXTENDED_REGISTRY_PATH)
    assert len(baseline.edges) == 80
    assert len(extended.edges) == 86
    recovered_keys = set(extended.by_key) - set(baseline.by_key)
    assert recovered_keys == {
        (
            "T3.acquire_from_black_table",
            "T8.deliver_unstacked_to_black_table",
        ),
        ("T3.deliver_to_floor", "T1.open_white_container"),
        ("T5.acquire_from_drawer", "T1.deliver_to_white_container"),
        ("T5.acquire_from_drawer", "T3.deliver_to_floor"),
        (
            "T6.acquire_moving_block",
            "T8.deliver_unstacked_to_black_table",
        ),
        ("T6.stack_on_support_block", "T5.open_drawer"),
    }
    for key in recovered_keys:
        spec = extended.by_key[key]
        shadow = _load(spec.policy_shadow_path)
        assert shadow["takeover_success"] is True
        assert shadow["terminal_state"] == "RUN_B"
        assert shadow["prefix_admission"]["valid"] is True
        assert shadow["control_timing"]["deadline_miss_count"] == 0
        assert shadow["robot_commands_published"] == 0


def test_extended_level2_search_uses_recovered_drawer_to_floor_edge(catalog):
    registry = EdgeRuntimeRegistry.load(EXTENDED_REGISTRY_PATH)
    initial = WorldState(
        drawer="open",
        white_container="unknown",
        blue_block_location="drawer",
        holding="none",
        gripper="open",
        contact_mode="free_space",
    )
    result = uniform_cost_search_level2(
        initial,
        {
            "blue_block_location": "floor",
            "holding": "none",
            "gripper": "open",
        },
        catalog.operators,
        registry,
        cost_config=catalog.target.cost_config,
    )
    assert result.best_plan is not None
    assert result.best_plan.operator_ids == (
        "T5.acquire_from_drawer",
        "T3.deliver_to_floor",
    )
    assert result.best_plan.policy_sequence == ("T5", "T3")
    compiled = compile_plan_to_multi_v2(
        result.best_plan,
        catalog,
        registry,
        composition_id="level2_drawer_to_floor_direct_recovered",
    )
    assert [spec.key for spec in compiled.edge_specs] == [
        ("T5.acquire_from_drawer", "T3.deliver_to_floor")
    ]
