from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("lerobot")

from lerobot_robot_doosan_a0509.interior_policy.catalog import load_operator_catalog
from lerobot_robot_doosan_a0509.interior_policy.contracts import WorldState
from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (
    EdgeRuntimeRegistry,
    Level2RegistryAdmission,
    compile_plan_to_multi_v2,
    uniform_cost_search_level2,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import BridgeAdmissionMode


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
CANDIDATE_REGISTRY = (
    REPOSITORY_ROOT
    / "docs/artifacts/t7_to_t3_flexible_level2_2026-08-30/edge_registry_v2.json"
)
VERIFIED_REGISTRY = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_spatial_floor_level2_2026-09-02/edge_registry_level2_spatial_v4.json"
)


@pytest.fixture(scope="module")
def catalog():
    return load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        audit_raw_frames=False,
    )


def _initial() -> WorldState:
    return WorldState(
        drawer="closed",
        white_container="closed",
        blue_block_location="drawer_top",
        holding="none",
        gripper="open",
        contact_mode="free_space",
    )


def test_v3_registry_exposes_all_semantic_edges_but_admits_only_verified():
    registry = EdgeRuntimeRegistry.load(VERIFIED_REGISTRY)
    statuses = [item.admission_status for item in registry.edges]
    assert len(statuses) == 127
    assert statuses.count("flexible_verified") == 118
    assert statuses.count("flexible_semantic_candidate") == 9

    flexible = Level2RegistryAdmission.from_registry(
        registry,
        bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
    )
    strict = Level2RegistryAdmission.from_registry(
        registry,
        bridge_admission_mode=BridgeAdmissionMode.STRICT_LEVEL2,
    )
    assert len(flexible.allowed_edges) == 118
    assert not strict.allowed_edges
    registry_keys = {item.key for item in registry.edges}
    assert ("T2.acquire_from_floor", "T3.deliver_to_floor") in registry_keys
    assert ("T3.deliver_to_floor", "T2.acquire_from_floor") not in registry_keys


def test_strict_does_not_mislabel_flexible_candidate_as_verified(catalog):
    registry = EdgeRuntimeRegistry.load(CANDIDATE_REGISTRY)
    operators = (
        catalog.by_id["T7.acquire_from_drawer_top"],
        catalog.by_id["T3.deliver_to_floor"],
    )
    result = uniform_cost_search_level2(
        _initial(),
        {"blue_block_location": "right_floor", "holding": "none"},
        operators,
        registry,
        bridge_admission_mode=BridgeAdmissionMode.STRICT_LEVEL2,
        allow_intermediate_release=False,
    )
    assert result.no_plan


def test_flexible_candidate_is_visible_but_not_executable(catalog):
    registry = EdgeRuntimeRegistry.load(CANDIDATE_REGISTRY)
    operators = (
        catalog.by_id["T7.acquire_from_drawer_top"],
        catalog.by_id["T3.deliver_to_floor"],
    )
    result = uniform_cost_search_level2(
        _initial(),
        {"blue_block_location": "right_floor", "holding": "none"},
        operators,
        registry,
        bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
        allow_intermediate_release=False,
    )
    assert result.no_plan


def test_flexible_verified_direct_t7_to_t3_without_release_detour(catalog):
    registry = EdgeRuntimeRegistry.load(VERIFIED_REGISTRY)
    operators = (
        catalog.by_id["T7.acquire_from_drawer_top"],
        catalog.by_id["T3.deliver_to_floor"],
    )
    result = uniform_cost_search_level2(
        _initial(),
        {"blue_block_location": "right_floor", "holding": "none"},
        operators,
        registry,
        bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
        allow_intermediate_release=False,
    )
    plan = result.best_plan
    assert plan is not None
    assert plan.operator_ids == (
        "T7.acquire_from_drawer_top",
        "T3.deliver_to_floor",
    )
    assert plan.policy_sequence == ("T7", "T3")
    # 1 + 1 operator costs and the configured 0.25 policy-switch penalty.
    assert plan.total_cost == pytest.approx(2.25)

    compiled = compile_plan_to_multi_v2(
        plan,
        catalog,
        registry,
        composition_id="t7_to_t3_flexible_test",
    )
    transition = compiled.mapping["transitions"][0]
    assert transition["bridge_admission_status"] == "flexible_verified"
    assert transition["bridge_admission_mode"] == "flexible_level2"
    assert transition["policy_shadow"].endswith("flexible_policy_shadow.json")
    assert transition["handoff_manifest"].endswith(
        "t7_to_t3_s2_runtime_reference_2026-08-31/"
        "flexible_reference_manifest.json"
    )
    runtime_reference = transition["runtime_source_reference"]
    assert runtime_reference["segment"] == "S2"
    assert runtime_reference["nominal_phase"] == pytest.approx(0.185)
    assert runtime_reference["phase_window"] == pytest.approx([0.16, 0.21])
    assert runtime_reference["episode_count"] == 30
    assert runtime_reference["medoid_episode"] == 15
    assert runtime_reference["validated_runtime_source_bank"] is True
    successor_reference = transition["runtime_successor_reference"]
    assert successor_reference["segment"] == "S2"
    assert successor_reference["reference_phase"] == pytest.approx(0.65)
    assert successor_reference["phase_window"] == pytest.approx([0.13, 0.68])
    assert successor_reference["support_episode"] == 23
    assert successor_reference["interior_path_margin_mm"] == pytest.approx(15.0)
    assert successor_reference["path_margin_to_high_mm"] >= 15.0
    assert successor_reference["runtime_phase_gate"] is False
    assert successor_reference["authority"] == (
        "bridge_reference_and_metadata_only"
    )
    tail = transition["source_execution_tail"]
    assert tail["source_operator"] == "T7.acquire_from_drawer_top"
    assert tail["semantic_exit_segment"] == "S1"
    assert tail["semantic_exit_phase"] == pytest.approx(1.0)
    assert tail["tracking_segment"] == "S2"
    assert tail["commit_phase_low"] == pytest.approx(0.16)
    assert tail["commit_phase_high"] == pytest.approx(0.21)
    assert tail["zero_symbolic_cost"] is True
    assert tail["fixed_z_minimum_used"] is False
    stage_supervisor = compiled.mapping["stages"][0]["phase_supervisor"]
    assert stage_supervisor["execution_tail"] == tail
    assert stage_supervisor["prearm_extra_phase"] == pytest.approx(0.16)
    provenance = compiled.mapping["planner_provenance"]
    assert provenance["physical_validation_performed"] is False
    assert provenance["dijkstra_operator_costs_unchanged"] is True
    assert provenance["zero_cost_execution_tail"]["records"][0]["enabled"] is True


def test_spatial_left_floor_to_right_floor_uses_verified_t2_t3_edge(catalog):
    registry = EdgeRuntimeRegistry.load(VERIFIED_REGISTRY)
    initial = WorldState(
        drawer="closed",
        white_container="closed",
        blue_block_location="left_floor",
        holding="none",
        gripper="open",
        contact_mode="free_space",
    )
    operators = (
        catalog.by_id["T2.acquire_from_floor"],
        catalog.by_id["T3.deliver_to_floor"],
    )
    result = uniform_cost_search_level2(
        initial,
        {"blue_block_location": "right_floor", "holding": "none"},
        operators,
        registry,
        bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
        allow_intermediate_release=False,
    )
    plan = result.best_plan
    assert plan is not None
    assert plan.operator_ids == (
        "T2.acquire_from_floor",
        "T3.deliver_to_floor",
    )
    assert plan.policy_sequence == ("T2", "T3")
    assert plan.total_cost == pytest.approx(2.25)
    compiled = compile_plan_to_multi_v2(
        plan, catalog, registry, composition_id="left_to_right_floor_level2"
    )
    assert tuple(spec.key for spec in compiled.edge_specs) == (
        ("T2.acquire_from_floor", "T3.deliver_to_floor"),
    )


def test_no_intermediate_release_allows_goal_delivery_but_blocks_detour(catalog):
    registry = EdgeRuntimeRegistry.load(VERIFIED_REGISTRY)
    operators = (
        catalog.by_id["T7.acquire_from_drawer_top"],
        catalog.by_id["T7.deliver_to_black_table"],
    )
    final_delivery = uniform_cost_search_level2(
        _initial(),
        {"blue_block_location": "black_table", "holding": "none"},
        operators,
        registry,
        bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
        allow_intermediate_release=False,
    )
    assert final_delivery.best_plan is not None

    detour = uniform_cost_search_level2(
        _initial(),
        {"blue_block_location": "right_floor", "holding": "none"},
        operators,
        registry,
        bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
        allow_intermediate_release=False,
    )
    assert detour.no_plan
    reasons = detour.diagnostics.rejected_by_operator["T7.deliver_to_black_table"]
    assert reasons["transition:intermediate_release_forbidden:black_table"] >= 1
