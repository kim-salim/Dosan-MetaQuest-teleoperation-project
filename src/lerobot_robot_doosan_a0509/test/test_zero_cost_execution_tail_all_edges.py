from __future__ import annotations

import json
from pathlib import Path

import pytest

pytest.importorskip("lerobot")

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
)
from lerobot_robot_doosan_a0509.interior_policy.planner import (
    CostBreakdown,
    Plan,
    PlanStep,
    operator_applicability,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    BridgeAdmissionMode,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
LEGACY_CATALOG_PATH = (
    REPOSITORY_ROOT
    / "config/interior_policy/a0509_t1_t6_operator_catalog_v1.json"
)
REGISTRY_PATH = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_flexible_level2_registry_2026-08-30"
    / "edge_registry_level2_v3.json"
)
INVENTORY_PATH = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_full_level2_coverage_2026-08-29"
    / "level1_edge_inventory.json"
)
ACQUISITION_OPERATORS = {
    "T1.acquire_from_black_table",
    "T2.acquire_from_floor",
    "T3.acquire_from_black_table",
    "T4.acquire_from_black_table",
    "T5.acquire_from_drawer",
    "T6.acquire_moving_block",
    "T7.acquire_from_drawer_top",
    "T8.acquire_top_block_from_stack",
}


def _two_step_plan(edge, catalog) -> Plan:
    source = catalog.by_id[edge["source_operator"]]
    successor = catalog.by_id[edge["successor_operator"]]
    state_before = WorldState(**dict(edge["witness_state_before"]))
    assert operator_applicability(state_before, source).valid
    state_after_source = apply_operator_effects(state_before, source)
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


def test_all_flexible_verified_edges_compile_with_global_execution_tail():
    catalog = load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        config_path=LEGACY_CATALOG_PATH,
        audit_raw_frames=False,
    )
    registry = EdgeRuntimeRegistry.load(REGISTRY_PATH)
    inventory = json.loads(INVENTORY_PATH.read_text(encoding="utf-8"))
    witnesses = {
        (edge["source_operator"], edge["successor_operator"]): edge
        for edge in inventory["edges"]
    }
    verified = [
        spec
        for spec in registry.edges
        if spec.admission_status == "flexible_verified"
    ]
    assert len(verified) == 119

    refreshed_tail_edge = (
        "T7.acquire_from_drawer_top",
        "T3.deliver_to_floor",
    )
    compiled_edges = 0
    stale_tail_edges_rejected = 0
    enabled_tail_edges = 0
    for index, spec in enumerate(verified):
        plan = _two_step_plan(witnesses[spec.key], catalog)
        original_cost = plan.total_cost
        is_legacy_tail_manifest = (
            spec.source_operator in ACQUISITION_OPERATORS
            and spec.source_operator != "T2.acquire_from_floor"
            and spec.key != refreshed_tail_edge
        )
        if is_legacy_tail_manifest:
            with pytest.raises(
                ValueError,
                match=(
                    "runtime source|execution-tail Bridge manifest lacks "
                    "runtime source support bank"
                ),
            ):
                compile_plan_to_multi_v2(
                    plan,
                    catalog,
                    registry,
                    composition_id=(
                        f"stale_flexible_execution_tail_{index:03d}"
                    ),
                    expected_bridge_admission_mode=(
                        BridgeAdmissionMode.FLEXIBLE_LEVEL2
                    ),
                )
            stale_tail_edges_rejected += 1
            continue

        compiled = compile_plan_to_multi_v2(
            plan,
            catalog,
            registry,
            composition_id=f"valid_flexible_execution_tail_{index:03d}",
            expected_bridge_admission_mode=(
                BridgeAdmissionMode.FLEXIBLE_LEVEL2
            ),
        )
        compiled_edges += 1
        transition = compiled.mapping["transitions"][0]
        tail = transition["source_execution_tail"]
        if spec.source_operator in ACQUISITION_OPERATORS:
            assert tail is not None
            assert tail["zero_symbolic_cost"] is True
            assert tail["fixed_z_minimum_used"] is False
            reference = transition["runtime_source_reference"]
            assert reference["segment"] == "S2"
            if spec.key == refreshed_tail_edge:
                assert reference["validated_runtime_source_bank"] is True
                assert reference["medoid_episode"] == 15
                successor_reference = transition[
                    "runtime_successor_reference"
                ]
                assert successor_reference["reference_phase"] == pytest.approx(
                    0.65
                )
                assert successor_reference["runtime_phase_gate"] is False
                assert successor_reference[
                    "validated_runtime_successor_bank"
                ] is True
            else:
                assert spec.source_operator == "T2.acquire_from_floor"
                assert reference["validated_runtime_source_bank"] is False
                assert reference["reference_phase"] == pytest.approx(0.5)
                assert reference["nominal_phase"] == pytest.approx(0.57)
                assert tail["collar_phase_low"] == pytest.approx(0.52)
                assert tail["commit_phase_low"] == pytest.approx(0.54)
                assert tail["commit_phase_high"] == pytest.approx(0.60)
                assert tail["deadline_phase"] == pytest.approx(0.65)
            enabled_tail_edges += 1
        else:
            assert tail is None, spec.key
        provenance = compiled.mapping["planner_provenance"]
        assert provenance["total_cost"] == pytest.approx(original_cost)
        assert provenance["dijkstra_operator_costs_unchanged"] is True

    # 52 verified acquisition-source edges existed in the prior registry.
    # Seven T2 edges already use the correct S2 segment (with an earlier
    # reference phase that can be joined causally), and the requested T7->T3
    # edge now has its strict 30-episode S2 medoid bank. The remaining 44
    # cross-segment legacy references must fail closed.
    assert enabled_tail_edges == 8
    assert stale_tail_edges_rejected == 44
    assert compiled_edges == 75
