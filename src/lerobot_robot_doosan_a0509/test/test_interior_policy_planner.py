"""Command-free feasibility tests for T1--T8 interior-policy composition."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from lerobot_robot_doosan_a0509.interior_policy.catalog import (
    LoadedOperatorCatalog,
    load_operator_catalog,
)
from lerobot_robot_doosan_a0509.interior_policy.contracts import (
    WorldState,
    apply_operator_effects,
)
from lerobot_robot_doosan_a0509.interior_policy.planner import (
    goal_satisfied,
    operator_applicability,
    uniform_cost_search,
)
from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (
    EdgeRuntimeRegistry,
    MultiV2CompilationConfig,
    compile_plan_to_multi_v2,
)
from lerobot_robot_doosan_a0509.interior_policy.simulator import replay_operators
from lerobot_robot_doosan_a0509.interior_policy.v2_adapter import (
    V2ShadowTransitionValidator,
    runtime_level,
)
from lerobot_robot_doosan_a0509.task_c_handoff.multi_stage import (
    MultiStagePlan,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
EXPECTED = (
    "T4.open_drawer",
    "T2.acquire_from_floor",
    "T4.deliver_to_drawer",
    "T4.close_drawer",
)


@pytest.fixture(scope="module")
def catalog() -> LoadedOperatorCatalog:
    # This is deliberately an integration fixture over the user's current
    # parquet/semantic/checkpoint tree.  It remains read-only and command-free.
    return load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        audit_raw_frames=True,
    )


def _controlled(catalog: LoadedOperatorCatalog):
    return uniform_cost_search(
        catalog.target.initial_state,
        catalog.target.goal,
        catalog.controlled_operators,
        cost_config=catalog.target.cost_config,
    )


def _full(catalog: LoadedOperatorCatalog, max_plans: int = 1):
    return uniform_cost_search(
        catalog.target.initial_state,
        catalog.target.goal,
        catalog.operators,
        cost_config=catalog.target.cost_config,
        max_plans=max_plans,
    )


def test_01_target_plan_exists_with_full_t1_t8_catalog(catalog):
    result = _full(catalog)
    assert not result.no_plan
    assert goal_satisfied(result.best_plan.final_state, catalog.target.goal)
    assert {item.policy_id for item in catalog.operators} == {
        "T1",
        "T2",
        "T3",
        "T4",
        "T5",
        "T6",
        "T7",
        "T8",
    }


def test_02_controlled_plan_is_exact_t4_t2_t4(catalog):
    plan = _controlled(catalog).best_plan
    assert plan is not None
    assert plan.operator_ids == EXPECTED
    assert plan.policy_sequence == ("T4", "T2", "T4")


def test_03_symbolic_replay_satisfies_partial_goal(catalog):
    plan = _controlled(catalog).best_plan
    replay = replay_operators(
        catalog.target.initial_state,
        tuple(step.operator for step in plan.steps),
        catalog.target.goal,
    )
    assert replay.goal_satisfied
    assert replay.final_state.drawer == "closed"
    assert replay.final_state.blue_block_location == "drawer"
    assert replay.final_state.holding == "none"
    assert replay.final_state.gripper == "open"


def test_04_forward_t4_reentry_after_t2_is_allowed(catalog):
    operators = catalog.by_id
    state = apply_operator_effects(
        catalog.target.initial_state, operators["T4.open_drawer"]
    )
    state = apply_operator_effects(state, operators["T2.acquire_from_floor"])
    status = operator_applicability(state, operators["T4.deliver_to_drawer"])
    assert status.valid
    state = apply_operator_effects(state, operators["T4.deliver_to_drawer"])
    assert state.cursor_t4 == 40


def test_05_backward_same_policy_reentry_is_rejected(catalog):
    operators = catalog.by_id
    state = catalog.target.initial_state
    for operator_id in EXPECTED[:3]:
        state = apply_operator_effects(state, operators[operator_id])
    status = operator_applicability(state, operators["T4.open_drawer"])
    assert not status.valid
    assert any(reason.startswith("forward_only:") for reason in status.reasons)


def test_06_drawer_must_be_open_before_delivery(catalog):
    state = WorldState(
        drawer="closed",
        white_container="closed",
        blue_block_location="held",
        holding="blue_block",
        gripper="closed",
        contact_mode="free_transport",
    )
    status = operator_applicability(
        state, catalog.by_id["T4.deliver_to_drawer"]
    )
    assert not status.valid
    assert any("drawer:expected=open:actual=closed" in item for item in status.reasons)


def test_07_cannot_open_drawer_while_holding_block(catalog):
    held = apply_operator_effects(
        catalog.target.initial_state, catalog.by_id["T2.acquire_from_floor"]
    )
    status = operator_applicability(held, catalog.by_id["T4.open_drawer"])
    assert not status.valid
    assert any("holding:expected=none:actual=blue_block" in item for item in status.reasons)


def test_08_t2_acquire_is_prefix_not_full_delivery(catalog):
    assert (
        catalog.by_id["T2.acquire_from_floor"].precondition_map[
            "blue_block_location"
        ]
        == "left_floor"
    )
    acquired = apply_operator_effects(
        catalog.target.initial_state, catalog.by_id["T2.acquire_from_floor"]
    )
    assert acquired.blue_block_location == "held"
    assert acquired.blue_block_location != "black_table"
    evidence = catalog.by_id["T2.acquire_from_floor"].evidence
    assert evidence.exit_segment == "S2"
    assert evidence.exit_phase == pytest.approx(0.5)
    assert evidence.certified_source_boundary_manifest is not None
    assert catalog.audit.operators[
        "T2.acquire_from_floor"
    ].certified_source_boundary_verified


def test_09_t3_direction_is_black_table_to_right_floor(catalog):
    acquire = catalog.by_id["T3.acquire_from_black_table"]
    deliver = catalog.by_id["T3.deliver_to_floor"]
    assert acquire.precondition_map["blue_block_location"] == "black_table"
    assert acquire.effect_map["blue_block_location"] == "held"
    assert deliver.precondition_map["blue_block_location"] == "held"
    assert deliver.effect_map["blue_block_location"] == "right_floor"
    assert (
        catalog.by_id["T2.deliver_to_black_table"].effect_map[
            "blue_block_location"
        ]
        == "black_table"
    )


def test_10_irrelevant_tasks_are_not_in_best_target_plan(catalog):
    plan = _full(catalog).best_plan
    assert plan.operator_ids == EXPECTED
    assert all(
        not item.startswith(("T1.", "T3.", "T6.", "T7.", "T8."))
        for item in plan.operator_ids
    )


def test_11_no_plan_without_any_drawer_delivery_operator(catalog):
    operators = tuple(
        item for item in catalog.operators if item.id != "T4.deliver_to_drawer"
    )
    result = uniform_cost_search(
        catalog.target.initial_state,
        catalog.target.goal,
        operators,
        cost_config=catalog.target.cost_config,
    )
    assert result.no_plan


def test_12_t5_drawer_open_alternative_exists_in_full_catalog(catalog):
    result = _full(catalog, max_plans=20)
    paths = {plan.operator_ids for plan in result.plans}
    assert EXPECTED in paths
    assert (
        "T5.open_drawer",
        "T2.acquire_from_floor",
        "T4.deliver_to_drawer",
        "T4.close_drawer",
    ) in paths


def test_13_frame_axiom_preserves_left_floor_block_while_opening_drawer(catalog):
    opened = apply_operator_effects(
        catalog.target.initial_state, catalog.by_id["T4.open_drawer"]
    )
    assert opened.drawer == "open"
    assert opened.blue_block_location == "left_floor"
    assert opened.white_container == "closed"


def test_14_unknown_is_not_a_precondition_wildcard(catalog):
    unknown = WorldState(
        drawer="unknown",
        white_container="closed",
        blue_block_location="left_floor",
        holding="none",
        gripper="open",
        contact_mode="free_space",
    )
    status = operator_applicability(unknown, catalog.by_id["T4.open_drawer"])
    assert not status.valid
    assert any("drawer:expected=closed:actual=unknown" in item for item in status.reasons)


def test_15_actual_t1_t8_artifacts_datasets_and_checkpoints_are_audited(catalog):
    assert tuple(sorted(catalog.audit.tasks)) == (
        "T1",
        "T2",
        "T3",
        "T4",
        "T5",
        "T6",
        "T7",
        "T8",
    )
    expected_episodes = {
        "T1": 30,
        "T2": 31,
        "T3": 30,
        "T4": 30,
        "T5": 30,
        "T6": 30,
        "T7": 30,
        "T8": 30,
    }
    reference_input = None
    for task_id, audit in catalog.audit.tasks.items():
        assert audit.episode_count == expected_episodes[task_id]
        assert audit.fps == 30
        assert audit.checkpoint_type == "act"
        assert audit.chunk_size == 100
        assert audit.n_action_steps == 100
        assert audit.action_shape == (7,)
        assert Path(audit.checkpoint, "model.safetensors").is_file()
        assert audit.semantic_sha256_verified is True
        assert audit.phase_support_sha256_verified is True
        assert audit.semantic_robot_executable is False
        assert audit.representative_episode_selected is False
        if reference_input is None:
            reference_input = audit.input_contract
        else:
            assert audit.input_contract == reference_input


def test_16_floor_overrides_are_explicit_and_source_artifacts_unchanged(catalog):
    overrides = {item.override_id: item for item in catalog.audit.physical_setup_overrides}
    assert overrides["T2_SOURCE_REGION_IS_LEFT_FLOOR"].planner_value == "left_floor"
    assert overrides["T3_OFF_BLACK_TABLE_IS_RIGHT_FLOOR"].planner_value == "right_floor"
    assert not overrides["T2_SOURCE_REGION_IS_LEFT_FLOOR"].mutates_source_artifact
    t2_audit = catalog.audit.operators["T2.acquire_from_floor"]
    t3_audit = catalog.audit.operators["T3.deliver_to_floor"]
    assert t2_audit.artifact_entry_state["blue_block"] == "source_region"
    assert t3_audit.artifact_exit_state_at_selected_boundary["blue_block"] == "off_black_table"


def test_17_raw_parquet_event_intervals_are_recomputed(catalog):
    assert catalog.audit.full_raw_frame_audit
    t4_open = catalog.audit.operators["T4.open_drawer"].frame_interval
    t2_acquire = catalog.audit.operators["T2.acquire_from_floor"].frame_interval
    t4_close = catalog.audit.operators["T4.close_drawer"].frame_interval
    assert t4_open.episode_count == 30
    assert t4_open.start_frame_min == 0
    assert 395 <= t4_open.end_frame_median <= 635
    assert t2_acquire.episode_count == 31
    assert t2_acquire.end_frame_median > 339
    assert t4_close.end_frame_min == 1799


def test_18_cost_is_base_plus_two_policy_switches(catalog):
    plan = _controlled(catalog).best_plan
    assert plan.total_cost == pytest.approx(4.5)
    assert [step.cost.base_cost for step in plan.steps] == [1.0] * 4
    assert [step.cost.policy_switch_penalty for step in plan.steps] == [
        0.0,
        0.25,
        0.25,
        0.0,
    ]


def test_19_current_v2_edges_have_exact_manifests_and_command_free_shadows(catalog):
    plan = _controlled(catalog).best_plan
    validator = V2ShadowTransitionValidator(catalog)
    edges = validator.validate_plan(plan)
    assert len(edges) == 2
    assert [edge.symbolic.transition_type for edge in edges] == [
        "empty_gripper_free_space_reposition",
        "held_blue_block_free_transport",
    ]
    assert all(edge.symbolic.valid for edge in edges)
    assert all(edge.structural_runtime_mapping for edge in edges)
    assert all(edge.exact_episode_manifest_ready for edge in edges)
    assert all(edge.policy_shadow_ready for edge in edges)
    assert all(edge.runtime_contract_compatible for edge in edges)
    assert all(edge.matching_episode_manifests for edge in edges)
    assert all(edge.matching_policy_shadow_reports for edge in edges)
    assert runtime_level(plan, edges) == (
        "LEVEL_2_RUNTIME_CONTRACT_COMPATIBLE"
    )


def test_20_v2_code_audit_maps_existing_runtime_without_importing_or_running_it(catalog):
    validator = V2ShadowTransitionValidator(catalog)
    assert validator.code_audit.complete
    mappings = validator.code_audit.mappings
    assert mappings["A_actual_ack_snapshot"].endswith("._prepare_bridge_commit")
    assert mappings["async_successor"] == "AsyncSuccessorController"
    assert mappings["fresh_B_prefix_admission"].endswith(".evaluate_prefix")
    assert "TaskCMultiLiveV2Strategy" in mappings["repeated_policy_visits"]


def test_21_catalog_does_not_claim_semantic_artifacts_are_robot_executable(catalog):
    for task in catalog.audit.tasks.values():
        assert task.semantic_robot_executable is False
    # Derived edge manifests and command-free policy shadows do not retroactively
    # certify the source semantic catalog for physical execution.  No test in
    # this module opens ROS or a robot connection.
    assert all(
        not value.certified_source_boundary_verified
        for key, value in catalog.audit.operators.items()
        if key != "T2.acquire_from_floor"
    )


def _compiler_manifest(
    path: Path,
    *,
    source,
    successor,
    source_gripper: str,
    source_holding: str,
    transport_floor_mm=None,
) -> None:
    value = {
        "schema_version": "a0509.task_c_handoff_episode.v2",
        "composition_id": "compiler_test",
        "handoff_id": f"h_{source.policy_id.lower()}_{successor.policy_id.lower()}",
        "source": {
            "task": source.policy_id,
            "segment": source.evidence.exit_segment,
            "phase": source.evidence.exit_phase,
            "support_episode": 0,
            "support_frame": 10,
            "nominal_pose_mm_deg": [100, 0, 400, 0, 150, 0],
            "nominal_velocity_mm_s": [5, 0, 0],
            "support_radius_mm": 1000,
            "semantic": {
                "gripper_state": source_gripper,
                "held_object": source_holding,
                "contact_mode": source.exit_contact_mode,
            },
        },
        "successor": {
            "task": successor.policy_id,
            "segment": successor.evidence.entry_segment,
            "phase": successor.evidence.entry_phase,
            "support_episode": 0,
            "support_frame": 20,
            "nominal_pose_mm_deg": [150, 0, 410, 0, 150, 0],
            "nominal_velocity_mm_s": [2, 0, 0],
            "support_radius_mm": 1000,
            "semantic": {
                "gripper_state": source_gripper,
                "held_object": source_holding,
                "contact_mode": successor.entry_contact_mode,
            },
        },
        "bridge": {
            "duration_s": 4.0,
            "nominal_length_mm": 50.0,
            "transport_floor_mm": transport_floor_mm,
        },
        "generation": {"method": "manual_reviewed"},
        "validation": {
            "hard_filter_passed": True,
            "semantic_authority": "external_planner",
            "robot_executable": False,
            "dry_run_only": True,
            "ik_checked": False,
            "collision_checked": False,
        },
    }
    path.write_text(json.dumps(value), encoding="utf-8")


def _compiler_shadow(
    path: Path,
    *,
    handoff_id: str,
    takeover_success: bool = True,
    transport_floor_mm=None,
) -> None:
    value = {
        "schema_version": "a0509.task_c_handoff_v2_policy_shadow.v1",
        "handoff_id": handoff_id,
        "robot_commands_published": 0,
        "live_enabled": False,
        "mux_selected": False,
        "terminal_state": "RUN_B" if takeover_success else "FAILED_HOLD",
        "takeover_success": takeover_success,
        "fallback_required": False,
        "prefix_admission": {"valid": takeover_success},
        "control_timing": {"deadline_miss_count": 0},
        "bridge": {"transport_floor_mm": transport_floor_mm},
    }
    path.write_text(json.dumps(value), encoding="utf-8")


def _compiler_registry(
    tmp_path: Path,
    catalog: LoadedOperatorCatalog,
    *,
    include_second: bool = True,
    second_transport_floor_mm=None,
) -> EdgeRuntimeRegistry:
    first_source = catalog.by_id["T4.open_drawer"]
    first_successor = catalog.by_id["T2.acquire_from_floor"]
    second_source = first_successor
    second_successor = catalog.by_id["T4.deliver_to_drawer"]
    first_path = tmp_path / "t4_to_t2.json"
    second_path = tmp_path / "t2_to_t4.json"
    _compiler_manifest(
        first_path,
        source=first_source,
        successor=first_successor,
        source_gripper="open",
        source_holding="none",
    )
    _compiler_manifest(
        second_path,
        source=second_source,
        successor=second_successor,
        source_gripper="closed",
        source_holding="blue_block",
        transport_floor_mm=second_transport_floor_mm,
    )
    first_shadow = tmp_path / "shadow_t4_to_t2.json"
    second_shadow = tmp_path / "shadow_t2_to_t4.json"
    _compiler_shadow(
        first_shadow,
        handoff_id="h_t4_t2",
    )
    _compiler_shadow(
        second_shadow,
        handoff_id="h_t2_t4",
        transport_floor_mm=second_transport_floor_mm,
    )
    edges = [
        {
            "source_operator": first_source.id,
            "successor_operator": first_successor.id,
            "handoff_manifest": first_path.name,
            "policy_shadow": first_shadow.name,
            "gripper_event": "closed_then_open",
        }
    ]
    if include_second:
        edges.append(
            {
                "source_operator": second_source.id,
                "successor_operator": second_successor.id,
                "handoff_manifest": second_path.name,
                "policy_shadow": second_shadow.name,
                "gripper_event": "open_then_closed",
            }
        )
    registry_path = tmp_path / "edge_registry.json"
    registry_path.write_text(
        json.dumps(
            {
                "schema_version": (
                    "a0509.interior_policy_v2_edge_registry.v1"
                ),
                "edges": edges,
            }
        ),
        encoding="utf-8",
    )
    return EdgeRuntimeRegistry.load(registry_path)


def test_22_dijkstra_plan_compiles_to_three_multi_v2_visits(catalog, tmp_path):
    plan = _controlled(catalog).best_plan
    compiled = compile_plan_to_multi_v2(
        plan,
        catalog,
        _compiler_registry(tmp_path, catalog),
        composition_id="t4_t2_t4_compiler_test",
        config=MultiV2CompilationConfig(final_inference_steps=1800),
    )
    assert [visit.policy_id for visit in compiled.visits] == ["T4", "T2", "T4"]
    assert compiled.visits[-1].operator_ids == (
        "T4.deliver_to_drawer",
        "T4.close_drawer",
    )
    output = compiled.write_json(tmp_path / "compiled_plan.json")
    loaded = MultiStagePlan.load(output)
    assert [stage.exit_authority for stage in loaded.stages] == [
        "phase_supervisor",
        "phase_supervisor",
        "policy_inference_end",
    ]
    assert loaded.stages[0].phase_supervisor.gripper_event_mode.value == (
        "closed_then_open"
    )
    assert loaded.stages[1].phase_supervisor.gripper_event_mode.value == (
        "open_then_closed"
    )
    assert loaded.stages[-1].inference_end_steps == 1800
    assert compiled.mapping["planner_provenance"]["z_minimum_enabled"] is False


def test_23_compiler_fails_closed_when_a_dijkstra_edge_is_unreviewed(
    catalog, tmp_path
):
    plan = _controlled(catalog).best_plan
    registry = _compiler_registry(tmp_path, catalog, include_second=False)
    with pytest.raises(ValueError, match="lacks a reviewed V2 edge"):
        compile_plan_to_multi_v2(
            plan,
            catalog,
            registry,
            composition_id="missing_edge",
        )


def test_24_compiler_preserves_verified_transport_floor_constraint(
    catalog, tmp_path
):
    plan = _controlled(catalog).best_plan
    registry = _compiler_registry(
        tmp_path,
        catalog,
        second_transport_floor_mm=300.0,
    )
    compiled = compile_plan_to_multi_v2(
        plan,
        catalog,
        registry,
        composition_id="z_floor_verified",
    )
    provenance = compiled.mapping["planner_provenance"]
    assert provenance["z_minimum_enabled"] is True
    assert provenance["zero_cost_execution_tail"][
        "fixed_z_minimum_used"
    ] is True
    assert provenance["bridge_transport_floors"] == [
        {
            "handoff_id": "h_t2_t4",
            "source_operator": "T2.acquire_from_floor",
            "successor_operator": "T4.deliver_to_drawer",
            "transport_floor_mm": 300.0,
            "source_endpoint_z_mm": 400.0,
            "successor_endpoint_z_mm": 410.0,
        }
    ]
    assert compiled.mapping["transitions"][1][
        "transport_floor_mm"
    ] == pytest.approx(300.0)


def test_24b_compiler_rejects_floor_above_a_bridge_endpoint(
    catalog, tmp_path
):
    plan = _controlled(catalog).best_plan
    registry = _compiler_registry(
        tmp_path,
        catalog,
        second_transport_floor_mm=450.0,
    )
    with pytest.raises(ValueError, match="exceeds a Bridge endpoint"):
        compile_plan_to_multi_v2(
            plan,
            catalog,
            registry,
            composition_id="z_floor_impossible",
        )


def test_25_compiler_rejects_failed_policy_shadow(catalog, tmp_path):
    plan = _controlled(catalog).best_plan
    registry = _compiler_registry(tmp_path, catalog)
    _compiler_shadow(
        tmp_path / "shadow_t2_to_t4.json",
        handoff_id="h_t2_t4",
        takeover_success=False,
    )
    with pytest.raises(ValueError, match="policy shadow terminal_state"):
        compile_plan_to_multi_v2(
            plan,
            catalog,
            registry,
            composition_id="failed_shadow",
        )


def test_26_t7_direction_is_drawer_top_to_black_table(catalog):
    acquire = catalog.by_id["T7.acquire_from_drawer_top"]
    deliver = catalog.by_id["T7.deliver_to_black_table"]
    assert acquire.precondition_map["blue_block_location"] == "drawer_top"
    assert acquire.effect_map["blue_block_location"] == "held"
    assert deliver.precondition_map["blue_block_location"] == "held"
    assert deliver.effect_map["blue_block_location"] == "black_table"

    initial = WorldState(
        drawer="closed",
        white_container="closed",
        blue_block_location="drawer_top",
        holding="none",
        gripper="open",
        contact_mode="free_space",
    )
    result = uniform_cost_search(
        initial,
        {
            "blue_block_location": "black_table",
            "holding": "none",
            "gripper": "open",
        },
        (acquire, deliver),
    )
    assert result.best_plan.operator_ids == (
        "T7.acquire_from_drawer_top",
        "T7.deliver_to_black_table",
    )
    assert result.best_plan.final_state.cursor_t7 == 30


def test_27_t8_is_symbolic_inverse_of_t6_stack(catalog):
    acquire = catalog.by_id["T8.acquire_top_block_from_stack"]
    deliver = catalog.by_id["T8.deliver_unstacked_to_black_table"]
    assert acquire.precondition_map["blue_block_location"] == "stacked"
    assert acquire.precondition_map["stack"] == "assembled"
    assert acquire.effect_map["blue_block_location"] == "held"
    assert acquire.effect_map["stack"] == "unassembled"
    assert deliver.effect_map["blue_block_location"] == "black_table"
    assert deliver.effect_map["stack"] == "unassembled"

    initial = WorldState(
        drawer="closed",
        white_container="closed",
        blue_block_location="stacked",
        holding="none",
        gripper="open",
        contact_mode="free_space",
        support_blue_block_location="black_table",
        stack="assembled",
    )
    result = uniform_cost_search(
        initial,
        {
            "blue_block_location": "black_table",
            "support_blue_block_location": "black_table",
            "stack": "unassembled",
            "holding": "none",
            "gripper": "open",
        },
        (acquire, deliver),
    )
    assert result.best_plan.operator_ids == (
        "T8.acquire_top_block_from_stack",
        "T8.deliver_unstacked_to_black_table",
    )
    assert result.best_plan.final_state.cursor_t8 == 30


def test_28_t7_t8_raw_intervals_and_artifact_abstractions_are_audited(catalog):
    t7 = catalog.audit.operators["T7.acquire_from_drawer_top"]
    t8 = catalog.audit.operators["T8.acquire_top_block_from_stack"]
    assert t7.frame_interval.episode_count == 30
    assert t8.frame_interval.episode_count == 30
    assert t7.artifact_entry_state["blue_block"] == "on_top_of_drawer"
    assert t8.artifact_entry_state["moving_blue_block"] == (
        "on_top_of_support_blue_block"
    )
    assert t8.artifact_exit_state_at_selected_boundary["stack"] == (
        "disassembling"
    )
    assert not t7.certified_source_boundary_verified
    assert not t8.certified_source_boundary_verified


def test_29_t6_then_t8_unseen_stack_roundtrip_is_symbolically_plannable(catalog):
    operator_ids = (
        "T6.acquire_moving_block",
        "T6.stack_on_support_block",
        "T8.acquire_top_block_from_stack",
        "T8.deliver_unstacked_to_black_table",
    )
    initial = WorldState(
        drawer="closed",
        white_container="closed",
        blue_block_location="moving_source",
        holding="none",
        gripper="open",
        contact_mode="free_space",
        support_blue_block_location="black_table",
        stack="unassembled",
    )
    final_goal = {
        "blue_block_location": "black_table",
        "support_blue_block_location": "black_table",
        "stack": "unassembled",
        "holding": "none",
        "gripper": "open",
    }

    # With only the final predicates, Dijkstra must avoid the needless stack
    # round trip and choose the legitimately shorter cross-policy route.
    direct = uniform_cost_search(
        initial,
        final_goal,
        tuple(catalog.by_id[item] for item in operator_ids),
        cost_config=catalog.target.cost_config,
    )
    assert direct.best_plan.operator_ids == (
        "T6.acquire_moving_block",
        "T8.deliver_unstacked_to_black_table",
    )

    # The temporal instruction "stack, then unstack" is represented as two
    # ordered symbolic goals. Each stage is still a deterministic Dijkstra
    # search over the same immutable WorldState/operator contracts.
    stack_goal = {
        "blue_block_location": "stacked",
        "support_blue_block_location": "black_table",
        "stack": "assembled",
        "holding": "none",
        "gripper": "open",
    }
    stack_result = uniform_cost_search(
        initial,
        stack_goal,
        tuple(catalog.by_id[item] for item in operator_ids[:2]),
        cost_config=catalog.target.cost_config,
    )
    unstack_result = uniform_cost_search(
        stack_result.best_plan.final_state,
        final_goal,
        tuple(catalog.by_id[item] for item in operator_ids[2:]),
        cost_config=catalog.target.cost_config,
    )
    combined = (
        stack_result.best_plan.operator_ids
        + unstack_result.best_plan.operator_ids
    )
    assert combined == operator_ids
    assert stack_result.best_plan.policy_sequence == ("T6",)
    assert unstack_result.best_plan.policy_sequence == ("T8",)
    assert (
        stack_result.best_plan.total_cost
        + unstack_result.best_plan.total_cost
    ) == pytest.approx(4.25)
    assert goal_satisfied(unstack_result.best_plan.final_state, final_goal)


def _minimal_state_before(operator):
    values = {
        "drawer": "unknown",
        "white_container": "unknown",
        "blue_block_location": "unknown",
        "holding": "unknown",
        "gripper": "unknown",
        "contact_mode": "unknown",
    }
    values.update(operator.precondition_map)
    return WorldState(**values)


def test_30_new_t7_t8_edges_reach_level2_from_real_command_free_evidence(catalog):
    validator = V2ShadowTransitionValidator(catalog)
    pairs = (
        ("T5.acquire_from_drawer", "T7.deliver_to_black_table"),
        ("T7.deliver_to_black_table", "T3.acquire_from_black_table"),
        ("T8.acquire_top_block_from_stack", "T2.deliver_to_black_table"),
        (
            "T8.deliver_unstacked_to_black_table",
            "T3.acquire_from_black_table",
        ),
    )
    for source_id, successor_id in pairs:
        source = catalog.by_id[source_id]
        successor = catalog.by_id[successor_id]
        state_after_source = apply_operator_effects(
            _minimal_state_before(source), source
        )
        result = validator.validate(source, successor, state_after_source)
        assert result.symbolic.valid
        assert result.structural_runtime_mapping
        assert result.exact_episode_manifest_ready
        assert result.policy_shadow_ready
        assert result.runtime_contract_compatible
        assert result.matching_episode_manifests
        assert result.matching_policy_shadow_reports
        assert not result.physical_validation_performed


def test_31_failed_t7_t8_edges_remain_level1_and_fail_closed(catalog):
    validator = V2ShadowTransitionValidator(catalog)
    cases = (
        (
            "T7.acquire_from_drawer_top",
            "T2.deliver_to_black_table",
            True,
            False,
        ),
        (
            "T6.stack_on_support_block",
            "T8.acquire_top_block_from_stack",
            # Tangent regularization now produces an exact Bridge manifest,
            # but fresh ACT-B prefix shadow admission still fails.  The edge
            # therefore remains Level-1 and fail-closed.
            True,
            False,
        ),
    )
    for source_id, successor_id, has_manifest, has_shadow in cases:
        source = catalog.by_id[source_id]
        successor = catalog.by_id[successor_id]
        state_after_source = apply_operator_effects(
            _minimal_state_before(source), source
        )
        result = validator.validate(source, successor, state_after_source)
        assert result.symbolic.valid
        assert result.exact_episode_manifest_ready is has_manifest
        assert result.policy_shadow_ready is has_shadow
        assert not result.runtime_contract_compatible
        assert result.missing_runtime_work


def test_32_unified_registry_contains_only_six_command_free_level2_edges(catalog):
    path = (
        REPOSITORY_ROOT
        / "docs/artifacts/t7_t8_level2_edge_validation_2026-08-29"
        / "edge_registry_all_level2.json"
    )
    registry = EdgeRuntimeRegistry.load(path)
    assert set(registry.by_key) == {
        ("T4.open_drawer", "T2.acquire_from_floor"),
        ("T2.acquire_from_floor", "T4.deliver_to_drawer"),
        ("T5.acquire_from_drawer", "T7.deliver_to_black_table"),
        ("T7.deliver_to_black_table", "T3.acquire_from_black_table"),
        ("T8.acquire_top_block_from_stack", "T2.deliver_to_black_table"),
        (
            "T8.deliver_unstacked_to_black_table",
            "T3.acquire_from_black_table",
        ),
    }
    for edge in registry.edges:
        assert edge.handoff_manifest_path.is_file()
        assert edge.policy_shadow_path.is_file()


@pytest.mark.parametrize(
    ("initial", "goal", "operator_ids", "expected_policies"),
    (
        (
            WorldState(
                drawer="open",
                white_container="unknown",
                blue_block_location="drawer",
                holding="none",
                gripper="open",
                contact_mode="free_space",
            ),
            {
                "blue_block_location": "black_table",
                "holding": "none",
                "gripper": "open",
                "contact_mode": "free_space",
            },
            (
                "T5.acquire_from_drawer",
                "T7.deliver_to_black_table",
            ),
            ("T5", "T7"),
        ),
        (
            WorldState(
                drawer="unknown",
                white_container="unknown",
                blue_block_location="stacked",
                holding="none",
                gripper="open",
                contact_mode="free_space",
                support_blue_block_location="black_table",
                stack="assembled",
            ),
            {
                "blue_block_location": "black_table",
                "holding": "none",
                "gripper": "open",
                "contact_mode": "free_space",
                "support_blue_block_location": "black_table",
                "stack": "unassembled",
            },
            (
                "T8.acquire_top_block_from_stack",
                "T2.deliver_to_black_table",
            ),
            ("T8", "T2"),
        ),
    ),
)
def test_33_new_level2_plans_compile_to_dynamic_multi_v2(
    catalog,
    initial,
    goal,
    operator_ids,
    expected_policies,
):
    registry = EdgeRuntimeRegistry.load(
        REPOSITORY_ROOT
        / "docs/artifacts/t7_t8_level2_edge_validation_2026-08-29"
        / "edge_registry_all_level2.json"
    )
    result = uniform_cost_search(
        initial,
        goal,
        tuple(catalog.by_id[item] for item in operator_ids),
        cost_config=catalog.target.cost_config,
    )
    assert result.best_plan.operator_ids == operator_ids
    compiled = compile_plan_to_multi_v2(
        result.best_plan,
        catalog,
        registry,
        composition_id="new_t7_t8_level2_compiler_test",
    )
    assert tuple(visit.policy_id for visit in compiled.visits) == expected_policies
    assert len(compiled.edge_specs) == 1
    assert compiled.edge_specs[0].key == (
        operator_ids[0],
        operator_ids[1],
    )
    assert (
        compiled.mapping["planner_provenance"][
            "physical_validation_performed"
        ]
        is False
    )
