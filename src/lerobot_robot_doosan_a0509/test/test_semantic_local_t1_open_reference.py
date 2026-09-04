from __future__ import annotations

import json
from pathlib import Path

import pytest

pytest.importorskip("lerobot")

from lerobot_robot_doosan_a0509.interior_policy.catalog import (
    load_operator_catalog,
)
from lerobot_robot_doosan_a0509.interior_policy.contracts import WorldState
from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (
    EdgeRuntimeRegistry,
    compile_plan_to_multi_v2,
    uniform_cost_search_level2,
)
from lerobot_robot_doosan_a0509.interior_policy.web_runtime_manifest import (
    build_web_runtime_support_manifest,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    BridgeAdmissionMode,
    EpisodeHandoffManifest,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
ARTIFACT_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/"
    "t1_t8_spatial_floor_level2_semantic_local_all_2026-09-03"
)
REGISTRY_PATH = ARTIFACT_ROOT / "edge_registry_level2_spatial_v7.json"
FLOOR400_REGISTRY_PATH = ARTIFACT_ROOT / "edge_registry_level2_spatial_v8.json"
DRAWER_REGISTRY_PATH = ARTIFACT_ROOT / "edge_registry_level2_spatial_v9.json"
DRAWER_TOP_REGISTRY_PATH = (
    ARTIFACT_ROOT / "edge_registry_level2_spatial_v10.json"
)
DRAWER_TOP_TAIL_REGISTRY_PATH = (
    ARTIFACT_ROOT / "edge_registry_level2_spatial_v11.json"
)
CATALOG_PATH = (
    REPOSITORY_ROOT
    / "config/interior_policy/a0509_t1_t8_operator_catalog_spatial_v2.json"
)
RUNTIME_TEMPLATE_PATH = (
    REPOSITORY_ROOT
    / "docs/artifacts/task_c_t2_t3_floor_to_floor_v2_2026-08-25"
    / "v2_fail_closed_support_runtime_manifest_t2_t3.json"
)
EXPECTED = {
    "T2.acquire_from_floor": 0.96,
    "T7.acquire_from_drawer_top": 0.94,
}


@pytest.fixture(scope="module")
def catalog():
    return load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        config_path=CATALOG_PATH,
        audit_raw_frames=False,
    )


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _resolved(value: str, base: Path) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (base / path).resolve()


@pytest.mark.parametrize(("successor", "phase"), EXPECTED.items())
def test_t1_open_edges_preserve_o1_and_join_before_acquisition_c1(
    successor: str,
    phase: float,
):
    registry = _load(REGISTRY_PATH)
    edge = next(
        item
        for item in registry["edges"]
        if item["source_operator"] == "T1.open_white_container"
        and item["successor_operator"] == successor
    )
    assert edge["admission_status"] == "flexible_verified"
    assert edge["validation_method"] == (
        "exact_semantic_exit_s2_"
        "semantic_local_interior_dynamic_future_join"
    )
    assert edge["evidence"]["source_reference_mode"] == "exact_semantic_exit"
    assert edge["evidence"]["mandatory_source_semantic_exit_preserved"] is True
    assert edge["evidence"]["semantic_local_refresh"] == (
        "promoted_exact_semantic_exit"
    )
    assert "semantic_local_not_applicable_reason" not in edge["evidence"]

    manifest_path = _resolved(edge["handoff_manifest"], ARTIFACT_ROOT)
    shadow_path = _resolved(edge["policy_shadow"], ARTIFACT_ROOT)
    manifest = EpisodeHandoffManifest.load(manifest_path)
    shadow = _load(shadow_path)
    assert manifest.source.task == "T1"
    assert manifest.source.segment == "S2"
    assert manifest.source.phase == pytest.approx(1.0)
    assert manifest.source.semantic.gripper_state == "open"
    assert manifest.source.semantic.held_object == "none"
    assert manifest.runtime_source_bank_path is None

    assert manifest.successor.segment == "S1"
    assert manifest.successor.phase == pytest.approx(phase)
    assert manifest.successor.phase < 1.0
    assert manifest.successor.semantic.gripper_state == "open"
    assert manifest.successor.semantic.held_object == "none"
    assert manifest.runtime_successor_bank_path is not None
    assert manifest.successor_interior_path_margin_mm == pytest.approx(15.0)
    assert shadow["handoff_id"] == manifest.handoff_id
    assert shadow["terminal_state"] == "RUN_B"
    assert shadow["takeover_success"] is True
    assert shadow["robot_commands_published"] == 0
    assert shadow["live_enabled"] is False
    assert shadow["mux_selected"] is False


@pytest.mark.parametrize(
    ("location", "acquire_operator"),
    (
        ("left_floor", "T2.acquire_from_floor"),
        ("drawer_top", "T7.acquire_from_drawer_top"),
    ),
)
def test_v7_compiles_t1_open_with_prearm_rearm_and_semantic_local_join(
    catalog,
    location: str,
    acquire_operator: str,
):
    registry = EdgeRuntimeRegistry.load(REGISTRY_PATH)
    operators = (
        catalog.by_id["T1.open_white_container"],
        catalog.by_id[acquire_operator],
        catalog.by_id["T1.deliver_to_white_container"],
    )
    initial = WorldState(
        drawer="closed",
        white_container="closed",
        blue_block_location=location,
        holding="none",
        gripper="open",
        contact_mode="free_space",
    )
    result = uniform_cost_search_level2(
        initial,
        {
            "white_container": "open",
            "blue_block_location": "white_container",
            "holding": "none",
            "gripper": "open",
        },
        operators,
        registry,
        bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
        allow_intermediate_release=False,
    )
    assert result.best_plan is not None
    assert result.best_plan.operator_ids == (
        "T1.open_white_container",
        acquire_operator,
        "T1.deliver_to_white_container",
    )

    compiled = compile_plan_to_multi_v2(
        result.best_plan,
        catalog,
        registry,
        composition_id=f"test_t1_open_{acquire_operator}",
        expected_bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
    )
    first_stage = compiled.mapping["stages"][0]
    first_transition = compiled.mapping["transitions"][0]
    assert first_stage["phase_supervisor"][
        "rearm_closed_then_open_at_prearm"
    ] is True
    assert first_stage["phase_supervisor"]["gripper_event"] == "closed_then_open"
    assert first_transition["source_execution_tail"] is None
    assert first_transition["runtime_source_reference"] is None
    assert first_transition["runtime_successor_reference"]["segment"] == "S1"
    assert first_transition["runtime_successor_reference"][
        "runtime_phase_gate"
    ] is False


@pytest.mark.parametrize(
    (
        "registry_path",
        "source_location",
        "acquire_operator",
        "successor_task",
        "successor_phase",
    ),
    (
        (
            DRAWER_REGISTRY_PATH,
            "left_floor",
            "T2.acquire_from_floor",
            "T2",
            0.96,
        ),
        (
            DRAWER_TOP_REGISTRY_PATH,
            "drawer_top",
            "T7.acquire_from_drawer_top",
            "T7",
            0.94,
        ),
    ),
)
def test_v9_v10_compile_drawer_compositions_with_t4_exact_exit_rearm(
    catalog,
    registry_path: Path,
    source_location: str,
    acquire_operator: str,
    successor_task: str,
    successor_phase: float,
):
    raw_registry = _load(registry_path)
    edge = next(
        item
        for item in raw_registry["edges"]
        if item["source_operator"] == "T4.open_drawer"
        and item["successor_operator"] == acquire_operator
    )
    assert edge["validation_method"] == (
        "exact_semantic_exit_s2_"
        "semantic_local_interior_dynamic_future_join"
    )
    assert edge["evidence"]["source_reference_mode"] == "exact_semantic_exit"

    manifest_path = _resolved(edge["handoff_manifest"], ARTIFACT_ROOT)
    shadow_path = _resolved(edge["policy_shadow"], ARTIFACT_ROOT)
    manifest = EpisodeHandoffManifest.load(manifest_path)
    shadow = _load(shadow_path)
    assert manifest.source.task == "T4"
    assert manifest.source.segment == "S2"
    assert manifest.source.phase == pytest.approx(1.0)
    assert manifest.successor.task == successor_task
    assert manifest.successor.segment == "S1"
    assert manifest.successor.phase == pytest.approx(successor_phase)
    assert manifest.transport_floor_mm is None
    assert shadow["handoff_id"] == manifest.handoff_id
    assert shadow["terminal_state"] == "RUN_B"
    assert shadow["takeover_success"] is True
    assert shadow["robot_commands_published"] == 0

    registry = EdgeRuntimeRegistry.load(registry_path)
    operator_ids = (
        "T4.open_drawer",
        acquire_operator,
        "T4.deliver_to_drawer",
        "T4.close_drawer",
    )
    initial = WorldState(
        drawer="closed",
        white_container="closed",
        blue_block_location=source_location,
        holding="none",
        gripper="open",
        contact_mode="free_space",
    )
    result = uniform_cost_search_level2(
        initial,
        {
            "drawer": "closed",
            "blue_block_location": "drawer",
            "holding": "none",
            "gripper": "open",
        },
        tuple(catalog.by_id[item] for item in operator_ids),
        registry,
        bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
        allow_intermediate_release=False,
    )
    assert result.best_plan is not None
    assert result.best_plan.operator_ids == operator_ids

    compiled = compile_plan_to_multi_v2(
        result.best_plan,
        catalog,
        registry,
        composition_id=f"test_t4_{successor_task}_t4_to_drawer",
        expected_bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
    )
    stages = compiled.mapping["stages"]
    transitions = compiled.mapping["transitions"]
    assert stages[0]["phase_supervisor"]["gripper_event"] == "closed_then_open"
    assert stages[0]["phase_supervisor"][
        "rearm_closed_then_open_at_prearm"
    ] is True
    assert stages[2]["operators"] == [
        "T4.deliver_to_drawer",
        "T4.close_drawer",
    ]
    assert transitions[0]["source_execution_tail"] is None
    assert transitions[0]["runtime_successor_reference"][
        "reference_phase"
    ] == pytest.approx(successor_phase)
    assert transitions[0]["transport_floor_mm"] is None
    assert transitions[1]["source_execution_tail"] is not None
    assert transitions[1]["runtime_successor_reference"][
        "reference_phase"
    ] == pytest.approx(0.8)


def test_v8_t1_t2_selects_the_shortest_floor400_safe_semantic_join():
    registry = _load(FLOOR400_REGISTRY_PATH)
    edge = next(
        item
        for item in registry["edges"]
        if item["source_operator"] == "T1.open_white_container"
        and item["successor_operator"] == "T2.acquire_from_floor"
    )
    assert edge["admission_status"] == "flexible_verified"
    assert edge["validation_method"].endswith("_transport_floor")
    assert edge["evidence"]["bridge_transport_floor_mm"] == pytest.approx(
        400.0
    )

    manifest_path = _resolved(edge["handoff_manifest"], ARTIFACT_ROOT)
    shadow_path = _resolved(edge["policy_shadow"], ARTIFACT_ROOT)
    manifest = EpisodeHandoffManifest.load(manifest_path)
    shadow = _load(shadow_path)
    selection = _load(
        manifest_path.parent / "semantic_local_reference_selection.json"
    )

    assert manifest.source.segment == "S2"
    assert manifest.source.phase == pytest.approx(1.0)
    assert manifest.successor.segment == "S1"
    assert manifest.successor.phase == pytest.approx(0.66)
    assert manifest.successor.nominal_position_mm[2] == pytest.approx(
        430.5010070800781
    )
    assert manifest.transport_floor_mm == pytest.approx(400.0)
    assert selection["bridge_transport_floor_mm"] == pytest.approx(400.0)
    assert selection["selected"]["geometry"]["selected"][
        "minimum_position_z_mm"
    ] >= 400.0

    phase_074 = next(
        item
        for item in selection["candidates"]
        if item["successor_phase"] == pytest.approx(0.74)
    )
    assert phase_074["geometry_valid"] is False
    assert phase_074["geometry_rejection_reason_counts"][
        "transport_floor_violation"
    ] > 0

    assert shadow["handoff_id"] == manifest.handoff_id
    assert shadow["terminal_state"] == "RUN_B"
    assert shadow["takeover_success"] is True
    assert shadow["robot_commands_published"] == 0
    assert shadow["bridge"]["transport_floor_mm"] == pytest.approx(400.0)
    assert shadow["bridge"]["minimum_position_z_mm"] >= 400.0


def test_v8_compiles_and_propagates_t1_t2_floor400_to_web_support(
    catalog,
    tmp_path,
):
    registry = EdgeRuntimeRegistry.load(FLOOR400_REGISTRY_PATH)
    operators = (
        catalog.by_id["T1.open_white_container"],
        catalog.by_id["T2.acquire_from_floor"],
        catalog.by_id["T1.deliver_to_white_container"],
    )
    initial = WorldState(
        drawer="closed",
        white_container="closed",
        blue_block_location="left_floor",
        holding="none",
        gripper="open",
        contact_mode="free_space",
    )
    result = uniform_cost_search_level2(
        initial,
        {
            "white_container": "open",
            "blue_block_location": "white_container",
            "holding": "none",
            "gripper": "open",
        },
        operators,
        registry,
        bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
        allow_intermediate_release=False,
    )
    assert result.best_plan is not None
    compiled = compile_plan_to_multi_v2(
        result.best_plan,
        catalog,
        registry,
        composition_id="test_t1_t2_floor400_v8",
        expected_bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
    )
    assert compiled.mapping["transitions"][0][
        "transport_floor_mm"
    ] == pytest.approx(400.0)
    provenance = compiled.mapping["planner_provenance"]
    assert provenance["z_minimum_enabled"] is True
    assert provenance["bridge_transport_floors"][0][
        "transport_floor_mm"
    ] == pytest.approx(400.0)

    plan_path = compiled.write_json(tmp_path / "multi_stage_plan.json")
    support_path = build_web_runtime_support_manifest(
        multi_stage_plan_path=plan_path,
        template_path=RUNTIME_TEMPLATE_PATH,
        output_path=tmp_path / "support_manifest.json",
    )
    support = _load(support_path)
    assert support["b_entries"][0]["minimum_bridge_z_mm"] == pytest.approx(
        400.0
    )
    assert support["planners"]["feasibility"][
        "enforce_payload_transport_floor"
    ] is True


def test_v11_compiles_t4_t7_t4_with_post_o1_s3_execution_tail(catalog):
    raw_registry = _load(DRAWER_TOP_TAIL_REGISTRY_PATH)
    edge = next(
        item
        for item in raw_registry["edges"]
        if item["source_operator"] == "T4.open_drawer"
        and item["successor_operator"] == "T7.acquire_from_drawer_top"
    )
    assert edge["validation_method"] == (
        "execution_tail_s3_semantic_local_interior_dynamic_future_join"
    )
    evidence = edge["evidence"]
    assert evidence["source_reference_mode"] == "execution_tail"
    assert evidence["reviewed_empty_gripper_tail"] is True
    assert evidence["runtime_source_segment"] == "S3"
    assert evidence["runtime_source_phase_window"] == pytest.approx(
        [0.17, 0.23]
    )
    assert evidence["semantic_local_refresh"] == "promoted_execution_tail_s3"

    manifest = EpisodeHandoffManifest.load(
        _resolved(edge["handoff_manifest"], ARTIFACT_ROOT)
    )
    assert manifest.source.segment == "S3"
    assert manifest.source.phase == pytest.approx(0.17)
    assert manifest.successor.segment == "S1"
    assert manifest.successor.phase == pytest.approx(0.94)
    assert manifest.transport_floor_mm is None
    assert manifest.runtime_source_bank_path is not None

    registry = EdgeRuntimeRegistry.load(DRAWER_TOP_TAIL_REGISTRY_PATH)
    operator_ids = (
        "T4.open_drawer",
        "T7.acquire_from_drawer_top",
        "T4.deliver_to_drawer",
        "T4.close_drawer",
    )
    initial = WorldState(
        drawer="closed",
        white_container="closed",
        blue_block_location="drawer_top",
        holding="none",
        gripper="open",
        contact_mode="free_space",
    )
    result = uniform_cost_search_level2(
        initial,
        {
            "drawer": "closed",
            "blue_block_location": "drawer",
            "holding": "none",
            "gripper": "open",
        },
        tuple(catalog.by_id[item] for item in operator_ids),
        registry,
        bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
        allow_intermediate_release=False,
    )
    assert result.best_plan is not None
    assert result.best_plan.operator_ids == operator_ids

    compiled = compile_plan_to_multi_v2(
        result.best_plan,
        catalog,
        registry,
        composition_id="test_t4_t7_t4_post_o1_tail_v11",
        expected_bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
    )
    first_stage = compiled.mapping["stages"][0]
    first_transition = compiled.mapping["transitions"][0]
    first_tail = first_transition["source_execution_tail"]
    assert first_tail["semantic_exit_segment"] == "S2"
    assert first_tail["tracking_segment"] == "S3"
    assert first_tail["commit_phase_low"] == pytest.approx(0.17)
    assert first_tail["commit_phase_high"] == pytest.approx(0.23)
    assert first_tail["deadline_phase"] == pytest.approx(0.27)
    supervisor = first_stage["phase_supervisor"]
    assert supervisor["gripper_event"] == "closed_then_open"
    assert supervisor["rearm_closed_then_open_at_prearm"] is True
    assert supervisor["semantic_event_phase_half_width"] == pytest.approx(0.05)
    assert supervisor["semantic_event_prearm_extra_phase"] == pytest.approx(0.02)
    assert supervisor["phase_half_width"] == pytest.approx(0.03)
    assert supervisor["prearm_extra_phase"] == pytest.approx(0.17)

    second_transition = compiled.mapping["transitions"][1]
    assert second_transition["source_operator"] == "T7.acquire_from_drawer_top"
    assert second_transition["source_execution_tail"]["tracking_segment"] == "S2"
    assert second_transition["source_execution_tail"][
        "commit_phase_low"
    ] == pytest.approx(0.16)
    assert compiled.mapping["planner_provenance"]["zero_cost_execution_tail"][
        "eligible_source_contract"
    ].endswith("empty_gripper_free_space")
