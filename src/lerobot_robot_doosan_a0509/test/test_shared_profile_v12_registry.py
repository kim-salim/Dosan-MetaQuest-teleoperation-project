from __future__ import annotations

import json
from pathlib import Path

import pytest

pytest.importorskip("lerobot")

from lerobot_robot_doosan_a0509.interior_policy.catalog import (
    load_operator_catalog,
)
from lerobot_robot_doosan_a0509.interior_policy.contracts import WorldState
from lerobot_robot_doosan_a0509.interior_policy.execution_tail import (
    LEGACY_EXECUTION_TAIL_PROFILE_ID,
    SHARED_EXECUTION_TAIL_PROFILE_ID,
)
from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (
    EdgeRuntimeRegistry,
    compile_plan_to_multi_v2,
    uniform_cost_search_level2,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    BridgeAdmissionMode,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
ARTIFACT_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_spatial_floor_level2_semantic_local_all_2026-09-03"
)
V11 = ARTIFACT_ROOT / "edge_registry_level2_spatial_v11.json"
V12 = ARTIFACT_ROOT / "edge_registry_level2_spatial_v12.json"
CATALOG = (
    REPOSITORY_ROOT
    / "config/interior_policy/a0509_t1_t8_operator_catalog_spatial_v2.json"
)


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def test_v12_declares_shared_profile_and_v11_remains_real_rollback():
    v11 = EdgeRuntimeRegistry.load(V11)
    v12 = EdgeRuntimeRegistry.load(V12)
    assert v11.execution_tail_profile_id == LEGACY_EXECUTION_TAIL_PROFILE_ID
    assert v12.execution_tail_profile_id == SHARED_EXECUTION_TAIL_PROFILE_ID

    raw = _load(V12)
    contract = raw["selection_contract"]
    assert contract["execution_tail_profile_id"] == (
        SHARED_EXECUTION_TAIL_PROFILE_ID
    )
    assert Path(contract["regression_registry"]).resolve() == V11.resolve()
    assert contract["regression_ready"] is True
    shared = contract["shared_execution_tail_handoff"]
    assert shared["fixed_bridge_geometry"] is False
    assert shared["fixed_phase_window"] is False
    assert shared["edge_specific_numeric_tuning"] is False
    assert shared["primary_window_replay_summary"][
        "valid_in_full_source_window_count"
    ] == 26
    assert shared["latched_scan_replay_summary"][
        "valid_in_full_source_window_count"
    ] == 30
    assert shared["latched_scan_replay_summary"][
        "all_episodes_command_safe_in_full_window"
    ] is True


def test_v12_compiles_t4_t7_t4_with_shared_situational_logic():
    catalog = load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        config_path=CATALOG,
        audit_raw_frames=False,
    )
    registry = EdgeRuntimeRegistry.load(V12)
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
        composition_id="test_t4_t7_t4_shared_profile_v12",
        expected_bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
    )
    first = compiled.mapping["transitions"][0]["source_execution_tail"]
    second = compiled.mapping["transitions"][1]["source_execution_tail"]
    assert first["source_operator"] == "T4.open_drawer"
    assert first["commit_phase_low"] == pytest.approx(0.17)
    assert first["commit_phase_high"] == pytest.approx(0.23)
    assert first["deadline_phase"] == pytest.approx(0.35)
    assert first["latch_commit_window_until_deadline"] is True
    assert second["source_operator"] == "T7.acquire_from_drawer_top"
    assert second["commit_phase_low"] == pytest.approx(0.16)
    assert second["commit_phase_high"] == pytest.approx(0.21)
    assert second["deadline_phase"] == pytest.approx(0.35)
    assert second["latch_commit_window_until_deadline"] is True
    assert first["commit_phase_low"] != second["commit_phase_low"]
    assert "operator_profile" not in first["derivation_method"]
    assert "operator_profile" not in second["derivation_method"]
    provenance = compiled.mapping["planner_provenance"]
    assert provenance["zero_cost_execution_tail"]["profile_id"] == (
        SHARED_EXECUTION_TAIL_PROFILE_ID
    )


def test_v12_keeps_t1_exact_exit_and_applies_shared_profile_to_t2():
    catalog = load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        config_path=CATALOG,
        audit_raw_frames=False,
    )
    registry = EdgeRuntimeRegistry.load(V12)
    operator_ids = (
        "T1.open_white_container",
        "T2.acquire_from_floor",
        "T1.deliver_to_white_container",
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
        composition_id="test_t1_t2_t1_shared_profile_v12",
        expected_bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
    )
    transitions = compiled.mapping["transitions"]
    assert transitions[0]["source_operator"] == "T1.open_white_container"
    assert transitions[0]["source_execution_tail"] is None
    assert transitions[0]["transport_floor_mm"] is None
    t2_tail = transitions[1]["source_execution_tail"]
    assert t2_tail["source_operator"] == "T2.acquire_from_floor"
    assert t2_tail["commit_phase_low"] == pytest.approx(0.54)
    assert t2_tail["commit_phase_high"] == pytest.approx(0.60)
    assert t2_tail["deadline_phase"] == pytest.approx(0.675)
    assert t2_tail["latch_commit_window_until_deadline"] is True
    assert "same_segment_continuation" in t2_tail["derivation_method"]
    assert "operator_profile" not in t2_tail["derivation_method"]
