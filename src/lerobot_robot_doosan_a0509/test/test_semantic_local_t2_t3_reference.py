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
from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    BridgeAdmissionMode,
    EpisodeHandoffManifest,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
ARTIFACT_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t2_to_t3_s2_runtime_reference_2026-09-03"
)
REGISTRY_PATH = (
    REPOSITORY_ROOT
    / "docs/artifacts/"
    "t1_t8_spatial_floor_level2_semantic_local_2026-09-03/"
    "edge_registry_level2_spatial_v5.json"
)
CATALOG_PATH = (
    REPOSITORY_ROOT
    / "config/interior_policy/a0509_t1_t8_operator_catalog_spatial_v2.json"
)
MANIFEST_PATH = ARTIFACT_ROOT / "flexible_reference_manifest.json"
SELECTION_PATH = ARTIFACT_ROOT / "semantic_local_reference_selection.json"
GEOMETRY_PATH = ARTIFACT_ROOT / "geometry_evaluation.json"
SHADOW_PATH = ARTIFACT_ROOT / "flexible_policy_shadow.json"


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def test_t2_t3_reference_uses_causal_s2_interior_boundaries():
    manifest = EpisodeHandoffManifest.load(MANIFEST_PATH)
    selection = _load(SELECTION_PATH)

    assert manifest.source.task == "T2"
    assert manifest.source.segment == "S2"
    assert manifest.source.phase == pytest.approx(0.56)
    assert manifest.source.phase <= selection["source_reference_phase_max"]
    assert manifest.source.support_episode == 26
    assert manifest.source.support_frame == 430

    assert manifest.successor.task == "T3"
    assert manifest.successor.segment == "S2"
    assert manifest.successor.phase == pytest.approx(0.53)
    assert manifest.successor.phase != pytest.approx(0.0)
    assert manifest.successor.support_episode == 23
    assert manifest.successor.support_frame == 375
    successor_reference = manifest.to_record()["generation"][
        "runtime_successor_reference"
    ]
    assert successor_reference["runtime_phase_gate"] is False


def test_t2_t3_semantic_local_objective_and_bounded_ranking():
    report = _load(SELECTION_PATH)
    selected = report["selected"]
    feasible = [
        item for item in report["candidates"] if item["geometry_valid"]
    ]

    assert report["pairs_considered"] == 175
    assert report["pairs_hard_passed"] == 175
    assert report["selected_candidate_rank"] == 14
    assert report["source_reference_phase_max"] == pytest.approx(0.57)
    assert max(item["source_phase"] for item in feasible) <= 0.57 + 1.0e-9
    assert selected["total_length_mm"] == pytest.approx(
        selected["source_prefix_mm"]
        + selected["bridge_length_mm"]
        + selected["successor_suffix_mm"]
    )
    assert selected["total_length_mm"] == pytest.approx(818.8438996185341)
    assert report["dijkstra_operator_cost_changed"] is False
    assert report["runtime_successor_phase_gate"] is False


def test_t2_t3_reference_retains_command_space_hard_limits():
    geometry = _load(GEOMETRY_PATH)
    selected = geometry["geometry"]["selected"]

    assert geometry["geometry"]["valid"] is True
    assert selected["hard_rejection_reasons"] == []
    assert selected["max_ack_span_axis_step_mm"] <= 7.5
    assert selected["max_ack_span_orientation_step_deg"] <= 1.25
    assert selected["max_command_acceleration_mm_s2"] <= 4000.0
    assert selected["max_command_jerk_mm_s3"] <= 4000.0
    assert geometry["runtime_successor_phase_gate"] is False
    assert geometry["robot_commands_published"] == 0


def test_fresh_t3_policy_shadow_takes_over_without_commands():
    manifest = EpisodeHandoffManifest.load(MANIFEST_PATH)
    shadow = _load(SHADOW_PATH)
    admission = shadow["prefix_admission"]
    execution = admission["execution_dynamics"]

    assert shadow["handoff_id"] == manifest.handoff_id
    assert shadow["successor_observation"]["episode"] == 23
    assert shadow["successor_observation"]["frame"] == 375
    assert shadow["terminal_state"] == "RUN_B"
    assert shadow["takeover_success"] is True
    assert shadow["fallback_required"] is False
    assert admission["valid"] is True
    assert admission["splice_index"] == 0
    assert execution["max_xyz_axis_step_mm"] <= 7.5
    assert execution["max_acceleration_mm_s2"] <= 4000.0
    assert shadow["control_timing"]["deadline_miss_count"] == 0
    assert shadow["robot_commands_published"] == 0
    assert shadow["live_enabled"] is False
    assert shadow["mux_selected"] is False


def test_spatial_v5_dijkstra_compiles_exact_t2_t3_runtime_edge():
    catalog = load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        config_path=CATALOG_PATH,
        audit_raw_frames=False,
    )
    registry = EdgeRuntimeRegistry.load(REGISTRY_PATH)
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
        {
            "blue_block_location": "right_floor",
            "holding": "none",
            "gripper": "open",
            "contact_mode": "free_space",
        },
        operators,
        registry,
        cost_config=catalog.target.cost_config,
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
        plan,
        catalog,
        registry,
        composition_id="t2_t3_semantic_local_level2_test",
        expected_bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
    )
    transition = compiled.mapping["transitions"][0]
    assert transition["bridge_admission_status"] == "flexible_verified"
    assert transition["handoff_manifest"] == str(MANIFEST_PATH)
    assert transition["policy_shadow"] == str(SHADOW_PATH)

    source = transition["runtime_source_reference"]
    assert source["segment"] == "S2"
    assert source["reference_phase"] == pytest.approx(0.56)
    assert source["nominal_phase"] == pytest.approx(0.57)
    assert source["phase_window"] == pytest.approx([0.54, 0.60])
    assert source["validated_runtime_source_bank"] is True

    successor = transition["runtime_successor_reference"]
    assert successor["segment"] == "S2"
    assert successor["reference_phase"] == pytest.approx(0.53)
    assert successor["phase_window"] == pytest.approx([0.13, 0.68])
    assert successor["runtime_phase_gate"] is False
    assert successor["validated_runtime_successor_bank"] is True

    tail = transition["source_execution_tail"]
    assert tail["source_operator"] == "T2.acquire_from_floor"
    assert tail["tracking_segment"] == "S2"
    assert tail["nominal_phase"] == pytest.approx(0.57)
    assert tail["commit_phase_low"] == pytest.approx(0.54)
    assert tail["commit_phase_high"] == pytest.approx(0.60)
    assert tail["zero_symbolic_cost"] is True
