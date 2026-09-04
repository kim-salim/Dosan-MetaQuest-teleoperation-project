from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("lerobot")

from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    EpisodeHandoffManifest,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
ARTIFACT_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t7_to_t3_s2_runtime_reference_2026-08-31"
)
MANIFEST_PATH = ARTIFACT_ROOT / "flexible_reference_manifest.json"
SUCCESSOR_BANK_PATH = ARTIFACT_ROOT / "t3_s2_entry_reference_bank.json"
WINDOW_PATH = ARTIFACT_ROOT / "t3_s2_empirical_transport_phase_window.json"
SELECTION_PATH = ARTIFACT_ROOT / "semantic_local_reference_selection.json"
GEOMETRY_PATH = ARTIFACT_ROOT / "geometry_evaluation.json"
SHADOW_PATH = ARTIFACT_ROOT / "flexible_policy_shadow.json"


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def test_t7_t3_uses_empirical_successor_interior_not_phase_zero():
    manifest = EpisodeHandoffManifest.load(MANIFEST_PATH)
    window = _load(WINDOW_PATH)
    bank = _load(SUCCESSOR_BANK_PATH)

    assert manifest.source.segment == "S2"
    assert manifest.source.phase == pytest.approx(0.16)
    assert manifest.successor.segment == "S2"
    assert manifest.successor.phase == pytest.approx(0.65)
    assert manifest.successor.phase != pytest.approx(0.0)
    assert window["phase_window"] == pytest.approx([0.13, 0.68])
    assert window["episode_count"] == 30
    assert window["runtime_z_minimum_enabled"] is False
    assert bank["selection"]["medoid_episode"] == 23
    semantic = bank["semantic_local_selection"]
    assert semantic["runtime_phase_gate"] is False
    assert semantic["path_margin_to_high_mm"] >= 15.0
    assert semantic["fixed_z_minimum_enabled"] is False


def test_semantic_local_objective_selects_minimum_hard_pass_candidate():
    report = _load(SELECTION_PATH)
    selected = report["selected"]
    feasible = [
        item for item in report["candidates"] if item["geometry_valid"]
    ]
    assert report["pairs_considered"] == 275
    assert report["pairs_hard_passed"] == 275
    assert feasible
    assert selected["total_length_mm"] == pytest.approx(
        min(item["total_length_mm"] for item in feasible)
    )
    assert selected["total_length_mm"] == pytest.approx(
        selected["source_prefix_mm"]
        + selected["bridge_length_mm"]
        + selected["successor_suffix_mm"]
    )
    # The support boundary itself is not admitted as a reference candidate.
    assert max(item["successor_phase"] for item in feasible) == pytest.approx(
        0.65
    )
    assert selected["successor"]["phase"] == pytest.approx(0.65)
    assert report["dijkstra_operator_cost_changed"] is False
    assert report["runtime_successor_phase_gate"] is False


def test_selected_bridge_retains_command_space_hard_limits():
    geometry = _load(GEOMETRY_PATH)
    selected = geometry["geometry"]["selected"]
    assert geometry["geometry"]["valid"] is True
    assert selected["hard_rejection_reasons"] == []
    assert selected["max_ack_span_axis_step_mm"] <= 7.5
    assert selected["max_orientation_step_deg"] <= 1.25
    assert selected["max_command_acceleration_mm_s2"] <= 4000.0
    assert selected["max_command_jerk_mm_s3"] <= 4000.0
    assert geometry["runtime_successor_phase_gate"] is False
    assert geometry["robot_commands_published"] == 0


def test_fresh_t3_policy_shadow_admits_interior_reference():
    manifest = EpisodeHandoffManifest.load(MANIFEST_PATH)
    shadow = _load(SHADOW_PATH)
    admission = shadow["prefix_admission"]
    execution = admission["execution_dynamics"]

    assert shadow["handoff_id"] == manifest.handoff_id
    assert shadow["successor_observation"]["episode"] == 23
    assert shadow["successor_observation"]["frame"] == 402
    assert shadow["terminal_state"] == "RUN_B"
    assert shadow["takeover_success"] is True
    assert shadow["fallback_required"] is False
    assert admission["valid"] is True
    assert execution["max_xyz_axis_step_mm"] <= 7.5
    assert execution["max_acceleration_mm_s2"] <= 4000.0
    assert shadow["control_timing"]["deadline_miss_count"] == 0
    assert shadow["robot_commands_published"] == 0


def test_manifest_round_trip_preserves_successor_reference_provenance():
    manifest = EpisodeHandoffManifest.load(MANIFEST_PATH)
    restored = EpisodeHandoffManifest.from_mapping(manifest.to_record())

    assert restored.runtime_successor_bank_path == (
        "t3_s2_entry_reference_bank.json"
    )
    assert restored.successor_reference_phase_window == pytest.approx(
        (0.13, 0.68)
    )
    assert restored.successor_interior_path_margin_mm == pytest.approx(15.0)
    assert restored.semantic_local_total_length_mm == pytest.approx(
        restored.semantic_local_source_prefix_mm
        + restored.semantic_local_bridge_length_mm
        + restored.semantic_local_successor_suffix_mm
    )
    np.testing.assert_allclose(
        restored.successor.nominal_position_mm,
        manifest.successor.nominal_position_mm,
    )
