from __future__ import annotations

import json
from pathlib import Path

import pytest

pytest.importorskip("lerobot")

from lerobot_robot_doosan_a0509.interior_policy.catalog import (
    load_operator_catalog,
)
from lerobot_robot_doosan_a0509.interior_policy.execution_tail import (
    derive_zero_cost_execution_tail,
)
from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (
    EdgeRuntimeRegistry,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    EpisodeHandoffManifest,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
ARTIFACT_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/"
    "t1_t8_spatial_floor_level2_semantic_local_all_2026-09-03"
)
REGISTRY_PATH = ARTIFACT_ROOT / "edge_registry_level2_spatial_v6.json"
SUMMARY_PATH = ARTIFACT_ROOT / "semantic_local_refresh_summary.json"
CATALOG_PATH = (
    REPOSITORY_ROOT
    / "config/interior_policy/a0509_t1_t8_operator_catalog_spatial_v2.json"
)


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _resolved(value: str, base: Path) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (base / path).resolve()


def test_all_held_transport_edges_use_semantic_local_references():
    catalog = load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        config_path=CATALOG_PATH,
        audit_raw_frames=False,
    )
    value = _load(REGISTRY_PATH)
    eligible = {
        operator.id
        for operator in catalog.operators
        if derive_zero_cost_execution_tail(operator).profile is not None
    }
    eligible_edges = [
        edge for edge in value["edges"] if edge["source_operator"] in eligible
    ]

    assert len(value["edges"]) == 127
    assert len(eligible) == 8
    assert len(eligible_edges) == 56
    assert all(
        edge["admission_status"] == "flexible_verified"
        for edge in eligible_edges
    )
    assert all(
        "semantic_local" in edge["validation_method"]
        for edge in eligible_edges
    )


def test_every_semantic_local_manifest_and_fresh_b_shadow_match():
    value = _load(REGISTRY_PATH)
    semantic_edges = [
        edge
        for edge in value["edges"]
        if "semantic_local" in edge["validation_method"]
    ]
    assert len(semantic_edges) == 56

    for edge in semantic_edges:
        manifest_path = _resolved(edge["handoff_manifest"], ARTIFACT_ROOT)
        shadow_path = _resolved(edge["policy_shadow"], ARTIFACT_ROOT)
        manifest = EpisodeHandoffManifest.load(manifest_path)
        shadow = _load(shadow_path)

        assert manifest.semantic_local_total_length_mm == pytest.approx(
            manifest.semantic_local_source_prefix_mm
            + manifest.semantic_local_bridge_length_mm
            + manifest.semantic_local_successor_suffix_mm
        )
        assert manifest.runtime_successor_bank_path is not None
        assert shadow["handoff_id"] == manifest.handoff_id
        assert shadow["terminal_state"] == "RUN_B"
        assert shadow["takeover_success"] is True
        assert shadow["fallback_required"] is False
        assert shadow["prefix_admission"]["valid"] is True
        assert shadow["control_timing"]["deadline_miss_count"] == 0
        assert shadow["robot_commands_published"] == 0
        assert shadow["live_enabled"] is False
        assert shadow["mux_selected"] is False


def test_non_applicable_source_contracts_preserve_baseline_edges():
    value = _load(REGISTRY_PATH)
    edges = [
        edge
        for edge in value["edges"]
        if edge["evidence"].get("semantic_local_refresh") == "not_applicable"
    ]

    assert len(edges) == 71
    assert all(
        edge["evidence"]["semantic_local_not_applicable_reason"]
        == "source_bridge_mode_is_not_held_object_free_transport"
        for edge in edges
    )


def test_refresh_summary_and_registry_contract_are_complete():
    summary = _load(SUMMARY_PATH)
    registry = _load(REGISTRY_PATH)
    loaded = EdgeRuntimeRegistry.load(REGISTRY_PATH)

    assert len(loaded.edges) == 127
    assert summary["counts"] == {
        "not_applicable_source_contract": 71,
        "preserved_reviewed_semantic_local": 2,
        "promoted_semantic_local": 54,
    }
    assert summary["robot_commands_published"] == 0
    assert summary["physical_validation_performed"] is False
    assert registry["selection_contract"]["eligible_edge_count"] == 56
    assert registry["selection_contract"]["dijkstra_operator_cost_changed"] is False
    assert registry["selection_contract"]["runtime_successor_phase_gate"] is False


def test_reviewed_and_rank_retry_choices_are_preserved():
    expected = {
        ("T2.acquire_from_floor", "T3.deliver_to_floor"): 14,
        ("T7.acquire_from_drawer_top", "T3.deliver_to_floor"): 0,
        ("T8.acquire_top_block_from_stack", "T3.deliver_to_floor"): 11,
    }
    value = _load(REGISTRY_PATH)
    by_key = {
        (edge["source_operator"], edge["successor_operator"]): edge
        for edge in value["edges"]
    }

    for key, selected_rank in expected.items():
        manifest_path = _resolved(by_key[key]["handoff_manifest"], ARTIFACT_ROOT)
        selection = _load(
            manifest_path.parent / "semantic_local_reference_selection.json"
        )
        # The earliest T7 artifact predates the explicit rank field; its
        # selected candidate is the rank-0 minimum by construction.
        actual_rank = selection.get("selected_candidate_rank", 0)
        assert actual_rank == selected_rank
