from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("lerobot")

from lerobot_robot_doosan_a0509.interior_policy.catalog import (
    load_operator_catalog,
)
from lerobot_robot_doosan_a0509.interior_policy.execution_tail import (
    ExecutionTailDerivationConfig,
    derive_zero_cost_execution_tail,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="module")
def catalog():
    return load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        audit_raw_frames=False,
    )


def test_all_t1_t8_acquisition_sources_receive_data_derived_tail(catalog):
    expected = {
        "T1.acquire_from_black_table": ("S4", 0.0),
        "T2.acquire_from_floor": ("S2", 0.5),
        "T3.acquire_from_black_table": ("S2", 0.0),
        "T4.acquire_from_black_table": ("S4", 0.0),
        "T5.acquire_from_drawer": ("S4", 0.0),
        "T6.acquire_moving_block": ("S2", 0.0),
        "T7.acquire_from_drawer_top": ("S2", 0.0),
        "T8.acquire_top_block_from_stack": ("S2", 0.0),
    }
    actual = {}
    for operator_id, (segment, start_phase) in expected.items():
        operator = catalog.by_id[operator_id]
        result = derive_zero_cost_execution_tail(operator)
        assert result.reason == "derived"
        assert result.profile is not None
        profile = result.profile
        actual[operator_id] = profile
        assert profile.tracking_segment == segment
        assert profile.tracking_start_phase == pytest.approx(start_phase)
        assert profile.commit_phase_low > start_phase
        assert profile.commit_phase_high > profile.commit_phase_low
        assert profile.deadline_phase > profile.commit_phase_high
        assert profile.zero_symbolic_cost is True
        assert profile.fixed_z_minimum_used is False
        assert profile.episode_count == (
            catalog.audit.tasks[operator.policy_id].episode_count
        )
        assert operator.base_cost == pytest.approx(1.0)

    assert set(actual) == set(expected)
    t7 = actual["T7.acquire_from_drawer_top"]
    assert t7.commit_phase_low == pytest.approx(0.16)
    assert t7.commit_phase_high == pytest.approx(0.21)
    assert t7.deadline_phase == pytest.approx(0.35)
    assert t7.latch_commit_window_until_deadline is True
    assert t7.collar_progress_mm_low > 50.0
    assert t7.collar_progress_mm_high > 120.0


def test_t2_profile_expands_same_segment_bridge_opportunity(catalog):
    result = derive_zero_cost_execution_tail(
        catalog.by_id["T2.acquire_from_floor"]
    )
    assert result.profile is not None
    profile = result.profile
    assert profile.same_segment_as_semantic_exit is True
    assert profile.semantic_exit_segment == "S2"
    assert profile.semantic_exit_phase == pytest.approx(0.5)
    assert profile.collar_phase_low == pytest.approx(0.52)
    assert profile.collar_phase_high == pytest.approx(0.60)
    assert profile.commit_phase_low == pytest.approx(0.54)
    assert profile.commit_phase_high == pytest.approx(0.60)
    assert profile.deadline_phase == pytest.approx(0.675)
    assert profile.latch_commit_window_until_deadline is True
    assert "same_segment_continuation" in profile.derivation_method
    assert "operator_profile" not in profile.derivation_method


def test_t2_scan_latch_can_be_disabled_without_operator_tuning(catalog):
    result = derive_zero_cost_execution_tail(
        catalog.by_id["T2.acquire_from_floor"],
        config=ExecutionTailDerivationConfig(
            latch_commit_window_until_scan_deadline=False,
        ),
    )
    assert result.profile is not None
    profile = result.profile
    assert profile.collar_phase_low == pytest.approx(0.52)
    assert profile.collar_phase_high == pytest.approx(0.60)
    assert profile.commit_phase_low == pytest.approx(0.54)
    assert profile.commit_phase_high == pytest.approx(0.60)
    assert profile.deadline_phase == pytest.approx(0.65)
    assert profile.latch_commit_window_until_deadline is False
    assert "operator_profile" not in profile.derivation_method


def test_non_held_source_preserves_exact_semantic_boundary(catalog):
    result = derive_zero_cost_execution_tail(
        catalog.by_id["T4.open_drawer"]
    )
    assert result.profile is None
    assert result.eligible is False
    assert result.reason == (
        "source_bridge_mode_is_not_held_object_free_transport"
    )



def test_t4_open_tail_requires_review_and_uses_post_o1_s3(catalog):
    config = ExecutionTailDerivationConfig(
        reviewed_empty_gripper_free_space_operators=("T4.open_drawer",)
    )
    result = derive_zero_cost_execution_tail(
        catalog.by_id["T4.open_drawer"],
        config=config,
    )
    assert result.reason == "derived"
    assert result.profile is not None
    profile = result.profile
    assert profile.semantic_exit_segment == "S2"
    assert profile.semantic_exit_phase == pytest.approx(1.0)
    assert profile.tracking_segment == "S3"
    assert profile.tracking_start_phase == pytest.approx(0.0)
    assert profile.collar_phase_low == pytest.approx(0.10)
    assert profile.collar_phase_high == pytest.approx(0.23)
    assert profile.commit_phase_low == pytest.approx(0.17)
    assert profile.commit_phase_high == pytest.approx(0.23)
    assert profile.deadline_phase == pytest.approx(0.35)
    assert profile.latch_commit_window_until_deadline is True
    assert "operator_profile" not in profile.derivation_method
    assert profile.fixed_z_minimum_used is False
