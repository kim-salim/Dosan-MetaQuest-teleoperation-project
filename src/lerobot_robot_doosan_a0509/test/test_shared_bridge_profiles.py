from __future__ import annotations

import json
from pathlib import Path

import pytest

pytest.importorskip("lerobot")

from lerobot_robot_doosan_a0509.interior_policy.execution_tail import (
    ExecutionTailDerivationConfig,
)
from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (
    EdgeRuntimeRegistry,
)
from lerobot_robot_doosan_a0509.task_c_handoff.bridge_profile import (
    load_bridge_generation_profile,
    validate_profile_against_validation_config,
)
from lerobot_robot_doosan_a0509.task_c_handoff.runtime_command_profile import (
    LEGACY_RUNTIME_COMMAND_PROFILE_ID,
    RAMP8P5_RUNTIME_COMMAND_PROFILE_ID,
    get_runtime_command_profile,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
BRIDGE_PROFILE = (
    REPOSITORY_ROOT
    / "config/realtime/a0509_flexible_bridge_profiles_v1.json"
)
TAIL_PROFILE = (
    REPOSITORY_ROOT
    / "config/realtime/a0509_execution_tail_handoff_profile_v2.json"
)
VALIDATION_CONFIG = (
    REPOSITORY_ROOT
    / "offline_tools/cross_task_handoff/validation_config_a0509_v2.json"
)
RUNTIME_COMMAND_PROFILES = (
    REPOSITORY_ROOT
    / "config/realtime/a0509_runtime_command_profiles_v1.json"
)
REGISTRY_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_spatial_floor_level2_semantic_local_all_2026-09-03"
)


def test_execution_tail_bridge_profile_is_shared_and_ramp_bounded():
    profile = load_bridge_generation_profile(
        BRIDGE_PROFILE,
        source_reference_mode="execution_tail",
    )
    assert profile.profile_id == "semantic_local_execution_tail_tangent_v1"
    assert profile.edge_specific_tuning is False
    assert profile.bridge_algorithm == "cubic_bezier_tangent_regularized_v1"
    assert profile.minimum_tangent_handle_chord_ratio == pytest.approx(0.04)
    assert profile.maximum_endpoint_speed_adjustment_mm_s == pytest.approx(12.0)
    assert profile.runtime_contract[
        "derived_unadjusted_axis_speed_ceiling_mm_s"
    ] == pytest.approx(112.5)
    assert profile.runtime_contract[
        "derived_flex_axis_speed_envelope_mm_s"
    ] == pytest.approx(112.5)
    validation = json.loads(VALIDATION_CONFIG.read_text(encoding="utf-8"))
    validate_profile_against_validation_config(profile, validation)


def test_saved_tail_profile_matches_runtime_defaults_without_edge_ids():
    saved = json.loads(TAIL_PROFILE.read_text(encoding="utf-8"))
    config = ExecutionTailDerivationConfig()
    different = saved["continuation_classes"]["different_segment"]
    same = saved["continuation_classes"]["same_segment"]

    assert saved["edge_specific_numeric_tuning"] is False
    assert saved["fixed_bridge_geometry"] is False
    assert saved["fixed_phase_window"] is False
    assert config.operator_overrides == {}
    assert config.max_relative_phase == pytest.approx(
        saved["invariants"]["maximum_relative_scan_phase"]
    )
    assert config.min_candidate_points == saved["invariants"][
        "minimum_candidate_points"
    ]
    for name, value in different.items():
        assert getattr(config, name) == pytest.approx(value)

    same_config = config.for_continuation_class(same_segment=True)
    for name, value in same.items():
        assert getattr(same_config, name) == pytest.approx(value)
    assert same_config.latch_commit_window_until_scan_deadline is True
    assert "collar_phase_window" in saved["data_derived_outputs"]
    assert "bridge_geometry" in saved["data_derived_outputs"]


def test_runtime_command_profiles_bind_ramp_ack_span_and_unchanged_limits():
    saved = json.loads(RUNTIME_COMMAND_PROFILES.read_text(encoding="utf-8"))
    legacy = get_runtime_command_profile(LEGACY_RUNTIME_COMMAND_PROFILE_ID)
    v13 = get_runtime_command_profile(RAMP8P5_RUNTIME_COMMAND_PROFILE_ID)

    for profile in (legacy, v13):
        record = saved["profiles"][profile.profile_id]
        for name in (
            "control_hz",
            "linear_ramp_mm_per_tick",
            "orientation_ramp_deg_per_tick",
            "max_ack_lag_steps",
            "acknowledged_command_span_steps",
            "axis_velocity_limit_mm_s",
            "ack_safe_axis_speed_envelope_mm_s",
            "cartesian_velocity_limit_mm_s",
            "acceleration_limit_mm_s2",
            "jerk_limit_mm_s3",
            "integrated_squared_jerk_limit",
        ):
            assert record[name] == pytest.approx(getattr(profile, name))
        assert record["bridge_ack_mode"] == profile.bridge_ack_mode

    assert legacy.linear_ramp_mm_per_tick == pytest.approx(7.5)
    assert legacy.axis_velocity_limit_mm_s == pytest.approx(225.0)
    assert legacy.ack_safe_axis_speed_envelope_mm_s == pytest.approx(112.5)
    assert v13.linear_ramp_mm_per_tick == pytest.approx(8.5)
    assert v13.axis_velocity_limit_mm_s == pytest.approx(255.0)
    assert v13.ack_safe_axis_speed_envelope_mm_s == pytest.approx(127.5)
    assert v13.acceleration_limit_mm_s2 == legacy.acceleration_limit_mm_s2
    assert v13.jerk_limit_mm_s3 == legacy.jerk_limit_mm_s3
    assert (
        v13.orientation_ramp_deg_per_tick
        == legacy.orientation_ramp_deg_per_tick
    )


def test_v13_registry_changes_only_runtime_profile_and_keeps_v12_rollback():
    v12 = EdgeRuntimeRegistry.load(
        REGISTRY_ROOT / "edge_registry_level2_spatial_v12.json"
    )
    v13 = EdgeRuntimeRegistry.load(
        REGISTRY_ROOT / "edge_registry_level2_spatial_v13.json"
    )
    assert v12.runtime_command_profile_id == LEGACY_RUNTIME_COMMAND_PROFILE_ID
    assert v13.runtime_command_profile_id == RAMP8P5_RUNTIME_COMMAND_PROFILE_ID
    assert v13.execution_tail_profile_id == v12.execution_tail_profile_id
    assert [item.key for item in v13.edges] == [item.key for item in v12.edges]


def test_v13_registry_fails_closed_on_command_profile_config_sha_mismatch(
    tmp_path: Path,
):
    value = json.loads(
        (REGISTRY_ROOT / "edge_registry_level2_spatial_v13.json").read_text(
            encoding="utf-8"
        )
    )
    value["selection_contract"][
        "runtime_command_profile_config_sha256"
    ] = "0" * 64
    corrupted = tmp_path / "v13_bad_profile_sha.json"
    corrupted.write_text(json.dumps(value), encoding="utf-8")
    with pytest.raises(ValueError, match="profile config SHA mismatch"):
        EdgeRuntimeRegistry.load(corrupted)
