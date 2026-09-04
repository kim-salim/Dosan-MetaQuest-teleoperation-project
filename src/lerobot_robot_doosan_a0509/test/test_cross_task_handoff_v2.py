from __future__ import annotations

import numpy as np

from lerobot_robot_doosan_a0509.task_c_handoff.compatibility import (
    HandoffCompatibilityEvaluator,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    EpisodeHandoffManifest,
    HandoffCompatibilityConfig,
)
from offline_tools.cross_task_handoff.authority import SemanticAuthority
from offline_tools.cross_task_handoff.build_episode_phase_index import (
    _contact_mode,
    _semantic_state_facts,
)
from offline_tools.cross_task_handoff.schema import (
    HandoffCandidate,
    PhaseIndexPoint,
)
from offline_tools.cross_task_handoff.select_diverse_handoffs import (
    episode_manifest,
    farthest_point_selection,
)
from offline_tools.cross_task_handoff.validate_handoff_candidates import (
    CandidateValidationConfig,
    semantic_hard_rejections,
    validate_candidate,
)
from offline_tools.task_c_bridge_v0.bezier_bridge import (
    CUBIC_BEZIER_FIXED,
    CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
)


def _point(
    *,
    task: str,
    episode: int,
    x_mm: float,
    phase: float,
    velocity_x_mm_s: float = 10.0,
    held_object: str = "blue_block",
    gripper_state: str = "closed",
    contact_mode: str = "free_transport_assumed",
    preconditions: tuple[str, ...] = ("blue_block=held",),
) -> PhaseIndexPoint:
    return PhaseIndexPoint(
        task=task,
        segment="S_transport",
        phase=phase,
        support_episode=episode,
        support_frame=100 + episode,
        position_mm=np.array([x_mm, 0.0, 400.0]),
        orientation_quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        velocity_mm_s=np.array([velocity_x_mm_s, 0.0, 0.0]),
        semantic_state="transport_blue_block",
        gripper_state=gripper_state,
        held_object=held_object,
        contact_mode=contact_mode,
        entry_preconditions=preconditions,
        completed_subgoals=preconditions,
        support_score=0.9,
    )


def _validation_config() -> CandidateValidationConfig:
    return CandidateValidationConfig(
        workspace_min_mm=np.array([0.0, -500.0, 0.0]),
        workspace_max_mm=np.array([700.0, 500.0, 700.0]),
        velocity_limit_mm_s=1000.0,
        axis_velocity_limit_mm_s=1000.0,
        acceleration_limit_mm_s2=1000.0,
        curvature_limit_per_mm=1.0e6,
        jerk_limit_mm_s3=1.0e6,
        integrated_squared_jerk_limit=1.0e12,
        backtracking_ratio_limit=10.0,
        linear_ramp_mm_per_tick=20.0,
        orientation_ramp_deg_per_tick=5.0,
    )


def _candidate(index: int) -> HandoffCandidate:
    source = _point(
        task="A",
        episode=index,
        x_mm=100.0 + 5.0 * index,
        phase=0.2 + 0.1 * index,
    )
    successor = _point(
        task="B",
        episode=index + 10,
        x_mm=180.0 + 10.0 * index,
        phase=0.1 + 0.15 * index,
        velocity_x_mm_s=5.0,
    )
    return HandoffCandidate(
        handoff_id=f"h_{index}",
        source=source,
        successor=successor,
        bridge_duration_s=2.0 + 0.2 * index,
        bridge_length_mm=80.0 + 5.0 * index,
        max_velocity_mm_s=100.0,
        max_axis_velocity_mm_s=90.0,
        max_acceleration_mm_s2=200.0,
        jerk_metric=1000.0 + index,
        max_orientation_step_deg=0.2,
        support_score=0.9 - 0.1 * index,
        feasible=True,
        rejection_reason=(),
        ik_checked=False,
        collision_checked=False,
        transport_floor_mm=350.0,
    )


def test_candidate_record_round_trip_and_diverse_selection_are_reproducible():
    candidates = [_candidate(index) for index in range(4)]
    restored = HandoffCandidate.from_record(candidates[0].to_record())
    assert restored.handoff_id == candidates[0].handoff_id
    assert restored.bridge_length_mm == candidates[0].bridge_length_mm
    assert restored.max_velocity_mm_s == candidates[0].max_velocity_mm_s

    first, first_metadata = farthest_point_selection(candidates, count=3)
    second, second_metadata = farthest_point_selection(candidates, count=3)
    assert [item.handoff_id for item in first] == [
        item.handoff_id for item in second
    ]
    assert len({item.handoff_id for item in first}) == 3
    assert first_metadata["method"] == "normalized_farthest_point_sampling"
    assert first_metadata["selected_indices"] == second_metadata["selected_indices"]


def test_regularized_candidate_recovers_borderline_curvature_only_opt_in():
    source = _point(
        task="A",
        episode=1,
        x_mm=0.0,
        phase=0.4,
        velocity_x_mm_s=3.0,
    )
    successor = _point(
        task="B",
        episode=2,
        x_mm=250.0,
        phase=0.2,
        velocity_x_mm_s=20.0,
    )
    successor = PhaseIndexPoint(
        **{
            **successor.__dict__,
            "position_mm": np.array([250.0, 10.0, 400.0]),
        }
    )
    config = _validation_config()
    config = CandidateValidationConfig(
        **{
            **config.__dict__,
            "curvature_limit_per_mm": 0.25,
        }
    )

    fixed = validate_candidate(
        source,
        successor,
        duration_s=4.0,
        config=config,
        transport_floor_mm=None,
        semantic_authority=SemanticAuthority.EXTERNAL_PLANNER,
    )
    recovered = validate_candidate(
        source,
        successor,
        duration_s=4.0,
        config=config,
        transport_floor_mm=None,
        semantic_authority=SemanticAuthority.EXTERNAL_PLANNER,
        bridge_algorithm=CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
        minimum_tangent_handle_chord_ratio=0.04,
        maximum_endpoint_speed_adjustment_mm_s=12.0,
    )

    assert fixed.bridge_algorithm == CUBIC_BEZIER_FIXED
    assert not fixed.feasible
    assert "curvature_limit" in fixed.rejection_reason
    assert recovered.feasible
    assert recovered.max_curvature_per_mm is not None
    assert recovered.max_curvature_per_mm < 0.25
    assert recovered.source_endpoint_speed_adjustment_mm_s < 12.0
    assert recovered.handoff_id != fixed.handoff_id

    restored = HandoffCandidate.from_record(recovered.to_record())
    manifest = EpisodeHandoffManifest.from_mapping(
        episode_manifest(
            "A_to_B_regularized",
            restored,
            support_radius_mm=100.0,
            support_orientation_radius_deg=10.0,
            prearm_radius_mm=40.0,
            commit_radius_mm=20.0,
            direction_cosine_minimum=0.7,
        )
    )
    assert manifest.bridge_algorithm == CUBIC_BEZIER_TANGENT_REGULARIZED_V1
    assert manifest.minimum_tangent_handle_chord_ratio == 0.04
    assert manifest.maximum_endpoint_speed_adjustment_mm_s == 12.0
    round_trip = EpisodeHandoffManifest.from_mapping(manifest.to_record())
    assert round_trip.handoff_id == manifest.handoff_id
    assert round_trip.bridge_algorithm == manifest.bridge_algorithm
    assert round_trip.minimum_tangent_handle_chord_ratio == 0.04
    assert round_trip.maximum_endpoint_speed_adjustment_mm_s == 12.0
    np.testing.assert_allclose(
        round_trip.source.nominal_position_mm,
        manifest.source.nominal_position_mm,
    )


def test_regularized_candidate_endpoint_speed_limit_is_hard_reject():
    source = _point(
        task="A",
        episode=1,
        x_mm=0.0,
        phase=0.4,
        velocity_x_mm_s=3.0,
    )
    successor = _point(
        task="B",
        episode=2,
        x_mm=250.0,
        phase=0.2,
        velocity_x_mm_s=20.0,
    )
    candidate = validate_candidate(
        source,
        successor,
        duration_s=4.0,
        config=_validation_config(),
        transport_floor_mm=None,
        semantic_authority=SemanticAuthority.EXTERNAL_PLANNER,
        bridge_algorithm=CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
        minimum_tangent_handle_chord_ratio=0.04,
        maximum_endpoint_speed_adjustment_mm_s=1.0,
    )
    assert not candidate.feasible
    assert "endpoint_speed_adjustment_limit" in candidate.rejection_reason


def test_fixture_contact_and_unknown_contact_are_not_free_transport():
    fixture = {
        "semantic_label": "open_drawer",
        "manipulated_object": "drawer_handle",
    }
    portable = {
        "semantic_label": "transport_blue_block_to_container",
        "manipulated_object": "blue_block",
    }
    assert _contact_mode(
        fixture,
        closed=True,
        held_object="drawer_handle",
    ) == "contact_manipulation_assumed"
    assert _contact_mode(
        portable,
        closed=True,
        held_object="blue_block",
    ) == "free_transport_assumed"
    assert _contact_mode(
        {},
        closed=False,
        held_object="none",
    ) == "free_motion_assumed"


def test_symbolic_state_facts_override_observed_gripper_and_holding_state():
    facts = _semantic_state_facts(
        {"drawer": "open", "gripper": "open", "held_object": "none"},
        gripper_state="closed",
        held_object="blue_block",
    )
    assert "drawer=open" in facts
    assert "gripper=closed" in facts
    assert "held_object=blue_block" in facts
    assert "gripper=open" not in facts


def test_semantic_mismatch_and_successor_precondition_are_hard_rejections():
    source = _point(task="A", episode=1, x_mm=100, phase=0.4)
    held_mismatch = _point(
        task="B",
        episode=2,
        x_mm=180,
        phase=0.2,
        held_object="red_block",
    )
    contact_mismatch = _point(
        task="B",
        episode=3,
        x_mm=180,
        phase=0.2,
        contact_mode="contact_manipulation_assumed",
    )
    unmet = _point(
        task="B",
        episode=4,
        x_mm=180,
        phase=0.2,
        preconditions=("drawer=open",),
    )
    assert "held_object_mismatch" in semantic_hard_rejections(source, held_mismatch)
    assert "contact_mode_mismatch" in semantic_hard_rejections(
        source, contact_mismatch
    )
    assert "successor_precondition_unmet" in semantic_hard_rejections(source, unmet)


def test_floor_violation_and_directionless_source_are_hard_rejected():
    source = _point(task="A", episode=1, x_mm=100, phase=0.4)
    successor = _point(
        task="B",
        episode=2,
        x_mm=180,
        phase=0.2,
        velocity_x_mm_s=5.0,
    )
    floor_rejected = validate_candidate(
        source,
        successor,
        duration_s=2.8,
        config=_validation_config(),
        transport_floor_mm=450.0,
    )
    assert not floor_rejected.feasible
    assert "transport_floor_violation" in floor_rejected.rejection_reason

    directionless = _point(
        task="A",
        episode=3,
        x_mm=100,
        phase=0.4,
        velocity_x_mm_s=0.0,
    )
    direction_rejected = validate_candidate(
        directionless,
        successor,
        duration_s=2.8,
        config=_validation_config(),
        transport_floor_mm=350.0,
    )
    assert not direction_rejected.feasible
    assert "source_direction_undefined" in direction_rejected.rejection_reason


def test_episode_manifest_orientation_support_is_enforced_by_outer_gate():
    value = episode_manifest(
        "A_to_B",
        _candidate(0),
        support_radius_mm=100.0,
        support_orientation_radius_deg=10.0,
        prearm_radius_mm=40.0,
        commit_radius_mm=20.0,
        direction_cosine_minimum=0.7,
    )
    manifest = EpisodeHandoffManifest.from_mapping(value)
    assert manifest.source.support_frame == value["source"]["support_frame"]
    assert (
        manifest.successor.support_frame
        == value["successor"]["support_frame"]
    )
    evaluator = HandoffCompatibilityEvaluator(
        HandoffCompatibilityConfig(
            max_first_xyz_axis_delta_mm=10.0,
            max_first_rotation_delta_deg=10.0,
            max_prefix_velocity_mm_s=1000.0,
            max_prefix_acceleration_mm_s2=10000.0,
            max_crossfade_command_acceleration_mm_s2=10000.0,
            max_bridge_prefix_velocity_mismatch_mm_s=1000.0,
            max_crossfade_xyz_axis_step_mm=10.0,
            max_crossfade_rotation_step_deg=10.0,
        ),
        prefix_steps=15,
        action_hz=30.0,
    )
    pose = np.array(
        [*manifest.successor.nominal_position_mm, 0.0, 180.0, 0.0],
        dtype=np.float64,
    )
    valid, reasons, metrics = evaluator.outer_ready(
        manifest=manifest,
        actual_semantic=manifest.source.semantic,
        current_pose_mm_deg=pose,
    )
    assert not valid
    assert "outside_successor_orientation_support" in reasons
    assert metrics["successor_support_orientation_delta_deg"] > 10.0



def test_external_planner_keeps_semantic_mismatch_as_diagnostic():
    source = _point(task="A", episode=1, x_mm=100.0, phase=0.4)
    successor = _point(
        task="B",
        episode=2,
        x_mm=180.0,
        phase=0.2,
        held_object="red_block",
    )
    guarded = validate_candidate(
        source,
        successor,
        duration_s=2.8,
        config=_validation_config(),
        transport_floor_mm=350.0,
    )
    assert not guarded.feasible
    assert "held_object_mismatch" in guarded.rejection_reason

    external = validate_candidate(
        source,
        successor,
        duration_s=2.8,
        config=_validation_config(),
        transport_floor_mm=350.0,
        semantic_authority=SemanticAuthority.EXTERNAL_PLANNER,
    )
    assert external.feasible
    assert external.semantic_authority is SemanticAuthority.EXTERNAL_PLANNER
    assert "held_object_mismatch" in external.semantic_diagnostics
    assert "held_object_mismatch" not in external.rejection_reason

    value = episode_manifest(
        "A_to_B",
        external,
        support_radius_mm=100.0,
        support_orientation_radius_deg=10.0,
        prearm_radius_mm=40.0,
        commit_radius_mm=20.0,
        direction_cosine_minimum=0.7,
    )
    manifest = EpisodeHandoffManifest.from_mapping(value)
    assert manifest.semantic_authority is SemanticAuthority.EXTERNAL_PLANNER
    assert "held_object_mismatch" in manifest.semantic_diagnostics
    restored = EpisodeHandoffManifest.from_mapping(manifest.to_record())
    assert restored.semantic_authority is SemanticAuthority.EXTERNAL_PLANNER
