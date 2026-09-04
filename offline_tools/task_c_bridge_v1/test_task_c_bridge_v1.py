from __future__ import annotations

import numpy as np
import pytest

from offline_tools.task_c_bridge_v0.trajectory_states import Trajectory
from offline_tools.task_c_bridge_v1.representative_trajectory import (
    TransportWindowConfig,
    build_representative_trajectory,
)
from offline_tools.task_c_bridge_v1.representative_optimizer import (
    RepresentativeBridgeOptimizer,
)
from offline_tools.task_c_bridge_v1.runtime_boundary import (
    RepresentativeBoundaryContract,
    RepresentativeBoundaryTracker,
)


def _trajectory(episode: int, lateral_offset: float = 0.0) -> Trajectory:
    frames = 60
    timestamp = np.arange(frames, dtype=np.float64) / 30.0
    progress = np.linspace(0.0, 1.0, frames)
    xyz = np.stack(
        (
            400.0 + 100.0 * progress,
            lateral_offset + 20.0 * progress,
            410.0 + 30.0 * np.sin(np.pi * progress),
        ),
        axis=1,
    )
    gripper = np.zeros(frames, dtype=bool)
    gripper[5:45] = True
    return Trajectory(
        dataset="synthetic",
        episode=episode,
        xyz_mm=xyz,
        timestamp_s=timestamp,
        gripper_closed=gripper,
        frame_index=np.arange(frames),
    )


def test_phase_alignment_builds_one_representative_without_z_scoring_xyz():
    trajectories = [_trajectory(0, -2.0), _trajectory(1, 0.0), _trajectory(2, 2.0)]
    representative = build_representative_trajectory(
        trajectories,
        role="a_exit",
        config=TransportWindowConfig(
            start_offset_frames=2,
            end_offset_frames=-2,
            minimum_transport_clearance_mm=0.0,
            minimum_z_mm=400.0,
            phase_points=11,
            velocity_window_frames=5,
            minimum_episodes=3,
        ),
    )

    assert representative.episode_count == 3
    assert representative.xyz_mm.shape == (11, 3)
    assert representative.covariance_mm2.shape == (11, 3, 3)
    assert representative.position(0.5)[0] > 400.0
    assert representative.position(0.5)[2] > 400.0
    assert representative.position(0.5)[1] == pytest.approx(
        np.median(representative.xyz_mm[:, 1]), abs=15.0
    )
    assert np.allclose(np.linalg.norm(representative.tangent, axis=1), 1.0)


def test_40mm_prearm_is_read_only_state_and_20mm_commit_is_debounced():
    contract = RepresentativeBoundaryContract(
        center_position_mm=np.array([0.0, 0.0, 0.0]),
        representative_velocity_mm_s=np.array([1.0, 0.0, 0.0]),
        representative_tangent=np.array([1.0, 0.0, 0.0]),
        prearm_radius_mm=40.0,
        commit_radius_mm=20.0,
        direction_cosine_minimum=0.7,
        approach_frames=3,
        commit_stable_frames=3,
        approach_epsilon_mm=0.0,
    )
    tracker = RepresentativeBoundaryTracker(contract)

    statuses = []
    for x in (-45.0, -39.0, -35.0, -30.0):
        statuses.append(
            tracker.update(
                tcp_position_mm=np.array([x, 0.0, 0.0]),
                tcp_velocity_mm_s=np.array([1.0, 0.0, 0.0]),
                open_before_close_observed=True,
                gripper_closed=True,
            )
        )
    assert statuses[-1].prearmed
    assert statuses[-1].just_prearmed
    assert not statuses[-1].commit_ready

    first = tracker.update(
        tcp_position_mm=np.array([-19.0, 0.0, 0.0]),
        tcp_velocity_mm_s=np.array([1.0, 0.0, 0.0]),
        open_before_close_observed=True,
        gripper_closed=True,
    )
    second = tracker.update(
        tcp_position_mm=np.array([-18.0, 0.0, 0.0]),
        tcp_velocity_mm_s=np.array([1.0, 0.0, 0.0]),
        open_before_close_observed=True,
        gripper_closed=True,
    )
    third = tracker.update(
        tcp_position_mm=np.array([-17.0, 0.0, 0.0]),
        tcp_velocity_mm_s=np.array([1.0, 0.0, 0.0]),
        open_before_close_observed=True,
        gripper_closed=True,
    )
    assert not first.commit_ready
    assert not second.commit_ready
    assert third.commit_ready


def test_boundary_rejects_wrong_direction_and_manifest_contract_is_strict():
    value = {
        "representative_boundary": {
            "enabled": True,
            "role": "a_exit",
            "boundary_id": "a:0.2",
            "center_position_mm": [1.0, 2.0, 3.0],
            "representative_velocity_mm_s": [0.0, 0.0, 10.0],
            "representative_tangent": [0.0, 0.0, 2.0],
            "prearm_radius_mm": 40.0,
            "commit_radius_mm": 20.0,
            "direction_cosine_minimum": 0.7,
            "approach_frames": 1,
            "commit_stable_frames": 1,
        }
    }
    contract = RepresentativeBoundaryContract.from_manifest(value)
    np.testing.assert_allclose(contract.representative_tangent, [0.0, 0.0, 1.0])
    tracker = RepresentativeBoundaryTracker(contract)
    status = tracker.update(
        tcp_position_mm=np.array([1.0, 2.0, 10.0]),
        tcp_velocity_mm_s=np.array([0.0, 0.0, -1.0]),
        open_before_close_observed=True,
        gripper_closed=True,
    )
    assert not status.prearmed
    assert "boundary_direction_not_aligned" in status.waiting_reasons

    value["representative_boundary"]["enabled"] = False
    with pytest.raises(ValueError, match="not enabled"):
        RepresentativeBoundaryContract.from_manifest(value)


def test_optimizer_uses_30hz_search_then_60hz_validation_and_one_representative_pair():
    representative_a = build_representative_trajectory(
        [_trajectory(0, -2.0), _trajectory(1, 0.0), _trajectory(2, 2.0)],
        role="a_exit",
        config=TransportWindowConfig(
            start_offset_frames=2,
            end_offset_frames=-2,
            minimum_transport_clearance_mm=0.0,
            minimum_z_mm=400.0,
            phase_points=11,
            velocity_window_frames=5,
            minimum_episodes=3,
        ),
    )
    representative_b = build_representative_trajectory(
        [
            _trajectory(0, -122.0),
            _trajectory(1, -120.0),
            _trajectory(2, -118.0),
        ],
        role="b_entry",
        config=TransportWindowConfig(
            start_offset_frames=2,
            end_offset_frames=-2,
            minimum_transport_clearance_mm=0.0,
            minimum_z_mm=400.0,
            phase_points=11,
            velocity_window_frames=5,
            minimum_episodes=3,
        ),
    )
    config = {
        "search": {
            "coarse_phase_step": 0.5,
            "refine_phase_step": 0.25,
            "refine_half_width": 0.25,
            "refine_seed_pairs": 1,
            "final_validation_top_n": 20,
            "final_validation_length_margin": 1.0,
            "robust_required_fraction": 0.0,
            "initial_terminal_velocity": "zero",
            "duration_search": {
                "bridge_duration_min_s": 2.0,
                "bridge_duration_max_s": 2.0,
                "bridge_duration_step_s": 0.5,
            },
        },
        "sampling": {
            "coarse_sample_hz": 30.0,
            "final_sample_hz": 60.0,
            "runtime_sample_hz": 60.0,
            "curvature_epsilon": 1e-9,
        },
        "feasibility": {
            "velocity_limit_mm_s": 10000.0,
            "acceleration_limit_mm_s2": 10000.0,
            "curvature_limit_per_mm": 10000.0,
            "jerk_limit_mm_s3": 1000000.0,
            "integrated_squared_jerk_limit": 1e12,
            "backtracking_ratio_limit": 10000.0,
            "enforce_payload_transport_floor": False,
        },
        "selection": {
            "near_shortest_delta": 0.005,
            "top_k": 3,
        },
        "live_boundary": {
            "prearm_radius_mm": 40.0,
            "commit_radius_mm": 20.0,
            "minimum_prearm_support_fraction": 0.0,
            "minimum_commit_support_fraction": 0.0,
        },
    }
    optimizer = RepresentativeBridgeOptimizer(
        representative_a=representative_a,
        representative_b=representative_b,
        config=config,
        workspace_min_mm=np.array([0.0, -500.0, 0.0]),
        workspace_max_mm=np.array([1000.0, 500.0, 1000.0]),
    )
    stages = []
    result = optimizer.optimize(record=lambda candidate: stages.append(candidate))

    assert result.coarse_evaluated == 9
    assert result.selected.sampling_hz == 60.0
    assert result.selected.search_stage == "final_validation"
    assert any(item.search_stage == "coarse" and item.sampling_hz == 30.0 for item in stages)
    assert len(result.top_k) <= 3


def test_boundary_semantic_override_preserves_geometry_but_removes_grasp_gate():
    contract = RepresentativeBoundaryContract(
        center_position_mm=np.zeros(3),
        representative_velocity_mm_s=np.array([1.0, 0.0, 0.0]),
        representative_tangent=np.array([1.0, 0.0, 0.0]),
        prearm_radius_mm=40.0,
        commit_radius_mm=20.0,
        direction_cosine_minimum=0.7,
        approach_frames=1,
        commit_stable_frames=1,
        approach_epsilon_mm=0.0,
    )
    tracker = RepresentativeBoundaryTracker(contract)
    for x_mm in (-30.0, -10.0):
        status = tracker.update(
            tcp_position_mm=np.array([x_mm, 0.0, 0.0]),
            tcp_velocity_mm_s=np.array([1.0, 0.0, 0.0]),
            open_before_close_observed=False,
            gripper_closed=False,
            semantic_ready=True,
        )
    assert status.commit_ready
    assert "open_before_close_not_observed" not in status.waiting_reasons
    assert "gripper_not_closed" not in status.waiting_reasons

    guarded = RepresentativeBoundaryTracker(contract)
    for x_mm in (-30.0, -10.0):
        guarded_status = guarded.update(
            tcp_position_mm=np.array([x_mm, 0.0, 0.0]),
            tcp_velocity_mm_s=np.array([1.0, 0.0, 0.0]),
            open_before_close_observed=False,
            gripper_closed=False,
        )
    assert not guarded_status.commit_ready
