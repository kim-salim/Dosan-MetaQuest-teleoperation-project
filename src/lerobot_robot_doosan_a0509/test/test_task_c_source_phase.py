from __future__ import annotations

import numpy as np
import pytest

from lerobot_robot_doosan_a0509.task_c_handoff.source_phase import (
    SourcePhaseSupportBank,
    SourcePhaseSupportConfig,
    SourcePhaseSupportTracker,
    clipped_phase_window,
)


def _support_bank(*, threshold_mm: float = 5.0) -> SourcePhaseSupportBank:
    phase = np.array([0.0, 0.5, 1.0], dtype=np.float64)
    episode_xyz = np.array(
        [
            [[200, 0, 0], [100, 0, 0], [300, 0, 0]],
            [[201, 0, 0], [101, 0, 0], [301, 0, 0]],
            [[202, 0, 0], [99, 0, 0], [302, 0, 0]],
            [[203, 0, 0], [102, 0, 0], [303, 0, 0]],
            [[204, 0, 0], [0, 0, 0], [304, 0, 0]],
        ],
        dtype=np.float64,
    )
    return SourcePhaseSupportBank(
        artifact_path="synthetic.npz",
        segment="S2",
        phase=phase,
        episode_ids=np.arange(5, dtype=np.int64),
        episode_xyz_mm=episode_xyz,
        median_xyz_mm=np.median(episode_xyz, axis=0),
        support_distance_threshold_mm=threshold_mm,
        support_threshold_source="configured",
        support_loo_quantile=0.95,
    )


def _tracker(*, threshold_mm: float = 5.0) -> SourcePhaseSupportTracker:
    return SourcePhaseSupportTracker(
        _support_bank(threshold_mm=threshold_mm),
        SourcePhaseSupportConfig(
            nominal_phase=0.5,
            phase_half_width=0.05,
            prearm_extra_phase=0.02,
            persistence_ticks=3,
            local_search_radius_indices=1,
            backward_tolerance=0.02,
        ),
    )


def _update(tracker: SourcePhaseSupportTracker, xyz: list[float]):
    return tracker.update(
        tcp_position_mm=np.asarray(xyz, dtype=np.float64),
        tcp_velocity_mm_s=np.zeros(3, dtype=np.float64),
        open_before_close_observed=True,
        gripper_closed=True,
        semantic_ready=True,
    )


def test_source_phase_window_is_clipped_at_segment_boundaries():
    assert clipped_phase_window(0.50, 0.05) == pytest.approx((0.45, 0.55))
    assert clipped_phase_window(0.02, 0.05) == pytest.approx((0.00, 0.07))


def test_median_far_but_actual_support_close_can_commit_after_persistence():
    tracker = _tracker()
    statuses = [_update(tracker, [1, 0, 0]) for _ in range(3)]
    first, final = statuses[0], statuses[-1]

    assert first.search_mode == "global"
    assert final.search_mode == "local"
    assert final.estimated_phase == pytest.approx(0.5)
    assert final.distance_to_component_median_mm == pytest.approx(99.0)
    assert final.distance_to_nearest_actual_support_mm == pytest.approx(1.0)
    assert final.nearest_actual_support_episode == 4
    assert final.phase_ready
    assert final.support_ready
    assert final.semantic_ready
    assert final.commit_ready


def test_median_and_actual_support_both_far_cannot_commit():
    tracker = _tracker()
    statuses = [_update(tracker, [50, 0, 0]) for _ in range(4)]
    assert all(not status.support_ready for status in statuses)
    assert all(not status.commit_ready for status in statuses)
    assert "source_actual_support_ood" in statuses[-1].waiting_reasons


def test_phase_outside_primary_window_cannot_commit():
    tracker = _tracker(threshold_mm=10.0)
    statuses = [_update(tracker, [200, 0, 0]) for _ in range(4)]
    assert statuses[-1].estimated_phase == pytest.approx(0.0)
    assert statuses[-1].support_ready
    assert not statuses[-1].phase_ready
    assert not statuses[-1].commit_ready


def test_presemantic_observation_does_not_poison_initial_phase_lock():
    tracker = _tracker()
    before = tracker.update(
        tcp_position_mm=np.array([200.0, 0.0, 0.0]),
        open_before_close_observed=True,
        gripper_closed=False,
        semantic_ready=False,
    )
    after = _update(tracker, [1, 0, 0])

    assert before.search_mode == "global"
    assert before.estimated_phase == pytest.approx(0.0)
    assert after.search_mode == "global"
    assert after.estimated_phase == pytest.approx(0.5)


def test_semantic_not_ready_resets_source_persistence():
    tracker = _tracker()
    _update(tracker, [1, 0, 0])
    _update(tracker, [1, 0, 0])
    status = tracker.update(
        tcp_position_mm=np.array([1.0, 0.0, 0.0]),
        open_before_close_observed=True,
        gripper_closed=True,
        semantic_ready=False,
    )
    assert status.persistence_ticks == 0
    assert not status.semantic_ready
    assert not status.commit_ready


def test_missed_primary_window_raises_deadline_status_without_late_commit():
    tracker = _tracker()
    for _ in range(3):
        status = _update(tracker, [50, 0, 0])
        assert not status.commit_ready
    status = _update(tracker, [300, 0, 0])

    assert status.estimated_phase == pytest.approx(1.0)
    assert status.deadline_exceeded
    assert not status.commit_ready
    assert "source_phase_deadline_exceeded" in status.waiting_reasons


def test_npz_loader_calibrates_nearest_support_not_median_residual(tmp_path):
    source = _support_bank()
    artifact = tmp_path / "semantic_support.npz"
    np.savez(
        artifact,
        S2_phase=source.phase,
        S2_episode_ids=source.episode_ids,
        S2_episode_xyz_mm=source.episode_xyz_mm,
        S2_median_xyz_mm=source.median_xyz_mm,
        # This deliberately huge median residual must not drive admission.
        S2_residual_p95_mm=np.full(source.phase.shape, 500.0),
    )
    loaded = SourcePhaseSupportBank.load(
        artifact,
        segment="S2",
        phase_window=(0.45, 0.55),
        support_distance_threshold_mm=None,
        support_loo_quantile=0.9,
    )
    assert loaded.support_threshold_source == "leave_one_out_q0.900"
    assert loaded.support_distance_threshold_mm < 500.0
    assert loaded.episode_ids.size == 5


def test_latched_window_survives_t2_like_phase_skip_until_deadline():
    phase = np.array([0.0, 0.53, 0.56, 0.59, 0.61, 0.66], dtype=np.float64)
    base_xyz = np.stack(
        [phase * 1000.0, np.zeros_like(phase), np.zeros_like(phase)],
        axis=1,
    )
    episode_xyz = np.stack(
        [base_xyz + np.array([0.0, offset, 0.0]) for offset in (-1.0, 0.0, 1.0)],
        axis=0,
    )
    bank = SourcePhaseSupportBank(
        artifact_path="synthetic_t2_skip.npz",
        segment="S2",
        phase=phase,
        episode_ids=np.arange(3, dtype=np.int64),
        episode_xyz_mm=episode_xyz,
        median_xyz_mm=np.median(episode_xyz, axis=0),
        support_distance_threshold_mm=5.0,
        support_threshold_source="configured",
        support_loo_quantile=0.95,
    )
    tracker = SourcePhaseSupportTracker(
        bank,
        SourcePhaseSupportConfig(
            nominal_phase=0.57,
            phase_half_width=0.03,
            prearm_extra_phase=0.04,
            persistence_ticks=3,
            local_search_radius_indices=2,
            backward_tolerance=0.02,
            deadline_extra_phase=0.05,
            latch_phase_ready_until_deadline=True,
        ),
    )

    statuses = []
    for value in (0.56, 0.59, 0.61):
        statuses.append(
            tracker.update(
                tcp_position_mm=np.array([value * 1000.0, 0.0, 0.0]),
                open_before_close_observed=True,
                gripper_closed=True,
                semantic_ready=True,
            )
        )

    assert statuses[0].raw_phase_ready
    assert statuses[1].raw_phase_ready
    assert not statuses[2].raw_phase_ready
    assert statuses[2].phase_window_latched
    assert statuses[2].phase_ready
    assert statuses[2].persistence_ticks == 3
    assert statuses[2].commit_ready
    assert not statuses[2].deadline_exceeded

    late = tracker.update(
        tcp_position_mm=np.array([660.0, 0.0, 0.0]),
        open_before_close_observed=True,
        gripper_closed=True,
        semantic_ready=True,
    )
    assert not late.phase_ready
    assert late.deadline_exceeded
