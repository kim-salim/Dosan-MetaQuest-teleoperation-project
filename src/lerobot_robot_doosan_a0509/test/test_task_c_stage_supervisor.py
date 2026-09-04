from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    BoundarySemanticState,
    HandoffBoundary,
)
from lerobot_robot_doosan_a0509.task_c_handoff.stage_supervisor import (
    GripperEventMode,
    StagePhaseSupervisor,
    StagePhaseSupervisorConfig,
    ZeroCostExecutionTail,
)


def _artifact(path: Path) -> Path:
    phase = np.asarray([0.0, 0.5, 1.0], dtype=np.float64)
    episodes = np.asarray([0, 1, 2], dtype=np.int64)
    xyz = np.asarray(
        [
            [[0, 0, 0], [100, 0, 0], [200, 0, 0]],
            [[0, 1, 0], [100, 1, 0], [200, 1, 0]],
            [[0, -1, 0], [100, -1, 0], [200, -1, 0]],
        ],
        dtype=np.float64,
    )
    np.savez(
        path,
        S2_phase=phase,
        S2_episode_ids=episodes,
        S2_episode_xyz_mm=xyz,
        S2_median_xyz_mm=np.median(xyz, axis=0),
    )
    return path


def _boundary(
    *,
    gripper: str,
    segment: str = "S2",
    phase: float = 0.5,
) -> HandoffBoundary:
    return HandoffBoundary(
        task="T4",
        segment=segment,
        phase=phase,
        support_episode=0,
        support_frame=10,
        nominal_position_mm=np.asarray([100, 0, 0], dtype=np.float64),
        nominal_orientation_quat_xyzw=np.asarray([0, 0, 0, 1], dtype=np.float64),
        nominal_velocity_mm_s=np.asarray([1, 0, 0], dtype=np.float64),
        semantic=BoundarySemanticState(
            gripper_state=gripper,
            held_object="none" if gripper == "open" else "blue_block",
            contact_mode="free_motion_assumed",
        ),
        support_radius_mm=100.0,
    )


def _supervisor(
    tmp_path: Path,
    *,
    gripper: str,
    event: GripperEventMode,
) -> StagePhaseSupervisor:
    return StagePhaseSupervisor(
        _boundary(gripper=gripper),
        StagePhaseSupervisorConfig(
            support_artifact_path=_artifact(tmp_path / f"{event.value}.npz"),
            persistence_ticks=3,
            support_distance_threshold_mm=5.0,
            gripper_event_mode=event,
        ),
    )


def _update(supervisor: StagePhaseSupervisor, *, closed: bool):
    return supervisor.update(
        tcp_position_mm=np.asarray([100, 0, 0], dtype=np.float64),
        tcp_velocity_mm_s=np.asarray([1, 0, 0], dtype=np.float64),
        gripper_closed=closed,
    )


def test_closed_then_open_does_not_lock_on_initial_open_approach(tmp_path: Path):
    supervisor = _supervisor(
        tmp_path,
        gripper="open",
        event=GripperEventMode.CLOSED_THEN_OPEN,
    )

    initial = _update(supervisor, closed=False)
    assert not initial.semantic_ready
    assert not initial.commit_ready
    _update(supervisor, closed=True)
    statuses = [_update(supervisor, closed=False) for _ in range(3)]

    assert statuses[-1].semantic_ready
    assert statuses[-1].phase_ready
    assert statuses[-1].support_ready
    assert statuses[-1].commit_ready


def test_closed_then_open_rearms_only_at_supported_prearm_geometry(
    tmp_path: Path,
):
    supervisor = StagePhaseSupervisor(
        _boundary(gripper="open", segment="S2", phase=1.0),
        StagePhaseSupervisorConfig(
            support_artifact_path=_artifact(tmp_path / "release_rearm.npz"),
            phase_half_width=0.05,
            prearm_extra_phase=0.45,
            persistence_ticks=3,
            support_distance_threshold_mm=5.0,
            gripper_event_mode=GripperEventMode.CLOSED_THEN_OPEN,
            rearm_closed_then_open_at_prearm=True,
        ),
    )

    # The learned T1 rollout may close, reopen, and regrasp early in S2.
    # Neither event is retained before the supported O1 prearm.
    supervisor.update(
        tcp_position_mm=np.asarray([0, 0, 0], dtype=np.float64),
        tcp_velocity_mm_s=None,
        gripper_closed=True,
    )
    early_open = supervisor.update(
        tcp_position_mm=np.asarray([0, 0, 0], dtype=np.float64),
        tcp_velocity_mm_s=None,
        gripper_closed=False,
    )
    assert not early_open.semantic_ready
    assert not early_open.prearmed
    assert supervisor.record()["semantic_armed"] is False
    assert supervisor.record()["seen_closed"] is False

    prearm_close = supervisor.update(
        tcp_position_mm=np.asarray([100, 0, 0], dtype=np.float64),
        tcp_velocity_mm_s=None,
        gripper_closed=True,
    )
    assert prearm_close.prearmed
    assert not prearm_close.semantic_ready
    assert supervisor.record()["semantic_armed"] is False
    assert supervisor.record()["prearm_closed_ticks"] == 1

    # A one-tick close is not stable and an intervening open resets it.
    supervisor.update(
        tcp_position_mm=np.asarray([100, 0, 0], dtype=np.float64),
        tcp_velocity_mm_s=None,
        gripper_closed=False,
    )
    assert supervisor.record()["semantic_armed"] is False
    assert supervisor.record()["prearm_closed_ticks"] == 0

    stable_closed = [
        supervisor.update(
            tcp_position_mm=np.asarray([100, 0, 0], dtype=np.float64),
            tcp_velocity_mm_s=None,
            gripper_closed=True,
        )
        for _ in range(3)
    ]
    assert all(not status.semantic_ready for status in stable_closed)
    assert supervisor.record()["semantic_armed"] is True
    assert supervisor.record()["seen_closed"] is True
    assert supervisor.record()["prearm_closed_ticks"] == 3

    released = [
        supervisor.update(
            tcp_position_mm=np.asarray([200, 0, 0], dtype=np.float64),
            tcp_velocity_mm_s=None,
            gripper_closed=False,
        )
        for _ in range(3)
    ]
    assert released[-1].estimated_phase == pytest.approx(1.0)
    assert released[-1].semantic_ready
    assert released[-1].support_ready
    assert released[-1].commit_ready


def test_open_then_closed_requires_the_acquisition_event(tmp_path: Path):
    supervisor = _supervisor(
        tmp_path,
        gripper="closed",
        event=GripperEventMode.OPEN_THEN_CLOSED,
    )

    before = _update(supervisor, closed=False)
    assert not before.semantic_ready
    statuses = [_update(supervisor, closed=True) for _ in range(3)]

    assert statuses[-1].semantic_ready
    assert statuses[-1].commit_ready


def test_phase_support_still_rejects_out_of_distribution_pose(tmp_path: Path):
    supervisor = _supervisor(
        tmp_path,
        gripper="closed",
        event=GripperEventMode.OPEN_THEN_CLOSED,
    )
    _update(supervisor, closed=False)
    statuses = [
        supervisor.update(
            tcp_position_mm=np.asarray([100, 50, 0], dtype=np.float64),
            tcp_velocity_mm_s=None,
            gripper_closed=True,
        )
        for _ in range(3)
    ]

    assert statuses[-1].phase_ready
    assert not statuses[-1].support_ready
    assert not statuses[-1].commit_ready

def _execution_tail() -> ZeroCostExecutionTail:
    return ZeroCostExecutionTail(
        source_operator="T7.acquire_from_drawer_top",
        semantic_exit_segment="S1",
        semantic_exit_phase=1.0,
        tracking_segment="S2",
        tracking_start_phase=0.0,
        collar_phase_low=0.25,
        collar_phase_high=0.75,
        commit_phase_low=0.45,
        commit_phase_high=0.55,
        nominal_phase=0.5,
        deadline_phase=0.75,
        phase_half_width=0.05,
        prearm_extra_phase=0.45,
        phase_points=3,
        episode_count=3,
        derivation_method="unit_test",
        same_segment_as_semantic_exit=False,
        collar_progress_mm_low=50.0,
        collar_progress_mm_high=150.0,
        support_residual_p90_limit_mm=2.0,
    )


def test_zero_cost_tail_prearms_at_grasp_then_commits_later_support(
    tmp_path: Path,
):
    artifact = _artifact(tmp_path / "tail.npz")
    supervisor = StagePhaseSupervisor(
        _boundary(gripper="closed", segment="S2", phase=0.5),
        StagePhaseSupervisorConfig(
            support_artifact_path=artifact,
            persistence_ticks=3,
            local_search_radius_indices=2,
            support_distance_threshold_mm=5.0,
            gripper_event_mode=GripperEventMode.OPEN_THEN_CLOSED,
            execution_tail=_execution_tail(),
        ),
    )

    before = supervisor.update(
        tcp_position_mm=np.asarray([0, 0, 0], dtype=np.float64),
        tcp_velocity_mm_s=None,
        gripper_closed=False,
    )
    grasp = supervisor.update(
        tcp_position_mm=np.asarray([0, 0, 0], dtype=np.float64),
        tcp_velocity_mm_s=None,
        gripper_closed=True,
    )
    assert not before.prearmed
    assert grasp.prearmed
    assert not grasp.commit_ready
    assert grasp.estimated_phase == pytest.approx(0.0)
    assert not supervisor.execution_collar_ready(grasp)
    assert not supervisor.adaptive_rebase_ready(grasp)

    statuses = [
        supervisor.update(
            tcp_position_mm=np.asarray([100, 0, 0], dtype=np.float64),
            tcp_velocity_mm_s=None,
            gripper_closed=True,
        )
        for _ in range(3)
    ]
    assert statuses[-1].commit_ready
    assert statuses[-1].estimated_phase == pytest.approx(0.5)
    assert supervisor.execution_collar_ready(statuses[-1])
    assert supervisor.adaptive_rebase_ready(statuses[-1])
    assert supervisor.execution_collar_bounds() == pytest.approx((0.25, 0.75))
    record = supervisor.record()
    assert record["segment"] == "S2"
    assert record["tracking_segment"] == "S2"
    assert record["runtime_source_reference_segment"] == "S2"
    assert record["semantic_exit_segment"] == "S1"
    assert record["execution_tail"]["zero_symbolic_cost"] is True


def test_zero_cost_tail_deadline_remains_active_after_commit(tmp_path: Path):
    supervisor = StagePhaseSupervisor(
        _boundary(gripper="closed", segment="S2", phase=0.5),
        StagePhaseSupervisorConfig(
            support_artifact_path=_artifact(tmp_path / "tail_deadline.npz"),
            persistence_ticks=1,
            local_search_radius_indices=2,
            support_distance_threshold_mm=5.0,
            gripper_event_mode=GripperEventMode.OPEN_THEN_CLOSED,
            execution_tail=_execution_tail(),
        ),
    )
    supervisor.update(
        tcp_position_mm=np.asarray([0, 0, 0], dtype=np.float64),
        tcp_velocity_mm_s=None,
        gripper_closed=False,
    )
    committed = supervisor.update(
        tcp_position_mm=np.asarray([100, 0, 0], dtype=np.float64),
        tcp_velocity_mm_s=None,
        gripper_closed=True,
    )
    assert committed.commit_ready
    late = supervisor.update(
        tcp_position_mm=np.asarray([200, 0, 0], dtype=np.float64),
        tcp_velocity_mm_s=None,
        gripper_closed=True,
    )
    assert late.estimated_phase == pytest.approx(1.0)
    assert late.deadline_exceeded
    assert "execution_tail_deadline_exceeded" in late.waiting_reasons


def test_execution_tail_manifest_must_match_runtime_tracking_source(
    tmp_path: Path,
):
    with pytest.raises(ValueError, match="tracking source differs"):
        StagePhaseSupervisor(
            _boundary(gripper="closed", segment="S1", phase=1.0),
            StagePhaseSupervisorConfig(
                support_artifact_path=_artifact(tmp_path / "mismatch.npz"),
                gripper_event_mode=GripperEventMode.OPEN_THEN_CLOSED,
                execution_tail=_execution_tail(),
            ),
        )



def test_same_segment_earlier_reference_phase_is_allowed_for_future_join(
    tmp_path: Path,
):
    supervisor = StagePhaseSupervisor(
        _boundary(gripper="closed", segment="S2", phase=0.25),
        StagePhaseSupervisorConfig(
            support_artifact_path=_artifact(tmp_path / "earlier_phase.npz"),
            gripper_event_mode=GripperEventMode.OPEN_THEN_CLOSED,
            execution_tail=_execution_tail(),
        ),
    )
    record = supervisor.record()
    assert record["runtime_source_reference_phase"] == pytest.approx(0.25)
    assert record["tracking_nominal_phase"] == pytest.approx(0.5)


def test_same_segment_future_reference_phase_is_rejected(
    tmp_path: Path,
):
    with pytest.raises(ValueError, match="reference phase is after"):
        StagePhaseSupervisor(
            _boundary(gripper="closed", segment="S2", phase=0.75),
            StagePhaseSupervisorConfig(
                support_artifact_path=_artifact(tmp_path / "future_phase.npz"),
                gripper_event_mode=GripperEventMode.OPEN_THEN_CLOSED,
                execution_tail=_execution_tail(),
            ),
        )


def _cross_segment_artifact(path: Path) -> Path:
    phase = np.asarray([0.0, 0.5, 1.0], dtype=np.float64)
    episodes = np.asarray([0, 1, 2], dtype=np.int64)
    offsets = np.asarray([[0, 0, 0], [0, 1, 0], [0, -1, 0]], dtype=np.float64)
    s2_median = np.asarray(
        [[0, 0, 0], [100, 0, 0], [200, 0, 0]], dtype=np.float64
    )
    s3_median = np.asarray(
        [[200, 0, 0], [300, 0, 0], [400, 0, 0]], dtype=np.float64
    )
    s2_xyz = s2_median[None, :, :] + offsets[:, None, :]
    s3_xyz = s3_median[None, :, :] + offsets[:, None, :]
    np.savez(
        path,
        S2_phase=phase,
        S2_episode_ids=episodes,
        S2_episode_xyz_mm=s2_xyz,
        S2_median_xyz_mm=s2_median,
        S3_phase=phase,
        S3_episode_ids=episodes,
        S3_episode_xyz_mm=s3_xyz,
        S3_median_xyz_mm=s3_median,
    )
    return path


def _open_cross_segment_execution_tail() -> ZeroCostExecutionTail:
    return ZeroCostExecutionTail(
        source_operator="T4.open_drawer",
        semantic_exit_segment="S2",
        semantic_exit_phase=1.0,
        tracking_segment="S3",
        tracking_start_phase=0.0,
        collar_phase_low=0.25,
        collar_phase_high=0.75,
        commit_phase_low=0.45,
        commit_phase_high=0.55,
        nominal_phase=0.5,
        deadline_phase=0.75,
        phase_half_width=0.05,
        prearm_extra_phase=0.45,
        phase_points=3,
        episode_count=3,
        derivation_method="unit_test_open_cross_segment",
        same_segment_as_semantic_exit=False,
        collar_progress_mm_low=50.0,
        collar_progress_mm_high=150.0,
        support_residual_p90_limit_mm=2.0,
    )


def test_cross_segment_open_event_gates_post_o1_tail_tracking(tmp_path: Path):
    supervisor = StagePhaseSupervisor(
        _boundary(gripper="open", segment="S3", phase=0.5),
        StagePhaseSupervisorConfig(
            support_artifact_path=_cross_segment_artifact(
                tmp_path / "open_cross_segment.npz"
            ),
            persistence_ticks=3,
            local_search_radius_indices=2,
            support_distance_threshold_mm=5.0,
            gripper_event_mode=GripperEventMode.CLOSED_THEN_OPEN,
            rearm_closed_then_open_at_prearm=True,
            semantic_event_phase_half_width=0.05,
            semantic_event_prearm_extra_phase=0.02,
            execution_tail=_open_cross_segment_execution_tail(),
        ),
    )

    # Being inside the S3 commit geometry before O1 must not prearm or lock S3.
    early_s3 = supervisor.update(
        tcp_position_mm=np.asarray([300, 0, 0], dtype=np.float64),
        tcp_velocity_mm_s=None,
        gripper_closed=False,
    )
    assert early_s3.phase_ready
    assert early_s3.support_ready
    assert not early_s3.prearmed
    assert not early_s3.semantic_ready
    assert supervisor.tracker._previous_phase_index is None

    # An early close/open away from the S2 O1 prearm geometry is forgotten.
    for closed in (True, False):
        supervisor.update(
            tcp_position_mm=np.asarray([0, 0, 0], dtype=np.float64),
            tcp_velocity_mm_s=None,
            gripper_closed=closed,
        )
    assert supervisor.record()["semantic_armed"] is False

    # Stable close arms O1; one driver-completed open latches it at S2 support.
    for _ in range(3):
        supervisor.update(
            tcp_position_mm=np.asarray([200, 0, 0], dtype=np.float64),
            tcp_velocity_mm_s=None,
            gripper_closed=True,
        )
    assert supervisor.record()["semantic_armed"] is True
    released = supervisor.update(
        tcp_position_mm=np.asarray([200, 0, 0], dtype=np.float64),
        tcp_velocity_mm_s=None,
        gripper_closed=False,
    )
    assert not released.commit_ready
    assert supervisor.record()["semantic_event_complete"] is True
    assert supervisor.record()["semantic_event_support"]["segment"] == "S2"

    # Only after O1 may the independent S3 tail window commit the transition.
    committed = [
        supervisor.update(
            tcp_position_mm=np.asarray([300, 0, 0], dtype=np.float64),
            tcp_velocity_mm_s=None,
            gripper_closed=False,
        )
        for _ in range(3)
    ]
    assert committed[-1].semantic_ready
    assert committed[-1].commit_ready
    record = supervisor.record()
    assert record["tracking_segment"] == "S3"
    assert record["semantic_exit_segment"] == "S2"
    assert record["separate_semantic_event_gate"] is True
