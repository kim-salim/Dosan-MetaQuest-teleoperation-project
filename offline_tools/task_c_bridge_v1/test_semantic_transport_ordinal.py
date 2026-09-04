from __future__ import annotations

import json

import numpy as np
import pytest

from lerobot_robot_doosan_a0509.interior_policy.execution_tail_reference import (
    _semantic_transport_transition_ordinal,
)
from offline_tools.task_c_bridge_v0.trajectory_states import Trajectory
from offline_tools.task_c_bridge_v1.representative_trajectory import (
    TransportWindowConfig,
    _transition_span,
    standardize_episode,
)


def _two_event_trajectory() -> Trajectory:
    frames = 12
    closed = np.asarray(
        [False, False, True, True, True, False, False, True, True, True, False, False]
    )
    return Trajectory(
        dataset="synthetic_two_event_task",
        episode=0,
        xyz_mm=np.stack(
            (
                np.arange(frames, dtype=np.float64),
                np.zeros(frames),
                np.full(frames, 100.0),
            ),
            axis=1,
        ),
        timestamp_s=np.arange(frames, dtype=np.float64) / 30.0,
        gripper_closed=closed,
        frame_index=np.arange(frames, dtype=np.int64),
    )


def test_transition_ordinal_pairs_each_close_with_its_next_open():
    trajectory = _two_event_trajectory()

    assert _transition_span(trajectory, 0) == (2, 5)
    assert _transition_span(trajectory, 1) == (7, 10)


def test_standardizer_can_select_second_semantic_transport():
    standardized = standardize_episode(
        _two_event_trajectory(),
        TransportWindowConfig(
            start_offset_frames=0,
            end_offset_frames=0,
            minimum_transport_clearance_mm=0.0,
            phase_points=5,
            velocity_window_frames=3,
            transition_ordinal=1,
        ),
    )

    assert standardized.close_index == 7
    assert standardized.open_index == 10
    assert standardized.source_start_index == 7
    assert standardized.source_end_index == 9


def test_semantic_c2_o2_segment_maps_to_second_gripper_span(tmp_path):
    graph = {
        "segments": [
            {
                "spec": {
                    "segment_id": "S4",
                    "start_anchor": "C2",
                    "end_anchor": "O2",
                }
            }
        ]
    }
    (tmp_path / "semantic_graph_v3.json").write_text(
        json.dumps(graph), encoding="utf-8"
    )
    support = tmp_path / "standardized_semantic_phase_trajectories.npz"

    assert _semantic_transport_transition_ordinal(support, "S4") == 1


def test_missing_transition_ordinal_is_rejected():
    with pytest.raises(ValueError, match="ordinal 2"):
        _transition_span(_two_event_trajectory(), 2)
