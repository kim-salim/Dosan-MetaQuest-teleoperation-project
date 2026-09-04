from __future__ import annotations
from collections import deque

from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("lerobot")
pytest.importorskip("rclpy")

from offline_tools.cross_task_handoff.authority import SemanticAuthority
from lerobot_robot_doosan_a0509.task_c_handoff.bridge_runtime import (
    BridgeGenerationError,
)
from lerobot_robot_doosan_a0509.task_c_handoff.source_phase import (
    SourcePhaseStatus,
)
from lerobot_robot_doosan_a0509.task_c_live_v2_rollout import (
    TaskCLiveV2Strategy,
)


def _ready_status() -> SourcePhaseStatus:
    return SourcePhaseStatus(
        estimated_phase=0.50,
        phase_index=50,
        phase_window_low=0.45,
        phase_window_high=0.55,
        distance_to_component_median_mm=25.0,
        distance_to_nearest_actual_support_mm=5.0,
        nearest_actual_support_episode=12,
        support_distance_threshold_mm=34.0,
        phase_ready=True,
        support_ready=True,
        semantic_ready=True,
        persistence_ticks=3,
        persistence_required=3,
        prearmed=True,
        just_prearmed=False,
        commit_ready=True,
        deadline_phase=0.55,
        deadline_exceeded=False,
        search_mode="local",
        search_low_index=45,
        search_high_index=55,
        tracker_latency_ms=0.02,
        waiting_reasons=(),
    )


class _Tracker:
    config = SimpleNamespace(prearm_phase_low=0.43)

    def update(self, **_kwargs):
        return _ready_status()


class _Bridge:
    def record(self):
        return {"steps": 84}


def _strategy() -> TaskCLiveV2Strategy:
    strategy = object.__new__(TaskCLiveV2Strategy)
    strategy.v2_config = SimpleNamespace(source_trigger_mode="phase_support")
    strategy._semantic_authority = SemanticAuthority.RUNTIME_GUARDED
    strategy._source_phase_tracker = _Tracker()
    strategy._source_phase_status = None
    strategy._source_phase_latency_ms = []
    strategy._prepared_bridge_commit = None
    strategy._source_bridge_last_failure = None
    strategy._v2_manifest = SimpleNamespace(
        source=SimpleNamespace(phase=0.5),
    )
    strategy._estimate_actual_boundary_velocity = lambda: np.zeros(3)
    strategy._event = lambda *_args, **_kwargs: None
    return strategy


def _inputs():
    capture = SimpleNamespace(
        tcp_position_mm=np.zeros(3),
        semantic_state=SimpleNamespace(gripper_closed=True),
        capture_completed_s=1.0,
    )
    state = np.zeros(13, dtype=np.float64)
    cut_status = SimpleNamespace(open_seen=True, ready=True)
    return capture, state, cut_status


def test_valid_phase_support_but_infeasible_bridge_does_not_commit():
    strategy = _strategy()
    strategy._prepare_bridge_commit = lambda *_args: (_ for _ in ()).throw(
        BridgeGenerationError(["curvature_limit"])
    )
    capture, state, cut_status = _inputs()

    ready = strategy._a_exit_commit_ready(None, capture, state, cut_status)

    assert not ready
    assert strategy._prepared_bridge_commit is None
    assert "curvature_limit" in strategy._source_bridge_last_failure


def test_valid_phase_support_and_feasible_bridge_allows_commit():
    strategy = _strategy()
    prepared = SimpleNamespace(bridge=_Bridge())
    strategy._prepare_bridge_commit = lambda *_args: prepared
    capture, state, cut_status = _inputs()

    ready = strategy._a_exit_commit_ready(None, capture, state, cut_status)

    assert ready
    assert strategy._prepared_bridge_commit is prepared
    assert strategy._source_bridge_last_failure is None


def test_bridge_is_prepared_before_act_a_engine_is_invalidated():
    strategy = object.__new__(TaskCLiveV2Strategy)
    order = []
    prepared = SimpleNamespace(
        bridge=_Bridge(),
        acknowledged_pose_mm_deg=np.zeros(6, dtype=np.float64),
        acknowledged_receive_count=7,
        measured_velocity_mm_s=np.zeros(3, dtype=np.float64),
        actual_semantic=SimpleNamespace(),
        actual_to_ack_position_error_mm=0.0,
        actual_to_ack_orientation_error_deg=0.0,
        capture_timestamp_s=1.0,
    )
    strategy._v2_manifest = SimpleNamespace(
        source=SimpleNamespace(phase=0.5),
        successor=SimpleNamespace(phase=0.7),
        successor_reference_phase_window=None,
        successor_interior_path_margin_mm=None,
        semantic_local_total_length_mm=None,
    )
    strategy._v2_runtime_config = SimpleNamespace()
    strategy._v2_coordinator = SimpleNamespace(
        mark_exit_commit=lambda **_kwargs: order.append("mark_exit"),
        mark_prepare_bridge=lambda **_kwargs: order.append("mark_prepare"),
        start_bridge=lambda *_args, **_kwargs: order.append("start_bridge"),
        tick=lambda **_kwargs: SimpleNamespace(),
    )
    strategy._engine = SimpleNamespace(
        pause=lambda: order.append("pause"),
        reset=lambda: order.append("reset"),
    )
    strategy._prepared_bridge_commit = None
    strategy._v2_acknowledged_command_history = deque(maxlen=2)
    strategy._v2_last_acknowledged_receive_count = None
    strategy._prepare_bridge_commit = lambda *_args: (
        order.append("prepare"),
        prepared,
    )[1]
    strategy._dispatch_v2_command = (
        lambda *_args, **_kwargs: order.append("dispatch")
    )
    strategy._event = lambda *_args, **_kwargs: order.append("event")
    strategy._source_phase_status = _ready_status()
    capture = SimpleNamespace(capture_completed_s=1.0, policy_input={})
    state = np.zeros(13, dtype=np.float64)

    strategy._begin_bridge(None, capture, state)

    assert order.index("prepare") < order.index("pause")
    assert order.index("prepare") < order.index("reset")
    assert order.index("pause") < order.index("dispatch")


def test_external_planner_phase_support_does_not_require_old_grasp_cut():
    strategy = _strategy()
    strategy._semantic_authority = SemanticAuthority.EXTERNAL_PLANNER

    class _ExternalTracker(_Tracker):
        def update(self, **kwargs):
            assert kwargs["semantic_ready"] is True
            return _ready_status()

    strategy._source_phase_tracker = _ExternalTracker()
    prepared = SimpleNamespace(bridge=_Bridge())
    strategy._prepare_bridge_commit = lambda *_args: prepared
    capture, state, cut_status = _inputs()
    cut_status.ready = False

    assert strategy._a_exit_commit_ready(None, capture, state, cut_status)
    assert strategy._prepared_bridge_commit is prepared
