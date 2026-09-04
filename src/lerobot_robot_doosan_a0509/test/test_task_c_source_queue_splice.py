from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("lerobot")

from lerobot.utils.action_interpolator import ActionInterpolator

from offline_tools.task_c_bridge_v0.runtime_policy import AsyncPolicySession
from lerobot_robot_doosan_a0509.task_c_handoff.bridge_runtime import (
    BridgeRuntimeLimits,
    BridgeRuntimeSnapshot,
)
from lerobot_robot_doosan_a0509.task_c_handoff.flexible_bridge import (
    AsyncFlexibleBridgeResult,
    FlexibleBridgeSearchConfig,
    FlexibleBridgeTemplate,
    rebase_cached_flexible_bridge,
    search_flexible_bridge_queue,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    BridgeAdmissionMode,
    EpisodeHandoffManifest,
    HandoffCompatibilityConfig,
    HandoffV2Config,
)
from lerobot_robot_doosan_a0509.task_c_handoff.source_queue_splice import (
    SourceActionQueueSnapshot,
    predictive_lookahead_steps,
    project_predictive_source_splice,
    snapshot_rtc_source_queue,
    source_snapshot_from_async_session,
)
from lerobot_robot_doosan_a0509.task_c_live_v2_rollout import (
    TaskCLiveV2Strategy,
    _BridgeSnapshotContext,
)
from quest_a0509_teleop.doosan_orientation import (
    doosan_zyz_deg_to_quaternion,
    quaternion_angle_deg,
)


def _runtime_snapshot() -> BridgeRuntimeSnapshot:
    pose = np.asarray([0.0, 0.0, 300.0, 0.0, 150.0, 0.0])
    return BridgeRuntimeSnapshot(
        timestamp_s=10.0,
        actual_pose_mm_deg=pose,
        acknowledged_pose_mm_deg=pose,
        actual_velocity_mm_s=np.zeros(3),
        gripper_target=1.0,
    )


class _Queue:
    def __init__(self, actions: np.ndarray) -> None:
        self.lock = threading.Lock()
        self.queue = torch.as_tensor(actions, dtype=torch.float32)
        self.last_index = 0
        self._a0509_live_generation = 7


def test_nominal_120ms_latency_selects_six_tick_splice() -> None:
    assert predictive_lookahead_steps(
        nominal_latency_s=0.12,
        observed_latencies_s=(),
        action_hz=30.0,
        margin_steps=2,
        minimum_steps=4,
        maximum_steps=8,
    ) == 6
    assert predictive_lookahead_steps(
        nominal_latency_s=0.12,
        observed_latencies_s=(0.18,),
        action_hz=30.0,
        margin_steps=2,
        minimum_steps=4,
        maximum_steps=8,
    ) == 8


def test_rtc_snapshot_is_bounded_and_lineage_changes_on_queue_merge() -> None:
    actions = np.zeros((20, 7), dtype=np.float64)
    actions[:, 0] = np.arange(20)
    actions[:, 3:6] = [0.0, 150.0, 0.0]
    actions[:, 6] = 1.0
    queue = _Queue(actions)
    engine = SimpleNamespace(action_queue=queue)
    interpolator = ActionInterpolator(multiplier=1)

    first = snapshot_rtc_source_queue(
        engine=engine,
        interpolator=interpolator,
        policy_id="T7",
        command_sequence=12,
        timestamp_s=10.0,
        action_hz=30.0,
        maximum_steps=9,
    )
    assert first is not None
    assert first.actions.shape == (9, 7)
    assert first.command_sequence == 12
    assert not first.actions.flags.writeable

    with queue.lock:
        queue.last_index += 3
    advanced = snapshot_rtc_source_queue(
        engine=engine,
        interpolator=interpolator,
        policy_id="T7",
        command_sequence=15,
        timestamp_s=10.1,
        action_hz=30.0,
        maximum_steps=2,
    )
    assert advanced is not None
    assert first.same_lineage(advanced)

    with queue.lock:
        queue.queue = queue.queue.clone()
    refreshed = snapshot_rtc_source_queue(
        engine=engine,
        interpolator=interpolator,
        policy_id="T7",
        command_sequence=15,
        timestamp_s=10.2,
        action_hz=30.0,
        maximum_steps=2,
    )
    assert refreshed is not None
    assert not first.same_lineage(refreshed)


def test_projection_matches_servol_axis_and_orientation_ramps() -> None:
    actions = np.zeros((8, 7), dtype=np.float64)
    actions[:, :3] = [100.0, -100.0, 350.0]
    actions[:, 3:6] = [30.0, 150.0, 0.0]
    actions[:, 6] = 1.0
    queue = _Queue(actions)
    snapshot = snapshot_rtc_source_queue(
        engine=SimpleNamespace(action_queue=queue),
        interpolator=ActionInterpolator(multiplier=1),
        policy_id="T7",
        command_sequence=20,
        timestamp_s=10.0,
        action_hz=30.0,
        maximum_steps=8,
    )
    assert snapshot is not None
    splice = project_predictive_source_splice(
        snapshot,
        _runtime_snapshot(),
        lookahead_steps=6,
        linear_ramp_mm_per_tick=7.5,
        orientation_ramp_deg_per_tick=1.25,
        velocity_window_steps=5,
    )
    np.testing.assert_allclose(
        splice.projected_pose_path_mm_deg[-1, :3],
        [45.0, -45.0, 345.0],
    )
    xyz_steps = np.abs(np.diff(splice.projected_pose_path_mm_deg[:, :3], axis=0))
    assert float(np.max(xyz_steps)) == pytest.approx(7.5)
    for before, after in zip(
        splice.projected_pose_path_mm_deg[:-1, 3:6],
        splice.projected_pose_path_mm_deg[1:, 3:6],
    ):
        angle = quaternion_angle_deg(
            doosan_zyz_deg_to_quaternion(before),
            doosan_zyz_deg_to_quaternion(after),
        )
        assert angle <= 1.25 + 1.0e-8
    assert splice.target_command_sequence == 26
    assert splice.predicted_snapshot.timestamp_s == 10.0
    assert splice.intended_timestamp_s == pytest.approx(10.2)


class _Backend:
    def reset(self) -> None:
        pass

    def infer(self, _observation) -> np.ndarray:
        actions = np.zeros((10, 7), dtype=np.float64)
        actions[:, 0] = np.arange(10)
        actions[:, 3:6] = [0.0, 150.0, 0.0]
        actions[:, 6] = 1.0
        return actions


def test_resident_policy_snapshot_is_atomic_and_generation_scoped() -> None:
    session = AsyncPolicySession("T3", _Backend(), action_hz=30.0)
    try:
        observed_s = time.monotonic()
        generation = session.prime(
            {},
            observation_timestamp_s=observed_s,
        )
        chunk = session.wait_for_chunk(generation, timeout_s=1.0)
        session.activate(chunk.generation)
        raw = session.active_queue_snapshot(maximum_steps=6)
        first = source_snapshot_from_async_session(
            raw,
            command_sequence=40,
            timestamp_s=time.monotonic(),
            maximum_steps=6,
        )
        assert first is not None
        assert first.generation == generation
        assert first.cursor == 0
        assert first.actions.shape == (6, 7)

        assert session.pop_action() is not None
        advanced = source_snapshot_from_async_session(
            session.active_queue_snapshot(maximum_steps=6),
            command_sequence=41,
            timestamp_s=time.monotonic(),
            maximum_steps=6,
        )
        assert advanced is not None
        assert first.same_lineage(advanced)
        assert advanced.cursor == 1

        session.deactivate_and_clear()
        assert session.active_queue_snapshot(maximum_steps=1) is None
    finally:
        session.close()


def _handoff_manifest() -> EpisodeHandoffManifest:
    semantic = {
        "gripper_state": "closed",
        "held_object": "blue_block",
        "contact_mode": "free_transport_assumed",
    }
    return EpisodeHandoffManifest.from_mapping(
        {
            "composition_id": "predictive_splice_test",
            "handoff_id": "t7_to_t3_predictive",
            "source": {
                "task": "T7",
                "segment": "S2",
                "phase": 0.18,
                "support_episode": 1,
                "support_frame": 10,
                "nominal_pose_mm_deg": [100, 0, 400, 0, 150, 0],
                "nominal_velocity_mm_s": [5, 0, 0],
                "support_radius_mm": 100,
                "semantic": semantic,
            },
            "successor": {
                "task": "T3",
                "segment": "S2",
                "phase": 0.2,
                "support_episode": 2,
                "support_frame": 20,
                "nominal_pose_mm_deg": [155, 10, 410, 0, 150, 0],
                "nominal_velocity_mm_s": [3, 0, 0],
                "support_radius_mm": 100,
                "semantic": semantic,
            },
            "bridge": {
                "duration_s": 2.8,
                "nominal_length_mm": 60,
                "transport_floor_mm": 300,
            },
            "generation": {
                "method": "flexible_reference",
                "bridge_admission_mode": "flexible_level2",
                "reference_only": True,
            },
            "validation": {
                "hard_filter_passed": False,
                "robot_executable": False,
                "dry_run_only": True,
                "ik_checked": False,
                "collision_checked": False,
            },
        }
    )


def _handoff_runtime() -> HandoffV2Config:
    return HandoffV2Config(
        enabled=True,
        bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
        handoff_window_steps=24,
        b_prefix_steps=15,
        crossfade_steps=15,
        compatibility=HandoffCompatibilityConfig(),
    )


def _handoff_limits() -> BridgeRuntimeLimits:
    return BridgeRuntimeLimits(
        workspace_min_mm=np.asarray([0, -500, 0]),
        workspace_max_mm=np.asarray([700, 500, 700]),
        velocity_limit_mm_s=1000,
        axis_velocity_limit_mm_s=1000,
        acceleration_limit_mm_s2=4000,
        curvature_limit_per_mm=0.25,
        jerk_limit_mm_s3=100000,
        integrated_squared_jerk_limit=1.0e12,
        backtracking_ratio_limit=10,
        linear_ramp_mm_per_tick=7.5,
        orientation_ramp_deg_per_tick=1.25,
    )


def test_prearm_rebase_uses_manifest_semantic_gripper():
    manifest = _handoff_manifest()
    runtime = _handoff_runtime()
    limits = _handoff_limits()
    actual_open = BridgeRuntimeSnapshot(
        timestamp_s=10.0,
        actual_pose_mm_deg=np.asarray([100, 0, 400, 0, 150, 0]),
        acknowledged_pose_mm_deg=np.asarray([100, 0, 400, 0, 150, 0]),
        actual_velocity_mm_s=np.asarray([5, 0, 0]),
        gripper_target=0.0,
    )
    source_queue = SourceActionQueueSnapshot(
        policy_id="T7",
        queue_kind="test",
        generation=3,
        revision_token="queue-3",
        cursor=0,
        command_sequence=100,
        timestamp_s=10.0,
        action_hz=30.0,
        actions=np.tile(
            np.asarray([108, 0, 400, 0, 150, 0, 0.0]),
            (8, 1),
        ),
    )
    splice = project_predictive_source_splice(
        source_queue,
        actual_open,
        lookahead_steps=4,
        linear_ramp_mm_per_tick=7.5,
        orientation_ramp_deg_per_tick=1.25,
        velocity_window_steps=5,
    )

    class Worker:
        inflight_generation = None
        requested_snapshot = None

        def poll(self, *, now_s):
            del now_s
            return AsyncFlexibleBridgeResult("idle", None)

        def request_rebase(
            self,
            template,
            requested_manifest,
            snapshot,
            runtime_config,
            requested_limits,
            search_config,
        ):
            del template, runtime_config, requested_limits, search_config
            assert requested_manifest is manifest
            self.requested_snapshot = snapshot
            self.inflight_generation = 1
            return 1

    worker = Worker()
    strategy = object.__new__(TaskCLiveV2Strategy)
    strategy._v2_manifest = manifest
    strategy._v2_runtime_config = runtime
    strategy._flexible_bridge_worker = worker
    strategy._flexible_bridge_search_config = FlexibleBridgeSearchConfig()
    strategy._flexible_bridge_template = SimpleNamespace(
        prepared_from_snapshot_timestamp_s=9.0
    )
    strategy._flexible_bridge_template_requested = True
    strategy._flexible_bridge_ready = None
    strategy._flexible_bridge_ready_splice = None
    strategy._flexible_bridge_ready_generation = None
    strategy._flexible_bridge_inflight_splice = None
    strategy._flexible_bridge_inflight_generation = None
    strategy._flexible_bridge_inflight_requested_s = None
    strategy._flexible_bridge_result_wall_latencies_s = []
    strategy._source_bridge_last_failure = None
    strategy._flexible_bridge_next_rebase_request_s = 0.0
    strategy.v2_config = SimpleNamespace(
        flexible_predictive_source_splice=True,
        flexible_bridge_rebase_retry_interval_s=0.1,
    )
    context = _BridgeSnapshotContext(
        snapshot=actual_open,
        acknowledged_receive_count=10,
        actual_semantic=manifest.source.semantic,
        actual_to_ack_position_error_mm=0.0,
        actual_to_ack_orientation_error_deg=0.0,
    )
    strategy._bridge_snapshot_context = lambda *_args: context
    strategy._bridge_runtime_limits = lambda: limits
    strategy._build_predictive_source_splice = lambda *_args: splice
    strategy._event = lambda *_args, **_kwargs: None

    strategy._try_prepare_flexible_bridge(
        SimpleNamespace(capture_completed_s=10.0),
        np.zeros(13),
        allow_commit=False,
    )

    assert worker.requested_snapshot is not None
    assert actual_open.gripper_target == 0.0
    assert manifest.source.semantic.gripper_target == 1.0
    assert worker.requested_snapshot.gripper_target == 1.0
    assert strategy._flexible_bridge_inflight_splice is not None
    assert (
        strategy._flexible_bridge_inflight_splice.predicted_snapshot.gripper_target
        == 1.0
    )


def _ready_strategy_fixture():
    manifest = _handoff_manifest()
    runtime = _handoff_runtime()
    limits = _handoff_limits()
    search_config = FlexibleBridgeSearchConfig()
    nominal = BridgeRuntimeSnapshot(
        timestamp_s=10.0,
        actual_pose_mm_deg=np.asarray([100, 0, 400, 0, 150, 0]),
        acknowledged_pose_mm_deg=np.asarray([100, 0, 400, 0, 150, 0]),
        actual_velocity_mm_s=np.asarray([5, 0, 0]),
        gripper_target=1.0,
    )
    template_search = search_flexible_bridge_queue(
        manifest,
        nominal,
        runtime,
        limits,
        search_config,
        live_mode=False,
    )
    assert template_search.valid
    template = FlexibleBridgeTemplate(
        handoff_id=manifest.handoff_id,
        search=template_search,
        prepared_from_snapshot_timestamp_s=10.0,
    )
    source_actions = np.tile(
        np.asarray([108, 0, 400, 0, 150, 0, 1.0]),
        (10, 1),
    )
    source_queue = SourceActionQueueSnapshot(
        policy_id="T7",
        queue_kind="test",
        generation=3,
        revision_token="queue-3",
        cursor=10,
        command_sequence=100,
        timestamp_s=10.0,
        action_hz=30.0,
        actions=source_actions,
    )
    splice = project_predictive_source_splice(
        source_queue,
        nominal,
        lookahead_steps=6,
        linear_ramp_mm_per_tick=7.5,
        orientation_ramp_deg_per_tick=1.25,
        velocity_window_steps=5,
    )
    ready = rebase_cached_flexible_bridge(
        template,
        manifest,
        splice.predicted_snapshot,
        runtime,
        limits,
        search_config,
    )
    assert ready.valid

    class Worker:
        inflight_generation = 99

        def poll(self, *, now_s):
            del now_s
            return AsyncFlexibleBridgeResult("idle", None)

    strategy = object.__new__(TaskCLiveV2Strategy)
    strategy._v2_manifest = manifest
    strategy._v2_runtime_config = runtime
    strategy._flexible_bridge_worker = Worker()
    strategy._flexible_bridge_search_config = search_config
    strategy._flexible_bridge_ready = ready
    strategy._flexible_bridge_ready_splice = splice
    strategy._flexible_bridge_ready_generation = 4
    strategy._flexible_bridge_inflight_splice = None
    strategy._flexible_bridge_inflight_generation = None
    strategy._flexible_bridge_inflight_requested_s = None
    strategy._flexible_bridge_result_wall_latencies_s = []
    strategy._source_bridge_last_failure = None
    strategy._flexible_bridge_next_rebase_request_s = 0.0
    strategy.v2_config = SimpleNamespace(
        flexible_predictive_source_splice=True,
        flexible_source_splice_late_tolerance_steps=1,
        flexible_bridge_max_result_age_sec=1.0,
        flexible_bridge_rebase_retry_interval_s=0.1,
    )
    context = _BridgeSnapshotContext(
        snapshot=splice.predicted_snapshot,
        acknowledged_receive_count=10,
        actual_semantic=manifest.source.semantic,
        actual_to_ack_position_error_mm=0.0,
        actual_to_ack_orientation_error_deg=0.0,
    )
    strategy._bridge_snapshot_context = lambda *_args: context
    strategy._bridge_runtime_limits = lambda: limits
    events = []
    strategy._event = lambda event, **details: events.append((event, details))
    capture = SimpleNamespace(capture_completed_s=10.15)
    return strategy, splice, capture, events


def test_ready_bridge_waits_for_predicted_sequence_then_commits() -> None:
    strategy, splice, capture, events = _ready_strategy_fixture()
    command_sequence = [splice.target_command_sequence - 1]
    strategy._source_command_sequence = lambda: command_sequence[0]
    strategy._predictive_source_lineage_matches = (
        lambda *_args: (True, splice.queue_snapshot)
    )
    assert strategy._try_prepare_flexible_bridge(
        capture,
        np.zeros(13),
        allow_commit=True,
    ) is None
    assert strategy._flexible_bridge_ready is not None

    command_sequence[0] = splice.target_command_sequence
    prepared = strategy._try_prepare_flexible_bridge(
        capture,
        np.zeros(13),
        allow_commit=True,
    )
    assert prepared is not None
    assert strategy._flexible_bridge_ready is None
    assert any(name == "flexible_bridge_commit_revalidation" for name, _ in events)


def test_changed_source_queue_lineage_discards_without_commit() -> None:
    strategy, splice, capture, events = _ready_strategy_fixture()
    strategy._source_command_sequence = lambda: splice.target_command_sequence
    changed = SourceActionQueueSnapshot(
        policy_id="T7",
        queue_kind="test",
        generation=4,
        revision_token="queue-4",
        cursor=16,
        command_sequence=splice.target_command_sequence,
        timestamp_s=10.15,
        action_hz=30.0,
        actions=np.tile(
            np.asarray([108, 0, 400, 0, 150, 0, 1.0]),
            (2, 1),
        ),
    )
    strategy._predictive_source_lineage_matches = (
        lambda *_args: (False, changed)
    )
    assert strategy._try_prepare_flexible_bridge(
        capture,
        np.zeros(13),
        allow_commit=True,
    ) is None
    assert strategy._flexible_bridge_ready is None
    discarded = [details for name, details in events if name == "predictive_source_splice_discarded"]
    assert discarded[-1]["reason"] == "source_queue_lineage_changed"
    assert discarded[-1]["act_a_queue_invalidated"] is False
