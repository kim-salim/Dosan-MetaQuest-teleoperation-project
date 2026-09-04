from __future__ import annotations

import ast
import inspect
import textwrap
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("lerobot")
pytest.importorskip("rclpy")

from lerobot_robot_doosan_a0509.task_c_live_entrypoint import (
    require_task_c_live_arguments,
)
from lerobot_robot_doosan_a0509.task_c_live_rollout import (
    ACTION_KEYS,
    LivePhase,
    TaskCLiveStrategy,
    TaskCLiveStrategyConfig,
    _blend_b_action_overlap,
    _inside_xyz_bounds,
    _writable_engine_snapshot,
)
from lerobot_robot_doosan_a0509.task_c_live_v2_rollout import (
    TaskCLiveV2StrategyConfig,
)
from lerobot_robot_doosan_a0509.topic_cache import TopicCache, TopicUnavailableError


def _valid_args() -> list[str]:
    return [
        "--strategy.type=task_c_live",
        "--strategy.acknowledge_uncertified_manifest=true",
        "--robot.mode=policy_live",
        "--return_to_initial_position=false",
        "--inference.type=rtc",
        "--inference.rtc.enabled=false",
        "--policy.n_action_steps=100",
    ]


def test_entrypoint_accepts_only_bounded_live_async_contract():
    require_task_c_live_arguments(_valid_args())
    replacements = {
        "policy_live": "policy_shadow",
        "task_c_live": "base",
        "return_to_initial_position=false": "return_to_initial_position=true",
        "inference.rtc.enabled=false": "inference.rtc.enabled=true",
        "acknowledge_uncertified_manifest=true": (
            "acknowledge_uncertified_manifest=false"
        ),
    }
    for old, new in replacements.items():
        with pytest.raises(ValueError):
            require_task_c_live_arguments(
                [argument.replace(old, new) for argument in _valid_args()]
            )


def test_strategy_never_changes_mux_source_or_live_state():
    tree = ast.parse(textwrap.dedent(inspect.getsource(TaskCLiveStrategy)))
    forbidden_literals = {
        "/vr/set_live_robot_output",
        "/control/select_lerobot",
        "/control/select_disabled",
    }
    string_values = {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }
    assert forbidden_literals.isdisjoint(string_values)


def test_live_config_is_bounded_and_validated():
    config = TaskCLiveStrategyConfig()
    assert config.representative_boundary_enabled is False
    assert config.b_execution_steps == 900
    assert config.b_execution_timeout_s == 30.0
    assert config.b_completion_mode == "legacy_release_settle"
    assert config.b_refresh_queue_threshold == 20
    assert config.b_refresh_overlap_steps == 15
    assert config.a_gripper_close_latch_threshold == 0.7
    assert config.b_endpoint_stop_before_final_refresh
    assert config.b_moving_overlap_primary_enabled is False
    assert config.b_endpoint_settle_position_tolerance_mm == 3.0
    assert config.b_endpoint_settle_velocity_tolerance_mm_s == 15.0
    assert config.b_overlap_search_max_skip_steps == 15
    assert config.b_stopped_endpoint_direct_handoff_enabled is False
    assert config.b_stopped_endpoint_position_bridge_enabled is False
    assert config.b_tail_duration_min_s == 0.2
    assert config.b_tail_duration_max_s == 2.5
    assert config.b_tail_duration_step_s == 0.1
    assert config.actual_tracking_position_tolerance_mm == 50.0
    assert config.bridge_ack_mode == "stop_and_wait"
    assert config.bridge_max_ack_lag_steps == 0
    assert config.actual_tracking_position_admission_margin_mm == 0.0
    assert (
        config.actual_tracking_position_tolerance_mm
        - config.actual_tracking_position_admission_margin_mm
        == 50.0
    )
    with pytest.raises(ValueError, match="b_execution_steps"):
        TaskCLiveStrategyConfig(b_execution_steps=2001)
    with pytest.raises(ValueError, match="position admission margin"):
        TaskCLiveStrategyConfig(
            actual_tracking_position_tolerance_mm=15.0,
            actual_tracking_position_admission_margin_mm=15.0,
        )
    with pytest.raises(ValueError, match="must be non-negative"):
        TaskCLiveStrategyConfig(
            actual_tracking_position_admission_margin_mm=-0.1,
        )
    with pytest.raises(ValueError, match="a_gripper_close_latch_threshold"):
        TaskCLiveStrategyConfig(a_gripper_close_latch_threshold=1.01)
    with pytest.raises(ValueError, match="tail duration bounds"):
        TaskCLiveStrategyConfig(
            b_tail_duration_min_s=1.0,
            b_tail_duration_max_s=0.5,
        )
    with pytest.raises(ValueError, match="b_overlap_search_max_skip_steps"):
        TaskCLiveStrategyConfig(b_overlap_search_max_skip_steps=-1)
    with pytest.raises(ValueError, match="bridge_ack_mode"):
        TaskCLiveStrategyConfig(bridge_ack_mode="unbounded")
    with pytest.raises(ValueError, match="bridge_max_ack_lag_steps"):
        TaskCLiveStrategyConfig(bridge_max_ack_lag_steps=2)
    with pytest.raises(ValueError, match="bounded_pipeline requires"):
        TaskCLiveStrategyConfig(
            bridge_ack_mode="bounded_pipeline",
            bridge_max_ack_lag_steps=0,
        )
    with pytest.raises(ValueError, match="representative_boundary_enabled"):
        TaskCLiveStrategyConfig(representative_boundary_enabled=1)
    with pytest.raises(
        ValueError,
        match="b_stopped_endpoint_direct_handoff_enabled",
    ):
        TaskCLiveStrategyConfig(b_stopped_endpoint_direct_handoff_enabled=1)
    with pytest.raises(
        ValueError,
        match="b_moving_overlap_primary_enabled",
    ):
        TaskCLiveStrategyConfig(b_moving_overlap_primary_enabled=1)
    with pytest.raises(
        ValueError,
        match="b_stopped_endpoint_position_bridge_enabled",
    ):
        TaskCLiveStrategyConfig(b_stopped_endpoint_position_bridge_enabled=1)
    with pytest.raises(ValueError, match="b_completion_mode"):
        TaskCLiveStrategyConfig(b_completion_mode="policy_magic_done")
    with pytest.raises(ValueError, match="cannot use a completion position gate"):
        TaskCLiveStrategyConfig(
            b_completion_mode="successor_owned",
            b_completion_position_gate_enabled=True,
        )


def test_moving_overlap_primary_is_forwarded_without_disabling_endpoint_fallback():
    source = inspect.getsource(TaskCLiveStrategy.setup)
    assert 'handoff_values["b_moving_overlap_primary_enabled"]' in source
    assert 'handoff_values["b_endpoint_stop_before_final_refresh"]' in source


def test_endpoint_position_bridge_is_forwarded_and_live_pose_validated():
    setup_source = inspect.getsource(TaskCLiveStrategy.setup)
    step_source = inspect.getsource(TaskCLiveStrategy._step_bridge)
    assert (
        'handoff_values["b_stopped_endpoint_position_bridge_enabled"]'
        in setup_source
    )
    assert "b_stopped_endpoint_position_bridge_handoff" in step_source
    assert step_source.index("_assess_bridge_passthrough") < step_source.index(
        'if proposal.source == "BEZIER_BRIDGE":'
    )


def test_v1_wrapper_uses_endpoint_boundary_fresh_b_not_mid_bridge_seed():
    project_root = Path(__file__).resolve().parents[3]
    wrapper = (
        project_root / "scripts" / "run_task_c_representative_v1_live_candidate.sh"
    ).read_text(encoding="utf-8")
    generic = (
        project_root / "scripts" / "run_task_c_live_candidate.sh"
    ).read_text(encoding="utf-8")
    assert "export MOVING_OVERLAP_PRIMARY_ENABLED=false" in wrapper
    assert "export STOPPED_ENDPOINT_DIRECT_HANDOFF_ENABLED=true" in wrapper
    assert "export STOPPED_ENDPOINT_POSITION_BRIDGE_ENABLED=true" in wrapper
    assert "--strategy.b_stopped_endpoint_position_bridge_enabled=" in generic


def test_stopped_endpoint_direct_handoff_requires_full_pose_validation_before_send():
    source = inspect.getsource(TaskCLiveStrategy._step_bridge)
    direct_branch = source.split('if proposal.source == "ACT-B":', 1)[1]
    assert "b_stopped_endpoint_direct_handoff" in direct_branch
    assert (
        direct_branch.index("_assess_b_refresh_chunk")
        < direct_branch.index("_submit_live_command")
    )
    assert "enforce_stream_ramp=True" in direct_branch


def test_b_overlap_blend_preserves_old_gripper_during_atomic_splice():
    old = np.array(
        [
            [0.0, 0.0, 0.0, 0.0, 150.0, 0.0, 1.0],
            [1.0, 0.0, 0.0, 0.0, 150.0, 0.0, 1.0],
            [2.0, 0.0, 0.0, 0.0, 150.0, 0.0, 1.0],
            [3.0, 0.0, 0.0, 0.0, 150.0, 0.0, 1.0],
        ]
    )
    fresh = old.copy()
    fresh[:, 0] += 0.5
    fresh[:, 6] = 0.0

    blended = _blend_b_action_overlap(old, fresh, overlap_steps=2)

    assert blended.shape == fresh.shape
    assert 0.0 < blended[0, 0] < fresh[0, 0]
    assert old[1, 0] < blended[1, 0] < fresh[1, 0]
    np.testing.assert_allclose(blended[2:], fresh[2:])
    np.testing.assert_allclose(blended[:2, 6], 1.0)


def test_xyz_bounds_are_finite_and_inclusive():
    minimum = (1.0, 2.0, 3.0)
    maximum = (4.0, 5.0, 6.0)
    assert _inside_xyz_bounds(np.array(minimum), minimum, maximum)
    assert _inside_xyz_bounds(np.array(maximum), minimum, maximum)
    assert not _inside_xyz_bounds(np.array([4.01, 5.0, 6.0]), minimum, maximum)
    assert not _inside_xyz_bounds(np.array([np.nan, 3.0, 4.0]), minimum, maximum)


def test_act_b_release_is_stable_policy_driven_and_open_latched():
    config = TaskCLiveStrategyConfig(b_release_stable_steps=3)
    strategy = TaskCLiveStrategy(config)
    state = np.zeros(13, dtype=np.float64)
    state[6:9] = (379.0, -214.0, 300.0)
    state[12] = 1.0
    request_open = np.array(
        [379.0, -214.0, 300.0, 0.0, 150.0, 0.0, 0.0],
        dtype=np.float64,
    )

    assert strategy._prepare_b_action(request_open, state)[6] == 1.0
    assert strategy._prepare_b_action(request_open, state)[6] == 1.0
    assert strategy._prepare_b_action(request_open, state)[6] == 0.0
    assert strategy._b_release_authorized

    policy_reclose = request_open.copy()
    policy_reclose[6] = 1.0
    assert strategy._prepare_b_action(policy_reclose, state)[6] == 0.0


def test_act_b_release_position_gate_is_legacy_opt_in_only():
    config = TaskCLiveStrategyConfig(
        b_release_stable_steps=3,
        b_release_position_gate_enabled=True,
    )
    strategy = TaskCLiveStrategy(config)
    state = np.zeros(13, dtype=np.float64)
    state[6:9] = (379.0, -214.0, 300.0)
    state[12] = 1.0
    outside = np.array(
        [379.0, -214.0, 300.0, 0.0, 150.0, 0.0, 0.0],
        dtype=np.float64,
    )
    strategy._prepare_b_action(outside, state)
    strategy._prepare_b_action(outside, state)
    with pytest.raises(RuntimeError, match="outside demonstration envelope"):
        strategy._prepare_b_action(outside, state)
    assert not strategy._b_release_authorized


def test_act_b_release_requires_post_command_driver_confirmation():
    config = TaskCLiveStrategyConfig(b_release_observation_stable_frames=2)
    strategy = TaskCLiveStrategy(config)
    cache = TopicCache()
    strategy._raw_robot = SimpleNamespace(cache=cache)
    command_s = time.monotonic()
    strategy._b_release_command_sent_s = command_s
    cache.update("gripper_completed_command", "open", receive_time=command_s + 0.01)
    cache.update("gripper_driver_busy", False, receive_time=command_s + 0.01)
    cache.update("gripper_last_command_ok", True, receive_time=command_s + 0.01)
    capture = SimpleNamespace(capture_completed_s=command_s + 0.02)
    state = np.zeros(13, dtype=np.float64)

    strategy._update_b_release_confirmation(capture, state)
    assert not strategy._b_release_confirmed
    strategy._update_b_release_confirmation(capture, state)
    assert strategy._b_release_confirmed


def test_act_b_completion_requires_open_release_and_settle_without_position_gate():
    config = TaskCLiveStrategyConfig(
        b_completion_mode="legacy_release_settle",
        b_completion_stable_frames=3,
    )
    strategy = TaskCLiveStrategy(config)
    strategy._b_release_confirmed = True
    state = np.zeros(13, dtype=np.float64)
    state[6:9] = (379.0, -214.0, 300.0)
    completed = False
    speed = None
    for index in range(15):
        strategy._actual_history.add(index / 30.0, state[6:9])
        completed, speed = strategy._update_b_completion(state)

    assert completed
    assert speed == pytest.approx(0.0)

    state[12] = 1.0
    with pytest.raises(RuntimeError, match="re-closed"):
        strategy._update_b_completion(state)


def test_successor_owned_completion_does_not_end_on_release_pause():
    config = TaskCLiveStrategyConfig(
        b_completion_mode="successor_owned",
        b_completion_stable_frames=3,
    )
    strategy = TaskCLiveStrategy(config)
    strategy._b_release_confirmed = True
    state = np.zeros(13, dtype=np.float64)
    state[6:9] = (379.0, -214.0, 300.0)

    for index in range(60):
        strategy._actual_history.add(index / 30.0, state[6:9])
        completed, speed = strategy._update_b_completion(state)
        assert not completed
        assert speed is None
    assert strategy._b_completion_stable_frames == 0

    state[12] = 1.0
    with pytest.raises(RuntimeError, match="re-closed"):
        strategy._update_b_completion(state)


def test_act_b_completion_position_gate_is_legacy_opt_in_only():
    config = TaskCLiveStrategyConfig(
        b_completion_stable_frames=3,
        b_completion_position_gate_enabled=True,
    )
    strategy = TaskCLiveStrategy(config)
    strategy._b_release_confirmed = True
    state = np.zeros(13, dtype=np.float64)
    state[6:9] = (379.0, -214.0, 300.0)
    for index in range(15):
        strategy._actual_history.add(index / 30.0, state[6:9])
        completed, speed = strategy._update_b_completion(state)

    assert not completed
    assert speed is None


def test_task_a_gripper_close_latch_blocks_policy_reopen_only_after_open_seen():
    strategy = TaskCLiveStrategy(TaskCLiveStrategyConfig())

    target, newly_latched = strategy._task_a_gripper_target(
        0.8,
        open_seen=False,
    )
    assert target == 0.8
    assert not newly_latched
    assert not strategy._a_gripper_close_latched

    target, newly_latched = strategy._task_a_gripper_target(
        0.8,
        open_seen=True,
    )
    assert target == 1.0
    assert newly_latched
    assert strategy._a_gripper_close_latched

    target, newly_latched = strategy._task_a_gripper_target(
        0.0,
        open_seen=True,
    )
    assert target == 1.0
    assert not newly_latched


def test_task_a_gripper_latch_rejects_nonfinite_target():
    strategy = TaskCLiveStrategy(TaskCLiveStrategyConfig())
    with pytest.raises(ValueError, match="must be finite"):
        strategy._task_a_gripper_target(float("nan"), open_seen=True)


def test_rtc_engine_snapshot_owns_writable_camera_memory():
    camera = np.arange(24, dtype=np.uint8).reshape(2, 3, 4)
    camera.setflags(write=False)
    original = {"observation.images.front": camera, "metadata": [camera]}
    snapshot = _writable_engine_snapshot(original)
    assert snapshot is not original
    assert snapshot["observation.images.front"].flags.writeable
    assert snapshot["metadata"][0].flags.writeable
    assert not np.shares_memory(snapshot["observation.images.front"], camera)
    snapshot["observation.images.front"][0, 0, 0] = 255
    assert camera[0, 0, 0] == 0


def test_transition_boundary_uses_fresh_streamer_command_reference():
    strategy = TaskCLiveStrategy(TaskCLiveStrategyConfig())
    actual = np.array([400.0, 0.0, 450.0, 0.0, 150.0, 0.0])
    target = actual.copy()

    with pytest.raises(RuntimeError, match="fresh ServoL commanded_posx"):
        strategy._assert_downstream_passthrough(
            target,
            actual_pose=actual,
            source="BEZIER_BRIDGE",
            enforce_stream_ramp=True,
        )

    strategy._last_command_pose = target.copy()
    target[0] += strategy._downstream_contract.linear_ramp_mm_per_tick + 0.01
    with pytest.raises(RuntimeError, match="linear_ramp"):
        strategy._assert_downstream_passthrough(
            target,
            actual_pose=actual,
            source="BEZIER_BRIDGE",
            enforce_stream_ramp=True,
            streamer_command_pose=actual,
        )

    strategy._assert_downstream_passthrough(
        target,
        actual_pose=actual,
        source="ACT-A",
        enforce_stream_ramp=False,
    )


def test_send_array_rejects_stale_streamer_command_before_publish():
    strategy = TaskCLiveStrategy(TaskCLiveStrategyConfig())
    cache = TopicCache()
    sent: list[dict[str, float]] = []
    strategy._raw_robot = SimpleNamespace(cache=cache)
    ctx = SimpleNamespace(
        hardware=SimpleNamespace(
            robot_wrapper=SimpleNamespace(send_action=sent.append),
        )
    )
    action = np.array([400.0, 0.0, 450.0, 0.0, 150.0, 0.0, 1.0])
    cache.update(
        "commanded_posx",
        tuple(action[:6]),
        receive_time=time.monotonic() - 1.0,
    )

    with pytest.raises(TopicUnavailableError, match="stale topic commanded_posx"):
        strategy._send_array(
            ctx,
            action,
            "BEZIER_BRIDGE",
            actual_pose=action[:6],
        )
    assert sent == []

    cache.update("commanded_posx", tuple(action[:6]))
    strategy._send_array(
        ctx,
        action,
        "BEZIER_BRIDGE",
        actual_pose=action[:6],
    )
    assert len(sent) == 1
    assert strategy._last_bridge_commanded_receive_count == 2


def test_bridge_waits_for_matching_new_streamer_acknowledgement():
    strategy = TaskCLiveStrategy(TaskCLiveStrategyConfig())
    cache = TopicCache()
    pose = np.array([400.0, 0.0, 450.0, 0.0, 150.0, 0.0])
    first = cache.update("commanded_posx", tuple(pose))
    strategy._raw_robot = SimpleNamespace(cache=cache)
    strategy._last_command_pose = pose.copy()
    strategy._last_bridge_commanded_receive_count = first.receive_count

    assert not strategy._bridge_streamer_acknowledged()
    cache.update("commanded_posx", tuple(pose + np.array([0.1, 0, 0, 0, 0, 0])))
    assert not strategy._bridge_streamer_acknowledged()
    cache.update("commanded_posx", tuple(pose))
    assert strategy._bridge_streamer_acknowledged()
    assert strategy._bridge_ack_wait_cycles == 2
    np.testing.assert_allclose(strategy._last_acknowledged_bridge_pose, pose)


def test_first_bridge_tick_captures_fresh_streamer_anchor():
    strategy = TaskCLiveStrategy(TaskCLiveStrategyConfig())
    cache = TopicCache()
    pose = np.array([400.0, 0.0, 450.0, 0.0, 150.0, 0.0])
    cache.update("commanded_posx", tuple(pose))
    strategy._raw_robot = SimpleNamespace(cache=cache)

    assert strategy._last_bridge_commanded_receive_count is None
    assert strategy._bridge_streamer_acknowledged()
    np.testing.assert_allclose(strategy._last_acknowledged_bridge_pose, pose)


def test_bounded_bridge_ack_advances_at_30hz_with_one_step_lag():
    strategy = TaskCLiveStrategy(
        TaskCLiveStrategyConfig(
            bridge_ack_mode="bounded_pipeline",
            bridge_max_ack_lag_steps=1,
        )
    )
    cache = TopicCache()
    anchor = np.array([400.0, 0.0, 450.0, 0.0, 150.0, 0.0])
    first = cache.update("commanded_posx", tuple(anchor))
    strategy._raw_robot = SimpleNamespace(cache=cache)
    strategy._reset_bridge_ack_tracking(anchor, first.receive_count)

    g0 = anchor + np.array([3.0, 0, 0, 0, 0, 0])
    strategy._register_bridge_sent_command(
        g0,
        receive_count_before_send=first.receive_count,
    )
    # No fresh ACK yet, but exactly one outstanding target is permitted.
    assert strategy._bridge_streamer_acknowledged()

    g1 = anchor + np.array([6.0, 0, 0, 0, 0, 0])
    strategy._register_bridge_sent_command(
        g1,
        receive_count_before_send=first.receive_count,
    )
    # Two outstanding targets cannot advance without downstream progress.
    assert not strategy._bridge_streamer_acknowledged()

    cache.update("commanded_posx", tuple(g0))
    assert strategy._bridge_streamer_acknowledged()
    assert strategy._bridge_ack_acknowledged_sequence == 0
    assert strategy._bridge_ack_pipeline_advance_cycles == 2
    assert strategy._bridge_ack_lag_hold_cycles == 1
    assert strategy._bridge_ack_max_decision_lag_steps == 2
    assert strategy._bridge_ack_max_outstanding_after_send_steps == 2


def test_bounded_bridge_ack_mismatch_holds_until_recognized_fresh_ack():
    strategy = TaskCLiveStrategy(
        TaskCLiveStrategyConfig(
            bridge_ack_mode="bounded_pipeline",
            bridge_max_ack_lag_steps=1,
        )
    )
    cache = TopicCache()
    anchor = np.array([400.0, 0.0, 450.0, 0.0, 150.0, 0.0])
    first = cache.update("commanded_posx", tuple(anchor))
    strategy._raw_robot = SimpleNamespace(cache=cache)
    strategy._reset_bridge_ack_tracking(anchor, first.receive_count)
    target = anchor + np.array([3.0, 0, 0, 0, 0, 0])
    strategy._register_bridge_sent_command(
        target,
        receive_count_before_send=first.receive_count,
    )

    cache.update(
        "commanded_posx",
        tuple(anchor + np.array([0.2, 0, 0, 0, 0, 0])),
    )
    assert not strategy._bridge_streamer_acknowledged()
    assert not strategy._bridge_streamer_acknowledged()
    assert strategy._bridge_ack_unresolved_mismatch

    cache.update("commanded_posx", tuple(target))
    assert strategy._bridge_streamer_acknowledged()
    assert not strategy._bridge_ack_unresolved_mismatch
    assert strategy._bridge_ack_pose_mismatch_cycles == 1


def test_bounded_bridge_ack_sustains_one_new_target_per_30hz_tick():
    strategy = TaskCLiveStrategy(
        TaskCLiveStrategyConfig(
            bridge_ack_mode="bounded_pipeline",
            bridge_max_ack_lag_steps=1,
        )
    )
    cache = TopicCache()
    anchor = np.array([400.0, 0.0, 450.0, 0.0, 150.0, 0.0])
    first = cache.update("commanded_posx", tuple(anchor))
    strategy._raw_robot = SimpleNamespace(cache=cache)
    strategy._reset_bridge_ack_tracking(anchor, first.receive_count)
    targets = [
        anchor + np.array([0.05 * (index + 1), 0, 0, 0, 0, 0])
        for index in range(120)
    ]
    strategy._register_bridge_sent_command(
        targets[0],
        receive_count_before_send=first.receive_count,
    )

    # Model the observed scheduler phase: at each decision the downstream ACK
    # is one generated target behind the most recently sent command.
    for next_index in range(1, len(targets)):
        if next_index >= 2:
            cache.update(
                "commanded_posx",
                tuple(targets[next_index - 2]),
            )
        assert strategy._bridge_streamer_acknowledged()
        baseline = cache.sample("commanded_posx")
        assert baseline is not None
        strategy._register_bridge_sent_command(
            targets[next_index],
            receive_count_before_send=baseline.receive_count,
        )

    assert strategy._bridge_ack_wait_cycles == 0
    assert strategy._bridge_ack_lag_hold_cycles == 0
    assert strategy._bridge_ack_sent_sequence == 119
    assert strategy._bridge_ack_max_decision_lag_steps == 1
    # At 30 Hz, 120 unique targets correspond to the configured four seconds.
    assert len(targets) / 30.0 == pytest.approx(4.0)


def test_t2_t3_four_second_wrapper_enables_bounded_ack_only_by_default():
    project_root = Path(__file__).resolve().parents[3]
    wrapper = (
        project_root / "scripts" / "run_task_c_t2_t3_v2_bridge4s.sh"
    ).read_text(encoding="utf-8")
    generic = (
        project_root / "scripts" / "run_task_c_async_handoff_v2_candidate.sh"
    ).read_text(encoding="utf-8")
    assert 'BRIDGE_ACK_MODE="${BRIDGE_ACK_MODE:-bounded_pipeline}"' in wrapper
    assert 'BRIDGE_MAX_ACK_LAG_STEPS="${BRIDGE_MAX_ACK_LAG_STEPS:-1}"' in wrapper
    assert 'B_COMPLETION_MODE="${B_COMPLETION_MODE:-successor_owned}"' in wrapper
    assert 'BRIDGE_ACK_MODE:-stop_and_wait' in generic
    assert 'B_COMPLETION_MODE:-successor_owned' in generic
    assert '--strategy.bridge_ack_mode=' in generic
    assert '--strategy.b_completion_mode=' in generic


def test_v2_defaults_to_successor_owned_completion():
    config = TaskCLiveV2StrategyConfig(
        handoff_episode_manifest="episode_manifest.json",
        v2_trace_jsonl_path="trace.jsonl",
    )
    assert config.b_completion_mode == "successor_owned"
    assert not config.b_completion_position_gate_enabled
    assert config.bridge_jerk_limit_mm_s3 is None
    assert config.bridge_integrated_squared_jerk_limit is None


def test_v13_runtime_profile_requires_exact_ramp_ack_and_dynamics():
    kwargs = {
        "handoff_episode_manifest": "episode_manifest.json",
        "v2_trace_jsonl_path": "trace.jsonl",
        "runtime_command_profile_id": "a0509_ramp8p5_ack_span2_v1",
        "downstream_control_hz": 30.0,
        "downstream_linear_ramp_mm_per_tick": 8.5,
        "downstream_orientation_ramp_deg_per_tick": 1.25,
        "bridge_ack_mode": "bounded_pipeline",
        "bridge_max_ack_lag_steps": 1,
        "max_prefix_velocity_mm_s": 300.0,
        "bridge_acceleration_limit_mm_s2": 4000.0,
        "bridge_jerk_limit_mm_s3": 4000.0,
        "bridge_integrated_squared_jerk_limit": 10000000.0,
        "max_crossfade_command_acceleration_mm_s2": 4000.0,
    }
    config = TaskCLiveV2StrategyConfig(**kwargs)
    assert config.downstream_linear_ramp_mm_per_tick == pytest.approx(8.5)

    with pytest.raises(
        ValueError,
        match="runtime command profile mismatch",
    ):
        TaskCLiveV2StrategyConfig(
            **{**kwargs, "downstream_linear_ramp_mm_per_tick": 8.4}
        )


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("bridge_jerk_limit_mm_s3", 0.0),
        ("bridge_integrated_squared_jerk_limit", -1.0),
    ],
)
def test_v2_rejects_invalid_optional_bridge_dynamics(field, value):
    kwargs = {
        "handoff_episode_manifest": "episode_manifest.json",
        "v2_trace_jsonl_path": "trace.jsonl",
        field: value,
    }
    with pytest.raises(ValueError, match=field):
        TaskCLiveV2StrategyConfig(**kwargs)


def test_tracking_admission_uses_exact_50_mm_position_limit():
    strategy = TaskCLiveStrategy(TaskCLiveStrategyConfig())
    actual = np.array([400.0, 0.0, 450.0, 0.0, 150.0, 0.0])
    candidate = actual.copy()
    candidate[0] += 50.0

    admitted, position_error, orientation_error = strategy._tracking_admission(
        actual,
        candidate,
    )
    assert admitted
    assert position_error == pytest.approx(50.0)
    assert orientation_error == pytest.approx(0.0)

    candidate[0] += 0.001
    admitted, position_error, _ = strategy._tracking_admission(actual, candidate)
    assert not admitted
    assert position_error == pytest.approx(50.001)


def test_tracking_backpressure_holds_then_commits_exactly_one_pending_command():
    strategy = TaskCLiveStrategy(TaskCLiveStrategyConfig())
    strategy.phase = LivePhase.BRIDGE
    cache = TopicCache()
    hold = np.array([400.0, 0.0, 450.0, 0.0, 150.0, 0.0, 1.0])
    candidate = hold.copy()
    candidate[0] += 6.0
    cache.update("commanded_posx", tuple(hold[:6]))
    strategy._raw_robot = SimpleNamespace(cache=cache)
    strategy._last_command_pose = hold[:6].copy()
    strategy._last_command_action = hold.copy()
    sent: list[dict[str, float]] = []
    ctx = SimpleNamespace(
        hardware=SimpleNamespace(
            robot_wrapper=SimpleNamespace(send_action=sent.append),
        )
    )

    lagging_actual = hold[:6].copy()
    lagging_actual[0] -= 45.0
    assert not strategy._submit_live_command(
        ctx,
        candidate,
        "BEZIER_BRIDGE",
        actual_pose=lagging_actual,
        enforce_stream_ramp=True,
    )
    assert strategy._pending_command is not None
    assert strategy._tracking_backpressure_events == 1
    assert strategy._tracking_backpressure_wait_cycles == 1
    assert strategy._tracking_hold_commands == 1
    assert len(sent) == 1
    np.testing.assert_allclose(list(sent[0].values()), hold)
    strategy._assert_tracking(lagging_actual)

    caught_up_actual = hold[:6].copy()
    caught_up_actual[0] -= 43.0
    assert strategy._flush_pending_command(
        ctx,
        actual_pose=caught_up_actual,
    )
    assert strategy._pending_command is None
    assert len(sent) == 2
    np.testing.assert_allclose(list(sent[1].values()), candidate)

    with pytest.raises(RuntimeError, match="actual position tracking error 16.000"):
        strict_strategy = TaskCLiveStrategy(
            TaskCLiveStrategyConfig(actual_tracking_position_tolerance_mm=15.0)
        )
        strict_strategy._last_command_pose = hold[:6].copy()
        beyond_strict_limit = hold[:6].copy()
        beyond_strict_limit[0] -= 16.0
        strict_strategy._assert_tracking(beyond_strict_limit)


def test_act_a_preserves_full_7d_command_for_bridge_tracking_hold():
    strategy = TaskCLiveStrategy(TaskCLiveStrategyConfig())
    outgoing = np.array(
        [400.0, 0.0, 450.0, 0.0, 150.0, 0.0, 0.8],
        dtype=np.float64,
    )
    strategy._interpolator = SimpleNamespace(
        needs_new_action=lambda: False,
        get=lambda: outgoing.copy(),
    )
    sent: list[dict[str, float]] = []
    ctx = SimpleNamespace(
        data=SimpleNamespace(ordered_action_keys=ACTION_KEYS),
        processors=SimpleNamespace(
            robot_action_processor=lambda payload: payload[0],
        ),
        hardware=SimpleNamespace(
            robot_wrapper=SimpleNamespace(send_action=sent.append),
        ),
    )

    result = strategy._send_next_a_action(
        ctx,
        processed_observation={},
        raw_observation={},
        actual_pose=outgoing[:6],
        gripper_close_latch_armed=True,
    )

    assert result is not None
    expected_hold = outgoing.copy()
    expected_hold[6] = 1.0
    np.testing.assert_allclose(strategy._last_command_action, expected_hold)
    np.testing.assert_allclose(list(sent[0].values()), expected_hold)

    cache = TopicCache()
    cache.update("commanded_posx", tuple(expected_hold[:6]))
    strategy._raw_robot = SimpleNamespace(cache=cache)
    strategy.phase = LivePhase.BRIDGE
    bridge_candidate = expected_hold.copy()
    bridge_candidate[0] += 3.0
    lagging_actual = expected_hold[:6].copy()
    lagging_actual[0] -= 48.0

    assert not strategy._submit_live_command(
        ctx,
        bridge_candidate,
        "BEZIER_BRIDGE",
        actual_pose=lagging_actual,
        enforce_stream_ramp=True,
    )
    assert strategy._pending_command is not None
    assert len(sent) == 2
    np.testing.assert_allclose(list(sent[1].values()), expected_hold)


def test_run_flushes_pending_command_before_consuming_another_policy_step():
    source = inspect.getsource(TaskCLiveStrategy.run)
    bridge_pending = source.index(
        "if self._pending_command is not None:",
        source.index("elif self.phase is LivePhase.BRIDGE:"),
    )
    bridge_step = source.index("self._step_bridge(ctx, capture, state)")
    b_pending = source.index(
        "if self._pending_command is not None:",
        source.index("elif self.phase is LivePhase.ACT_B:"),
    )
    b_step = source.index("self._step_b(ctx, capture, state)")
    assert bridge_pending < bridge_step
    assert b_pending < b_step


def test_bridge_orientation_uses_rate_limited_logical_clock():
    source = inspect.getsource(TaskCLiveStrategy._step_bridge)
    assert "self._coordinator.bridge_elapsed_s" in source
    assert "capture.capture_completed_s - self._coordinator.bridge_started_s" not in source


def test_bridge_replan_uses_acknowledged_command_state_for_atomic_splice():
    observation_source = inspect.getsource(TaskCLiveStrategy._runtime_observation)
    bridge_source = inspect.getsource(TaskCLiveStrategy._step_bridge)
    assert "acknowledged_command_position_mm=command_position" in observation_source
    assert "acknowledged_command_velocity_mm_s=command_velocity" in observation_source
    assert "ACT-B tail p0 differs from acknowledged commanded_posx" in bridge_source
    assert "ACT-B tail v0 differs from the acknowledged Bridge tangent" in bridge_source


def test_initial_bridge_anchors_and_holds_acknowledged_command_before_progress():
    source = inspect.getsource(TaskCLiveStrategy._begin_bridge)
    assert 'cache.require(\n            "commanded_posx"' in source
    assert "acknowledged_command_position_mm=streamer_pose[:3]" in source
    assert "acknowledged_command_velocity_mm_s=measured_velocity" in source
    assert "initial Bridge p0 differs from acknowledged commanded_posx" in source
    assert "initial_hold = full_action(" in source
    assert 'initial_hold,\n            "BEZIER_BRIDGE"' in source


def test_representative_boundary_preplan_is_command_free_and_final_plan_precedes_clear():
    preplan = inspect.getsource(
        TaskCLiveStrategy._read_only_representative_preplan
    )
    assert "_send_array(" not in preplan
    assert "send_action(" not in preplan
    assert ".pause(" not in preplan
    assert ".reset(" not in preplan
    assert "policy_queue_mutated=False" in preplan

    begin = inspect.getsource(TaskCLiveStrategy._begin_bridge)
    final_plan = begin.index(
        "final_validation = self._coordinator.initial_planner.plan("
    )
    representative_pause = begin.index(
        "self._engine.pause()",
        final_plan,
    )
    assert final_plan < representative_pause
