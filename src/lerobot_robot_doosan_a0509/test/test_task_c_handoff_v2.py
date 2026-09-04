from __future__ import annotations
from collections import deque

import threading
import time
from dataclasses import replace

import numpy as np
import pytest

pytest.importorskip("lerobot")

from offline_tools.cross_task_handoff.authority import SemanticAuthority
from offline_tools.task_c_bridge_v0.runtime_policy import (
    AsyncPolicySession,
    PolicyChunk,
)
from offline_tools.task_c_bridge_v0.bezier_bridge import (
    CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
)
from lerobot_robot_doosan_a0509.task_c_handoff.async_successor import (
    AsyncSuccessorController,
)
from lerobot_robot_doosan_a0509.task_c_handoff.bridge_runtime import (
    BridgeGenerationError,
    BridgeRuntimeLimits,
    BridgeRuntimeSnapshot,
    instantiate_bridge_queue,
)
from lerobot_robot_doosan_a0509.task_c_handoff.compatibility import (
    HandoffCompatibilityEvaluator,
)
from lerobot_robot_doosan_a0509.task_c_handoff.coordinator import (
    TaskCHandoffV2Coordinator,
)
from lerobot_robot_doosan_a0509.task_c_handoff.endpoint_fallback import (
    EndpointFallbackConfig,
    EndpointFallbackState,
    MultiEndpointFallbackCoordinator,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    BoundarySemanticState,
    EpisodeHandoffManifest,
    HandoffCompatibilityConfig,
    HandoffV2Config,
    HandoffV2State,
)
from lerobot_robot_doosan_a0509.task_c_handoff.soft_handoff import (
    SoftHandoffPlan,
    build_soft_handoff,
    quintic_smoothstep,
)
from quest_a0509_teleop.doosan_orientation import (
    doosan_zyz_deg_to_quaternion,
    quaternion_angle_deg,
)


def _manifest(*, duration_s: float = 2.8, support_radius_mm: float = 1000.0):
    semantic = {
        "gripper_state": "closed",
        "held_object": "blue_block",
        "contact_mode": "free_transport_assumed",
        "entry_preconditions": ["grasp_complete"],
    }
    return EpisodeHandoffManifest.from_mapping(
        {
            "schema_version": "a0509.task_c_handoff_episode.v2",
            "composition_id": "test_a_to_b",
            "handoff_id": "h_test",
            "source": {
                "task": "A",
                "segment": "S2",
                "phase": 0.6,
                "support_episode": 1,
                "support_frame": 101,
                "nominal_pose_mm_deg": [100, 0, 400, 0, 150, 0],
                "nominal_velocity_mm_s": [5, 0, 0],
                "support_radius_mm": support_radius_mm,
                "semantic": semantic,
            },
            "successor": {
                "task": "B",
                "segment": "S2",
                "phase": 0.2,
                "support_episode": 2,
                "support_frame": 202,
                "nominal_pose_mm_deg": [150, 10, 410, 1, 150, 0],
                "nominal_velocity_mm_s": [2, 0, 0],
                "support_radius_mm": support_radius_mm,
                "semantic": semantic,
            },
            "bridge": {
                "duration_s": duration_s,
                "nominal_length_mm": 52,
                "transport_floor_mm": 350,
            },
            "generation": {"method": "corridor_diverse"},
            "validation": {
                "hard_filter_passed": True,
                "robot_executable": False,
                "dry_run_only": True,
                "ik_checked": False,
                "collision_checked": False,
            },
        }
    )


def _limits() -> BridgeRuntimeLimits:
    return BridgeRuntimeLimits(
        workspace_min_mm=np.array([0, -500, 0]),
        workspace_max_mm=np.array([700, 500, 700]),
        velocity_limit_mm_s=1000,
        axis_velocity_limit_mm_s=1000,
        acceleration_limit_mm_s2=1000,
        curvature_limit_per_mm=1e6,
        jerk_limit_mm_s3=10000,
        integrated_squared_jerk_limit=1e9,
        backtracking_ratio_limit=10,
        linear_ramp_mm_per_tick=20,
        orientation_ramp_deg_per_tick=2,
    )


def _snapshot() -> BridgeRuntimeSnapshot:
    return BridgeRuntimeSnapshot(
        timestamp_s=1.0,
        actual_pose_mm_deg=np.array([100, 0, 400, 0, 150, 0]),
        acknowledged_pose_mm_deg=np.array([100, 0, 400, 0, 150, 0]),
        actual_velocity_mm_s=np.array([5, 0, 0]),
    )


def _command_history() -> deque[np.ndarray]:
    anchor = _snapshot().acknowledged_pose_mm_deg
    return deque((anchor.copy(), anchor.copy()), maxlen=2)



def _compatibility() -> HandoffCompatibilityConfig:
    return HandoffCompatibilityConfig(
        max_first_xyz_axis_delta_mm=1000,
        max_first_rotation_delta_deg=180,
        max_prefix_velocity_mm_s=10000,
        max_prefix_acceleration_mm_s2=100000,
        max_crossfade_command_acceleration_mm_s2=100000,
        max_bridge_prefix_velocity_mismatch_mm_s=10000,
        max_crossfade_xyz_axis_step_mm=20,
        max_crossfade_rotation_step_deg=2,
    )


def test_quintic_weight_has_c2_zero_boundary_derivatives():
    assert quintic_smoothstep(0.0) == 0.0
    assert quintic_smoothstep(1.0) == 1.0

    def first(s: float) -> float:
        return 30 * s**2 - 60 * s**3 + 30 * s**4

    def second(s: float) -> float:
        return 60 * s - 180 * s**2 + 120 * s**3

    assert first(0.0) == first(1.0) == 0.0
    assert second(0.0) == second(1.0) == 0.0


def test_bridge_queue_length_and_handoff_window_index():
    ticks = iter([10.0, 10.002])
    config = HandoffV2Config(
        enabled=True,
        handoff_window_steps=24,
        b_prefix_steps=15,
        crossfade_steps=15,
        compatibility=_compatibility(),
    )
    queue = instantiate_bridge_queue(
        _manifest(), _snapshot(), config, _limits(), clock=lambda: next(ticks)
    )
    assert queue.steps == 84
    assert queue.handoff_window_start_index == 60
    assert queue.in_handoff_window(60)
    assert not queue.in_handoff_window(59)
    np.testing.assert_allclose(queue.actions[-1, :3], [150, 10, 410])
    assert queue.actions[-1, 6] == 1.0
    assert queue.ik_checked is False
    assert queue.collision_checked is False


def test_runtime_reconstructs_regularized_manifest_and_audits_adjustment():
    manifest = replace(
        _manifest(),
        bridge_algorithm=CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
        minimum_tangent_handle_chord_ratio=0.04,
        maximum_endpoint_speed_adjustment_mm_s=12.0,
    )
    queue = instantiate_bridge_queue(
        manifest,
        _snapshot(),
        HandoffV2Config(
            enabled=True,
            handoff_window_steps=24,
            b_prefix_steps=15,
            crossfade_steps=15,
            compatibility=_compatibility(),
        ),
        _limits(),
        clock=lambda: 10.0,
    )
    assert queue.bridge_algorithm == CUBIC_BEZIER_TANGENT_REGULARIZED_V1
    assert queue.minimum_tangent_handle_chord_ratio == 0.04
    assert queue.successor_endpoint_speed_adjustment_mm_s > 0.0
    assert queue.record()["bridge_algorithm"] == (
        CUBIC_BEZIER_TANGENT_REGULARIZED_V1
    )


def test_runtime_regularization_speed_limit_fails_closed_before_execution():
    manifest = replace(
        _manifest(),
        bridge_algorithm=CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
        minimum_tangent_handle_chord_ratio=0.04,
        maximum_endpoint_speed_adjustment_mm_s=0.1,
    )
    with pytest.raises(
        BridgeGenerationError,
        match="endpoint_speed_adjustment_limit",
    ):
        instantiate_bridge_queue(
            manifest,
            _snapshot(),
            HandoffV2Config(
                enabled=True,
                handoff_window_steps=24,
                b_prefix_steps=15,
                crossfade_steps=15,
                compatibility=_compatibility(),
            ),
            _limits(),
            clock=lambda: 10.0,
        )


def test_bridge_queue_prevalidates_two_step_bounded_ack_span():
    lagged_limits = replace(_limits(), ack_pipeline_max_lag_steps=1)
    queue = instantiate_bridge_queue(
        _manifest(),
        _snapshot(),
        HandoffV2Config(
            enabled=True,
            handoff_window_steps=24,
            b_prefix_steps=15,
            crossfade_steps=15,
            compatibility=_compatibility(),
        ),
        lagged_limits,
        clock=lambda: 10.0,
    )
    assert queue.ack_span_steps == 2
    assert (
        queue.maximum_ack_span_axis_step_mm
        > queue.maximum_axis_step_mm
    )

    separating_limit = 0.5 * (
        queue.maximum_axis_step_mm
        + queue.maximum_ack_span_axis_step_mm
    )
    with pytest.raises(
        BridgeGenerationError,
        match="ack_pipeline_linear_ramp_limit",
    ):
        instantiate_bridge_queue(
            _manifest(),
            _snapshot(),
            HandoffV2Config(
                enabled=True,
                handoff_window_steps=24,
                b_prefix_steps=15,
                crossfade_steps=15,
                compatibility=_compatibility(),
            ),
            replace(
                lagged_limits,
                linear_ramp_mm_per_tick=separating_limit,
            ),
            clock=lambda: 10.0,
        )


def test_successor_phase_is_metadata_not_outer_or_prefix_authority():
    manifest = _manifest()
    far_phase_manifest = replace(
        manifest,
        successor=replace(manifest.successor, phase=0.99),
    )
    evaluator = HandoffCompatibilityEvaluator(
        _compatibility(),
        prefix_steps=15,
        action_hz=30.0,
    )
    current_pose = np.array([150, 10, 410, 1, 150, 0], dtype=np.float64)
    low_ready = evaluator.outer_ready(
        manifest=manifest,
        actual_semantic=manifest.successor.semantic,
        current_pose_mm_deg=current_pose,
    )
    far_ready = evaluator.outer_ready(
        manifest=far_phase_manifest,
        actual_semantic=far_phase_manifest.successor.semantic,
        current_pose_mm_deg=current_pose,
    )
    assert low_ready == far_ready
    assert far_ready[0]

    actions = np.tile(
        np.array([150, 10, 410, 1, 150, 0, 1], dtype=np.float64),
        (100, 1),
    )
    chunk = PolicyChunk("B", 1, 0.0, 0.01, 0.01, 30.0, actions)
    admission = evaluator.evaluate_prefix(
        chunk,
        bridge_reference_action=actions[0],
        bridge_velocity_mm_s=np.zeros(3, dtype=np.float64),
        expected_semantic=far_phase_manifest.successor.semantic,
    )
    assert admission.valid


def test_nominal_successor_phase_cannot_override_incompatible_fresh_prefix():
    manifest = _manifest()
    evaluator = HandoffCompatibilityEvaluator(
        replace(
            _compatibility(),
            max_first_xyz_axis_delta_mm=1.0,
        ),
        prefix_steps=15,
        action_hz=30.0,
    )
    actions = np.tile(
        np.array([250, 10, 410, 1, 150, 0, 1], dtype=np.float64),
        (100, 1),
    )
    chunk = PolicyChunk("B", 1, 0.0, 0.01, 0.01, 30.0, actions)
    admission = evaluator.evaluate_prefix(
        chunk,
        bridge_reference_action=np.array(
            [150, 10, 410, 1, 150, 0, 1],
            dtype=np.float64,
        ),
        bridge_velocity_mm_s=np.zeros(3, dtype=np.float64),
        expected_semantic=manifest.successor.semantic,
    )
    assert manifest.successor.phase == pytest.approx(0.2)
    assert not admission.valid
    assert "first_xyz_axis_delta" in admission.failure_reasons


def test_prefix_velocity_and_acceleration_calculation():
    config = replace(
        _compatibility(),
        max_prefix_acceleration_mm_s2=1,
    )
    evaluator = HandoffCompatibilityEvaluator(
        config,
        prefix_steps=3,
        action_hz=30,
    )
    actions = np.array(
        [
            [1, 0, 0, 0, 150, 0, 1],
            [2, 0, 0, 0, 150, 0, 1],
            [4, 0, 0, 0, 150, 0, 1],
        ],
        dtype=np.float64,
    )
    chunk = PolicyChunk("B", 1, 0, 0.01, 0.01, 30, actions)
    admission = evaluator.evaluate_prefix(
        chunk,
        bridge_reference_action=np.array([0, 0, 0, 0, 150, 0, 1]),
        bridge_velocity_mm_s=np.array([30, 0, 0]),
        expected_semantic=_manifest().successor.semantic,
    )
    assert admission.valid
    assert admission.dynamics.max_velocity_mm_s == 60.0
    assert admission.dynamics.max_acceleration_mm_s2 == 900.0
    assert admission.dynamics.bridge_prefix_velocity_mismatch_mm_s == 0.0
    assert "prefix_acceleration" not in admission.failure_reasons
    assert admission.record()["raw_prefix_acceleration_hard_reject"] is False



def test_raw_prefix_acceleration_limit_is_optional_but_command_limit_is_required():
    config = replace(_compatibility(), max_prefix_acceleration_mm_s2=None)
    config.require_live_thresholds()
    assert config.max_prefix_acceleration_mm_s2 is None
    with pytest.raises(ValueError, match="max_crossfade_command_acceleration"):
        replace(config, max_crossfade_command_acceleration_mm_s2=None).require_live_thresholds()


def test_crossfade_output_uses_slerp_and_preserves_discrete_gripper():
    bridge = np.array(
        [
            [0, 0, 0, 179, 20, 179, 1],
            [1, 0, 0, 180, 20, 180, 1],
            [2, 0, 0, 181, 20, 181, 1],
        ],
        dtype=np.float64,
    )
    successor = np.array(
        [
            [3, 0, 0, -179, -20, -179, 0],
            [4, 0, 0, -178, -20, -178, 0],
            [5, 0, 0, -177, -20, -177, 0],
        ],
        dtype=np.float64,
    )
    plan = build_soft_handoff(
        bridge,
        successor,
        steps=3,
        held_gripper_target=1.0,
    )
    np.testing.assert_allclose(plan.actions[-1, :3], successor[-1, :3])
    assert np.all(plan.actions[:, 6] == 1.0)
    final_angle = quaternion_angle_deg(
        doosan_zyz_deg_to_quaternion(plan.actions[-1, 3:6]),
        doosan_zyz_deg_to_quaternion(successor[-1, 3:6]),
    )
    assert final_angle < 1e-6


def test_prospective_crossfade_admission_checks_emitted_stream():
    steps = 15
    bridge = np.tile(
        np.array([0, 0, 0, 0, 150, 0, 1], dtype=np.float64),
        (steps, 1),
    )
    bridge[:, 0] = np.arange(steps, dtype=np.float64)
    successor = np.tile(
        np.array([30, 0, 0, 0, 150, 0, 1], dtype=np.float64),
        (100, 1),
    )
    successor[:, 0] += np.arange(100, dtype=np.float64)
    chunk = PolicyChunk("B", 1, 0, 0.01, 0.01, 30, successor)
    plan = build_soft_handoff(
        bridge,
        successor,
        steps=steps,
        held_gripper_target=1.0,
    )
    config = replace(
        _compatibility(),
        max_first_xyz_axis_delta_mm=75.0,
        max_prefix_acceleration_mm_s2=100000.0,
        max_crossfade_xyz_axis_step_mm=6.67,
        max_crossfade_rotation_step_deg=1.0,
    )
    evaluator = HandoffCompatibilityEvaluator(
        config,
        prefix_steps=steps,
        action_hz=30.0,
    )
    admission = evaluator.evaluate_prefix(
        chunk,
        bridge_reference_action=bridge[0],
        bridge_velocity_mm_s=np.array([30, 0, 0], dtype=np.float64),
        expected_semantic=_manifest().successor.semantic,
        soft_handoff_plan=plan,
        command_history_mm_deg=np.array(
            [[-2, 0, 0, 0, 150, 0], [-1, 0, 0, 0, 150, 0]], dtype=np.float64
        ),
        post_crossfade_action=successor[steps],
    )
    assert admission.valid
    assert admission.dynamics.first_xyz_axis_delta_mm[0] == 30.0
    assert admission.execution_dynamics is not None
    assert admission.execution_dynamics.max_xyz_axis_step_mm < 6.67
    assert admission.execution_dynamics.continuation_steps == 1
    assert admission.execution_dynamics.command_history_steps == 2
    expected_first_step = plan.actions[0, 0] - (-1.0)
    assert admission.execution_dynamics.first_xyz_axis_step_mm[0] == pytest.approx(
        expected_first_step
    )
    np.testing.assert_allclose(
        admission.execution_dynamics.previous_command_velocity_mm_s,
        [30, 0, 0],
    )
    np.testing.assert_allclose(
        admission.execution_dynamics.first_crossfade_velocity_mm_s,
        [expected_first_step * 30.0, 0, 0],
    )

    strict = HandoffCompatibilityEvaluator(
        replace(config, max_crossfade_xyz_axis_step_mm=2.0),
        prefix_steps=steps,
        action_hz=30.0,
    ).evaluate_prefix(
        chunk,
        bridge_reference_action=bridge[0],
        bridge_velocity_mm_s=np.array([30, 0, 0], dtype=np.float64),
        expected_semantic=_manifest().successor.semantic,
        soft_handoff_plan=plan,
        command_history_mm_deg=np.array(
            [[-2, 0, 0, 0, 150, 0], [-1, 0, 0, 0, 150, 0]], dtype=np.float64
        ),
        post_crossfade_action=successor[steps],
    )
    assert not strict.valid
    assert "crossfade_xyz_axis_step" in strict.failure_reasons

    command_accel_strict = HandoffCompatibilityEvaluator(
        replace(config, max_crossfade_command_acceleration_mm_s2=1.0),
        prefix_steps=steps,
        action_hz=30.0,
    ).evaluate_prefix(
        chunk,
        bridge_reference_action=bridge[0],
        bridge_velocity_mm_s=np.array([30, 0, 0], dtype=np.float64),
        expected_semantic=_manifest().successor.semantic,
        soft_handoff_plan=plan,
        command_history_mm_deg=np.array(
            [[-2, 0, 0, 0, 150, 0], [-1, 0, 0, 0, 150, 0]], dtype=np.float64
        ),
        post_crossfade_action=successor[steps],
    )
    assert not command_accel_strict.valid
    assert "crossfade_command_acceleration" in command_accel_strict.failure_reasons


def _ack_span_admission(
    *,
    history_x_mm: tuple[float, float],
    prospective_x_mm: tuple[float, float, float],
    span_limit_mm: float,
):
    actions = np.asarray(
        [
            [prospective_x_mm[0], 0, 400, 0, 150, 0, 1],
            [prospective_x_mm[1], 0, 400, 0, 150, 0, 1],
        ],
        dtype=np.float64,
    )
    continuation = np.asarray(
        [prospective_x_mm[2], 0, 400, 0, 150, 0, 1],
        dtype=np.float64,
    )
    plan = SoftHandoffPlan(
        actions=actions,
        weights=np.zeros(2, dtype=np.float64),
        b_actions_consumed=2,
        bridge_actions_consumed=2,
    )
    chunk_actions = np.concatenate((actions, continuation[None, :]), axis=0)
    chunk = PolicyChunk("B", 1, 0, 0.01, 0.01, 30, chunk_actions)
    config = replace(
        _compatibility(),
        max_first_xyz_axis_delta_mm=1000.0,
        max_prefix_velocity_mm_s=300.0,
        max_bridge_prefix_velocity_mismatch_mm_s=1000.0,
        max_crossfade_xyz_axis_step_mm=span_limit_mm,
        max_crossfade_rotation_step_deg=1.25,
        acknowledged_command_span_steps=2,
        max_acknowledged_xyz_axis_span_mm=span_limit_mm,
        max_acknowledged_rotation_span_deg=1.25,
        max_crossfade_command_acceleration_mm_s2=4000.0,
    )
    return HandoffCompatibilityEvaluator(
        config,
        prefix_steps=2,
        action_hz=30.0,
    ).evaluate_prefix(
        chunk,
        bridge_reference_action=actions[0],
        bridge_velocity_mm_s=np.zeros(3, dtype=np.float64),
        expected_semantic=_manifest().successor.semantic,
        soft_handoff_plan=plan,
        command_history_mm_deg=np.asarray(
            [
                [history_x_mm[0], 0, 400, 0, 150, 0],
                [history_x_mm[1], 0, 400, 0, 150, 0],
            ],
            dtype=np.float64,
        ),
        post_crossfade_action=continuation,
    )


def test_crossfade_ack_span_replays_last_7p692_mm_failure_and_v13_boundary():
    # Last run: consecutive commands were individually about 3.484/4.208 mm,
    # but one-command-lag ACK compared C[0] against cmd[k-1] (7.692 mm).
    rollback = _ack_span_admission(
        history_x_mm=(0.0, 3.483501112343141),
        prospective_x_mm=(7.692, 11.0, 14.0),
        span_limit_mm=7.5,
    )
    assert rollback.execution_dynamics is not None
    assert rollback.execution_dynamics.max_xyz_axis_step_mm == pytest.approx(
        4.208498887656859
    )
    assert (
        rollback.execution_dynamics.max_acknowledged_xyz_axis_span_mm
        == pytest.approx(7.692)
    )
    assert "crossfade_xyz_axis_step" not in rollback.failure_reasons
    assert "crossfade_acknowledged_xyz_axis_span" in rollback.failure_reasons

    v13 = _ack_span_admission(
        history_x_mm=(0.0, 3.483501112343141),
        prospective_x_mm=(7.692, 11.0, 14.0),
        span_limit_mm=8.5,
    )
    assert v13.valid
    assert v13.execution_dynamics is not None
    assert v13.execution_dynamics.acknowledged_command_span_steps == 2

    over_v13 = _ack_span_admission(
        history_x_mm=(0.0, 4.3),
        prospective_x_mm=(8.6, 12.9, 17.2),
        span_limit_mm=8.5,
    )
    assert not over_v13.valid
    assert "crossfade_xyz_axis_step" not in over_v13.failure_reasons
    assert "crossfade_acknowledged_xyz_axis_span" in over_v13.failure_reasons


class _DelayedBackend:
    def __init__(
        self,
        latency_s: float,
        *,
        fail: bool = False,
        release: threading.Event | None = None,
    ) -> None:
        self.latency_s = latency_s
        self.fail = fail
        self.release = release

    def reset(self) -> None:
        pass

    def infer(self, _observation):
        if self.release is not None:
            self.release.wait()
        else:
            time.sleep(self.latency_s)
        if self.fail:
            raise RuntimeError("fake B failure")
        actions = np.tile(
            np.array([150, 10, 410, 1, 150, 0, 1], dtype=np.float64),
            (100, 1),
        )
        return actions


def _coordinator(
    backend,
    *,
    duration_s: float = 2.8,
    window_steps: int = 60,
    max_age_s: float = 10.0,
):
    session = AsyncPolicySession("B", backend, action_hz=30)
    successor = AsyncSuccessorController(session, max_result_age_s=max_age_s)
    config = HandoffV2Config(
        enabled=True,
        handoff_window_steps=window_steps,
        b_prefix_steps=15,
        crossfade_steps=15,
        max_b_result_age_sec=max_age_s,
        request_retry_interval_steps=1,
        compatibility=_compatibility(),
    )
    evaluator = HandoffCompatibilityEvaluator(
        config.compatibility,
        prefix_steps=15,
        action_hz=30,
    )
    coordinator = TaskCHandoffV2Coordinator(
        manifest=_manifest(duration_s=duration_s),
        config=config,
        successor=successor,
        evaluator=evaluator,
    )
    ticks = iter([1.0, 1.001])
    bridge = instantiate_bridge_queue(
        coordinator.manifest,
        _snapshot(),
        config,
        _limits(),
        clock=lambda: next(ticks),
    )
    coordinator.start_bridge(bridge, timestamp_s=1.0)
    return coordinator, session, bridge


@pytest.mark.parametrize("latency_s", [0.05, 0.10, 0.20, 0.50])
def test_artificial_b_latency_never_blocks_bridge_tick(latency_s: float):
    coordinator, session, bridge = _coordinator(
        _DelayedBackend(latency_s),
        duration_s=20.0,
        window_steps=590,
        max_age_s=2.0,
    )
    semantic = coordinator.manifest.source.semantic
    command_count = 0
    maximum_tick_s = 0.0
    command_history = _command_history()
    started = time.monotonic()
    try:
        while time.monotonic() - started < 1.5:
            now = time.monotonic()
            tick_started = time.perf_counter()
            command = coordinator.tick(
                timestamp_s=now,
                actual_pose_mm_deg=np.array([150, 10, 410, 1, 150, 0]),
                actual_semantic=semantic,
                policy_input={"state": np.zeros(13)},
                command_history_mm_deg=np.stack(tuple(command_history), axis=0),
                observation_timestamp_s=now,
            )
            maximum_tick_s = max(maximum_tick_s, time.perf_counter() - tick_started)
            if command.action is not None:
                command_history.append(command.action[:6].copy())
                command_count += 1
            if coordinator.state is HandoffV2State.RUN_B:
                break
            time.sleep(0.002)
        assert command_count > 1
        assert maximum_tick_s < 0.02
        assert coordinator.bridge_index > 1
    finally:
        session.close()


def test_window_timeout_keeps_bridge_commands_then_requests_fallback():
    release = threading.Event()
    coordinator, session, bridge = _coordinator(
        _DelayedBackend(0, release=release),
        duration_s=1.0,
        window_steps=20,
    )
    semantic = coordinator.manifest.source.semantic
    command_history = _command_history()
    sources = []
    try:
        for index in range(bridge.steps + 2):
            command = coordinator.tick(
                timestamp_s=1.0 + index / 30.0,
                actual_pose_mm_deg=np.array([150, 10, 410, 1, 150, 0]),
                command_history_mm_deg=np.stack(tuple(command_history), axis=0),
                actual_semantic=semantic,
                policy_input={"state": np.zeros(13)},
                observation_timestamp_s=1.0 + index / 30.0,
            )
            sources.append(command.source)
            if command.action is not None:
                command_history.append(command.action[:6].copy())
            if command.endpoint_fallback_required:
                break
        assert "BEZIER_BRIDGE_V2" in sources
        assert sources[-1] == "ENDPOINT_FALLBACK"
        assert coordinator.state is HandoffV2State.ENDPOINT_FALLBACK
    finally:
        release.set()
        session.close()


class _EndpointBackend:
    def __init__(self, latency_s: float, endpoint: np.ndarray) -> None:
        self.latency_s = float(latency_s)
        self.endpoint = np.asarray(endpoint, dtype=np.float64)

    def reset(self) -> None:
        pass

    def infer(self, _observation):
        time.sleep(self.latency_s)
        return np.repeat(self.endpoint[None, :], 100, axis=0)


def _endpoint_runtime(
    latency_s: float,
    *,
    gripper_target: float = 1.0,
    max_age_s: float = 2.0,
    successor_action: np.ndarray | None = None,
    compatibility: HandoffCompatibilityConfig | None = None,
):
    endpoint = np.array(
        [150, 10, 410, 1, 150, 0, gripper_target],
        dtype=np.float64,
    )
    semantic = BoundarySemanticState(
        gripper_state="closed" if gripper_target >= 0.5 else "open",
        held_object="blue_block" if gripper_target >= 0.5 else "none",
        contact_mode="free_transport_assumed",
        entry_preconditions=("grasp_complete",) if gripper_target >= 0.5 else (),
    )
    base = _manifest()
    manifest = replace(
        base,
        source=replace(base.source, semantic=semantic),
        successor=replace(base.successor, semantic=semantic),
    )
    session = AsyncPolicySession(
        "B",
        _EndpointBackend(
            latency_s,
            endpoint if successor_action is None else successor_action,
        ),
        action_hz=30.0,
    )
    successor = AsyncSuccessorController(
        session,
        max_result_age_s=max_age_s,
    )
    evaluator = HandoffCompatibilityEvaluator(
        _compatibility() if compatibility is None else compatibility,
        prefix_steps=15,
        action_hz=30.0,
    )
    started = time.monotonic()
    runtime = MultiEndpointFallbackCoordinator(
        endpoint_action=endpoint,
        manifest=manifest,
        config=EndpointFallbackConfig(
            control_hz=30.0,
            settle_position_tolerance_mm=3.0,
            settle_velocity_tolerance_mm_s=15.0,
            settle_min_hold_s=0.01,
            settle_timeout_s=0.5,
            total_timeout_s=2.0,
            request_retry_interval_steps=1,
            crossfade_steps=15,
        ),
        successor=successor,
        evaluator=evaluator,
        started_s=started,
        moving_failure_reason="handoff_window_expired",
    )
    return runtime, session, endpoint, semantic


@pytest.mark.parametrize("latency_s", [0.05, 0.10, 0.20, 0.50])
def test_endpoint_fallback_holds_without_blocking_while_fresh_b_runs(latency_s):
    runtime, session, endpoint, semantic = _endpoint_runtime(latency_s)
    command_history = deque((endpoint[:6].copy(), endpoint[:6].copy()), maxlen=2)
    maximum_tick_s = 0.0
    hold_commands = 0
    transition = None
    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            now = time.monotonic()
            tick_started = time.perf_counter()
            command = runtime.tick(
                timestamp_s=now,
                actual_pose_mm_deg=endpoint[:6],
                acknowledged_pose_mm_deg=endpoint[:6],
                actual_velocity_mm_s=np.zeros(3),
                command_history_mm_deg=np.stack(tuple(command_history)),
                actual_semantic=semantic,
                policy_input={"state": np.zeros(13)},
                observation_timestamp_s=now,
            )
            maximum_tick_s = max(maximum_tick_s, time.perf_counter() - tick_started)
            if command.action is not None:
                command_history.append(command.action[:6].copy())
            hold_commands += int(command.source == "ENDPOINT_HOLD_V2")
            if command.transition_to_b_after_commit:
                transition = command
                break
            time.sleep(0.002)
        assert transition is not None
        assert runtime.state is EndpointFallbackState.RUN_B
        assert hold_commands > 1
        assert maximum_tick_s < 0.02
        assert session.active_generation == transition.successor_generation
    finally:
        session.close()


def test_endpoint_fallback_supports_open_gripper_edges():
    runtime, session, endpoint, semantic = _endpoint_runtime(
        0.01,
        gripper_target=0.0,
    )
    command_history = deque((endpoint[:6].copy(), endpoint[:6].copy()), maxlen=2)
    emitted_gripper = []
    try:
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            command = runtime.tick(
                timestamp_s=now,
                actual_pose_mm_deg=endpoint[:6],
                acknowledged_pose_mm_deg=endpoint[:6],
                actual_velocity_mm_s=np.zeros(3),
                command_history_mm_deg=np.stack(tuple(command_history)),
                actual_semantic=semantic,
                policy_input={},
                observation_timestamp_s=now,
            )
            if command.action is not None:
                emitted_gripper.append(float(command.action[6]))
                command_history.append(command.action[:6].copy())
            if command.transition_to_b_after_commit:
                break
            time.sleep(0.002)
        assert runtime.state is EndpointFallbackState.RUN_B
        assert emitted_gripper
        assert set(emitted_gripper) == {0.0}
    finally:
        session.close()


def test_endpoint_fallback_settle_timeout_fails_closed():
    runtime, session, endpoint, semantic = _endpoint_runtime(0.01)
    try:
        command = runtime.tick(
            timestamp_s=runtime.started_s + 0.51,
            actual_pose_mm_deg=endpoint[:6] + np.array([10, 0, 0, 0, 0, 0]),
            acknowledged_pose_mm_deg=endpoint[:6],
            actual_velocity_mm_s=np.zeros(3),
            command_history_mm_deg=np.stack((endpoint[:6], endpoint[:6])),
            actual_semantic=semantic,
            policy_input={},
            observation_timestamp_s=runtime.started_s + 0.51,
        )
        assert command.action is None
        assert command.failure_reason == "bridge_endpoint_settle_timeout"
        assert runtime.state is EndpointFallbackState.FAILED_HOLD
    finally:
        session.close()


def test_endpoint_fallback_rejects_stale_b_and_keeps_holding():
    runtime, session, endpoint, semantic = _endpoint_runtime(
        0.03,
        max_age_s=0.005,
    )
    history = np.stack((endpoint[:6], endpoint[:6]))
    hold_count = 0
    try:
        deadline = time.monotonic() + 0.15
        while time.monotonic() < deadline:
            now = time.monotonic()
            command = runtime.tick(
                timestamp_s=now,
                actual_pose_mm_deg=endpoint[:6],
                acknowledged_pose_mm_deg=endpoint[:6],
                actual_velocity_mm_s=np.zeros(3),
                command_history_mm_deg=history,
                actual_semantic=semantic,
                policy_input={},
                observation_timestamp_s=now,
            )
            hold_count += int(command.action is not None)
            assert not command.transition_to_b_after_commit
            time.sleep(0.002)
        assert hold_count > 1
        assert runtime.state is not EndpointFallbackState.RUN_B
        assert any(
            event.event == "endpoint_act_b_shadow_result"
            and event.details["stale"] is True
            for event in runtime.events
        )
    finally:
        session.close()


def test_endpoint_fallback_retries_incompatible_prefix_without_command_gap():
    endpoint = np.array([150, 10, 410, 1, 150, 0, 1], dtype=np.float64)
    far_successor = endpoint.copy()
    far_successor[0] += 50.0
    runtime, session, endpoint, semantic = _endpoint_runtime(
        0.005,
        successor_action=far_successor,
        compatibility=replace(
            _compatibility(),
            max_first_xyz_axis_delta_mm=5.0,
        ),
    )
    history = np.stack((endpoint[:6], endpoint[:6]))
    command_count = 0
    try:
        deadline = time.monotonic() + 0.15
        while time.monotonic() < deadline:
            now = time.monotonic()
            command = runtime.tick(
                timestamp_s=now,
                actual_pose_mm_deg=endpoint[:6],
                acknowledged_pose_mm_deg=endpoint[:6],
                actual_velocity_mm_s=np.zeros(3),
                command_history_mm_deg=history,
                actual_semantic=semantic,
                policy_input={},
                observation_timestamp_s=now,
            )
            command_count += int(command.action is not None)
            assert not command.transition_to_b_after_commit
            time.sleep(0.002)
        assert command_count > 1
        assert runtime.state is not EndpointFallbackState.RUN_B
        record = runtime.record()
        assert record["request_count"] >= 2
        assert record["rejection_count"] >= 1
        assert "first_xyz_axis_delta" in (
            record["last_admission"]["failure_reasons"]
        )
    finally:
        session.close()


def test_v2_coordinator_records_endpoint_fallback_takeover():
    coordinator, session, _bridge = _coordinator(
        _DelayedBackend(0.01),
    )
    try:
        coordinator.state = HandoffV2State.ENDPOINT_FALLBACK
        coordinator.mark_endpoint_takeover(
            timestamp_s=2.0,
            generation=3,
            crossfade_steps=15,
        )
        assert coordinator.state is HandoffV2State.RUN_B
        event = next(
            item
            for item in reversed(coordinator.events)
            if item.event == "act_b_full_takeover"
        )
        assert event.details["endpoint_fallback"] is True
        assert event.details["generation"] == 3
    finally:
        session.close()


def test_worker_exception_does_not_create_command_gap():
    coordinator, session, _bridge = _coordinator(
        _DelayedBackend(0.02, fail=True),
        duration_s=3.0,
        window_steps=80,
    )
    semantic = coordinator.manifest.source.semantic
    command_history = _command_history()
    commands_while_failure_routes = 0
    try:
        for index in range(40):
            command = coordinator.tick(
                timestamp_s=time.monotonic(),
                actual_pose_mm_deg=np.array([150, 10, 410, 1, 150, 0]),
                command_history_mm_deg=np.stack(tuple(command_history), axis=0),
                actual_semantic=semantic,
                policy_input={},
                observation_timestamp_s=time.monotonic(),
            )
            commands_while_failure_routes += int(command.action is not None)
            if command.action is not None:
                command_history.append(command.action[:6].copy())
            time.sleep(0.002)
        assert commands_while_failure_routes == 40
        assert any(event.event == "act_b_shadow_result" for event in coordinator.events)
    finally:
        session.close()


def test_generation_stale_result_is_rejected_by_existing_session_contract():
    session = AsyncPolicySession("B", _DelayedBackend(0.02), action_hz=30)
    try:
        first = session.prime({}, observation_timestamp_s=time.monotonic())
        second = session.prime({}, observation_timestamp_s=time.monotonic())
        assert second == first + 1
        deadline = time.monotonic() + 1.0
        accepted = None
        while time.monotonic() < deadline and accepted is None:
            accepted = session.poll()
            time.sleep(0.002)
        assert accepted is not None
        assert accepted.generation == second
        assert session.stats().stale_chunks_dropped >= 1
    finally:
        session.close()


def test_bridge_holds_actual_snapshot_gripper_instead_of_manifest_gripper():
    snapshot = replace(_snapshot(), gripper_target=0.0)
    queue = instantiate_bridge_queue(
        _manifest(),
        snapshot,
        HandoffV2Config(
            enabled=True,
            handoff_window_steps=24,
            b_prefix_steps=15,
            crossfade_steps=15,
            compatibility=_compatibility(),
        ),
        _limits(),
        clock=lambda: 10.0,
    )
    assert np.all(queue.actions[:, 6] == 0.0)
    assert queue.record()["gripper_target"] == 0.0


def test_external_planner_demotes_semantic_support_and_gripper_mismatch_only():
    manifest = replace(
        _manifest(support_radius_mm=1.0),
        semantic_authority=SemanticAuthority.EXTERNAL_PLANNER,
    )
    evaluator = HandoffCompatibilityEvaluator(
        replace(_compatibility(), max_first_xyz_axis_delta_mm=1.0),
        prefix_steps=15,
        action_hz=30.0,
        semantic_authority=SemanticAuthority.EXTERNAL_PLANNER,
    )
    actual_semantic = BoundarySemanticState(
        gripper_state="open",
        held_object="none",
        contact_mode="contact_manipulation_assumed",
    )
    pose = np.array([500, 0, 400, 0, 150, 0], dtype=np.float64)
    valid, reasons, metrics = evaluator.outer_ready(
        manifest=manifest,
        actual_semantic=actual_semantic,
        current_pose_mm_deg=pose,
    )
    assert valid
    assert reasons == ()
    assert "gripper_state_mismatch" in metrics["semantic_diagnostics"]
    assert "outside_successor_position_support" in metrics["support_diagnostics"]

    actions = np.tile(
        np.array([500, 0, 400, 0, 150, 0, 0], dtype=np.float64),
        (100, 1),
    )
    admission = evaluator.evaluate_prefix(
        PolicyChunk("B", 1, 0.0, 0.01, 0.01, 30.0, actions),
        bridge_reference_action=np.array([500, 0, 400, 0, 150, 0, 1]),
        bridge_velocity_mm_s=np.zeros(3),
        expected_semantic=manifest.successor.semantic,
    )
    assert admission.valid
    assert not admission.gripper_compatible
    assert not admission.gripper_compatibility_enforced

    actions[0, 0] += 2.0
    rejected = evaluator.evaluate_prefix(
        PolicyChunk("B", 2, 0.0, 0.01, 0.01, 30.0, actions),
        bridge_reference_action=np.array([500, 0, 400, 0, 150, 0, 1]),
        bridge_velocity_mm_s=np.zeros(3),
        expected_semantic=manifest.successor.semantic,
    )
    assert not rejected.valid
    assert "first_xyz_axis_delta" in rejected.failure_reasons


def test_coordinator_rejects_semantic_authority_mismatch():
    session = AsyncPolicySession("B", _DelayedBackend(0.0), action_hz=30.0)
    successor = AsyncSuccessorController(session, max_result_age_s=1.0)
    guarded_config = HandoffV2Config(
        enabled=True,
        handoff_window_steps=24,
        b_prefix_steps=15,
        crossfade_steps=15,
        compatibility=_compatibility(),
    )
    guarded_evaluator = HandoffCompatibilityEvaluator(
        guarded_config.compatibility,
        prefix_steps=15,
        action_hz=30.0,
    )
    external_manifest = replace(
        _manifest(),
        semantic_authority=SemanticAuthority.EXTERNAL_PLANNER,
    )
    try:
        with pytest.raises(ValueError, match="manifest/config"):
            TaskCHandoffV2Coordinator(
                manifest=external_manifest,
                config=guarded_config,
                successor=successor,
                evaluator=guarded_evaluator,
            )
        with pytest.raises(ValueError, match="evaluator/config"):
            TaskCHandoffV2Coordinator(
                manifest=external_manifest,
                config=replace(
                    guarded_config,
                    semantic_authority=SemanticAuthority.EXTERNAL_PLANNER,
                ),
                successor=successor,
                evaluator=guarded_evaluator,
            )
    finally:
        session.close()

def test_runtime_workspace_can_disable_only_z_minimum():
    manifest = replace(
        _manifest(),
        successor=replace(
            _manifest().successor,
            nominal_position_mm=np.array([150.0, 10.0, -10.0]),
        ),
        transport_floor_mm=None,
    )
    snapshot = replace(
        _snapshot(),
        actual_pose_mm_deg=np.array([100.0, 0.0, -20.0, 0.0, 150.0, 0.0]),
        acknowledged_pose_mm_deg=np.array(
            [100.0, 0.0, -20.0, 0.0, 150.0, 0.0]
        ),
    )
    config = HandoffV2Config(
        enabled=True,
        handoff_window_steps=24,
        b_prefix_steps=15,
        crossfade_steps=15,
        compatibility=_compatibility(),
    )

    with pytest.raises(BridgeGenerationError, match="workspace_violation"):
        instantiate_bridge_queue(
            manifest,
            snapshot,
            config,
            _limits(),
            clock=lambda: 1.0,
        )

    queue = instantiate_bridge_queue(
        manifest,
        snapshot,
        config,
        replace(
            _limits(),
            workspace_min_limit_enabled=(True, True, False),
        ),
        clock=lambda: 1.0,
    )

    assert np.min(queue.actions[:, 2]) < 0.0
    assert queue.workspace_min_limit_enabled == (True, True, False)
    assert queue.record()["workspace_min_limit_enabled"] == [True, True, False]
    assert queue.transport_floor_mm is None
