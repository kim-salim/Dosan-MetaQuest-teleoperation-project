"""Runtime contracts for dual-ACT Task-C V0."""

from __future__ import annotations

import threading
import time
import unittest
from typing import Any, Callable

import numpy as np

from .bridge_optimizer import NoFeasibleBridgeError
from .runtime_bridge import RuntimeBEntry, RuntimeBridgePlanner
from .runtime_orchestrator import (
    RuntimeHandoffConfig,
    RuntimeObservation,
    RuntimePhase,
    TaskCRealtimeCoordinator,
)
from .runtime_policy import (
    AsyncPolicySession,
    PolicyChunk,
    assess_policy_chunk,
)
from .semantic_candidates import apply_representative_b_velocities
from .trajectory_states import CandidatePoint, SemanticState, Trajectory
from .velocity_estimation import RuntimeVelocityHistory


class FakeBackend:
    def __init__(
        self,
        factory: Callable[[Any, int], np.ndarray] | None = None,
    ) -> None:
        self.calls = 0
        self.resets = 0
        self.factory = factory or self._constant

    @staticmethod
    def _constant(_observation: Any, _call: int) -> np.ndarray:
        actions = np.zeros((100, 7), dtype=np.float64)
        actions[:, 0] = np.arange(1, 101, dtype=np.float64) / 30.0
        return actions

    def reset(self) -> None:
        self.resets += 1

    def infer(self, observation: Any) -> np.ndarray:
        call = self.calls
        self.calls += 1
        return np.asarray(self.factory(observation, call), dtype=np.float64)


def dynamic_actions(observation: Any, _call: int) -> np.ndarray:
    start = np.asarray(observation["tcp_position_mm"], dtype=np.float64)
    velocity = np.asarray(
        observation.get("intent_velocity_mm_s", [1.0, 0.0, 0.0]),
        dtype=np.float64,
    )
    time_s = np.arange(1, 101, dtype=np.float64) / 30.0
    actions = np.zeros((100, 7), dtype=np.float64)
    actions[:, :3] = start[None, :] + time_s[:, None] * velocity[None, :]
    actions[:, 6] = 1.0
    return actions


def make_chunk(
    start: np.ndarray,
    velocity: np.ndarray,
    *,
    generation: int = 1,
    observation_timestamp_s: float = 10.0,
) -> PolicyChunk:
    time_s = np.arange(1, 101, dtype=np.float64) / 30.0
    actions = np.zeros((100, 7), dtype=np.float64)
    actions[:, :3] = start[None, :] + time_s[:, None] * velocity[None, :]
    return PolicyChunk(
        policy_id="ACT-B",
        generation=generation,
        observation_timestamp_s=observation_timestamp_s,
        completed_timestamp_s=observation_timestamp_s + 0.1,
        inference_latency_s=0.1,
        action_hz=30.0,
        actions=actions,
    )


def planner(
    *,
    duration_s: float = 1.0,
    workspace_min: np.ndarray | None = None,
    workspace_max: np.ndarray | None = None,
) -> RuntimeBridgePlanner:
    limits = {
        "velocity_limit_mm_s": 1e9,
        "acceleration_limit_mm_s2": 1e9,
        "curvature_limit_per_mm": 1e9,
        "jerk_limit_mm_s3": 1e9,
        "integrated_squared_jerk_limit": 1e18,
        "backtracking_ratio_limit": 1e9,
        "enforce_payload_transport_floor": False,
    }
    return RuntimeBridgePlanner(
        semantic_config={"require_same_gripper": True},
        feasibility_config=limits,
        sampling_config={
            "bridge_sample_hz": 60.0,
            "curvature_epsilon": 1e-9,
        },
        selection_config={"near_shortest_delta": 0.0, "top_k": 5},
        duration_config={
            "bridge_duration_min_s": duration_s,
            "bridge_duration_max_s": duration_s,
            "bridge_duration_step_s": 0.1,
        },
        workspace_min_mm=(
            np.array([-1000.0, -1000.0, -1000.0])
            if workspace_min is None
            else workspace_min
        ),
        workspace_max_mm=(
            np.array([1000.0, 1000.0, 1000.0])
            if workspace_max is None
            else workspace_max
        ),
    )


def strict_position_bridge_planner() -> RuntimeBridgePlanner:
    return RuntimeBridgePlanner(
        semantic_config={"require_same_gripper": True},
        feasibility_config={
            "velocity_limit_mm_s": 300.0,
            "axis_velocity_limit_mm_s": 200.1,
            "acceleration_limit_mm_s2": 300.0,
            "boundary_acceleration_jump_limit_mm_s2": 300.0,
            "curvature_limit_per_mm": 0.25,
            "jerk_limit_mm_s3": 800.0,
            "integrated_squared_jerk_limit": 1_000_000.0,
            "backtracking_ratio_limit": 2.5,
            "enforce_payload_transport_floor": False,
        },
        sampling_config={
            "bridge_sample_hz": 60.0,
            "curvature_epsilon": 1e-9,
        },
        selection_config={"near_shortest_delta": 0.005, "top_k": 10},
        duration_config={
            "bridge_duration_min_s": 0.2,
            "bridge_duration_max_s": 2.5,
            "bridge_duration_step_s": 0.1,
        },
        workspace_min_mm=np.array([0.0, -500.0, 0.0]),
        workspace_max_mm=np.array([700.0, 500.0, 700.0]),
    )


class RejectingTailPlanner:
    def __init__(self, failure_counts: dict[str, int]) -> None:
        self.failure_counts = dict(failure_counts)
        self.sampling_config = {
            "bridge_sample_hz": 60.0,
            "curvature_epsilon": 1e-9,
        }

    def plan(self, **_kwargs: Any) -> None:
        raise NoFeasibleBridgeError(
            "no feasible runtime bridge",
            failure_counts=self.failure_counts,
            candidates_evaluated=24,
        )


class VelocityRejectingPositionPlanner:
    """Reproduce a velocity-tail failure while accepting a safe zero tail."""

    def __init__(self, delegate: RuntimeBridgePlanner) -> None:
        self.delegate = delegate
        self.sampling_config = delegate.sampling_config

    def plan(self, **kwargs: Any):
        terminal_velocity = np.asarray(
            kwargs.get("terminal_velocity_override_mm_s"),
            dtype=np.float64,
        )
        if float(np.linalg.norm(terminal_velocity)) > 1e-9:
            raise NoFeasibleBridgeError(
                "no feasible runtime bridge",
                failure_counts={"curvature_limit": 24},
                candidates_evaluated=24,
            )
        return self.delegate.plan(**kwargs)


def entry(
    entry_id: str,
    position: np.ndarray,
    *,
    velocity: np.ndarray | None = None,
    retained_length: float = 0.0,
) -> RuntimeBEntry:
    return RuntimeBEntry(
        entry_id=entry_id,
        dataset="B",
        episode=0,
        frame=100,
        semantic_label="b_transport",
        semantic_state=SemanticState(
            gripper_closed=True,
            holding=True,
            contact_mode="free_transport_assumed",
            completed_subgoals=("grasp_complete",),
        ),
        position_mm=np.asarray(position, dtype=float),
        demonstration_velocity_mm_s=(
            np.array([1.0, 0.0, 0.0])
            if velocity is None
            else np.asarray(velocity, dtype=float)
        ),
        b_retained_length_mm=retained_length,
    )


class RuntimeObservationTest(unittest.TestCase):
    def test_acknowledged_command_state_is_paired_and_owned(self) -> None:
        position = np.array([1.0, 2.0, 3.0])
        command_position = np.array([4.0, 5.0, 6.0])
        command_velocity = np.array([7.0, 8.0, 9.0])
        observation = RuntimeObservation(
            timestamp_s=1.0,
            tcp_position_mm=position,
            semantic_state=SemanticState(True, holding=True),
            acknowledged_command_position_mm=command_position,
            acknowledged_command_velocity_mm_s=command_velocity,
        )

        position[:] = -1.0
        command_position[:] = -1.0
        command_velocity[:] = -1.0
        np.testing.assert_allclose(observation.tcp_position_mm, [1.0, 2.0, 3.0])
        np.testing.assert_allclose(
            observation.acknowledged_command_position_mm,
            [4.0, 5.0, 6.0],
        )
        np.testing.assert_allclose(
            observation.acknowledged_command_velocity_mm_s,
            [7.0, 8.0, 9.0],
        )

        with self.assertRaisesRegex(ValueError, "must be paired"):
            RuntimeObservation(
                timestamp_s=1.0,
                tcp_position_mm=np.zeros(3),
                semantic_state=SemanticState(True, holding=True),
                acknowledged_command_position_mm=np.zeros(3),
            )


class VelocitySourceTest(unittest.TestCase):
    def test_measured_runtime_velocity_is_causal_window_fit(self) -> None:
        history = RuntimeVelocityHistory(30)
        actual = np.array([12.0, -3.0, 5.0])
        for index in range(15):
            timestamp = index / 30.0
            history.add(timestamp, np.array([1.0, 2.0, 3.0]) + actual * timestamp)
        estimated = history.estimate(
            window_frames=15,
            method="linear_regression",
            velocity_epsilon=1e-9,
        )
        np.testing.assert_allclose(estimated, actual, atol=1e-10)
        policy_intent = np.array([-999.0, 500.0, 0.0])
        self.assertFalse(np.allclose(estimated, policy_intent))

    def test_act_chunk_velocity_uses_multiple_postprocessed_targets(self) -> None:
        start = np.array([100.0, -20.0, 300.0])
        expected = np.array([30.0, -15.0, 6.0])
        chunk = make_chunk(start, expected)
        result = assess_policy_chunk(
            chunk,
            start,
            velocity_window_steps=15,
            velocity_method="linear_regression",
            velocity_epsilon=1e-9,
            first_position_jump_limit_mm=10.0,
            predicted_velocity_limit_mm_s=100.0,
        )
        self.assertTrue(result.valid)
        self.assertEqual(result.velocity_window_steps, 15)
        np.testing.assert_allclose(
            result.intended_velocity_mm_s,
            expected,
            atol=1e-10,
        )

    def test_act_chunk_bridge_gap_is_not_counted_as_policy_velocity(self) -> None:
        policy_start = np.array([100.0, -20.0, 300.0])
        observation = policy_start + np.array([0.0, 65.0, 0.0])
        expected = np.array([30.0, -15.0, 6.0])
        result = assess_policy_chunk(
            make_chunk(policy_start, expected),
            observation,
            velocity_window_steps=15,
            velocity_method="linear_regression",
            velocity_epsilon=1e-9,
            first_position_jump_limit_mm=75.0,
            predicted_velocity_limit_mm_s=100.0,
        )
        self.assertTrue(result.valid, result.failure_reasons)
        self.assertGreater(result.first_position_jump_mm, 60.0)
        self.assertLess(result.max_predicted_velocity_mm_s, 100.0)
        np.testing.assert_allclose(
            result.intended_velocity_mm_s,
            expected,
            atol=1e-10,
        )

    def test_act_chunk_bridge_gap_still_obeys_first_target_jump_limit(self) -> None:
        policy_start = np.array([100.0, -20.0, 300.0])
        chunk = make_chunk(policy_start, np.array([30.0, 0.0, 0.0]))
        observation = policy_start + np.array([0.0, 76.0, 0.0])
        result = assess_policy_chunk(
            chunk,
            observation,
            velocity_window_steps=15,
            velocity_method="linear_regression",
            velocity_epsilon=1e-9,
            first_position_jump_limit_mm=75.0,
            predicted_velocity_limit_mm_s=100.0,
        )
        self.assertFalse(result.valid)
        self.assertIn("b_first_action_position_jump", result.failure_reasons)
        self.assertNotIn("b_predicted_velocity_limit", result.failure_reasons)

    def test_act_chunk_internal_target_speed_limit_remains_active(self) -> None:
        chunk = make_chunk(
            np.array([100.0, -20.0, 300.0]),
            np.array([120.0, 0.0, 0.0]),
        )
        result = assess_policy_chunk(
            chunk,
            chunk.first_xyz_mm,
            velocity_window_steps=15,
            velocity_method="linear_regression",
            velocity_epsilon=1e-9,
            first_position_jump_limit_mm=75.0,
            predicted_velocity_limit_mm_s=100.0,
        )
        self.assertFalse(result.valid)
        self.assertNotIn("b_first_action_position_jump", result.failure_reasons)
        self.assertIn("b_predicted_velocity_limit", result.failure_reasons)

    def test_observation_capture_timestamp_is_not_worker_start_time(self) -> None:
        session = AsyncPolicySession("ACT-B", FakeBackend(), action_hz=30.0)
        try:
            capture = time.monotonic() - 5.0
            generation = session.prime({}, observation_timestamp_s=capture)
            chunk = session.wait_for_chunk(generation, 1.0)
            self.assertEqual(chunk.observation_timestamp_s, capture)
            self.assertGreaterEqual(chunk.observation_age_s(time.monotonic()), 5.0)
        finally:
            session.close()


class PolicyIsolationTest(unittest.TestCase):
    def test_warmup_outputs_are_discarded_and_queues_are_separate(self) -> None:
        lock = threading.Lock()
        a = AsyncPolicySession(
            "ACT-A", FakeBackend(), action_hz=30.0, inference_lock=lock
        )
        b = AsyncPolicySession(
            "ACT-B", FakeBackend(), action_hz=30.0, inference_lock=lock
        )
        try:
            a.warmup({}, inferences=2)
            b.warmup({}, inferences=2)
            self.assertIsNone(a.ready_chunk)
            self.assertIsNone(b.ready_chunk)
            generation_a = a.prime({}, observation_timestamp_s=time.monotonic())
            generation_b = b.prime({}, observation_timestamp_s=time.monotonic())
            a.wait_for_chunk(generation_a, 1.0)
            b.wait_for_chunk(generation_b, 1.0)
            a.activate(generation_a)
            b.activate(generation_b)
            self.assertIsNotNone(a.pop_action())
            b_size = b.queue_size
            a.deactivate_and_clear()
            self.assertEqual(b.queue_size, b_size)
            self.assertEqual(a.queue_size, 0)
        finally:
            a.close()
            b.close()

    def test_refresh_generation_preserves_active_queue_until_atomic_activation(self) -> None:
        session = AsyncPolicySession("ACT-B", FakeBackend(), action_hz=30.0)
        try:
            first = session.prime({}, observation_timestamp_s=time.monotonic())
            session.wait_for_chunk(first, 1.0)
            session.activate(first)
            np.testing.assert_allclose(session.pop_action()[0], 1.0 / 30.0)

            second = session.prime(
                {},
                observation_timestamp_s=time.monotonic(),
                preserve_active=True,
            )
            self.assertEqual(session.active_generation, first)
            self.assertEqual(session.queue_size, 99)
            np.testing.assert_allclose(session.pop_action()[0], 2.0 / 30.0)

            session.wait_for_chunk(second, 1.0)
            self.assertEqual(session.active_generation, first)
            self.assertEqual(session.queue_size, 98)
            replacement = np.full((4, 7), 42.0, dtype=np.float64)
            session.activate(second, actions=replacement)
            self.assertEqual(session.active_generation, second)
            self.assertEqual(session.queue_size, 4)
            np.testing.assert_allclose(session.pop_action(), replacement[0])
        finally:
            session.close()

    def test_stale_generation_result_is_dropped(self) -> None:
        session = AsyncPolicySession("ACT-B", FakeBackend(), action_hz=30.0)
        try:
            first = session.prime({}, observation_timestamp_s=time.monotonic())
            time.sleep(0.01)
            second = session.prime({}, observation_timestamp_s=time.monotonic())
            self.assertGreater(second, first)
            chunk = session.wait_for_chunk(second, 1.0)
            self.assertEqual(chunk.generation, second)
            self.assertGreaterEqual(session.stats().stale_chunks_dropped, 1)
        finally:
            session.close()


class RepresentativeVelocityTest(unittest.TestCase):
    @staticmethod
    def _candidate(episode: int, velocity: np.ndarray) -> CandidatePoint:
        xyz = np.column_stack(
            (np.arange(5, dtype=float), np.zeros(5), np.ones(5) * 300.0)
        )
        orientation = np.full((5, 3), float(episode))
        trajectory = Trajectory(
            dataset="B",
            episode=episode,
            xyz_mm=xyz,
            timestamp_s=np.arange(5, dtype=float) / 30.0,
            gripper_closed=np.ones(5, dtype=bool),
            frame_index=np.arange(5),
            orientation_payload=orientation,
        )
        return CandidatePoint(
            role="b_entry",
            trajectory=trajectory,
            index=2,
            semantic_label="b_transport",
            semantic_state=SemanticState(True, holding=True),
            velocity_mm_s=np.asarray(velocity, dtype=float),
            velocity_method="linear_regression",
            semantic_phase=0.5,
        )

    def test_episode_balanced_representative_preserves_source_payload(self) -> None:
        candidates = [
            self._candidate(0, np.array([1.0, 10.0, 0.0])),
            self._candidate(1, np.array([2.0, 20.0, 0.0])),
            self._candidate(2, np.array([100.0, 30.0, 0.0])),
        ]
        orientation_before = [
            item.trajectory.orientation_payload.copy() for item in candidates
        ]
        replaced = apply_representative_b_velocities(
            candidates,
            {
                "enabled": True,
                "phase_bin_width": 1.0,
                "minimum_demonstrations": 3,
                "method": "component_median",
            },
        )
        for item, before in zip(replaced, orientation_before, strict=True):
            np.testing.assert_allclose(item.velocity_mm_s, [2.0, 20.0, 0.0])
            self.assertEqual(item.velocity_source_episodes, (0, 1, 2))
            self.assertEqual(item.velocity_sample_count, 3)
            np.testing.assert_array_equal(item.trajectory.orientation_payload, before)


class RuntimePlannerTest(unittest.TestCase):
    def test_shortest_feasible_duration_is_explicit_opt_in(self) -> None:
        endpoint = np.array(
            [414.46822853117965, -156.5547452550639, 418.76172698212395]
        )
        first_target = np.array(
            [411.3686828613281, -160.4897003173828, 411.2499694824219]
        )
        runtime_planner = strict_position_bridge_planner()
        kwargs = {
            "position_a_mm": endpoint,
            "velocity_a_mm_s": np.zeros(3),
            "semantic_state_a": SemanticState(True, holding=True),
            "a_retained_length_mm": 0.0,
            "b_entries": [entry("endpoint", endpoint, velocity=np.zeros(3))],
            "terminal_velocity_override_mm_s": np.zeros(3),
            "terminal_position_hint_mm": first_target,
            "terminal_position_tolerance_mm": 75.0,
            "terminal_position_override_mm": first_target,
            "terminal_velocity_source": "stopped_endpoint_position_bridge",
        }

        smoothest = runtime_planner.plan(**kwargs)
        shortest = runtime_planner.plan(
            **kwargs,
            duration_preference="shortest_feasible",
        )

        self.assertAlmostEqual(smoothest.selected.bridge.duration_s, 2.5)
        self.assertAlmostEqual(shortest.selected.bridge.duration_s, 0.6)
        self.assertFalse(shortest.selected.failure_reasons)

    def test_axis_velocity_limit_rejects_streamer_unrealisable_bridge(self) -> None:
        limited = planner(duration_s=1.0)
        limited.feasibility_config["axis_velocity_limit_mm_s"] = 10.0
        with self.assertRaises(NoFeasibleBridgeError):
            limited.plan(
                position_a_mm=np.array([0.0, 0.0, 0.0]),
                velocity_a_mm_s=np.zeros(3),
                semantic_state_a=SemanticState(True, holding=True),
                a_retained_length_mm=0.0,
                b_entries=[entry("too_fast_x", np.array([100.0, 0.0, 0.0]))],
            )

    def test_fresh_act_b_velocity_is_exact_terminal_boundary(self) -> None:
        override = np.array([-4.0, 3.0, 2.0])
        result = planner(duration_s=2.0).plan(
            position_a_mm=np.array([0.0, 0.0, 0.0]),
            velocity_a_mm_s=np.array([5.0, 0.0, 0.0]),
            semantic_state_a=SemanticState(True, holding=True),
            a_retained_length_mm=50.0,
            committed_bridge_prefix_length_mm=12.5,
            b_entries=[entry("one", np.array([20.0, 10.0, 5.0]))],
            terminal_velocity_override_mm_s=override,
            terminal_velocity_source="fresh_act_b_postprocessed_chunk",
        )
        np.testing.assert_allclose(
            result.selected.bridge.velocity(0.0),
            [5.0, 0.0, 0.0],
        )
        np.testing.assert_allclose(
            result.selected.bridge.velocity(1.0),
            override,
        )
        self.assertEqual(
            result.selected.terminal_velocity_source,
            "fresh_act_b_postprocessed_chunk",
        )
        self.assertEqual(
            result.selected.committed_bridge_prefix_length_mm,
            12.5,
        )
        self.assertAlmostEqual(
            result.selected.total_c_estimate_mm,
            50.0
            + 12.5
            + result.selected.metrics.length_mm
            + result.selected.entry.b_retained_length_mm,
        )

    def test_model_target_position_is_dynamic_filter_not_semantic_filter(self) -> None:
        entries = [
            entry("short_but_wrong", np.array([20.0, 0.0, 0.0]), retained_length=0.0),
            entry("model_consistent", np.array([100.0, 0.0, 0.0]), retained_length=500.0),
        ]
        result = planner(duration_s=2.0).plan(
            position_a_mm=np.array([0.0, 0.0, 0.0]),
            velocity_a_mm_s=np.array([1.0, 0.0, 0.0]),
            semantic_state_a=SemanticState(True, holding=True),
            a_retained_length_mm=10.0,
            b_entries=entries,
            terminal_position_hint_mm=np.array([100.0, 0.0, 0.0]),
            terminal_position_tolerance_mm=5.0,
        )
        self.assertEqual(result.selected.entry.entry_id, "model_consistent")
        self.assertEqual(result.semantic_rejected_entries, 0)
        self.assertEqual(
            result.failure_counts["act_b_entry_position_inconsistent"],
            1,
        )

    def test_model_target_can_become_exact_dynamic_bridge_endpoint(self) -> None:
        model_target = np.array([102.0, 3.0, 1.0])
        result = planner(duration_s=2.0).plan(
            position_a_mm=np.array([0.0, 0.0, 0.0]),
            velocity_a_mm_s=np.array([1.0, 0.0, 0.0]),
            semantic_state_a=SemanticState(True, holding=True),
            a_retained_length_mm=10.0,
            b_entries=[entry("nearby-template", np.array([100.0, 0.0, 0.0]))],
            terminal_position_hint_mm=model_target,
            terminal_position_tolerance_mm=5.0,
            terminal_position_override_mm=model_target,
        )
        np.testing.assert_allclose(result.selected.bridge.p3, model_target)
        self.assertFalse(
            np.allclose(result.selected.bridge.p3, result.selected.entry.position_mm)
        )

    def test_no_feasible_runtime_bridge_is_explicit(self) -> None:
        impossible = planner(
            duration_s=1.0,
            workspace_min=np.array([100.0, 100.0, 100.0]),
            workspace_max=np.array([200.0, 200.0, 200.0]),
        )
        with self.assertRaises(NoFeasibleBridgeError) as captured:
            impossible.plan(
                position_a_mm=np.array([0.0, 0.0, 0.0]),
                velocity_a_mm_s=np.array([1.0, 0.0, 0.0]),
                semantic_state_a=SemanticState(True, holding=True),
                a_retained_length_mm=0.0,
                b_entries=[entry("outside", np.array([10.0, 0.0, 0.0]))],
            )
        self.assertEqual(captured.exception.candidates_evaluated, 1)
        self.assertEqual(captured.exception.semantic_rejected_entries, 0)
        self.assertEqual(
            captured.exception.failure_counts["workspace_violation"],
            1,
        )


class CoordinatorTest(unittest.TestCase):
    def _coordinator(
        self,
        *,
        final_refresh: bool = False,
        endpoint_stop: bool = False,
        moving_overlap_primary: bool = False,
        overlap_max_skip_steps: int = 0,
        stopped_endpoint_direct_handoff: bool = False,
        stopped_endpoint_position_bridge: bool = False,
        bridge_progress_max_step_s: float | None = None,
    ) -> tuple[TaskCRealtimeCoordinator, AsyncPolicySession, AsyncPolicySession]:
        lock = threading.Lock()
        a = AsyncPolicySession(
            "ACT-A",
            FakeBackend(dynamic_actions),
            action_hz=30.0,
            inference_lock=lock,
        )
        b = AsyncPolicySession(
            "ACT-B",
            FakeBackend(dynamic_actions),
            action_hz=30.0,
            inference_lock=lock,
        )
        config = RuntimeHandoffConfig(
            warmup_inferences=0,
            velocity_window_frames=5,
            b_prime_lead_s=1.0,
            b_final_refresh_enabled=final_refresh,
            b_final_refresh_lead_s=0.2,
            b_endpoint_stop_before_final_refresh=endpoint_stop,
            b_moving_overlap_primary_enabled=moving_overlap_primary,
            b_endpoint_settle_position_tolerance_mm=0.5,
            b_endpoint_settle_velocity_tolerance_mm_s=0.5,
            b_endpoint_settle_min_hold_s=0.05,
            b_endpoint_settle_timeout_s=0.5,
            b_overlap_search_max_skip_steps=overlap_max_skip_steps,
            b_stopped_endpoint_direct_handoff_enabled=(
                stopped_endpoint_direct_handoff
            ),
            b_stopped_endpoint_position_bridge_enabled=(
                stopped_endpoint_position_bridge
            ),
            b_stopped_endpoint_direct_position_limit_mm=6.67,
            b_inference_timeout_s=2.0,
            b_chunk_max_observation_age_s=2.0,
            b_first_action_position_jump_limit_mm=1e6,
            b_predicted_velocity_limit_mm_s=1e6,
            bridge_progress_max_step_s=bridge_progress_max_step_s,
            handoff_position_tolerance_mm=1e6,
            handoff_velocity_tolerance_mm_s=1e6,
            handoff_first_target_tolerance_mm=1e6,
        )
        coordinator = TaskCRealtimeCoordinator(
            policy_a=a,
            policy_b=b,
            initial_planner=planner(duration_s=2.0),
            tail_planner=planner(duration_s=0.5),
            b_entries=[
                entry(
                    "runtime-b",
                    np.array([10.0, 0.0, 0.0]),
                    velocity=np.array([1.0, 0.0, 0.0]),
                    retained_length=20.0,
                )
            ],
            config=config,
        )
        return coordinator, a, b

    def test_live_bridge_progress_cannot_catch_up_multiple_ticks(self) -> None:
        coordinator, a, b = self._coordinator(
            bridge_progress_max_step_s=1.0 / 30.0,
        )
        try:
            live_plan = planner(duration_s=2.0).plan(
                position_a_mm=np.array([0.0, 0.0, 0.0]),
                velocity_a_mm_s=np.array([1.0, 0.0, 0.0]),
                semantic_state_a=SemanticState(True, holding=True),
                a_retained_length_mm=0.0,
                b_entries=coordinator.b_entries,
            ).selected
            coordinator.current_plan = live_plan
            coordinator.bridge_started_s = 0.0
            coordinator.phase = RuntimePhase.BRIDGE_RUNNING

            proposal = coordinator.step(
                self._observation(0.25, live_plan.bridge.p0)
            )
            self.assertAlmostEqual(coordinator.bridge_elapsed_s, 1.0 / 30.0)
            np.testing.assert_allclose(
                proposal.xyz_mm,
                live_plan.bridge.position((1.0 / 30.0) / live_plan.bridge.duration_s),
            )

            coordinator.step(self._observation(0.5, proposal.xyz_mm))
            self.assertAlmostEqual(coordinator.bridge_elapsed_s, 2.0 / 30.0)
        finally:
            a.close()
            b.close()

    @staticmethod
    def _observation(
        timestamp: float,
        position: np.ndarray,
        *,
        acknowledged_position: np.ndarray | None = None,
        acknowledged_velocity: np.ndarray | None = None,
    ) -> RuntimeObservation:
        return RuntimeObservation(
            timestamp_s=timestamp,
            tcp_position_mm=position,
            semantic_state=SemanticState(
                True,
                holding=True,
                contact_mode="free_transport_assumed",
                completed_subgoals=("grasp_complete",),
            ),
            policy_input={
                "tcp_position_mm": np.asarray(position, dtype=float),
                "intent_velocity_mm_s": np.array([1.0, 0.0, 0.0]),
            },
            acknowledged_command_position_mm=acknowledged_position,
            acknowledged_command_velocity_mm_s=acknowledged_velocity,
        )

    def test_endpoint_refresh_initial_bridge_plans_zero_terminal_velocity(
        self,
    ) -> None:
        coordinator, a, b = self._coordinator(
            final_refresh=True,
            endpoint_stop=True,
        )
        try:
            coordinator.phase = RuntimePhase.A_RUNNING
            for index in range(5):
                timestamp = index / 30.0
                coordinator.history.add(
                    timestamp,
                    np.array([timestamp, 0.0, 0.0]),
                )
            cut_position = np.array([4.0 / 30.0, 0.0, 0.0])
            plan = coordinator.request_cut(
                self._observation(
                    4.0 / 30.0,
                    cut_position,
                    acknowledged_position=cut_position,
                    acknowledged_velocity=np.array([1.0, 0.0, 0.0]),
                ),
                semantic_cut_valid=True,
                a_retained_length_mm=0.0,
            )

            np.testing.assert_allclose(
                plan.bridge.velocity(1.0),
                np.zeros(3),
                atol=1e-12,
            )
            self.assertEqual(
                plan.terminal_velocity_source,
                "planned_endpoint_stop",
            )
            initial_event = next(
                item
                for item in coordinator.events
                if item.event == "initial_bridge_planned"
            )
            self.assertEqual(
                initial_event.details["terminal_velocity_source"],
                "planned_endpoint_stop",
            )
        finally:
            a.close()
            b.close()

    def test_endpoint_stop_holds_until_settled_then_requests_fresh_b(
        self,
    ) -> None:
        coordinator, a, b = self._coordinator(
            final_refresh=True,
            endpoint_stop=True,
        )
        try:
            stopped_plan = planner(duration_s=0.5).plan(
                position_a_mm=np.array([0.0, 0.0, 0.0]),
                velocity_a_mm_s=np.array([1.0, 0.0, 0.0]),
                semantic_state_a=SemanticState(True, holding=True),
                a_retained_length_mm=0.0,
                b_entries=coordinator.b_entries,
                terminal_velocity_override_mm_s=np.zeros(3),
                terminal_velocity_source="planned_endpoint_stop",
            ).selected
            endpoint = stopped_plan.bridge.p3.copy()
            coordinator.current_plan = stopped_plan
            coordinator.bridge_started_s = 0.0
            coordinator.bridge_elapsed_s = stopped_plan.bridge.duration_s
            coordinator._bridge_last_observation_s = stopped_plan.bridge.duration_s
            coordinator.phase = RuntimePhase.BRIDGE_RUNNING
            coordinator.history.clear()
            for timestamp in np.linspace(0.35, 0.49, 5):
                coordinator.history.add(float(timestamp), endpoint)

            first_hold = coordinator.step(
                self._observation(
                    0.5,
                    endpoint,
                    acknowledged_position=endpoint,
                    acknowledged_velocity=np.zeros(3),
                )
            )
            self.assertEqual(first_hold.source, "BEZIER_BRIDGE")
            np.testing.assert_allclose(first_hold.xyz_mm, endpoint)
            self.assertEqual(coordinator.b_final_refresh_count, 0)
            self.assertIsNone(coordinator.b_generation)

            settled_hold = coordinator.step(
                self._observation(
                    0.56,
                    endpoint,
                    acknowledged_position=endpoint,
                    acknowledged_velocity=np.zeros(3),
                )
            )
            self.assertEqual(settled_hold.source, "BEZIER_BRIDGE")
            np.testing.assert_allclose(settled_hold.xyz_mm, endpoint)
            self.assertEqual(coordinator.b_final_refresh_count, 1)
            self.assertIsNotNone(coordinator.b_generation)
            self.assertEqual(
                coordinator.b_generation_role,
                "execution_refresh",
            )
            events = [item.event for item in coordinator.events]
            self.assertIn("bridge_endpoint_stop_hold_started", events)
            self.assertIn("act_b_endpoint_settled_refresh_requested", events)
            self.assertNotIn(
                "act_b_final_near_entry_refresh_requested",
                events,
            )
            refresh = next(
                item
                for item in coordinator.events
                if item.event == "act_b_endpoint_settled_refresh_requested"
            )
            self.assertLessEqual(refresh.details["measured_speed_mm_s"], 0.5)
            self.assertEqual(
                refresh.details["splice_velocity_mm_s"],
                [0.0, 0.0, 0.0],
            )
        finally:
            a.close()
            b.close()

    def test_endpoint_stop_plan_attempts_moving_overlap_before_stopping(
        self,
    ) -> None:
        coordinator, a, b = self._coordinator(
            final_refresh=True,
            endpoint_stop=True,
            moving_overlap_primary=True,
        )
        try:
            stopped_plan = planner(duration_s=2.0).plan(
                position_a_mm=np.array([0.0, 0.0, 0.0]),
                velocity_a_mm_s=np.array([1.0, 0.0, 0.0]),
                semantic_state_a=SemanticState(True, holding=True),
                a_retained_length_mm=0.0,
                b_entries=coordinator.b_entries,
                terminal_velocity_override_mm_s=np.zeros(3),
                terminal_velocity_source="planned_endpoint_stop",
            ).selected
            coordinator.current_plan = stopped_plan
            coordinator.bridge_started_s = 0.0
            coordinator.bridge_elapsed_s = 0.9
            coordinator._bridge_last_observation_s = 0.9
            coordinator.phase = RuntimePhase.BRIDGE_RUNNING
            for timestamp in np.linspace(0.75, 0.9, 5):
                coordinator.history.add(
                    float(timestamp),
                    stopped_plan.bridge.position(float(timestamp) / 2.0),
                )

            for index in range(101):
                timestamp = 1.0 + index / 3000.0
                committed = min(
                    coordinator.bridge_elapsed_s,
                    stopped_plan.bridge.duration_s,
                )
                acknowledged_position = stopped_plan.bridge.position(
                    committed / stopped_plan.bridge.duration_s
                )
                acknowledged_velocity = stopped_plan.bridge.velocity(
                    committed / stopped_plan.bridge.duration_s
                )
                coordinator.step(
                    self._observation(
                        timestamp,
                        stopped_plan.bridge.position(
                            min(1.0, timestamp / stopped_plan.bridge.duration_s)
                        ),
                        acknowledged_position=acknowledged_position,
                        acknowledged_velocity=acknowledged_velocity,
                    )
                )
                if any(
                    item.event == "bridge_tail_replanned_from_fresh_act_b"
                    for item in coordinator.events
                ):
                    break
                time.sleep(0.001)

            requested = next(
                item
                for item in coordinator.events
                if item.event
                == "act_b_fresh_generation_requested_during_bridge"
            )
            self.assertTrue(requested.details["moving_overlap_primary"])
            self.assertTrue(
                requested.details["endpoint_stop_fallback_available"]
            )
            tail = next(
                item
                for item in coordinator.events
                if item.event == "bridge_tail_replanned_from_fresh_act_b"
            )
            self.assertTrue(tail.details["moving_overlap_primary"])
            self.assertTrue(tail.details["endpoint_stop_fallback_consumed"])
            self.assertIsNot(coordinator.current_plan, stopped_plan)
            self.assertEqual(
                coordinator.current_plan.terminal_velocity_source,
                "fresh_act_b_postprocessed_chunk",
            )
            self.assertNotIn(
                "bridge_endpoint_stop_hold_started",
                [item.event for item in coordinator.events],
            )
        finally:
            a.close()
            b.close()

    def test_failed_moving_overlap_retains_stopped_endpoint_fallback(
        self,
    ) -> None:
        coordinator, a, b = self._coordinator(
            final_refresh=True,
            endpoint_stop=True,
            moving_overlap_primary=True,
        )
        try:
            stopped_plan = planner(duration_s=2.0).plan(
                position_a_mm=np.array([0.0, 0.0, 0.0]),
                velocity_a_mm_s=np.array([1.0, 0.0, 0.0]),
                semantic_state_a=SemanticState(True, holding=True),
                a_retained_length_mm=0.0,
                b_entries=coordinator.b_entries,
                terminal_velocity_override_mm_s=np.zeros(3),
                terminal_velocity_source="planned_endpoint_stop",
            ).selected
            coordinator.current_plan = stopped_plan
            coordinator.tail_planner = RejectingTailPlanner(
                {"curvature_limit": 24}
            )
            coordinator.bridge_started_s = 0.0
            coordinator.bridge_elapsed_s = 0.9
            coordinator._bridge_last_observation_s = 0.9
            coordinator.phase = RuntimePhase.BRIDGE_RUNNING
            for timestamp in np.linspace(0.75, 0.9, 5):
                coordinator.history.add(
                    float(timestamp),
                    stopped_plan.bridge.position(float(timestamp) / 2.0),
                )

            for index in range(101):
                timestamp = 1.0 + index / 3000.0
                committed = min(
                    coordinator.bridge_elapsed_s,
                    stopped_plan.bridge.duration_s,
                )
                acknowledged_position = stopped_plan.bridge.position(
                    committed / stopped_plan.bridge.duration_s
                )
                acknowledged_velocity = stopped_plan.bridge.velocity(
                    committed / stopped_plan.bridge.duration_s
                )
                coordinator.step(
                    self._observation(
                        timestamp,
                        stopped_plan.bridge.position(
                            min(1.0, timestamp / stopped_plan.bridge.duration_s)
                        ),
                        acknowledged_position=acknowledged_position,
                        acknowledged_velocity=acknowledged_velocity,
                    )
                )
                if any(
                    item.event
                    == "act_b_tail_seed_rejected_existing_bridge_retained"
                    for item in coordinator.events
                ):
                    break
                time.sleep(0.001)

            fallback = next(
                item
                for item in coordinator.events
                if item.event
                == "act_b_tail_seed_rejected_existing_bridge_retained"
            )
            self.assertTrue(fallback.details["existing_bridge_retained"])
            self.assertTrue(
                fallback.details["endpoint_stop_fallback_retained"]
            )
            self.assertIs(coordinator.current_plan, stopped_plan)
            self.assertIs(coordinator.phase, RuntimePhase.BRIDGE_RUNNING)
            self.assertEqual(coordinator.failure_reasons, [])

            coordinator.history.clear()
            endpoint = stopped_plan.bridge.p3.copy()
            for timestamp in np.linspace(1.8, 1.99, 5):
                coordinator.history.add(float(timestamp), endpoint)
            coordinator.bridge_elapsed_s = stopped_plan.bridge.duration_s
            coordinator._bridge_last_observation_s = stopped_plan.bridge.duration_s
            coordinator.b_endpoint_hold_started_s = None
            coordinator.step(
                self._observation(
                    2.0,
                    endpoint,
                    acknowledged_position=endpoint,
                    acknowledged_velocity=np.zeros(3),
                )
            )
            coordinator.step(
                self._observation(
                    2.06,
                    endpoint,
                    acknowledged_position=endpoint,
                    acknowledged_velocity=np.zeros(3),
                )
            )
            self.assertEqual(coordinator.b_final_refresh_count, 1)
            self.assertEqual(
                coordinator.b_generation_role,
                "execution_refresh",
            )
            self.assertIn(
                "act_b_endpoint_settled_refresh_requested",
                [item.event for item in coordinator.events],
            )
        finally:
            a.close()
            b.close()

    def test_endpoint_overlap_skips_first_infeasible_b_action_only(self) -> None:
        coordinator, a, b = self._coordinator(
            final_refresh=True,
            endpoint_stop=True,
            overlap_max_skip_steps=15,
        )
        try:
            endpoint = np.array(
                [444.64794921875, -157.01255798339844, 412.30645751953125]
            )
            first_target = np.array(
                [438.3622131347656, -164.7031707763672, 407.20318603515625]
            )
            intended_velocity = np.array(
                [-2.028305053711245, -18.41421563284737, -32.070599147251585]
            )
            coordinator.b_entries = (
                entry(
                    "measured-endpoint",
                    endpoint,
                    velocity=np.zeros(3),
                ),
            )
            coordinator.tail_planner = RuntimeBridgePlanner(
                semantic_config={"require_same_gripper": True},
                feasibility_config={
                    "velocity_limit_mm_s": 300.0,
                    "acceleration_limit_mm_s2": 300.0,
                    "curvature_limit_per_mm": 0.25,
                    "jerk_limit_mm_s3": 800.0,
                    "integrated_squared_jerk_limit": 1_000_000.0,
                    "backtracking_ratio_limit": 2.5,
                    "enforce_payload_transport_floor": False,
                },
                sampling_config={
                    "bridge_sample_hz": 60.0,
                    "curvature_epsilon": 1e-9,
                },
                selection_config={"near_shortest_delta": 0.0, "top_k": 5},
                duration_config={
                    "bridge_duration_min_s": 0.2,
                    "bridge_duration_max_s": 0.5,
                    "bridge_duration_step_s": 0.1,
                },
                workspace_min_mm=np.array([0.0, -500.0, 0.0]),
                workspace_max_mm=np.array([700.0, 500.0, 700.0]),
            )
            stopped_plan = planner(duration_s=0.5).plan(
                position_a_mm=endpoint + np.array([0.0, 20.0, 0.0]),
                velocity_a_mm_s=np.zeros(3),
                semantic_state_a=SemanticState(True, holding=True),
                a_retained_length_mm=0.0,
                b_entries=coordinator.b_entries,
                terminal_velocity_override_mm_s=np.zeros(3),
                terminal_velocity_source="planned_endpoint_stop",
            ).selected
            coordinator.current_plan = stopped_plan
            coordinator.bridge_started_s = 0.0
            coordinator.bridge_elapsed_s = stopped_plan.bridge.duration_s
            coordinator._bridge_last_observation_s = stopped_plan.bridge.duration_s
            coordinator.phase = RuntimePhase.B_PRIMING
            coordinator.history.clear()
            for timestamp in np.linspace(0.8, 1.0, 5):
                coordinator.history.add(float(timestamp), endpoint)

            generation = b.prime(
                {
                    "tcp_position_mm": first_target - intended_velocity / 30.0,
                    "intent_velocity_mm_s": intended_velocity,
                },
                observation_timestamp_s=1.0,
            )
            full_chunk = b.wait_for_chunk(generation, 1.0)
            coordinator.b_generation = generation
            coordinator.b_generation_role = "execution_refresh"
            coordinator.b_final_refresh_count = 1
            coordinator.b_prime_started_s = 1.0
            coordinator.b_prime_position_mm = endpoint.copy()
            observation = self._observation(
                1.1,
                endpoint,
                acknowledged_position=endpoint,
                acknowledged_velocity=np.zeros(3),
            )

            result = coordinator._accept_b_chunk_and_replan(
                observation,
                full_chunk,
                committed_elapsed_s=stopped_plan.bridge.duration_s,
                proposal_step_s=0.0,
            )

            self.assertIsNone(result)
            self.assertIs(coordinator.phase, RuntimePhase.B_READY)
            self.assertEqual(coordinator.b_action_start_index, 1)
            np.testing.assert_allclose(
                coordinator.b_chunk.first_xyz_mm,
                full_chunk.actions[1, :3],
            )
            np.testing.assert_allclose(
                coordinator.current_plan.bridge.p3,
                full_chunk.actions[1, :3],
            )
            tail_event = coordinator.events[-1]
            self.assertEqual(tail_event.details["b_action_start_index"], 1)
            self.assertAlmostEqual(
                tail_event.details["b_skipped_prefix_duration_s"],
                1.0 / 30.0,
            )

            coordinator.bridge_elapsed_s = coordinator.current_plan.bridge.duration_s
            coordinator._bridge_last_observation_s = 1.2
            handoff = coordinator._handoff_to_b(
                self._observation(1.2, coordinator.current_plan.bridge.p3)
            )
            self.assertEqual(handoff.source, "ACT-B")
            np.testing.assert_allclose(handoff.full_policy_action, full_chunk.actions[1])
            self.assertEqual(b.queue_size, 98)
        finally:
            a.close()
            b.close()

    def test_stopped_endpoint_direct_handoff_after_smoothness_tail_failure(
        self,
    ) -> None:
        coordinator, a, b = self._coordinator(
            final_refresh=True,
            endpoint_stop=True,
            stopped_endpoint_direct_handoff=True,
        )
        try:
            endpoint = np.array(
                [414.46822853117965, -156.5547452550639, 418.76172698212395]
            )
            first_target = np.array(
                [411.4325256347656, -159.16690063476562, 412.39599609375]
            )
            intended_velocity = np.array(
                [3.6512636457166585, -9.001538412911497, -55.33807591029598]
            )
            coordinator.b_entries = (
                entry(
                    "measured-endpoint",
                    endpoint,
                    velocity=np.zeros(3),
                ),
            )
            coordinator.tail_planner = RejectingTailPlanner(
                {
                    "acceleration_limit": 2,
                    "backtracking_limit": 12,
                    "curvature_limit": 24,
                    "integrated_squared_jerk_limit": 1,
                    "jerk_limit": 3,
                }
            )
            stopped_plan = planner(duration_s=0.5).plan(
                position_a_mm=endpoint + np.array([0.0, 20.0, 0.0]),
                velocity_a_mm_s=np.zeros(3),
                semantic_state_a=SemanticState(True, holding=True),
                a_retained_length_mm=0.0,
                b_entries=coordinator.b_entries,
                terminal_velocity_override_mm_s=np.zeros(3),
                terminal_velocity_source="planned_endpoint_stop",
            ).selected
            coordinator.current_plan = stopped_plan
            coordinator.bridge_started_s = 0.0
            coordinator.bridge_elapsed_s = stopped_plan.bridge.duration_s
            coordinator._bridge_last_observation_s = stopped_plan.bridge.duration_s
            coordinator.phase = RuntimePhase.B_PRIMING
            coordinator.history.clear()
            for timestamp in np.linspace(0.8, 1.0, 5):
                coordinator.history.add(float(timestamp), endpoint)

            generation = b.prime(
                {
                    "tcp_position_mm": first_target
                    - intended_velocity / 30.0,
                    "intent_velocity_mm_s": intended_velocity,
                },
                observation_timestamp_s=1.0,
            )
            full_chunk = b.wait_for_chunk(generation, 1.0)
            coordinator.b_generation = generation
            coordinator.b_generation_role = "execution_refresh"
            coordinator.b_final_refresh_count = 1
            coordinator.b_endpoint_hold_started_s = 0.5
            coordinator.b_prime_started_s = 1.0
            coordinator.b_prime_position_mm = endpoint.copy()
            observation = self._observation(
                1.1,
                endpoint,
                acknowledged_position=endpoint,
                acknowledged_velocity=np.zeros(3),
            )

            result = coordinator._accept_b_chunk_and_replan(
                observation,
                full_chunk,
                committed_elapsed_s=stopped_plan.bridge.duration_s,
                proposal_step_s=0.0,
            )

            self.assertIsNone(result)
            self.assertIs(coordinator.phase, RuntimePhase.B_READY)
            self.assertTrue(coordinator.b_stopped_endpoint_direct_handoff)
            self.assertIs(coordinator.current_plan, stopped_plan)
            ready = next(
                item
                for item in coordinator.events
                if item.event
                == "act_b_stopped_endpoint_direct_handoff_ready"
            )
            self.assertGreater(ready.details["first_target_jump_mm"], 6.67)
            self.assertLessEqual(
                ready.details["first_target_max_axis_step_mm"], 6.67
            )
            self.assertEqual(
                ready.details["direct_position_metric"],
                "max_abs_cartesian_axis",
            )
            self.assertFalse(ready.details["safety_limits_relaxed"])

            handoff = coordinator._handoff_to_b(observation)
            self.assertEqual(handoff.source, "ACT-B")
            np.testing.assert_allclose(
                handoff.full_policy_action,
                full_chunk.actions[0],
            )
            atomic = next(
                item
                for item in coordinator.events
                if item.event == "atomic_bridge_to_act_b_handoff"
            )
            self.assertEqual(
                atomic.details["handoff_mode"],
                "stopped_endpoint_direct",
            )
            self.assertTrue(
                atomic.details["full_pose_live_validation_required"]
            )
            self.assertFalse(atomic.details["safety_limits_relaxed"])
        finally:
            a.close()
            b.close()

    def test_stopped_endpoint_position_bridge_absorbs_measured_first_target_gap(
        self,
    ) -> None:
        coordinator, a, b = self._coordinator(
            final_refresh=True,
            endpoint_stop=True,
            stopped_endpoint_position_bridge=True,
        )
        try:
            endpoint = np.array(
                [414.46822853117965, -156.5547452550639, 418.76172698212395]
            )
            first_target = np.array(
                [411.3686828613281, -160.4897003173828, 411.2499694824219]
            )
            intended_velocity = np.array(
                [4.169488089424885, -10.304395948137515, -47.50804792131732]
            )
            coordinator.b_entries = (
                entry(
                    "measured-endpoint",
                    endpoint,
                    velocity=np.zeros(3),
                ),
            )
            coordinator.tail_planner = VelocityRejectingPositionPlanner(
                strict_position_bridge_planner()
            )
            stopped_plan = planner(duration_s=0.5).plan(
                position_a_mm=endpoint + np.array([0.0, 20.0, 0.0]),
                velocity_a_mm_s=np.zeros(3),
                semantic_state_a=SemanticState(True, holding=True),
                a_retained_length_mm=0.0,
                b_entries=coordinator.b_entries,
                terminal_velocity_override_mm_s=np.zeros(3),
                terminal_velocity_source="planned_endpoint_stop",
            ).selected
            coordinator.current_plan = stopped_plan
            coordinator.bridge_started_s = 0.0
            coordinator.bridge_elapsed_s = stopped_plan.bridge.duration_s
            coordinator._bridge_last_observation_s = stopped_plan.bridge.duration_s
            coordinator.phase = RuntimePhase.B_PRIMING
            coordinator.history.clear()
            for timestamp in np.linspace(0.8, 1.0, 5):
                coordinator.history.add(float(timestamp), endpoint)

            generation = b.prime(
                {
                    "tcp_position_mm": (
                        first_target - intended_velocity / 30.0
                    ),
                    "intent_velocity_mm_s": intended_velocity,
                },
                observation_timestamp_s=1.0,
            )
            full_chunk = b.wait_for_chunk(generation, 1.0)
            coordinator.b_generation = generation
            coordinator.b_generation_role = "execution_refresh"
            coordinator.b_final_refresh_count = 1
            coordinator.b_endpoint_hold_started_s = 0.5
            coordinator.b_prime_started_s = 1.0
            coordinator.b_prime_position_mm = endpoint.copy()
            observation = self._observation(
                1.1,
                endpoint,
                acknowledged_position=endpoint,
                acknowledged_velocity=np.zeros(3),
            )

            result = coordinator._accept_b_chunk_and_replan(
                observation,
                full_chunk,
                committed_elapsed_s=stopped_plan.bridge.duration_s,
                proposal_step_s=0.0,
            )

            self.assertIsNone(result)
            self.assertIs(coordinator.phase, RuntimePhase.B_READY)
            self.assertFalse(coordinator.b_stopped_endpoint_direct_handoff)
            self.assertTrue(
                coordinator.b_stopped_endpoint_position_bridge_handoff
            )
            self.assertEqual(
                coordinator.current_plan.terminal_velocity_source,
                "stopped_endpoint_position_bridge",
            )
            self.assertAlmostEqual(
                coordinator.current_plan.bridge.duration_s,
                0.6,
            )
            np.testing.assert_allclose(
                coordinator.current_plan.bridge.p0,
                endpoint,
            )
            np.testing.assert_allclose(
                coordinator.current_plan.bridge.p3,
                first_target,
            )
            np.testing.assert_allclose(
                coordinator.current_plan.bridge.velocity(0.0),
                np.zeros(3),
                atol=1e-10,
            )
            np.testing.assert_allclose(
                coordinator.current_plan.bridge.velocity(1.0),
                np.zeros(3),
                atol=1e-10,
            )
            ready = next(
                item
                for item in coordinator.events
                if item.event
                == "act_b_stopped_endpoint_position_bridge_ready"
            )
            self.assertTrue(ready.details["all_runtime_bridge_limits_rechecked"])
            self.assertEqual(
                ready.details["duration_preference"],
                "shortest_feasible",
            )

            coordinator.bridge_elapsed_s = (
                coordinator.current_plan.bridge.duration_s
            )
            coordinator._bridge_last_observation_s = 1.8
            coordinator.history.clear()
            for timestamp in np.linspace(1.6, 1.8, 5):
                coordinator.history.add(float(timestamp), first_target)
            handoff = coordinator._handoff_to_b(
                self._observation(
                    1.8,
                    first_target,
                    acknowledged_position=first_target,
                    acknowledged_velocity=np.zeros(3),
                )
            )
            self.assertEqual(handoff.source, "ACT-B")
            np.testing.assert_allclose(
                handoff.full_policy_action,
                full_chunk.actions[0],
            )
            atomic = next(
                item
                for item in coordinator.events
                if item.event == "atomic_bridge_to_act_b_handoff"
            )
            self.assertEqual(
                atomic.details["handoff_mode"],
                "stopped_endpoint_position_bridge",
            )
            self.assertTrue(
                atomic.details["full_pose_live_validation_required"]
            )
            self.assertFalse(atomic.details["safety_limits_relaxed"])
        finally:
            a.close()
            b.close()

    def test_stopped_endpoint_direct_handoff_rejects_single_axis_over_limit(
        self,
    ) -> None:
        coordinator, a, b = self._coordinator(
            final_refresh=True,
            endpoint_stop=True,
            stopped_endpoint_direct_handoff=True,
        )
        try:
            endpoint = np.array([414.0, -156.0, 419.0])
            stopped_plan = planner(duration_s=0.5).plan(
                position_a_mm=endpoint + np.array([0.0, 20.0, 0.0]),
                velocity_a_mm_s=np.zeros(3),
                semantic_state_a=SemanticState(True, holding=True),
                a_retained_length_mm=0.0,
                b_entries=[entry("endpoint", endpoint)],
                terminal_velocity_override_mm_s=np.zeros(3),
                terminal_velocity_source="planned_endpoint_stop",
            ).selected
            chunk = make_chunk(
                endpoint + np.array([6.671, 0.0, 0.0]),
                np.zeros(3),
                observation_timestamp_s=1.0,
            )
            assessment = assess_policy_chunk(
                chunk,
                endpoint,
                velocity_window_steps=15,
                velocity_method="linear_regression",
                velocity_epsilon=1e-9,
                first_position_jump_limit_mm=1e6,
                predicted_velocity_limit_mm_s=1e6,
            )
            coordinator.current_plan = stopped_plan
            coordinator.b_generation_role = "execution_refresh"
            coordinator.b_final_refresh_count = 1
            coordinator.b_endpoint_hold_started_s = 0.5

            armed = coordinator._arm_stopped_endpoint_direct_handoff(
                self._observation(
                    1.1,
                    endpoint,
                    acknowledged_position=endpoint,
                    acknowledged_velocity=np.zeros(3),
                ),
                chunk=chunk,
                assessment=assessment,
                splice_position=endpoint,
                splice_velocity=np.zeros(3),
                measured_velocity=np.zeros(3),
                planner_failure_counts={"curvature_limit": 24},
                planning_started_s=1.0,
                planning_completed_s=1.1,
            )

            self.assertFalse(armed)
            rejected = coordinator.events[-1]
            self.assertEqual(
                rejected.event,
                "act_b_stopped_endpoint_direct_handoff_rejected",
            )
            self.assertIn(
                "first_target_exceeds_direct_position_limit",
                rejected.details["failure_reasons"],
            )
            self.assertAlmostEqual(
                rejected.details["first_target_max_axis_step_mm"],
                6.671,
            )
        finally:
            a.close()
            b.close()

    def test_stopped_endpoint_direct_handoff_rejects_payload_failure(
        self,
    ) -> None:
        coordinator, a, b = self._coordinator(
            final_refresh=True,
            endpoint_stop=True,
            stopped_endpoint_direct_handoff=True,
        )
        try:
            endpoint = np.array([10.0, 0.0, 0.0])
            coordinator.b_entries = (
                entry("endpoint", endpoint, velocity=np.zeros(3)),
            )
            coordinator.tail_planner = RejectingTailPlanner(
                {
                    "curvature_limit": 24,
                    "payload_clearance_violation": 24,
                }
            )
            stopped_plan = planner(duration_s=0.5).plan(
                position_a_mm=endpoint + np.array([0.0, 1.0, 0.0]),
                velocity_a_mm_s=np.zeros(3),
                semantic_state_a=SemanticState(True, holding=True),
                a_retained_length_mm=0.0,
                b_entries=coordinator.b_entries,
                terminal_velocity_override_mm_s=np.zeros(3),
                terminal_velocity_source="planned_endpoint_stop",
            ).selected
            coordinator.current_plan = stopped_plan
            coordinator.bridge_started_s = 0.0
            coordinator.bridge_elapsed_s = stopped_plan.bridge.duration_s
            coordinator._bridge_last_observation_s = stopped_plan.bridge.duration_s
            coordinator.phase = RuntimePhase.B_PRIMING
            coordinator.history.clear()
            for timestamp in np.linspace(0.8, 1.0, 5):
                coordinator.history.add(float(timestamp), endpoint)
            generation = b.prime(
                {
                    "tcp_position_mm": endpoint,
                    "intent_velocity_mm_s": np.array([1.0, 0.0, 0.0]),
                },
                observation_timestamp_s=1.0,
            )
            chunk = b.wait_for_chunk(generation, 1.0)
            coordinator.b_generation = generation
            coordinator.b_generation_role = "execution_refresh"
            coordinator.b_final_refresh_count = 1
            coordinator.b_endpoint_hold_started_s = 0.5
            coordinator.b_prime_started_s = 1.0
            coordinator.b_prime_position_mm = endpoint.copy()
            observation = self._observation(
                1.1,
                endpoint,
                acknowledged_position=endpoint,
                acknowledged_velocity=np.zeros(3),
            )

            proposal = coordinator._accept_b_chunk_and_replan(
                observation,
                chunk,
                committed_elapsed_s=stopped_plan.bridge.duration_s,
                proposal_step_s=0.0,
            )

            self.assertIsNotNone(proposal)
            self.assertIs(coordinator.phase, RuntimePhase.FAILED_HOLD)
            self.assertFalse(coordinator.b_stopped_endpoint_direct_handoff)
            rejected = next(
                item
                for item in coordinator.events
                if item.event
                == "act_b_stopped_endpoint_direct_handoff_rejected"
            )
            self.assertIn(
                "non_smoothness_planner_failure",
                rejected.details["failure_reasons"],
            )
        finally:
            a.close()
            b.close()

    def test_tail_replan_splices_from_acknowledged_command_and_tangent(self) -> None:
        coordinator, a, b = self._coordinator(final_refresh=True)
        try:
            old_plan = planner(duration_s=2.0).plan(
                position_a_mm=np.array([0.0, 0.0, 0.0]),
                velocity_a_mm_s=np.array([1.0, 0.0, 0.0]),
                semantic_state_a=SemanticState(True, holding=True),
                a_retained_length_mm=0.0,
                b_entries=coordinator.b_entries,
            ).selected
            committed_elapsed_s = 1.0
            committed_u = committed_elapsed_s / old_plan.bridge.duration_s
            acknowledged_position = old_plan.bridge.position(committed_u)
            acknowledged_velocity = old_plan.bridge.velocity(committed_u)
            actual_position = acknowledged_position + np.array([0.0, -7.117, 0.0])
            for index in range(5):
                coordinator.history.add(
                    0.8 + index / 30.0,
                    actual_position + np.array([index * 0.01, 0.0, 0.0]),
                )

            coordinator.current_plan = old_plan
            coordinator.bridge_started_s = 0.0
            coordinator.bridge_elapsed_s = committed_elapsed_s
            coordinator._bridge_last_observation_s = committed_elapsed_s
            coordinator.phase = RuntimePhase.B_PRIMING
            coordinator.b_generation_role = "tail_seed"
            coordinator.b_prime_position_mm = actual_position.copy()
            chunk = make_chunk(
                np.array([10.0, 0.0, 0.0]),
                np.array([1.0, 0.0, 0.0]),
                generation=1,
                observation_timestamp_s=0.9,
            )
            observation = self._observation(
                committed_elapsed_s + 1.0 / 30.0,
                actual_position,
                acknowledged_position=acknowledged_position,
                acknowledged_velocity=acknowledged_velocity,
            )
            result = coordinator._accept_b_chunk_and_replan(
                observation,
                chunk,
                committed_elapsed_s=committed_elapsed_s,
                proposal_step_s=1.0 / 30.0,
            )

            self.assertIsNone(result)
            self.assertIs(coordinator.phase, RuntimePhase.B_READY)
            np.testing.assert_allclose(
                coordinator.current_plan.bridge.p0,
                acknowledged_position,
                atol=1e-12,
            )
            np.testing.assert_allclose(
                coordinator.current_plan.bridge.velocity(0.0),
                acknowledged_velocity,
                atol=1e-12,
            )
            self.assertGreater(
                np.linalg.norm(
                    coordinator.current_plan.bridge.p0 - actual_position
                ),
                7.0,
            )
            first_outgoing = coordinator.current_plan.bridge.position(
                coordinator.bridge_elapsed_s
                / coordinator.current_plan.bridge.duration_s
            )
            self.assertLess(
                float(np.max(np.abs(first_outgoing - acknowledged_position))),
                6.67,
            )
            tail_event = coordinator.events[-1]
            self.assertEqual(
                tail_event.details["splice_position_source"],
                "acknowledged_commanded_posx",
            )
            self.assertEqual(
                tail_event.details["splice_velocity_source"],
                "acknowledged_bridge_tangent",
            )
            self.assertAlmostEqual(
                tail_event.details["actual_to_splice_position_error_mm"],
                7.117,
                places=6,
            )
            self.assertTrue(
                tail_event.details["atomic_splice_at_acknowledged_tick"]
            )
        finally:
            a.close()
            b.close()

    def test_infeasible_tail_seed_retains_bridge_until_execution_refresh(
        self,
    ) -> None:
        coordinator, a, b = self._coordinator(final_refresh=True)
        try:
            old_plan = planner(duration_s=1.0).plan(
                position_a_mm=np.array([0.0, 0.0, 0.0]),
                velocity_a_mm_s=np.array([1.0, 0.0, 0.0]),
                semantic_state_a=SemanticState(True, holding=True),
                a_retained_length_mm=0.0,
                b_entries=coordinator.b_entries,
            ).selected
            committed_elapsed_s = 0.6
            committed_u = committed_elapsed_s / old_plan.bridge.duration_s
            splice_position = old_plan.bridge.position(committed_u)
            splice_velocity = old_plan.bridge.velocity(committed_u)
            for timestamp in np.linspace(0.45, committed_elapsed_s, 5):
                coordinator.history.add(
                    float(timestamp),
                    old_plan.bridge.position(float(timestamp)),
                )

            coordinator.current_plan = old_plan
            coordinator.bridge_started_s = 0.0
            coordinator.bridge_elapsed_s = committed_elapsed_s
            coordinator._bridge_last_observation_s = committed_elapsed_s
            coordinator.phase = RuntimePhase.B_PRIMING
            coordinator.b_generation = 1
            coordinator.b_generation_role = "tail_seed"
            coordinator.b_tail_seed_attempted = True
            coordinator.b_prime_position_mm = splice_position.copy()
            coordinator.tail_planner.workspace_min_mm = np.array(
                [100.0, 100.0, 100.0]
            )
            coordinator.tail_planner.workspace_max_mm = np.array(
                [200.0, 200.0, 200.0]
            )
            observation = self._observation(
                committed_elapsed_s + 1.0 / 30.0,
                splice_position,
                acknowledged_position=splice_position,
                acknowledged_velocity=splice_velocity,
            )

            proposal = coordinator._accept_b_chunk_and_replan(
                observation,
                make_chunk(
                    np.array([10.0, 0.0, 0.0]),
                    np.array([1.0, 0.0, 0.0]),
                    generation=1,
                    observation_timestamp_s=0.55,
                ),
                committed_elapsed_s=committed_elapsed_s,
                proposal_step_s=1.0 / 30.0,
            )

            self.assertIsNone(proposal)
            self.assertIs(coordinator.phase, RuntimePhase.BRIDGE_RUNNING)
            self.assertIs(coordinator.current_plan, old_plan)
            self.assertIsNone(coordinator.b_generation)
            self.assertIsNone(coordinator.b_chunk)
            self.assertEqual(coordinator.failure_reasons, [])
            fallback = coordinator.events[-1]
            self.assertEqual(
                fallback.event,
                "act_b_tail_seed_rejected_existing_bridge_retained",
            )
            self.assertTrue(fallback.details["existing_bridge_retained"])
            self.assertTrue(fallback.details["execution_refresh_still_required"])
            self.assertFalse(fallback.details["safety_limits_relaxed"])

            coordinator.tail_planner = planner(duration_s=0.5)
            coordinator.bridge_elapsed_s = 0.79
            coordinator._bridge_last_observation_s = 0.79
            refresh_position = old_plan.bridge.position(0.81)
            coordinator.step(self._observation(0.81, refresh_position))
            self.assertEqual(coordinator.b_final_refresh_count, 1)
            self.assertEqual(
                coordinator.b_generation_role,
                "execution_refresh",
            )
            self.assertIn(
                "act_b_final_near_entry_refresh_requested",
                [item.event for item in coordinator.events],
            )
        finally:
            a.close()
            b.close()

    def test_infeasible_execution_refresh_still_fails_closed(self) -> None:
        coordinator, a, b = self._coordinator(final_refresh=True)
        try:
            old_plan = planner(duration_s=1.0).plan(
                position_a_mm=np.array([0.0, 0.0, 0.0]),
                velocity_a_mm_s=np.array([1.0, 0.0, 0.0]),
                semantic_state_a=SemanticState(True, holding=True),
                a_retained_length_mm=0.0,
                b_entries=coordinator.b_entries,
            ).selected
            splice_position = old_plan.bridge.position(0.8)
            splice_velocity = old_plan.bridge.velocity(0.8)
            for timestamp in np.linspace(0.65, 0.8, 5):
                coordinator.history.add(
                    float(timestamp),
                    old_plan.bridge.position(float(timestamp)),
                )
            coordinator.current_plan = old_plan
            coordinator.bridge_started_s = 0.0
            coordinator.bridge_elapsed_s = 0.8
            coordinator._bridge_last_observation_s = 0.8
            coordinator.phase = RuntimePhase.B_PRIMING
            coordinator.b_generation = 2
            coordinator.b_generation_role = "execution_refresh"
            coordinator.b_final_refresh_count = 1
            coordinator.b_prime_position_mm = splice_position.copy()
            coordinator.tail_planner.workspace_min_mm = np.array(
                [100.0, 100.0, 100.0]
            )
            coordinator.tail_planner.workspace_max_mm = np.array(
                [200.0, 200.0, 200.0]
            )

            proposal = coordinator._accept_b_chunk_and_replan(
                self._observation(
                    0.81,
                    splice_position,
                    acknowledged_position=splice_position,
                    acknowledged_velocity=splice_velocity,
                ),
                make_chunk(
                    np.array([10.0, 0.0, 0.0]),
                    np.array([1.0, 0.0, 0.0]),
                    generation=2,
                    observation_timestamp_s=0.75,
                ),
                committed_elapsed_s=0.8,
                proposal_step_s=0.01,
            )

            self.assertIs(coordinator.phase, RuntimePhase.FAILED_HOLD)
            self.assertIsNone(proposal.xyz_mm)
            self.assertIn(
                "no_feasible_act_b_velocity_matched_tail",
                coordinator.failure_reasons,
            )
            self.assertNotIn(
                "act_b_tail_seed_rejected_existing_bridge_retained",
                [item.event for item in coordinator.events],
            )
        finally:
            a.close()
            b.close()

    def test_cut_uses_measured_tcp_not_act_a_intent(self) -> None:
        coordinator, a, b = self._coordinator()
        try:
            coordinator.phase = RuntimePhase.A_RUNNING
            actual_velocity = np.array([8.0, -2.0, 1.0])
            for index in range(5):
                timestamp = index / 30.0
                coordinator.history.add(
                    timestamp,
                    np.array([2.0, 3.0, 400.0]) + actual_velocity * timestamp,
                )
            observation = self._observation(
                4.0 / 30.0,
                np.array([2.0, 3.0, 400.0])
                + actual_velocity * (4.0 / 30.0),
            )
            plan = coordinator.request_cut(
                observation,
                semantic_cut_valid=True,
                a_retained_length_mm=100.0,
                a_policy_intent_velocity_mm_s=np.array([999.0, 0.0, 0.0]),
            )
            np.testing.assert_allclose(
                coordinator.a_measured_velocity_mm_s,
                actual_velocity,
                atol=1e-9,
            )
            np.testing.assert_allclose(
                plan.bridge.velocity(0.0),
                actual_velocity,
                atol=1e-9,
            )
            self.assertFalse(
                np.allclose(
                    plan.bridge.velocity(0.0),
                    coordinator.a_policy_intent_velocity_mm_s,
                )
            )
        finally:
            a.close()
            b.close()

    def test_cut_anchors_position_to_acknowledged_command_but_keeps_actual_velocity(
        self,
    ) -> None:
        coordinator, a, b = self._coordinator()
        try:
            coordinator.phase = RuntimePhase.A_RUNNING
            actual_velocity = np.array([8.0, -2.0, 1.0])
            actual_position = np.array([2.0, 3.0, 400.0])
            for index in range(5):
                timestamp = index / 30.0
                actual_position = (
                    np.array([2.0, 3.0, 400.0])
                    + actual_velocity * timestamp
                )
                coordinator.history.add(timestamp, actual_position)
            acknowledged_position = actual_position + np.array(
                [1.144, 0.570, 8.749]
            )
            observation = self._observation(
                4.0 / 30.0,
                actual_position,
                acknowledged_position=acknowledged_position,
                acknowledged_velocity=actual_velocity,
            )

            plan = coordinator.request_cut(
                observation,
                semantic_cut_valid=True,
                a_retained_length_mm=100.0,
            )

            np.testing.assert_allclose(plan.bridge.p0, acknowledged_position)
            np.testing.assert_allclose(
                plan.bridge.velocity(0.0),
                actual_velocity,
                atol=1e-9,
            )
            initial_event = next(
                item
                for item in coordinator.events
                if item.event == "initial_bridge_planned"
            )
            self.assertEqual(
                initial_event.details["bridge_start_position_source"],
                "acknowledged_commanded_posx",
            )
            self.assertEqual(
                initial_event.details["bridge_start_velocity_source"],
                "measured_tcp_history",
            )
            self.assertAlmostEqual(
                initial_event.details[
                    "actual_to_bridge_start_position_error_mm"
                ],
                float(
                    np.linalg.norm(
                        acknowledged_position - actual_position
                    )
                ),
            )
        finally:
            a.close()
            b.close()

    def test_full_dry_transition_replans_from_fresh_b_chunk(self) -> None:
        coordinator, a, b = self._coordinator(final_refresh=False)
        try:
            start = self._observation(0.0, np.array([0.0, 0.0, 0.0]))
            coordinator.warmup_models(
                observation_a=start.policy_input,
                observation_b=start.policy_input,
                timestamp_s=0.0,
            )
            coordinator.start_a(start)
            for _ in range(100):
                coordinator.step(start)
                if coordinator.phase is RuntimePhase.A_RUNNING:
                    break
                time.sleep(0.001)
            self.assertIs(coordinator.phase, RuntimePhase.A_RUNNING)

            for index in range(1, 6):
                position = np.array([index / 30.0, 0.0, 0.0])
                coordinator.history.add(index / 30.0, position)
            cut_time = 5.0 / 30.0
            cut = self._observation(cut_time, np.array([cut_time, 0.0, 0.0]))
            coordinator.request_cut(
                cut,
                semantic_cut_valid=True,
                a_retained_length_mm=25.0,
            )
            self.assertIsNotNone(coordinator.a_policy_intent_velocity_mm_s)
            cut_event = coordinator.events[-2]
            self.assertEqual(
                cut_event.details["act_a_intent_velocity_source"],
                "active_act_a_postprocessed_chunk",
            )
            initial = coordinator.current_plan
            self.assertIsNotNone(initial)
            prime_time = cut_time + 1.05
            initial_u = (prime_time - cut_time) / initial.bridge.duration_s
            prime_position = initial.bridge.position(initial_u)
            prime_observation = self._observation(prime_time, prime_position)
            coordinator.step(prime_observation)

            for index in range(100):
                time.sleep(0.001)
                poll_time = prime_time + (index + 1) / 3000.0
                coordinator.step(self._observation(poll_time, prime_position))
                if coordinator.phase is RuntimePhase.B_READY:
                    break
            self.assertIs(coordinator.phase, RuntimePhase.B_READY)
            self.assertEqual(
                coordinator.current_plan.terminal_velocity_source,
                "fresh_act_b_postprocessed_chunk",
            )
            self.assertEqual(coordinator.b_generation_role, "execution")
            np.testing.assert_allclose(
                coordinator.current_plan.bridge.p3,
                coordinator.b_chunk.first_xyz_mm,
                atol=1e-12,
            )
            np.testing.assert_allclose(
                coordinator.current_plan.bridge.velocity(1.0),
                coordinator.b_assessment.intended_velocity_mm_s,
                atol=1e-8,
            )
            self.assertGreater(coordinator.committed_bridge_prefix_length_mm, 0.0)
            self.assertAlmostEqual(
                coordinator.current_plan.total_c_estimate_mm,
                25.0
                + coordinator.committed_bridge_prefix_length_mm
                + coordinator.current_plan.metrics.length_mm
                + coordinator.current_plan.entry.b_retained_length_mm,
            )

            tail_start = coordinator.bridge_started_s
            tail = coordinator.current_plan.bridge
            proposal = None
            for index in range(1, 21):
                elapsed = min(
                    tail.duration_s,
                    index * tail.duration_s / 20.0,
                )
                position = tail.position(elapsed / tail.duration_s)
                proposal = coordinator.step(
                    self._observation(tail_start + elapsed, position)
                )
            self.assertIsNotNone(proposal)
            self.assertEqual(proposal.source, "ACT-B")
            self.assertIs(coordinator.phase, RuntimePhase.B_RUNNING)
            self.assertEqual(a.queue_size, 0)
            self.assertFalse(proposal.robot_executable)
            self.assertTrue(proposal.dry_run_only)
            events = [item.event for item in coordinator.events]
            self.assertIn("bridge_tail_replanned_from_fresh_act_b", events)
            self.assertIn("atomic_bridge_to_act_b_handoff", events)
            initial_event = next(
                item
                for item in coordinator.events
                if item.event == "initial_bridge_planned"
            )
            tail_event = next(
                item
                for item in coordinator.events
                if item.event == "bridge_tail_replanned_from_fresh_act_b"
            )
            for event in (initial_event, tail_event):
                self.assertGreaterEqual(event.details["planning_latency_s"], 0.0)
                self.assertGreaterEqual(
                    event.details["planning_completed_timestamp_s"],
                    event.details["planning_started_timestamp_s"],
                )
        finally:
            a.close()
            b.close()

    def test_final_near_entry_refresh_uses_new_generation(self) -> None:
        coordinator, a, b = self._coordinator(final_refresh=True)
        try:
            live_plan = planner(duration_s=1.0).plan(
                position_a_mm=np.array([0.0, 0.0, 0.0]),
                velocity_a_mm_s=np.array([1.0, 0.0, 0.0]),
                semantic_state_a=SemanticState(True, holding=True),
                a_retained_length_mm=0.0,
                b_entries=coordinator.b_entries,
            ).selected
            coordinator.current_plan = live_plan
            coordinator.bridge_started_s = 0.0
            coordinator.phase = RuntimePhase.B_READY
            old_generation = b.prime(
                {
                    "tcp_position_mm": np.array([8.0, 0.0, 0.0]),
                    "intent_velocity_mm_s": np.array([1.0, 0.0, 0.0]),
                },
                observation_timestamp_s=0.1,
            )
            old_chunk = b.wait_for_chunk(old_generation, 1.0)
            coordinator.b_generation = old_generation
            coordinator.b_generation_role = "tail_seed"
            coordinator.b_chunk = old_chunk
            coordinator.b_assessment = assess_policy_chunk(
                old_chunk,
                np.array([8.0, 0.0, 0.0]),
                velocity_window_steps=15,
                velocity_method="linear_regression",
                velocity_epsilon=1e-9,
                first_position_jump_limit_mm=1e6,
                predicted_velocity_limit_mm_s=1e6,
            )
            for timestamp in np.linspace(0.65, 0.8, 5):
                coordinator.history.add(
                    float(timestamp),
                    live_plan.bridge.position(float(timestamp)),
                )
            refresh_time = 0.85
            refresh_position = live_plan.bridge.position(refresh_time)
            observation = self._observation(refresh_time, refresh_position)
            coordinator.step(observation)
            self.assertIs(coordinator.phase, RuntimePhase.B_PRIMING)
            self.assertEqual(coordinator.b_final_refresh_count, 1)
            self.assertGreater(coordinator.b_generation, old_generation)
            self.assertEqual(coordinator.b_generation_role, "execution_refresh")
            for index in range(100):
                time.sleep(0.001)
                poll_time = refresh_time + (index + 1) / 3000.0
                coordinator.step(
                    self._observation(poll_time, refresh_position)
                )
                if coordinator.phase is RuntimePhase.B_READY:
                    break
            self.assertIs(coordinator.phase, RuntimePhase.B_READY)
            self.assertEqual(
                coordinator.current_plan.terminal_velocity_source,
                "fresh_act_b_postprocessed_chunk",
            )
            self.assertGreater(coordinator.committed_bridge_prefix_length_mm, 0.0)
            self.assertAlmostEqual(
                coordinator.current_plan.total_c_estimate_mm,
                coordinator.a_retained_length_mm
                + coordinator.committed_bridge_prefix_length_mm
                + coordinator.current_plan.metrics.length_mm
                + coordinator.current_plan.entry.b_retained_length_mm,
            )
            events = [item.event for item in coordinator.events]
            self.assertIn("act_b_final_near_entry_refresh_requested", events)
            refresh_event = next(
                item
                for item in coordinator.events
                if item.event == "act_b_final_near_entry_refresh_requested"
            )
            self.assertEqual(
                refresh_event.details["generation_role"],
                "execution_refresh",
            )

            execution_generation = coordinator.b_generation
            tail_start = coordinator.bridge_started_s
            tail = coordinator.current_plan.bridge
            proposal = None
            for index in range(1, 21):
                elapsed = min(
                    tail.duration_s,
                    index * tail.duration_s / 20.0,
                )
                position = tail.position(elapsed / tail.duration_s)
                proposal = coordinator.step(
                    self._observation(tail_start + elapsed, position)
                )
                if coordinator.phase is RuntimePhase.B_RUNNING:
                    break
            self.assertIsNotNone(proposal)
            self.assertIs(coordinator.phase, RuntimePhase.B_RUNNING)
            self.assertEqual(proposal.source, "ACT-B")
            self.assertEqual(coordinator.b_generation, execution_generation)
            handoff_event = next(
                item
                for item in coordinator.events
                if item.event == "atomic_bridge_to_act_b_handoff"
            )
            self.assertEqual(
                handoff_event.details["generation_role"],
                "execution_refresh",
            )
        finally:
            a.close()
            b.close()

    def test_tail_seed_generation_cannot_enter_act_b(self) -> None:
        coordinator, a, b = self._coordinator(final_refresh=True)
        try:
            live_plan = planner(duration_s=0.5).plan(
                position_a_mm=np.array([0.0, 0.0, 0.0]),
                velocity_a_mm_s=np.array([1.0, 0.0, 0.0]),
                semantic_state_a=SemanticState(True, holding=True),
                a_retained_length_mm=0.0,
                b_entries=coordinator.b_entries,
            ).selected
            generation = b.prime(
                {
                    "tcp_position_mm": live_plan.bridge.p3,
                    "intent_velocity_mm_s": np.array([1.0, 0.0, 0.0]),
                },
                observation_timestamp_s=live_plan.bridge.duration_s,
            )
            chunk = b.wait_for_chunk(generation, 1.0)
            coordinator.current_plan = live_plan
            coordinator.bridge_started_s = 0.0
            coordinator.bridge_elapsed_s = live_plan.bridge.duration_s
            coordinator._bridge_last_observation_s = live_plan.bridge.duration_s
            coordinator.phase = RuntimePhase.B_READY
            coordinator.b_generation = generation
            coordinator.b_generation_role = "tail_seed"
            coordinator.b_chunk = chunk
            coordinator.b_assessment = assess_policy_chunk(
                chunk,
                live_plan.bridge.p3,
                velocity_window_steps=15,
                velocity_method="linear_regression",
                velocity_epsilon=1e-9,
                first_position_jump_limit_mm=1e6,
                predicted_velocity_limit_mm_s=1e6,
            )
            coordinator.b_final_refresh_count = 1
            for index in range(5):
                timestamp = live_plan.bridge.duration_s - (4 - index) / 30.0
                coordinator.history.add(
                    timestamp,
                    live_plan.bridge.p3
                    - np.array([(4 - index) / 30.0, 0.0, 0.0]),
                )

            proposal = coordinator.step(
                self._observation(
                    live_plan.bridge.duration_s + 0.01,
                    live_plan.bridge.p3,
                )
            )

            self.assertIs(coordinator.phase, RuntimePhase.FAILED_HOLD)
            self.assertIsNone(proposal.xyz_mm)
            self.assertIn(
                "act_b_execution_generation_role_mismatch",
                coordinator.failure_reasons,
            )
        finally:
            a.close()
            b.close()

    def test_missing_b_observation_fails_closed_without_xyz(self) -> None:
        coordinator, a, b = self._coordinator()
        try:
            coordinator.phase = RuntimePhase.A_RUNNING
            for index in range(5):
                coordinator.history.add(
                    index / 30.0,
                    np.array([index / 30.0, 0.0, 0.0]),
                )
            cut = RuntimeObservation(
                timestamp_s=4.0 / 30.0,
                tcp_position_mm=np.array([4.0 / 30.0, 0.0, 0.0]),
                semantic_state=SemanticState(True, holding=True),
                policy_input=None,
            )
            coordinator.request_cut(
                cut,
                semantic_cut_valid=True,
                a_retained_length_mm=0.0,
            )
            plan = coordinator.current_plan
            timestamp = cut.timestamp_s + plan.bridge.duration_s - 0.5
            observation = RuntimeObservation(
                timestamp_s=timestamp,
                tcp_position_mm=plan.bridge.position(0.75),
                semantic_state=SemanticState(True, holding=True),
                policy_input=None,
            )
            proposal = coordinator.step(observation)
            self.assertIs(coordinator.phase, RuntimePhase.FAILED_HOLD)
            self.assertIsNone(proposal.xyz_mm)
            self.assertFalse(proposal.robot_executable)
            self.assertIn("b_policy_input_missing", proposal.failure_reasons)
        finally:
            a.close()
            b.close()


if __name__ == "__main__":
    unittest.main()
