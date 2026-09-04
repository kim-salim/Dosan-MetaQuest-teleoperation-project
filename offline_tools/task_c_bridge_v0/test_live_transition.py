"""Contracts for the command-producing Task-C pose bridge support."""

from __future__ import annotations

import unittest

import numpy as np

from quest_a0509_teleop.doosan_orientation import (
    doosan_zyz_deg_to_quaternion,
    quaternion_angle_deg,
)

from .bezier_bridge import build_velocity_matched_bezier
from .live_transition import (
    CutTriggerConfig,
    DownstreamControlContract,
    LiveCutTrigger,
    OrientationBridge,
    assess_bridge_stream_contract,
    assess_policy_pose_chunk,
    full_action,
    pose_delta_metrics,
)
from .runtime_policy import PolicyChunk


def _chunk(actions: np.ndarray) -> PolicyChunk:
    return PolicyChunk(
        policy_id="ACT-B",
        generation=1,
        observation_timestamp_s=1.0,
        completed_timestamp_s=1.1,
        inference_latency_s=0.1,
        action_hz=30.0,
        actions=actions,
    )


class LiveTransitionTest(unittest.TestCase):
    def test_downstream_contract_intersects_manifest_workspace_and_speed(self) -> None:
        contract = DownstreamControlContract()
        minimum, maximum = contract.intersect_workspace(
            np.array([276.0, -318.0, 179.0]),
            np.array([581.0, 338.0, 646.0]),
        )
        np.testing.assert_allclose(minimum, [276.0, -318.0, 179.0])
        np.testing.assert_allclose(maximum, [581.0, 338.0, 600.0])
        self.assertAlmostEqual(contract.conservative_velocity_limit_mm_s, 200.1)
        self.assertAlmostEqual(contract.angular_velocity_limit_deg_s, 30.0)

    def test_bridge_stream_contract_accepts_unmodified_slow_bridge(self) -> None:
        bridge = build_velocity_matched_bezier(
            np.array([300.0, 0.0, 400.0]),
            np.zeros(3),
            np.array([330.0, 0.0, 400.0]),
            np.zeros(3),
            2.0,
        )
        assessment = assess_bridge_stream_contract(
            bridge,
            np.array([0.0, 150.0, 0.0]),
            np.array([5.0, 150.0, 0.0]),
            contract=DownstreamControlContract(),
        )
        self.assertTrue(assessment.valid, assessment.failure_reasons)
        self.assertLessEqual(assessment.max_linear_axis_step_mm, 6.67)
        self.assertLessEqual(assessment.max_orientation_step_deg, 1.0)

    def test_bridge_stream_contract_rejects_ramp_and_guard_intervention(self) -> None:
        fast_outside = build_velocity_matched_bezier(
            np.array([300.0, 0.0, 590.0]),
            np.zeros(3),
            np.array([400.0, 0.0, 610.0]),
            np.zeros(3),
            0.2,
        )
        assessment = assess_bridge_stream_contract(
            fast_outside,
            np.array([0.0, 150.0, 0.0]),
            np.array([90.0, 150.0, 0.0]),
            contract=DownstreamControlContract(),
        )
        self.assertFalse(assessment.valid)
        self.assertIn("downstream_workspace_clamp", assessment.failure_reasons)
        self.assertIn("downstream_linear_ramp", assessment.failure_reasons)
        self.assertIn("downstream_orientation_ramp", assessment.failure_reasons)

    def test_orientation_bridge_is_physical_shortest_path_and_exact_endpoint(self) -> None:
        start = np.array([179.0, 45.0, -179.0])
        target = np.array([-179.0, 46.0, 179.0])
        bridge = OrientationBridge(start, target, duration_s=2.0)
        samples = [bridge.sample(value) for value in np.linspace(0.0, 2.0, 61)]
        self.assertAlmostEqual(
            quaternion_angle_deg(
                doosan_zyz_deg_to_quaternion(samples[0]),
                doosan_zyz_deg_to_quaternion(start),
            ),
            0.0,
            places=8,
        )
        self.assertAlmostEqual(
            quaternion_angle_deg(
                doosan_zyz_deg_to_quaternion(samples[-1]),
                doosan_zyz_deg_to_quaternion(target),
            ),
            0.0,
            places=8,
        )
        physical_steps = [
            quaternion_angle_deg(
                doosan_zyz_deg_to_quaternion(left),
                doosan_zyz_deg_to_quaternion(right),
            )
            for left, right in zip(samples, samples[1:])
        ]
        self.assertLess(max(physical_steps), 1.0)

    def test_pose_chunk_assessment_uses_quaternion_not_euler_wrap(self) -> None:
        actions = np.zeros((10, 7), dtype=np.float64)
        actions[:, :3] = np.arange(1, 11)[:, None] * np.array([0.1, 0.0, 0.0])
        actions[:, 3:6] = np.array([-179.0, 45.0, 179.0])
        actions[:, 6] = 1.0
        assessment = assess_policy_pose_chunk(
            _chunk(actions),
            np.array([0.0, 0.0, 0.0, 179.0, 45.0, -179.0]),
            window_steps=10,
            first_position_jump_limit_mm=5.0,
            first_orientation_jump_limit_deg=10.0,
            predicted_velocity_limit_mm_s=50.0,
            predicted_angular_velocity_limit_deg_s=300.0,
        )
        self.assertTrue(assessment.valid, assessment.failure_reasons)
        self.assertLess(assessment.first_orientation_jump_deg, 5.0)

    def test_pose_chunk_assessment_rejects_orientation_jump(self) -> None:
        actions = np.zeros((5, 7), dtype=np.float64)
        actions[:, 3:6] = np.array([90.0, 90.0, 0.0])
        assessment = assess_policy_pose_chunk(
            _chunk(actions),
            np.zeros(6),
            window_steps=5,
            first_position_jump_limit_mm=5.0,
            first_orientation_jump_limit_deg=10.0,
            predicted_velocity_limit_mm_s=50.0,
            predicted_angular_velocity_limit_deg_s=300.0,
        )
        self.assertFalse(assessment.valid)
        self.assertIn("policy_first_orientation_jump", assessment.failure_reasons)

    def test_pose_chunk_assessment_has_independent_axis_velocity_limit(self) -> None:
        actions = np.zeros((5, 7), dtype=np.float64)
        actions[:, :3] = np.arange(1, 6)[:, None] * np.array([7.0, 1.0, 1.0])
        assessment = assess_policy_pose_chunk(
            _chunk(actions),
            np.zeros(6),
            window_steps=5,
            first_position_jump_limit_mm=10.0,
            first_orientation_jump_limit_deg=10.0,
            predicted_velocity_limit_mm_s=300.0,
            predicted_angular_velocity_limit_deg_s=300.0,
            predicted_axis_velocity_limit_mm_s=200.1,
        )
        self.assertFalse(assessment.valid)
        self.assertLess(assessment.max_predicted_velocity_mm_s, 300.0)
        self.assertGreater(assessment.max_predicted_axis_velocity_mm_s, 200.1)
        self.assertIn(
            "policy_predicted_axis_velocity_limit",
            assessment.failure_reasons,
        )

    def test_pose_chunk_bridge_gap_is_not_counted_as_policy_velocity(self) -> None:
        actions = np.zeros((5, 7), dtype=np.float64)
        actions[:, :3] = (
            np.array([100.0, 0.0, 400.0])
            + np.arange(5)[:, None] * np.array([1.0, 0.0, 0.0])
        )
        actions[:, 3:6] = np.array([0.0, 150.0, 0.0])
        observation = np.array([100.0, 65.0, 400.0, 0.0, 150.0, 0.0])
        assessment = assess_policy_pose_chunk(
            _chunk(actions),
            observation,
            window_steps=5,
            first_position_jump_limit_mm=75.0,
            first_orientation_jump_limit_deg=10.0,
            predicted_velocity_limit_mm_s=100.0,
            predicted_angular_velocity_limit_deg_s=100.0,
        )
        self.assertTrue(assessment.valid, assessment.failure_reasons)
        self.assertGreater(assessment.first_position_jump_mm, 60.0)
        self.assertAlmostEqual(assessment.max_predicted_velocity_mm_s, 30.0)

    def test_pose_chunk_bridge_gap_still_obeys_position_jump_limit(self) -> None:
        actions = np.zeros((5, 7), dtype=np.float64)
        actions[:, :3] = (
            np.array([100.0, 0.0, 400.0])
            + np.arange(5)[:, None] * np.array([1.0, 0.0, 0.0])
        )
        actions[:, 3:6] = np.array([0.0, 150.0, 0.0])
        observation = np.array([100.0, 76.0, 400.0, 0.0, 150.0, 0.0])
        assessment = assess_policy_pose_chunk(
            _chunk(actions),
            observation,
            window_steps=5,
            first_position_jump_limit_mm=75.0,
            first_orientation_jump_limit_deg=10.0,
            predicted_velocity_limit_mm_s=100.0,
            predicted_angular_velocity_limit_deg_s=100.0,
        )
        self.assertFalse(assessment.valid)
        self.assertIn("policy_first_position_jump", assessment.failure_reasons)
        self.assertNotIn(
            "policy_predicted_velocity_limit",
            assessment.failure_reasons,
        )

    def test_pose_chunk_internal_target_speed_limit_remains_active(self) -> None:
        actions = np.zeros((5, 7), dtype=np.float64)
        actions[:, :3] = (
            np.array([100.0, 0.0, 400.0])
            + np.arange(5)[:, None] * np.array([4.0, 0.0, 0.0])
        )
        actions[:, 3:6] = np.array([0.0, 150.0, 0.0])
        assessment = assess_policy_pose_chunk(
            _chunk(actions),
            actions[0, :6],
            window_steps=5,
            first_position_jump_limit_mm=75.0,
            first_orientation_jump_limit_deg=10.0,
            predicted_velocity_limit_mm_s=100.0,
            predicted_angular_velocity_limit_deg_s=100.0,
        )
        self.assertFalse(assessment.valid)
        self.assertNotIn("policy_first_position_jump", assessment.failure_reasons)
        self.assertIn(
            "policy_predicted_velocity_limit",
            assessment.failure_reasons,
        )

    def test_cut_trigger_requires_live_open_close_and_transport_phase(self) -> None:
        trigger = LiveCutTrigger(
            CutTriggerConfig(
                open_stable_frames=2,
                closed_stable_frames=3,
                post_close_delay_s=0.2,
                transport_z_min_mm=400.0,
            )
        )
        self.assertFalse(
            trigger.update(
                timestamp_s=0.0,
                tcp_z_mm=420.0,
                gripper_closed=True,
            ).ready
        )
        trigger.update(timestamp_s=0.1, tcp_z_mm=300.0, gripper_closed=False)
        trigger.update(timestamp_s=0.2, tcp_z_mm=300.0, gripper_closed=False)
        trigger.update(timestamp_s=0.3, tcp_z_mm=350.0, gripper_closed=True)
        trigger.update(timestamp_s=0.4, tcp_z_mm=390.0, gripper_closed=True)
        waiting = trigger.update(
            timestamp_s=0.5,
            tcp_z_mm=399.0,
            gripper_closed=True,
        )
        self.assertFalse(waiting.ready)
        self.assertEqual(waiting.waiting_reasons, ("transport_phase_z_not_reached",))
        ready = trigger.update(
            timestamp_s=0.51,
            tcp_z_mm=401.0,
            gripper_closed=True,
        )
        self.assertTrue(ready.ready)

    def test_full_action_and_pose_delta(self) -> None:
        action = full_action(
            np.array([1.0, 2.0, 3.0]),
            np.array([10.0, 20.0, 30.0]),
            gripper_target=1.0,
        )
        np.testing.assert_allclose(action, [1.0, 2.0, 3.0, 10.0, 20.0, 30.0, 1.0])
        position, orientation = pose_delta_metrics(action[:6], action[:6])
        self.assertEqual(position, 0.0)
        self.assertAlmostEqual(orientation, 0.0)


if __name__ == "__main__":
    unittest.main()
