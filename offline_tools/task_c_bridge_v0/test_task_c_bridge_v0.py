"""Contract tests for the position-only Task-C V0 implementation."""

from __future__ import annotations

import copy
import unittest

import numpy as np

from .bezier_bridge import (
    build_tangent_regularized_bezier,
    build_velocity_matched_bezier,
)
from .bridge_metrics import BridgeMetrics, evaluate_bridge, feasibility_reasons
from .bridge_optimizer import (
    CandidateEvaluation,
    NoFeasibleBridgeError,
    select_best_candidate,
)
from .semantic_candidates import (
    SemanticWindow,
    generate_candidates,
    is_semantically_compatible,
)
from .trajectory_states import CandidatePoint, SemanticState, Trajectory


def trajectory(
    xyz: np.ndarray,
    gripper: np.ndarray | None = None,
    orientation: np.ndarray | None = None,
) -> Trajectory:
    xyz = np.asarray(xyz, dtype=float)
    length = len(xyz)
    return Trajectory(
        dataset="test",
        episode=0,
        xyz_mm=xyz,
        timestamp_s=np.arange(length, dtype=float) / 30.0,
        gripper_closed=np.zeros(length, dtype=bool) if gripper is None else gripper,
        frame_index=np.arange(length),
        orientation_payload=orientation,
    )


def candidate(
    role: str,
    index: int,
    xyz: np.ndarray,
    velocity: np.ndarray,
    state: SemanticState | None = None,
) -> CandidatePoint:
    return CandidatePoint(
        role=role,  # type: ignore[arg-type]
        trajectory=trajectory(xyz),
        index=index,
        semantic_label="post_release_open" if role == "a_cut" else "pre_grasp_open",
        semantic_state=state or SemanticState(False),
        velocity_mm_s=np.asarray(velocity, dtype=float),
        velocity_method="test",
    )


def metrics(length: float, **overrides: float | bool) -> BridgeMetrics:
    values = dict(
        length_mm=length,
        max_velocity_mm_s=1.0,
        max_acceleration_mm_s2=1.0,
        max_curvature_per_mm=0.01,
        max_jerk_mm_s3=1.0,
        integrated_squared_jerk=1.0,
        backtracking_ratio=1.0,
        workspace_satisfied=True,
        finite=True,
    )
    values.update(overrides)
    return BridgeMetrics(**values)  # type: ignore[arg-type]


class BezierMathTest(unittest.TestCase):
    def setUp(self) -> None:
        self.p0 = np.array([0.0, 0.0, 0.0])
        self.p3 = np.array([10.0, 8.0, 2.0])
        self.v0 = np.array([2.0, -1.0, 0.5])
        self.v1 = np.array([-1.0, 3.0, 0.25])
        self.bridge = build_velocity_matched_bezier(self.p0, self.v0, self.p3, self.v1, 2.5)

    def test_01_start_position(self) -> None:
        np.testing.assert_allclose(self.bridge.position(0.0), self.p0)

    def test_02_end_position(self) -> None:
        np.testing.assert_allclose(self.bridge.position(1.0), self.p3)

    def test_03_start_velocity(self) -> None:
        np.testing.assert_allclose(self.bridge.velocity(0.0), self.v0)

    def test_04_end_velocity(self) -> None:
        np.testing.assert_allclose(self.bridge.velocity(1.0), self.v1)

    def test_05_opposite_velocity_curves(self) -> None:
        bridge = build_velocity_matched_bezier(
            self.p0, np.array([0.0, 5.0, 0.0]), self.p3, np.array([0.0, -5.0, 0.0]), 2.0
        )
        midpoint = bridge.position(0.5)
        self.assertTrue(np.all(np.isfinite(midpoint)))
        self.assertGreater(float(midpoint[1]), 4.0)

    def test_19_duration_changes_control_and_acceleration(self) -> None:
        short = build_velocity_matched_bezier(self.p0, self.v0, self.p3, self.v1, 1.0)
        long = build_velocity_matched_bezier(self.p0, self.v0, self.p3, self.v1, 4.0)
        self.assertFalse(np.allclose(short.p1, long.p1))
        self.assertFalse(np.allclose(short.acceleration(0.0), long.acceleration(0.0)))

    def test_22_tangent_regularization_preserves_endpoint_directions(self) -> None:
        bridge, diagnostics = build_tangent_regularized_bezier(
            np.array([0.0, 0.0, 400.0]),
            np.array([3.0, 0.0, 0.0]),
            np.array([250.0, 10.0, 400.0]),
            np.array([20.0, 0.0, 0.0]),
            4.0,
            minimum_handle_chord_ratio=0.04,
        )
        source_direction = bridge.velocity(0.0)
        successor_direction = bridge.velocity(1.0)
        np.testing.assert_allclose(
            source_direction / np.linalg.norm(source_direction),
            [1.0, 0.0, 0.0],
        )
        np.testing.assert_allclose(
            successor_direction / np.linalg.norm(successor_direction),
            [1.0, 0.0, 0.0],
        )
        self.assertAlmostEqual(
            diagnostics.minimum_handle_mm,
            0.04 * diagnostics.chord_length_mm,
        )
        self.assertGreater(diagnostics.source_speed_adjustment_mm_s, 0.0)

    def test_23_tangent_regularization_requires_defined_directions(self) -> None:
        with self.assertRaisesRegex(ValueError, "source_direction_undefined"):
            build_tangent_regularized_bezier(
                self.p0,
                np.zeros(3),
                self.p3,
                self.v1,
                4.0,
                minimum_handle_chord_ratio=0.04,
            )


class SemanticContractTest(unittest.TestCase):
    def test_06_far_xyz_not_semantic_rejection(self) -> None:
        a = candidate("a_cut", 1, np.array([[0, 0, 0], [0, 0, 0], [1, 0, 0]]), [1, 0, 0])
        b = candidate("b_entry", 1, np.array([[10000, 0, 0], [10000, 0, 0], [10001, 0, 0]]), [1, 0, 0])
        self.assertTrue(is_semantically_compatible(a, b, {"require_same_gripper": True}))

    def test_07_near_xyz_holding_incompatible(self) -> None:
        xyz = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]])
        a = candidate("a_cut", 1, xyz, [1, 0, 0], SemanticState(False, holding=False))
        b = candidate("b_entry", 1, xyz, [1, 0, 0], SemanticState(False, holding=True))
        self.assertFalse(is_semantically_compatible(a, b, {"require_same_gripper": True}))

    def test_08_generation_does_not_need_low_speed(self) -> None:
        length = 80
        xyz = np.column_stack((np.arange(length), np.zeros(length), np.zeros(length)))
        gripper = np.zeros(length, dtype=bool)
        gripper[30:60] = True
        result = generate_candidates(
            [trajectory(xyz, gripper)],
            role="a_cut",
            windows=[SemanticWindow("gripper_open", 0, 10, 5, "post_release_open", "open")],
            velocity_config={
                "velocity_window_frames": 7,
                "velocity_smoothing_method": "linear_regression",
                "velocity_epsilon": 1e-9,
            },
        )
        self.assertEqual([item.index for item in result], [60, 65, 70])

    def test_09_orientation_not_used_in_ranking(self) -> None:
        original = np.arange(30, dtype=float).reshape(10, 3)
        changed = original + 9999.0
        xyz = np.column_stack((np.arange(10), np.zeros(10), np.zeros(10)))
        t1 = trajectory(xyz, orientation=original)
        t2 = trajectory(xyz, orientation=changed)
        self.assertAlmostEqual(t1.total_length_mm, t2.total_length_mm)

    def test_10_orientation_payload_is_not_modified(self) -> None:
        orientation = np.arange(30, dtype=float).reshape(10, 3)
        before = copy.deepcopy(orientation)
        item = trajectory(np.column_stack((np.arange(10), np.zeros(10), np.zeros(10))), orientation=orientation)
        item.orientation_payload[0, 0] = -1.0
        np.testing.assert_array_equal(orientation, before)

    def test_21_closed_transport_candidates_assume_holding_and_clearance(self) -> None:
        length = 120
        xyz = np.column_stack(
            (
                np.arange(length),
                np.zeros(length),
                np.r_[
                    np.full(30, 300.0),
                    np.linspace(300.0, 450.0, 30),
                    np.linspace(450.0, 350.0, 30),
                    np.full(30, 350.0),
                ],
            )
        )
        gripper = np.zeros(length, dtype=bool)
        gripper[30:90] = True
        result = generate_candidates(
            [trajectory(xyz, gripper)],
            role="a_cut",
            windows=[
                SemanticWindow(
                    "closed_transport",
                    10,
                    -10,
                    5,
                    "a_closed_holding_transport",
                    "closed",
                    25.0,
                )
            ],
            velocity_config={
                "velocity_window_frames": 7,
                "velocity_smoothing_method": "linear_regression",
                "velocity_epsilon": 1e-9,
            },
        )
        self.assertTrue(result)
        self.assertTrue(all(item.semantic_state.holding is True for item in result))
        self.assertTrue(all(item.semantic_state.gripper_closed for item in result))
        self.assertTrue(all(item.minimum_bridge_z_mm == 375.0 for item in result))
        self.assertTrue(all(item.position_mm[2] >= 375.0 for item in result))


class MetricsAndSelectionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.bridge = build_velocity_matched_bezier(
            np.array([0.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([10.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
            2.0,
        )
        self.metric, self.sample = evaluate_bridge(
            self.bridge,
            sample_hz=120.0,
            curvature_epsilon=1e-9,
            workspace_min_mm=np.array([-1, -1, -1]),
            workspace_max_mm=np.array([11, 1, 1]),
        )

    def test_11_path_length(self) -> None:
        self.assertAlmostEqual(self.metric.length_mm, 10.0, places=5)

    def test_12_max_velocity(self) -> None:
        self.assertAlmostEqual(
            self.metric.max_velocity_mm_s,
            float(np.max(np.linalg.norm(self.sample["velocity_mm_s"], axis=1))),
        )

    def test_13_max_acceleration(self) -> None:
        self.assertGreater(self.metric.max_acceleration_mm_s2, 0.0)

    def test_14_curvature(self) -> None:
        self.assertAlmostEqual(self.metric.max_curvature_per_mm, 0.0)

    def test_15_jerk(self) -> None:
        self.assertGreaterEqual(self.metric.max_jerk_mm_s3, 0.0)

    def test_16_workspace_rejection(self) -> None:
        metric, _ = evaluate_bridge(
            self.bridge,
            sample_hz=60,
            curvature_epsilon=1e-9,
            workspace_min_mm=np.array([1, -1, -1]),
            workspace_max_mm=np.array([9, 1, 1]),
        )
        reasons = feasibility_reasons(metric, self._limits())
        self.assertIn("workspace_violation", reasons)

    def test_17_acceleration_rejection(self) -> None:
        limits = self._limits()
        limits["acceleration_limit_mm_s2"] = 0.01
        self.assertIn("acceleration_limit", feasibility_reasons(self.metric, limits))

    def test_18_total_c_not_bridge_length_selects_winner(self) -> None:
        a = candidate("a_cut", 1, np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]]), [1, 0, 0])
        b = candidate("b_entry", 1, np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]]), [1, 0, 0])
        first = CandidateEvaluation(a, b, self.bridge, metrics(250), 300, 350, 900, True, [], True, [])
        second = CandidateEvaluation(a, b, self.bridge, metrics(70), 520, 490, 1080, True, [], True, [])
        self.assertIs(select_best_candidate([first, second], 0.0), first)

    def test_20_no_feasible_candidate_is_explicit(self) -> None:
        with self.assertRaises(NoFeasibleBridgeError):
            select_best_candidate([], 0.0)

    @staticmethod
    def _limits() -> dict[str, float]:
        return {
            "velocity_limit_mm_s": 1e9,
            "acceleration_limit_mm_s2": 1e9,
            "curvature_limit_per_mm": 1e9,
            "jerk_limit_mm_s3": 1e9,
            "integrated_squared_jerk_limit": 1e18,
            "backtracking_ratio_limit": 1e9,
        }


if __name__ == "__main__":
    unittest.main()
