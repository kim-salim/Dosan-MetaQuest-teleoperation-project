import math

import pytest

from quest_a0509_teleop.pose_resampler import (
    fixed_rate_alpha,
    PoseResampler,
    quaternion_slerp,
)


IDENTITY = (0.0, 0.0, 0.0, 1.0)


def yaw_quaternion(degrees: float) -> tuple[float, float, float, float]:
    half = math.radians(degrees) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def make_resampler(**overrides) -> PoseResampler:
    parameters = {
        "max_samples": 128,
        "buffer_duration_sec": 1.0,
        "max_interpolation_gap_sec": 0.08,
        "full_prediction_sec": 0.02,
        "prediction_decay_sec": 0.08,
        "velocity_estimation_min_sec": 0.02,
        "max_prediction_linear_speed_m_s": 0.5,
        "max_prediction_angular_speed_deg_s": 90.0,
    }
    parameters.update(overrides)
    return PoseResampler(**parameters)


def test_position_lerp_and_quaternion_slerp_share_query_fraction():
    resampler = make_resampler()
    assert resampler.push(
        receive_time_sec=0.0,
        position_m=(0.0, 0.0, 0.0),
        quaternion_xyzw=IDENTITY,
    )
    assert resampler.push(
        receive_time_sec=0.05,
        position_m=(1.0, 2.0, 3.0),
        quaternion_xyzw=yaw_quaternion(90.0),
    )

    pose = resampler.sample(0.025)

    assert pose is not None
    assert pose.mode == "interpolate"
    assert pose.position_m == pytest.approx((0.5, 1.0, 1.5))
    assert pose.quaternion_xyzw == pytest.approx(yaw_quaternion(45.0))


def test_bursty_receive_gap_is_interpolated_at_fixed_query_time():
    resampler = make_resampler()
    for timestamp, position in (
        (0.000, 0.00),
        (0.003, 0.01),
        (0.006, 0.02),
        (0.009, 0.03),
        (0.059, 0.04),
        (0.062, 0.05),
    ):
        assert resampler.push(
            receive_time_sec=timestamp,
            position_m=(position, 0.0, 0.0),
            quaternion_xyzw=IDENTITY,
        )

    pose = resampler.sample(0.034)

    assert pose is not None
    assert pose.mode == "interpolate"
    assert pose.position_m[0] == pytest.approx(0.035)


def test_prediction_speed_is_bounded_then_decays_to_a_hold():
    resampler = make_resampler()
    resampler.push(
        receive_time_sec=0.0,
        position_m=(0.0, 0.0, 0.0),
        quaternion_xyzw=IDENTITY,
    )
    resampler.push(
        receive_time_sec=0.05,
        position_m=(0.05, 0.0, 0.0),
        quaternion_xyzw=IDENTITY,
    )

    short = resampler.sample(0.06)
    decaying = resampler.sample(0.10)
    held = resampler.sample(0.14)
    held_later = resampler.sample(0.20)

    assert short is not None and short.mode == "extrapolate"
    assert short.position_m[0] == pytest.approx(0.055)
    assert decaying is not None and decaying.mode == "decay"
    assert decaying.position_m[0] == pytest.approx(0.07125)
    assert held is not None and held.mode == "hold"
    assert held_later is not None and held_later.mode == "hold"
    assert held.position_m == pytest.approx((0.075, 0.0, 0.0))
    assert held_later.position_m == pytest.approx(held.position_m)


def test_large_receive_gap_is_not_interpolated_as_continuous_motion():
    resampler = make_resampler(max_interpolation_gap_sec=0.08)
    resampler.push(
        receive_time_sec=0.0,
        position_m=(0.0, 0.0, 0.0),
        quaternion_xyzw=IDENTITY,
    )
    resampler.push(
        receive_time_sec=0.2,
        position_m=(1.0, 0.0, 0.0),
        quaternion_xyzw=yaw_quaternion(90.0),
    )

    pose = resampler.sample(0.1)

    assert pose is not None
    assert pose.mode == "hold"
    assert pose.position_m == pytest.approx((0.0, 0.0, 0.0))
    assert pose.quaternion_xyzw == pytest.approx(IDENTITY)


def test_non_monotonic_receive_time_is_rejected_and_clear_removes_history():
    resampler = make_resampler()
    assert resampler.push(
        receive_time_sec=1.0,
        position_m=(0.0, 0.0, 0.0),
        quaternion_xyzw=IDENTITY,
    )
    assert not resampler.push(
        receive_time_sec=1.0,
        position_m=(1.0, 0.0, 0.0),
        quaternion_xyzw=IDENTITY,
    )
    assert resampler.sample_count == 1

    resampler.clear()

    assert resampler.sample_count == 0
    assert resampler.sample(1.0) is None


def test_fixed_rate_filter_preserves_nominal_time_constant():
    tick_alpha = fixed_rate_alpha(0.4, 72.0, 1.0 / 30.0)

    assert tick_alpha == pytest.approx(0.706530, abs=1.0e-5)


def test_slerp_handles_equivalent_opposite_quaternion_sign():
    output = quaternion_slerp(IDENTITY, (0.0, 0.0, 0.0, -1.0), 0.5)

    assert output == pytest.approx(IDENTITY)
