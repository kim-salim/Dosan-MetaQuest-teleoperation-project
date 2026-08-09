import math

from lerobot_robot_doosan_a0509.episode_reset_orchestrator import (
    pose_is_near_anchor,
    pose_window_is_stable,
    quaternion_angle_deg,
    QuestPoseSample,
)


def sample(t, x=0.0, yaw_quaternion=(0.0, 0.0, 0.0, 1.0)):
    return QuestPoseSample(
        receive_time=t,
        position_m=(x, 0.0, 0.0),
        quaternion_xyzw=yaw_quaternion,
    )


def test_quaternion_angle_is_sign_invariant():
    assert quaternion_angle_deg((0, 0, 0, 1), (0, 0, 0, -1)) == 0.0


def test_pose_window_requires_fresh_samples_covering_the_interval():
    assert not pose_window_is_stable(
        [sample(9.8), sample(10.0)],
        now=10.0,
        window_sec=0.5,
        max_age_sec=0.3,
        max_translation_m=0.008,
        max_rotation_deg=5.0,
    )
    samples = [sample(9.5 + index * 0.1, x=index * 0.0005) for index in range(6)]
    assert pose_window_is_stable(
        samples,
        now=10.0,
        window_sec=0.5,
        max_age_sec=0.3,
        max_translation_m=0.008,
        max_rotation_deg=5.0,
    )


def test_pose_window_rejects_motion_and_stale_input():
    moving = [sample(9.5), sample(9.7, x=0.02), sample(10.0)]
    assert not pose_window_is_stable(
        moving,
        now=10.0,
        window_sec=0.5,
        max_age_sec=0.3,
        max_translation_m=0.008,
        max_rotation_deg=5.0,
    )
    stable = [sample(9.0 + index * 0.1) for index in range(6)]
    assert not pose_window_is_stable(
        stable,
        now=10.0,
        window_sec=0.5,
        max_age_sec=0.3,
        max_translation_m=0.008,
        max_rotation_deg=5.0,
    )


def test_pose_window_rejects_large_rotation():
    ten_degrees = math.radians(10.0 / 2.0)
    samples = [sample(9.5 + index * 0.1) for index in range(5)]
    samples.append(
        sample(
            10.0,
            yaw_quaternion=(
                0.0,
                0.0,
                math.sin(ten_degrees),
                math.cos(ten_degrees),
            ),
        )
    )
    assert not pose_window_is_stable(
        samples,
        now=10.0,
        window_sec=0.5,
        max_age_sec=0.3,
        max_translation_m=0.008,
        max_rotation_deg=5.0,
    )


def test_preflight_pose_uses_shortest_rotation_around_180_degrees():
    anchor = [400.0, 0.0, 350.0, 0.0, 179.0, 0.0]
    near = [403.0, 4.0, 350.0, 0.0, -179.0, 0.0]
    assert pose_is_near_anchor(
        near,
        anchor,
        position_limit_mm=10.0,
        rotation_limit_deg=3.0,
    )
