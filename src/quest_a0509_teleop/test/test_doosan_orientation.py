import pytest

from quest_a0509_teleop.doosan_orientation import (
    apply_doosan_zyz_b_delta_deg,
    doosan_zyz_deg_to_quaternion,
    limit_doosan_zyz_geodesic_deg,
    quaternion_angle_deg,
    quaternion_to_doosan_zyz_deg,
    step_doosan_zyz_toward_deg,
)


def assert_same_orientation(left, right, tolerance_deg=1.0e-6):
    error = quaternion_angle_deg(
        doosan_zyz_deg_to_quaternion(left),
        doosan_zyz_deg_to_quaternion(right),
    )
    assert error <= tolerance_deg


@pytest.mark.parametrize(
    "pose",
    (
        [3.0, -179.0, 4.0],
        [3.0, 179.0, 4.0],
        [0.1, 151.8, -1.3],
        [30.0, 0.0, 20.0],
        [30.0, 180.0, 20.0],
    ),
)
def test_doosan_zyz_round_trip_stays_near_reference(pose):
    quaternion = doosan_zyz_deg_to_quaternion(pose)
    recovered = quaternion_to_doosan_zyz_deg(quaternion, pose)

    assert_same_orientation(recovered, pose)
    assert max(abs(recovered[index] - pose[index]) for index in range(3)) < 1.0e-5


def test_positive_beta_equivalent_family_has_same_physical_orientation():
    assert_same_orientation(
        [3.0, -179.0, 4.0],
        [183.0, 179.0, 184.0],
    )


def test_legacy_sign_and_gain_are_preserved_as_physical_b_axis_motion():
    anchor = [30.0, 150.0, 10.0]

    positive_quest_roll = apply_doosan_zyz_b_delta_deg(
        anchor,
        -0.7 * 10.0,
        anchor,
    )
    negative_quest_roll = apply_doosan_zyz_b_delta_deg(
        anchor,
        -0.7 * -10.0,
        anchor,
    )

    assert positive_quest_roll == pytest.approx([30.0, 143.0, 10.0], abs=1.0e-8)
    assert negative_quest_roll == pytest.approx([30.0, 157.0, 10.0], abs=1.0e-8)
    assert quaternion_angle_deg(
        doosan_zyz_deg_to_quaternion(anchor),
        doosan_zyz_deg_to_quaternion(positive_quest_roll),
    ) == pytest.approx(7.0, abs=1.0e-8)


def test_negative_180_crossing_keeps_all_zyz_components_continuous():
    anchor = [3.0, -172.0, 4.0]
    previous = anchor
    for delta in range(0, -31, -2):
        target = apply_doosan_zyz_b_delta_deg(anchor, float(delta), previous)
        expected = [anchor[0], anchor[1] + delta, anchor[2]]
        assert_same_orientation(target, expected)
        assert max(abs(target[index] - previous[index]) for index in range(3)) <= 2.0 + 1.0e-8
        previous = target


def test_positive_180_crossing_keeps_all_zyz_components_continuous():
    anchor = [3.0, 172.0, 4.0]
    previous = anchor
    for delta in range(0, 31, 2):
        target = apply_doosan_zyz_b_delta_deg(anchor, float(delta), previous)
        expected = [anchor[0], anchor[1] + delta, anchor[2]]
        assert_same_orientation(target, expected)
        assert max(abs(target[index] - previous[index]) for index in range(3)) <= 2.0 + 1.0e-8
        previous = target


def test_streamer_rotation_step_uses_one_degree_physical_slerp_at_180():
    current = [3.0, 179.0, 4.0]
    target = [3.0, 181.0, 4.0]

    stepped, requested_angle, limited = step_doosan_zyz_toward_deg(
        current,
        target,
        1.0,
    )

    assert limited
    assert requested_angle == pytest.approx(2.0, abs=1.0e-8)
    assert quaternion_angle_deg(
        doosan_zyz_deg_to_quaternion(current),
        doosan_zyz_deg_to_quaternion(stepped),
    ) == pytest.approx(1.0, abs=1.0e-8)
    assert stepped == pytest.approx([3.0, 180.0, 4.0], abs=1.0e-7)


def test_safety_limit_uses_geodesic_angle_instead_of_euler_slots():
    anchor = [3.0, 170.0, 4.0]
    target = [3.0, 270.0, 4.0]

    limited_target, requested_angle, limited = limit_doosan_zyz_geodesic_deg(
        anchor,
        target,
        90.0,
        anchor,
    )

    assert limited
    assert requested_angle == pytest.approx(100.0, abs=1.0e-8)
    assert quaternion_angle_deg(
        doosan_zyz_deg_to_quaternion(anchor),
        doosan_zyz_deg_to_quaternion(limited_target),
    ) == pytest.approx(90.0, abs=1.0e-8)
    assert_same_orientation(limited_target, [3.0, 260.0, 4.0])
