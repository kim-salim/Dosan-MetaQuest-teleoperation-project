import time
from pathlib import Path

from quest_a0509_teleop.metaquest_full_session_node import (
    live_fault_reason,
    MetaQuestFullSessionNode,
    pose_offset_metrics,
    pose_within_limits,
    SessionHealth,
    SessionMetrics,
    shortest_angle_delta_deg,
)


def test_initial_state_waits_for_fresh_quest_callbacks_after_user_input():
    node = object.__new__(MetaQuestFullSessionNode)
    node.calibration_valid = True
    node.teleop_ready = True
    node.source = 'DISABLED'
    node.live_state = False
    node.gripper_driver_busy = False
    node.gripper_last_command_ok = True
    node.heartbeat_timeout_sec = 1.0
    node.gripper_input_timeout_sec = 1.0
    node.last_pose_heartbeat = time.monotonic() - 5.0
    node.last_gripper_input = time.monotonic() - 5.0

    def spin_until(predicate, _timeout_sec, _label):
        assert not predicate()
        node.last_pose_heartbeat = time.monotonic()
        node.last_gripper_input = time.monotonic()
        assert predicate()

    node.spin_until = spin_until
    node._wait_for_initial_state()


def test_session_waits_for_anchor_published_after_set_anchor_request():
    source = (
        Path(__file__).resolve().parents[1]
        / 'quest_a0509_teleop'
        / 'metaquest_full_session_node.py'
    ).read_text()

    requested = source.index('anchor_requested_at = time.monotonic()')
    service_call = source.index("result['set_anchor'] =", requested)
    fresh_check = source.index(
        'self.anchor_time >= anchor_requested_at',
        service_call,
    )

    assert requested < service_call < fresh_check


def healthy_session(**overrides):
    values = {
        'calibration_valid': True,
        'teleop_ready': True,
        'source': 'METAQUEST',
        'selected_valid': True,
        'live_state': True,
        'pose_heartbeat_age_sec': 0.05,
        'gripper_input_age_sec': 0.05,
        'gripper_busy': False,
        'gripper_last_command_ok': True,
        'gripper_command_age_sec': None,
    }
    values.update(overrides)
    return SessionHealth(**values)


def fault(health):
    return live_fault_reason(
        health,
        heartbeat_timeout_sec=1.0,
        gripper_input_timeout_sec=1.0,
        gripper_command_timeout_sec=2.0,
    )


def test_shortest_angle_delta_wraps_at_180_degrees():
    assert shortest_angle_delta_deg(-179.0, 179.0) == 2.0
    assert shortest_angle_delta_deg(179.0, -179.0) == -2.0


def test_pose_offset_metrics_uses_xyz_norm_and_shortest_rotation():
    position, rotation, norm = pose_offset_metrics(
        [103.0, 204.0, 312.0, -179.0, 5.0, 179.0],
        [100.0, 200.0, 300.0, 179.0, 1.0, -179.0],
    )
    assert position == [3.0, 4.0, 12.0]
    assert rotation == [2.0, 4.0, -2.0]
    assert norm == 13.0


def test_preflight_waits_for_safety_ramp_to_converge():
    anchor = [432.0, -4.0, 457.0, 177.0, 210.0, 177.0]
    ramping = [432.0, -4.0, 457.0, 107.0, 166.0, 107.0]
    converged = [432.0, -4.0, 457.0, 175.0, 208.0, 175.0]
    assert not pose_within_limits(
        ramping,
        anchor,
        position_limit_mm=10.0,
        rotation_limit_deg=3.0,
    )
    assert pose_within_limits(
        converged,
        anchor,
        position_limit_mm=10.0,
        rotation_limit_deg=3.0,
    )


def test_healthy_live_session_has_no_fault():
    assert fault(healthy_session()) is None


def test_each_arm_gate_stops_the_session():
    assert 'calibration' in fault(
        healthy_session(calibration_valid=False)
    )
    assert 'teleop_ready' in fault(healthy_session(teleop_ready=False))
    assert 'source changed' in fault(healthy_session(source='DISABLED'))
    assert 'selected' in fault(healthy_session(selected_valid=False))
    assert 'Live' in fault(healthy_session(live_state=False))


def test_pose_and_gripper_input_watchdogs_stop_the_session():
    assert 'pose heartbeat' in fault(
        healthy_session(pose_heartbeat_age_sec=1.01)
    )
    assert 'gripper input' in fault(
        healthy_session(gripper_input_age_sec=1.01)
    )


def test_gripper_command_may_be_busy_inside_completion_deadline():
    assert fault(
        healthy_session(
            gripper_busy=True,
            gripper_last_command_ok=False,
            gripper_command_age_sec=1.9,
        )
    ) is None


def test_gripper_busy_timeout_and_failed_completion_stop_session():
    assert 'busy' in fault(
        healthy_session(
            gripper_busy=True,
            gripper_last_command_ok=False,
            gripper_command_age_sec=2.01,
        )
    )
    assert 'failed' in fault(
        healthy_session(
            gripper_busy=False,
            gripper_last_command_ok=False,
            gripper_command_age_sec=2.01,
        )
    )


def test_metrics_track_motion_without_applying_a_session_envelope():
    metrics = SessionMetrics()
    anchor = [400.0, 0.0, 350.0, 0.0, 179.0, 0.0]
    metrics.record_safe(
        [520.0, 160.0, 300.0, 0.0, -179.0, 0.0],
        anchor,
    )
    assert metrics.safe_samples == 1
    assert metrics.max_target_position_norm_mm > 200.0
    assert metrics.max_target_rotation_abs_deg == 2.0


def test_gripper_validation_requires_both_physical_directions():
    metrics = SessionMetrics(gripper_driver_commands=['stop', 'close'])
    assert not metrics.gripper_validation_pass
    metrics.gripper_driver_commands.extend(['stop', 'open', 'stop'])
    assert metrics.gripper_validation_pass


def test_accepted_commands_alone_do_not_pass_gripper_validation():
    metrics = SessionMetrics(
        gripper_driver_accepted_commands=['close', 'open'],
    )
    assert not metrics.gripper_validation_pass
