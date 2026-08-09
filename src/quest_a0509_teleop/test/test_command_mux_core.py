import math

from quest_a0509_teleop.command_mux_core import CommandMuxCore, ControlSource


POSX = [400.0, 0.0, 350.0, 0.0, 60.0, 0.0]


def ready_core() -> CommandMuxCore:
    core = CommandMuxCore()
    core.set_teleop_ready(True)
    core.set_metaquest_calibration_valid(True, now=0.0)
    return core


def test_initial_source_is_disabled():
    assert CommandMuxCore().source is ControlSource.DISABLED


def test_metaquest_selection_requires_valid_calibration():
    core = CommandMuxCore()
    core.set_teleop_ready(True)
    decision = core.select_source(ControlSource.METAQUEST, now=0.0)
    assert not decision.accepted
    assert "calibration is invalid" in decision.reason
    assert core.source is ControlSource.DISABLED


def test_invalid_calibration_disables_active_metaquest_and_requests_hold():
    core = ready_core()
    core.select_source(ControlSource.METAQUEST, now=0.0)
    core.set_live_enabled(True)
    decision = core.set_metaquest_calibration_valid(False, now=0.1)
    assert decision is not None and decision.accepted
    assert decision.source_changed
    assert decision.disable_and_hold
    assert decision.gripper_command == "stop"
    assert core.source is ControlSource.DISABLED
    assert not core.live_enabled


def test_invalid_metaquest_calibration_does_not_block_lerobot():
    core = CommandMuxCore()
    core.set_teleop_ready(True)
    core.set_metaquest_calibration_valid(False, now=0.0)
    assert core.select_source(ControlSource.LEROBOT, now=0.0).accepted
    decision = core.receive_arm_target(ControlSource.LEROBOT, POSX, now=0.01)
    assert decision.target_posx == tuple(POSX)


def test_source_never_changes_from_input_alone():
    core = ready_core()
    core.note_metaquest_valid_pose(0.0)
    decision = core.receive_arm_target(ControlSource.METAQUEST, POSX, now=0.0)
    assert not decision.accepted
    assert core.source is ControlSource.DISABLED


def test_active_source_transition_requires_live_false():
    core = ready_core()
    assert core.select_source(ControlSource.METAQUEST, now=0.0).accepted
    core.set_live_enabled(True)
    decision = core.select_source(ControlSource.LEROBOT, now=0.1)
    assert not decision.accepted
    assert core.source is ControlSource.METAQUEST


def test_disabled_transition_is_allowed_while_live():
    core = ready_core()
    core.select_source(ControlSource.METAQUEST, now=0.0)
    core.set_live_enabled(True)
    decision = core.select_source(ControlSource.DISABLED, now=0.1)
    assert decision.accepted
    assert decision.gripper_command == "stop"
    assert decision.disable_and_hold
    assert not core.live_enabled
    assert core.source is ControlSource.DISABLED


def test_repeated_disabled_request_still_requests_stop_and_hold():
    core = ready_core()
    core.set_live_enabled(True)
    decision = core.select_source(ControlSource.DISABLED, now=0.0)
    assert decision.accepted
    assert decision.gripper_command == "stop"
    assert decision.disable_and_hold
    assert not core.live_enabled


def test_source_change_clears_target_and_requires_new_one():
    core = ready_core()
    core.note_metaquest_valid_pose(0.0)
    decision = core.select_source(ControlSource.METAQUEST, now=0.0)
    assert decision.gripper_command == "stop"
    assert not core.has_new_selected_target
    gripper = core.receive_metaquest_gripper("close", now=0.01)
    assert gripper.gripper_command is None
    arm = core.receive_arm_target(ControlSource.METAQUEST, POSX, now=0.02)
    assert arm.target_posx == tuple(POSX)
    assert arm.emit_heartbeat


def test_target_validation_rejects_wrong_length_nan_and_inf():
    core = ready_core()
    core.select_source(ControlSource.LEROBOT, now=0.0)
    for bad in ([1.0] * 5, [*POSX[:5], math.nan], [*POSX[:5], math.inf]):
        decision = core.receive_arm_target(ControlSource.LEROBOT, bad, now=0.01)
        assert not decision.accepted
        assert decision.target_posx is None


def test_metaquest_timeout_uses_mapper_accepted_pose_heartbeat():
    core = ready_core()
    core.note_metaquest_valid_pose(0.0)
    core.select_source(ControlSource.METAQUEST, now=0.0)
    core.receive_arm_target(ControlSource.METAQUEST, POSX, now=0.01)
    assert core.check_timeout(now=0.99) is None
    timeout = core.check_timeout(now=1.01)
    assert timeout is not None and timeout.timed_out
    assert timeout.disable_and_hold
    assert core.source is ControlSource.DISABLED


def test_lerobot_timeout_uses_target_reception():
    core = ready_core()
    core.select_source(ControlSource.LEROBOT, now=0.0)
    core.receive_arm_target(ControlSource.LEROBOT, POSX, now=0.05)
    assert core.check_timeout(now=0.34) is None
    timeout = core.check_timeout(now=0.36)
    assert timeout is not None and timeout.timed_out
    assert timeout.disable_and_hold
    assert core.source is ControlSource.DISABLED


def test_timeout_has_no_automatic_recovery():
    core = ready_core()
    core.select_source(ControlSource.LEROBOT, now=0.0)
    assert core.check_timeout(now=0.31).timed_out
    core.receive_arm_target(ControlSource.LEROBOT, POSX, now=0.32)
    assert core.source is ControlSource.DISABLED


def test_lerobot_gripper_hysteresis_and_duplicate_suppression():
    core = ready_core()
    core.update_commanded_gripper_state(0.0)
    core.select_source(ControlSource.LEROBOT, now=0.0)
    core.receive_arm_target(ControlSource.LEROBOT, POSX, now=0.01)
    core.set_live_enabled(True)
    assert core.receive_lerobot_gripper(0.5, now=0.02).gripper_command is None
    assert core.receive_lerobot_gripper(0.1, now=0.03).gripper_command is None
    close = core.receive_lerobot_gripper(0.9, now=0.04)
    assert close.gripper_command == "close"
    duplicate = core.receive_lerobot_gripper(0.9, now=0.05)
    assert duplicate.gripper_command is None


def test_arm_and_gripper_always_follow_selected_source():
    core = ready_core()
    core.note_metaquest_valid_pose(0.0)
    core.select_source(ControlSource.METAQUEST, now=0.0)
    ignored_arm = core.receive_arm_target(ControlSource.LEROBOT, POSX, now=0.01)
    ignored_gripper = core.receive_lerobot_gripper(1.0, now=0.01)
    assert ignored_arm.target_posx is None
    assert ignored_gripper.gripper_command is None


def test_metaquest_release_does_not_preempt_required_tool_do_pulse():
    core = CommandMuxCore(metaquest_gripper_min_pulse_sec=0.25)
    core.set_teleop_ready(True)
    core.set_metaquest_calibration_valid(True, now=0.0)
    core.note_metaquest_valid_pose(0.0)
    core.select_source(ControlSource.METAQUEST, now=0.0)
    core.receive_arm_target(ControlSource.METAQUEST, POSX, now=0.01)
    core.set_live_enabled(True)

    close = core.receive_metaquest_gripper("close", now=0.02)
    release = core.receive_metaquest_gripper("stop", now=0.22)
    close_again = core.receive_metaquest_gripper("close", now=0.40)

    assert close.gripper_command == "close"
    assert release.gripper_command is None
    assert "pulse can self-complete" in release.reason
    assert close_again.gripper_command == "close"


def test_default_release_window_protects_extended_physical_pulse_but_not_live_stop():
    core = CommandMuxCore()
    core.set_teleop_ready(True)
    core.set_metaquest_calibration_valid(True, now=0.0)
    core.note_metaquest_valid_pose(0.0)
    core.select_source(ControlSource.METAQUEST, now=0.0)
    core.receive_arm_target(ControlSource.METAQUEST, POSX, now=0.01)
    core.set_live_enabled(True)

    close = core.receive_metaquest_gripper("close", now=0.02)
    core.note_metaquest_valid_pose(1.0)
    core.receive_arm_target(ControlSource.METAQUEST, POSX, now=1.0)
    release = core.receive_metaquest_gripper("stop", now=1.02)
    safety_stop = core.set_live_enabled(False)

    assert close.gripper_command == "close"
    assert release.gripper_command is None
    assert "1.500s" in release.reason
    assert safety_stop is not None
    assert safety_stop.gripper_command == "stop"


def test_metaquest_watchdog_stop_after_pulse_window_is_forwarded():
    core = CommandMuxCore(metaquest_gripper_min_pulse_sec=0.25)
    core.set_teleop_ready(True)
    core.set_metaquest_calibration_valid(True, now=0.0)
    core.note_metaquest_valid_pose(0.0)
    core.select_source(ControlSource.METAQUEST, now=0.0)
    core.receive_arm_target(ControlSource.METAQUEST, POSX, now=0.01)
    core.set_live_enabled(True)
    core.receive_metaquest_gripper("open", now=0.02)

    stop = core.receive_metaquest_gripper("stop", now=0.32)

    assert stop.gripper_command == "stop"


def test_calibration_invalidation_stop_bypasses_pulse_grace():
    core = CommandMuxCore(metaquest_gripper_min_pulse_sec=0.25)
    core.set_teleop_ready(True)
    core.set_metaquest_calibration_valid(True, now=0.0)
    core.note_metaquest_valid_pose(0.0)
    core.select_source(ControlSource.METAQUEST, now=0.0)
    core.receive_arm_target(ControlSource.METAQUEST, POSX, now=0.01)
    core.set_live_enabled(True)
    core.receive_metaquest_gripper("close", now=0.02)

    invalidated = core.set_metaquest_calibration_valid(False, now=0.03)

    assert invalidated is not None
    assert invalidated.gripper_command == "stop"
    assert invalidated.disable_and_hold


def test_live_gate_stops_and_rejects_gripper_until_reenabled():
    core = ready_core()
    core.note_metaquest_valid_pose(0.0)
    core.select_source(ControlSource.METAQUEST, now=0.0)
    core.receive_arm_target(ControlSource.METAQUEST, POSX, now=0.01)

    while_disabled = core.receive_metaquest_gripper("close", now=0.02)
    assert not while_disabled.accepted
    assert while_disabled.gripper_command is None

    core.set_live_enabled(True)
    close = core.receive_metaquest_gripper("close", now=0.03)
    assert close.gripper_command == "close"

    disabled = core.set_live_enabled(False)
    assert disabled is not None
    assert disabled.gripper_command == "stop"

    after_disable = core.receive_metaquest_gripper("open", now=0.04)
    assert not after_disable.accepted
    assert after_disable.gripper_command is None
