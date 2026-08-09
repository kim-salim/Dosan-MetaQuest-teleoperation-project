from jrt_gripper_io.driver_diagnostics import GripperDriverDiagnostics


def test_accepted_command_sets_busy_and_pending_result():
    state = GripperDriverDiagnostics()
    state.accept("close")
    assert state.accepted_command == "close"
    assert state.busy
    assert not state.last_command_ok


def test_successful_close_updates_commanded_state():
    state = GripperDriverDiagnostics()
    state.accept("close")
    state.complete("close")
    assert not state.busy
    assert state.last_command_ok
    assert state.commanded_state == 1.0


def test_successful_open_updates_commanded_state():
    state = GripperDriverDiagnostics(commanded_state=1.0)
    state.accept("open")
    state.complete("open")
    assert state.commanded_state == 0.0


def test_stop_completion_preserves_commanded_state_latch():
    state = GripperDriverDiagnostics(commanded_state=1.0)
    state.accept("stop")
    state.complete("stop")
    assert state.commanded_state == 1.0
    assert state.last_command_ok


def test_service_failure_does_not_update_commanded_state():
    state = GripperDriverDiagnostics(commanded_state=0.0)
    state.accept("close")
    state.fail()
    assert not state.busy
    assert not state.last_command_ok
    assert state.commanded_state == 0.0


def test_successful_failsafe_stop_preserves_original_failure_result():
    state = GripperDriverDiagnostics(commanded_state=0.0)
    state.accept("close")
    state.fail()
    state.accept("stop")
    state.complete("stop", preserve_failure=True)
    assert not state.busy
    assert not state.last_command_ok
    assert state.commanded_state == 0.0


def test_dry_run_completion_does_not_claim_hardware_commanded_state():
    state = GripperDriverDiagnostics()
    state.accept("close")
    state.complete("close", update_commanded_state=False)
    assert state.commanded_state is None
