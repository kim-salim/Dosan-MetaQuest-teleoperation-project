from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_active_plan_queues_commands_and_stop_has_priority():
    source = (
        PACKAGE_ROOT / "jrt_gripper_io" / "jrt_tool_io_driver_node.py"
    ).read_text()
    active_handler = source.split("        if self._is_plan_active():", 1)[1].split(
        '        if command != "stop" and command == self._last_command:', 1
    )[0]
    assert "self._pending_command.offer(command)" in active_handler
    assert "self._interrupt_active_plan_for_stop()" in active_handler
    assert "while Tool DO plan is active" in active_handler
    assert "ignoring gripper command" not in active_handler


def test_every_async_tool_do_call_has_a_response_deadline_and_failsafe_stop():
    source = (
        PACKAGE_ROOT / "jrt_gripper_io" / "jrt_tool_io_driver_node.py"
    ).read_text()
    assert "self._step_timeout_timer = self.create_timer(" in source
    assert "Tool DO service response timed out after" in source
    assert 'self._pending_command.offer("stop")' in source
    assert "self._start_pending_command(failsafe=True)" in source


def test_completed_command_is_emitted_only_after_the_plan_completes():
    source = (
        PACKAGE_ROOT / "jrt_gripper_io" / "jrt_tool_io_driver_node.py"
    ).read_text()
    completion = source.split(
        "if not self._plan_steps:",
        1,
    )[1].split("step = self._plan_steps.popleft()", 1)[0]
    assert '"completed_command_topic"' in source
    assert 'completed_command=(\n' in completion
    assert "if self.diagnostics.last_command_ok" in completion


def test_pulse_delay_starts_only_after_requested_tool_do_readback_is_confirmed():
    source = (
        PACKAGE_ROOT / "jrt_gripper_io" / "jrt_tool_io_driver_node.py"
    ).read_text()
    set_response = source.split(
        "def _on_step_response",
        1,
    )[1].split("def _request_step_readback", 1)[0]
    readback_response = source.split(
        "def _on_readback_response",
        1,
    )[1].split("def _on_readback_poll", 1)[0]
    confirmed_step = source.split(
        "def _finish_confirmed_step",
        1,
    )[1].split("def _on_step_timeout", 1)[0]

    assert "self._request_step_readback(step)" in set_response
    assert "if observed == expected:" in readback_response
    assert "self._finish_confirmed_step(step)" in readback_response
    assert "step.delay_after_sec" not in set_response
    assert "step.delay_after_sec" in confirmed_step


def test_readback_timeout_enters_the_existing_failsafe_stop_path():
    source = (
        PACKAGE_ROOT / "jrt_gripper_io" / "jrt_tool_io_driver_node.py"
    ).read_text()
    timeout_handler = source.split(
        "def _handle_readback_timeout",
        1,
    )[1].split("def _finish_confirmed_step", 1)[0]

    assert 'self.declare_parameter("readback_timeout_sec", 0.5)' in source
    assert 'self.declare_parameter("readback_poll_sec", 0.01)' in source
    assert "self._handle_service_failure()" in timeout_handler
    assert 'self._pending_command.offer("stop")' in source
