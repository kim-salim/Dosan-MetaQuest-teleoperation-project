from jrt_gripper_io.gripper_logic import (
    ToolDoStep,
    command_from_buttons,
    plan_tool_do_command,
    plan_tool_do_sequence,
)


def test_command_from_buttons_a_only_closes():
    assert command_from_buttons([1, 0], 0, 1) == "close"


def test_command_from_buttons_b_only_opens():
    assert command_from_buttons([0, 1], 0, 1) == "open"


def test_command_from_buttons_none_stops():
    assert command_from_buttons([0, 0], 0, 1) == "stop"


def test_command_from_buttons_both_stops():
    assert command_from_buttons([1, 1], 0, 1) == "stop"


def test_close_plan_turns_open_off_then_close_on():
    assert plan_tool_do_command("close", 1, 2) == [(2, 0), (1, 1), (1, 0)]


def test_open_plan_turns_close_off_then_open_on():
    assert plan_tool_do_command("open", 1, 2) == [(1, 0), (2, 1), (2, 0)]


def test_stop_plan_turns_both_off():
    assert plan_tool_do_command("stop", 1, 2) == [(1, 0), (2, 0)]


def test_level_close_plan_holds_close_on():
    assert plan_tool_do_command("close", 1, 2, command_mode="level") == [
        (2, 0),
        (1, 1),
    ]


def test_pulse_close_sequence_has_interlock_and_pulse_delays():
    assert plan_tool_do_sequence(
        "close",
        1,
        2,
        pulse_sec=0.2,
        interlock_sec=0.05,
    ) == [
        ToolDoStep(2, 0, 0.05),
        ToolDoStep(1, 1, 0.2),
        ToolDoStep(1, 0, 0.0),
    ]
