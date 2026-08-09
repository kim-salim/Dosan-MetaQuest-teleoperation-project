from pathlib import Path

from jrt_gripper_io.gripper_logic import command_from_buttons, plan_tool_do_command


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parent


def test_physical_gripper_mapping_defaults_to_do1_open_do2_close():
    driver = (
        PACKAGE_ROOT / "jrt_gripper_io" / "jrt_tool_io_driver_node.py"
    ).read_text()
    io_launch = (PACKAGE_ROOT / "launch" / "jrt_gripper_io.launch.py").read_text()
    robot_launch = (
        PACKAGE_ROOT / "launch" / "jrt_gripper_robot_bringup.launch.py"
    ).read_text()
    full_launch = (
        WORKSPACE_SRC
        / "quest_a0509_teleop"
        / "launch"
        / "a0509_full_bringup_with_gripper.launch.py"
    ).read_text()

    assert 'self.declare_parameter("close_do_index", 2)' in driver
    assert 'self.declare_parameter("open_do_index", 1)' in driver
    assert "GetToolDigitalOutput" in driver
    assert "/dsr01/dsr_controller2/io/get_tool_digital_output" in driver
    assert 'self.declare_parameter("pulse_sec", 0.50)' in driver
    for launch in (io_launch, robot_launch):
        assert 'DeclareLaunchArgument("close_do_index", default_value="2")' in launch
        assert 'DeclareLaunchArgument("open_do_index", default_value="1")' in launch
        assert 'DeclareLaunchArgument(\n                "get_tool_do_service"' in launch
        assert 'DeclareLaunchArgument("pulse_sec", default_value="0.50")' in launch
    assert (
        'DeclareLaunchArgument("gripper_close_do_index", default_value="2")'
        in full_launch
    )
    assert (
        'DeclareLaunchArgument("gripper_open_do_index", default_value="1")'
        in full_launch
    )
    assert '"gripper_get_tool_do_service"' in full_launch
    assert '"gripper_readback_timeout_sec"' in full_launch
    assert '"gripper_readback_poll_sec"' in full_launch
    assert "/dsr01/dsr_controller2/io/get_tool_digital_output" in full_launch
    assert 'DeclareLaunchArgument("gripper_pulse_sec", default_value="0.50")' in full_launch


def test_manual_scripts_use_confirmed_physical_mapping():
    for name in (
        "gripper_all_off.sh",
        "gripper_close_pulse.sh",
        "gripper_open_pulse.sh",
    ):
        script = (PACKAGE_ROOT / "scripts" / name).read_text()
        assert 'CLOSE_INDEX="${CLOSE_INDEX:-2}"' in script
        assert 'OPEN_INDEX="${OPEN_INDEX:-1}"' in script
        if name != "gripper_all_off.sh":
            assert 'PULSE_SEC="${PULSE_SEC:-0.50}"' in script


def test_quest_buttons_resolve_to_confirmed_physical_outputs():
    """Lock the complete A/B -> semantic command -> physical DO contract."""
    close_command = command_from_buttons([1, 0], 0, 1)
    open_command = command_from_buttons([0, 1], 0, 1)

    assert close_command == "close"
    assert open_command == "open"
    assert plan_tool_do_command(close_command, 2, 1) == [
        (1, 0),
        (2, 1),
        (2, 0),
    ]
    assert plan_tool_do_command(open_command, 2, 1) == [
        (2, 0),
        (1, 1),
        (1, 0),
    ]
