from pathlib import Path

from quest_a0509_teleop.safety_guard_node import clamp_workspace_axis


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_x_minimum_is_clamped_at_fifty_millimeters():
    assert clamp_workspace_axis(40.0, 50.0, 650.0, True) == 50.0
    assert clamp_workspace_axis(70.0, 50.0, 650.0, True) == 70.0


def test_z_minimum_is_disabled_but_z_maximum_remains_active():
    assert clamp_workspace_axis(-250.0, 0.0, 600.0, False) == -250.0
    assert clamp_workspace_axis(700.0, 0.0, 600.0, False) == 600.0


def test_runtime_config_disables_only_the_z_minimum():
    config = (
        PACKAGE_ROOT
        / "config"
        / "xyz_position_only.yaml"
    ).read_text()

    assert "workspace_min_xyz_mm: [50.0, -350.0, 0.0]" in config
    assert "workspace_min_limit_enabled: [true, true, false]" in config
