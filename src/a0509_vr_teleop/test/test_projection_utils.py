from a0509_vr_teleop.projection_utils import (
    DEFAULT_WORKSPACE,
    apply_ramp_limit,
    is_inside_workspace,
    project_to_workspace_boundary,
)


def test_inside_workspace_passes_without_projection():
    pose = [400.0, 0.0, 350.0, 0.0, 150.0, 0.0]
    assert is_inside_workspace(pose, DEFAULT_WORKSPACE)
    assert project_to_workspace_boundary(pose, pose, DEFAULT_WORKSPACE) == pose


def test_x_max_outside_projects_to_x_boundary():
    last = [400.0, 0.0, 350.0, 0.0, 150.0, 0.0]
    raw = [900.0, 0.0, 350.0, 0.0, 150.0, 0.0]
    projected = project_to_workspace_boundary(last, raw, DEFAULT_WORKSPACE)
    assert projected[0] == DEFAULT_WORKSPACE.x_max_mm
    assert projected[1:] == raw[1:]


def test_multi_axis_outside_uses_first_segment_boundary():
    last = [400.0, 0.0, 350.0, 0.0, 150.0, 0.0]
    raw = [900.0, 500.0, 350.0, 0.0, 150.0, 0.0]
    projected = project_to_workspace_boundary(last, raw, DEFAULT_WORKSPACE)
    assert projected[0] == DEFAULT_WORKSPACE.x_max_mm
    assert projected[1] == 350.0


def test_linear_ramp_does_not_exceed_20mm_per_tick():
    last = [400.0, 0.0, 350.0, 0.0, 150.0, 0.0]
    target = [460.0, -50.0, 390.0, 0.0, 150.0, 0.0]
    ramped = apply_ramp_limit(last, target, 20.0, 3.0)
    assert ramped[:3] == [420.0, -20.0, 370.0]


def test_angular_ramp_does_not_exceed_3deg_per_tick():
    last = [400.0, 0.0, 350.0, 0.0, 150.0, 0.0]
    target = [400.0, 0.0, 350.0, 10.0, 140.0, -8.0]
    ramped = apply_ramp_limit(last, target, 20.0, 3.0)
    assert ramped[3:] == [3.0, 147.0, -3.0]
