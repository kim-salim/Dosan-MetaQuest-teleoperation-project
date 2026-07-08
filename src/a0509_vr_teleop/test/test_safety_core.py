from a0509_vr_teleop.projection_utils import DEFAULT_WORKSPACE
from a0509_vr_teleop.safety_core import SafetyGuardCore, SafetyInput


def test_tracking_lost_holds_last_target():
    core = SafetyGuardCore(DEFAULT_WORKSPACE)
    core.reset([400.0, 0.0, 350.0, 0.0, 150.0, 0.0])
    output = core.update(
        SafetyInput(
            raw_pose=[450.0, 0.0, 350.0, 0.0, 150.0, 0.0],
            tracking_ok=False,
            deadman_pressed=True,
            stamp_sec=1.0,
        ),
        now_sec=1.0,
    )
    assert output.hold
    assert output.reason == "tracking_lost"
    assert output.target_pose == [400.0, 0.0, 350.0, 0.0, 150.0, 0.0]


def test_deadman_release_holds_last_target():
    core = SafetyGuardCore(DEFAULT_WORKSPACE)
    core.reset([400.0, 0.0, 350.0, 0.0, 150.0, 0.0])
    output = core.update(
        SafetyInput(
            raw_pose=[450.0, 0.0, 350.0, 0.0, 150.0, 0.0],
            tracking_ok=True,
            deadman_pressed=False,
            stamp_sec=1.0,
        ),
        now_sec=1.0,
    )
    assert output.hold
    assert output.reason == "deadman_released"
    assert output.target_pose == [400.0, 0.0, 350.0, 0.0, 150.0, 0.0]


def test_outside_workspace_projects_then_ramps():
    core = SafetyGuardCore(DEFAULT_WORKSPACE, linear_ramp_mm_per_tick=20.0)
    core.reset([400.0, 0.0, 350.0, 0.0, 150.0, 0.0])
    output = core.update(
        SafetyInput(
            raw_pose=[900.0, 0.0, 350.0, 0.0, 150.0, 0.0],
            tracking_ok=True,
            deadman_pressed=True,
            stamp_sec=1.0,
        ),
        now_sec=1.0,
    )
    assert output.projected
    assert output.projected_pose[0] == DEFAULT_WORKSPACE.x_max_mm
    assert output.target_pose[0] == 420.0
