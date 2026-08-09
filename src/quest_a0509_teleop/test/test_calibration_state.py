import json

from quest_a0509_teleop.calibration_state import CalibrationState


def test_required_calibration_starts_invalid_and_serializes_contract():
    state = CalibrationState(
        initial_correction_deg=3.0,
        mapping_fingerprint="abc123",
        required=True,
    )
    payload = json.loads(state.snapshot.to_json())
    assert payload["state"] == "UNCALIBRATED"
    assert payload["valid"] is False
    assert payload["mapping_fingerprint"] == "abc123"
    assert payload["xy_yaw_correction_deg"] == 3.0


def test_calibration_complete_exposes_quality_and_reset_invalidates():
    state = CalibrationState(
        initial_correction_deg=0.0,
        mapping_fingerprint="mapping-v1",
        required=True,
    )
    started = state.start(0.0)
    assert started.state == "CALIBRATING"
    completed = state.complete(
        correction_deg=-12.5,
        observed_angle_deg=12.5,
        distance_m=0.08,
        before_yaw_mm=[78.1, 17.3, 0.0],
        after_yaw_mm=[80.0, 0.0, 0.0],
    )
    assert completed.state == "VALID"
    assert completed.valid
    assert completed.distance_m == 0.08
    assert completed.position_delta_after_yaw_mm == [80.0, 0.0, 0.0]

    reset = state.invalidate("reset", reset_correction=True)
    assert reset.state == "INVALID"
    assert not reset.valid
    assert reset.xy_yaw_correction_deg == 0.0


def test_optional_session_calibration_starts_valid():
    state = CalibrationState(
        initial_correction_deg=5.0,
        mapping_fingerprint="mapping-v1",
        required=False,
    )
    assert state.snapshot.valid
    assert state.snapshot.state == "VALID"
