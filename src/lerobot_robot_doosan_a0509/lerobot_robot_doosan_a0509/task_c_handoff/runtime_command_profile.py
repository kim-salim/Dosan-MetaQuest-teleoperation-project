"""Versioned Task-C command profiles shared by planning and live runtime.

The profile binds the ServoL ramp, bounded ACK pipeline, and proposal-command
dynamics into one identifier.  It does not represent a Doosan hardware
rating and loading it performs no ROS or robot I/O.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any


LEGACY_RUNTIME_COMMAND_PROFILE_ID = "a0509_ramp7p5_ack_span2_v1"
RAMP8P5_RUNTIME_COMMAND_PROFILE_ID = "a0509_ramp8p5_ack_span2_v1"


@dataclass(frozen=True)
class RuntimeCommandProfile:
    profile_id: str
    control_hz: float
    linear_ramp_mm_per_tick: float
    orientation_ramp_deg_per_tick: float
    bridge_ack_mode: str
    max_ack_lag_steps: int
    acknowledged_command_span_steps: int
    axis_velocity_limit_mm_s: float
    ack_safe_axis_speed_envelope_mm_s: float
    cartesian_velocity_limit_mm_s: float
    acceleration_limit_mm_s2: float
    jerk_limit_mm_s3: float
    integrated_squared_jerk_limit: float

    def __post_init__(self) -> None:
        if not self.profile_id:
            raise ValueError("runtime command profile_id must not be empty")
        values = (
            self.control_hz,
            self.linear_ramp_mm_per_tick,
            self.orientation_ramp_deg_per_tick,
            self.axis_velocity_limit_mm_s,
            self.ack_safe_axis_speed_envelope_mm_s,
            self.cartesian_velocity_limit_mm_s,
            self.acceleration_limit_mm_s2,
            self.jerk_limit_mm_s3,
            self.integrated_squared_jerk_limit,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("runtime command profile limits must be finite and positive")
        if self.bridge_ack_mode != "bounded_pipeline":
            raise ValueError("reviewed runtime profiles require bounded_pipeline")
        if self.max_ack_lag_steps != 1:
            raise ValueError("reviewed runtime profiles require one ACK lag step")
        if self.acknowledged_command_span_steps != 1 + self.max_ack_lag_steps:
            raise ValueError("acknowledged command span is inconsistent with ACK lag")
        expected_axis_limit = self.linear_ramp_mm_per_tick * self.control_hz
        if not math.isclose(
            self.axis_velocity_limit_mm_s,
            expected_axis_limit,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            raise ValueError("axis velocity limit is stale for the linear ramp")
        expected_ack_envelope = expected_axis_limit / self.acknowledged_command_span_steps
        if not math.isclose(
            self.ack_safe_axis_speed_envelope_mm_s,
            expected_ack_envelope,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            raise ValueError("ACK-safe axis-speed envelope is stale")

    def to_record(self) -> dict[str, Any]:
        return {
            "schema_version": "a0509.runtime_command_profile.v1",
            **asdict(self),
            "max_acknowledged_xyz_axis_span_mm": self.linear_ramp_mm_per_tick,
            "max_acknowledged_rotation_span_deg": (
                self.orientation_ramp_deg_per_tick
            ),
            "physical_validation_performed": False,
        }


_PROFILES = {
    LEGACY_RUNTIME_COMMAND_PROFILE_ID: RuntimeCommandProfile(
        profile_id=LEGACY_RUNTIME_COMMAND_PROFILE_ID,
        control_hz=30.0,
        linear_ramp_mm_per_tick=7.5,
        orientation_ramp_deg_per_tick=1.25,
        bridge_ack_mode="bounded_pipeline",
        max_ack_lag_steps=1,
        acknowledged_command_span_steps=2,
        axis_velocity_limit_mm_s=225.0,
        ack_safe_axis_speed_envelope_mm_s=112.5,
        cartesian_velocity_limit_mm_s=300.0,
        acceleration_limit_mm_s2=4000.0,
        jerk_limit_mm_s3=4000.0,
        integrated_squared_jerk_limit=10_000_000.0,
    ),
    RAMP8P5_RUNTIME_COMMAND_PROFILE_ID: RuntimeCommandProfile(
        profile_id=RAMP8P5_RUNTIME_COMMAND_PROFILE_ID,
        control_hz=30.0,
        linear_ramp_mm_per_tick=8.5,
        orientation_ramp_deg_per_tick=1.25,
        bridge_ack_mode="bounded_pipeline",
        max_ack_lag_steps=1,
        acknowledged_command_span_steps=2,
        axis_velocity_limit_mm_s=255.0,
        ack_safe_axis_speed_envelope_mm_s=127.5,
        cartesian_velocity_limit_mm_s=300.0,
        acceleration_limit_mm_s2=4000.0,
        jerk_limit_mm_s3=4000.0,
        integrated_squared_jerk_limit=10_000_000.0,
    ),
}

SUPPORTED_RUNTIME_COMMAND_PROFILE_IDS = frozenset(_PROFILES)


def get_runtime_command_profile(profile_id: str) -> RuntimeCommandProfile:
    try:
        return _PROFILES[str(profile_id)]
    except KeyError as exc:
        raise ValueError(
            f"unsupported runtime command profile: {profile_id!r}"
        ) from exc
