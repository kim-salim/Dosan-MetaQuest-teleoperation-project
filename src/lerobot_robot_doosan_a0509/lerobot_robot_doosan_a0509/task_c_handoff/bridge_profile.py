"""Versioned, edge-independent Bridge generation profiles.

The profile is an offline build contract.  Loading or applying one performs no
ROS I/O and cannot enable Live or select the command MUX.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

from offline_tools.task_c_bridge_v0.bezier_bridge import (
    CUBIC_BEZIER_FIXED,
    CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
)


PROFILE_SCHEMA_VERSION = "a0509.flexible_bridge_generation_profiles.v1"
PROFILE_PROVENANCE_SCHEMA_VERSION = (
    "a0509.flexible_bridge_generation_profile_resolution.v1"
)
SUPPORTED_SOURCE_REFERENCE_MODES = {
    "exact_semantic_exit",
    "execution_tail",
}


def _finite_positive(value: object, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


@dataclass(frozen=True)
class ResolvedBridgeGenerationProfile:
    """One validated profile plus immutable provenance."""

    profile_id: str
    profile_sha256: str
    profile_config_path: Path
    description: str
    source_reference_modes: tuple[str, ...]
    edge_specific_tuning: bool
    bridge_duration_s: float
    transport_floor_policy: str
    bridge_admission_mode: str
    bridge_algorithm: str
    minimum_tangent_handle_chord_ratio: float
    maximum_endpoint_speed_adjustment_mm_s: float | None
    runtime_contract: Mapping[str, float | int]

    def validate_source_reference_mode(self, mode: str) -> None:
        if mode not in self.source_reference_modes:
            raise ValueError(
                f"profile {self.profile_id} does not apply to source "
                f"reference mode {mode!r}"
            )

    def provenance_record(self) -> dict[str, Any]:
        return {
            "schema_version": PROFILE_PROVENANCE_SCHEMA_VERSION,
            "profile_id": self.profile_id,
            "profile_sha256": self.profile_sha256,
            "profile_config": str(self.profile_config_path),
            "edge_specific_tuning": self.edge_specific_tuning,
            "source_reference_modes": list(self.source_reference_modes),
            "bridge_duration_s": self.bridge_duration_s,
            "transport_floor_policy": self.transport_floor_policy,
            "bridge_admission_mode": self.bridge_admission_mode,
            "bridge_algorithm": self.bridge_algorithm,
            "minimum_tangent_handle_chord_ratio": (
                self.minimum_tangent_handle_chord_ratio
            ),
            "maximum_endpoint_speed_adjustment_mm_s": (
                self.maximum_endpoint_speed_adjustment_mm_s
            ),
            "runtime_contract": dict(self.runtime_contract),
            "robot_commands_published": 0,
            "physical_validation_performed": False,
        }


def load_bridge_generation_profile(
    path: str | Path,
    *,
    source_reference_mode: str,
    profile_id: str | None = None,
) -> ResolvedBridgeGenerationProfile:
    """Load the shared profile selected by source-reference class."""

    if source_reference_mode not in SUPPORTED_SOURCE_REFERENCE_MODES:
        raise ValueError(
            f"unsupported source reference mode: {source_reference_mode}"
        )
    config_path = Path(path).expanduser().resolve()
    value = json.loads(config_path.read_text(encoding="utf-8"))
    if value.get("schema_version") != PROFILE_SCHEMA_VERSION:
        raise ValueError("unsupported Bridge generation profile schema")
    selected_id = profile_id
    if selected_id is None:
        selected_id = str(
            value["default_profile_by_source_reference_mode"][
                source_reference_mode
            ]
        )
    raw_profile = dict(value["profiles"][selected_id])
    canonical = json.dumps(
        {
            "schema_version": PROFILE_SCHEMA_VERSION,
            "profile_id": selected_id,
            "profile": raw_profile,
        },
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    digest = hashlib.sha256(canonical).hexdigest()

    applicability = dict(raw_profile["applicability"])
    modes = tuple(str(item) for item in applicability["source_reference_modes"])
    if not modes or any(
        item not in SUPPORTED_SOURCE_REFERENCE_MODES for item in modes
    ):
        raise ValueError("profile has invalid source reference modes")
    edge_specific_tuning = bool(applicability["edge_specific_tuning"])
    if edge_specific_tuning:
        raise ValueError("shared Bridge profile cannot allow edge-specific tuning")

    bridge = dict(raw_profile["bridge"])
    generation = dict(raw_profile["generation"])
    algorithm = str(generation["bridge_algorithm"])
    ratio = float(generation["minimum_tangent_handle_chord_ratio"])
    raw_adjustment = generation["maximum_endpoint_speed_adjustment_mm_s"]
    adjustment = (
        None
        if raw_adjustment is None
        else _finite_positive(raw_adjustment, "endpoint speed adjustment")
    )
    if algorithm == CUBIC_BEZIER_FIXED:
        if ratio != 0.0 or adjustment is not None:
            raise ValueError("fixed Bridge profile cannot regularize tangents")
    elif algorithm == CUBIC_BEZIER_TANGENT_REGULARIZED_V1:
        if not 0.0 < ratio <= 0.25 or adjustment is None:
            raise ValueError("invalid tangent-regularized Bridge profile")
    else:
        raise ValueError(f"unsupported Bridge algorithm: {algorithm}")

    runtime_contract = {
        str(key): value for key, value in dict(raw_profile["runtime_contract"]).items()
    }
    resolved = ResolvedBridgeGenerationProfile(
        profile_id=str(selected_id),
        profile_sha256=digest,
        profile_config_path=config_path,
        description=str(raw_profile["description"]),
        source_reference_modes=modes,
        edge_specific_tuning=edge_specific_tuning,
        bridge_duration_s=_finite_positive(
            bridge["duration_s"], "bridge duration"
        ),
        transport_floor_policy=str(bridge["transport_floor_policy"]),
        bridge_admission_mode=str(generation["bridge_admission_mode"]),
        bridge_algorithm=algorithm,
        minimum_tangent_handle_chord_ratio=ratio,
        maximum_endpoint_speed_adjustment_mm_s=adjustment,
        runtime_contract=runtime_contract,
    )
    resolved.validate_source_reference_mode(source_reference_mode)
    _validate_derived_speed_contract(resolved)
    return resolved


def _validate_derived_speed_contract(
    profile: ResolvedBridgeGenerationProfile,
) -> None:
    contract = profile.runtime_contract
    hz = _finite_positive(contract["control_hz"], "control_hz")
    ramp = _finite_positive(
        contract["linear_ramp_mm_per_tick"],
        "linear_ramp_mm_per_tick",
    )
    span = int(contract["acknowledged_command_span_steps"])
    if span < 1:
        raise ValueError("acknowledged command span must be positive")
    expected_unadjusted = ramp * hz / span
    declared_unadjusted = float(
        contract["derived_unadjusted_axis_speed_ceiling_mm_s"]
    )
    if not math.isclose(
        expected_unadjusted, declared_unadjusted, abs_tol=1.0e-9
    ):
        raise ValueError("derived unadjusted axis-speed ceiling is stale")
    # Tangent regularization raises only a degenerate low-speed handle. Its
    # allowance must not be interpreted as a higher ACK-span speed ceiling.
    expected_flex = expected_unadjusted
    declared_flex = float(contract["derived_flex_axis_speed_envelope_mm_s"])
    if not math.isclose(expected_flex, declared_flex, abs_tol=1.0e-9):
        raise ValueError("derived flex axis-speed envelope is stale")


def validate_profile_against_validation_config(
    profile: ResolvedBridgeGenerationProfile,
    validation_config: Mapping[str, Any],
) -> None:
    """Fail if the profile was derived under a different safety contract."""

    dynamics = dict(validation_config["dynamics"])
    downstream = dict(validation_config["downstream"])
    expected = {
        "control_hz": downstream["control_hz"],
        "linear_ramp_mm_per_tick": downstream["linear_ramp_mm_per_tick"],
        "orientation_ramp_deg_per_tick": downstream[
            "orientation_ramp_deg_per_tick"
        ],
        "velocity_limit_mm_s": dynamics["velocity_limit_mm_s"],
        "axis_velocity_limit_mm_s": dynamics["axis_velocity_limit_mm_s"],
        "acceleration_limit_mm_s2": dynamics["acceleration_limit_mm_s2"],
        "curvature_limit_per_mm": dynamics["curvature_limit_per_mm"],
        "jerk_limit_mm_s3": dynamics["jerk_limit_mm_s3"],
        "integrated_squared_jerk_limit": dynamics[
            "integrated_squared_jerk_limit"
        ],
    }
    for name, expected_value in expected.items():
        actual = float(profile.runtime_contract[name])
        if not math.isclose(
            actual, float(expected_value), abs_tol=1.0e-9, rel_tol=0.0
        ):
            raise ValueError(
                f"Bridge profile runtime contract mismatch for {name}: "
                f"profile={actual} validation={expected_value}"
            )


def apply_bridge_generation_profile(
    manifest_record: Mapping[str, Any],
    profile: ResolvedBridgeGenerationProfile,
    *,
    source_reference_mode: str,
) -> dict[str, Any]:
    """Return a profiled manifest record without mutating the input."""

    profile.validate_source_reference_mode(source_reference_mode)
    value = deepcopy(dict(manifest_record))
    bridge = dict(value["bridge"])
    bridge["duration_s"] = profile.bridge_duration_s
    value["bridge"] = bridge
    generation = dict(value.get("generation", {}))
    generation.update(
        {
            "method": "flexible_reference",
            "bridge_admission_mode": profile.bridge_admission_mode,
            "reference_only": True,
            "bridge_algorithm": profile.bridge_algorithm,
            "tangent_regularization": {
                "minimum_handle_chord_ratio": (
                    profile.minimum_tangent_handle_chord_ratio
                ),
                "maximum_endpoint_speed_adjustment_mm_s": (
                    profile.maximum_endpoint_speed_adjustment_mm_s
                ),
            },
            "bridge_profile": profile.provenance_record(),
        }
    )
    value["generation"] = generation
    return value


def attach_bridge_profile_provenance(
    manifest_record: Mapping[str, Any],
    profile: ResolvedBridgeGenerationProfile,
) -> dict[str, Any]:
    """Restore profile metadata after a model round trip."""

    value = deepcopy(dict(manifest_record))
    generation = dict(value["generation"])
    generation["bridge_profile"] = profile.provenance_record()
    value["generation"] = generation
    return value


def evaluate_source_bank_speed_coverage(
    source_bank: Mapping[str, Any],
    profile: ResolvedBridgeGenerationProfile,
) -> dict[str, Any]:
    """Compute analytical source-speed coverage; this is not a success claim."""

    raw_limit = float(
        profile.runtime_contract[
            "derived_unadjusted_axis_speed_ceiling_mm_s"
        ]
    )
    profile_limit = float(
        profile.runtime_contract["derived_flex_axis_speed_envelope_mm_s"]
    )
    nominal_phase = float(source_bank["nominal_phase"])
    episodes = list(source_bank["episodes"])
    by_phase: dict[float, list[float]] = {}
    episode_samples: list[list[tuple[float, float]]] = []
    for episode in episodes:
        samples: list[tuple[float, float]] = []
        for sample in episode["samples"]:
            phase = round(float(sample["phase"]), 9)
            speed = max(abs(float(item)) for item in sample["velocity_mm_s"])
            samples.append((phase, speed))
            by_phase.setdefault(phase, []).append(speed)
        if not samples:
            raise ValueError("source-bank episode has no samples")
        episode_samples.append(samples)

    def coverage(*, phase_high: float | None) -> dict[str, Any]:
        best = [
            min(
                speed
                for phase, speed in samples
                if phase_high is None or phase <= phase_high + 1.0e-9
            )
            for samples in episode_samples
        ]
        raw_count = sum(speed <= raw_limit for speed in best)
        profile_count = sum(speed <= profile_limit for speed in best)
        total = len(best)
        return {
            "episode_count": total,
            "unadjusted_episode_count": raw_count,
            "unadjusted_fraction": raw_count / total,
            "profile_admissible_episode_count": profile_count,
            "profile_admissible_fraction": profile_count / total,
            "worst_episode_best_axis_speed_mm_s": max(best),
        }

    return {
        "schema_version": "a0509.bridge_profile_source_speed_coverage.v1",
        "profile": profile.provenance_record(),
        "source_operator": source_bank.get("source_operator"),
        "source_segment": source_bank.get("segment"),
        "source_phase_window": list(source_bank["phase_window"]),
        "source_nominal_phase": nominal_phase,
        "analytical_limits": {
            "unadjusted_axis_speed_ceiling_mm_s": raw_limit,
            "profile_axis_speed_gate_mm_s": profile_limit,
            "endpoint_speed_adjustment_semantics": (
                "minimum_handle_low_speed_increase_only"
            ),
            "interpretation": (
                "high-speed gate only; geometry, dynamics, worker "
                "latency, and physical task success remain independently gated"
            ),
        },
        "nominal_phase_prefix": coverage(phase_high=nominal_phase),
        "full_source_phase_window": coverage(phase_high=None),
        "per_phase": [
            {
                "phase": phase,
                "sample_count": len(speeds),
                "unadjusted_count": sum(
                    speed <= raw_limit for speed in speeds
                ),
                "profile_admissible_count": sum(
                    speed <= profile_limit for speed in speeds
                ),
            }
            for phase, speeds in sorted(by_phase.items())
        ],
        "robot_commands_published": 0,
        "physical_validation_performed": False,
    }
