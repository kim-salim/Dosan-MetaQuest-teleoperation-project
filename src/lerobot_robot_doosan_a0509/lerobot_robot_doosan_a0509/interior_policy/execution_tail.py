"""Derive a zero-symbolic-cost source collar from real semantic support.

The symbolic planner still finishes an operator at its declared semantic
boundary.  For a FLEXIBLE_LEVEL2 policy switch, however, a reviewed ACT may
continue a short distance into its already learned post-boundary free-space
interval before the runtime invalidates its queue.  This module derives that
*execution tail* offline from the phase-support artifact.

No height threshold is used.  The collar is selected from normalized
arc-length phase, empirical Cartesian progress, and cross-episode support
dispersion.  It changes neither WorldState nor Dijkstra cost.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from lerobot_robot_doosan_a0509.task_c_handoff.stage_supervisor import (
    ZeroCostExecutionTail,
)

from .contracts import InteriorPolicyOperator


DERIVATION_METHOD = "empirical_progress_support_collar_latched_v2"
SHARED_EXECUTION_TAIL_PROFILE_ID = (
    "a0509_shared_execution_tail_latched_v2_20260904"
)
LEGACY_EXECUTION_TAIL_PROFILE_ID = "a0509_execution_tail_legacy_v11"
SUPPORTED_EXECUTION_TAIL_PROFILE_IDS = frozenset(
    {SHARED_EXECUTION_TAIL_PROFILE_ID, LEGACY_EXECUTION_TAIL_PROFILE_ID}
)


def _default_operator_overrides() -> dict[str, dict[str, float]]:
    """Keep the production derivation free of operator-specific tuning."""

    return {}


@dataclass(frozen=True)
class ExecutionTailDerivationConfig:
    """Bounded, reproducible collar-selection parameters."""

    profile_id: str = SHARED_EXECUTION_TAIL_PROFILE_ID
    enabled: bool = True
    max_relative_phase: float = 0.35
    min_relative_phase: float = 0.03
    progress_fraction_low: float = 0.25
    progress_fraction_high: float = 0.65
    support_residual_quantile: float = 0.75
    commit_fraction: float = 0.60
    deadline_extra_relative_phase: float = 0.04
    latch_commit_window_until_scan_deadline: bool = True
    latch_operator_profile_until_deadline: bool = False
    apply_same_segment_continuation_profile: bool = True
    min_candidate_points: int = 3
    operator_overrides: Mapping[str, Mapping[str, float]] = field(
        default_factory=_default_operator_overrides
    )
    reviewed_empty_gripper_free_space_operators: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        profile_id = str(self.profile_id).strip()
        if profile_id not in SUPPORTED_EXECUTION_TAIL_PROFILE_IDS:
            raise ValueError(
                f"unsupported execution-tail profile: {profile_id!r}"
            )
        object.__setattr__(self, "profile_id", profile_id)
        for name in (
            "max_relative_phase",
            "min_relative_phase",
            "progress_fraction_low",
            "progress_fraction_high",
            "support_residual_quantile",
            "commit_fraction",
            "deadline_extra_relative_phase",
        ):
            value = float(getattr(self, name))
            if not np.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be finite and in [0, 1]")
        if self.min_relative_phase >= self.max_relative_phase:
            raise ValueError("execution-tail relative phase range is empty")
        if self.progress_fraction_low >= self.progress_fraction_high:
            raise ValueError("execution-tail progress fraction range is empty")
        if not 0.0 < self.support_residual_quantile <= 1.0:
            raise ValueError("support_residual_quantile must be in (0, 1]")
        if not 0.0 <= self.commit_fraction <= 1.0:
            raise ValueError("commit_fraction must be in [0, 1]")
        for name in (
            "latch_commit_window_until_scan_deadline",
            "latch_operator_profile_until_deadline",
            "apply_same_segment_continuation_profile",
        ):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f"{name} must be bool")
        if (
            isinstance(self.min_candidate_points, bool)
            or self.min_candidate_points < 2
        ):
            raise ValueError("min_candidate_points must be at least two")
        allowed_override_fields = {
            "max_relative_phase",
            "min_relative_phase",
            "progress_fraction_low",
            "progress_fraction_high",
            "support_residual_quantile",
            "commit_fraction",
            "deadline_extra_relative_phase",
        }
        normalized: dict[str, dict[str, float]] = {}
        for operator_id, raw_values in self.operator_overrides.items():
            name = str(operator_id).strip()
            if not name:
                raise ValueError("execution-tail override operator id is empty")
            values = dict(raw_values)
            unknown = set(values) - allowed_override_fields
            if unknown:
                raise ValueError(
                    "unsupported execution-tail override fields for "
                    f"{name}: {','.join(sorted(unknown))}"
                )
            normalized[name] = {
                key: float(value) for key, value in values.items()
            }
        object.__setattr__(self, "operator_overrides", normalized)
        reviewed = tuple(
            str(operator_id).strip()
            for operator_id in self.reviewed_empty_gripper_free_space_operators
        )
        if any(not operator_id for operator_id in reviewed):
            raise ValueError(
                "reviewed empty-gripper execution-tail operator id is empty"
            )
        if len(reviewed) != len(set(reviewed)):
            raise ValueError(
                "reviewed empty-gripper execution-tail operators must be unique"
            )
        object.__setattr__(
            self,
            "reviewed_empty_gripper_free_space_operators",
            reviewed,
        )

    @classmethod
    def legacy_v11(
        cls,
        *,
        enabled: bool = True,
        reviewed_empty_gripper_free_space_operators: tuple[str, ...] = (),
    ) -> "ExecutionTailDerivationConfig":
        """Reproduce the historical v11 tail contract for rollback."""

        return cls(
            profile_id=LEGACY_EXECUTION_TAIL_PROFILE_ID,
            enabled=enabled,
            latch_commit_window_until_scan_deadline=False,
            latch_operator_profile_until_deadline=True,
            apply_same_segment_continuation_profile=False,
            operator_overrides={
                "T2.acquire_from_floor": {
                    "progress_fraction_low": 0.10,
                    "progress_fraction_high": 0.90,
                    "support_residual_quantile": 0.95,
                    "commit_fraction": 0.25,
                    "deadline_extra_relative_phase": 0.10,
                }
            },
            reviewed_empty_gripper_free_space_operators=(
                reviewed_empty_gripper_free_space_operators
            ),
        )

    def for_operator(self, operator_id: str) -> "ExecutionTailDerivationConfig":
        """Apply one source-operator profile without changing global defaults."""

        values = self.operator_overrides.get(str(operator_id))
        if not values:
            return self
        # Clear overrides on the effective copy so this method is idempotent.
        return replace(self, operator_overrides={}, **dict(values))

    def for_continuation_class(
        self, *, same_segment: bool
    ) -> "ExecutionTailDerivationConfig":
        """Use one generic profile for an in-segment semantic continuation."""

        if (
            not same_segment
            or not self.apply_same_segment_continuation_profile
        ):
            return self
        return replace(
            self,
            progress_fraction_low=0.10,
            progress_fraction_high=0.90,
            support_residual_quantile=0.95,
            commit_fraction=0.25,
            deadline_extra_relative_phase=0.10,
        )


@dataclass(frozen=True)
class ExecutionTailDerivation:
    profile: ZeroCostExecutionTail | None
    eligible: bool
    reason: str

    def record(self) -> dict[str, Any]:
        return {
            "eligible": self.eligible,
            "reason": self.reason,
            "profile": None if self.profile is None else self.profile.to_record(),
        }


def _longest_contiguous_run(indices: np.ndarray) -> np.ndarray:
    values = np.asarray(indices, dtype=np.int64)
    if values.size == 0:
        return values
    split_points = np.flatnonzero(np.diff(values) != 1) + 1
    runs = np.split(values, split_points)
    # Deterministic tie-break: prefer the earlier supported collar.
    return max(runs, key=lambda run: (run.size, -int(run[0])))


def _continuation_segment(
    operator: InteriorPolicyOperator,
    graph: Mapping[str, Any],
) -> tuple[str, float, bool] | None:
    segments = [dict(item["spec"]) for item in graph["segments"]]
    ids = [str(item["segment_id"]) for item in segments]
    source_segment = operator.evidence.exit_segment
    source_phase = float(operator.evidence.exit_phase)
    if source_phase < 1.0 - 1.0e-12:
        return source_segment, source_phase, True
    try:
        index = ids.index(source_segment)
    except ValueError as exc:
        raise ValueError(
            f"operator {operator.id} exit segment is absent from semantic graph"
        ) from exc
    if index + 1 >= len(ids):
        return None
    return ids[index + 1], 0.0, False


def _execution_tail_eligible(
    operator: InteriorPolicyOperator,
    config: ExecutionTailDerivationConfig,
) -> tuple[bool, str]:
    effects = operator.effect_map
    if operator.bridge_mode == "held_object_free_transport":
        if operator.exit_contact_mode != "free_transport":
            return False, "source_exit_contact_is_not_free_transport"
        if effects.get("gripper") != "closed":
            return False, "source_effect_gripper_is_not_closed"
        if effects.get("holding") in {None, "none", "unknown"}:
            return False, "source_effect_has_no_known_held_object"
        return True, "eligible_held_object_free_transport"

    if (
        operator.id
        not in config.reviewed_empty_gripper_free_space_operators
    ):
        return False, "source_bridge_mode_is_not_held_object_free_transport"
    if operator.bridge_mode != "empty_gripper_free_space":
        return False, "reviewed_source_bridge_mode_is_not_empty_gripper_free_space"
    if operator.exit_contact_mode != "free_space":
        return False, "reviewed_source_exit_contact_is_not_free_space"
    if effects.get("gripper") != "open":
        return False, "reviewed_source_effect_gripper_is_not_open"
    if effects.get("holding") != "none":
        return False, "reviewed_source_effect_holding_is_not_none"
    return True, "eligible_reviewed_empty_gripper_free_space"


def derive_zero_cost_execution_tail(
    operator: InteriorPolicyOperator,
    *,
    config: ExecutionTailDerivationConfig | None = None,
) -> ExecutionTailDerivation:
    """Return a data-derived collar for one source operator, if eligible.

    Held-object/free-transport exits are enabled by default. Empty-gripper
    free-space exits require an explicit per-operator review allowlist so an
    ordinary open/contact operator cannot silently borrow its successor motion.
    """

    base_cfg = config or ExecutionTailDerivationConfig()
    if not base_cfg.enabled:
        return ExecutionTailDerivation(None, False, "execution_tail_disabled")
    eligible, reason = _execution_tail_eligible(operator, base_cfg)
    if not eligible:
        return ExecutionTailDerivation(None, False, reason)

    graph_path = Path(operator.evidence.semantic_artifact).expanduser().resolve()
    support_path = Path(
        operator.evidence.phase_support_artifact
    ).expanduser().resolve()
    graph = json.loads(graph_path.read_text(encoding="utf-8"))
    continuation = _continuation_segment(operator, graph)
    if continuation is None:
        return ExecutionTailDerivation(None, True, "no_forward_semantic_support")
    segment, tracking_start_phase, same_segment = continuation
    class_cfg = base_cfg.for_continuation_class(
        same_segment=bool(same_segment)
    )
    cfg = class_cfg.for_operator(operator.id)
    operator_profile_applied = cfg is not class_cfg

    segment_spec = next(
        dict(item["spec"])
        for item in graph["segments"]
        if str(item["spec"]["segment_id"]) == segment
    )
    entry_state = dict(segment_spec.get("entry_state", {}))
    expected_gripper = operator.effect_map.get("gripper")
    expected_holding = operator.effect_map.get("holding")
    aliases = dict(operator.evidence.held_object_aliases)
    artifact_holding = str(entry_state.get("held_object", "unknown"))
    canonical_holding = aliases.get(artifact_holding, artifact_holding)
    if entry_state.get("gripper") != expected_gripper:
        return ExecutionTailDerivation(
            None, True, "continuation_entry_gripper_mismatch"
        )
    if canonical_holding != expected_holding:
        return ExecutionTailDerivation(
            None, True, "continuation_entry_held_object_mismatch"
        )
    for persistent_key in ("drawer", "white_container"):
        expected_value = operator.effect_map.get(persistent_key)
        if (
            expected_value is not None
            and entry_state.get(persistent_key) != expected_value
        ):
            return ExecutionTailDerivation(
                None,
                True,
                f"continuation_entry_{persistent_key}_mismatch",
            )

    prefix = f"{segment}_"
    with np.load(support_path, allow_pickle=False) as archive:
        required = {
            "phase": prefix + "phase",
            "episode_ids": prefix + "episode_ids",
            "episode_xyz_mm": prefix + "episode_xyz_mm",
            "median_xyz_mm": prefix + "median_xyz_mm",
        }
        missing = [key for key in required.values() if key not in archive.files]
        if missing:
            raise ValueError(
                f"execution-tail support lacks {segment}: {','.join(missing)}"
            )
        phase = np.asarray(archive[required["phase"]], dtype=np.float64)
        episode_ids = np.asarray(
            archive[required["episode_ids"]], dtype=np.int64
        )
        episode_xyz = np.asarray(
            archive[required["episode_xyz_mm"]], dtype=np.float64
        )
        median_xyz = np.asarray(
            archive[required["median_xyz_mm"]], dtype=np.float64
        )

    remaining = max(1.0 - tracking_start_phase, 1.0e-12)
    relative_phase = (phase - tracking_start_phase) / remaining
    scan = (
        (relative_phase >= -1.0e-12)
        & (relative_phase <= cfg.max_relative_phase + 1.0e-12)
    )
    if np.count_nonzero(scan) < cfg.min_candidate_points:
        return ExecutionTailDerivation(None, True, "insufficient_early_phase_support")

    start_index = int(np.argmin(np.abs(phase - tracking_start_phase)))
    progress_mm = np.linalg.norm(median_xyz - median_xyz[start_index], axis=1)
    max_early_progress = float(np.max(progress_mm[scan]))
    if not np.isfinite(max_early_progress) or max_early_progress <= 1.0e-9:
        return ExecutionTailDerivation(None, True, "zero_early_cartesian_progress")
    progress_fraction = progress_mm / max_early_progress

    residual_mm = np.linalg.norm(
        episode_xyz - median_xyz[None, :, :], axis=2
    )
    residual_p90_mm = np.percentile(residual_mm, 90.0, axis=0)
    residual_limit_mm = float(
        np.quantile(residual_p90_mm[scan], cfg.support_residual_quantile)
    )
    candidate_mask = (
        scan
        & (relative_phase >= cfg.min_relative_phase - 1.0e-12)
        & (progress_fraction >= cfg.progress_fraction_low - 1.0e-12)
        & (progress_fraction <= cfg.progress_fraction_high + 1.0e-12)
        & (residual_p90_mm <= residual_limit_mm + 1.0e-12)
    )
    run = _longest_contiguous_run(np.flatnonzero(candidate_mask))
    if run.size < cfg.min_candidate_points:
        return ExecutionTailDerivation(
            None, True, "no_supported_contiguous_execution_collar"
        )

    split_offset = int(np.floor(cfg.commit_fraction * (run.size - 1)))
    commit_run = run[split_offset:]
    commit_low = float(phase[int(commit_run[0])])
    commit_high = float(phase[int(commit_run[-1])])
    nominal_phase = 0.5 * (commit_low + commit_high)
    phase_half_width = 0.5 * (commit_high - commit_low)
    if phase_half_width <= 0.0:
        return ExecutionTailDerivation(None, True, "zero_width_commit_collar")
    ordinary_deadline = min(
        1.0,
        commit_high + cfg.deadline_extra_relative_phase * remaining,
    )
    scan_deadline = min(
        1.0,
        tracking_start_phase + cfg.max_relative_phase * remaining,
    )
    deadline_phase = (
        max(ordinary_deadline, scan_deadline)
        if cfg.latch_commit_window_until_scan_deadline
        else ordinary_deadline
    )
    latch_until_deadline = bool(
        cfg.latch_commit_window_until_scan_deadline
        or (cfg.latch_operator_profile_until_deadline and operator_profile_applied)
    )
    profile = ZeroCostExecutionTail(
        source_operator=operator.id,
        semantic_exit_segment=operator.evidence.exit_segment,
        semantic_exit_phase=float(operator.evidence.exit_phase),
        tracking_segment=segment,
        tracking_start_phase=float(tracking_start_phase),
        collar_phase_low=float(phase[int(run[0])]),
        collar_phase_high=float(phase[int(run[-1])]),
        commit_phase_low=commit_low,
        commit_phase_high=commit_high,
        nominal_phase=nominal_phase,
        deadline_phase=deadline_phase,
        phase_half_width=phase_half_width,
        prearm_extra_phase=max(0.0, commit_low - tracking_start_phase),
        phase_points=int(phase.size),
        episode_count=int(episode_ids.size),
        derivation_method=(
            DERIVATION_METHOD
            + f":profile:{base_cfg.profile_id}"
            + (
                ":same_segment_continuation"
                if same_segment
                and base_cfg.apply_same_segment_continuation_profile
                else ""
            )
            + (f":operator_profile:{operator.id}" if operator_profile_applied else "")
        ),
        same_segment_as_semantic_exit=bool(same_segment),
        collar_progress_mm_low=float(progress_mm[int(run[0])]),
        collar_progress_mm_high=float(progress_mm[int(run[-1])]),
        support_residual_p90_limit_mm=residual_limit_mm,
        zero_symbolic_cost=True,
        fixed_z_minimum_used=False,
        latch_commit_window_until_deadline=latch_until_deadline,
    )
    return ExecutionTailDerivation(profile, True, "derived")
