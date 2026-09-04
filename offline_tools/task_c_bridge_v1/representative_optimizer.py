"""Coarse-to-fine shortest-path optimization over representative A/B paths."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Callable, Iterable

import numpy as np

from offline_tools.task_c_bridge_v0.bezier_bridge import (
    CubicBezierBridge,
    build_velocity_matched_bezier,
)
from offline_tools.task_c_bridge_v0.bridge_metrics import (
    BridgeMetrics,
    evaluate_bridge,
    feasibility_reasons,
)
from offline_tools.task_c_bridge_v1.representative_trajectory import (
    RepresentativeTrajectory,
)


@dataclass(frozen=True)
class RepresentativeBridgeCandidate:
    a_phase: float
    b_phase: float
    duration_s: float
    bridge: CubicBezierBridge
    metrics: BridgeMetrics
    a_retained_length_mm: float
    b_retained_length_mm: float
    total_c_length_mm: float
    start_acceleration_jump_mm_s2: float
    end_acceleration_jump_mm_s2: float
    sampling_hz: float
    feasible: bool
    failure_reasons: tuple[str, ...]
    search_stage: str
    robust_pass_fraction: float | None = None
    robust_checks: int = 0
    a_prearm_support_fraction: float = 0.0
    a_commit_support_fraction: float = 0.0

    @property
    def smoothness_key(self) -> tuple[float, float, float, float, float]:
        return (
            self.start_acceleration_jump_mm_s2,
            self.end_acceleration_jump_mm_s2,
            self.metrics.max_acceleration_mm_s2,
            self.metrics.max_curvature_per_mm,
            self.metrics.integrated_squared_jerk,
        )

    def to_record(self) -> dict[str, Any]:
        return {
            "a_phase": self.a_phase,
            "b_phase": self.b_phase,
            "duration_s": self.duration_s,
            "control_points_mm": {
                "p0": self.bridge.p0.tolist(),
                "p1": self.bridge.p1.tolist(),
                "p2": self.bridge.p2.tolist(),
                "p3": self.bridge.p3.tolist(),
            },
            "a_retained_length_mm": self.a_retained_length_mm,
            "bridge_length_mm": self.metrics.length_mm,
            "b_retained_length_mm": self.b_retained_length_mm,
            "total_c_length_mm": self.total_c_length_mm,
            "max_velocity_mm_s": self.metrics.max_velocity_mm_s,
            "max_acceleration_mm_s2": self.metrics.max_acceleration_mm_s2,
            "max_curvature_per_mm": self.metrics.max_curvature_per_mm,
            "max_jerk_mm_s3": self.metrics.max_jerk_mm_s3,
            "integrated_squared_jerk": self.metrics.integrated_squared_jerk,
            "backtracking_ratio": self.metrics.backtracking_ratio,
            "workspace_satisfied": self.metrics.workspace_satisfied,
            "start_acceleration_jump_mm_s2": (
                self.start_acceleration_jump_mm_s2
            ),
            "end_acceleration_jump_mm_s2": self.end_acceleration_jump_mm_s2,
            "sampling_hz": self.sampling_hz,
            "feasible": self.feasible,
            "failure_reasons": list(self.failure_reasons),
            "search_stage": self.search_stage,
            "robust_pass_fraction": self.robust_pass_fraction,
            "robust_checks": self.robust_checks,
            "a_prearm_support_fraction": self.a_prearm_support_fraction,
            "a_commit_support_fraction": self.a_commit_support_fraction,
        }


@dataclass(frozen=True)
class RepresentativeOptimizationResult:
    selected: RepresentativeBridgeCandidate
    top_k: tuple[RepresentativeBridgeCandidate, ...]
    coarse_evaluated: int
    refined_evaluated: int
    final_validated: int
    robust_validated: int
    feasible_candidates: int
    failures_by_reason: dict[str, int]


def _grid(step: float) -> np.ndarray:
    if not np.isfinite(step) or not 0.0 < step <= 1.0:
        raise ValueError("phase step must be in (0, 1]")
    count = int(np.floor(1.0 / step + 1e-9))
    values = np.arange(count + 1, dtype=np.float64) * step
    if values[-1] < 1.0 - 1e-9:
        values = np.append(values, 1.0)
    values[-1] = 1.0
    return values


def _duration_grid(config: dict[str, Any]) -> np.ndarray:
    start = float(config["bridge_duration_min_s"])
    stop = float(config["bridge_duration_max_s"])
    step = float(config["bridge_duration_step_s"])
    if start <= 0.0 or stop < start or step <= 0.0:
        raise ValueError("invalid representative duration search")
    count = int(np.floor((stop - start) / step + 1e-9)) + 1
    return start + np.arange(count, dtype=np.float64) * step


def _select(
    candidates: Iterable[RepresentativeBridgeCandidate],
    near_shortest_delta: float,
) -> RepresentativeBridgeCandidate:
    feasible = [candidate for candidate in candidates if candidate.feasible]
    if not feasible:
        raise ValueError("no feasible representative Bridge candidate")
    shortest = min(candidate.total_c_length_mm for candidate in feasible)
    near = [
        candidate
        for candidate in feasible
        if candidate.total_c_length_mm
        <= shortest * (1.0 + near_shortest_delta)
    ]
    return min(
        near,
        key=lambda candidate: (
            candidate.smoothness_key,
            candidate.total_c_length_mm,
            candidate.duration_s,
        ),
    )


class RepresentativeBridgeOptimizer:
    """Optimize one A/B representative pair without discarding source spread."""

    def __init__(
        self,
        *,
        representative_a: RepresentativeTrajectory,
        representative_b: RepresentativeTrajectory,
        config: dict[str, Any],
        workspace_min_mm: np.ndarray,
        workspace_max_mm: np.ndarray,
    ) -> None:
        if representative_a.role != "a_exit":
            raise ValueError("representative A must have a_exit role")
        if representative_b.role != "b_entry":
            raise ValueError("representative B must have b_entry role")
        self.a = representative_a
        self.b = representative_b
        self.config = config
        self.workspace_min_mm = np.asarray(workspace_min_mm, dtype=np.float64)
        self.workspace_max_mm = np.asarray(workspace_max_mm, dtype=np.float64)
        self._a_support_cache: dict[float, tuple[float, float]] = {}
        if (
            self.workspace_min_mm.shape != (3,)
            or self.workspace_max_mm.shape != (3,)
            or np.any(self.workspace_min_mm >= self.workspace_max_mm)
        ):
            raise ValueError("representative workspace bounds are invalid")

    def _a_boundary_support(self, phase: float) -> tuple[float, float]:
        key = round(float(phase), 10)
        cached = self._a_support_cache.get(key)
        if cached is not None:
            return cached
        positions, _, _ = self.a.episode_values(phase)
        distances = np.linalg.norm(positions - self.a.position(phase), axis=1)
        boundary = self.config["live_boundary"]
        result = (
            float(
                np.mean(
                    distances <= float(boundary["prearm_radius_mm"])
                )
            ),
            float(
                np.mean(
                    distances <= float(boundary["commit_radius_mm"])
                )
            ),
        )
        self._a_support_cache[key] = result
        return result

    @property
    def terminal_stop(self) -> bool:
        return str(self.config["search"].get("initial_terminal_velocity", "zero")) == "zero"

    def _terminal_velocity(self, b_phase: float) -> np.ndarray:
        if self.terminal_stop:
            return np.zeros(3, dtype=np.float64)
        return self.b.velocity(b_phase)

    def _evaluate(
        self,
        *,
        a_phase: float,
        b_phase: float,
        duration_s: float,
        sample_hz: float,
        stage: str,
        position_a_mm: np.ndarray | None = None,
        velocity_a_mm_s: np.ndarray | None = None,
        acceleration_a_mm_s2: np.ndarray | None = None,
        position_b_mm: np.ndarray | None = None,
        velocity_b_mm_s: np.ndarray | None = None,
        acceleration_b_mm_s2: np.ndarray | None = None,
    ) -> RepresentativeBridgeCandidate:
        p_a = (
            self.a.position(a_phase)
            if position_a_mm is None
            else np.asarray(position_a_mm, dtype=np.float64)
        )
        v_a = (
            self.a.velocity(a_phase)
            if velocity_a_mm_s is None
            else np.asarray(velocity_a_mm_s, dtype=np.float64)
        )
        a_a = (
            self.a.acceleration(a_phase)
            if acceleration_a_mm_s2 is None
            else np.asarray(acceleration_a_mm_s2, dtype=np.float64)
        )
        p_b = (
            self.b.position(b_phase)
            if position_b_mm is None
            else np.asarray(position_b_mm, dtype=np.float64)
        )
        default_v_b = self._terminal_velocity(b_phase)
        v_b = (
            default_v_b
            if velocity_b_mm_s is None
            else np.asarray(velocity_b_mm_s, dtype=np.float64)
        )
        if self.terminal_stop:
            v_b = np.zeros(3, dtype=np.float64)
        a_b = (
            self.b.acceleration(b_phase)
            if acceleration_b_mm_s2 is None
            else np.asarray(acceleration_b_mm_s2, dtype=np.float64)
        )
        if self.terminal_stop:
            a_b = np.zeros(3, dtype=np.float64)
        bridge = build_velocity_matched_bezier(
            p_a,
            v_a,
            p_b,
            v_b,
            float(duration_s),
        )
        sampling = self.config["sampling"]
        metrics, sample = evaluate_bridge(
            bridge,
            sample_hz=float(sample_hz),
            curvature_epsilon=float(sampling["curvature_epsilon"]),
            workspace_min_mm=self.workspace_min_mm,
            workspace_max_mm=self.workspace_max_mm,
        )
        failures = feasibility_reasons(metrics, self.config["feasibility"])
        axis_limit = self.config["feasibility"].get("axis_velocity_limit_mm_s")
        if axis_limit is not None and float(
            np.max(np.abs(sample["velocity_mm_s"]))
        ) > float(axis_limit):
            failures.append("axis_velocity_limit")
        required_floor = max(
            self.a.minimum_bridge_z_mm,
            self.b.minimum_bridge_z_mm,
        )
        floor_tolerance = float(
            self.config["feasibility"].get(
                "transport_floor_tolerance_mm", 1e-6
            )
        )
        if (
            self.config["feasibility"].get(
                "enforce_payload_transport_floor", True
            )
            and float(np.min(sample["position_mm"][:, 2]))
            < required_floor - floor_tolerance
        ):
            failures.append("payload_clearance_violation")
        start_acceleration_jump = float(
            np.linalg.norm(bridge.acceleration(0.0) - a_a)
        )
        end_acceleration_jump = float(
            np.linalg.norm(bridge.acceleration(1.0) - a_b)
        )
        jump_limit = self.config["feasibility"].get(
            "boundary_acceleration_jump_limit_mm_s2"
        )
        if jump_limit is not None:
            if start_acceleration_jump > float(jump_limit):
                failures.append("a_boundary_acceleration_jump")
            if end_acceleration_jump > float(jump_limit):
                failures.append("b_boundary_acceleration_jump")
        prearm_support, commit_support = self._a_boundary_support(a_phase)
        live_boundary = self.config["live_boundary"]
        if prearm_support < float(
            live_boundary["minimum_prearm_support_fraction"]
        ):
            failures.append("a_boundary_prearm_support")
        if commit_support < float(
            live_boundary["minimum_commit_support_fraction"]
        ):
            failures.append("a_boundary_commit_support")
        a_retained = self.a.prefix_length(a_phase)
        b_retained = self.b.suffix_length(b_phase)
        return RepresentativeBridgeCandidate(
            a_phase=float(a_phase),
            b_phase=float(b_phase),
            duration_s=float(duration_s),
            bridge=bridge,
            metrics=metrics,
            a_retained_length_mm=a_retained,
            b_retained_length_mm=b_retained,
            total_c_length_mm=a_retained + metrics.length_mm + b_retained,
            start_acceleration_jump_mm_s2=start_acceleration_jump,
            end_acceleration_jump_mm_s2=end_acceleration_jump,
            sampling_hz=float(sample_hz),
            feasible=not failures,
            failure_reasons=tuple(dict.fromkeys(failures)),
            search_stage=stage,
            a_prearm_support_fraction=prearm_support,
            a_commit_support_fraction=commit_support,
        )

    def _phase_refinement(self, center: float) -> np.ndarray:
        search = self.config["search"]
        half_width = float(search["refine_half_width"])
        step = float(search["refine_phase_step"])
        start = max(0.0, center - half_width)
        stop = min(1.0, center + half_width)
        count = int(np.floor((stop - start) / step + 1e-9)) + 1
        values = start + np.arange(count, dtype=np.float64) * step
        if values[-1] < stop - 1e-9:
            values = np.append(values, stop)
        return np.unique(np.round(values, 10))

    def _robust_fraction(
        self,
        candidate: RepresentativeBridgeCandidate,
        sample_hz: float,
    ) -> tuple[float, int]:
        a_positions, a_velocities, a_accelerations = self.a.episode_values(
            candidate.a_phase
        )
        b_positions, b_velocities, b_accelerations = self.b.episode_values(
            candidate.b_phase
        )
        checks: list[RepresentativeBridgeCandidate] = []
        for position, velocity, acceleration in zip(
            a_positions, a_velocities, a_accelerations, strict=True
        ):
            checks.append(
                self._evaluate(
                    a_phase=candidate.a_phase,
                    b_phase=candidate.b_phase,
                    duration_s=candidate.duration_s,
                    sample_hz=sample_hz,
                    stage="robust_a_residual",
                    position_a_mm=position,
                    velocity_a_mm_s=velocity,
                    acceleration_a_mm_s2=acceleration,
                )
            )
        for position, velocity, acceleration in zip(
            b_positions, b_velocities, b_accelerations, strict=True
        ):
            checks.append(
                self._evaluate(
                    a_phase=candidate.a_phase,
                    b_phase=candidate.b_phase,
                    duration_s=candidate.duration_s,
                    sample_hz=sample_hz,
                    stage="robust_b_residual",
                    position_b_mm=position,
                    velocity_b_mm_s=velocity,
                    acceleration_b_mm_s2=acceleration,
                )
            )
        passed = sum(item.feasible for item in checks)
        return float(passed / len(checks)), len(checks)

    def optimize(
        self,
        *,
        record: Callable[[RepresentativeBridgeCandidate], None] | None = None,
    ) -> RepresentativeOptimizationResult:
        search = self.config["search"]
        selection = self.config["selection"]
        durations = _duration_grid(search["duration_search"])
        coarse_phases = _grid(float(search["coarse_phase_step"]))
        coarse_hz = float(self.config["sampling"]["coarse_sample_hz"])
        final_hz = float(self.config["sampling"]["final_sample_hz"])
        evaluated: dict[tuple[float, float, float], RepresentativeBridgeCandidate] = {}
        failures: dict[str, int] = {}

        def add(candidate: RepresentativeBridgeCandidate) -> None:
            key = (
                round(candidate.a_phase, 10),
                round(candidate.b_phase, 10),
                round(candidate.duration_s, 10),
            )
            previous = evaluated.get(key)
            if previous is None or candidate.search_stage != "coarse":
                evaluated[key] = candidate
            for reason in candidate.failure_reasons:
                failures[reason] = failures.get(reason, 0) + 1
            if record is not None:
                record(candidate)

        coarse_count = 0
        for a_phase in coarse_phases:
            for b_phase in coarse_phases:
                for duration in durations:
                    candidate = self._evaluate(
                        a_phase=float(a_phase),
                        b_phase=float(b_phase),
                        duration_s=float(duration),
                        sample_hz=coarse_hz,
                        stage="coarse",
                    )
                    coarse_count += 1
                    add(candidate)

        coarse_feasible = [
            candidate
            for candidate in evaluated.values()
            if candidate.feasible and candidate.search_stage == "coarse"
        ]
        if not coarse_feasible:
            raise ValueError("coarse representative search found no feasible Bridge")
        ordered_coarse = sorted(
            coarse_feasible,
            key=lambda candidate: (
                candidate.total_c_length_mm,
                candidate.smoothness_key,
            ),
        )
        seed_pairs: list[tuple[float, float]] = []
        for candidate in ordered_coarse:
            pair = (candidate.a_phase, candidate.b_phase)
            if pair not in seed_pairs:
                seed_pairs.append(pair)
            if len(seed_pairs) >= int(search["refine_seed_pairs"]):
                break

        refined_count = 0
        for a_center, b_center in seed_pairs:
            for a_phase in self._phase_refinement(a_center):
                for b_phase in self._phase_refinement(b_center):
                    for duration in durations:
                        key = (
                            round(float(a_phase), 10),
                            round(float(b_phase), 10),
                            round(float(duration), 10),
                        )
                        if key in evaluated:
                            continue
                        candidate = self._evaluate(
                            a_phase=float(a_phase),
                            b_phase=float(b_phase),
                            duration_s=float(duration),
                            sample_hz=coarse_hz,
                            stage="refined",
                        )
                        refined_count += 1
                        add(candidate)

        feasible = [candidate for candidate in evaluated.values() if candidate.feasible]
        if not feasible:
            raise ValueError("representative refinement found no feasible Bridge")
        shortest = min(candidate.total_c_length_mm for candidate in feasible)
        validation_margin = float(selection["near_shortest_delta"]) + float(
            search.get("final_validation_length_margin", 0.01)
        )
        validation_pool = [
            candidate
            for candidate in feasible
            if candidate.total_c_length_mm <= shortest * (1.0 + validation_margin)
        ]
        validation_pool = sorted(
            validation_pool,
            key=lambda candidate: (
                candidate.total_c_length_mm,
                candidate.smoothness_key,
            ),
        )[: int(search["final_validation_top_n"])]

        final_candidates: list[RepresentativeBridgeCandidate] = []
        robust_count = 0
        robust_required = float(search["robust_required_fraction"])
        for candidate in validation_pool:
            final_candidate = self._evaluate(
                a_phase=candidate.a_phase,
                b_phase=candidate.b_phase,
                duration_s=candidate.duration_s,
                sample_hz=final_hz,
                stage="final_validation",
            )
            if not final_candidate.feasible:
                if record is not None:
                    record(final_candidate)
                continue
            robust_fraction, checks = self._robust_fraction(
                final_candidate, final_hz
            )
            robust_count += 1
            robust_candidate = replace(
                final_candidate,
                robust_pass_fraction=robust_fraction,
                robust_checks=checks,
                feasible=robust_fraction >= robust_required,
                failure_reasons=(
                    final_candidate.failure_reasons
                    if robust_fraction >= robust_required
                    else final_candidate.failure_reasons
                    + ("representative_robust_fraction",)
                ),
            )
            if record is not None:
                record(robust_candidate)
            final_candidates.append(robust_candidate)
        selected = _select(
            final_candidates,
            float(selection["near_shortest_delta"]),
        )
        final_feasible = [candidate for candidate in final_candidates if candidate.feasible]
        final_shortest = min(
            candidate.total_c_length_mm for candidate in final_feasible
        )
        near = [
            candidate
            for candidate in final_feasible
            if candidate.total_c_length_mm
            <= final_shortest * (1.0 + float(selection["near_shortest_delta"]))
        ]
        top_k = tuple(
            sorted(
                near,
                key=lambda candidate: (
                    candidate.smoothness_key,
                    candidate.total_c_length_mm,
                ),
            )[: int(selection["top_k"])]
        )
        return RepresentativeOptimizationResult(
            selected=selected,
            top_k=top_k,
            coarse_evaluated=coarse_count,
            refined_evaluated=refined_count,
            final_validated=len(validation_pool),
            robust_validated=robust_count,
            feasible_candidates=len(feasible),
            failures_by_reason=failures,
        )
