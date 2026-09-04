"""Exhaustive semantic-pair and duration search for composed Task C."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable

import numpy as np

from .bezier_bridge import CubicBezierBridge, build_velocity_matched_bezier
from .bridge_metrics import BridgeMetrics, evaluate_bridge, feasibility_reasons
from .semantic_candidates import (
    is_a_cut_valid,
    is_b_entry_valid,
    semantic_compatibility,
)
from .trajectory_states import CandidatePoint


class NoFeasibleBridgeError(RuntimeError):
    def __init__(
        self,
        message: str,
        *,
        failure_counts: dict[str, int] | None = None,
        candidates_evaluated: int = 0,
        semantic_rejected_entries: int = 0,
    ) -> None:
        super().__init__(message)
        self.failure_counts = dict(failure_counts or {})
        self.candidates_evaluated = int(candidates_evaluated)
        self.semantic_rejected_entries = int(semantic_rejected_entries)


@dataclass
class CandidateEvaluation:
    a: CandidatePoint
    b: CandidatePoint
    bridge: CubicBezierBridge
    metrics: BridgeMetrics
    a_retained_length_mm: float
    b_retained_length_mm: float
    total_c_length_mm: float
    semantic_compatible: bool
    semantic_unchecked: list[str]
    feasible: bool
    failure_reasons: list[str]

    @property
    def smoothness_key(self) -> tuple[float, float, float]:
        return (
            self.metrics.max_acceleration_mm_s2,
            self.metrics.max_curvature_per_mm,
            self.metrics.integrated_squared_jerk,
        )


def select_best_candidate(
    feasible: list[CandidateEvaluation], near_shortest_delta: float
) -> CandidateEvaluation:
    if not feasible:
        raise NoFeasibleBridgeError("no semantically and dynamically feasible bridge")
    shortest = min(candidate.total_c_length_mm for candidate in feasible)
    near = [
        candidate
        for candidate in feasible
        if candidate.total_c_length_mm <= shortest * (1.0 + near_shortest_delta)
    ]
    return min(near, key=lambda candidate: (candidate.smoothness_key, candidate.total_c_length_mm))


class BridgeOptimizer:
    def __init__(self, config: dict):
        self.config = config
        self.runtime_b_entry_evaluations: list[CandidateEvaluation] = []

    def duration_candidates(self) -> np.ndarray:
        duration = self.config["duration_search"]
        start = float(duration["bridge_duration_min_s"])
        stop = float(duration["bridge_duration_max_s"])
        step = float(duration["bridge_duration_step_s"])
        if start <= 0.0 or stop < start or step <= 0.0:
            raise ValueError("invalid duration search bounds")
        count = int(np.floor((stop - start) / step + 1e-9)) + 1
        return start + np.arange(count) * step

    def optimize(
        self,
        a_candidates: Iterable[CandidatePoint],
        b_candidates: Iterable[CandidatePoint],
        *,
        workspace_min_mm: np.ndarray,
        workspace_max_mm: np.ndarray,
        record: Callable[[CandidateEvaluation], None] | None = None,
    ) -> tuple[CandidateEvaluation, list[CandidateEvaluation], dict[str, int]]:
        self.runtime_b_entry_evaluations = []
        semantic_config = self.config["semantic_compatibility"]
        limits = self.config["feasibility"]
        sample_hz = float(self.config["sampling"]["bridge_sample_hz"])
        curvature_epsilon = float(self.config["sampling"]["curvature_epsilon"])
        feasible: list[CandidateEvaluation] = []
        counters = {
            "candidate_pairs": 0,
            "semantic_rejected_pairs": 0,
            "duration_candidates_evaluated": 0,
            "feasible_candidates": 0,
        }
        for a in a_candidates:
            if not is_a_cut_valid(a, semantic_config):
                continue
            a_length = float(a.trajectory.cumulative_length_mm[a.index])
            for b in b_candidates:
                if not is_b_entry_valid(b, semantic_config):
                    continue
                counters["candidate_pairs"] += 1
                compatible, semantic_reasons, unchecked = semantic_compatibility(
                    a, b, semantic_config
                )
                if not compatible:
                    counters["semantic_rejected_pairs"] += 1
                    # Semantic-incompatible pairs are hard-filtered before T search.
                    continue
                b_length = float(
                    b.trajectory.total_length_mm
                    - b.trajectory.cumulative_length_mm[b.index]
                )
                for duration in self.duration_candidates():
                    counters["duration_candidates_evaluated"] += 1
                    bridge = build_velocity_matched_bezier(
                        a.position_mm,
                        a.velocity_mm_s,
                        b.position_mm,
                        b.velocity_mm_s,
                        float(duration),
                    )
                    metrics, sample = evaluate_bridge(
                        bridge,
                        sample_hz=sample_hz,
                        curvature_epsilon=curvature_epsilon,
                        workspace_min_mm=workspace_min_mm,
                        workspace_max_mm=workspace_max_mm,
                    )
                    failures = semantic_reasons + feasibility_reasons(metrics, limits)
                    endpoint_floors = [
                        floor
                        for floor in (a.minimum_bridge_z_mm, b.minimum_bridge_z_mm)
                        if floor is not None
                    ]
                    if endpoint_floors and self.config["feasibility"].get(
                        "enforce_payload_transport_floor", False
                    ):
                        required_floor = max(endpoint_floors)
                        tolerance = float(
                            self.config["feasibility"].get(
                                "transport_floor_tolerance_mm", 1e-6
                            )
                        )
                        if float(np.min(sample["position_mm"][:, 2])) < required_floor - tolerance:
                            failures.append("payload_clearance_violation")
                    evaluation = CandidateEvaluation(
                        a=a,
                        b=b,
                        bridge=bridge,
                        metrics=metrics,
                        a_retained_length_mm=a_length,
                        b_retained_length_mm=b_length,
                        total_c_length_mm=a_length + metrics.length_mm + b_length,
                        semantic_compatible=True,
                        semantic_unchecked=unchecked,
                        feasible=not failures,
                        failure_reasons=failures,
                    )
                    if record is not None:
                        record(evaluation)
                    if evaluation.feasible:
                        feasible.append(evaluation)
        counters["feasible_candidates"] = len(feasible)
        best_by_b_entry: dict[
            tuple[str, int, int, str], CandidateEvaluation
        ] = {}
        for evaluation in feasible:
            key = (
                evaluation.b.trajectory.dataset,
                evaluation.b.trajectory.episode,
                evaluation.b.frame,
                evaluation.b.semantic_label,
            )
            incumbent = best_by_b_entry.get(key)
            if incumbent is None or (
                evaluation.total_c_length_mm,
                evaluation.smoothness_key,
            ) < (
                incumbent.total_c_length_mm,
                incumbent.smoothness_key,
            ):
                best_by_b_entry[key] = evaluation
        runtime_entry_limit = int(
            self.config["selection"].get("runtime_entry_top_k", 20)
        )
        self.runtime_b_entry_evaluations = sorted(
            best_by_b_entry.values(),
            key=lambda candidate: (
                candidate.total_c_length_mm,
                candidate.smoothness_key,
            ),
        )[:runtime_entry_limit]
        best = select_best_candidate(
            feasible,
            float(self.config["selection"]["near_shortest_delta"]),
        )
        top_k = sorted(
            feasible,
            key=lambda candidate: (candidate.total_c_length_mm, candidate.smoothness_key),
        )[: int(self.config["selection"]["top_k"])]
        return best, top_k, counters
