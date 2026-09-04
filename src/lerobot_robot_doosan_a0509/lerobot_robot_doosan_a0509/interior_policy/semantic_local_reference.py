"""Offline semantic-local boundary selection for FLEXIBLE_LEVEL2.

The Dijkstra operator/switch cost is intentionally not changed here.  This
module only selects a physical Bridge reference inside a planner-approved
semantic edge using

    source semantic prefix + command-safe Bridge + successor semantic suffix.

The selected successor phase is reference metadata, never ACT-B authority.
Runtime takeover remains gated by a fresh policy prefix.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from offline_tools.cross_task_handoff.schema import PhaseIndexPoint

from lerobot_robot_doosan_a0509.task_c_handoff.bridge_runtime import (
    BridgeRuntimeLimits,
)
from lerobot_robot_doosan_a0509.task_c_handoff.flexible_bridge import (
    FlexibleBridgeSearchConfig,
    FlexibleBridgeSearchResult,
    nominal_medoid_bridge_snapshot,
    search_flexible_bridge_queue,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    EpisodeHandoffManifest,
    HandoffV2Config,
)

from .execution_tail_reference import (
    SEMANTIC_LOCAL_SELECTION_METHOD,
    boundary_mapping_from_reference_sample,
)


SEMANTIC_LOCAL_REPORT_SCHEMA_VERSION = (
    "a0509.semantic_local_bridge_reference_selection.v1"
)


@dataclass(frozen=True)
class SemanticLocalSelectionConfig:
    successor_interior_path_margin_mm: float = 15.0
    source_phase_step: float = 0.005
    successor_phase_step: float = 0.02
    max_pair_candidates: int = 384
    selected_candidate_rank: int = 0
    source_reference_phase_max: float | None = None

    def __post_init__(self) -> None:
        positive = (
            self.source_phase_step,
            self.successor_phase_step,
            self.max_pair_candidates,
        )
        if any(not np.isfinite(value) or value <= 0 for value in positive):
            raise ValueError("semantic-local selection bounds must be positive")
        if (
            not np.isfinite(self.successor_interior_path_margin_mm)
            or self.successor_interior_path_margin_mm < 0.0
        ):
            raise ValueError(
                "successor interior path margin must be finite and non-negative"
            )
        if (
            isinstance(self.selected_candidate_rank, bool)
            or self.selected_candidate_rank < 0
        ):
            raise ValueError("selected_candidate_rank must be non-negative")
        if self.source_reference_phase_max is not None and (
            not np.isfinite(self.source_reference_phase_max)
            or not 0.0 <= self.source_reference_phase_max <= 1.0
        ):
            raise ValueError(
                "source_reference_phase_max must be finite and in [0, 1]"
            )


@dataclass(frozen=True)
class _EpisodeArc:
    phase: np.ndarray
    xyz_mm: np.ndarray
    cumulative_mm: np.ndarray

    @property
    def total_mm(self) -> float:
        return float(self.cumulative_mm[-1])

    def prefix_mm(self, phase: float) -> float:
        return float(np.interp(float(phase), self.phase, self.cumulative_mm))

    def suffix_mm(self, phase: float) -> float:
        return self.total_mm - self.prefix_mm(phase)


@dataclass(frozen=True)
class SemanticLocalSelectionResult:
    manifest: EpisodeHandoffManifest | None
    geometry: FlexibleBridgeSearchResult | None
    source_point: PhaseIndexPoint | None
    successor_point: PhaseIndexPoint | None
    source_prefix_mm: float | None
    bridge_length_mm: float | None
    successor_suffix_mm: float | None
    total_length_mm: float | None
    path_margin_from_low_mm: float | None
    path_margin_to_high_mm: float | None
    pairs_considered: int
    pairs_hard_passed: int
    rejected_reason_counts: Mapping[str, int]
    candidates: tuple[Mapping[str, Any], ...]
    selected_candidate_rank: int | None

    source_reference_phase_max: float | None
    @property
    def valid(self) -> bool:
        return self.manifest is not None and self.geometry is not None

    def to_record(self) -> dict[str, Any]:
        selected = None
        if self.valid:
            assert self.manifest is not None
            assert self.source_point is not None
            assert self.successor_point is not None
            selected = {
                "handoff_id": self.manifest.handoff_id,
                "source": self.source_point.to_record(),
                "candidate_rank": self.selected_candidate_rank,
                "successor": self.successor_point.to_record(),
                "source_prefix_mm": self.source_prefix_mm,
                "bridge_length_mm": self.bridge_length_mm,
                "successor_suffix_mm": self.successor_suffix_mm,
                "total_length_mm": self.total_length_mm,
                "path_margin_from_low_mm": self.path_margin_from_low_mm,
                "path_margin_to_high_mm": self.path_margin_to_high_mm,
                "geometry": self.geometry.record(),
            }
        return {
            "schema_version": SEMANTIC_LOCAL_REPORT_SCHEMA_VERSION,
            "objective": "source_semantic_prefix+Bridge+successor_semantic_suffix",
            "dijkstra_operator_cost_changed": False,
            "successor_phase_role": "bridge_reference_and_metadata_only",
            "runtime_successor_phase_gate": False,
            "valid": self.valid,
            "pairs_considered": self.pairs_considered,
            "pairs_hard_passed": self.pairs_hard_passed,
            "rejected_reason_counts": dict(self.rejected_reason_counts),
            "selected_candidate_rank": self.selected_candidate_rank,
            "selected": selected,
            "bridge_transport_floor_mm": (
                None
                if self.manifest is None
                else self.manifest.transport_floor_mm
            ),
            "source_reference_phase_max": self.source_reference_phase_max,
            "candidates": list(self.candidates),
            "robot_commands_published": 0,
            "physical_validation_performed": False,
        }


def _episode_samples(
    bank: Mapping[str, Any],
    *,
    episode: int,
) -> list[PhaseIndexPoint]:
    records = [
        item
        for item in bank.get("episodes", ())
        if int(item["episode"]) == int(episode)
    ]
    if len(records) != 1:
        raise ValueError(f"reference bank lacks unique episode {episode}")
    return sorted(
        (
            PhaseIndexPoint.from_record(item)
            for item in records[0]["samples"]
        ),
        key=lambda item: (item.phase, item.support_frame),
    )


def _downsample_phase(
    points: list[PhaseIndexPoint],
    *,
    minimum_step: float,
) -> list[PhaseIndexPoint]:
    if not points:
        return []
    selected = [points[0]]
    for point in points[1:-1]:
        if point.phase >= selected[-1].phase + minimum_step - 1.0e-12:
            selected.append(point)
    if points[-1] != selected[-1]:
        selected.append(points[-1])
    return selected


def _load_episode_arc(
    artifact_path: str | Path,
    *,
    segment: str,
    episode: int,
) -> _EpisodeArc:
    prefix = f"{segment}_"
    with np.load(Path(artifact_path), allow_pickle=False) as archive:
        phase = np.asarray(archive[prefix + "phase"], dtype=np.float64)
        episode_ids = np.asarray(
            archive[prefix + "episode_ids"], dtype=np.int64
        )
        xyz = np.asarray(
            archive[prefix + "episode_xyz_mm"], dtype=np.float64
        )
    offsets = np.flatnonzero(episode_ids == int(episode))
    if len(offsets) != 1:
        raise ValueError(
            f"semantic support lacks unique episode {episode} for {segment}"
        )
    episode_xyz = xyz[int(offsets[0])]
    cumulative = np.concatenate(
        (
            np.zeros(1, dtype=np.float64),
            np.cumsum(np.linalg.norm(np.diff(episode_xyz, axis=0), axis=1)),
        )
    )
    return _EpisodeArc(phase, episode_xyz, cumulative)


def _compatible_with_template(
    point: PhaseIndexPoint,
    template: Any,
) -> bool:
    return (
        point.task == template.task
        and point.segment == template.segment
        and point.gripper_state == template.semantic.gripper_state
        and point.held_object == template.semantic.held_object
        and point.contact_mode == template.semantic.contact_mode
    )


def _candidate_manifest(
    base: EpisodeHandoffManifest,
    source_bank: Mapping[str, Any],
    successor_bank: Mapping[str, Any],
    source: PhaseIndexPoint,
    successor: PhaseIndexPoint,
) -> EpisodeHandoffManifest:
    value = base.to_record()
    identity = json.dumps(
        {
            "source": [source.support_episode, source.support_frame],
            "successor": [successor.support_episode, successor.support_frame],
            "duration_s": base.bridge_duration_s,
            "transport_floor_mm": base.transport_floor_mm,
            "method": SEMANTIC_LOCAL_SELECTION_METHOD,
        },
        sort_keys=True,
    ).encode("utf-8")
    value["handoff_id"] = (
        "h_semantic_" + hashlib.sha256(identity).hexdigest()[:12]
    )
    value["source"] = boundary_mapping_from_reference_sample(
        source_bank,
        source,
        support_radius_mm=base.source.support_radius_mm,
        prearm_radius_mm=base.source.prearm_radius_mm,
        commit_radius_mm=base.source.commit_radius_mm,
    )
    value["successor"] = boundary_mapping_from_reference_sample(
        successor_bank,
        successor,
        support_radius_mm=base.successor.support_radius_mm,
        prearm_radius_mm=base.successor.prearm_radius_mm,
        commit_radius_mm=base.successor.commit_radius_mm,
    )
    value["bridge"]["nominal_length_mm"] = None
    generation = dict(value["generation"])
    generation.pop("runtime_successor_reference", None)
    value["generation"] = generation
    return EpisodeHandoffManifest.from_mapping(value)


def select_semantic_local_reference(
    *,
    base_manifest: EpisodeHandoffManifest,
    source_bank: Mapping[str, Any],
    successor_bank: Mapping[str, Any],
    source_phase_support_artifact: str | Path,
    successor_phase_support_artifact: str | Path,
    runtime_successor_bank_path: str | Path,
    runtime_config: HandoffV2Config,
    limits: BridgeRuntimeLimits,
    bridge_search_config: FlexibleBridgeSearchConfig,
    config: SemanticLocalSelectionConfig | None = None,
) -> SemanticLocalSelectionResult:
    """Select one command-safe interior reference without robot I/O."""

    selection_config = config or SemanticLocalSelectionConfig()
    successor_bank_path = str(runtime_successor_bank_path).strip()
    if not successor_bank_path:
        raise ValueError("runtime_successor_bank_path must not be empty")
    source_episode = int(source_bank["selection"]["medoid_episode"])
    successor_episode = int(successor_bank["selection"]["medoid_episode"])
    source_points = _episode_samples(source_bank, episode=source_episode)
    successor_points = _episode_samples(
        successor_bank, episode=successor_episode
    )
    source_points = [
        item
        for item in source_points
        if _compatible_with_template(item, base_manifest.source)
    ]
    successor_points = [
        item
        for item in successor_points
        if _compatible_with_template(item, base_manifest.successor)
    ]
    source_points = _downsample_phase(
        source_points, minimum_step=selection_config.source_phase_step
    )
    successor_points = _downsample_phase(
        successor_points,
        minimum_step=selection_config.successor_phase_step,
    )
    if selection_config.source_reference_phase_max is not None:
        maximum = float(selection_config.source_reference_phase_max)
        source_points = [
            item for item in source_points if item.phase <= maximum + 1.0e-9
        ]
    if not source_points or not successor_points:
        raise ValueError(
            "semantic-local selection has no compatible source/successor support"
        )

    source_arc = _load_episode_arc(
        source_phase_support_artifact,
        segment=base_manifest.source.segment,
        episode=source_episode,
    )
    successor_arc = _load_episode_arc(
        successor_phase_support_artifact,
        segment=base_manifest.successor.segment,
        episode=successor_episode,
    )
    successor_low, successor_high = (
        float(item) for item in successor_bank["phase_window"]
    )
    low_arc = successor_arc.prefix_mm(successor_low)
    high_arc = successor_arc.prefix_mm(successor_high)
    interior: list[tuple[PhaseIndexPoint, float, float]] = []
    for point in successor_points:
        point_arc = successor_arc.prefix_mm(point.phase)
        from_low = point_arc - low_arc
        to_high = high_arc - point_arc
        if (
            from_low + 1.0e-9
            >= selection_config.successor_interior_path_margin_mm
            and to_high + 1.0e-9
            >= selection_config.successor_interior_path_margin_mm
        ):
            interior.append((point, from_low, to_high))
    if not interior:
        raise ValueError(
            "successor support has no candidate after applying interior margin"
        )

    pair_count = len(source_points) * len(interior)
    if pair_count > selection_config.max_pair_candidates:
        raise ValueError(
            "semantic-local bounded search exceeds max_pair_candidates: "
            f"pairs={pair_count} limit={selection_config.max_pair_candidates}"
        )

    candidates: list[dict[str, Any]] = []
    passing: list[
        tuple[
            tuple[float, float, float, float],
            EpisodeHandoffManifest,
            FlexibleBridgeSearchResult,
            PhaseIndexPoint,
            PhaseIndexPoint,
            float,
            float,
            float,
            float,
            float,
        ]
    ] = []
    rejected: dict[str, int] = {}
    for source in source_points:
        source_prefix = source_arc.prefix_mm(source.phase)
        for successor, from_low, to_high in interior:
            candidate_manifest = _candidate_manifest(
                base_manifest,
                source_bank,
                successor_bank,
                source,
                successor,
            )
            snapshot = nominal_medoid_bridge_snapshot(
                candidate_manifest, timestamp_s=0.0
            )
            search = search_flexible_bridge_queue(
                candidate_manifest,
                snapshot,
                runtime_config,
                limits,
                bridge_search_config,
                live_mode=False,
            )
            row: dict[str, Any] = {
                "source_phase": source.phase,
                "source_episode": source.support_episode,
                "source_frame": source.support_frame,
                "successor_phase": successor.phase,
                "successor_episode": successor.support_episode,
                "successor_frame": successor.support_frame,
                "path_margin_from_low_mm": from_low,
                "path_margin_to_high_mm": to_high,
                "geometry_valid": search.valid,
                "geometry_search_latency_ms": search.search_latency_s * 1000.0,
                "geometry_rejection_reason_counts": dict(
                    search.rejected_reason_counts
                ),
                "bridge_transport_floor_mm": (
                    candidate_manifest.transport_floor_mm
                ),
            }
            if not search.valid or search.queue is None or search.selected is None:
                candidates.append(row)
                for reason, count in search.rejected_reason_counts.items():
                    rejected[reason] = rejected.get(reason, 0) + int(count)
                continue
            bridge_length = float(search.queue.metrics.length_mm)
            successor_suffix = successor_arc.suffix_mm(successor.phase)
            total = source_prefix + bridge_length + successor_suffix
            row.update(
                {
                    "source_prefix_mm": source_prefix,
                    "bridge_length_mm": bridge_length,
                    "successor_suffix_mm": successor_suffix,
                    "total_length_mm": total,
                    "geometry_score": float(search.selected.score),
                    "generator_type": search.selected.generator_type,
                    "max_position_axis_step_mm": (
                        search.selected.max_position_axis_step_mm
                    ),
                    "max_orientation_step_deg": (
                        search.selected.max_orientation_step_deg
                    ),
                    "max_acceleration_mm_s2": (
                        search.selected.max_acceleration_mm_s2
                    ),
                    "max_jerk_mm_s3": search.selected.max_jerk_mm_s3,
                    "minimum_position_z_mm": (
                        search.selected.minimum_position_z_mm
                    ),
                }
            )
            candidates.append(row)
            rank = (
                total,
                float(search.selected.score),
                successor_suffix,
                source.phase,
            )
            passing.append(
                (
                    rank,
                    candidate_manifest,
                    search,
                    source,
                    successor,
                    source_prefix,
                    bridge_length,
                    successor_suffix,
                    from_low,
                    to_high,
                )
            )
    if not passing:
        return SemanticLocalSelectionResult(
            manifest=None,
            geometry=None,
            source_point=None,
            successor_point=None,
            source_prefix_mm=None,
            bridge_length_mm=None,
            successor_suffix_mm=None,
            total_length_mm=None,
            path_margin_from_low_mm=None,
            path_margin_to_high_mm=None,
            pairs_considered=pair_count,
            pairs_hard_passed=0,
            selected_candidate_rank=None,
            rejected_reason_counts=rejected,
            candidates=tuple(candidates),
            source_reference_phase_max=(
                selection_config.source_reference_phase_max
            ),
        )
    ranked = sorted(passing, key=lambda item: item[0])
    selected_rank = int(selection_config.selected_candidate_rank)
    if selected_rank >= len(ranked):
        raise ValueError(
            "selected semantic-local candidate rank is out of range: "
            f"rank={selected_rank} candidates={len(ranked)}"
        )
    (
        _rank,
        selected_manifest,
        selected_search,
        selected_source,
        selected_successor,
        source_prefix,
        bridge_length,
        successor_suffix,
        from_low,
        to_high,
    ) = ranked[selected_rank]
    total = source_prefix + bridge_length + successor_suffix
    value = selected_manifest.to_record()
    value["bridge"]["nominal_length_mm"] = bridge_length
    generation = dict(value["generation"])
    generation["runtime_successor_reference"] = {
        "support_bank": successor_bank_path,
        "selection_method": SEMANTIC_LOCAL_SELECTION_METHOD,
        "phase_window": list(successor_bank["phase_window"]),
        "selected_reference_phase": selected_successor.phase,
        "runtime_phase_gate": False,
        "authority": "bridge_reference_and_metadata_only",
        "interior_path_margin_mm": (
            selection_config.successor_interior_path_margin_mm
        ),
        "semantic_local_objective": {
            "source_prefix_mm": source_prefix,
            "bridge_length_mm": bridge_length,
            "successor_suffix_mm": successor_suffix,
            "total_length_mm": total,
        },
    }
    value["generation"] = generation
    final_manifest = EpisodeHandoffManifest.from_mapping(value)
    return SemanticLocalSelectionResult(
        manifest=final_manifest,
        geometry=selected_search,
        source_point=selected_source,
        successor_point=selected_successor,
        source_prefix_mm=source_prefix,
        bridge_length_mm=bridge_length,
        successor_suffix_mm=successor_suffix,
        total_length_mm=total,
        path_margin_from_low_mm=from_low,
        path_margin_to_high_mm=to_high,
        pairs_considered=pair_count,
        pairs_hard_passed=len(passing),
        selected_candidate_rank=selected_rank,
        rejected_reason_counts=rejected,
        candidates=tuple(candidates),
        source_reference_phase_max=(
            selection_config.source_reference_phase_max
        ),
    )
