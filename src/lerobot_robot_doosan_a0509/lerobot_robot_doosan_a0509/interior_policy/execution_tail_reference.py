"""Real-episode reference banks for zero-cost execution-tail Bridges.

This module is command-free. It reads the audited LeRobot datasets and semantic
phase artifacts, selects a real episode medoid near the component-median path,
and validates the sidecar used by FLEXIBLE_LEVEL2 manifests.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from offline_tools.cross_task_handoff.build_episode_phase_index import (
    build_phase_index,
)
from offline_tools.cross_task_handoff.schema import PhaseIndexPoint

from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    EpisodeHandoffManifest,
)
from lerobot_robot_doosan_a0509.task_c_handoff.source_phase import (
    SourcePhaseSupportBank,
)


RUNTIME_SOURCE_BANK_SCHEMA_VERSION = (
    "a0509.execution_tail_runtime_source_bank.v1"
)
MEDOID_SELECTION_METHOD = (
    "minimum_window_rms_to_component_median_real_episode_v1"
)
SEMANTIC_LOCAL_SELECTION_METHOD = (
    "minimum_semantic_prefix_bridge_suffix_with_empirical_interior_margin_v1"
)
EMPIRICAL_TRANSPORT_WINDOW_SCHEMA_VERSION = (
    "a0509.empirical_transport_semantic_phase_window.v1"
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_support_window(
    artifact_path: Path,
    *,
    segment: str,
    phase_window: tuple[float, float],
) -> dict[str, np.ndarray]:
    prefix = f"{segment}_"
    required = {
        "phase": prefix + "phase",
        "episode_ids": prefix + "episode_ids",
        "episode_xyz_mm": prefix + "episode_xyz_mm",
        "median_xyz_mm": prefix + "median_xyz_mm",
    }
    with np.load(artifact_path, allow_pickle=False) as archive:
        missing = [name for name in required.values() if name not in archive.files]
        if missing:
            raise ValueError(
                f"phase support lacks {segment}: {','.join(missing)}"
            )
        values = {
            name: np.asarray(archive[key]).copy()
            for name, key in required.items()
        }
    low, high = (float(item) for item in phase_window)
    mask = (
        (values["phase"] >= low - 1.0e-12)
        & (values["phase"] <= high + 1.0e-12)
    )
    if not np.any(mask):
        raise ValueError("runtime source phase window has no support points")
    values["window_mask"] = mask
    return values


def build_runtime_reference_bank(
    *,
    reference_id: str,
    role: str,
    task_id: str,
    dataset_root: str | Path,
    semantic_graph_path: str | Path,
    phase_support_artifact: str | Path,
    segment: str,
    phase_window: tuple[float, float],
    nominal_phase: float,
    source_operator: str | None = None,
    phase_step: float = 0.005,
    support_loo_quantile: float = 0.95,
) -> dict[str, Any]:
    """Build one 30-episode phase-window bank and select its real medoid."""

    if role not in {
        "source_execution_tail",
        "source_semantic_exit",
        "successor_entry",
    }:
        raise ValueError("unsupported runtime reference bank role")
    low, high = (float(item) for item in phase_window)
    nominal = float(nominal_phase)
    if not (0.0 <= low <= nominal <= high <= 1.0):
        raise ValueError("reference phase window must contain nominal phase")
    dataset = Path(dataset_root).expanduser().resolve()
    graph_path = Path(semantic_graph_path).expanduser().resolve()
    support_path = Path(phase_support_artifact).expanduser().resolve()
    if not dataset.is_dir() or not graph_path.is_file() or not support_path.is_file():
        raise FileNotFoundError("runtime reference inputs are incomplete")

    support = _load_support_window(
        support_path,
        segment=segment,
        phase_window=(low, high),
    )
    phase = np.asarray(support["phase"], dtype=np.float64)
    episode_ids = np.asarray(support["episode_ids"], dtype=np.int64)
    episode_xyz = np.asarray(support["episode_xyz_mm"], dtype=np.float64)
    median_xyz = np.asarray(support["median_xyz_mm"], dtype=np.float64)
    mask = np.asarray(support["window_mask"], dtype=np.bool_)
    residual = episode_xyz[:, mask] - median_xyz[None, mask]
    rms_mm = np.sqrt(np.mean(np.sum(np.square(residual), axis=2), axis=1))
    medoid_offset = int(np.argmin(rms_mm))
    medoid_episode = int(episode_ids[medoid_offset])

    phase_index = build_phase_index(
        dataset_root=dataset,
        semantic_graph_path=graph_path,
        task_id=task_id,
        phase_step=phase_step,
        velocity_window_frames=15,
    )
    points = [
        PhaseIndexPoint.from_record(item)
        for item in phase_index["points"]
        if str(item["segment"]) == segment
        and low - phase_step * 0.51
        <= float(item["phase"])
        <= high + phase_step * 0.51
    ]
    by_episode: dict[int, list[PhaseIndexPoint]] = {}
    for point in points:
        by_episode.setdefault(point.support_episode, []).append(point)
    if set(by_episode) != set(int(item) for item in episode_ids):
        raise ValueError("phase-index episodes differ from semantic support")
    selected_points = sorted(
        by_episode[medoid_episode],
        key=lambda item: (
            abs(item.phase - nominal),
            item.support_frame,
        ),
    )
    if not selected_points:
        raise ValueError("medoid episode lacks nominal phase support")
    selected = selected_points[0]
    if abs(selected.phase - nominal) > phase_step * 0.51:
        raise ValueError("phase step cannot represent the nominal reference")

    calibrated = SourcePhaseSupportBank.load(
        support_path,
        segment=segment,
        phase_window=(low, high),
        support_distance_threshold_mm=None,
        support_loo_quantile=support_loo_quantile,
    )
    episode_records = []
    rms_by_episode = {
        int(episode): float(rms)
        for episode, rms in zip(episode_ids, rms_mm, strict=True)
    }
    for episode in sorted(by_episode):
        episode_records.append(
            {
                "episode": episode,
                "window_rms_to_component_median_mm": rms_by_episode[episode],
                "samples": [
                    item.to_record()
                    for item in sorted(
                        by_episode[episode],
                        key=lambda value: value.phase,
                    )
                ],
            }
        )
    ranking = sorted(
        (
            {"episode": int(episode), "window_rms_mm": float(rms)}
            for episode, rms in zip(episode_ids, rms_mm, strict=True)
        ),
        key=lambda item: (item["window_rms_mm"], item["episode"]),
    )
    return {
        "schema_version": RUNTIME_SOURCE_BANK_SCHEMA_VERSION,
        "reference_id": str(reference_id),
        "role": role,
        "source_operator": source_operator,
        "task": str(task_id),
        "segment": str(segment),
        "phase_window": [low, high],
        "nominal_phase": nominal,
        "phase_step": float(phase_step),
        "episode_count": int(len(episode_ids)),
        "dataset_root": str(dataset),
        "semantic_graph": str(graph_path),
        "phase_support_artifact": str(support_path),
        "phase_support_sha256": _sha256(support_path),
        "support_phase_points": phase[mask].tolist(),
        "support_distance_threshold_mm": (
            calibrated.support_distance_threshold_mm
        ),
        "support_threshold_source": calibrated.support_threshold_source,
        "selection": {
            "method": MEDOID_SELECTION_METHOD,
            "medoid_episode": medoid_episode,
            "medoid_window_rms_mm": float(rms_mm[medoid_offset]),
            "episode_ranking": ranking,
            "synthetic_median_used_as_command": False,
            "real_episode_selected": True,
        },
        "selected_boundary": selected.to_record(),
        "episodes": episode_records,
        "robot_commands_published": 0,
        "physical_validation_performed": False,
    }


def boundary_mapping_from_reference_sample(
    bank: Mapping[str, Any],
    sample: Mapping[str, Any] | PhaseIndexPoint,
    *,
    support_radius_mm: float | None = None,
    prearm_radius_mm: float = 40.0,
    commit_radius_mm: float = 20.0,
) -> dict[str, Any]:
    """Convert one real bank sample into a HandoffBoundary mapping."""

    selected = (
        sample
        if isinstance(sample, PhaseIndexPoint)
        else PhaseIndexPoint.from_record(dict(sample))
    )
    radius = (
        float(bank["support_distance_threshold_mm"])
        if support_radius_mm is None
        else float(support_radius_mm)
    )
    return {
        "task": selected.task,
        "segment": selected.segment,
        "phase": selected.phase,
        "support_episode": selected.support_episode,
        "support_frame": selected.support_frame,
        "nominal_pose": {
            "position_mm": selected.position_mm.tolist(),
            "orientation_quat_xyzw": (
                selected.orientation_quat_xyzw.tolist()
            ),
        },
        "nominal_velocity_mm_s": selected.velocity_mm_s.tolist(),
        "semantic": {
            "gripper_state": selected.gripper_state,
            "held_object": selected.held_object,
            "contact_mode": selected.contact_mode,
            "semantic_state": selected.semantic_state,
            "entry_preconditions": list(selected.entry_preconditions),
        },
        "support_radius_mm": radius,
        "support_orientation_radius_deg": 30.0,
        "prearm_radius_mm": float(prearm_radius_mm),
        "commit_radius_mm": float(commit_radius_mm),
        "direction_cosine_minimum": 0.7,
        "approach_frames": 3,
        "commit_stable_frames": 3,
    }


def boundary_mapping_from_reference_bank(
    bank: Mapping[str, Any],
    *,
    support_radius_mm: float | None = None,
    prearm_radius_mm: float = 40.0,
    commit_radius_mm: float = 20.0,
) -> dict[str, Any]:
    """Convert the selected real sample into a HandoffBoundary mapping."""

    return boundary_mapping_from_reference_sample(
        bank,
        dict(bank["selected_boundary"]),
        support_radius_mm=support_radius_mm,
        prearm_radius_mm=prearm_radius_mm,
        commit_radius_mm=commit_radius_mm,
    )


def _semantic_transport_transition_ordinal(
    support_path: Path, segment: str
) -> int:
    """Resolve Cn->On segment anchors to a zero-based gripper span."""

    graph_path = support_path.parent / "semantic_graph_v3.json"
    if not graph_path.is_file():
        return 0
    graph = json.loads(graph_path.read_text(encoding="utf-8"))
    matches = [
        dict(item["spec"])
        for item in graph.get("segments", ())
        if str(item.get("spec", {}).get("segment_id")) == str(segment)
    ]
    if len(matches) != 1:
        raise ValueError(f"semantic graph lacks unique segment {segment}")
    start = str(matches[0].get("start_anchor", ""))
    end = str(matches[0].get("end_anchor", ""))
    if not (start.startswith("C") and end.startswith("O")):
        return 0
    try:
        close_ordinal = int(start[1:])
        open_ordinal = int(end[1:])
    except ValueError as exc:
        raise ValueError("semantic transport anchors are not ordinal events") from exc
    if close_ordinal < 1 or close_ordinal != open_ordinal:
        raise ValueError("semantic transport requires matching Cn->On anchors")
    return close_ordinal - 1


def derive_empirical_transport_phase_window(
    *,
    dataset_root: str | Path,
    phase_support_artifact: str | Path,
    segment: str,
    expected_episode_count: int | None = None,
    start_offset_frames: int = 30,
    end_offset_frames: int = -30,
    minimum_transport_clearance_mm: float = 50.0,
    start_quantile: float = 0.95,
    end_quantile: float = 0.05,
) -> dict[str, Any]:
    """Map existing 30-episode transport support back to semantic phase.

    This derives a candidate *reference domain*. It does not enable a runtime
    Z-minimum or certify collision/IK safety. The robust common interval uses
    an upper quantile of episode starts and a lower quantile of episode ends so
    a late outlier cannot drag the successor reference next to release.
    """

    from offline_tools.task_c_bridge_v0.dataset_io import (
        load_lerobot_trajectories,
    )
    from offline_tools.task_c_bridge_v1.representative_trajectory import (
        TransportWindowConfig,
        standardize_episode,
    )

    if not 0.0 <= start_quantile <= 1.0:
        raise ValueError("start_quantile must be in [0, 1]")
    if not 0.0 <= end_quantile <= 1.0:
        raise ValueError("end_quantile must be in [0, 1]")
    dataset = Path(dataset_root).expanduser().resolve()
    support_path = Path(phase_support_artifact).expanduser().resolve()
    transition_ordinal = _semantic_transport_transition_ordinal(
        support_path, segment
    )
    trajectories, info = load_lerobot_trajectories(dataset, str(dataset))
    if expected_episode_count is not None and (
        len(trajectories) != int(expected_episode_count)
    ):
        raise ValueError(
            "empirical transport window episode count differs from catalog"
        )
    prefix = f"{segment}_"
    with np.load(support_path, allow_pickle=False) as archive:
        phase = np.asarray(archive[prefix + "phase"], dtype=np.float64)
        episode_ids = np.asarray(
            archive[prefix + "episode_ids"], dtype=np.int64
        )
        episode_xyz = np.asarray(
            archive[prefix + "episode_xyz_mm"], dtype=np.float64
        )
    config = TransportWindowConfig(
        start_offset_frames=int(start_offset_frames),
        end_offset_frames=int(end_offset_frames),
        minimum_transport_clearance_mm=float(
            minimum_transport_clearance_mm
        ),
        minimum_z_mm=None,
        phase_points=len(phase),
        velocity_window_frames=15,
        minimum_episodes=(
            len(trajectories)
            if expected_episode_count is None
            else int(expected_episode_count)
        ),
        floor_quantile=0.95,
        transition_ordinal=transition_ordinal,
    )
    records: list[dict[str, Any]] = []
    failures: list[str] = []
    for trajectory in trajectories:
        try:
            standardized = standardize_episode(trajectory, config)
        except ValueError as exc:
            failures.append(str(exc))
            continue
        offsets = np.flatnonzero(episode_ids == trajectory.episode)
        if len(offsets) != 1:
            failures.append(
                f"episode {trajectory.episode} semantic support missing"
            )
            continue
        episode_offset = int(offsets[0])
        semantic_xyz = episode_xyz[episode_offset]
        start_xyz = trajectory.xyz_mm[standardized.source_start_index]
        end_xyz = trajectory.xyz_mm[standardized.source_end_index]
        start_index = int(
            np.argmin(np.linalg.norm(semantic_xyz - start_xyz, axis=1))
        )
        end_index = int(
            np.argmin(np.linalg.norm(semantic_xyz - end_xyz, axis=1))
        )
        records.append(
            {
                "episode": int(trajectory.episode),
                "phase_low": float(phase[start_index]),
                "phase_high": float(phase[end_index]),
                "source_start_frame": int(
                    trajectory.frame_index[standardized.source_start_index]
                ),
                "source_end_frame": int(
                    trajectory.frame_index[standardized.source_end_index]
                ),
                "event_relative_clearance_floor_mm": float(
                    standardized.minimum_bridge_z_mm
                ),
            }
        )
    required = (
        len(trajectories)
        if expected_episode_count is None
        else int(expected_episode_count)
    )
    if len(records) != required:
        raise ValueError(
            "empirical transport support did not retain every expected "
            f"episode: retained={len(records)} required={required} "
            f"failures={failures}"
        )
    lows = np.asarray([item["phase_low"] for item in records])
    highs = np.asarray([item["phase_high"] for item in records])
    low = float(np.quantile(lows, start_quantile, method="higher"))
    high = float(np.quantile(highs, end_quantile, method="lower"))
    if not 0.0 <= low < high <= 1.0:
        raise ValueError(
            "robust empirical transport phase interval is empty"
        )
    return {
        "schema_version": EMPIRICAL_TRANSPORT_WINDOW_SCHEMA_VERSION,
        "segment": segment,
        "phase_window": [low, high],
        "episode_count": len(records),
        "fps": int(info["fps"]),
        "start_quantile": float(start_quantile),
        "end_quantile": float(end_quantile),
        "start_offset_frames": int(start_offset_frames),
        "end_offset_frames": int(end_offset_frames),
        "minimum_transport_clearance_mm": float(
            minimum_transport_clearance_mm
        ),
        "transition_ordinal": int(transition_ordinal),
        "fixed_z_minimum_enabled": False,
        "runtime_z_minimum_enabled": False,
        "role": "successor_candidate_domain_not_runtime_safety_gate",
        "episodes": records,
        "failures": failures,
    }


def _resolved_bank_path(
    manifest: EpisodeHandoffManifest,
    manifest_path: str | Path,
) -> Path:
    raw = manifest.runtime_source_bank_path
    if raw is None:
        raise ValueError(
            "execution-tail Bridge manifest lacks runtime source support bank"
        )
    value = Path(raw).expanduser()
    if not value.is_absolute():
        value = Path(manifest_path).expanduser().resolve().parent / value
    return value.resolve()


def validate_runtime_source_reference(
    manifest: EpisodeHandoffManifest,
    *,
    manifest_path: str | Path,
    execution_tail: Any,
) -> dict[str, Any]:
    """Fail closed unless manifest, tail, and selected medoid are identical."""

    bank_path = _resolved_bank_path(manifest, manifest_path)
    if not bank_path.is_file():
        raise FileNotFoundError(
            f"runtime source support bank not found: {bank_path}"
        )
    bank = json.loads(bank_path.read_text(encoding="utf-8"))
    if bank.get("schema_version") != RUNTIME_SOURCE_BANK_SCHEMA_VERSION:
        raise ValueError("unsupported runtime source support bank schema")
    expected_window = (
        float(execution_tail.commit_phase_low),
        float(execution_tail.commit_phase_high),
    )
    checks = (
        (
            str(bank.get("role")) == "source_execution_tail",
            "bank role",
        ),
        (
            str(bank.get("source_operator")) == execution_tail.source_operator,
            "source operator",
        ),
        (str(bank.get("task")) == manifest.source.task, "source task"),
        (
            str(bank.get("segment")) == execution_tail.tracking_segment,
            "tracking segment",
        ),
        (
            np.allclose(
                np.asarray(bank.get("phase_window"), dtype=np.float64),
                np.asarray(expected_window),
                atol=1.0e-12,
                rtol=0.0,
            ),
            "phase window",
        ),
        (
            abs(float(bank.get("nominal_phase")) - execution_tail.nominal_phase)
            <= 1.0e-12,
            "nominal phase",
        ),
        (
            int(bank.get("episode_count", -1)) == execution_tail.episode_count,
            "episode count",
        ),
        (
            str(bank.get("selection", {}).get("method"))
            == manifest.source_reference_selection_method
            == MEDOID_SELECTION_METHOD,
            "medoid selection method",
        ),
    )
    for valid, label in checks:
        if not valid:
            raise ValueError(
                f"runtime source support bank {label} disagrees with "
                "execution-tail manifest"
            )
    manifest_window = tuple(manifest.source_reference_phase_window or ())
    if not np.allclose(
        np.asarray(manifest_window, dtype=np.float64),
        np.asarray(expected_window),
        atol=1.0e-12,
        rtol=0.0,
    ):
        raise ValueError(
            "manifest runtime source phase window differs from execution tail"
        )
    selected = PhaseIndexPoint.from_record(dict(bank["selected_boundary"]))
    boundary = manifest.source
    if (
        selected.task != boundary.task
        or selected.segment != boundary.segment
        or selected.support_episode != boundary.support_episode
        or selected.support_frame != boundary.support_frame
    ):
        raise ValueError(
            "manifest source does not identify the selected real episode medoid"
        )
    if not np.allclose(
        selected.position_mm,
        boundary.nominal_position_mm,
        atol=1.0e-6,
        rtol=0.0,
    ):
        raise ValueError("manifest source position differs from medoid bank")
    quaternion_dot = abs(
        float(
            np.dot(
                selected.orientation_quat_xyzw,
                boundary.nominal_orientation_quat_xyzw,
            )
        )
    )
    if 1.0 - quaternion_dot > 1.0e-8:
        raise ValueError("manifest source orientation differs from medoid bank")
    if not np.allclose(
        selected.velocity_mm_s,
        boundary.nominal_velocity_mm_s,
        atol=1.0e-6,
        rtol=0.0,
    ):
        raise ValueError("manifest source velocity differs from medoid bank")
    episodes = bank.get("episodes")
    if not isinstance(episodes, list) or len(episodes) != execution_tail.episode_count:
        raise ValueError("runtime source bank does not contain all episodes")
    episode_ids = [int(item["episode"]) for item in episodes]
    if len(episode_ids) != len(set(episode_ids)):
        raise ValueError("runtime source bank contains duplicate episodes")
    return {
        "bank_path": str(bank_path),
        "selection_method": MEDOID_SELECTION_METHOD,
        "medoid_episode": selected.support_episode,
        "medoid_frame": selected.support_frame,
        "segment": selected.segment,
        "nominal_phase": float(execution_tail.nominal_phase),
        "phase_window": list(expected_window),
        "episode_count": len(episode_ids),
    }

def _resolved_successor_bank_path(
    manifest: EpisodeHandoffManifest,
    manifest_path: str | Path,
) -> Path:
    raw = manifest.runtime_successor_bank_path
    if raw is None:
        raise ValueError(
            "semantic-local Bridge manifest lacks successor support bank"
        )
    value = Path(raw).expanduser()
    if not value.is_absolute():
        value = Path(manifest_path).expanduser().resolve().parent / value
    return value.resolve()


def validate_runtime_successor_reference(
    manifest: EpisodeHandoffManifest,
    *,
    manifest_path: str | Path,
    successor_operator: Any,
) -> dict[str, Any]:
    """Validate an interior B reference without turning phase into authority."""

    bank_path = _resolved_successor_bank_path(manifest, manifest_path)
    if not bank_path.is_file():
        raise FileNotFoundError(
            f"runtime successor support bank not found: {bank_path}"
        )
    bank = json.loads(bank_path.read_text(encoding="utf-8"))
    if bank.get("schema_version") != RUNTIME_SOURCE_BANK_SCHEMA_VERSION:
        raise ValueError("unsupported runtime successor support bank schema")
    window = tuple(float(item) for item in bank.get("phase_window", ()))
    manifest_window = tuple(manifest.successor_reference_phase_window or ())
    checks = (
        (str(bank.get("role")) == "successor_entry", "bank role"),
        (str(bank.get("task")) == manifest.successor.task, "task"),
        (str(bank.get("segment")) == manifest.successor.segment, "segment"),
        (
            len(window) == 2
            and np.allclose(
                np.asarray(window),
                np.asarray(manifest_window),
                atol=1.0e-12,
                rtol=0.0,
            ),
            "phase window",
        ),
        (
            str(bank.get("semantic_local_selection", {}).get("method"))
            == manifest.successor_reference_selection_method
            == SEMANTIC_LOCAL_SELECTION_METHOD,
            "selection method",
        ),
        (
            bank.get("semantic_local_selection", {}).get(
                "runtime_phase_gate"
            )
            is False,
            "runtime phase authority",
        ),
    )
    for valid, label in checks:
        if not valid:
            raise ValueError(
                f"runtime successor support bank {label} disagrees with "
                "semantic-local manifest"
            )
    selected = PhaseIndexPoint.from_record(dict(bank["selected_boundary"]))
    boundary = manifest.successor
    if (
        selected.task != boundary.task
        or selected.segment != boundary.segment
        or abs(selected.phase - boundary.phase) > 1.0e-12
        or selected.support_episode != boundary.support_episode
        or selected.support_frame != boundary.support_frame
    ):
        raise ValueError(
            "manifest successor does not identify the selected real support"
        )
    if not np.allclose(
        selected.position_mm,
        boundary.nominal_position_mm,
        atol=1.0e-6,
        rtol=0.0,
    ):
        raise ValueError("manifest successor position differs from support bank")
    quaternion_dot = abs(
        float(
            np.dot(
                selected.orientation_quat_xyzw,
                boundary.nominal_orientation_quat_xyzw,
            )
        )
    )
    if 1.0 - quaternion_dot > 1.0e-8:
        raise ValueError(
            "manifest successor orientation differs from support bank"
        )
    if not np.allclose(
        selected.velocity_mm_s,
        boundary.nominal_velocity_mm_s,
        atol=1.0e-6,
        rtol=0.0,
    ):
        raise ValueError("manifest successor velocity differs from support bank")
    evidence = successor_operator.evidence
    phase_high = (
        evidence.exit_phase
        if evidence.exit_segment == evidence.entry_segment
        else 1.0
    )
    if (
        selected.segment != evidence.entry_segment
        or selected.phase < evidence.entry_phase - 1.0e-12
        or selected.phase > phase_high + 1.0e-12
    ):
        raise ValueError(
            "successor reference lies outside the successor semantic interval"
        )
    selection = dict(bank["semantic_local_selection"])
    margin = float(manifest.successor_interior_path_margin_mm)
    if (
        float(selection.get("path_margin_from_low_mm", -1.0))
        + 1.0e-9
        < margin
        or float(selection.get("path_margin_to_high_mm", -1.0))
        + 1.0e-9
        < margin
    ):
        raise ValueError(
            "successor reference violates the empirical interior path margin"
        )
    objective = dict(selection.get("semantic_local_objective", {}))
    expected_objective = {
        "source_prefix_mm": manifest.semantic_local_source_prefix_mm,
        "bridge_length_mm": manifest.semantic_local_bridge_length_mm,
        "successor_suffix_mm": manifest.semantic_local_successor_suffix_mm,
        "total_length_mm": manifest.semantic_local_total_length_mm,
    }
    for name, expected in expected_objective.items():
        if not np.isclose(
            float(objective.get(name, np.nan)),
            float(expected),
            atol=1.0e-6,
            rtol=1.0e-9,
        ):
            raise ValueError(
                f"successor reference objective {name} differs from manifest"
            )
    episodes = bank.get("episodes")
    if not isinstance(episodes, list) or not episodes:
        raise ValueError("runtime successor bank contains no episode support")
    episode_ids = [int(item["episode"]) for item in episodes]
    if len(episode_ids) != len(set(episode_ids)):
        raise ValueError("runtime successor bank contains duplicate episodes")
    return {
        "bank_path": str(bank_path),
        "selection_method": SEMANTIC_LOCAL_SELECTION_METHOD,
        "support_episode": selected.support_episode,
        "support_frame": selected.support_frame,
        "segment": selected.segment,
        "reference_phase": selected.phase,
        "phase_window": list(window),
        "episode_count": len(episode_ids),
        "interior_path_margin_mm": margin,
        "path_margin_from_low_mm": float(
            selection["path_margin_from_low_mm"]
        ),
        "path_margin_to_high_mm": float(
            selection["path_margin_to_high_mm"]
        ),
        "semantic_local_objective": expected_objective,
        "runtime_phase_gate": False,
        "authority": "bridge_reference_and_metadata_only",
        "validated_runtime_successor_bank": True,
    }
