"""Read-only T1--T8 artifact audit and interior operator catalog loader."""

from __future__ import annotations

import hashlib
import json
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .contracts import (
    POLICY_IDS,
    InteriorPolicyOperator,
    SemanticIntervalEvidence,
    WorldState,
)
from .planner import PlannerCostConfig


CATALOG_SCHEMA_VERSION = "a0509.interior_policy_catalog.v1"


def discover_repository_root(start: Path | None = None) -> Path:
    current = (start or Path(__file__)).resolve()
    for candidate in (current, *current.parents):
        if (candidate / "offline_tools").is_dir() and (candidate / "config").is_dir():
            return candidate
    raise FileNotFoundError("could not locate Dosan-MetaQuest repository root")


def _resolve(repo_root: Path, value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = repo_root / path
    return path.resolve()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while True:
            chunk = handle.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _declared_checksum(path: Path) -> str | None:
    checksums = path.parent / "checksums.sha256"
    if not checksums.is_file():
        return None
    for line in checksums.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) >= 2 and fields[-1].lstrip("*") == path.name:
            return fields[0]
    return None


@dataclass(frozen=True)
class PhysicalSetupOverride:
    override_id: str
    task: str
    artifact_field: str
    artifact_value: str
    planner_field: str
    planner_value: str
    authority: str
    mutates_source_artifact: bool

    def to_record(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class TaskArtifactAudit:
    task_id: str
    dataset_root: str
    dataset_task_description: str
    semantic_task_description: str
    episode_count: int
    total_frames: int
    fps: int
    semantic_artifact: str
    phase_support_artifact: str
    checkpoint: str
    checkpoint_type: str
    chunk_size: int
    n_action_steps: int
    input_contract: Mapping[str, tuple[int, ...]]
    action_shape: tuple[int, ...]
    semantic_segments: tuple[str, ...]
    event_grammar: tuple[str, ...]
    representative_kind: str
    representative_episode_selected: bool
    semantic_robot_executable: bool
    semantic_sha256_verified: bool | None
    phase_support_sha256_verified: bool | None

    def to_record(self) -> dict[str, Any]:
        value = asdict(self)
        value["input_contract"] = {
            key: list(shape) for key, shape in self.input_contract.items()
        }
        value["action_shape"] = list(self.action_shape)
        return value


@dataclass(frozen=True)
class FrameIntervalSummary:
    episode_count: int
    start_frame_min: int
    start_frame_median: float
    start_frame_max: int
    end_frame_min: int
    end_frame_median: float
    end_frame_max: int
    interval_frames_min: int
    interval_frames_median: float
    interval_frames_max: int

    def to_record(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class OperatorArtifactAudit:
    operator_id: str
    semantic_labels: tuple[str, ...]
    parent_subgoals: tuple[str, ...]
    artifact_entry_state: Mapping[str, str]
    artifact_exit_state_at_selected_boundary: Mapping[str, str]
    gripper_semantics: tuple[str, ...]
    phase_support_episode_counts: Mapping[str, int]
    frame_interval: FrameIntervalSummary | None
    certified_source_boundary_verified: bool
    evidence_limitations: tuple[str, ...]

    def to_record(self) -> dict[str, Any]:
        value = asdict(self)
        if self.frame_interval is not None:
            value["frame_interval"] = self.frame_interval.to_record()
        return value


@dataclass(frozen=True)
class CatalogAudit:
    tasks: Mapping[str, TaskArtifactAudit]
    operators: Mapping[str, OperatorArtifactAudit]
    physical_setup_overrides: tuple[PhysicalSetupOverride, ...]
    full_raw_frame_audit: bool

    def to_record(self) -> dict[str, Any]:
        return {
            "tasks": {key: value.to_record() for key, value in self.tasks.items()},
            "operators": {
                key: value.to_record() for key, value in self.operators.items()
            },
            "physical_setup_overrides": [
                item.to_record() for item in self.physical_setup_overrides
            ],
            "full_raw_frame_audit": self.full_raw_frame_audit,
        }


@dataclass(frozen=True)
class TargetExperiment:
    initial_state: WorldState
    goal: Mapping[str, str]
    controlled_operator_whitelist: tuple[str, ...]
    cost_config: PlannerCostConfig


@dataclass(frozen=True)
class LoadedOperatorCatalog:
    catalog_id: str
    operators: tuple[InteriorPolicyOperator, ...]
    target: TargetExperiment
    audit: CatalogAudit
    config_path: Path
    repository_root: Path

    @property
    def by_id(self) -> dict[str, InteriorPolicyOperator]:
        return {item.id: item for item in self.operators}

    @property
    def controlled_operators(self) -> tuple[InteriorPolicyOperator, ...]:
        mapping = self.by_id
        return tuple(mapping[item] for item in self.target.controlled_operator_whitelist)


def _checksum_verified(path: Path) -> bool | None:
    expected = _declared_checksum(path)
    return None if expected is None else _sha256(path) == expected


def _load_task_audit(
    task_id: str,
    value: Mapping[str, Any],
    *,
    repo_root: Path,
) -> tuple[TaskArtifactAudit, dict[str, Any], dict[str, np.ndarray]]:
    import pyarrow.parquet as pq

    dataset_root = _resolve(repo_root, str(value["dataset_root"]))
    semantic_path = _resolve(repo_root, str(value["semantic_artifact"]))
    phase_path = _resolve(repo_root, str(value["phase_support_artifact"]))
    checkpoint = _resolve(repo_root, str(value["checkpoint"]))
    required = (
        dataset_root / "meta/info.json",
        dataset_root / "meta/tasks.parquet",
        semantic_path,
        phase_path,
        checkpoint / "config.json",
        checkpoint / "model.safetensors",
    )
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError("catalog evidence is missing: " + ", ".join(missing))

    info = json.loads((dataset_root / "meta/info.json").read_text(encoding="utf-8"))
    task_rows = pq.read_table(dataset_root / "meta/tasks.parquet").to_pylist()
    descriptions = tuple(str(row["task"]) for row in task_rows)
    expected_description = str(value["task_description"])
    if descriptions != (expected_description,):
        raise ValueError(
            f"{task_id} tasks.parquet differs from catalog: {descriptions!r}"
        )
    expected_episodes = int(value["expected_episode_count"])
    if int(info["total_episodes"]) != expected_episodes:
        raise ValueError(f"{task_id} dataset episode count changed")

    graph = json.loads(semantic_path.read_text(encoding="utf-8"))
    if graph.get("schema_version") != "a0509.semantic_only_graph.v3":
        raise ValueError(f"{task_id} is not a semantic-only v3 artifact")
    if str(graph.get("task_description")) != expected_description:
        raise ValueError(f"{task_id} semantic task description changed")
    if int(graph.get("episode_count", -1)) != expected_episodes:
        raise ValueError(f"{task_id} semantic episode count changed")
    graph_task = graph.get("task_id")
    if graph_task is not None and str(graph_task).upper() != task_id:
        raise ValueError(f"{task_id} semantic artifact task_id mismatch")

    phase_npz = np.load(phase_path, allow_pickle=False)
    arrays = {key: np.asarray(phase_npz[key]) for key in phase_npz.files}
    segment_ids = tuple(str(item["spec"]["segment_id"]) for item in graph["segments"])
    for segment in segment_ids:
        episode_key = f"{segment}_episode_ids"
        xyz_key = f"{segment}_episode_xyz_mm"
        if episode_key not in arrays or xyz_key not in arrays:
            raise ValueError(f"{task_id} phase support lacks {segment}")
        if arrays[episode_key].shape != (expected_episodes,):
            raise ValueError(f"{task_id} {segment} support episode count changed")
        if arrays[xyz_key].shape != (expected_episodes, 101, 3):
            raise ValueError(f"{task_id} {segment} support shape changed")

    policy_config = json.loads((checkpoint / "config.json").read_text(encoding="utf-8"))
    input_contract = {
        str(key): tuple(int(item) for item in feature["shape"])
        for key, feature in policy_config["input_features"].items()
    }
    output_shape = tuple(int(item) for item in policy_config["output_features"]["action"]["shape"])
    if policy_config.get("type") != "act" or output_shape != (7,):
        raise ValueError(f"{task_id} checkpoint is not the expected 7D ACT")
    if int(policy_config.get("chunk_size", -1)) != 100:
        raise ValueError(f"{task_id} ACT chunk_size is not 100")
    if int(policy_config.get("n_action_steps", -1)) != 100:
        raise ValueError(f"{task_id} ACT n_action_steps is not 100")

    representative = dict(graph.get("representative", {}))
    scope = dict(graph.get("scope", {}))
    audit = TaskArtifactAudit(
        task_id=task_id,
        dataset_root=str(dataset_root),
        dataset_task_description=descriptions[0],
        semantic_task_description=str(graph["task_description"]),
        episode_count=expected_episodes,
        total_frames=int(info["total_frames"]),
        fps=int(info["fps"]),
        semantic_artifact=str(semantic_path),
        phase_support_artifact=str(phase_path),
        checkpoint=str(checkpoint),
        checkpoint_type=str(policy_config["type"]),
        chunk_size=int(policy_config["chunk_size"]),
        n_action_steps=int(policy_config["n_action_steps"]),
        input_contract=input_contract,
        action_shape=output_shape,
        semantic_segments=segment_ids,
        event_grammar=tuple(str(item) for item in graph["event_grammar"]),
        representative_kind=str(representative.get("synthetic", "unknown")),
        representative_episode_selected=bool(
            representative.get("individual_episode_selected", False)
        ),
        semantic_robot_executable=bool(scope.get("robot_executable", False)),
        semantic_sha256_verified=_checksum_verified(semantic_path),
        phase_support_sha256_verified=_checksum_verified(phase_path),
    )
    return audit, graph, arrays


def _segment_mapping(graph: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    return {
        str(item["spec"]["segment_id"]): item for item in graph["segments"]
    }


def _boundary_state(segment: Mapping[str, Any], phase: float) -> dict[str, str]:
    spec = segment["spec"]
    # Semantic events occur at segment endpoints.  An interior phase retains
    # the entry-side discrete state; the standardized artifact has no invented
    # intermediate object-state transition.
    state = spec["exit_state"] if phase >= 1.0 - 1e-12 else spec["entry_state"]
    return {str(key): str(value) for key, value in state.items()}


def _phase_index(
    xyz_mm: np.ndarray,
    start_index: int,
    end_index: int,
    phase: float,
) -> int:
    if phase <= 0.0:
        return start_index
    if phase >= 1.0:
        return end_index
    xyz = np.asarray(xyz_mm[start_index : end_index + 1], dtype=np.float64)
    arc = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))))
    if arc[-1] <= 1e-9:
        raise ValueError("operator boundary segment has zero Cartesian arc length")
    normalized = arc / arc[-1]
    return start_index + int(np.argmin(np.abs(normalized - phase)))


def _raw_frame_summary(
    *,
    trajectories: list[Any],
    graph: Mapping[str, Any],
    entry_segment: str,
    entry_phase: float,
    exit_segment: str,
    exit_phase: float,
) -> FrameIntervalSummary:
    from offline_tools.semantic_segmentation.semantic_phase_core import (
        detect_gripper_event_sequence,
        resolve_anchor_index,
    )

    segments = _segment_mapping(graph)
    expected_events = tuple(str(item) for item in graph["event_grammar"])
    starts: list[int] = []
    ends: list[int] = []
    lengths: list[int] = []
    for trajectory in trajectories:
        events = detect_gripper_event_sequence(trajectory, expected_events)
        entry_spec = segments[entry_segment]["spec"]
        exit_spec = segments[exit_segment]["spec"]
        entry_start = resolve_anchor_index(entry_spec["start_anchor"], trajectory, events)
        entry_end = resolve_anchor_index(entry_spec["end_anchor"], trajectory, events)
        exit_start = resolve_anchor_index(exit_spec["start_anchor"], trajectory, events)
        exit_end = resolve_anchor_index(exit_spec["end_anchor"], trajectory, events)
        start_index = _phase_index(
            trajectory.xyz_mm, entry_start, entry_end, entry_phase
        )
        end_index = _phase_index(
            trajectory.xyz_mm, exit_start, exit_end, exit_phase
        )
        if start_index >= end_index:
            raise ValueError("operator interval does not make forward frame progress")
        start_frame = int(trajectory.frame_index[start_index])
        end_frame = int(trajectory.frame_index[end_index])
        starts.append(start_frame)
        ends.append(end_frame)
        lengths.append(end_frame - start_frame + 1)
    return FrameIntervalSummary(
        episode_count=len(trajectories),
        start_frame_min=min(starts),
        start_frame_median=float(statistics.median(starts)),
        start_frame_max=max(starts),
        end_frame_min=min(ends),
        end_frame_median=float(statistics.median(ends)),
        end_frame_max=max(ends),
        interval_frames_min=min(lengths),
        interval_frames_median=float(statistics.median(lengths)),
        interval_frames_max=max(lengths),
    )


def _verify_certified_boundary(
    path: Path | None,
    *,
    policy_id: str,
    segment: str,
    phase: float,
) -> bool:
    if path is None:
        return False
    if not path.is_file():
        raise FileNotFoundError(f"certified source boundary manifest missing: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    source = value.get("source", {})
    validation = value.get("validation", {})
    if str(source.get("task", "")).upper() != policy_id:
        raise ValueError("certified source boundary policy mismatch")
    if str(source.get("segment")) != segment:
        raise ValueError("certified source boundary segment mismatch")
    if abs(float(source.get("phase")) - phase) > 1e-9:
        raise ValueError("certified source boundary phase mismatch")
    return bool(validation.get("hard_filter_passed", False))


def load_operator_catalog(
    *,
    repository_root: Path | None = None,
    config_path: Path | None = None,
    audit_raw_frames: bool = False,
) -> LoadedOperatorCatalog:
    repo_root = (repository_root or discover_repository_root()).resolve()
    source = (
        config_path.resolve()
        if config_path is not None
        else repo_root / "config/interior_policy/a0509_t1_t8_operator_catalog_spatial_v2.json"
    )
    value = json.loads(source.read_text(encoding="utf-8"))
    if value.get("schema_version") != CATALOG_SCHEMA_VERSION:
        raise ValueError("unsupported interior policy catalog schema")

    overrides = tuple(
        PhysicalSetupOverride(**dict(item))
        for item in value.get("physical_setup_overrides", ())
    )
    override_ids = {item.override_id for item in overrides}
    task_audits: dict[str, TaskArtifactAudit] = {}
    graphs: dict[str, dict[str, Any]] = {}
    arrays_by_task: dict[str, dict[str, np.ndarray]] = {}
    trajectories_by_task: dict[str, list[Any]] = {}
    task_values = dict(value["tasks"])
    unknown_tasks = set(task_values) - set(POLICY_IDS)
    if unknown_tasks:
        raise ValueError(
            "catalog contains unsupported task ids: "
            + ", ".join(sorted(unknown_tasks))
        )
    for task_id in POLICY_IDS:
        if task_id not in task_values:
            continue
        audit, graph, arrays = _load_task_audit(
            task_id, dict(task_values[task_id]), repo_root=repo_root
        )
        task_audits[task_id] = audit
        graphs[task_id] = graph
        arrays_by_task[task_id] = arrays
        if audit_raw_frames:
            from offline_tools.task_c_bridge_v0.dataset_io import (
                load_lerobot_trajectories,
            )

            trajectories, _ = load_lerobot_trajectories(
                Path(audit.dataset_root), task_id.lower()
            )
            trajectories_by_task[task_id] = trajectories

    operators: list[InteriorPolicyOperator] = []
    operator_audits: dict[str, OperatorArtifactAudit] = {}
    for raw in value["operators"]:
        item = dict(raw)
        policy_id = str(item["policy_id"])
        task_audit = task_audits[policy_id]
        graph = graphs[policy_id]
        segment_map = _segment_mapping(graph)
        segment_ids = tuple(str(segment) for segment in item["semantic_segment_ids"])
        graph_order = tuple(segment_map)
        indices = tuple(graph_order.index(segment) for segment in segment_ids)
        if indices != tuple(range(indices[0], indices[-1] + 1)):
            raise ValueError(f"{item['id']} semantic segments are not contiguous")
        entry = dict(item["entry_boundary"])
        exit_value = dict(item["exit_boundary"])
        if str(entry["segment"]) != segment_ids[0]:
            raise ValueError(f"{item['id']} entry is not the first interval segment")
        if str(exit_value["segment"]) != segment_ids[-1]:
            raise ValueError(f"{item['id']} exit is not the last interval segment")

        physical_overrides = tuple(
            str(override) for override in item.get("physical_setup_overrides", ())
        )
        unknown_overrides = set(physical_overrides) - override_ids
        if unknown_overrides:
            raise ValueError(f"{item['id']} refers to unknown physical override")
        certified_value = item.get("certified_source_boundary_manifest")
        certified_path = (
            None
            if certified_value is None
            else _resolve(repo_root, str(certified_value))
        )
        evidence = SemanticIntervalEvidence(
            dataset_root=task_audit.dataset_root,
            semantic_artifact=task_audit.semantic_artifact,
            phase_support_artifact=task_audit.phase_support_artifact,
            checkpoint=task_audit.checkpoint,
            semantic_segment_ids=segment_ids,
            entry_segment=str(entry["segment"]),
            entry_phase=float(entry["phase"]),
            entry_anchor=str(entry["anchor"]),
            exit_segment=str(exit_value["segment"]),
            exit_phase=float(exit_value["phase"]),
            exit_anchor=str(exit_value["anchor"]),
            artifact_relaxations=tuple(
                str(text) for text in item.get("artifact_relaxations", ())
            ),
            held_object_aliases=tuple(
                sorted(
                    (
                        str(raw),
                        str(canonical),
                    )
                    for raw, canonical in dict(
                        item.get("held_object_aliases", {})
                    ).items()
                )
            ),
            physical_setup_overrides=physical_overrides,
            certified_source_boundary_manifest=(
                None if certified_path is None else str(certified_path)
            ),
        )
        operator = InteriorPolicyOperator.create(
            id=str(item["id"]),
            policy_id=policy_id,
            entry_order=int(item["entry_order"]),
            exit_order=int(item["exit_order"]),
            preconditions=dict(item["preconditions"]),
            effects=dict(item["effects"]),
            entry_contact_mode=str(item["entry_contact_mode"]),
            exit_contact_mode=str(item["exit_contact_mode"]),
            bridge_mode=item.get("bridge_mode"),
            base_cost=float(item["base_cost"]),
            evidence=evidence,
        )
        operators.append(operator)

        arrays = arrays_by_task[policy_id]
        support_counts = {
            segment: int(arrays[f"{segment}_episode_ids"].shape[0])
            for segment in segment_ids
        }
        selected_segments = [segment_map[segment]["spec"] for segment in segment_ids]
        limitations = list(evidence.artifact_relaxations)
        if physical_overrides:
            limitations.append(
                "physical setup override is planner metadata, not a source artifact edit"
            )
        certified = _verify_certified_boundary(
            certified_path,
            policy_id=policy_id,
            segment=evidence.exit_segment,
            phase=evidence.exit_phase,
        )
        if not certified:
            limitations.append("selected exit is not an existing hard-filter-passed V2 source boundary")
        frame_summary = None
        if audit_raw_frames:
            frame_summary = _raw_frame_summary(
                trajectories=trajectories_by_task[policy_id],
                graph=graph,
                entry_segment=evidence.entry_segment,
                entry_phase=evidence.entry_phase,
                exit_segment=evidence.exit_segment,
                exit_phase=evidence.exit_phase,
            )
        operator_audits[operator.id] = OperatorArtifactAudit(
            operator_id=operator.id,
            semantic_labels=tuple(str(spec["semantic_label"]) for spec in selected_segments),
            parent_subgoals=tuple(
                dict.fromkeys(str(spec["parent_subgoal"]) for spec in selected_segments)
            ),
            artifact_entry_state=_boundary_state(
                segment_map[evidence.entry_segment], evidence.entry_phase
            ),
            artifact_exit_state_at_selected_boundary=_boundary_state(
                segment_map[evidence.exit_segment], evidence.exit_phase
            ),
            gripper_semantics=tuple(
                str(spec["gripper_semantics"]) for spec in selected_segments
            ),
            phase_support_episode_counts=support_counts,
            frame_interval=frame_summary,
            certified_source_boundary_verified=certified,
            evidence_limitations=tuple(limitations),
        )

    target_value = dict(value["target_experiment"])
    initial = WorldState(**dict(target_value["initial_state"]))
    goal = {str(key): str(item) for key, item in dict(target_value["goal"]).items()}
    cost_value = dict(target_value["cost"])
    target = TargetExperiment(
        initial_state=initial,
        goal=goal,
        controlled_operator_whitelist=tuple(
            str(item) for item in target_value["controlled_operator_whitelist"]
        ),
        cost_config=PlannerCostConfig(
            policy_switch_penalty=float(cost_value["policy_switch_penalty"]),
            same_policy_continuation_penalty=float(
                cost_value["same_policy_continuation_penalty"]
            ),
        ),
    )
    operator_ids = {item.id for item in operators}
    if len(operator_ids) != len(operators):
        raise ValueError("operator catalog contains duplicate identifiers")
    if not set(target.controlled_operator_whitelist).issubset(operator_ids):
        raise ValueError("controlled whitelist refers to unknown operators")
    audit = CatalogAudit(
        tasks=task_audits,
        operators=operator_audits,
        physical_setup_overrides=overrides,
        full_raw_frame_audit=audit_raw_frames,
    )
    return LoadedOperatorCatalog(
        catalog_id=str(value["catalog_id"]),
        operators=tuple(operators),
        target=target,
        audit=audit,
        config_path=source,
        repository_root=repo_root,
    )
