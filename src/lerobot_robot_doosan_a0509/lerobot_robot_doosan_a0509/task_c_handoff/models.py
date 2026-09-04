"""Validated configuration and episode manifest models for Task-C V2."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from offline_tools.task_c_bridge_v0.bezier_bridge import (
    CUBIC_BEZIER_FIXED,
    CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
)
from offline_tools.cross_task_handoff.authority import (
    SemanticAuthority,
    parse_semantic_authority,
)
from quest_a0509_teleop.doosan_orientation import (
    doosan_zyz_deg_to_quaternion,
)


def _finite_vector(value: Any, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite shape ({size},)")
    return result.copy()


def _unit_quaternion(value: Any, name: str) -> np.ndarray:
    quaternion = _finite_vector(value, 4, name)
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError(f"{name} must have non-zero norm")
    return quaternion / norm


class HandoffMode(str, Enum):
    ENDPOINT_V1 = "endpoint_v1"
    ASYNC_WINDOW_V2 = "async_window_v2"


class BridgeAdmissionMode(str, Enum):
    """Offline/runtime authority used to admit a Task-C Bridge edge."""

    STRICT_LEVEL2 = "strict_level2"
    FLEXIBLE_LEVEL2 = "flexible_level2"


class HandoffV2State(str, Enum):
    WAITING = "WAITING"
    LOAD_POLICIES = "LOAD_POLICIES"
    RUN_A = "RUN_A"
    A_EXIT_COMMIT = "A_EXIT_COMMIT"
    PREPARE_BRIDGE = "PREPARE_BRIDGE"
    RUN_BRIDGE = "RUN_BRIDGE"
    HANDOFF_WINDOW = "HANDOFF_WINDOW"
    B_SHADOW_PENDING = "B_SHADOW_PENDING"
    B_PREFIX_ADMISSION = "B_PREFIX_ADMISSION"
    SOFT_HANDOFF = "SOFT_HANDOFF"
    RUN_B = "RUN_B"
    ENDPOINT_FALLBACK = "ENDPOINT_FALLBACK"
    COMPLETE = "COMPLETE"
    FAILED_HOLD = "FAILED_HOLD"


@dataclass(frozen=True)
class BoundarySemanticState:
    gripper_state: str
    held_object: str
    contact_mode: str
    semantic_state: str = ""
    entry_preconditions: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        gripper = str(self.gripper_state).strip().lower()
        if gripper not in {"open", "closed"}:
            raise ValueError("gripper_state must be open or closed")
        held = str(self.held_object).strip()
        contact = str(self.contact_mode).strip()
        if not held or not contact:
            raise ValueError("held_object and contact_mode must not be empty")
        object.__setattr__(self, "gripper_state", gripper)
        object.__setattr__(self, "held_object", held)
        object.__setattr__(self, "contact_mode", contact)
        object.__setattr__(
            self,
            "entry_preconditions",
            tuple(str(item) for item in self.entry_preconditions),
        )

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "BoundarySemanticState":
        gripper = value.get("gripper_state", value.get("gripper"))
        held = value.get("held_object", "none")
        contact = value.get("contact_mode", "unknown")
        return cls(
            gripper_state=str(gripper),
            held_object=str(held),
            contact_mode=str(contact),
            semantic_state=str(value.get("semantic_state", "")),
            entry_preconditions=tuple(value.get("entry_preconditions", ())),
        )

    @property
    def gripper_target(self) -> float:
        return 1.0 if self.gripper_state == "closed" else 0.0


@dataclass(frozen=True)
class HandoffBoundary:
    task: str
    segment: str
    phase: float
    support_episode: int
    support_frame: int
    nominal_position_mm: np.ndarray
    nominal_orientation_quat_xyzw: np.ndarray
    nominal_velocity_mm_s: np.ndarray
    semantic: BoundarySemanticState
    support_radius_mm: float
    support_orientation_radius_deg: float = 30.0
    prearm_radius_mm: float = 40.0
    commit_radius_mm: float = 20.0
    direction_cosine_minimum: float = 0.7
    approach_frames: int = 3
    commit_stable_frames: int = 3

    def __post_init__(self) -> None:
        if not self.task or not self.segment:
            raise ValueError("boundary task and segment must not be empty")
        if not 0.0 <= float(self.phase) <= 1.0:
            raise ValueError("boundary phase must be in [0, 1]")
        if self.support_episode < 0 or self.support_frame < 0:
            raise ValueError(
                "support_episode and support_frame must be non-negative"
            )
        if not np.isfinite(self.support_radius_mm) or self.support_radius_mm <= 0.0:
            raise ValueError("support_radius_mm must be positive")
        if (
            not np.isfinite(self.support_orientation_radius_deg)
            or not 0.0 < self.support_orientation_radius_deg <= 180.0
        ):
            raise ValueError(
                "support_orientation_radius_deg must be in (0, 180]"
            )
        if not 0.0 < self.commit_radius_mm < self.prearm_radius_mm:
            raise ValueError("boundary radii must satisfy 0 < commit < prearm")
        if not -1.0 <= self.direction_cosine_minimum <= 1.0:
            raise ValueError("direction_cosine_minimum must be in [-1, 1]")
        if self.approach_frames < 1 or self.commit_stable_frames < 1:
            raise ValueError("boundary persistence frames must be positive")
        object.__setattr__(
            self,
            "nominal_position_mm",
            _finite_vector(self.nominal_position_mm, 3, "nominal position"),
        )
        object.__setattr__(
            self,
            "nominal_orientation_quat_xyzw",
            _unit_quaternion(
                self.nominal_orientation_quat_xyzw,
                "nominal orientation quaternion",
            ),
        )
        object.__setattr__(
            self,
            "nominal_velocity_mm_s",
            _finite_vector(self.nominal_velocity_mm_s, 3, "nominal velocity"),
        )

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "HandoffBoundary":
        pose = dict(value.get("nominal_pose", {}))
        position = pose.get(
            "position_mm",
            value.get("nominal_position_mm"),
        )
        quaternion = pose.get(
            "orientation_quat_xyzw",
            value.get("nominal_orientation_quat_xyzw"),
        )
        legacy_pose = value.get("nominal_pose_mm_deg")
        if legacy_pose is not None:
            legacy = _finite_vector(legacy_pose, 6, "nominal_pose_mm_deg")
            if position is None:
                position = legacy[:3]
            if quaternion is None:
                quaternion = doosan_zyz_deg_to_quaternion(legacy[3:6])
        if position is None or quaternion is None:
            raise ValueError(
                "boundary nominal pose requires position_mm and quaternion"
            )
        semantic = BoundarySemanticState.from_mapping(
            dict(value.get("semantic", value.get("semantic_state", {})))
        )
        return cls(
            task=str(value["task"]),
            segment=str(value["segment"]),
            phase=float(value["phase"]),
            support_episode=int(value["support_episode"]),
            support_frame=int(value["support_frame"]),
            nominal_position_mm=position,
            nominal_orientation_quat_xyzw=quaternion,
            nominal_velocity_mm_s=value["nominal_velocity_mm_s"],
            semantic=semantic,
            support_radius_mm=float(value.get("support_radius_mm", 90.0)),
            support_orientation_radius_deg=float(
                value.get("support_orientation_radius_deg", 30.0)
            ),
            prearm_radius_mm=float(value.get("prearm_radius_mm", 40.0)),
            commit_radius_mm=float(value.get("commit_radius_mm", 20.0)),
            direction_cosine_minimum=float(
                value.get("direction_cosine_minimum", 0.7)
            ),
            approach_frames=int(value.get("approach_frames", 3)),
            commit_stable_frames=int(value.get("commit_stable_frames", 3)),
        )

    def to_record(self) -> dict[str, Any]:
        return {
            "task": self.task,
            "segment": self.segment,
            "phase": self.phase,
            "support_episode": self.support_episode,
            "support_frame": self.support_frame,
            "nominal_pose": {
                "position_mm": self.nominal_position_mm.tolist(),
                "orientation_quat_xyzw": (
                    self.nominal_orientation_quat_xyzw.tolist()
                ),
            },
            "nominal_velocity_mm_s": self.nominal_velocity_mm_s.tolist(),
            "semantic": asdict(self.semantic),
            "support_radius_mm": self.support_radius_mm,
            "support_orientation_radius_deg": (
                self.support_orientation_radius_deg
            ),
            "prearm_radius_mm": self.prearm_radius_mm,
            "commit_radius_mm": self.commit_radius_mm,
            "direction_cosine_minimum": self.direction_cosine_minimum,
            "approach_frames": self.approach_frames,
            "commit_stable_frames": self.commit_stable_frames,
        }


@dataclass(frozen=True)
class EpisodeHandoffManifest:
    composition_id: str
    handoff_id: str
    source: HandoffBoundary
    successor: HandoffBoundary
    bridge_duration_s: float
    nominal_length_mm: float | None
    generation_method: str
    bridge_admission_mode: BridgeAdmissionMode = BridgeAdmissionMode.STRICT_LEVEL2
    # A flexible reference may originate from the best semantic candidate that
    # failed the legacy global-curvature gate. It is guidance only and never a
    # claim that the stored curve is robot executable.
    reference_only: bool = False
    bridge_algorithm: str = CUBIC_BEZIER_FIXED
    minimum_tangent_handle_chord_ratio: float = 0.0
    maximum_endpoint_speed_adjustment_mm_s: float | None = None
    semantic_authority: SemanticAuthority = SemanticAuthority.RUNTIME_GUARDED
    semantic_diagnostics: tuple[str, ...] = ()
    hard_filter_passed: bool = False
    robot_executable: bool = False
    dry_run_only: bool = True
    transport_floor_mm: float | None = None
    ik_checked: bool = False
    collision_checked: bool = False
    # Optional FLEXIBLE_LEVEL2 provenance for a physical source boundary that
    # lies in a zero-symbolic-cost execution tail. The symbolic operator exit
    # remains in the planner; source is the real-episode medoid used by the
    # runtime Bridge reference. The compiler validates this sidecar before it
    # admits the edge.
    runtime_source_bank_path: str | None = None
    source_reference_selection_method: str | None = None
    source_reference_phase_window: tuple[float, float] | None = None
    # Optional successor-side reference selected inside the semantic interval.
    # This is a Bridge attractor/audit record only. It never grants ACT-B
    # control authority; fresh ACT-B prefix admission remains the authority.
    runtime_successor_bank_path: str | None = None
    successor_reference_selection_method: str | None = None
    successor_reference_phase_window: tuple[float, float] | None = None
    successor_interior_path_margin_mm: float | None = None
    semantic_local_source_prefix_mm: float | None = None
    semantic_local_bridge_length_mm: float | None = None
    semantic_local_successor_suffix_mm: float | None = None
    semantic_local_total_length_mm: float | None = None
    schema_version: str = "a0509.task_c_handoff_episode.v2"

    def __post_init__(self) -> None:
        try:
            admission_mode = BridgeAdmissionMode(self.bridge_admission_mode)
        except ValueError as exc:
            raise ValueError("unsupported Bridge admission mode") from exc
        authority = parse_semantic_authority(self.semantic_authority)
        diagnostics = tuple(
            dict.fromkeys(str(item) for item in self.semantic_diagnostics)
        )
        object.__setattr__(self, "semantic_authority", authority)
        object.__setattr__(self, "bridge_admission_mode", admission_mode)
        object.__setattr__(self, "semantic_diagnostics", diagnostics)
        if self.schema_version != "a0509.task_c_handoff_episode.v2":
            raise ValueError("unsupported Task-C V2 episode manifest schema")
        if not self.composition_id or not self.handoff_id:
            raise ValueError("composition_id and handoff_id must not be empty")
        if not np.isfinite(self.bridge_duration_s) or self.bridge_duration_s <= 0.0:
            raise ValueError("bridge duration must be positive")
        if self.nominal_length_mm is not None and (
            not np.isfinite(self.nominal_length_mm) or self.nominal_length_mm < 0.0
        ):
            raise ValueError("nominal bridge length must be non-negative")
        if self.transport_floor_mm is not None and not np.isfinite(
            self.transport_floor_mm
        ):
            raise ValueError("transport floor must be finite")
        for name in (
            "reference_only",
            "hard_filter_passed",
            "robot_executable",
            "dry_run_only",
            "ik_checked",
            "collision_checked",
        ):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f"{name} must be boolean")
        if admission_mode is BridgeAdmissionMode.STRICT_LEVEL2:
            if not self.hard_filter_passed:
                raise ValueError(
                    "STRICT_LEVEL2 manifest requires hard_filter_passed=true"
                )
            if self.reference_only:
                raise ValueError("STRICT_LEVEL2 cannot use reference_only")
        elif not self.hard_filter_passed and not self.reference_only:
            raise ValueError(
                "FLEXIBLE_LEVEL2 rejected references require reference_only=true"
            )
        if self.robot_executable or not self.dry_run_only:
            raise ValueError(
                "initial V2 accepts only uncertified dry-run-only manifests"
            )
        reference_values = (
            self.runtime_source_bank_path,
            self.source_reference_selection_method,
            self.source_reference_phase_window,
        )
        if any(value is not None for value in reference_values):
            if any(value is None for value in reference_values):
                raise ValueError(
                    "runtime source reference requires bank path, selection "
                    "method, and phase window"
                )
            if admission_mode is not BridgeAdmissionMode.FLEXIBLE_LEVEL2:
                raise ValueError(
                    "runtime source reference is available only in "
                    "FLEXIBLE_LEVEL2"
                )
            bank_path = str(self.runtime_source_bank_path).strip()
            selection = str(self.source_reference_selection_method).strip()
            window = tuple(
                float(item) for item in self.source_reference_phase_window
            )
            if not bank_path or not selection:
                raise ValueError(
                    "runtime source reference path/method must not be empty"
                )
            if len(window) != 2 or not (
                0.0 <= window[0] <= self.source.phase <= window[1] <= 1.0
            ):
                raise ValueError(
                    "runtime source reference window must contain source phase"
                )
            object.__setattr__(self, "runtime_source_bank_path", bank_path)
            object.__setattr__(
                self, "source_reference_selection_method", selection
            )
            object.__setattr__(
                self, "source_reference_phase_window", window
            )
        successor_reference_values = (
            self.runtime_successor_bank_path,
            self.successor_reference_selection_method,
            self.successor_reference_phase_window,
            self.successor_interior_path_margin_mm,
        )
        semantic_local_values = (
            self.semantic_local_source_prefix_mm,
            self.semantic_local_bridge_length_mm,
            self.semantic_local_successor_suffix_mm,
            self.semantic_local_total_length_mm,
        )
        if any(value is not None for value in successor_reference_values):
            if any(value is None for value in successor_reference_values):
                raise ValueError(
                    "runtime successor reference requires bank path, "
                    "selection method, phase window, and interior margin"
                )
            if admission_mode is not BridgeAdmissionMode.FLEXIBLE_LEVEL2:
                raise ValueError(
                    "runtime successor reference is available only in "
                    "FLEXIBLE_LEVEL2"
                )
            bank_path = str(self.runtime_successor_bank_path).strip()
            selection = str(
                self.successor_reference_selection_method
            ).strip()
            window = tuple(
                float(item) for item in self.successor_reference_phase_window
            )
            margin = float(self.successor_interior_path_margin_mm)
            if not bank_path or not selection:
                raise ValueError(
                    "runtime successor reference path/method must not be empty"
                )
            if len(window) != 2 or not (
                0.0 <= window[0] <= self.successor.phase <= window[1] <= 1.0
            ):
                raise ValueError(
                    "runtime successor reference window must contain "
                    "successor phase"
                )
            if not np.isfinite(margin) or margin < 0.0:
                raise ValueError(
                    "successor interior path margin must be finite and "
                    "non-negative"
                )
            if any(value is None for value in semantic_local_values):
                raise ValueError(
                    "runtime successor reference requires all semantic-local "
                    "path-length components"
                )
            components = tuple(float(value) for value in semantic_local_values)
            if any(
                not np.isfinite(value) or value < 0.0
                for value in components
            ):
                raise ValueError(
                    "semantic-local path lengths must be finite and "
                    "non-negative"
                )
            if not np.isclose(
                components[0] + components[1] + components[2],
                components[3],
                atol=1.0e-6,
                rtol=1.0e-9,
            ):
                raise ValueError(
                    "semantic-local total must equal prefix + Bridge + suffix"
                )
            object.__setattr__(self, "runtime_successor_bank_path", bank_path)
            object.__setattr__(
                self, "successor_reference_selection_method", selection
            )
            object.__setattr__(
                self, "successor_reference_phase_window", window
            )
            object.__setattr__(
                self, "successor_interior_path_margin_mm", margin
            )
            names = (
                "semantic_local_source_prefix_mm",
                "semantic_local_bridge_length_mm",
                "semantic_local_successor_suffix_mm",
                "semantic_local_total_length_mm",
            )
            for name, component in zip(names, components, strict=True):
                object.__setattr__(self, name, component)
        elif any(value is not None for value in semantic_local_values):
            raise ValueError(
                "semantic-local path lengths require a runtime successor "
                "reference"
            )
        if self.generation_method not in {
            "corridor_diverse",
            "normalized_farthest_point",
            "manual_reviewed",
            "flexible_reference",
        }:
            raise ValueError("unsupported V2 generation method")
        if self.bridge_algorithm not in {
            CUBIC_BEZIER_FIXED,
            CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
        }:
            raise ValueError("unsupported V2 Bridge algorithm")
        ratio = float(self.minimum_tangent_handle_chord_ratio)
        adjustment = self.maximum_endpoint_speed_adjustment_mm_s
        if self.bridge_algorithm == CUBIC_BEZIER_FIXED:
            if ratio != 0.0 or adjustment is not None:
                raise ValueError(
                    "fixed V2 Bridge cannot use tangent regularization"
                )
        else:
            if not np.isfinite(ratio) or not 0.0 < ratio <= 0.25:
                raise ValueError(
                    "regularized V2 Bridge ratio must be in (0, 0.25]"
                )
            if adjustment is None or (
                not np.isfinite(adjustment) or adjustment <= 0.0
            ):
                raise ValueError(
                    "regularized V2 Bridge requires a positive speed limit"
                )

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "EpisodeHandoffManifest":
        bridge = dict(value["bridge"])
        generation = dict(value.get("generation", {}))
        tangent = dict(generation.get("tangent_regularization", {}))
        runtime_source = dict(generation.get("runtime_source_reference", {}))
        runtime_successor = dict(
            generation.get("runtime_successor_reference", {})
        )
        semantic_local = dict(
            runtime_successor.get("semantic_local_objective", {})
        )
        validation = dict(value.get("validation", {}))
        return cls(
            schema_version=str(
                value.get("schema_version", "a0509.task_c_handoff_episode.v2")
            ),
            composition_id=str(value["composition_id"]),
            handoff_id=str(value["handoff_id"]),
            source=HandoffBoundary.from_mapping(dict(value["source"])),
            successor=HandoffBoundary.from_mapping(dict(value["successor"])),
            bridge_duration_s=float(bridge["duration_s"]),
            nominal_length_mm=(
                None
                if bridge.get("nominal_length_mm") is None
                else float(bridge["nominal_length_mm"])
            ),
            generation_method=str(generation.get("method", "corridor_diverse")),
            bridge_admission_mode=BridgeAdmissionMode(
                generation.get(
                    "bridge_admission_mode",
                    BridgeAdmissionMode.STRICT_LEVEL2.value,
                )
            ),
            reference_only=bool(generation.get("reference_only", False)),
            bridge_algorithm=str(
                generation.get("bridge_algorithm", CUBIC_BEZIER_FIXED)
            ),
            minimum_tangent_handle_chord_ratio=float(
                tangent.get("minimum_handle_chord_ratio", 0.0)
            ),
            maximum_endpoint_speed_adjustment_mm_s=(
                None
                if tangent.get("maximum_endpoint_speed_adjustment_mm_s")
                is None
                else float(
                    tangent["maximum_endpoint_speed_adjustment_mm_s"]
                )
            ),
            semantic_authority=parse_semantic_authority(
                validation.get(
                    "semantic_authority",
                    SemanticAuthority.RUNTIME_GUARDED.value,
                )
            ),
            semantic_diagnostics=tuple(
                validation.get("semantic_diagnostics", ())
            ),
            hard_filter_passed=validation.get("hard_filter_passed", False),
            robot_executable=validation.get("robot_executable", False),
            dry_run_only=validation.get("dry_run_only", True),
            transport_floor_mm=(
                None
                if bridge.get("transport_floor_mm") is None
                else float(bridge["transport_floor_mm"])
            ),
            ik_checked=validation.get("ik_checked", False),
            collision_checked=validation.get("collision_checked", False),
            runtime_source_bank_path=(
                None
                if runtime_source.get("support_bank") is None
                else str(runtime_source["support_bank"])
            ),
            source_reference_selection_method=(
                None
                if runtime_source.get("selection_method") is None
                else str(runtime_source["selection_method"])
            ),
            source_reference_phase_window=(
                None
                if runtime_source.get("phase_window") is None
                else tuple(
                    float(item) for item in runtime_source["phase_window"]
                )
            ),
            runtime_successor_bank_path=(
                None
                if runtime_successor.get("support_bank") is None
                else str(runtime_successor["support_bank"])
            ),
            successor_reference_selection_method=(
                None
                if runtime_successor.get("selection_method") is None
                else str(runtime_successor["selection_method"])
            ),
            successor_reference_phase_window=(
                None
                if runtime_successor.get("phase_window") is None
                else tuple(
                    float(item)
                    for item in runtime_successor["phase_window"]
                )
            ),
            successor_interior_path_margin_mm=(
                None
                if runtime_successor.get("interior_path_margin_mm") is None
                else float(runtime_successor["interior_path_margin_mm"])
            ),
            semantic_local_source_prefix_mm=(
                None
                if semantic_local.get("source_prefix_mm") is None
                else float(semantic_local["source_prefix_mm"])
            ),
            semantic_local_bridge_length_mm=(
                None
                if semantic_local.get("bridge_length_mm") is None
                else float(semantic_local["bridge_length_mm"])
            ),
            semantic_local_successor_suffix_mm=(
                None
                if semantic_local.get("successor_suffix_mm") is None
                else float(semantic_local["successor_suffix_mm"])
            ),
            semantic_local_total_length_mm=(
                None
                if semantic_local.get("total_length_mm") is None
                else float(semantic_local["total_length_mm"])
            ),
        )

    @classmethod
    def load(cls, path: str | Path) -> "EpisodeHandoffManifest":
        manifest_path = Path(path).expanduser().resolve()
        if not manifest_path.is_file():
            raise FileNotFoundError(f"V2 episode manifest not found: {manifest_path}")
        text = manifest_path.read_text(encoding="utf-8")
        if manifest_path.suffix.lower() == ".json":
            value = json.loads(text)
        else:
            try:
                import yaml
            except ImportError as exc:
                raise RuntimeError("PyYAML is required for YAML handoff manifests") from exc
            value = yaml.safe_load(text)
        if not isinstance(value, dict):
            raise ValueError("V2 episode manifest must contain a mapping")
        return cls.from_mapping(value)

    def to_record(self) -> dict[str, Any]:
        generation = {
            "method": self.generation_method,
            "bridge_admission_mode": self.bridge_admission_mode.value,
            "reference_only": self.reference_only,
            "bridge_algorithm": self.bridge_algorithm,
            "tangent_regularization": {
                "minimum_handle_chord_ratio": (
                    self.minimum_tangent_handle_chord_ratio
                ),
                "maximum_endpoint_speed_adjustment_mm_s": (
                    self.maximum_endpoint_speed_adjustment_mm_s
                ),
            },
        }
        if self.runtime_source_bank_path is not None:
            generation["runtime_source_reference"] = {
                "support_bank": self.runtime_source_bank_path,
                "selection_method": self.source_reference_selection_method,
                "phase_window": list(self.source_reference_phase_window),
            }
        if self.runtime_successor_bank_path is not None:
            generation["runtime_successor_reference"] = {
                "support_bank": self.runtime_successor_bank_path,
                "selection_method": (
                    self.successor_reference_selection_method
                ),
                "phase_window": list(
                    self.successor_reference_phase_window
                ),
                "selected_reference_phase": self.successor.phase,
                "runtime_phase_gate": False,
                "authority": "bridge_reference_and_metadata_only",
                "interior_path_margin_mm": (
                    self.successor_interior_path_margin_mm
                ),
                "semantic_local_objective": {
                    "source_prefix_mm": (
                        self.semantic_local_source_prefix_mm
                    ),
                    "bridge_length_mm": (
                        self.semantic_local_bridge_length_mm
                    ),
                    "successor_suffix_mm": (
                        self.semantic_local_successor_suffix_mm
                    ),
                    "total_length_mm": (
                        self.semantic_local_total_length_mm
                    ),
                },
            }
        return {
            "schema_version": self.schema_version,
            "composition_id": self.composition_id,
            "handoff_id": self.handoff_id,
            "source": self.source.to_record(),
            "successor": self.successor.to_record(),
            "bridge": {
                "duration_s": self.bridge_duration_s,
                "nominal_length_mm": self.nominal_length_mm,
                "transport_floor_mm": self.transport_floor_mm,
            },
            "generation": generation,
            "validation": {
                "hard_filter_passed": self.hard_filter_passed,
                "semantic_authority": self.semantic_authority.value,
                "semantic_checks_enforced_by_runtime": (
                    self.semantic_authority.runtime_semantic_checks_enforced
                ),
                "semantic_diagnostics": list(self.semantic_diagnostics),
                "robot_executable": self.robot_executable,
                "dry_run_only": self.dry_run_only,
                "ik_checked": self.ik_checked,
                "collision_checked": self.collision_checked,
            },
        }


@dataclass(frozen=True)
class HandoffCompatibilityConfig:
    max_first_xyz_axis_delta_mm: float | None = None
    max_first_rotation_delta_deg: float | None = None
    max_prefix_velocity_mm_s: float | None = None
    # Diagnostic reference only. Raw ACT-B finite-difference acceleration is
    # observation-conditioned policy output and is not a runtime reject gate.
    max_prefix_acceleration_mm_s2: float | None = None
    max_bridge_prefix_velocity_mismatch_mm_s: float | None = None
    max_crossfade_xyz_axis_step_mm: float | None = None
    max_crossfade_rotation_step_deg: float | None = None
    # A bounded one-command pipeline may compare the next proposal against
    # cmd[k-1], not merely cmd[k]. These limits therefore apply across the
    # complete acknowledgement span (two commands when max ACK lag is one).
    acknowledged_command_span_steps: int = 1
    max_acknowledged_xyz_axis_span_mm: float | None = None
    max_acknowledged_rotation_span_deg: float | None = None
    # Hard limit for commands reconstructed from two acknowledgements plus the
    # prospective crossfade and first continuation command.
    max_crossfade_command_acceleration_mm_s2: float | None = None
    gripper_open_threshold: float = 0.3
    gripper_close_threshold: float = 0.7

    def __post_init__(self) -> None:
        if (
            not isinstance(self.acknowledged_command_span_steps, int)
            or isinstance(self.acknowledged_command_span_steps, bool)
            or self.acknowledged_command_span_steps not in {1, 2}
        ):
            raise ValueError("acknowledged command span must be one or two steps")
        if (
            self.max_acknowledged_xyz_axis_span_mm is None
            and self.max_crossfade_xyz_axis_step_mm is not None
        ):
            object.__setattr__(
                self,
                "max_acknowledged_xyz_axis_span_mm",
                self.max_crossfade_xyz_axis_step_mm,
            )
        if (
            self.max_acknowledged_rotation_span_deg is None
            and self.max_crossfade_rotation_step_deg is not None
        ):
            object.__setattr__(
                self,
                "max_acknowledged_rotation_span_deg",
                self.max_crossfade_rotation_step_deg,
            )
        values = (
            self.max_first_xyz_axis_delta_mm,
            self.max_first_rotation_delta_deg,
            self.max_prefix_velocity_mm_s,
            self.max_prefix_acceleration_mm_s2,
            self.max_bridge_prefix_velocity_mismatch_mm_s,
            self.max_crossfade_xyz_axis_step_mm,
            self.max_crossfade_rotation_step_deg,
            self.max_acknowledged_xyz_axis_span_mm,
            self.max_acknowledged_rotation_span_deg,
            self.max_crossfade_command_acceleration_mm_s2,
        )
        if any(value is not None and (not np.isfinite(value) or value <= 0.0) for value in values):
            raise ValueError("compatibility thresholds must be positive or null")
        if not (
            0.0
            <= self.gripper_open_threshold
            < self.gripper_close_threshold
            <= 1.0
        ):
            raise ValueError("gripper thresholds must satisfy 0 <= open < close <= 1")

    def require_live_thresholds(self) -> None:
        missing = [
            name
            for name in (
                "max_first_xyz_axis_delta_mm",
                "max_first_rotation_delta_deg",
                "max_prefix_velocity_mm_s",
                "max_bridge_prefix_velocity_mismatch_mm_s",
                "max_crossfade_xyz_axis_step_mm",
                "max_crossfade_rotation_step_deg",
                "max_acknowledged_xyz_axis_span_mm",
                "max_acknowledged_rotation_span_deg",
                "max_crossfade_command_acceleration_mm_s2",
            )
            if getattr(self, name) is None
        ]
        if missing:
            raise ValueError(
                "live V2 compatibility thresholds are unresolved: "
                + ",".join(missing)
            )


@dataclass(frozen=True)
class HandoffV2Config:
    enabled: bool = False
    handoff_window_steps: int = 24
    b_prefix_steps: int = 15
    crossfade_steps: int = 15
    max_b_result_age_sec: float = 0.30
    max_inflight_b_requests: int = 1
    request_retry_interval_steps: int = 3
    enable_soft_handoff: bool = True
    enable_endpoint_fallback: bool = True
    control_hz: float = 30.0
    bridge_admission_mode: BridgeAdmissionMode = BridgeAdmissionMode.STRICT_LEVEL2
    adaptive_b_max_splice_index: int = 0
    adaptive_b_max_candidates: int = 1
    semantic_authority: SemanticAuthority = SemanticAuthority.RUNTIME_GUARDED
    compatibility: HandoffCompatibilityConfig = field(
        default_factory=HandoffCompatibilityConfig
    )

    def __post_init__(self) -> None:
        try:
            admission_mode = BridgeAdmissionMode(self.bridge_admission_mode)
        except ValueError as exc:
            raise ValueError("unsupported Bridge admission mode") from exc
        object.__setattr__(self, "bridge_admission_mode", admission_mode)
        object.__setattr__(
            self,
            "semantic_authority",
            parse_semantic_authority(self.semantic_authority),
        )
        if not isinstance(self.enabled, bool):
            raise ValueError("V2 enabled must be boolean")
        for name in (
            "handoff_window_steps",
            "b_prefix_steps",
            "crossfade_steps",
            "max_inflight_b_requests",
            "request_retry_interval_steps",
            "adaptive_b_max_candidates",
        ):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        if (
            not isinstance(self.adaptive_b_max_splice_index, int)
            or isinstance(self.adaptive_b_max_splice_index, bool)
            or self.adaptive_b_max_splice_index < 0
        ):
            raise ValueError("adaptive_b_max_splice_index must be non-negative")
        if admission_mode is BridgeAdmissionMode.STRICT_LEVEL2 and (
            self.adaptive_b_max_splice_index != 0
            or self.adaptive_b_max_candidates != 1
        ):
            raise ValueError("STRICT_LEVEL2 preserves the legacy B[0] splice")
        if self.max_inflight_b_requests != 1:
            raise ValueError("initial V2 supports exactly one in-flight B request")
        if self.crossfade_steps > self.b_prefix_steps:
            raise ValueError("crossfade_steps cannot exceed b_prefix_steps")
        if self.crossfade_steps > self.handoff_window_steps:
            raise ValueError("crossfade_steps cannot exceed handoff_window_steps")
        if self.max_b_result_age_sec <= 0.0 or self.control_hz <= 0.0:
            raise ValueError("V2 timing values must be positive")
        if not isinstance(self.enable_soft_handoff, bool) or not isinstance(
            self.enable_endpoint_fallback, bool
        ):
            raise ValueError("V2 feature flags must be boolean")
