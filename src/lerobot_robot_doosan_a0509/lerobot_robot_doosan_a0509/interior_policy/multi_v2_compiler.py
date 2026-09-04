"""Compile an offline symbolic Plan into the existing Task-C Multi-V2 schema.

The compiler is deliberately command-free.  It does not load ACT weights,
open cameras, create ROS entities, or generate a Bridge in a control tick.
Its job is to prove that every policy switch in a Dijkstra/UCS result has an
explicit, exact-boundary V2 episode manifest and an explicit source supervisor
contract before a live runtime plan can even be parsed.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Mapping

from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    BridgeAdmissionMode,
    EpisodeHandoffManifest,
)
from lerobot_robot_doosan_a0509.task_c_handoff.runtime_command_profile import (
    LEGACY_RUNTIME_COMMAND_PROFILE_ID,
    get_runtime_command_profile,
)
from lerobot_robot_doosan_a0509.task_c_handoff.stage_supervisor import (
    GripperEventMode,
)

from .catalog import LoadedOperatorCatalog
from .execution_tail import (
    ExecutionTailDerivationConfig,
    LEGACY_EXECUTION_TAIL_PROFILE_ID,
    SHARED_EXECUTION_TAIL_PROFILE_ID,
    SUPPORTED_EXECUTION_TAIL_PROFILE_IDS,
    derive_zero_cost_execution_tail,
)
from .execution_tail_reference import (
    validate_runtime_source_reference,
    validate_runtime_successor_reference,
)
from .planner import (
    Applicability,
    Plan,
    PlanStep,
    PlannerCostConfig,
    SearchResult,
    TransitionCostEstimator,
    uniform_cost_search,
    zero_transition_cost,
)


EDGE_REGISTRY_SCHEMA_VERSION = "a0509.interior_policy_v2_edge_registry.v1"
EDGE_REGISTRY_SCHEMA_VERSION_V2 = "a0509.interior_policy_v2_edge_registry.v2"
EDGE_REGISTRY_SCHEMA_VERSION_V3 = "a0509.interior_policy_v2_edge_registry.v3"
STRICT_VERIFIED = "strict_verified"
FLEXIBLE_VERIFIED = "flexible_verified"
FLEXIBLE_SEMANTIC_CANDIDATE = "flexible_semantic_candidate"
TEMPORARILY_UNAVAILABLE = "temporarily_unavailable"
SOURCE_REFERENCE_MODES = frozenset(
    {"exact_semantic_exit", "execution_tail"}
)
REVIEWED_EMPTY_GRIPPER_EXECUTION_TAIL_OPERATORS = frozenset(
    {"T4.open_drawer"}
)


def _resolved_from(base: Path, value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = base / path
    return path.resolve()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True)
class EdgeRuntimeSpec:
    """Planner-reviewed runtime evidence for one operator-to-operator switch."""

    source_operator: str
    successor_operator: str
    handoff_manifest_path: Path | None
    policy_shadow_path: Path | None
    gripper_event: GripperEventMode
    admission_status: str = STRICT_VERIFIED
    uncertainty_penalty: float = 0.0
    validation_method: str = "strict_reference"
    source_reference_mode: str | None = None

    @classmethod
    def from_mapping(
        cls,
        value: Mapping[str, Any],
        *,
        base_dir: Path,
    ) -> "EdgeRuntimeSpec":
        source = str(value.get("source_operator", "")).strip()
        successor = str(value.get("successor_operator", "")).strip()
        manifest_value = str(value.get("handoff_manifest", "")).strip()
        shadow_value = str(value.get("policy_shadow", "")).strip()
        raw_event = value.get("gripper_event")
        raw_evidence = value.get("evidence")
        raw_source_reference_mode = (
            raw_evidence.get("source_reference_mode")
            if isinstance(raw_evidence, Mapping)
            else None
        )
        source_reference_mode = (
            None
            if raw_source_reference_mode is None
            or not str(raw_source_reference_mode).strip()
            else str(raw_source_reference_mode).strip()
        )
        status = str(value.get("admission_status", STRICT_VERIFIED)).strip()
        if status not in {
            STRICT_VERIFIED,
            FLEXIBLE_VERIFIED,
            FLEXIBLE_SEMANTIC_CANDIDATE,
            TEMPORARILY_UNAVAILABLE,
        }:
            raise ValueError("unsupported V2 edge admission_status")
        if not source or not successor:
            raise ValueError(
                "each V2 edge requires source_operator and successor_operator"
            )
        if status in {STRICT_VERIFIED, FLEXIBLE_VERIFIED} and not shadow_value:
            raise ValueError(f"{status} V2 edge requires policy_shadow")
        if status in {
            STRICT_VERIFIED,
            FLEXIBLE_VERIFIED,
            FLEXIBLE_SEMANTIC_CANDIDATE,
        } and not manifest_value:
            raise ValueError(f"{status} V2 edge requires handoff_manifest")
        if raw_event is None:
            raise ValueError(
                f"edge {source}->{successor} requires an explicit gripper_event"
            )
        return cls(
            source_operator=source,
            successor_operator=successor,
            handoff_manifest_path=(
                None
                if not manifest_value
                else _resolved_from(base_dir, manifest_value)
            ),
            policy_shadow_path=(
                None if not shadow_value else _resolved_from(base_dir, shadow_value)
            ),
            gripper_event=GripperEventMode(str(raw_event)),
            admission_status=status,
            uncertainty_penalty=float(value.get("uncertainty_penalty", 0.0)),
            validation_method=str(
                value.get(
                    "validation_method",
                    (
                        "strict_reference"
                        if status == STRICT_VERIFIED
                        else "reference_guided_flexible"
                    ),
                )
            ).strip(),
            source_reference_mode=source_reference_mode,
        )

    def __post_init__(self) -> None:
        if self.uncertainty_penalty < 0.0:
            raise ValueError("edge uncertainty_penalty must be non-negative")
        if not self.validation_method:
            raise ValueError("edge validation_method must not be empty")
        if (
            self.source_reference_mode is not None
            and self.source_reference_mode not in SOURCE_REFERENCE_MODES
        ):
            raise ValueError("unsupported edge source_reference_mode")

    @property
    def key(self) -> tuple[str, str]:
        return self.source_operator, self.successor_operator

    def to_record(self) -> dict[str, Any]:
        return {
            "source_operator": self.source_operator,
            "successor_operator": self.successor_operator,
            "handoff_manifest": (
                None
                if self.handoff_manifest_path is None
                else str(self.handoff_manifest_path)
            ),
            "policy_shadow": (
                None if self.policy_shadow_path is None else str(self.policy_shadow_path)
            ),
            "gripper_event": self.gripper_event.value,
            "admission_status": self.admission_status,
            "uncertainty_penalty": self.uncertainty_penalty,
            "validation_method": self.validation_method,
            "source_reference_mode": self.source_reference_mode,
        }


@dataclass(frozen=True)
class EdgeRuntimeRegistry:
    source_path: Path
    edges: tuple[EdgeRuntimeSpec, ...]
    execution_tail_profile_id: str = LEGACY_EXECUTION_TAIL_PROFILE_ID
    runtime_command_profile_id: str = LEGACY_RUNTIME_COMMAND_PROFILE_ID
    schema_version: str = EDGE_REGISTRY_SCHEMA_VERSION

    @classmethod
    def load(cls, path: str | Path) -> "EdgeRuntimeRegistry":
        source_path = Path(path).expanduser().resolve()
        if not source_path.is_file():
            raise FileNotFoundError(f"V2 edge registry not found: {source_path}")
        value = json.loads(source_path.read_text(encoding="utf-8"))
        schema = str(value.get("schema_version", ""))
        if schema not in {
            EDGE_REGISTRY_SCHEMA_VERSION,
            EDGE_REGISTRY_SCHEMA_VERSION_V2,
            EDGE_REGISTRY_SCHEMA_VERSION_V3,
        }:
            raise ValueError("unsupported interior-policy V2 edge registry")
        contract = value.get("selection_contract", {})
        if not isinstance(contract, Mapping):
            raise ValueError("edge registry selection_contract must be a mapping")
        execution_tail_profile_id = str(
            contract.get(
                "execution_tail_profile_id",
                LEGACY_EXECUTION_TAIL_PROFILE_ID,
            )
        ).strip()
        if execution_tail_profile_id not in SUPPORTED_EXECUTION_TAIL_PROFILE_IDS:
            raise ValueError("unsupported registry execution-tail profile")
        runtime_command_profile_id = str(
            contract.get(
                "runtime_command_profile_id",
                LEGACY_RUNTIME_COMMAND_PROFILE_ID,
            )
        ).strip()
        runtime_profile = get_runtime_command_profile(runtime_command_profile_id)
        raw_profile_config = contract.get("runtime_command_profile_config")
        raw_profile_sha256 = contract.get(
            "runtime_command_profile_config_sha256"
        )
        if (raw_profile_config is None) != (raw_profile_sha256 is None):
            raise ValueError(
                "runtime command profile config path/SHA must appear together"
            )
        if raw_profile_config is not None:
            profile_config_path = _resolved_from(
                source_path.parent,
                str(raw_profile_config),
            )
            if not profile_config_path.is_file():
                raise FileNotFoundError(
                    f"runtime command profile config not found: {profile_config_path}"
                )
            if _sha256(profile_config_path) != str(raw_profile_sha256):
                raise ValueError("runtime command profile config SHA mismatch")
            saved_profiles = json.loads(
                profile_config_path.read_text(encoding="utf-8")
            )
            if (
                saved_profiles.get("schema_version")
                != "a0509.runtime_command_profiles.v1"
            ):
                raise ValueError("unsupported runtime command profile config")
            saved_profile = saved_profiles.get("profiles", {}).get(
                runtime_command_profile_id
            )
            if not isinstance(saved_profile, Mapping):
                raise ValueError("runtime command profile is absent from config")
            for name, expected in runtime_profile.to_record().items():
                if name in {
                    "schema_version",
                    "profile_id",
                    "max_acknowledged_xyz_axis_span_mm",
                    "max_acknowledged_rotation_span_deg",
                    "physical_validation_performed",
                }:
                    continue
                if saved_profile.get(name) != expected:
                    raise ValueError(
                        f"runtime command profile config mismatch for {name}"
                    )
        raw_edges = value.get("edges")
        if not isinstance(raw_edges, list) or not raw_edges:
            raise ValueError("V2 edge registry requires a non-empty edges list")
        edges = tuple(
            EdgeRuntimeSpec.from_mapping(item, base_dir=source_path.parent)
            for item in raw_edges
        )
        keys = [item.key for item in edges]
        if len(keys) != len(set(keys)):
            raise ValueError("V2 edge registry contains duplicate operator edges")
        return cls(
            source_path=source_path,
            edges=edges,
            execution_tail_profile_id=execution_tail_profile_id,
            runtime_command_profile_id=runtime_command_profile_id,
            schema_version=schema,
        )

    @property
    def by_key(self) -> dict[tuple[str, str], EdgeRuntimeSpec]:
        return {item.key: item for item in self.edges}

    def require(self, source: str, successor: str) -> EdgeRuntimeSpec:
        try:
            return self.by_key[(source, successor)]
        except KeyError as exc:
            raise ValueError(
                "Dijkstra plan lacks a reviewed V2 edge: "
                f"{source}->{successor}"
            ) from exc


@dataclass(frozen=True)
class Level2RegistryAdmission:
    """UCS admission backed by explicit command-free V2 edge evidence.

    The first operator and same-policy forward continuations do not require a
    Bridge edge. Every cross-policy transition must be present in the registry.
    STRICT accepts policy-shadowed legacy evidence. FLEXIBLE accepts only
    policy-shadowed FLEXIBLE evidence. A semantic candidate remains visible in
    the registry for research/audit but never enters an executable UCS path.
    """

    allowed_edges: frozenset[tuple[str, str]]

    @classmethod
    def from_registry(
        cls,
        registry: EdgeRuntimeRegistry,
        *,
        bridge_admission_mode: BridgeAdmissionMode = BridgeAdmissionMode.STRICT_LEVEL2,
    ) -> "Level2RegistryAdmission":
        mode = BridgeAdmissionMode(bridge_admission_mode)
        required_status = (
            STRICT_VERIFIED
            if mode is BridgeAdmissionMode.STRICT_LEVEL2
            else FLEXIBLE_VERIFIED
        )
        allowed = {
            item.key
            for item in registry.edges
            if item.admission_status == required_status
        }
        return cls(frozenset(allowed))

    def __call__(
        self,
        previous_operator,
        next_operator,
        _state,
    ) -> Applicability:
        if previous_operator is None:
            return Applicability(valid=True, reasons=())
        if previous_operator.policy_id == next_operator.policy_id:
            return Applicability(valid=True, reasons=())
        key = (previous_operator.id, next_operator.id)
        if key in self.allowed_edges:
            return Applicability(valid=True, reasons=())
        return Applicability(
            valid=False,
            reasons=(
                "level2_edge_unavailable:"
                f"{previous_operator.id}->{next_operator.id}",
            ),
        )


def uniform_cost_search_level2(
    initial_state,
    goal,
    operators,
    edge_registry: EdgeRuntimeRegistry,
    *,
    cost_config: PlannerCostConfig | None = None,
    transition_cost_estimator: TransitionCostEstimator = zero_transition_cost,
    max_plans: int = 1,
    bridge_admission_mode: BridgeAdmissionMode = BridgeAdmissionMode.STRICT_LEVEL2,
    allow_intermediate_release: bool = True,
) -> SearchResult:
    """Run Dijkstra/UCS while admitting only reviewed Level-2 switches."""

    mode = BridgeAdmissionMode(bridge_admission_mode)
    registry_by_key = edge_registry.by_key

    def combined_transition_cost(previous, successor, state):
        base = float(transition_cost_estimator(previous, successor, state))
        if previous is None or previous.policy_id == successor.policy_id:
            return base
        spec = registry_by_key.get((previous.id, successor.id))
        return base + (0.0 if spec is None else spec.uncertainty_penalty)

    registry_admission = Level2RegistryAdmission.from_registry(
        edge_registry,
        bridge_admission_mode=mode,
    )

    def combined_admission(previous, successor, state):
        admitted = registry_admission(previous, successor, state)
        if not admitted.valid or allow_intermediate_release:
            return admitted
        if state.holding == "blue_block":
            effects = successor.effect_map
            if effects.get("holding") == "none":
                # A release whose destination is not the requested symbolic
                # goal is a detour and is forbidden in direct/no-release mode.
                location = effects.get("blue_block_location", state.blue_block_location)
                goal_location = goal.get("blue_block_location")
                if goal_location is not None and location != str(goal_location):
                    return Applicability(
                        valid=False,
                        reasons=(f"intermediate_release_forbidden:{location}",),
                    )
        return admitted

    return uniform_cost_search(
        initial_state,
        goal,
        operators,
        cost_config=cost_config,
        transition_cost_estimator=combined_transition_cost,
        transition_admission=combined_admission,
        max_plans=max_plans,
    )


@dataclass(frozen=True)
class PolicyVisit:
    """One contiguous run of operators owned by the same frozen ACT."""

    index: int
    policy_id: str
    steps: tuple[PlanStep, ...]

    @property
    def first_operator(self):
        return self.steps[0].operator

    @property
    def last_operator(self):
        return self.steps[-1].operator

    @property
    def operator_ids(self) -> tuple[str, ...]:
        return tuple(step.operator.id for step in self.steps)

    @property
    def stage_id(self) -> str:
        first = self.first_operator.id.split(".", 1)[1]
        last = self.last_operator.id.split(".", 1)[1]
        behavior = first if first == last else f"{first}_through_{last}"
        return f"visit_{self.index:02d}_{self.policy_id.lower()}_{behavior}"


def collapse_plan_visits(plan: Plan) -> tuple[PolicyVisit, ...]:
    if not plan.steps:
        raise ValueError("cannot compile an empty/already-satisfied plan")
    grouped: list[list[PlanStep]] = []
    for step in plan.steps:
        if grouped and grouped[-1][-1].operator.policy_id == step.operator.policy_id:
            grouped[-1].append(step)
        else:
            grouped.append([step])
    return tuple(
        PolicyVisit(
            index=index,
            policy_id=steps[0].operator.policy_id,
            steps=tuple(steps),
        )
        for index, steps in enumerate(grouped)
    )


@dataclass(frozen=True)
class MultiV2CompilationConfig:
    final_inference_steps: int = 1800
    phase_half_width: float = 0.05
    prearm_extra_phase: float = 0.02
    persistence_ticks: int = 3
    local_search_radius_indices: int = 5
    backward_tolerance: float = 0.02
    support_distance_threshold_mm: float | None = None
    support_loo_quantile: float = 0.95
    execution_tail: ExecutionTailDerivationConfig = field(
        default_factory=ExecutionTailDerivationConfig
    )

    def __post_init__(self) -> None:
        if (
            isinstance(self.final_inference_steps, bool)
            or self.final_inference_steps < 1
        ):
            raise ValueError("final_inference_steps must be a positive integer")
        if not 0.0 < self.phase_half_width <= 0.5:
            raise ValueError("phase_half_width must be in (0, 0.5]")
        if not 0.0 <= self.prearm_extra_phase <= 0.5:
            raise ValueError("prearm_extra_phase must be in [0, 0.5]")
        if self.persistence_ticks < 1:
            raise ValueError("persistence_ticks must be positive")
        if self.local_search_radius_indices < 1:
            raise ValueError("local_search_radius_indices must be positive")
        if not 0.0 <= self.backward_tolerance <= 1.0:
            raise ValueError("backward_tolerance must be in [0, 1]")
        if self.support_distance_threshold_mm is not None and (
            self.support_distance_threshold_mm <= 0.0
        ):
            raise ValueError(
                "support_distance_threshold_mm must be positive or null"
            )
        if not 0.0 < self.support_loo_quantile <= 1.0:
            raise ValueError("support_loo_quantile must be in (0, 1]")
        if not isinstance(
            self.execution_tail, ExecutionTailDerivationConfig
        ):
            raise ValueError(
                "execution_tail must be ExecutionTailDerivationConfig"
            )


@dataclass(frozen=True)
class CompiledMultiV2Plan:
    mapping: Mapping[str, Any]
    visits: tuple[PolicyVisit, ...]
    edge_specs: tuple[EdgeRuntimeSpec, ...]

    def write_json(self, path: str | Path) -> Path:
        output = Path(path).expanduser().resolve()
        if output.exists():
            raise FileExistsError(
                f"refusing to overwrite compiled Multi-V2 plan: {output}"
            )
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(self.mapping, ensure_ascii=False, indent=2, sort_keys=True)
            + "\n",
            encoding="utf-8",
        )
        return output


def _assert_catalog_plan_identity(
    plan: Plan,
    catalog: LoadedOperatorCatalog,
) -> None:
    known = catalog.by_id
    for step in plan.steps:
        operator = step.operator
        if operator.id not in known:
            raise ValueError(f"plan operator is absent from catalog: {operator.id}")
        if operator != known[operator.id]:
            raise ValueError(
                f"plan operator differs from current catalog: {operator.id}"
            )


def _assert_exact_edge(
    source_step: PlanStep,
    successor_step: PlanStep,
    spec: EdgeRuntimeSpec,
    *,
    manifest: EpisodeHandoffManifest,
    execution_tail: Any | None,
) -> tuple[
    EpisodeHandoffManifest,
    Mapping[str, Any] | None,
    Mapping[str, Any] | None,
]:
    """Validate symbolic and physical source boundaries without conflating them."""

    source = source_step.operator
    successor = successor_step.operator
    runtime_source_reference = None
    runtime_successor_reference = None
    if execution_tail is None:
        source_checks = (
            (
                manifest.source.task == source.policy_id,
                "source task",
                manifest.source.task,
                source.policy_id,
            ),
            (
                manifest.source.segment == source.evidence.exit_segment,
                "source segment",
                manifest.source.segment,
                source.evidence.exit_segment,
            ),
            (
                abs(manifest.source.phase - source.evidence.exit_phase) <= 1.0e-9,
                "source phase",
                manifest.source.phase,
                source.evidence.exit_phase,
            ),
        )
        if manifest.runtime_source_bank_path is not None:
            raise ValueError(
                f"edge {spec.source_operator}->{spec.successor_operator} "
                "declares a runtime source bank without an execution tail"
            )
    else:
        if execution_tail.source_operator != source.id:
            raise ValueError(
                f"edge {spec.source_operator}->{spec.successor_operator} "
                "execution tail belongs to another source operator"
            )
        if (
            execution_tail.semantic_exit_segment
            != source.evidence.exit_segment
            or abs(
                execution_tail.semantic_exit_phase
                - source.evidence.exit_phase
            )
            > 1.0e-9
        ):
            raise ValueError(
                f"edge {spec.source_operator}->{spec.successor_operator} "
                "execution-tail semantic exit differs from planner operator"
            )
        source_checks = (
            (
                manifest.source.task == source.policy_id,
                "runtime source task",
                manifest.source.task,
                source.policy_id,
            ),
            (
                manifest.source.segment == execution_tail.tracking_segment,
                "runtime source tracking segment",
                manifest.source.segment,
                execution_tail.tracking_segment,
            ),
        )
        if manifest.source.phase > execution_tail.nominal_phase + 1.0e-9:
            raise ValueError(
                f"edge {spec.source_operator}->{spec.successor_operator} "
                "runtime source reference phase is after the execution-tail "
                "nominal phase; a causal future join cannot be guaranteed"
            )
        if manifest.runtime_source_bank_path is None:
            # Backward-compatible same-segment reference. The actual/ACK state
            # is projected forward from this earlier reference at runtime. It
            # is explicit in provenance, but is not mislabelled as medoid-bank
            # validated evidence.
            runtime_source_reference = {
                "bank_path": None,
                "selection_method": (
                    "legacy_same_segment_reference_future_join"
                ),
                "medoid_episode": None,
                "medoid_frame": None,
                "segment": manifest.source.segment,
                "reference_phase": manifest.source.phase,
                "nominal_phase": float(execution_tail.nominal_phase),
                "phase_window": [
                    float(execution_tail.commit_phase_low),
                    float(execution_tail.commit_phase_high),
                ],
                "episode_count": int(execution_tail.episode_count),
                "validated_runtime_source_bank": False,
            }
        else:
            runtime_source_reference = validate_runtime_source_reference(
                manifest,
                manifest_path=spec.handoff_manifest_path,
                execution_tail=execution_tail,
            )
            runtime_source_reference = {
                **runtime_source_reference,
                "reference_phase": manifest.source.phase,
                "validated_runtime_source_bank": True,
            }

    successor_phase_check: tuple[tuple[bool, str, Any, Any], ...]
    if manifest.runtime_successor_bank_path is None:
        successor_phase_check = (
            (
                abs(
                    manifest.successor.phase
                    - successor.evidence.entry_phase
                )
                <= 1.0e-9,
                "successor phase",
                manifest.successor.phase,
                successor.evidence.entry_phase,
            ),
        )
    else:
        if (
            manifest.bridge_admission_mode
            is not BridgeAdmissionMode.FLEXIBLE_LEVEL2
        ):
            raise ValueError(
                f"edge {spec.source_operator}->{spec.successor_operator} "
                "interior successor reference requires FLEXIBLE_LEVEL2"
            )
        runtime_successor_reference = validate_runtime_successor_reference(
            manifest,
            manifest_path=spec.handoff_manifest_path,
            successor_operator=successor,
        )
        successor_phase_check = ()

    checks = (
        *source_checks,
        (
            manifest.successor.task == successor.policy_id,
            "successor task",
            manifest.successor.task,
            successor.policy_id,
        ),
        (
            manifest.successor.segment == successor.evidence.entry_segment,
            "successor segment",
            manifest.successor.segment,
            successor.evidence.entry_segment,
        ),
        *successor_phase_check,
    )
    for valid, field, actual, expected in checks:
        if not valid:
            raise ValueError(
                f"edge {spec.source_operator}->{spec.successor_operator} "
                f"{field} mismatch: manifest={actual!r} plan={expected!r}"
            )
    expected_gripper = source_step.state_after.gripper
    if (
        expected_gripper != "unknown"
        and manifest.source.semantic.gripper_state != expected_gripper
    ):
        raise ValueError(
            f"edge {spec.source_operator}->{spec.successor_operator} source "
            "gripper disagrees with symbolic state"
        )
    expected_holding = source_step.state_after.holding
    actual_holding = manifest.source.semantic.held_object
    held_aliases = dict(source.evidence.held_object_aliases)
    holding_matches = (
        actual_holding == expected_holding
        or held_aliases.get(actual_holding) == expected_holding
    )
    if expected_holding != "unknown" and not holding_matches:
        raise ValueError(
            f"edge {spec.source_operator}->{spec.successor_operator} source "
            "held object disagrees with symbolic state and explicit aliases"
        )
    if manifest.transport_floor_mm is not None:
        floor = float(manifest.transport_floor_mm)
        endpoint_minimum = min(
            float(manifest.source.nominal_position_mm[2]),
            float(manifest.successor.nominal_position_mm[2]),
        )
        if endpoint_minimum < floor - 1.0e-6:
            raise ValueError(
                f"edge {spec.source_operator}->{spec.successor_operator} "
                f"transport_floor_mm={floor} exceeds a Bridge endpoint "
                f"minimum Z={endpoint_minimum}"
            )
    return manifest, runtime_source_reference, runtime_successor_reference


def _assert_command_free_policy_shadow(
    manifest: EpisodeHandoffManifest,
    spec: EdgeRuntimeSpec,
) -> Mapping[str, Any]:
    path = spec.policy_shadow_path
    if path is None:
        raise ValueError(
            f"edge {spec.source_operator}->{spec.successor_operator} "
            "verified policy shadow path is absent"
        )
    if not path.is_file():
        raise FileNotFoundError(
            f"edge policy shadow not found: {path}"
        )
    value = json.loads(path.read_text(encoding="utf-8"))
    expected = {
        "schema_version": "a0509.task_c_handoff_v2_policy_shadow.v1",
        "handoff_id": manifest.handoff_id,
        "robot_commands_published": 0,
        "live_enabled": False,
        "mux_selected": False,
        "terminal_state": "RUN_B",
        "takeover_success": True,
        "fallback_required": False,
    }
    for field, expected_value in expected.items():
        if value.get(field) != expected_value:
            raise ValueError(
                f"edge {spec.source_operator}->{spec.successor_operator} "
                f"policy shadow {field}={value.get(field)!r}; "
                f"expected {expected_value!r}"
            )
    if manifest.transport_floor_mm is not None:
        bridge = value.get("bridge")
        shadow_floor = (
            None
            if not isinstance(bridge, Mapping)
            else bridge.get("transport_floor_mm")
        )
        if shadow_floor is None or abs(
            float(shadow_floor) - float(manifest.transport_floor_mm)
        ) > 1.0e-6:
            raise ValueError(
                f"edge {spec.source_operator}->{spec.successor_operator} "
                "policy shadow did not validate the exact transport_floor_mm"
            )
    admission = value.get("prefix_admission")
    if not isinstance(admission, Mapping) or admission.get("valid") is not True:
        raise ValueError(
            f"edge {spec.source_operator}->{spec.successor_operator} "
            "policy shadow lacks a valid fresh-prefix admission"
        )
    timing = value.get("control_timing")
    if (
        not isinstance(timing, Mapping)
        or int(timing.get("deadline_miss_count", -1)) != 0
    ):
        raise ValueError(
            f"edge {spec.source_operator}->{spec.successor_operator} "
            "policy shadow has control deadline misses"
        )
    return value


def compile_plan_to_multi_v2(
    plan: Plan,
    catalog: LoadedOperatorCatalog,
    edge_registry: EdgeRuntimeRegistry,
    *,
    composition_id: str,
    config: MultiV2CompilationConfig | None = None,
    expected_bridge_admission_mode: BridgeAdmissionMode | None = None,
) -> CompiledMultiV2Plan:
    """Compile a symbolic plan without claiming physical executability."""

    if not composition_id.strip():
        raise ValueError("composition_id must not be empty")
    _assert_catalog_plan_identity(plan, catalog)
    visits = collapse_plan_visits(plan)
    if len(visits) < 2:
        raise ValueError("Multi-V2 compilation requires at least one policy switch")
    compile_config = config or MultiV2CompilationConfig()

    used_policy_ids: list[str] = []
    for visit in visits:
        if visit.policy_id not in used_policy_ids:
            used_policy_ids.append(visit.policy_id)
    policies: dict[str, dict[str, Any]] = {}
    for policy_id in used_policy_ids:
        checkpoint = catalog.audit.tasks[policy_id].checkpoint
        policies[policy_id] = {
            "checkpoint": checkpoint,
            "reuse_context_policy": policy_id == visits[0].policy_id,
        }

    stages: list[dict[str, Any]] = []
    transitions: list[dict[str, Any]] = []
    compiled_edges: list[EdgeRuntimeSpec] = []
    compiled_modes: set[BridgeAdmissionMode] = set()
    execution_tail_records: list[dict[str, Any]] = []
    transport_floor_records: list[dict[str, Any]] = []
    for index, visit in enumerate(visits):
        terminal = index == len(visits) - 1
        if terminal:
            stages.append(
                {
                    "stage_id": visit.stage_id,
                    "policy_id": visit.policy_id,
                    "exit_authority": "policy_inference_end",
                    "inference_end_steps": compile_config.final_inference_steps,
                    "terminal": True,
                    "operators": list(visit.operator_ids),
                }
            )
            continue

        successor_visit = visits[index + 1]
        source_step = visit.steps[-1]
        successor_step = successor_visit.steps[0]
        spec = edge_registry.require(
            source_step.operator.id,
            successor_step.operator.id,
        )
        if spec.handoff_manifest_path is None:
            raise ValueError(
                f"edge {spec.source_operator}->{spec.successor_operator} "
                "has no executable handoff manifest"
            )
        manifest = EpisodeHandoffManifest.load(spec.handoff_manifest_path)
        execution_tail_result = None
        execution_tail = None
        if (
            manifest.bridge_admission_mode
            is BridgeAdmissionMode.FLEXIBLE_LEVEL2
        ):
            tail_config = compile_config.execution_tail
            if (
                edge_registry.execution_tail_profile_id
                == LEGACY_EXECUTION_TAIL_PROFILE_ID
            ):
                tail_config = ExecutionTailDerivationConfig.legacy_v11(
                    enabled=tail_config.enabled,
                    reviewed_empty_gripper_free_space_operators=(
                        tail_config.reviewed_empty_gripper_free_space_operators
                    ),
                )
            elif tail_config.profile_id != SHARED_EXECUTION_TAIL_PROFILE_ID:
                raise ValueError("compiler execution-tail profile differs from registry")
            elif tail_config.operator_overrides:
                raise ValueError(
                    "shared execution-tail profile forbids operator overrides"
                )
            if (
                spec.source_reference_mode == "execution_tail"
                and source_step.operator.id
                in REVIEWED_EMPTY_GRIPPER_EXECUTION_TAIL_OPERATORS
            ):
                reviewed = tuple(
                    dict.fromkeys(
                        (
                            *tail_config.reviewed_empty_gripper_free_space_operators,
                            source_step.operator.id,
                        )
                    )
                )
                tail_config = replace(
                    tail_config,
                    reviewed_empty_gripper_free_space_operators=reviewed,
                )
            execution_tail_result = derive_zero_cost_execution_tail(
                source_step.operator,
                config=tail_config,
            )
            if spec.source_reference_mode != "exact_semantic_exit":
                execution_tail = execution_tail_result.profile
            if (
                spec.source_reference_mode == "execution_tail"
                and execution_tail is None
            ):
                raise ValueError(
                    f"edge {spec.source_operator}->{spec.successor_operator} "
                    "requests an unavailable execution tail: "
                    f"{execution_tail_result.reason}"
                )
        (
            manifest,
            runtime_source_reference,
            runtime_successor_reference,
        ) = _assert_exact_edge(
            source_step,
            successor_step,
            spec,
            manifest=manifest,
            execution_tail=execution_tail,
        )
        if spec.admission_status == STRICT_VERIFIED:
            if (
                manifest.bridge_admission_mode
                is not BridgeAdmissionMode.STRICT_LEVEL2
            ):
                raise ValueError(
                    f"edge {spec.source_operator}->{spec.successor_operator} "
                    "strict registry entry requires a STRICT_LEVEL2 manifest"
                )
            _assert_command_free_policy_shadow(manifest, spec)
        elif spec.admission_status == FLEXIBLE_VERIFIED:
            if (
                manifest.bridge_admission_mode
                is not BridgeAdmissionMode.FLEXIBLE_LEVEL2
            ):
                raise ValueError(
                    f"edge {spec.source_operator}->{spec.successor_operator} "
                    "flexible registry entry requires a FLEXIBLE_LEVEL2 manifest"
                )
            _assert_command_free_policy_shadow(manifest, spec)
        elif spec.admission_status == FLEXIBLE_SEMANTIC_CANDIDATE:
            raise ValueError(
                f"edge {spec.source_operator}->{spec.successor_operator} "
                "is an unverified flexible candidate and cannot be compiled"
            )
        else:
            raise ValueError(
                f"edge {spec.source_operator}->{spec.successor_operator} "
                "is temporarily unavailable"
            )
        if (
            expected_bridge_admission_mode is not None
            and manifest.bridge_admission_mode
            is not BridgeAdmissionMode(expected_bridge_admission_mode)
        ):
            raise ValueError(
                f"edge {spec.source_operator}->{spec.successor_operator} "
                "does not match the requested Bridge admission mode"
            )
        compiled_modes.add(manifest.bridge_admission_mode)
        if len(compiled_modes) > 1:
            raise ValueError(
                "Multi-V2 compilation cannot mix STRICT_LEVEL2 and "
                "FLEXIBLE_LEVEL2 manifests in one runtime plan"
            )
        compiled_edges.append(spec)
        if manifest.transport_floor_mm is not None:
            transport_floor_records.append(
                {
                    "handoff_id": manifest.handoff_id,
                    "source_operator": source_step.operator.id,
                    "successor_operator": successor_step.operator.id,
                    "transport_floor_mm": float(
                        manifest.transport_floor_mm
                    ),
                    "source_endpoint_z_mm": float(
                        manifest.source.nominal_position_mm[2]
                    ),
                    "successor_endpoint_z_mm": float(
                        manifest.successor.nominal_position_mm[2]
                    ),
                }
            )
        tail_record = {
            "stage_id": visit.stage_id,
            "source_operator": source_step.operator.id,
            "bridge_admission_mode": manifest.bridge_admission_mode.value,
            "enabled": execution_tail is not None,
            "eligible": (
                False
                if execution_tail_result is None
                else execution_tail_result.eligible
            ),
            "reason": (
                "strict_level2_exact_boundary_preserved"
                if execution_tail_result is None
                else execution_tail_result.reason
            ),
            "profile": (
                None
                if execution_tail is None
                else execution_tail.to_record()
            ),
            "runtime_source_reference": runtime_source_reference,
        }
        execution_tail_records.append(tail_record)
        supervisor_record = {
            "support_artifact": (
                source_step.operator.evidence.phase_support_artifact
            ),
            "phase_half_width": compile_config.phase_half_width,
            "prearm_extra_phase": compile_config.prearm_extra_phase,
            "semantic_event_phase_half_width": compile_config.phase_half_width,
            "semantic_event_prearm_extra_phase": compile_config.prearm_extra_phase,
            "persistence_ticks": compile_config.persistence_ticks,
            "local_search_radius_indices": (
                compile_config.local_search_radius_indices
            ),
            "backward_tolerance": compile_config.backward_tolerance,
            "support_distance_threshold_mm": (
                compile_config.support_distance_threshold_mm
            ),
            "support_loo_quantile": compile_config.support_loo_quantile,
            "gripper_event": spec.gripper_event.value,
            # Learned handle manipulation may contain an early open/regrasp.
            # Forget that history and arm O1 only after the supported S2
            # prearm geometry has been reached.
            "rearm_closed_then_open_at_prearm": bool(
                source_step.operator.id
                in {"T1.open_white_container", "T4.open_drawer"}
                and spec.gripper_event
                is GripperEventMode.CLOSED_THEN_OPEN
            ),
        }
        if execution_tail is not None:
            supervisor_record.update(
                {
                    "phase_half_width": execution_tail.phase_half_width,
                    "prearm_extra_phase": (
                        execution_tail.prearm_extra_phase
                    ),
                    "deadline_extra_phase": max(
                        0.0,
                        execution_tail.deadline_phase
                        - execution_tail.commit_phase_high,
                    ),
                    "execution_tail": execution_tail.to_record(),
                }
            )
        stages.append(
            {
                "stage_id": visit.stage_id,
                "policy_id": visit.policy_id,
                "exit_authority": "phase_supervisor",
                "operators": list(visit.operator_ids),
                "phase_supervisor": supervisor_record,
            }
        )
        transition_record = {
                "transition_id": (
                    f"edge_{index:02d}_{visit.policy_id.lower()}_to_"
                    f"{successor_visit.policy_id.lower()}"
                ),
                "source_stage": visit.stage_id,
                "successor_stage": successor_visit.stage_id,
                "source_operator": source_step.operator.id,
                "successor_operator": successor_step.operator.id,
                "handoff_manifest": str(spec.handoff_manifest_path),
                "bridge_admission_status": spec.admission_status,
                "bridge_validation_method": spec.validation_method,
                "bridge_admission_mode": manifest.bridge_admission_mode.value,
                "uncertainty_penalty": spec.uncertainty_penalty,
                "source_execution_tail": (
                    None
                    if execution_tail is None
                    else execution_tail.to_record()
                ),
                "runtime_source_reference": runtime_source_reference,
                "runtime_successor_reference": runtime_successor_reference,
                "transport_floor_mm": manifest.transport_floor_mm,
            }
        if spec.policy_shadow_path is not None:
            transition_record["policy_shadow"] = str(spec.policy_shadow_path)
        transitions.append(transition_record)

    mapping: dict[str, Any] = {
        "schema_version": "a0509.task_c_multi_stage_plan.v1",
        "composition_id": composition_id,
        "runtime_command_profile_id": (
            edge_registry.runtime_command_profile_id
        ),
        "policies": policies,
        "stages": stages,
        "transitions": transitions,
        "planner_provenance": {
            "catalog_id": catalog.catalog_id,
            "catalog_path": str(catalog.config_path),
            "edge_registry": str(edge_registry.source_path),
            "operators": list(plan.operator_ids),
            "policy_sequence": list(plan.policy_sequence),
            "total_cost": plan.total_cost,
            "compiler_contract": (
                "offline_episode_level_only_no_30hz_planning"
            ),
            "physical_validation_performed": False,
            "command_free_policy_shadow_required": "all_verified_edges",
            "final_policy_done_token_available": False,
            "final_termination": (
                "configured_policy_inference_horizon"
            ),
            "z_minimum_enabled": bool(transport_floor_records),
            "bridge_transport_floors": transport_floor_records,
            "bridge_admission_mode": next(iter(compiled_modes)).value,
            "runtime_command_profile": get_runtime_command_profile(
                edge_registry.runtime_command_profile_id
            ).to_record(),
            "dijkstra_operator_costs_unchanged": True,
            "zero_cost_execution_tail": {
                "profile_id": edge_registry.execution_tail_profile_id,
                "enabled_for_flexible_level2": (
                    compile_config.execution_tail.enabled
                ),
                "eligible_source_contract": (
                    "held_object_free_transport_plus_registry_reviewed_"
                    "empty_gripper_free_space"
                    if any(
                        item["enabled"]
                        and item["source_operator"]
                        in REVIEWED_EMPTY_GRIPPER_EXECUTION_TAIL_OPERATORS
                        for item in execution_tail_records
                    )
                    else "held_object_free_transport_only"
                ),
                "fixed_z_minimum_used": bool(transport_floor_records),
                "records": execution_tail_records,
            },
        },
    }
    return CompiledMultiV2Plan(
        mapping=mapping,
        visits=visits,
        edge_specs=tuple(compiled_edges),
    )


def compile_controlled_catalog_plan(
    catalog: LoadedOperatorCatalog,
    edge_registry: EdgeRuntimeRegistry,
    *,
    composition_id: str,
    config: MultiV2CompilationConfig | None = None,
    bridge_admission_mode: BridgeAdmissionMode = BridgeAdmissionMode.STRICT_LEVEL2,
) -> CompiledMultiV2Plan:
    """Run the catalog's controlled UCS experiment, then compile its best plan."""

    result = uniform_cost_search_level2(
        catalog.target.initial_state,
        catalog.target.goal,
        catalog.controlled_operators,
        edge_registry,
        cost_config=catalog.target.cost_config,
        bridge_admission_mode=bridge_admission_mode,
    )
    if result.best_plan is None:
        raise ValueError("controlled catalog experiment returned NO_PLAN")
    return compile_plan_to_multi_v2(
        result.best_plan,
        catalog,
        edge_registry,
        composition_id=composition_id,
        config=config,
        expected_bridge_admission_mode=bridge_admission_mode,
    )
