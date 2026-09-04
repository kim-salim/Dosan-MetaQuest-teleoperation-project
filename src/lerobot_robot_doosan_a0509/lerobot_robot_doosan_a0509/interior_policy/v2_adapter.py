"""Command-free mapping from symbolic policy edges to the current Task-C V2."""

from __future__ import annotations

import ast
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

from .catalog import LoadedOperatorCatalog
from .contracts import InteriorPolicyOperator, WorldState
from .planner import Plan, operator_applicability


@dataclass(frozen=True)
class SymbolicTransitionValidation:
    valid: bool
    reasons: tuple[str, ...]
    transition_type: str

    def to_record(self) -> dict[str, Any]:
        return asdict(self)


class SymbolicTransitionValidator:
    def validate(
        self,
        source_operator: InteriorPolicyOperator,
        successor_operator: InteriorPolicyOperator,
        world_state_after_source: WorldState,
    ) -> SymbolicTransitionValidation:
        reasons: list[str] = []
        applicability = operator_applicability(
            world_state_after_source, successor_operator
        )
        reasons.extend(applicability.reasons)
        if source_operator.exit_contact_mode != successor_operator.entry_contact_mode:
            reasons.append(
                "contact_mode_mismatch:"
                f"{source_operator.exit_contact_mode}->"
                f"{successor_operator.entry_contact_mode}"
            )
        if (
            source_operator.exit_contact_mode == "free_space"
            and world_state_after_source.gripper == "open"
            and world_state_after_source.holding == "none"
        ):
            transition_type = "empty_gripper_free_space_reposition"
        elif (
            source_operator.exit_contact_mode == "free_transport"
            and world_state_after_source.gripper == "closed"
            and world_state_after_source.holding == "blue_block"
        ):
            transition_type = "held_blue_block_free_transport"
        else:
            transition_type = "other_symbolic_transition"
        return SymbolicTransitionValidation(
            valid=not reasons,
            reasons=tuple(reasons),
            transition_type=transition_type,
        )


@dataclass(frozen=True)
class V2CodeAudit:
    required_symbols: Mapping[str, bool]
    complete: bool
    mappings: Mapping[str, str]

    def to_record(self) -> dict[str, Any]:
        return {
            "required_symbols": dict(self.required_symbols),
            "complete": self.complete,
            "mappings": dict(self.mappings),
        }


def _ast_symbols(path: Path) -> tuple[set[str], dict[str, set[str]]]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    names: set[str] = set()
    methods: dict[str, set[str]] = {}
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            names.add(node.name)
        elif isinstance(node, ast.ClassDef):
            names.add(node.name)
            methods[node.name] = {
                child.name
                for child in node.body
                if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef))
            }
    return names, methods


def audit_current_v2_code(repository_root: Path) -> V2CodeAudit:
    package = (
        repository_root
        / "src/lerobot_robot_doosan_a0509/lerobot_robot_doosan_a0509"
    )
    files = {
        "live_v2": package / "task_c_live_v2_rollout.py",
        "multi_live_v2": package / "task_c_multi_live_v2_rollout.py",
        "bridge": package / "task_c_handoff/bridge_runtime.py",
        "successor": package / "task_c_handoff/async_successor.py",
        "compatibility": package / "task_c_handoff/compatibility.py",
        "soft_handoff": package / "task_c_handoff/soft_handoff.py",
        "coordinator": package / "task_c_handoff/coordinator.py",
        "multi_stage": package / "task_c_handoff/multi_stage.py",
        "stage_supervisor": package / "task_c_handoff/stage_supervisor.py",
    }
    parsed = {key: _ast_symbols(path) for key, path in files.items()}
    checks = {
        "TaskCLiveV2Strategy": "TaskCLiveV2Strategy" in parsed["live_v2"][0],
        "actual_ack_bridge_commit": {
            "_prepare_bridge_commit",
            "_begin_bridge",
            "_step_bridge",
            "_dispatch_v2_command",
        }.issubset(parsed["live_v2"][1].get("TaskCLiveV2Strategy", set())),
        "BridgeRuntimeSnapshot": "BridgeRuntimeSnapshot" in parsed["bridge"][0],
        "instantiate_bridge_queue": "instantiate_bridge_queue" in parsed["bridge"][0],
        "AsyncSuccessorController": "AsyncSuccessorController" in parsed["successor"][0],
        "fresh_prefix_evaluator": (
            "evaluate_prefix"
            in parsed["compatibility"][1].get("HandoffCompatibilityEvaluator", set())
        ),
        "quintic_soft_handoff": {
            "quintic_smoothstep",
            "build_soft_handoff",
        }.issubset(parsed["soft_handoff"][0]),
        "TaskCHandoffV2Coordinator": "TaskCHandoffV2Coordinator" in parsed["coordinator"][0],
        "MultiStagePlan": "MultiStagePlan" in parsed["multi_stage"][0],
        "TaskCMultiLiveV2Strategy": (
            "TaskCMultiLiveV2Strategy" in parsed["multi_live_v2"][0]
        ),
        "StagePhaseSupervisor": (
            "StagePhaseSupervisor" in parsed["stage_supervisor"][0]
        ),
        "automatic_phase_stage_cut": {
            "_update_automatic_phase_supervisor",
            "_reset_current_phase_supervisor",
        }.issubset(
            parsed["multi_live_v2"][1].get(
                "TaskCMultiLiveV2Strategy", set()
            )
        ),
        "terminal_policy_inference_horizon": (
            "_maybe_complete_terminal_inference"
            in parsed["multi_live_v2"][1].get(
                "TaskCMultiLiveV2Strategy", set()
            )
        ),
        "workspace_minimum_axis_mask": (
            "workspace_min_limit_enabled"
            in files["bridge"].read_text(encoding="utf-8")
            and "workspace_min_limit_enabled"
            in files["live_v2"].read_text(encoding="utf-8")
        ),
    }
    mappings = {
        "A_actual_ack_snapshot": "TaskCLiveV2Strategy._prepare_bridge_commit",
        "A_queue_invalidate_after_feasible_bridge": "TaskCLiveV2Strategy._begin_bridge",
        "runtime_bridge_queue": "bridge_runtime.instantiate_bridge_queue",
        "30Hz_bridge_consumption": "TaskCHandoffV2Coordinator.tick",
        "async_successor": "AsyncSuccessorController",
        "fresh_B_prefix_admission": "HandoffCompatibilityEvaluator.evaluate_prefix",
        "soft_crossfade": "soft_handoff.build_soft_handoff",
        "repeated_policy_visits": "TaskCMultiLiveV2Strategy + MultiStagePlan",
        "external_stage_authority": "MultiStageCoordinator.try_request_next",
        "stage_exit_authority": (
            "TaskCMultiLiveV2Strategy._update_automatic_phase_supervisor"
        ),
        "terminal_inference_horizon": (
            "TaskCMultiLiveV2Strategy._maybe_complete_terminal_inference"
        ),
        "z_minimum_axis_mask": (
            "TaskCLiveV2Strategy._bridge_runtime_limits"
        ),
    }
    return V2CodeAudit(checks, all(checks.values()), mappings)


@dataclass(frozen=True)
class V2ShadowTransitionValidation:
    source_operator: str
    successor_operator: str
    symbolic: SymbolicTransitionValidation
    source_support_available: bool
    successor_support_available: bool
    source_checkpoint_available: bool
    successor_checkpoint_available: bool
    v2_code_contract_available: bool
    external_planner_authority_available: bool
    structural_runtime_mapping: bool
    exact_episode_manifest_ready: bool
    matching_episode_manifests: tuple[str, ...]
    policy_shadow_ready: bool
    matching_policy_shadow_reports: tuple[str, ...]
    runtime_contract_compatible: bool
    missing_runtime_work: tuple[str, ...]
    physical_validation_performed: bool = False

    def to_record(self) -> dict[str, Any]:
        value = asdict(self)
        value["symbolic"] = self.symbolic.to_record()
        return value


class V2ShadowTransitionValidator:
    """Static/artifact validation only; this class cannot command a robot."""

    def __init__(self, catalog: LoadedOperatorCatalog) -> None:
        self.catalog = catalog
        self.repository_root = catalog.repository_root
        self.code_audit = audit_current_v2_code(self.repository_root)
        authority_path = (
            self.repository_root
            / "offline_tools/cross_task_handoff/authority.py"
        )
        authority_text = authority_path.read_text(encoding="utf-8")
        self.external_planner_authority_available = (
            'EXTERNAL_PLANNER = "external_planner"' in authority_text
            and "runtime_semantic_checks_enforced" in authority_text
        )

    @staticmethod
    def _support_available(operator: InteriorPolicyOperator, *, exit_side: bool) -> bool:
        path = Path(operator.evidence.phase_support_artifact)
        if not path.is_file():
            return False
        key = (
            operator.evidence.exit_segment
            if exit_side
            else operator.evidence.entry_segment
        )
        try:
            import numpy as np

            value = np.load(path, allow_pickle=False)
            return (
                f"{key}_episode_xyz_mm" in value.files
                and f"{key}_episode_ids" in value.files
            )
        except (OSError, ValueError):
            return False

    def _matching_manifests(
        self,
        source: InteriorPolicyOperator,
        successor: InteriorPolicyOperator,
    ) -> tuple[str, ...]:
        matches: list[str] = []
        pattern = "docs/artifacts/**/episode_manifests/*.json"
        for path in sorted(self.repository_root.glob(pattern)):
            try:
                value = json.loads(path.read_text(encoding="utf-8"))
                source_value = dict(value["source"])
                successor_value = dict(value["successor"])
                validation = dict(value.get("validation", {}))
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                continue
            exact = (
                str(source_value.get("task", "")).upper() == source.policy_id
                and str(source_value.get("segment")) == source.evidence.exit_segment
                and abs(
                    float(source_value.get("phase", -1.0))
                    - source.evidence.exit_phase
                )
                <= 1e-9
                and str(successor_value.get("task", "")).upper()
                == successor.policy_id
                and str(successor_value.get("segment"))
                == successor.evidence.entry_segment
                and abs(
                    float(successor_value.get("phase", -1.0))
                    - successor.evidence.entry_phase
                )
                <= 1e-9
                and bool(validation.get("hard_filter_passed", False))
            )
            if exact:
                matches.append(str(path.resolve()))
        return tuple(matches)

    def _matching_policy_shadows(
        self,
        manifests: Sequence[str],
    ) -> tuple[str, ...]:
        handoff_ids: set[str] = set()
        for manifest_path in manifests:
            try:
                value = json.loads(
                    Path(manifest_path).read_text(encoding="utf-8")
                )
                handoff_ids.add(str(value["handoff_id"]))
            except (KeyError, OSError, json.JSONDecodeError):
                continue
        if not handoff_ids:
            return ()
        matches: list[str] = []
        for path in sorted(
            self.repository_root.glob(
                "docs/artifacts/**/policy_shadow*.json"
            )
        ):
            try:
                value = json.loads(path.read_text(encoding="utf-8"))
                admission = value.get("prefix_admission")
                timing = value.get("control_timing")
                valid = (
                    value.get("schema_version")
                    == "a0509.task_c_handoff_v2_policy_shadow.v1"
                    and str(value.get("handoff_id")) in handoff_ids
                    and value.get("robot_commands_published") == 0
                    and value.get("live_enabled") is False
                    and value.get("mux_selected") is False
                    and value.get("terminal_state") == "RUN_B"
                    and value.get("takeover_success") is True
                    and value.get("fallback_required") is False
                    and isinstance(admission, Mapping)
                    and admission.get("valid") is True
                    and isinstance(timing, Mapping)
                    and int(timing.get("deadline_miss_count", -1)) == 0
                )
            except (OSError, TypeError, ValueError, json.JSONDecodeError):
                continue
            if valid:
                matches.append(str(path.resolve()))
        return tuple(matches)

    def validate(
        self,
        source_operator: InteriorPolicyOperator,
        successor_operator: InteriorPolicyOperator,
        world_state_after_source: WorldState,
    ) -> V2ShadowTransitionValidation:
        symbolic = SymbolicTransitionValidator().validate(
            source_operator, successor_operator, world_state_after_source
        )
        source_support = self._support_available(source_operator, exit_side=True)
        successor_support = self._support_available(successor_operator, exit_side=False)
        source_checkpoint = (
            Path(source_operator.evidence.checkpoint) / "model.safetensors"
        ).is_file()
        successor_checkpoint = (
            Path(successor_operator.evidence.checkpoint) / "model.safetensors"
        ).is_file()
        supported_transition = symbolic.transition_type in {
            "empty_gripper_free_space_reposition",
            "held_blue_block_free_transport",
        }
        structural = all(
            (
                symbolic.valid,
                source_support,
                successor_support,
                source_checkpoint,
                successor_checkpoint,
                self.code_audit.complete,
                self.external_planner_authority_available,
                supported_transition,
            )
        )
        manifests = self._matching_manifests(source_operator, successor_operator)
        exact_manifest = bool(manifests)
        shadow_reports = self._matching_policy_shadows(manifests)
        policy_shadow_ready = bool(shadow_reports)
        missing: list[str] = []
        if not structural:
            missing.append("symbolic_or_v2_structural_contract")
        if not exact_manifest:
            missing.extend(
                (
                    "planner_reviewed_edge_episode_manifest",
                    "edge_specific_bridge_hard_filter_and_dry_run",
                )
            )
        if not policy_shadow_ready:
            missing.append("edge_specific_fresh_successor_prefix_shadow")
        if symbolic.transition_type == "empty_gripper_free_space_reposition" and not exact_manifest:
            missing.append("empty_gripper_reposition_edge_validation")
        return V2ShadowTransitionValidation(
            source_operator=source_operator.id,
            successor_operator=successor_operator.id,
            symbolic=symbolic,
            source_support_available=source_support,
            successor_support_available=successor_support,
            source_checkpoint_available=source_checkpoint,
            successor_checkpoint_available=successor_checkpoint,
            v2_code_contract_available=self.code_audit.complete,
            external_planner_authority_available=(
                self.external_planner_authority_available
            ),
            structural_runtime_mapping=structural,
            exact_episode_manifest_ready=exact_manifest,
            matching_episode_manifests=manifests,
            policy_shadow_ready=policy_shadow_ready,
            matching_policy_shadow_reports=shadow_reports,
            runtime_contract_compatible=(
                structural and exact_manifest and policy_shadow_ready
            ),
            missing_runtime_work=tuple(dict.fromkeys(missing)),
            physical_validation_performed=False,
        )

    def validate_plan(self, plan: Plan) -> tuple[V2ShadowTransitionValidation, ...]:
        edges: list[V2ShadowTransitionValidation] = []
        for source_step, successor_step in zip(plan.steps, plan.steps[1:]):
            if source_step.operator.policy_id == successor_step.operator.policy_id:
                continue
            edges.append(
                self.validate(
                    source_step.operator,
                    successor_step.operator,
                    source_step.state_after,
                )
            )
        return tuple(edges)


def runtime_level(
    plan: Plan | None,
    edges: Sequence[V2ShadowTransitionValidation],
) -> str:
    if plan is None:
        return "NOT_SYMBOLICALLY_PLANNABLE"
    if edges and all(edge.runtime_contract_compatible for edge in edges):
        return "LEVEL_2_RUNTIME_CONTRACT_COMPATIBLE"
    return "LEVEL_1_SYMBOLICALLY_PLANNABLE"
