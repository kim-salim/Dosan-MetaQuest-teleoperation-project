"""Episode-level initial/goal planning for the A0509 web controller.

This module is deliberately command-free.  It turns an operator-confirmed
symbolic initial state and a partial symbolic goal into a Level-2-only UCS
plan, validates every cross-policy edge against the reviewed registry, and
emits the existing Multi-V2 plan schema.  It never imports ROS, opens a camera,
loads a policy, or enables robot motion.

The browser is not an authority: the web backend must call this module again
even when the browser has already visualized an apparently identical plan.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

from lerobot_robot_doosan_a0509.task_c_handoff.multi_stage import MultiStagePlan
from lerobot_robot_doosan_a0509.task_c_handoff.models import BridgeAdmissionMode

from .catalog import LoadedOperatorCatalog
from .contracts import STATE_FACT_FIELDS, InteriorPolicyOperator, WorldState
from .multi_v2_compiler import (
    EdgeRuntimeRegistry,
    MultiV2CompilationConfig,
    compile_plan_to_multi_v2,
    uniform_cost_search_level2,
)
from .planner import Plan, PlannerCostConfig, SearchResult, goal_satisfied


DEFAULT_FINAL_INFERENCE_STEPS = 1800
MAX_FINAL_INFERENCE_STEPS = 18000
MAX_WEB_COST = 1000.0
WEB_REQUEST_SCHEMA_VERSION = "a0509.web_episode_plan_request.v1"
WEB_RESULT_SCHEMA_VERSION = "a0509.web_episode_plan_result.v1"


def _canonical_json(value: Mapping[str, Any]) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_new_json(path: Path, value: Mapping[str, Any]) -> Path:
    output = path.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    return output


@dataclass(frozen=True)
class EpisodePlanRequest:
    initial_state: WorldState
    goal: Mapping[str, str]
    enabled_operator_ids: tuple[str, ...]
    cost_config: PlannerCostConfig
    final_inference_steps: int = DEFAULT_FINAL_INFERENCE_STEPS
    max_plans: int = 3
    bridge_admission_mode: BridgeAdmissionMode = BridgeAdmissionMode.FLEXIBLE_LEVEL2
    allow_intermediate_release: bool = False

    @classmethod
    def from_mapping(
        cls,
        value: Mapping[str, Any],
        *,
        catalog: LoadedOperatorCatalog,
    ) -> "EpisodePlanRequest":
        raw_initial = value.get("initial_state")
        raw_goal = value.get("goal")
        if not isinstance(raw_initial, Mapping):
            raise ValueError("initial_state must be a mapping")
        if not isinstance(raw_goal, Mapping) or not raw_goal:
            raise ValueError("goal must be a non-empty partial predicate mapping")

        allowed_world_fields = set(WorldState.__dataclass_fields__)
        extra_initial = set(raw_initial) - allowed_world_fields
        if extra_initial:
            raise ValueError(
                "initial_state contains unsupported fields: "
                + ",".join(sorted(str(item) for item in extra_initial))
            )
        required = {
            "drawer",
            "white_container",
            "blue_block_location",
            "holding",
            "gripper",
            "contact_mode",
        }
        missing = required - set(raw_initial)
        if missing:
            raise ValueError(
                "initial_state is missing fields: " + ",".join(sorted(missing))
            )
        initial = WorldState(**dict(raw_initial))

        goal = {str(key): str(item) for key, item in raw_goal.items()}
        unsupported_goal = set(goal) - set(STATE_FACT_FIELDS)
        if unsupported_goal:
            raise ValueError(
                "goal contains unsupported fields: "
                + ",".join(sorted(unsupported_goal))
            )
        # Eagerly validates goal values through the same WorldState contract.
        for field, expected in goal.items():
            candidate = dict(initial.to_record())
            candidate[field] = expected
            WorldState(**candidate)

        known = catalog.by_id
        raw_ids = value.get("enabled_operator_ids")
        if raw_ids is None:
            operator_ids = tuple(sorted(known))
        else:
            if not isinstance(raw_ids, Sequence) or isinstance(raw_ids, (str, bytes)):
                raise ValueError("enabled_operator_ids must be an array")
            operator_ids = tuple(dict.fromkeys(str(item) for item in raw_ids))
            if not operator_ids:
                raise ValueError("at least one operator must be enabled")
            unknown = set(operator_ids) - set(known)
            if unknown:
                raise ValueError(
                    "unknown enabled operators: " + ",".join(sorted(unknown))
                )

        switch_penalty = float(value.get("policy_switch_penalty", 0.25))
        continuation_penalty = float(
            value.get("same_policy_continuation_penalty", 0.0)
        )
        for name, cost in (
            ("policy_switch_penalty", switch_penalty),
            ("same_policy_continuation_penalty", continuation_penalty),
        ):
            if not math.isfinite(cost) or not 0.0 <= cost <= MAX_WEB_COST:
                raise ValueError(
                    f"{name} must be finite and in [0, {MAX_WEB_COST:g}]"
                )
        final_steps = int(
            value.get("final_inference_steps", DEFAULT_FINAL_INFERENCE_STEPS)
        )
        max_plans = int(value.get("max_plans", 3))
        bridge_admission_mode = BridgeAdmissionMode(
            value.get(
                "bridge_admission_mode",
                BridgeAdmissionMode.FLEXIBLE_LEVEL2.value,
            )
        )
        allow_intermediate_release = value.get("allow_intermediate_release", False)
        if not isinstance(allow_intermediate_release, bool):
            raise ValueError("allow_intermediate_release must be boolean")
        if (
            isinstance(value.get("final_inference_steps"), bool)
            or not 1 <= final_steps <= MAX_FINAL_INFERENCE_STEPS
        ):
            raise ValueError("final_inference_steps must be an integer in [1, 18000]")
        if isinstance(value.get("max_plans"), bool) or not 1 <= max_plans <= 10:
            raise ValueError("max_plans must be in [1, 10]")

        request = cls(
            initial_state=initial,
            goal=goal,
            enabled_operator_ids=operator_ids,
            cost_config=PlannerCostConfig(
                policy_switch_penalty=switch_penalty,
                same_policy_continuation_penalty=continuation_penalty,
            ),
            final_inference_steps=final_steps,
            max_plans=max_plans,
            bridge_admission_mode=bridge_admission_mode,
            allow_intermediate_release=allow_intermediate_release,
        )
        # Also validates goal keys if the initial state already satisfies it.
        goal_satisfied(request.initial_state, request.goal)
        return request

    def to_record(self) -> dict[str, Any]:
        return {
            "schema_version": WEB_REQUEST_SCHEMA_VERSION,
            "initial_state": self.initial_state.to_record(),
            "goal": dict(self.goal),
            "enabled_operator_ids": list(self.enabled_operator_ids),
            "policy_switch_penalty": self.cost_config.policy_switch_penalty,
            "same_policy_continuation_penalty": (
                self.cost_config.same_policy_continuation_penalty
            ),
            "final_inference_steps": self.final_inference_steps,
            "max_plans": self.max_plans,
            "bridge_admission_mode": self.bridge_admission_mode.value,
            "allow_intermediate_release": self.allow_intermediate_release,
            "forward_only": True,
            "level2_only": True,
        }


@dataclass(frozen=True)
class EpisodePlanDecision:
    request: EpisodePlanRequest
    search: SearchResult
    selected_plan: Plan | None
    execution_kind: str
    runtime_blockers: tuple[str, ...]

    @property
    def runtime_compilable(self) -> bool:
        return self.execution_kind == "multi_v2" and not self.runtime_blockers

    def to_record(self) -> dict[str, Any]:
        return {
            "schema_version": WEB_RESULT_SCHEMA_VERSION,
            "request": self.request.to_record(),
            "search": self.search.to_record(),
            "selected_plan": (
                None if self.selected_plan is None else self.selected_plan.to_record()
            ),
            "execution": {
                "kind": self.execution_kind,
                "runtime_compilable": self.runtime_compilable,
                "blockers": list(self.runtime_blockers),
                "physical_validation_performed": False,
                "initial_state_authority": "operator_asserted_not_perception_verified",
            },
        }


def _operators_for_request(
    catalog: LoadedOperatorCatalog,
    request: EpisodePlanRequest,
) -> tuple[InteriorPolicyOperator, ...]:
    by_id = catalog.by_id
    return tuple(by_id[item] for item in request.enabled_operator_ids)


def _runtime_classification(
    plan: Plan | None,
    *,
    catalog: LoadedOperatorCatalog,
    request: EpisodePlanRequest,
) -> tuple[str, tuple[str, ...]]:
    if plan is None:
        return "no_plan", ("LEVEL2_NO_PLAN",)
    if not plan.steps:
        return "no_op", ("GOAL_ALREADY_SATISFIED",)
    if len(plan.policy_sequence) < 2:
        return (
            "single_policy",
            (
                "SINGLE_POLICY_INTERIOR_EXECUTION_NOT_ROUTED_BY_MULTI_V2",
            ),
        )

    blockers: list[str] = []
    first = plan.steps[0].operator
    last = plan.steps[-1].operator
    policy_operators = [
        item for item in catalog.operators if item.policy_id == last.policy_id
    ]
    terminal_exit = max(item.exit_order for item in policy_operators)
    if last.exit_order != terminal_exit:
        blockers.append(
            "FINAL_OPERATOR_IS_NOT_POLICY_TAIL_NO_LEARNED_DONE_TOKEN"
        )

    # Current physical Multi-V2 startup still uses the reviewed common prep
    # envelope and initial open-gripper contract.  Planning remains available
    # for other initial states, but the web must not arm those plans yet.
    if request.initial_state.holding != "none":
        blockers.append("INITIAL_HOLDING_STATE_NOT_SUPPORTED_BY_WEB_V1")
    if request.initial_state.gripper != "open":
        blockers.append("INITIAL_GRIPPER_MUST_BE_OPEN_FOR_WEB_V1")
    if request.initial_state.contact_mode != "free_space":
        blockers.append("INITIAL_CONTACT_MODE_MUST_BE_FREE_SPACE_FOR_WEB_V1")

    first_policy_min_entry = min(
        item.entry_order
        for item in catalog.operators
        if item.policy_id == first.policy_id
    )
    if first.entry_order != first_policy_min_entry:
        blockers.append("INITIAL_INTERIOR_POLICY_REENTRY_NOT_START_CERTIFIED")

    return "multi_v2", tuple(blockers)


def plan_level2_episode(
    catalog: LoadedOperatorCatalog,
    edge_registry: EdgeRuntimeRegistry,
    request: EpisodePlanRequest,
) -> EpisodePlanDecision:
    """Run authoritative Level-2-only UCS for one requested episode."""

    search = uniform_cost_search_level2(
        request.initial_state,
        request.goal,
        _operators_for_request(catalog, request),
        edge_registry,
        cost_config=request.cost_config,
        max_plans=request.max_plans,
        bridge_admission_mode=request.bridge_admission_mode,
        allow_intermediate_release=request.allow_intermediate_release,
    )
    plan = search.best_plan
    execution_kind, blockers = _runtime_classification(
        plan,
        catalog=catalog,
        request=request,
    )
    return EpisodePlanDecision(
        request=request,
        search=search,
        selected_plan=plan,
        execution_kind=execution_kind,
        runtime_blockers=blockers,
    )


def compile_episode_decision(
    decision: EpisodePlanDecision,
    *,
    catalog: LoadedOperatorCatalog,
    edge_registry: EdgeRuntimeRegistry,
    output_path: str | Path,
    composition_id: str,
) -> Path:
    """Compile an eligible authoritative decision into Multi-V2 JSON."""

    if not decision.runtime_compilable or decision.selected_plan is None:
        reasons = ",".join(decision.runtime_blockers) or decision.execution_kind
        raise ValueError(f"episode plan is not Multi-V2 compilable: {reasons}")
    compiled = compile_plan_to_multi_v2(
        decision.selected_plan,
        catalog,
        edge_registry,
        composition_id=composition_id,
        config=MultiV2CompilationConfig(
            final_inference_steps=decision.request.final_inference_steps,
        ),
        expected_bridge_admission_mode=(
            decision.request.bridge_admission_mode
        ),
    )
    mapping = copy.deepcopy(dict(compiled.mapping))
    provenance = dict(mapping["planner_provenance"])
    provenance.update(
        {
            "web_request": decision.request.to_record(),
            "initial_state": decision.request.initial_state.to_record(),
            "goal": dict(decision.request.goal),
            "level2_only": True,
            "forward_only": True,
            "selected_plan_replayed_goal_satisfied": goal_satisfied(
                decision.selected_plan.final_state,
                decision.request.goal,
            ),
            "catalog_sha256": _sha256(catalog.config_path),
            "edge_registry_sha256": _sha256(edge_registry.source_path),
            "browser_plan_trusted": False,
            "server_replanned": True,
            "physical_validation_performed": False,
        }
    )
    mapping["planner_provenance"] = provenance
    output = _write_new_json(Path(output_path), mapping)
    # Parse the emitted file immediately; no malformed plan may reach a runner.
    MultiStagePlan.load(output)
    return output


def decision_fingerprint(
    request: EpisodePlanRequest,
    *,
    catalog: LoadedOperatorCatalog,
    edge_registry: EdgeRuntimeRegistry,
) -> str:
    value = {
        "request": request.to_record(),
        "catalog_sha256": _sha256(catalog.config_path),
        "registry_sha256": _sha256(edge_registry.source_path),
    }
    return hashlib.sha256(_canonical_json(value).encode("utf-8")).hexdigest()
