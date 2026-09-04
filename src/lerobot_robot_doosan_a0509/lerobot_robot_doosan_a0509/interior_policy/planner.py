"""Deterministic heapq uniform-cost planner for interior policy operators."""

from __future__ import annotations

import heapq
import itertools
from collections import Counter
from dataclasses import dataclass, field
from typing import Mapping, Protocol, Sequence

from .contracts import (
    STATE_FACT_FIELDS,
    InteriorPolicyOperator,
    WorldState,
    apply_operator_effects,
)


class TransitionCostEstimator(Protocol):
    def __call__(
        self,
        previous_operator: InteriorPolicyOperator | None,
        next_operator: InteriorPolicyOperator,
        state: WorldState,
    ) -> float: ...


class TransitionAdmission(Protocol):
    """Decide whether one operator may follow another in this search mode.

    The default planner remains purely symbolic. A runtime-constrained caller
    can inject a command-free edge-registry admission function without making
    the planner depend on Task-C, ROS, or the 30 Hz control path.
    """

    def __call__(
        self,
        previous_operator: InteriorPolicyOperator | None,
        next_operator: InteriorPolicyOperator,
        state: WorldState,
    ) -> "Applicability": ...


def zero_transition_cost(
    _previous_operator: InteriorPolicyOperator | None,
    _next_operator: InteriorPolicyOperator,
    _state: WorldState,
) -> float:
    return 0.0


def allow_all_transitions(
    _previous_operator: InteriorPolicyOperator | None,
    _next_operator: InteriorPolicyOperator,
    _state: WorldState,
) -> "Applicability":
    return Applicability(valid=True, reasons=())


@dataclass(frozen=True)
class PlannerCostConfig:
    policy_switch_penalty: float = 0.25
    same_policy_continuation_penalty: float = 0.0

    def __post_init__(self) -> None:
        if self.policy_switch_penalty < 0.0:
            raise ValueError("policy_switch_penalty must be non-negative")
        if self.same_policy_continuation_penalty < 0.0:
            raise ValueError("same_policy_continuation_penalty must be non-negative")


@dataclass(frozen=True)
class Applicability:
    valid: bool
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class CostBreakdown:
    base_cost: float
    policy_switch_penalty: float
    same_policy_continuation_penalty: float
    transition_cost: float

    @property
    def total(self) -> float:
        return (
            self.base_cost
            + self.policy_switch_penalty
            + self.same_policy_continuation_penalty
            + self.transition_cost
        )

    def to_record(self) -> dict[str, float]:
        return {
            "base_cost": self.base_cost,
            "policy_switch_penalty": self.policy_switch_penalty,
            "same_policy_continuation_penalty": self.same_policy_continuation_penalty,
            "transition_cost": self.transition_cost,
            "total": self.total,
        }


@dataclass(frozen=True)
class PlanStep:
    operator: InteriorPolicyOperator
    state_before: WorldState
    state_after: WorldState
    cost: CostBreakdown

    def to_record(self) -> dict[str, object]:
        return {
            "operator": self.operator.id,
            "policy_id": self.operator.policy_id,
            "cost": self.cost.to_record(),
            "state_before": self.state_before.to_record(),
            "state_after": self.state_after.to_record(),
        }


@dataclass(frozen=True)
class Plan:
    steps: tuple[PlanStep, ...]
    total_cost: float
    final_state: WorldState

    @property
    def operator_ids(self) -> tuple[str, ...]:
        return tuple(step.operator.id for step in self.steps)

    @property
    def policy_sequence(self) -> tuple[str, ...]:
        return collapse_policy_sequence(
            tuple(step.operator.policy_id for step in self.steps)
        )

    def to_record(self) -> dict[str, object]:
        return {
            "operators": list(self.operator_ids),
            "policy_sequence": list(self.policy_sequence),
            "total_cost": self.total_cost,
            "steps": [step.to_record() for step in self.steps],
            "final_state": self.final_state.to_record(),
        }


@dataclass(frozen=True)
class PlannerDiagnostics:
    expanded_states: int
    generated_states: int
    stale_frontier_entries: int
    rejected_by_operator: Mapping[str, Mapping[str, int]]

    def to_record(self) -> dict[str, object]:
        return {
            "expanded_states": self.expanded_states,
            "generated_states": self.generated_states,
            "stale_frontier_entries": self.stale_frontier_entries,
            "rejected_by_operator": {
                key: dict(value) for key, value in self.rejected_by_operator.items()
            },
        }


@dataclass(frozen=True)
class SearchResult:
    plans: tuple[Plan, ...]
    diagnostics: PlannerDiagnostics

    @property
    def best_plan(self) -> Plan | None:
        return None if not self.plans else self.plans[0]

    @property
    def no_plan(self) -> bool:
        return not self.plans

    def to_record(self) -> dict[str, object]:
        return {
            "status": "NO_PLAN" if self.no_plan else "PLAN_FOUND",
            "plans": [plan.to_record() for plan in self.plans],
            "diagnostics": self.diagnostics.to_record(),
        }


@dataclass(order=True)
class _FrontierItem:
    priority: tuple[float, int, tuple[str, ...], int]
    state: WorldState = field(compare=False)
    steps: tuple[PlanStep, ...] = field(compare=False)


def collapse_policy_sequence(policy_ids: Sequence[str]) -> tuple[str, ...]:
    result: list[str] = []
    for policy_id in policy_ids:
        if not result or result[-1] != policy_id:
            result.append(policy_id)
    return tuple(result)


def goal_satisfied(state: WorldState, goal: Mapping[str, object]) -> bool:
    for field_name, expected in goal.items():
        if field_name not in STATE_FACT_FIELDS:
            raise ValueError(f"goal cannot constrain unsupported field: {field_name}")
        if getattr(state, field_name) != str(expected):
            return False
    return True


def operator_applicability(
    state: WorldState,
    operator: InteriorPolicyOperator,
) -> Applicability:
    reasons: list[str] = []
    cursor = state.cursor_for(operator.policy_id)
    if operator.entry_order < cursor:
        reasons.append(
            f"forward_only:entry_order={operator.entry_order}<cursor={cursor}"
        )
    for field_name, expected in operator.preconditions:
        actual = getattr(state, field_name)
        if actual != expected:
            reasons.append(
                f"precondition:{field_name}:expected={expected}:actual={actual}"
            )
    return Applicability(valid=not reasons, reasons=tuple(reasons))


def _cost_breakdown(
    state: WorldState,
    previous_operator: InteriorPolicyOperator | None,
    operator: InteriorPolicyOperator,
    config: PlannerCostConfig,
    estimator: TransitionCostEstimator,
) -> CostBreakdown:
    switched = state.last_policy is not None and state.last_policy != operator.policy_id
    continued = state.last_policy is not None and state.last_policy == operator.policy_id
    transition_cost = float(estimator(previous_operator, operator, state))
    if transition_cost < 0.0:
        raise ValueError("transition cost estimator returned a negative cost")
    return CostBreakdown(
        base_cost=float(operator.base_cost),
        policy_switch_penalty=config.policy_switch_penalty if switched else 0.0,
        same_policy_continuation_penalty=(
            config.same_policy_continuation_penalty if continued else 0.0
        ),
        transition_cost=transition_cost,
    )


def uniform_cost_search(
    initial_state: WorldState,
    goal: Mapping[str, object],
    operators: Sequence[InteriorPolicyOperator],
    *,
    cost_config: PlannerCostConfig | None = None,
    transition_cost_estimator: TransitionCostEstimator = zero_transition_cost,
    transition_admission: TransitionAdmission = allow_all_transitions,
    max_plans: int = 1,
) -> SearchResult:
    """Return deterministic lowest-cost goal states using Dijkstra/UCS.

    Planning runs at episode/effect/failure frequency.  Nothing in this
    function is intended for the 30 Hz command thread.
    """

    if max_plans < 1:
        raise ValueError("max_plans must be positive")
    config = cost_config or PlannerCostConfig()
    ordered_operators = tuple(sorted(operators, key=lambda item: item.id))
    if len({item.id for item in ordered_operators}) != len(ordered_operators):
        raise ValueError("operator identifiers must be unique")
    # Validate goal keys eagerly, including an already-satisfied goal.
    goal_satisfied(initial_state, goal)

    counter = itertools.count()
    frontier: list[_FrontierItem] = []
    heapq.heappush(
        frontier,
        _FrontierItem((0.0, 0, (), next(counter)), initial_state, ()),
    )
    # Transition admission/cost may depend on the exact previous operator, not
    # only on WorldState.last_policy.  Keep it in the dominance key so a cheap
    # arrival through an incompatible edge cannot suppress a valid arrival.
    best_cost: dict[tuple[WorldState, str | None], float] = {
        (initial_state, None): 0.0
    }
    operator_by_id = {item.id: item for item in ordered_operators}
    rejected: dict[str, Counter[str]] = {
        item.id: Counter() for item in ordered_operators
    }
    expanded = 0
    generated = 1
    stale = 0
    plans: list[Plan] = []

    while frontier and len(plans) < max_plans:
        item = heapq.heappop(frontier)
        current_cost = item.priority[0]
        previous_id = None if not item.steps else item.steps[-1].operator.id
        search_key = (item.state, previous_id)
        if current_cost > best_cost.get(search_key, float("inf")) + 1e-12:
            stale += 1
            continue
        if goal_satisfied(item.state, goal):
            plans.append(
                Plan(
                    steps=item.steps,
                    total_cost=current_cost,
                    final_state=item.state,
                )
            )
            continue
        expanded += 1
        previous = (
            None
            if not item.steps
            else operator_by_id[item.steps[-1].operator.id]
        )
        for operator in ordered_operators:
            applicability = operator_applicability(item.state, operator)
            if not applicability.valid:
                for reason in applicability.reasons:
                    rejected[operator.id][reason] += 1
                continue
            transition = transition_admission(previous, operator, item.state)
            if not transition.valid:
                reasons = transition.reasons or ("transition_rejected",)
                for reason in reasons:
                    rejected[operator.id][f"transition:{reason}"] += 1
                continue
            breakdown = _cost_breakdown(
                item.state,
                previous,
                operator,
                config,
                transition_cost_estimator,
            )
            next_state = apply_operator_effects(item.state, operator)
            next_cost = current_cost + breakdown.total
            next_key = (next_state, operator.id)
            known = best_cost.get(next_key)
            if known is not None and next_cost >= known - 1e-12:
                rejected[operator.id]["dominated_state_cost"] += 1
                continue
            step = PlanStep(operator, item.state, next_state, breakdown)
            steps = item.steps + (step,)
            path = tuple(value.operator.id for value in steps)
            best_cost[next_key] = next_cost
            heapq.heappush(
                frontier,
                _FrontierItem(
                    (next_cost, len(steps), path, next(counter)),
                    next_state,
                    steps,
                ),
            )
            generated += 1

    diagnostics = PlannerDiagnostics(
        expanded_states=expanded,
        generated_states=generated,
        stale_frontier_entries=stale,
        rejected_by_operator={
            key: dict(sorted(value.items())) for key, value in rejected.items()
        },
    )
    return SearchResult(tuple(plans), diagnostics)
