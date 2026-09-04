"""Pure symbolic replay utilities for interior-policy plans."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

from .contracts import InteriorPolicyOperator, WorldState, apply_operator_effects
from .planner import goal_satisfied, operator_applicability


@dataclass(frozen=True)
class ReplayResult:
    states: tuple[WorldState, ...]
    operators: tuple[str, ...]
    goal_satisfied: bool

    @property
    def final_state(self) -> WorldState:
        return self.states[-1]


def replay_operators(
    initial_state: WorldState,
    operators: Sequence[InteriorPolicyOperator],
    goal: Mapping[str, object],
) -> ReplayResult:
    states = [initial_state]
    ids: list[str] = []
    current = initial_state
    for operator in operators:
        applicability = operator_applicability(current, operator)
        if not applicability.valid:
            raise ValueError(
                f"replay operator {operator.id} is not applicable: "
                + ";".join(applicability.reasons)
            )
        current = apply_operator_effects(current, operator)
        states.append(current)
        ids.append(operator.id)
    return ReplayResult(
        states=tuple(states),
        operators=tuple(ids),
        goal_satisfied=goal_satisfied(current, goal),
    )


def _compact_state(state: WorldState) -> str:
    return "\n".join(
        (
            f"drawer={state.drawer}",
            f"white_container={state.white_container}",
            f"block={state.blue_block_location}",
            f"holding={state.holding}",
            f"gripper={state.gripper}",
            f"contact={state.contact_mode}",
            (
                "cursors="
                f"T1:{state.cursor_t1},T2:{state.cursor_t2},T3:{state.cursor_t3},"
                f"T4:{state.cursor_t4},T5:{state.cursor_t5},T6:{state.cursor_t6},"
                f"T7:{state.cursor_t7},T8:{state.cursor_t8}"
            ),
        )
    )


def format_replay(result: ReplayResult) -> str:
    lines: list[str] = []
    for index, state in enumerate(result.states):
        lines.append(f"STEP {index}")
        if index:
            lines.append(f"ACTION {result.operators[index - 1]}")
        lines.append(_compact_state(state))
        lines.append("")
    lines.append("GOAL SATISFIED" if result.goal_satisfied else "GOAL NOT SATISFIED")
    return "\n".join(lines)
