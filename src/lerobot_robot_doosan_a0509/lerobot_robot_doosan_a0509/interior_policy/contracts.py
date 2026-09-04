"""Hashable symbolic contracts for interior intervals of frozen ACT policies."""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from typing import Any, Mapping


_ALLOWED = {
    "drawer": frozenset({"open", "closed", "unknown"}),
    "white_container": frozenset({"open", "closed", "unknown"}),
    "blue_block_location": frozenset(
        {
            "floor",
            "left_floor",
            "right_floor",
            "black_table",
            "drawer",
            "white_container",
            "held",
            "moving_source",
            "stacked",
            "drawer_top",
            "unknown",
        }
    ),
    "holding": frozenset({"none", "blue_block", "unknown"}),
    "gripper": frozenset({"open", "closed", "unknown"}),
    "contact_mode": frozenset(
        {"free_space", "free_transport", "contact_manipulation", "unknown"}
    ),
    "support_blue_block_location": frozenset(
        {"black_table", "absent", "unknown"}
    ),
    "stack": frozenset({"unassembled", "assembled", "unknown"}),
}

STATE_FACT_FIELDS = tuple(_ALLOWED)
POLICY_IDS = ("T1", "T2", "T3", "T4", "T5", "T6", "T7", "T8")
_CURSOR_FIELD = {policy: f"cursor_{policy.lower()}" for policy in POLICY_IDS}


def _facts(value: Mapping[str, object]) -> tuple[tuple[str, str], ...]:
    result: list[tuple[str, str]] = []
    for key, raw in value.items():
        if key not in STATE_FACT_FIELDS:
            raise ValueError(f"unsupported symbolic fact field: {key}")
        item = str(raw)
        if item not in _ALLOWED[key]:
            choices = ",".join(sorted(_ALLOWED[key]))
            raise ValueError(f"invalid {key}={item!r}; expected one of {choices}")
        result.append((key, item))
    return tuple(sorted(result))


@dataclass(frozen=True)
class WorldState:
    """Immutable symbolic node used by uniform-cost search.

    ``unknown`` is a real value, not a wildcard.  It therefore never satisfies
    a concrete precondition such as ``drawer=open``.
    """

    drawer: str
    white_container: str
    blue_block_location: str
    holding: str
    gripper: str
    contact_mode: str
    support_blue_block_location: str = "unknown"
    stack: str = "unknown"

    cursor_t1: int = 0
    cursor_t2: int = 0
    cursor_t3: int = 0
    cursor_t4: int = 0
    cursor_t5: int = 0
    cursor_t6: int = 0
    cursor_t7: int = 0
    cursor_t8: int = 0

    last_policy: str | None = None

    def __post_init__(self) -> None:
        for field, choices in _ALLOWED.items():
            value = str(getattr(self, field))
            if value not in choices:
                allowed = ",".join(sorted(choices))
                raise ValueError(f"invalid {field}={value!r}; expected {allowed}")
        for policy, field in _CURSOR_FIELD.items():
            value = getattr(self, field)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{field} must be a non-negative integer")
        if self.last_policy is not None and self.last_policy not in POLICY_IDS:
            raise ValueError("last_policy must be T1..T8 or None")

    def cursor_for(self, policy_id: str) -> int:
        try:
            field = _CURSOR_FIELD[policy_id]
        except KeyError as exc:
            raise ValueError(f"unknown policy_id: {policy_id}") from exc
        return int(getattr(self, field))

    def with_cursor(self, policy_id: str, value: int) -> "WorldState":
        if value < self.cursor_for(policy_id):
            raise ValueError("policy cursor cannot move backward")
        return replace(self, **{_CURSOR_FIELD[policy_id]: int(value)})

    def to_record(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class SemanticIntervalEvidence:
    """Traceable evidence for an operator interval, not an execution claim."""

    dataset_root: str
    semantic_artifact: str
    phase_support_artifact: str
    checkpoint: str
    semantic_segment_ids: tuple[str, ...]
    entry_segment: str
    entry_phase: float
    entry_anchor: str
    exit_segment: str
    exit_phase: float
    exit_anchor: str
    artifact_relaxations: tuple[str, ...] = ()
    held_object_aliases: tuple[tuple[str, str], ...] = ()
    physical_setup_overrides: tuple[str, ...] = ()
    certified_source_boundary_manifest: str | None = None

    def __post_init__(self) -> None:
        if not self.semantic_segment_ids:
            raise ValueError("semantic interval requires at least one segment")
        if self.entry_segment not in self.semantic_segment_ids:
            raise ValueError("entry segment must belong to the interval")
        if self.exit_segment not in self.semantic_segment_ids:
            raise ValueError("exit segment must belong to the interval")
        for name, phase in (
            ("entry_phase", self.entry_phase),
            ("exit_phase", self.exit_phase),
        ):
            if not 0.0 <= float(phase) <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        alias_keys = [raw for raw, _canonical in self.held_object_aliases]
        if len(alias_keys) != len(set(alias_keys)):
            raise ValueError("held-object aliases must have unique source labels")
        for raw, canonical in self.held_object_aliases:
            if not raw:
                raise ValueError("held-object alias source must not be empty")
            if canonical not in _ALLOWED["holding"]:
                raise ValueError(
                    "held-object alias target must be a valid planner holding value"
                )

    def to_record(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class InteriorPolicyOperator:
    """One forward-only semantic manipulation interval of a frozen policy."""

    id: str
    policy_id: str
    entry_order: int
    exit_order: int
    preconditions: tuple[tuple[str, str], ...]
    effects: tuple[tuple[str, str], ...]
    entry_contact_mode: str
    exit_contact_mode: str
    bridge_mode: str | None
    base_cost: float
    evidence: SemanticIntervalEvidence

    def __post_init__(self) -> None:
        if self.policy_id not in POLICY_IDS:
            raise ValueError("operator policy_id must be T1..T8")
        if not self.id.startswith(f"{self.policy_id}."):
            raise ValueError("operator id must start with '<policy_id>.'")
        if self.entry_order < 0 or self.exit_order <= self.entry_order:
            raise ValueError("operator order must make strict forward progress")
        if self.entry_contact_mode not in _ALLOWED["contact_mode"]:
            raise ValueError("invalid entry_contact_mode")
        if self.exit_contact_mode not in _ALLOWED["contact_mode"]:
            raise ValueError("invalid exit_contact_mode")
        if self.base_cost <= 0.0:
            raise ValueError("operator base_cost must be positive")
        # Revalidate tuple-backed facts and reject duplicate keys.
        for name, facts in (("preconditions", self.preconditions), ("effects", self.effects)):
            if tuple(facts) != _facts(dict(facts)) or len(dict(facts)) != len(facts):
                raise ValueError(f"{name} must be sorted unique symbolic facts")
        pre = dict(self.preconditions)
        effects = dict(self.effects)
        if pre.get("contact_mode", self.entry_contact_mode) != self.entry_contact_mode:
            raise ValueError("entry contact mode conflicts with preconditions")
        if effects.get("contact_mode", self.exit_contact_mode) != self.exit_contact_mode:
            raise ValueError("exit contact mode conflicts with effects")

    @classmethod
    def create(
        cls,
        *,
        id: str,
        policy_id: str,
        entry_order: int,
        exit_order: int,
        preconditions: Mapping[str, object],
        effects: Mapping[str, object],
        entry_contact_mode: str,
        exit_contact_mode: str,
        bridge_mode: str | None,
        base_cost: float,
        evidence: SemanticIntervalEvidence,
    ) -> "InteriorPolicyOperator":
        return cls(
            id=id,
            policy_id=policy_id,
            entry_order=int(entry_order),
            exit_order=int(exit_order),
            preconditions=_facts(preconditions),
            effects=_facts(effects),
            entry_contact_mode=str(entry_contact_mode),
            exit_contact_mode=str(exit_contact_mode),
            bridge_mode=None if bridge_mode is None else str(bridge_mode),
            base_cost=float(base_cost),
            evidence=evidence,
        )

    @property
    def precondition_map(self) -> dict[str, str]:
        return dict(self.preconditions)

    @property
    def effect_map(self) -> dict[str, str]:
        return dict(self.effects)

    def to_record(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "policy_id": self.policy_id,
            "entry_order": self.entry_order,
            "exit_order": self.exit_order,
            "preconditions": self.precondition_map,
            "effects": self.effect_map,
            "entry_contact_mode": self.entry_contact_mode,
            "exit_contact_mode": self.exit_contact_mode,
            "bridge_mode": self.bridge_mode,
            "base_cost": self.base_cost,
            "evidence": self.evidence.to_record(),
        }


def apply_operator_effects(
    state: WorldState,
    operator: InteriorPolicyOperator,
) -> WorldState:
    """Apply partial effects with a frame axiom and monotonic policy cursor."""

    current_cursor = state.cursor_for(operator.policy_id)
    if operator.entry_order < current_cursor:
        raise ValueError(
            f"forward-only violation: {operator.id} entry={operator.entry_order} "
            f"cursor={current_cursor}"
        )
    updated = replace(
        state,
        **operator.effect_map,
        **{_CURSOR_FIELD[operator.policy_id]: operator.exit_order},
        last_policy=operator.policy_id,
    )
    return updated
