"""Shared authority contract for semantic handoff decisions.

The default keeps the reviewed V2 behavior.  ``external_planner`` means that
the selected episode manifest is the semantic authorization: Bridge/runtime
code still records semantic/support mismatches, but does not use them to
grant or deny control authority.  Geometric, dynamic, freshness, generation,
tracking, MUX, and fail-closed checks remain runtime responsibilities.
"""

from __future__ import annotations

from enum import Enum


class SemanticAuthority(str, Enum):
    RUNTIME_GUARDED = "runtime_guarded"
    EXTERNAL_PLANNER = "external_planner"

    @property
    def runtime_semantic_checks_enforced(self) -> bool:
        return self is SemanticAuthority.RUNTIME_GUARDED


def parse_semantic_authority(value: str | SemanticAuthority) -> SemanticAuthority:
    if isinstance(value, SemanticAuthority):
        return value
    try:
        return SemanticAuthority(str(value).strip().lower())
    except ValueError as exc:
        choices = ",".join(item.value for item in SemanticAuthority)
        raise ValueError(f"semantic_authority must be one of: {choices}") from exc
