#!/usr/bin/env python3
"""Compile every reviewed edge under its registry-selected tail profile.

This audit is command-free: it builds two-step symbolic witness plans, loads
the existing manifests and policy-shadow reports, and runs only the offline
Multi-V2 compiler.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Mapping


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "src/lerobot_robot_doosan_a0509"
for value in (str(REPOSITORY_ROOT), str(PACKAGE_ROOT)):
    if value not in sys.path:
        sys.path.insert(0, value)

from lerobot_robot_doosan_a0509.interior_policy.catalog import (  # noqa: E402
    load_operator_catalog,
)
from lerobot_robot_doosan_a0509.interior_policy.contracts import (  # noqa: E402
    WorldState,
    apply_operator_effects,
)
from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (  # noqa: E402
    FLEXIBLE_VERIFIED,
    EdgeRuntimeRegistry,
    compile_plan_to_multi_v2,
)
from lerobot_robot_doosan_a0509.interior_policy.planner import (  # noqa: E402
    CostBreakdown,
    Plan,
    PlanStep,
    operator_applicability,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (  # noqa: E402
    BridgeAdmissionMode,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--catalog", type=Path, default=None)
    parser.add_argument("--inventory", type=Path, default=None)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.expanduser().resolve().read_text(encoding="utf-8"))


def _write(path: Path, value: Mapping[str, Any], *, force: bool) -> None:
    resolved = path.expanduser().resolve()
    if resolved.exists() and not force:
        raise FileExistsError(f"refusing to overwrite audit: {resolved}")
    resolved.parent.mkdir(parents=True, exist_ok=True)
    resolved.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _two_step_plan(witness: Mapping[str, Any], catalog: Any) -> Plan:
    source = catalog.by_id[str(witness["source_operator"])]
    successor = catalog.by_id[str(witness["successor_operator"])]
    state_before = WorldState(**dict(witness["witness_state_before"]))
    if not operator_applicability(state_before, source).valid:
        raise ValueError("source operator is invalid for inventory witness")
    state_after_source = apply_operator_effects(state_before, source)
    if not operator_applicability(state_after_source, successor).valid:
        raise ValueError("successor operator is invalid for inventory witness")
    state_after_successor = apply_operator_effects(state_after_source, successor)
    source_cost = CostBreakdown(
        base_cost=source.base_cost,
        policy_switch_penalty=0.0,
        same_policy_continuation_penalty=0.0,
        transition_cost=0.0,
    )
    successor_cost = CostBreakdown(
        base_cost=successor.base_cost,
        policy_switch_penalty=catalog.target.cost_config.policy_switch_penalty,
        same_policy_continuation_penalty=0.0,
        transition_cost=0.0,
    )
    steps = (
        PlanStep(source, state_before, state_after_source, source_cost),
        PlanStep(
            successor,
            state_after_source,
            state_after_successor,
            successor_cost,
        ),
    )
    return Plan(
        steps=steps,
        total_cost=sum(step.cost.total for step in steps),
        final_state=state_after_successor,
    )


def main() -> None:
    args = _parse_args()
    registry_path = args.registry.expanduser().resolve()
    raw_registry = _load(registry_path)
    contract = dict(raw_registry.get("selection_contract", {}))
    catalog_path = (
        Path(args.catalog).expanduser().resolve()
        if args.catalog is not None
        else Path(str(contract["source_catalog"])).expanduser().resolve()
    )
    inventory_path = (
        Path(args.inventory).expanduser().resolve()
        if args.inventory is not None
        else Path(str(contract["source_inventory"])).expanduser().resolve()
    )
    catalog = load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        config_path=catalog_path,
        audit_raw_frames=False,
    )
    registry = EdgeRuntimeRegistry.load(registry_path)
    inventory = _load(inventory_path)
    witnesses = {
        (str(item["source_operator"]), str(item["successor_operator"])): item
        for item in inventory["edges"]
    }
    verified = [
        item for item in registry.edges if item.admission_status == FLEXIBLE_VERIFIED
    ]
    results: list[dict[str, Any]] = []
    failures: list[dict[str, str]] = []
    for index, spec in enumerate(verified):
        try:
            plan = _two_step_plan(witnesses[spec.key], catalog)
            compiled = compile_plan_to_multi_v2(
                plan,
                catalog,
                registry,
                composition_id=f"shared_profile_edge_audit_{index:03d}",
                expected_bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
            )
            if (
                compiled.mapping["runtime_command_profile_id"]
                != registry.runtime_command_profile_id
            ):
                raise ValueError("compiled runtime command profile differs")
            transition = compiled.mapping["transitions"][0]
            tail = transition["source_execution_tail"]
            if tail is not None and (
                f"profile:{registry.execution_tail_profile_id}"
                not in str(tail["derivation_method"])
            ):
                raise ValueError("compiled tail profile provenance differs")
            results.append(
                {
                    "source_operator": spec.source_operator,
                    "successor_operator": spec.successor_operator,
                    "source_reference_mode": spec.source_reference_mode,
                    "execution_tail": tail,
                }
            )
        except Exception as exc:  # Audit must report every edge, not stop first.
            failures.append(
                {
                    "source_operator": spec.source_operator,
                    "successor_operator": spec.successor_operator,
                    "error": f"{type(exc).__name__}: {exc}",
                }
            )

    tails = [item["execution_tail"] for item in results if item["execution_tail"]]
    report = {
        "schema_version": "a0509.shared_profile_registry_compile_audit.v1",
        "registry": str(registry_path),
        "catalog": str(catalog_path),
        "inventory": str(inventory_path),
        "execution_tail_profile_id": registry.execution_tail_profile_id,
        "runtime_command_profile_id": registry.runtime_command_profile_id,
        "runtime_command_profile": contract.get("runtime_command_profile"),
        "summary": {
            "verified_edge_count": len(verified),
            "compiled_edge_count": len(results),
            "failed_edge_count": len(failures),
            "execution_tail_edge_count": len(tails),
            "exact_or_non_tail_edge_count": len(results) - len(tails),
            "all_verified_edges_compiled": not failures,
        },
        "data_derived_source_profiles": {
            item["source_operator"]: {
                "commit_phase_window": [
                    item["execution_tail"]["commit_phase_low"],
                    item["execution_tail"]["commit_phase_high"],
                ],
                "deadline_phase": item["execution_tail"]["deadline_phase"],
                "latch_commit_window_until_deadline": item[
                    "execution_tail"
                ]["latch_commit_window_until_deadline"],
            }
            for item in results
            if item["execution_tail"] is not None
        },
        "failures": failures,
        "edges": results,
        "robot_commands_published": 0,
        "live_enabled": False,
        "mux_selected": False,
        "physical_validation_performed": False,
    }
    _write(args.output, report, force=args.force)
    print(
        "SHARED_PROFILE_REGISTRY_AUDIT_COMPLETE "
        f"compiled={len(results)}/{len(verified)} tails={len(tails)} "
        f"failures={len(failures)} output={args.output.expanduser().resolve()} "
        "robot_commands_published=0"
    )
    if failures:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
