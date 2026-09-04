#!/usr/bin/env python3
"""Offline T1--T8 UCS feasibility probe; never connects to ROS or a robot."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "src/lerobot_robot_doosan_a0509"
for value in (str(REPOSITORY_ROOT), str(PACKAGE_ROOT)):
    if value not in sys.path:
        sys.path.insert(0, value)

from lerobot_robot_doosan_a0509.interior_policy.catalog import (  # noqa: E402
    load_operator_catalog,
)
from lerobot_robot_doosan_a0509.interior_policy.planner import (  # noqa: E402
    operator_applicability,
    uniform_cost_search,
)
from lerobot_robot_doosan_a0509.interior_policy.simulator import (  # noqa: E402
    format_replay,
    replay_operators,
)
from lerobot_robot_doosan_a0509.interior_policy.v2_adapter import (  # noqa: E402
    V2ShadowTransitionValidator,
    runtime_level,
)


EXPECTED_CONTROLLED = (
    "T4.open_drawer",
    "T2.acquire_from_floor",
    "T4.deliver_to_drawer",
    "T4.close_drawer",
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--catalog",
        type=Path,
        default=(
            REPOSITORY_ROOT
            / "config/interior_policy/a0509_t1_t8_operator_catalog_spatial_v2.json"
        ),
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--skip-raw-frame-audit",
        action="store_true",
        help="Skip raw parquet event/frame audit (semantic NPZ is still validated).",
    )
    parser.add_argument("--max-alternatives", type=int, default=12)
    return parser.parse_args()


def _operator_runs(operator_ids: tuple[str, ...]) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    for operator_id in operator_ids:
        policy = operator_id.split(".", 1)[0]
        if result and result[-1]["policy_id"] == policy:
            result[-1]["operators"].append(operator_id)
        else:
            result.append({"policy_id": policy, "operators": [operator_id]})
    return result


def run(args: argparse.Namespace) -> dict[str, object]:
    catalog = load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        config_path=args.catalog,
        audit_raw_frames=not args.skip_raw_frame_audit,
    )
    target = catalog.target
    controlled = uniform_cost_search(
        target.initial_state,
        target.goal,
        catalog.controlled_operators,
        cost_config=target.cost_config,
        max_plans=1,
    )
    full = uniform_cost_search(
        target.initial_state,
        target.goal,
        catalog.operators,
        cost_config=target.cost_config,
        max_plans=args.max_alternatives,
    )
    controlled_plan = controlled.best_plan
    full_plan = full.best_plan
    if controlled_plan is None:
        replay_record = None
        replay_text = "NO CONTROLLED PLAN"
        edge_records: list[dict[str, object]] = []
        code_audit = None
    else:
        replay = replay_operators(
            target.initial_state,
            tuple(step.operator for step in controlled_plan.steps),
            target.goal,
        )
        replay_record = {
            "operators": list(replay.operators),
            "states": [state.to_record() for state in replay.states],
            "goal_satisfied": replay.goal_satisfied,
        }
        replay_text = format_replay(replay)
        validator = V2ShadowTransitionValidator(catalog)
        edges = validator.validate_plan(controlled_plan)
        edge_records = [edge.to_record() for edge in edges]
        code_audit = validator.code_audit.to_record()

    alternative_paths = [list(plan.operator_ids) for plan in full.plans]
    t5_alternative = next(
        (path for path in alternative_paths if path and path[0] == "T5.open_drawer"),
        None,
    )
    full_ids = set() if full_plan is None else set(full_plan.operator_ids)
    selection_analysis = {
        "T1": (
            "white-container effects do not satisfy any target drawer predicate; "
            "positive costs make the detour dominated"
        ),
        "T3": (
            "its acquisition requires block=black_table, while the initial block is "
            "floor; its delivery moves black_table->floor, opposite to the goal"
        ),
        "T5": (
            "T5.open_drawer is a same-cost symbolic alternative; the deterministic "
            "T4 choice is a lexical tie-break, not evidence of higher physical success"
        ),
        "T6": (
            "its acquisition requires moving_source plus a support block and its effect "
            "is stacked, not block=drawer"
        ),
        "T7": (
            "its acquisition requires block=drawer_top, while the initial block is "
            "floor; its delivery target is black_table rather than drawer"
        ),
        "T8": (
            "its acquisition requires an assembled stack on a support block, while "
            "the initial block is floor; its delivery target is black_table"
        ),
        "selected_best_operator_ids": sorted(full_ids),
    }
    backward_reentry_rejected = False
    if controlled_plan is not None and len(controlled_plan.steps) >= 3:
        old_t4_entry = catalog.by_id["T4.open_drawer"]
        backward_status = operator_applicability(
            controlled_plan.steps[2].state_after,
            old_t4_entry,
        )
        backward_reentry_rejected = (
            not backward_status.valid
            and any(
                reason.startswith("forward_only:")
                for reason in backward_status.reasons
            )
        )
    status = {
        "PASS_1_full_catalog_plan_exists": full_plan is not None,
        "PASS_2_controlled_exact_T4_T2_T4": (
            controlled_plan is not None
            and controlled_plan.operator_ids == EXPECTED_CONTROLLED
            and controlled_plan.policy_sequence == ("T4", "T2", "T4")
        ),
        "PASS_3_symbolic_replay_goal": bool(
            replay_record is not None and replay_record["goal_satisfied"]
        ),
        "PASS_4_backward_reentry_rejected": backward_reentry_rejected,
        "PASS_5_V2_edges_classified": len(edge_records) == 2,
    }
    level = runtime_level(
        controlled_plan,
        ()
        if controlled_plan is None
        else V2ShadowTransitionValidator(catalog).validate_plan(controlled_plan),
    )
    result: dict[str, object] = {
        "schema_version": "a0509.interior_policy_symbolic_probe.v1",
        "safety_scope": {
            "robot_motion_performed": False,
            "ros_used": False,
            "live_on_used": False,
            "mux_lerobot_used": False,
            "servo_commands_used": False,
            "gpu_inference_used": False,
        },
        "catalog_id": catalog.catalog_id,
        "catalog_operator_count": len(catalog.operators),
        "catalog_policy_ids": sorted({item.policy_id for item in catalog.operators}),
        "initial_state": target.initial_state.to_record(),
        "goal": dict(target.goal),
        "controlled_search": controlled.to_record(),
        "full_catalog_search": full.to_record(),
        "controlled_replay": replay_record,
        "controlled_execution_runs": (
            [] if controlled_plan is None else _operator_runs(controlled_plan.operator_ids)
        ),
        "T5_alternative": t5_alternative,
        "selection_analysis": selection_analysis,
        "v2_code_audit": code_audit,
        "v2_edge_shadow_validation": edge_records,
        "classification": level,
        "physical_validation_performed": False,
        "pass_conditions": status,
        "artifact_audit": catalog.audit.to_record(),
        "replay_text": replay_text,
    }
    return result


def main() -> None:
    args = _parse_args()
    if args.max_alternatives < 2:
        raise SystemExit("--max-alternatives must be at least 2")
    result = run(args)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    controlled = result["controlled_search"]["plans"]
    full = result["full_catalog_search"]["plans"]
    print("SYMBOLIC_PROBE_SCOPE=OFFLINE_COMMAND_FREE")
    policies = ",".join(result["catalog_policy_ids"])
    print(f"CATALOG_OPERATORS={result['catalog_operator_count']} policies={policies}")
    if controlled:
        plan = controlled[0]
        print("CONTROLLED_OPERATORS=" + " -> ".join(plan["operators"]))
        print("CONTROLLED_POLICIES=" + " -> ".join(plan["policy_sequence"]))
        print(f"CONTROLLED_COST={plan['total_cost']:.3f}")
    else:
        print("CONTROLLED_RESULT=NO_PLAN")
    if full:
        print("FULL_BEST_OPERATORS=" + " -> ".join(full[0]["operators"]))
        print(f"FULL_BEST_COST={full[0]['total_cost']:.3f}")
    else:
        print("FULL_RESULT=NO_PLAN")
    if result["T5_alternative"] is not None:
        print("T5_ALTERNATIVE=" + " -> ".join(result["T5_alternative"]))
    print("\n" + result["replay_text"])
    for edge in result["v2_edge_shadow_validation"]:
        print(
            "V2_EDGE "
            f"{edge['source_operator']}->{edge['successor_operator']} "
            f"type={edge['symbolic']['transition_type']} "
            f"structural={edge['structural_runtime_mapping']} "
            f"exact_manifest={edge['exact_episode_manifest_ready']} "
            f"runtime_contract={edge['runtime_contract_compatible']}"
        )
    print("CLASSIFICATION=" + str(result["classification"]))
    print(
        "PASS_CONDITIONS="
        + json.dumps(result["pass_conditions"], sort_keys=True)
    )
    if args.output is not None:
        print("OUTPUT=" + str(args.output.resolve()))


if __name__ == "__main__":
    main()
