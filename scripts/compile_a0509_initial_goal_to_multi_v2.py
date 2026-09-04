#!/usr/bin/env python3
"""Compile an arbitrary operator-confirmed initial/goal request, command-free."""

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
from lerobot_robot_doosan_a0509.interior_policy.episode_control import (  # noqa: E402
    EpisodePlanRequest,
    compile_episode_decision,
    decision_fingerprint,
    plan_level2_episode,
)
from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (  # noqa: E402
    EdgeRuntimeRegistry,
)
from lerobot_robot_doosan_a0509.interior_policy.web_runtime_manifest import (  # noqa: E402
    build_web_runtime_support_manifest,
)
from lerobot_robot_doosan_a0509.task_c_handoff.runtime_command_profile import (  # noqa: E402
    get_runtime_command_profile,
)


DEFAULT_CATALOG = (
    REPOSITORY_ROOT
    / "config/interior_policy/a0509_t1_t8_operator_catalog_spatial_v2.json"
)
DEFAULT_REGISTRY = (
    REPOSITORY_ROOT
    / "docs/artifacts/t1_t8_spatial_floor_level2_2026-09-02/"
    "edge_registry_level2_spatial_v4.json"
)
DEFAULT_TEMPLATE = (
    REPOSITORY_ROOT
    / "docs/artifacts/task_c_t2_t3_floor_to_floor_v2_2026-08-25/"
    "v2_fail_closed_support_runtime_manifest_t2_t3.json"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--request", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--edge-registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument("--runtime-template", type=Path, default=DEFAULT_TEMPLATE)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    raw = json.loads(args.request.expanduser().resolve().read_text(encoding="utf-8"))
    catalog = load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        config_path=args.catalog,
        audit_raw_frames=False,
    )
    registry = EdgeRuntimeRegistry.load(args.edge_registry)
    request = EpisodePlanRequest.from_mapping(raw, catalog=catalog)
    decision = plan_level2_episode(catalog, registry, request)
    result = decision.to_record()
    fingerprint = decision_fingerprint(
        request,
        catalog=catalog,
        edge_registry=registry,
    )
    result["fingerprint"] = fingerprint
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    result_path = output_dir / "authoritative_plan_result.json"
    result_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print("A0509_INITIAL_GOAL_PLAN_SCOPE=OFFLINE_COMMAND_FREE")
    print("LEVEL2_ONLY=true FORWARD_ONLY=true")
    if decision.selected_plan is None:
        print("RESULT=NO_PLAN")
        print(f"OUTPUT={result_path}")
        raise SystemExit(2)
    print("OPERATORS=" + " -> ".join(decision.selected_plan.operator_ids))
    print("POLICIES=" + " -> ".join(decision.selected_plan.policy_sequence))
    print(f"TOTAL_COST={decision.selected_plan.total_cost:.3f}")
    print(f"EXECUTION_KIND={decision.execution_kind}")
    if decision.runtime_blockers:
        print("RUNTIME_BLOCKERS=" + ",".join(decision.runtime_blockers))
        print(f"OUTPUT={result_path}")
        raise SystemExit(3)

    composition_id = f"web_dijkstra_{fingerprint[:16]}"
    plan_path = compile_episode_decision(
        decision,
        catalog=catalog,
        edge_registry=registry,
        output_path=output_dir / "multi_stage_plan.json",
        composition_id=composition_id,
    )
    runtime_path = build_web_runtime_support_manifest(
        multi_stage_plan_path=plan_path,
        template_path=args.runtime_template,
        output_path=output_dir / "base_runtime_support_manifest.json",
        command_profile=get_runtime_command_profile(
            registry.runtime_command_profile_id
        ),
    )
    print("RUNTIME_COMPILABLE=true")
    print(f"MULTI_STAGE_PLAN={plan_path}")
    print(f"BASE_RUNTIME_MANIFEST={runtime_path}")
    print("PHYSICAL_VALIDATION_PERFORMED=false")
    print(f"OUTPUT={result_path}")


if __name__ == "__main__":
    main()
