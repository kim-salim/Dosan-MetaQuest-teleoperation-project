#!/usr/bin/env python3
"""Command-free validation and summary for a Task-C multi-stage plan."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from lerobot_robot_doosan_a0509.task_c_handoff.multi_stage import MultiStagePlan


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("plan", type=Path)
    args = parser.parse_args()
    plan = MultiStagePlan.load(args.plan)

    missing_models = [
        str(policy.checkpoint / "model.safetensors")
        for policy in plan.policies
        if not (policy.checkpoint / "model.safetensors").is_file()
    ]
    if missing_models:
        raise SystemExit(
            "missing policy checkpoints:\n" + "\n".join(missing_models)
        )

    value = {
        "schema_version": plan.schema_version,
        "composition_id": plan.composition_id,
        "unique_policy_count": len(plan.policies),
        "stage_visit_count": len(plan.stages),
        "policies": [
            {
                "policy_id": item.policy_id,
                "checkpoint": str(item.checkpoint),
                "reuse_context_policy": item.reuse_context_policy,
            }
            for item in plan.policies
        ],
        "stages": [
            {
                "stage_index": index,
                "stage_id": item.stage_id,
                "policy_id": item.policy_id,
                "exit_authority": item.exit_authority,
                "terminal": item.terminal,
            }
            for index, item in enumerate(plan.stages)
        ],
        "transitions": [
            {
                "transition_id": item.transition_id,
                "source_stage": item.source_stage,
                "successor_stage": item.successor_stage,
                "handoff_id": item.handoff_manifest.handoff_id,
                "handoff_manifest": str(item.handoff_manifest_path),
                "semantic_authority": (
                    item.handoff_manifest.semantic_authority.value
                ),
                "bridge_duration_s": item.handoff_manifest.bridge_duration_s,
                "robot_executable": item.handoff_manifest.robot_executable,
                "dry_run_only": item.handoff_manifest.dry_run_only,
                "ik_checked": item.handoff_manifest.ik_checked,
                "collision_checked": item.handoff_manifest.collision_checked,
            }
            for item in plan.transitions
        ],
        "physical_execution_performed": False,
    }
    print(json.dumps(value, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
