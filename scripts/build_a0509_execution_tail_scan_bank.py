#!/usr/bin/env python3
"""Build recorded support for a shared execution-tail scan, without robot I/O.

The runtime manifest keeps its data-derived commit window.  This audit bank
extends only from that window's lower edge to the common data-derived scan
deadline so every recorded episode can be replayed through the same retry
logic.  It is evidence, not a wider semantic commit window.
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
from lerobot_robot_doosan_a0509.interior_policy.execution_tail import (  # noqa: E402
    ExecutionTailDerivationConfig,
    derive_zero_cost_execution_tail,
)
from lerobot_robot_doosan_a0509.interior_policy.execution_tail_reference import (  # noqa: E402
    build_runtime_reference_bank,
)


DEFAULT_CATALOG = (
    REPOSITORY_ROOT
    / "config/interior_policy/a0509_t1_t6_operator_catalog_v1.json"
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-operator", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument(
        "--allow-reviewed-empty-gripper-tail",
        action="store_true",
    )
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def _write(path: Path, value: Mapping[str, Any], *, force: bool) -> None:
    resolved = path.expanduser().resolve()
    if resolved.exists() and not force:
        raise FileExistsError(f"refusing to overwrite scan bank: {resolved}")
    resolved.parent.mkdir(parents=True, exist_ok=True)
    resolved.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    args = _parse_args()
    catalog = load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        config_path=args.catalog,
        audit_raw_frames=False,
    )
    operator = catalog.by_id[str(args.source_operator)]
    config = (
        ExecutionTailDerivationConfig(
            reviewed_empty_gripper_free_space_operators=(operator.id,),
        )
        if args.allow_reviewed_empty_gripper_tail
        else None
    )
    result = derive_zero_cost_execution_tail(operator, config=config)
    if result.reason != "derived" or result.profile is None:
        raise RuntimeError(
            f"{operator.id} execution tail unavailable: {result.reason}"
        )
    tail = result.profile
    audit = catalog.audit.tasks[operator.policy_id]
    bank = build_runtime_reference_bank(
        reference_id=(
            f"{operator.id}:{tail.tracking_segment}_latched_scan_evidence"
        ),
        role="source_execution_tail",
        source_operator=operator.id,
        task_id=operator.policy_id,
        dataset_root=audit.dataset_root,
        semantic_graph_path=audit.semantic_artifact,
        phase_support_artifact=audit.phase_support_artifact,
        segment=tail.tracking_segment,
        phase_window=(tail.commit_phase_low, tail.deadline_phase),
        nominal_phase=tail.nominal_phase,
    )
    bank["scan_contract"] = {
        "purpose": "command_free_latched_retry_evidence_only",
        "primary_commit_phase_window": [
            tail.commit_phase_low,
            tail.commit_phase_high,
        ],
        "scan_phase_window": [tail.commit_phase_low, tail.deadline_phase],
        "commit_window_widened": False,
        "execution_tail": tail.to_record(),
        "edge_specific_numeric_tuning": False,
        "robot_commands_published": 0,
        "physical_validation_performed": False,
    }
    _write(args.output, bank, force=args.force)
    print(
        "EXECUTION_TAIL_SCAN_BANK_OK "
        f"source={operator.id} segment={tail.tracking_segment} "
        f"window={tail.commit_phase_low:.3f}:{tail.deadline_phase:.3f} "
        f"episodes={bank['episode_count']} "
        f"output={args.output.expanduser().resolve()} "
        "robot_commands_published=0"
    )


if __name__ == "__main__":
    main()
