#!/usr/bin/env python3
"""Compile the validated controlled UCS result into a command-free Multi-V2 plan."""

from __future__ import annotations

import argparse
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
from lerobot_robot_doosan_a0509.interior_policy.multi_v2_compiler import (  # noqa: E402
    EdgeRuntimeRegistry,
    MultiV2CompilationConfig,
    compile_controlled_catalog_plan,
)
from lerobot_robot_doosan_a0509.task_c_handoff.multi_stage import (  # noqa: E402
    MultiStagePlan,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (  # noqa: E402
    BridgeAdmissionMode,
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
    parser.add_argument(
        "--edge-registry",
        type=Path,
        default=(
            REPOSITORY_ROOT
            / "docs/artifacts/t1_t8_spatial_floor_level2_2026-09-02"
            / "edge_registry_level2_spatial_v4.json"
        ),
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--composition-id",
        default="dijkstra_t4_open_t2_pick_t4_deliver_close",
    )
    parser.add_argument("--final-inference-steps", type=int, default=1800)
    parser.add_argument("--phase-half-width", type=float, default=0.05)
    parser.add_argument("--prearm-extra-phase", type=float, default=0.02)
    parser.add_argument("--persistence-ticks", type=int, default=3)
    parser.add_argument("--local-search-radius-indices", type=int, default=5)
    parser.add_argument("--backward-tolerance", type=float, default=0.02)
    parser.add_argument("--support-loo-quantile", type=float, default=0.95)
    parser.add_argument("--support-distance-threshold-mm", type=float)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    catalog = load_operator_catalog(
        repository_root=REPOSITORY_ROOT,
        config_path=args.catalog,
        audit_raw_frames=False,
    )
    registry = EdgeRuntimeRegistry.load(args.edge_registry)
    compiled = compile_controlled_catalog_plan(
        catalog,
        registry,
        composition_id=args.composition_id,
        bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
        config=MultiV2CompilationConfig(
            final_inference_steps=args.final_inference_steps,
            phase_half_width=args.phase_half_width,
            prearm_extra_phase=args.prearm_extra_phase,
            persistence_ticks=args.persistence_ticks,
            local_search_radius_indices=args.local_search_radius_indices,
            backward_tolerance=args.backward_tolerance,
            support_distance_threshold_mm=(
                args.support_distance_threshold_mm
            ),
            support_loo_quantile=args.support_loo_quantile,
        ),
    )
    output = compiled.write_json(args.output)
    parsed = MultiStagePlan.load(output)
    print("DIJKSTRA_MULTI_V2_COMPILE_SCOPE=OFFLINE_COMMAND_FREE")
    print("OPERATORS=" + " -> ".join(compiled.mapping["planner_provenance"]["operators"]))
    print("POLICIES=" + " -> ".join(stage.policy_id for stage in parsed.stages))
    print(f"STAGES={len(parsed.stages)} EDGES={len(parsed.transitions)}")
    print(f"FINAL_INFERENCE_STEPS={parsed.stages[-1].inference_end_steps}")
    print("Z_MINIMUM_ENABLED=false")
    print("PHYSICAL_VALIDATION_PERFORMED=false")
    print(f"OUTPUT={output}")


if __name__ == "__main__":
    main()
