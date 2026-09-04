#!/usr/bin/env python3
"""Build the command-free T2 execution-tail -> T3 interior reference."""

from __future__ import annotations

import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_ROOT = REPOSITORY_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from rebuild_a0509_t7_t3_execution_tail_reference import main  # noqa: E402


if __name__ == "__main__":
    main(
        [
            "--catalog",
            str(
                REPOSITORY_ROOT
                / "config/interior_policy/"
                "a0509_t1_t8_operator_catalog_spatial_v2.json"
            ),
            "--baseline-manifest",
            str(
                REPOSITORY_ROOT
                / "docs/artifacts/t1_t8_flexible_level2_registry_2026-08-30/"
                "edges/t2_acquire_from_floor__to__t3_deliver_to_floor/"
                "flexible_reference_manifest.json"
            ),
            "--output-root",
            str(
                REPOSITORY_ROOT
                / "docs/artifacts/t2_to_t3_s2_runtime_reference_2026-09-03"
            ),
            "--source-operator", "T2.acquire_from_floor",
            "--successor-operator", "T3.deliver_to_floor",
            "--expected-source-segment", "S2",
            "--expected-commit-phase-low", "0.54",
            "--expected-commit-phase-high", "0.60",
            "--selection-rank", "14",
        ] + sys.argv[1:]
    )
