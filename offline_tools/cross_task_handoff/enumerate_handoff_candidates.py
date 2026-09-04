"""Enumerate and hard-validate cross-task V2 boundary candidates."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from offline_tools.task_c_bridge_v0.bezier_bridge import (
    CUBIC_BEZIER_FIXED,
    CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
)

from .authority import SemanticAuthority, parse_semantic_authority
from .schema import PhaseIndexPoint
from .validate_handoff_candidates import (
    CandidateValidationConfig,
    validate_candidate,
)


def _load_points(
    path: Path,
    task: str,
    segment: str,
    *,
    phase: float | None = None,
    phase_tolerance: float = 1.0e-9,
) -> list[PhaseIndexPoint]:
    if phase is not None and not 0.0 <= float(phase) <= 1.0:
        raise ValueError("phase filter must be in [0, 1]")
    if phase_tolerance < 0.0:
        raise ValueError("phase_tolerance must be non-negative")
    value = json.loads(path.read_text(encoding="utf-8"))
    if value.get("schema_version") != "a0509.cross_task_phase_index.v2":
        raise ValueError(f"unsupported phase index: {path}")
    points = [PhaseIndexPoint.from_record(item) for item in value["points"]]
    selected = [
        point
        for point in points
        if point.task == task
        and point.segment == segment
        and (
            phase is None
            or abs(point.phase - float(phase)) <= phase_tolerance
        )
    ]
    if not selected:
        suffix = "" if phase is None else f" phase={float(phase):.6f}"
        raise ValueError(f"no points for {task}/{segment}{suffix} in {path}")
    return sorted(
        selected,
        key=lambda point: (
            point.phase,
            point.support_episode,
            point.support_frame,
        ),
    )


def enumerate_candidates(
    *,
    source_points: list[PhaseIndexPoint],
    successor_points: list[PhaseIndexPoint],
    durations_s: list[float],
    config: CandidateValidationConfig,
    transport_floor_mm: float | None,
    max_pairs: int | None,
    semantic_authority: SemanticAuthority | str = SemanticAuthority.RUNTIME_GUARDED,
    bridge_algorithm: str = CUBIC_BEZIER_FIXED,
    minimum_tangent_handle_chord_ratio: float = 0.0,
    maximum_endpoint_speed_adjustment_mm_s: float | None = None,
) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    pairs = 0
    for source in source_points:
        for successor in successor_points:
            if max_pairs is not None and pairs >= max_pairs:
                return output
            pairs += 1
            for duration_s in durations_s:
                output.append(
                    validate_candidate(
                        source,
                        successor,
                        duration_s=duration_s,
                        config=config,
                        transport_floor_mm=transport_floor_mm,
                        semantic_authority=semantic_authority,
                        bridge_algorithm=bridge_algorithm,
                        minimum_tangent_handle_chord_ratio=(
                            minimum_tangent_handle_chord_ratio
                        ),
                        maximum_endpoint_speed_adjustment_mm_s=(
                            maximum_endpoint_speed_adjustment_mm_s
                        ),
                    ).to_record()
                )
    return output


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-index", type=Path, required=True)
    parser.add_argument("--source-task", required=True)
    parser.add_argument("--source-segment", required=True)
    parser.add_argument("--source-phase", type=float)
    parser.add_argument("--successor-index", type=Path, required=True)
    parser.add_argument("--successor-task", required=True)
    parser.add_argument("--successor-segment", required=True)
    parser.add_argument("--successor-phase", type=float)
    parser.add_argument("--duration-s", type=float, action="append", required=True)
    parser.add_argument("--validation-config", type=Path, required=True)
    parser.add_argument("--transport-floor-mm", type=float)
    parser.add_argument("--max-pairs", type=int)
    parser.add_argument(
        "--bridge-algorithm",
        choices=(
            CUBIC_BEZIER_FIXED,
            CUBIC_BEZIER_TANGENT_REGULARIZED_V1,
        ),
        default=CUBIC_BEZIER_FIXED,
    )
    parser.add_argument(
        "--minimum-tangent-handle-chord-ratio",
        type=float,
        default=0.0,
    )
    parser.add_argument(
        "--maximum-endpoint-speed-adjustment-mm-s",
        type=float,
    )
    parser.add_argument(
        "--semantic-authority",
        choices=[item.value for item in SemanticAuthority],
        default=SemanticAuthority.RUNTIME_GUARDED.value,
    )
    parser.add_argument("--composition-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    authority = parse_semantic_authority(args.semantic_authority)
    config = CandidateValidationConfig.from_mapping(
        json.loads(args.validation_config.read_text(encoding="utf-8"))
    )
    candidates = enumerate_candidates(
        source_points=_load_points(
            args.source_index,
            args.source_task,
            args.source_segment,
            phase=args.source_phase,
        ),
        successor_points=_load_points(
            args.successor_index,
            args.successor_task,
            args.successor_segment,
            phase=args.successor_phase,
        ),
        durations_s=args.duration_s,
        config=config,
        transport_floor_mm=args.transport_floor_mm,
        max_pairs=args.max_pairs,
        semantic_authority=authority,
        bridge_algorithm=args.bridge_algorithm,
        minimum_tangent_handle_chord_ratio=(
            args.minimum_tangent_handle_chord_ratio
        ),
        maximum_endpoint_speed_adjustment_mm_s=(
            args.maximum_endpoint_speed_adjustment_mm_s
        ),
    )
    result = {
        "schema_version": "a0509.cross_task_handoff_library.v2",
        "composition_id": args.composition_id,
        "generation_method": "corridor_diverse",
        "bridge_algorithm": args.bridge_algorithm,
        "minimum_tangent_handle_chord_ratio": (
            args.minimum_tangent_handle_chord_ratio
        ),
        "maximum_endpoint_speed_adjustment_mm_s": (
            args.maximum_endpoint_speed_adjustment_mm_s
        ),
        "hard_filter_before_diversity": True,
        "semantic_authority": authority.value,
        "semantic_checks_enforced_by_runtime": authority.runtime_semantic_checks_enforced,
        "ik_checker_available": False,
        "collision_checker_available": False,
        "candidates": candidates,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        f"HANDOFF_ENUMERATION_OK total={len(candidates)} "
        f"feasible={sum(item['feasible'] for item in candidates)}"
    )


if __name__ == "__main__":
    main()
