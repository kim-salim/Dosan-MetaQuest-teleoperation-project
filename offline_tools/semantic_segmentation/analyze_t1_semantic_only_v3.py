"""T1 semantic-only graph without selecting any representative episode."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from offline_tools.semantic_segmentation import analyze_t1_semantic_only_v2 as base
from offline_tools.semantic_segmentation.semantic_phase_core import (
    aggregate_standardized_segment,
    detect_gripper_event_sequence,
    resample_cartesian_span,
    resolve_anchor_index,
)
from offline_tools.task_c_bridge_v0.dataset_io import load_lerobot_trajectories


def _plot_projection(
    ax: Any,
    aggregates: list[dict[str, Any]],
    event_centers: dict[str, np.ndarray],
    axes: tuple[int, int],
    xlabel: str,
    ylabel: str,
    title: str,
) -> None:
    for color, aggregate in zip(base.COLORS, aggregates):
        for episode_xyz in aggregate["episode_xyz_mm"]:
            ax.plot(
                episode_xyz[:, axes[0]],
                episode_xyz[:, axes[1]],
                color=color,
                alpha=0.065,
                linewidth=0.65,
            )
        median = aggregate["median_xyz_mm"]
        ax.plot(
            median[:, axes[0]],
            median[:, axes[1]],
            color=color,
            linewidth=3.2,
        )
    for event, center in event_centers.items():
        ax.scatter(
            center[axes[0]],
            center[axes[1]],
            color="#111111",
            s=34,
            zorder=8,
        )
        ax.annotate(
            event,
            (center[axes[0]], center[axes[1]]),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=9,
        )
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title, loc="left", fontweight="bold")
    ax.grid(True, alpha=0.2)
    ax.set_aspect("equal", adjustable="datalim")


def _plot_semantic_graph(
    output_dir: Path,
    aggregates: list[dict[str, Any]],
    event_centers: dict[str, np.ndarray],
) -> None:
    base._configure_plot_font()
    figure = plt.figure(figsize=(18, 12), constrained_layout=True)
    grid = figure.add_gridspec(2, 2, height_ratios=(1.05, 0.95))
    ax3d = figure.add_subplot(grid[0, 0], projection="3d")
    ax_xz = figure.add_subplot(grid[0, 1])
    ax_residual = figure.add_subplot(grid[1, 0])
    ax_state = figure.add_subplot(grid[1, 1])

    for color, aggregate in zip(base.COLORS, aggregates):
        for episode_xyz in aggregate["episode_xyz_mm"]:
            ax3d.plot(*episode_xyz.T, color=color, alpha=0.05, linewidth=0.55)
        ax3d.plot(*aggregate["median_xyz_mm"].T, color=color, linewidth=3.2)
    for event, center in event_centers.items():
        ax3d.scatter(*center, color="#111111", s=32)
        ax3d.text(*center, f"  {event}", fontsize=9)
    ax3d.set_xlabel("X [mm]")
    ax3d.set_ylabel("Y [mm]")
    ax3d.set_zlabel("Z [mm]")
    ax3d.set_title(
        "A. 30개 semantic-phase 궤적과 component-median 대표 경로",
        loc="left",
        fontweight="bold",
    )
    ax3d.view_init(elev=24, azim=-58)

    _plot_projection(
        ax_xz,
        aggregates,
        event_centers,
        (0, 2),
        "X [mm]",
        "Z [mm]",
        "B. X-Z: 굵은 색 선=phase별 median, 흐린 선=30개 분포",
    )

    for index, (color, aggregate) in enumerate(zip(base.COLORS, aggregates)):
        x = index + aggregate["phase"]
        ax_residual.fill_between(
            x,
            aggregate["residual_p50_mm"],
            aggregate["residual_p90_mm"],
            color=color,
            alpha=0.22,
        )
        ax_residual.plot(
            x,
            aggregate["residual_p90_mm"],
            color=color,
            linewidth=2.0,
            label="p90" if index == 0 else None,
        )
        ax_residual.plot(
            x,
            aggregate["residual_p50_mm"],
            color=color,
            linewidth=1.0,
            alpha=0.75,
            label="p50" if index == 0 else None,
        )
        upper = float(np.max(aggregate["residual_p90_mm"]))
        ax_residual.text(
            index + 0.5,
            upper + 1.0,
            aggregate["spec"].segment_id,
            ha="center",
            fontsize=9,
        )
    for boundary in range(1, len(aggregates)):
        ax_residual.axvline(boundary, color="#AAAAAA", linewidth=0.8)
    ax_residual.set_xlim(0.0, float(len(aggregates)))
    ax_residual.set_xlabel("concatenated semantic phase (각 구간 0→1)")
    ax_residual.set_ylabel("대표 궤적으로부터 Cartesian residual [mm]")
    ax_residual.set_title(
        "C. 30개 episode의 phase별 분산(p50/p90)",
        loc="left",
        fontweight="bold",
    )
    ax_residual.grid(True, alpha=0.2)
    ax_residual.legend(loc="upper left", fontsize=8)

    base._plot_state_graph(ax_state, aggregates)
    figure.suptitle(
        "T1 semantic-only standardized graph (30 episodes)\n"
        f'TASK_DESCRIPTION="{base.TASK_DESCRIPTION}"',
        fontsize=17,
        fontweight="bold",
    )
    figure.savefig(output_dir / "t1_semantic_only_graph.png", bbox_inches="tight")
    figure.savefig(output_dir / "t1_semantic_only_graph.svg", bbox_inches="tight")
    plt.close(figure)


def _report(aggregates: list[dict[str, Any]]) -> str:
    rows = []
    for aggregate in aggregates:
        spec = aggregate["spec"]
        rows.append(
            f"| {spec.segment_id} | `{spec.parent_subgoal}` | `{spec.semantic_label}` | "
            f"{spec.gripper_semantics} | {spec.manipulated_object or 'none'} | "
            f"{aggregate['path_length_mm']['median']:.1f} | "
            f"{aggregate['phase_residual_mm']['p90_mean']:.1f} | {spec.confidence} |"
        )
    return f"""# T1 semantic-only standardized graph V3

TASK_DESCRIPTION: `{base.TASK_DESCRIPTION}`

## 설계 범위

30개 demonstration에서 semantic 정보만 구분한다. 각 구간은 Cartesian
arc-length phase `0..1`로 표준화하고 동일 phase의 component median을 대표
궤적으로 사용한다. 특정 실제 episode를 대표로 선택하지 않는다.

![T1 semantic-only graph](t1_semantic_only_graph.png)

![T1 semantic hierarchy](t1_semantic_hierarchy.png)

## 계층적 semantic segment

| ID | 상위 subgoal | Semantic label | Gripper 의미 | 대상 | 대표 경로 길이 [mm] | phase 평균 p90 residual [mm] | 신뢰도 |
|---|---|---|---|---|---:|---:|---|
{chr(10).join(rows)}

## Semantic event grammar

`C1 → O1 → C2 → O2`

- C1: 흰색 컨테이너 손잡이 파지
- O1: 컨테이너를 연 뒤 손잡이 해제
- C2: 검은 테이블의 파란 블록 파지
- O2: 흰색 컨테이너 안에 파란 블록 방출

이 event는 semantic anchor이며 정책 전환 지점을 의미하지 않는다.

## Semantic world state

```text
Container closed, blue block on black table, gripper open
  → Container open, blue block on black table, gripper open
  → Container open, blue block held, gripper closed
  → Container open, blue block in container, gripper open
```

## 대표 정보의 정의

- 굵은 색 경로: 30개 궤적의 동일 semantic phase에서 계산한 component median
- 흐린 색 경로: 대표 선택에 사용하지 않는 30개 개별 분포
- 영상·이미지·frame 번호의 평균 또는 실제 대표 episode 선택 없음

## 포함하지 않은 판단

- 전환 적합도와 전환 phase
- Bridge 비용과 구형 경계
- 상위 planner의 task 조합 결정

## 산출물

- `semantic_graph_v3.json`: 계층, event, world-state, 대표 통계
- `semantic_segment_catalog.csv`: aggregate semantic segment
- `semantic_event_catalog.csv`: 의미 anchor
- `representative_semantic_phase.csv`: phase별 median/mean XYZ와 residual
- `standardized_semantic_phase_trajectories.npz`: 30개 표준화 궤적과 covariance
"""


def analyze(dataset_root: Path, output_dir: Path, phase_points: int) -> dict[str, Any]:
    source = dataset_root.resolve()
    target = output_dir.resolve()
    if source == target or source in target.parents:
        raise ValueError("output directory must not be inside the source dataset")
    trajectories, info = load_lerobot_trajectories(dataset_root, "t1")
    if len(trajectories) != 30:
        raise ValueError(f"expected 30 T1 episodes, found {len(trajectories)}")
    if info.get("task_descriptions") != [base.TASK_DESCRIPTION]:
        raise ValueError("T1 TASK_DESCRIPTION differs from the audited semantic spec")
    episode_events = [
        detect_gripper_event_sequence(trajectory, base.EVENT_SEQUENCE)
        for trajectory in trajectories
    ]

    aggregates = []
    for spec in base.SEGMENTS:
        standardized = []
        for trajectory, events in zip(trajectories, episode_events):
            start = resolve_anchor_index(spec.start_anchor, trajectory, events)
            end = resolve_anchor_index(spec.end_anchor, trajectory, events)
            standardized.append(
                resample_cartesian_span(trajectory, start, end, phase_points)
            )
        aggregates.append(aggregate_standardized_segment(spec, standardized))

    event_centers = base._event_centers(trajectories, episode_events)
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest = {
        "schema_version": "a0509.semantic_only_graph.v3",
        "dataset_repo_id": "local/a0509_blue_block_t1_throw_away_inthe_container",
        "task_description": base.TASK_DESCRIPTION,
        "episode_count": len(trajectories),
        "event_grammar": list(base.EVENT_SEQUENCE),
        "events": {
            event: {
                **base.EVENTS[event],
                "representative_position_mm": event_centers[event],
            }
            for event in base.EVENT_SEQUENCE
        },
        "hierarchy": {
            "task": "discard_blue_block_in_white_container",
            "subgoals": [
                "open_white_container",
                "acquire_blue_block",
                "place_blue_block_in_white_container",
                "finish_task",
            ],
        },
        "segments": [
            {
                "spec": aggregate["spec"],
                "path_length_mm": aggregate["path_length_mm"],
                "phase_residual_mm": aggregate["phase_residual_mm"],
            }
            for aggregate in aggregates
        ],
        "representative": {
            "synthetic": "component median XYZ at each semantic phase",
            "individual_episode_selected": False,
        },
        "standardization": {
            "axis": "per-segment Cartesian arc-length phase 0..1",
            "coordinate_frame": "Doosan base",
            "units": "mm",
            "phase_points": phase_points,
            "frame_number_average_used": False,
            "image_pixel_average_used": False,
        },
        "scope": {
            "semantic_information_only": True,
            "transition_fitness_included": False,
            "spherical_boundaries_included": False,
            "bridge_planning_included": False,
            "robot_executable": False,
            "original_dataset_modified": False,
        },
    }

    base._write_tables(output_dir, aggregates, event_centers)
    base._write_npz(output_dir, aggregates)
    (output_dir / "semantic_graph_v3.json").write_text(
        json.dumps(base._jsonable(manifest), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    _plot_semantic_graph(output_dir, aggregates, event_centers)
    base._plot_hierarchy(output_dir, aggregates)
    (output_dir / "README.md").write_text(_report(aggregates), encoding="utf-8")
    base._checksums(output_dir)
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--phase-points", type=int, default=101)
    args = parser.parse_args()
    if args.phase_points < 3:
        parser.error("--phase-points must be at least 3")
    manifest = analyze(args.dataset_root, args.output_dir, args.phase_points)
    print(
        json.dumps(
            {
                "output_dir": str(args.output_dir.resolve()),
                "episodes": manifest["episode_count"],
                "segments": len(manifest["segments"]),
                "event_grammar": manifest["event_grammar"],
                "individual_episode_selected": manifest["representative"]["individual_episode_selected"],
                "semantic_only": manifest["scope"]["semantic_information_only"],
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
