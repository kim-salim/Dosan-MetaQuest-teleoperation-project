"""Create the T1 semantic-only graph requested for upper-planner input.

The exported representation contains no averaged frame index, no spherical
runtime boundary, and no transition fitness. Gripper changes are semantic
anchors; every motion edge is standardized by Cartesian arc-length phase.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from offline_tools.semantic_segmentation.semantic_phase_core import (
    SemanticSegmentSpec,
    aggregate_standardized_segment,
    detect_gripper_event_sequence,
    representative_episode_medoid,
    resample_cartesian_span,
    resolve_anchor_index,
)
from offline_tools.task_c_bridge_v0.dataset_io import load_lerobot_trajectories


TASK_DESCRIPTION = "Open the White Container throw away the blue block"
EVENT_SEQUENCE = ("C1", "O1", "C2", "O2")

EVENTS = {
    "C1": {
        "semantic_label": "grasp_white_container_handle",
        "label_ko": "흰색 컨테이너 손잡이 파지",
        "gripper_change": "open_to_closed",
        "manipulated_object": "white_container_handle",
        "semantic_effect": "container_handle_held",
        "confidence": "high",
    },
    "O1": {
        "semantic_label": "release_handle_after_opening_container",
        "label_ko": "컨테이너를 연 뒤 손잡이 해제",
        "gripper_change": "closed_to_open",
        "manipulated_object": "white_container_handle",
        "semantic_effect": "white_container_open",
        "confidence": "high",
    },
    "C2": {
        "semantic_label": "grasp_blue_block_on_black_table",
        "label_ko": "검은 테이블의 파란 블록 파지",
        "gripper_change": "open_to_closed",
        "manipulated_object": "blue_block",
        "semantic_effect": "blue_block_held",
        "confidence": "high",
    },
    "O2": {
        "semantic_label": "release_blue_block_in_white_container",
        "label_ko": "흰색 컨테이너 안에 파란 블록 방출",
        "gripper_change": "closed_to_open",
        "manipulated_object": "blue_block",
        "semantic_effect": "blue_block_in_white_container",
        "confidence": "high",
    },
}

SEGMENTS = (
    SemanticSegmentSpec(
        segment_id="S1",
        parent_subgoal="open_white_container",
        semantic_label="approach_white_container_handle",
        label_ko="흰색 컨테이너 손잡이로 접근",
        start_anchor="START",
        end_anchor="C1",
        gripper_semantics="open; closes at C1",
        manipulated_object="white_container_handle",
        entry_state={
            "gripper": "open",
            "white_container": "closed",
            "blue_block": "on_black_table",
            "held_object": "none",
        },
        exit_state={
            "gripper": "closed",
            "white_container": "closed",
            "blue_block": "on_black_table",
            "held_object": "white_container_handle",
        },
        evidence=("TASK_DESCRIPTION", "C1", "TCP approach motion", "multi-camera review"),
        confidence="high",
    ),
    SemanticSegmentSpec(
        segment_id="S2",
        parent_subgoal="open_white_container",
        semantic_label="manipulate_white_container_open",
        label_ko="손잡이를 조작하여 흰색 컨테이너 열기",
        start_anchor="C1",
        end_anchor="O1",
        gripper_semantics="closed; opens at O1",
        manipulated_object="white_container_handle",
        entry_state={
            "gripper": "closed",
            "white_container": "closed",
            "blue_block": "on_black_table",
            "held_object": "white_container_handle",
        },
        exit_state={
            "gripper": "open",
            "white_container": "open",
            "blue_block": "on_black_table",
            "held_object": "none",
        },
        evidence=("TASK_DESCRIPTION", "C1-O1", "TCP handle motion", "multi-camera review"),
        confidence="high",
    ),
    SemanticSegmentSpec(
        segment_id="S3",
        parent_subgoal="acquire_blue_block",
        semantic_label="approach_blue_block_on_black_table",
        label_ko="검은 테이블 위 파란 블록으로 접근",
        start_anchor="O1",
        end_anchor="C2",
        gripper_semantics="open; closes at C2",
        manipulated_object="blue_block",
        entry_state={
            "gripper": "open",
            "white_container": "open",
            "blue_block": "on_black_table",
            "held_object": "none",
        },
        exit_state={
            "gripper": "closed",
            "white_container": "open",
            "blue_block": "held",
            "held_object": "blue_block",
        },
        evidence=("TASK_DESCRIPTION", "O1-C2", "TCP descent/approach", "multi-camera review"),
        confidence="high",
    ),
    SemanticSegmentSpec(
        segment_id="S4",
        parent_subgoal="place_blue_block_in_white_container",
        semantic_label="lift_transport_and_align_blue_block",
        label_ko="파란 블록을 들어 컨테이너로 운반·정렬",
        start_anchor="C2",
        end_anchor="O2",
        gripper_semantics="closed; opens at O2",
        manipulated_object="blue_block",
        entry_state={
            "gripper": "closed",
            "white_container": "open",
            "blue_block": "held",
            "held_object": "blue_block",
        },
        exit_state={
            "gripper": "open",
            "white_container": "open",
            "blue_block": "in_white_container",
            "held_object": "none",
        },
        evidence=("TASK_DESCRIPTION", "C2-O2", "TCP lift/transport", "multi-camera review"),
        confidence="high",
    ),
    SemanticSegmentSpec(
        segment_id="S5",
        parent_subgoal="finish_task",
        semantic_label="retract_after_blue_block_release",
        label_ko="블록 방출 후 컨테이너에서 후퇴",
        start_anchor="O2",
        end_anchor="END",
        gripper_semantics="open",
        manipulated_object=None,
        entry_state={
            "gripper": "open",
            "white_container": "open",
            "blue_block": "in_white_container",
            "held_object": "none",
        },
        exit_state={
            "gripper": "open",
            "white_container": "open",
            "blue_block": "in_white_container",
            "held_object": "none",
        },
        evidence=("O2", "TCP post-release motion"),
        confidence="medium",
    ),
)

COLORS = ("#4C78A8", "#F58518", "#54A24B", "#E45756", "#72B7B2")


def _configure_plot_font() -> None:
    candidates = (
        Path("/usr/share/fonts/truetype/nanum/NanumGothic.ttf"),
        Path("/usr/share/opentype/noto/NotoSansCJK-Regular.ttc"),
    )
    for path in candidates:
        if path.exists():
            from matplotlib import font_manager

            font_manager.fontManager.addfont(str(path))
            plt.rcParams["font.family"] = font_manager.FontProperties(
                fname=str(path)
            ).get_name()
            break
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["figure.dpi"] = 120
    plt.rcParams["savefig.dpi"] = 220


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, SemanticSegmentSpec):
        return {
            field: _jsonable(getattr(value, field))
            for field in value.__dataclass_fields__
        }
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _event_centers(trajectories: list[Any], episode_events: list[dict[str, int]]) -> dict[str, np.ndarray]:
    return {
        event: np.median(
            np.stack(
                [
                    trajectory.xyz_mm[events[event]]
                    for trajectory, events in zip(trajectories, episode_events)
                ],
                axis=0,
            ),
            axis=0,
        )
        for event in EVENT_SEQUENCE
    }


def _plot_projection(
    ax: Any,
    aggregates: list[dict[str, Any]],
    medoid_offset: int,
    event_centers: dict[str, np.ndarray],
    axes: tuple[int, int],
    xlabel: str,
    ylabel: str,
    title: str,
) -> None:
    for color, aggregate in zip(COLORS, aggregates):
        for episode_xyz in aggregate["episode_xyz_mm"]:
            ax.plot(
                episode_xyz[:, axes[0]],
                episode_xyz[:, axes[1]],
                color=color,
                alpha=0.065,
                linewidth=0.65,
            )
        median = aggregate["median_xyz_mm"]
        medoid = aggregate["episode_xyz_mm"][medoid_offset]
        ax.plot(
            median[:, axes[0]],
            median[:, axes[1]],
            color=color,
            linewidth=3.2,
        )
        ax.plot(
            medoid[:, axes[0]],
            medoid[:, axes[1]],
            color="#111111",
            linewidth=1.0,
            linestyle="--",
            alpha=0.8,
        )
    for event, center in event_centers.items():
        ax.scatter(center[axes[0]], center[axes[1]], color="#111111", s=34, zorder=8)
        ax.annotate(event, (center[axes[0]], center[axes[1]]), xytext=(5, 5), textcoords="offset points", fontsize=9)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title, loc="left", fontweight="bold")
    ax.grid(True, alpha=0.2)
    ax.set_aspect("equal", adjustable="datalim")


def _plot_state_graph(ax: Any, aggregates: list[dict[str, Any]]) -> None:
    ax.set_xlim(-0.25, 5.25)
    ax.set_ylim(-1.8, 2.2)
    ax.axis("off")
    ax.set_title("D. 계층적 semantic subgoal과 상태 변화", loc="left", fontweight="bold")

    node_names = ("START", "C1", "O1", "C2", "O2", "TAIL")
    state_lines = (
        "G: open\nContainer: closed\nBlock: black table",
        "G: closed\nHeld: handle",
        "G: open\nContainer: open\nBlock: black table",
        "G: closed\nContainer: open\nHeld: blue block",
        "G: open\nContainer: open\nBlock: in container",
        "G: open\nState unchanged",
    )
    for index, (name, state) in enumerate(zip(node_names, state_lines)):
        ax.scatter(
            [index],
            [0.25],
            s=900,
            color="#F2F4F7" if name not in EVENT_SEQUENCE else "#FFF3CD",
            edgecolor="#222222",
            linewidth=1.2,
            zorder=5,
        )
        ax.text(index, 0.25, name, ha="center", va="center", fontweight="bold", fontsize=9, zorder=6)
        ax.text(index, -0.55, state, ha="center", va="top", fontsize=8)

    for index, (color, aggregate) in enumerate(zip(COLORS, aggregates)):
        spec = aggregate["spec"]
        ax.annotate(
            "",
            xy=(index + 0.84, 0.25),
            xytext=(index + 0.16, 0.25),
            arrowprops={"arrowstyle": "-|>", "lw": 7, "color": color, "alpha": 0.85},
        )
        ax.text(
            index + 0.5,
            0.82 if index % 2 == 0 else 1.22,
            f"{spec.segment_id}  {spec.label_ko}\nG: {spec.gripper_semantics}",
            ha="center",
            va="center",
            fontsize=8.5,
            bbox={"boxstyle": "round,pad=0.25", "fc": "white", "ec": color},
        )

    parent_groups = (
        (0.0, 2.0, "open_white_container", "#B9D6F2"),
        (2.0, 3.0, "acquire_blue_block", "#CDECCF"),
        (3.0, 4.0, "place_blue_block_in_white_container", "#F8CCCC"),
        (4.0, 5.0, "finish_task", "#CFE8E6"),
    )
    for start, end, label, color in parent_groups:
        ax.plot([start + 0.05, end - 0.05], [1.82, 1.82], color=color, linewidth=9, solid_capstyle="round")
        ax.text((start + end) / 2, 2.02, label, ha="center", va="center", fontsize=9, fontweight="bold")


def _plot_semantic_graph(
    output_dir: Path,
    aggregates: list[dict[str, Any]],
    medoid: dict[str, Any],
    event_centers: dict[str, np.ndarray],
) -> None:
    _configure_plot_font()
    medoid_ids = aggregates[0]["episode_ids"]
    medoid_offset = int(np.flatnonzero(medoid_ids == medoid["episode_index"])[0])
    figure = plt.figure(figsize=(18, 12), constrained_layout=True)
    grid = figure.add_gridspec(2, 2, height_ratios=(1.05, 0.95))
    ax3d = figure.add_subplot(grid[0, 0], projection="3d")
    ax_xz = figure.add_subplot(grid[0, 1])
    ax_residual = figure.add_subplot(grid[1, 0])
    ax_state = figure.add_subplot(grid[1, 1])

    for color, aggregate in zip(COLORS, aggregates):
        for episode_xyz in aggregate["episode_xyz_mm"]:
            ax3d.plot(*episode_xyz.T, color=color, alpha=0.05, linewidth=0.55)
        median = aggregate["median_xyz_mm"]
        medoid_xyz = aggregate["episode_xyz_mm"][medoid_offset]
        ax3d.plot(*median.T, color=color, linewidth=3.2)
        ax3d.plot(*medoid_xyz.T, color="#111111", linewidth=1.0, linestyle="--", alpha=0.75)
    for event, center in event_centers.items():
        ax3d.scatter(*center, color="#111111", s=32)
        ax3d.text(*center, f"  {event}", fontsize=9)
    ax3d.set_xlabel("X [mm]")
    ax3d.set_ylabel("Y [mm]")
    ax3d.set_zlabel("Z [mm]")
    ax3d.set_title("A. 30개 semantic-phase 궤적과 대표 경로", loc="left", fontweight="bold")
    ax3d.view_init(elev=24, azim=-58)

    _plot_projection(
        ax_xz,
        aggregates,
        medoid_offset,
        event_centers,
        (0, 2),
        "X [mm]",
        "Z [mm]",
        "B. X-Z 대표 궤적: 굵은 선=median, 검은 점선=실제 medoid episode",
    )

    for index, (color, aggregate) in enumerate(zip(COLORS, aggregates)):
        x = index + aggregate["phase"]
        ax_residual.fill_between(
            x,
            aggregate["residual_p50_mm"],
            aggregate["residual_p90_mm"],
            color=color,
            alpha=0.22,
        )
        ax_residual.plot(x, aggregate["residual_p90_mm"], color=color, linewidth=2.0)
        ax_residual.plot(x, aggregate["residual_p50_mm"], color=color, linewidth=1.0, linestyle="--")
        ax_residual.text(index + 0.5, ax_residual.get_ylim()[1] * 0.91 if ax_residual.get_ylim()[1] else 1, aggregate["spec"].segment_id, ha="center", fontsize=9)
    for boundary in range(1, len(aggregates)):
        ax_residual.axvline(boundary, color="#888888", linewidth=0.8, linestyle=":")
    ax_residual.set_xlim(0.0, float(len(aggregates)))
    ax_residual.set_xlabel("concatenated semantic phase (각 구간 0→1)")
    ax_residual.set_ylabel("대표 궤적으로부터 Cartesian residual [mm]")
    ax_residual.set_title("C. 30개 episode의 phase별 분산(p50 점선, p90 실선)", loc="left", fontweight="bold")
    ax_residual.grid(True, alpha=0.2)

    _plot_state_graph(ax_state, aggregates)
    figure.suptitle(
        "T1 semantic-only standardized graph (30 episodes)\n"
        f'TASK_DESCRIPTION="{TASK_DESCRIPTION}"',
        fontsize=17,
        fontweight="bold",
    )
    figure.savefig(output_dir / "t1_semantic_only_graph.png", bbox_inches="tight")
    figure.savefig(output_dir / "t1_semantic_only_graph.svg", bbox_inches="tight")
    plt.close(figure)


def _plot_hierarchy(output_dir: Path, aggregates: list[dict[str, Any]]) -> None:
    _configure_plot_font()
    figure, (ax_hierarchy, ax_states) = plt.subplots(2, 1, figsize=(17, 10), constrained_layout=True)
    for ax in (ax_hierarchy, ax_states):
        ax.axis("off")

    ax_hierarchy.set_xlim(0, 10)
    ax_hierarchy.set_ylim(0, 5)
    ax_hierarchy.set_title("A. T1 hierarchical semantic ontology", loc="left", fontweight="bold")
    ax_hierarchy.text(5, 4.5, "T1: discard_blue_block_in_white_container", ha="center", va="center", fontsize=13, fontweight="bold", bbox={"boxstyle": "round,pad=0.35", "fc": "#E8EDF4", "ec": "#34495E"})
    parents = (
        (1.5, "open_white_container"),
        (4.0, "acquire_blue_block"),
        (6.7, "place_blue_block_in_white_container"),
        (9.0, "finish_task"),
    )
    for x, label in parents:
        ax_hierarchy.annotate("", xy=(x, 3.55), xytext=(5, 4.2), arrowprops={"arrowstyle": "->", "color": "#777777"})
        ax_hierarchy.text(x, 3.35, label, ha="center", va="center", fontsize=10, fontweight="bold", bbox={"boxstyle": "round,pad=0.25", "fc": "white", "ec": "#777777"})
    parent_x = {label: x for x, label in parents}
    child_counts: dict[str, int] = {}
    child_total = {parent: sum(spec.parent_subgoal == parent for spec in SEGMENTS) for _, parent in parents}
    for color, aggregate in zip(COLORS, aggregates):
        spec = aggregate["spec"]
        offset = child_counts.get(spec.parent_subgoal, 0)
        child_counts[spec.parent_subgoal] = offset + 1
        total = child_total[spec.parent_subgoal]
        x = parent_x[spec.parent_subgoal] + (offset - (total - 1) / 2) * 1.35
        ax_hierarchy.annotate("", xy=(x, 1.85), xytext=(parent_x[spec.parent_subgoal], 3.05), arrowprops={"arrowstyle": "->", "color": color})
        ax_hierarchy.text(x, 1.55, f"{spec.segment_id}\n{spec.semantic_label}\nG: {spec.gripper_semantics}", ha="center", va="center", fontsize=8.5, bbox={"boxstyle": "round,pad=0.3", "fc": "white", "ec": color})

    ax_states.set_xlim(-0.2, 4.2)
    ax_states.set_ylim(-0.5, 3.7)
    ax_states.set_title("B. Semantic world-state progression", loc="left", fontweight="bold")
    semantic_states = (
        ("Initial", "Container: closed\nBlue block: black table\nHeld: none\nGripper: open"),
        ("After O1", "Container: open\nBlue block: black table\nHeld: none\nGripper: open"),
        ("After C2", "Container: open\nBlue block: held\nHeld: blue block\nGripper: closed"),
        ("After O2", "Container: open\nBlue block: in container\nHeld: none\nGripper: open"),
    )
    for index, (name, state) in enumerate(semantic_states):
        ax_states.text(index * 1.32, 1.55, f"{name}\n\n{state}", ha="center", va="center", fontsize=10, bbox={"boxstyle": "round,pad=0.45", "fc": "#F7F9FB", "ec": "#536878"})
        if index < len(semantic_states) - 1:
            event = ("open_white_container", "grasp_blue_block", "release_blue_block")[index]
            ax_states.annotate(event, xy=(index * 1.32 + 1.05, 1.55), xytext=(index * 1.32 + 0.28, 1.55), ha="center", va="bottom", arrowprops={"arrowstyle": "-|>", "lw": 2.2, "color": "#536878"}, fontsize=8)
    ax_states.text(0, -0.1, "직접 관측: gripper_commanded_state + TCP motion", fontsize=9, color="#555555")
    ax_states.text(2.0, -0.1, "의미 부여: TASK_DESCRIPTION + 멀티카메라 검토", fontsize=9, color="#555555")
    figure.suptitle("T1 semantic hierarchy and state changes", fontsize=16, fontweight="bold")
    figure.savefig(output_dir / "t1_semantic_hierarchy.png", bbox_inches="tight")
    figure.savefig(output_dir / "t1_semantic_hierarchy.svg", bbox_inches="tight")
    plt.close(figure)


def _write_tables(output_dir: Path, aggregates: list[dict[str, Any]], event_centers: dict[str, np.ndarray]) -> None:
    with (output_dir / "semantic_segment_catalog.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = (
            "segment_id",
            "parent_subgoal",
            "semantic_label",
            "label_ko",
            "start_anchor",
            "end_anchor",
            "gripper_semantics",
            "manipulated_object",
            "entry_state",
            "exit_state",
            "path_length_median_mm",
            "phase_residual_p90_mean_mm",
            "confidence",
        )
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for aggregate in aggregates:
            spec = aggregate["spec"]
            writer.writerow(
                {
                    "segment_id": spec.segment_id,
                    "parent_subgoal": spec.parent_subgoal,
                    "semantic_label": spec.semantic_label,
                    "label_ko": spec.label_ko,
                    "start_anchor": spec.start_anchor,
                    "end_anchor": spec.end_anchor,
                    "gripper_semantics": spec.gripper_semantics,
                    "manipulated_object": spec.manipulated_object or "none",
                    "entry_state": json.dumps(spec.entry_state, ensure_ascii=False, sort_keys=True),
                    "exit_state": json.dumps(spec.exit_state, ensure_ascii=False, sort_keys=True),
                    "path_length_median_mm": aggregate["path_length_mm"]["median"],
                    "phase_residual_p90_mean_mm": aggregate["phase_residual_mm"]["p90_mean"],
                    "confidence": spec.confidence,
                }
            )

    with (output_dir / "semantic_event_catalog.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = (
            "event_id",
            "semantic_label",
            "label_ko",
            "gripper_change",
            "manipulated_object",
            "semantic_effect",
            "representative_x_mm",
            "representative_y_mm",
            "representative_z_mm",
            "confidence",
        )
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for event in EVENT_SEQUENCE:
            item = EVENTS[event]
            center = event_centers[event]
            writer.writerow(
                {
                    "event_id": event,
                    **item,
                    "representative_x_mm": center[0],
                    "representative_y_mm": center[1],
                    "representative_z_mm": center[2],
                }
            )

    with (output_dir / "representative_semantic_phase.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = (
            "segment_id",
            "semantic_label",
            "semantic_phase",
            "median_x_mm",
            "median_y_mm",
            "median_z_mm",
            "mean_x_mm",
            "mean_y_mm",
            "mean_z_mm",
            "residual_p50_mm",
            "residual_p90_mm",
            "residual_p95_mm",
        )
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for aggregate in aggregates:
            spec = aggregate["spec"]
            for index, phase in enumerate(aggregate["phase"]):
                median = aggregate["median_xyz_mm"][index]
                mean = aggregate["mean_xyz_mm"][index]
                writer.writerow(
                    {
                        "segment_id": spec.segment_id,
                        "semantic_label": spec.semantic_label,
                        "semantic_phase": phase,
                        "median_x_mm": median[0],
                        "median_y_mm": median[1],
                        "median_z_mm": median[2],
                        "mean_x_mm": mean[0],
                        "mean_y_mm": mean[1],
                        "mean_z_mm": mean[2],
                        "residual_p50_mm": aggregate["residual_p50_mm"][index],
                        "residual_p90_mm": aggregate["residual_p90_mm"][index],
                        "residual_p95_mm": aggregate["residual_p95_mm"][index],
                    }
                )


def _write_npz(output_dir: Path, aggregates: list[dict[str, Any]]) -> None:
    arrays: dict[str, np.ndarray] = {}
    for aggregate in aggregates:
        prefix = aggregate["spec"].segment_id
        for name in (
            "phase",
            "episode_ids",
            "episode_xyz_mm",
            "median_xyz_mm",
            "mean_xyz_mm",
            "covariance_mm2",
            "mad_mm",
            "residual_p50_mm",
            "residual_p90_mm",
            "residual_p95_mm",
        ):
            arrays[f"{prefix}_{name}"] = np.asarray(aggregate[name])
    np.savez_compressed(output_dir / "standardized_semantic_phase_trajectories.npz", **arrays)


def _report(aggregates: list[dict[str, Any]], medoid: dict[str, Any]) -> str:
    rows = []
    for aggregate in aggregates:
        spec = aggregate["spec"]
        rows.append(
            f"| {spec.segment_id} | `{spec.parent_subgoal}` | `{spec.semantic_label}` | "
            f"{spec.gripper_semantics} | {spec.manipulated_object or 'none'} | "
            f"{aggregate['path_length_mm']['median']:.1f} | "
            f"{aggregate['phase_residual_mm']['p90_mean']:.1f} | {spec.confidence} |"
        )
    return f"""# T1 semantic-only standardized graph V2

TASK_DESCRIPTION: `{TASK_DESCRIPTION}`

## 설계 범위

이 버전은 30개 demonstration에서 semantic 정보만 구분한다. 각 구간은
Cartesian arc-length phase `0..1`로 표준화하며, 대표 경로는 phase별 component
median이다. 원시 frame 번호의 평균, 구형 runtime 경계, 전환 적합도와 Bridge
계산은 포함하지 않는다.

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

이 event는 의미 anchor일 뿐 정책 전환 지점이 아니다.

## Semantic world state

```text
Container closed, blue block on black table, gripper open
  → Container open, blue block on black table, gripper open
  → Container open, blue block held, gripper closed
  → Container open, blue block in container, gripper open
```

## 대표 자료의 정의

- 합성 representative: 30개를 동일 semantic phase에서 component median
- 실제 representative demonstration: episode `{medoid['episode_index']}`
- 실제 episode 선택법: 각 segment의 median 경로에 대한 RMS Cartesian residual을
  정규화하여 합한 값이 가장 작은 medoid
- 영상이나 이미지 픽셀을 평균하지 않음

## 포함하지 않은 판단

- 어느 구간이 전환에 적합한지
- A/B 중 어느 phase에서 정책을 바꿀지
- Bridge 비용·안전 조건·구형 경계
- 상위 planner의 task 조합 결정

이 판단들은 향후 상위 planner가 본 semantic label, gripper 의미, 조작 대상과
world-state 정보를 입력으로 받아 수행한다.

## 산출물

- `semantic_graph_v2.json`: 계층, event, world-state, 대표 통계
- `semantic_segment_catalog.csv`: 5개 aggregate semantic segment
- `semantic_event_catalog.csv`: C1/O1/C2/O2 의미 anchor
- `representative_semantic_phase.csv`: phase별 대표 XYZ와 residual
- `standardized_semantic_phase_trajectories.npz`: 30개 표준화 궤적과 covariance
"""


def _checksums(output_dir: Path) -> None:
    lines = []
    for path in sorted(output_dir.iterdir()):
        if path.is_file() and path.name != "checksums.sha256":
            lines.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}")
    (output_dir / "checksums.sha256").write_text("\n".join(lines) + "\n", encoding="utf-8")


def analyze(dataset_root: Path, output_dir: Path, phase_points: int) -> dict[str, Any]:
    source = dataset_root.resolve()
    target = output_dir.resolve()
    if source == target or source in target.parents:
        raise ValueError("output directory must not be inside the source dataset")
    trajectories, info = load_lerobot_trajectories(dataset_root, "t1")
    if len(trajectories) != 30:
        raise ValueError(f"expected 30 T1 episodes, found {len(trajectories)}")
    if info.get("task_descriptions") != [TASK_DESCRIPTION]:
        raise ValueError("T1 TASK_DESCRIPTION differs from the audited semantic spec")
    episode_events = [
        detect_gripper_event_sequence(trajectory, EVENT_SEQUENCE)
        for trajectory in trajectories
    ]

    aggregates = []
    for spec in SEGMENTS:
        standardized = []
        for trajectory, events in zip(trajectories, episode_events):
            start = resolve_anchor_index(spec.start_anchor, trajectory, events)
            end = resolve_anchor_index(spec.end_anchor, trajectory, events)
            standardized.append(
                resample_cartesian_span(trajectory, start, end, phase_points)
            )
        aggregates.append(aggregate_standardized_segment(spec, standardized))

    medoid = representative_episode_medoid(aggregates)
    event_centers = _event_centers(trajectories, episode_events)
    output_dir.mkdir(parents=True, exist_ok=True)

    manifest = {
        "schema_version": "a0509.semantic_only_graph.v2",
        "dataset_repo_id": "local/a0509_blue_block_t1_throw_away_inthe_container",
        "task_description": TASK_DESCRIPTION,
        "episode_count": len(trajectories),
        "event_grammar": list(EVENT_SEQUENCE),
        "events": {
            event: {**EVENTS[event], "representative_position_mm": event_centers[event]}
            for event in EVENT_SEQUENCE
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
            "real_demonstration_medoid": medoid,
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

    _write_tables(output_dir, aggregates, event_centers)
    _write_npz(output_dir, aggregates)
    (output_dir / "semantic_graph_v2.json").write_text(
        json.dumps(_jsonable(manifest), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    _plot_semantic_graph(output_dir, aggregates, medoid, event_centers)
    _plot_hierarchy(output_dir, aggregates)
    (output_dir / "README.md").write_text(_report(aggregates, medoid), encoding="utf-8")
    _checksums(output_dir)
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
                "medoid_episode": manifest["representative"]["real_demonstration_medoid"]["episode_index"],
                "semantic_only": manifest["scope"]["semantic_information_only"],
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
