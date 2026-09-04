"""Build a standardized semantic graph from all T1 demonstrations.

This is a read-only dataset analysis.  It never writes below ``dataset_root``
and never publishes a robot command.  The implementation follows Task-C
Representative Bridge V1: physical XYZ stays in the Doosan base frame while
each semantic interval is resampled by normalized Cartesian arc length.

The semantic labels are external annotations inferred from the dataset task
description, the audited C1-O1-C2-O2 gripper sequence, TCP motion, and the
prior multi-camera review.  They are not language inputs learned by ACT.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from offline_tools.task_c_bridge_v0.dataset_io import load_lerobot_trajectories
from offline_tools.task_c_bridge_v0.trajectory_states import Trajectory


EXPECTED_TASK_DESCRIPTION = "Open the White Container throw away the blue block"
EVENT_IDS = ("C1", "O1", "C2", "O2")
EVENT_SEMANTICS = {
    "C1": "grasp_white_container_handle",
    "O1": "release_handle_after_opening_container",
    "C2": "grasp_blue_block_on_black_table",
    "O2": "release_blue_block_in_white_container",
}
EVENT_SEMANTICS_KO = {
    "C1": "흰색 컨테이너 손잡이 파지",
    "O1": "컨테이너 개방 후 손잡이 해제",
    "C2": "검은 테이블의 파란 블록 파지",
    "O2": "흰색 컨테이너 안에 블록 방출",
}


@dataclass(frozen=True)
class SegmentDefinition:
    segment_id: str
    semantic_label: str
    label_ko: str
    expected_gripper: str
    confidence: str


SEGMENT_DEFINITIONS = (
    SegmentDefinition(
        "S0_E1",
        "approach_and_grasp_white_container",
        "컨테이너 손잡이 접근 및 파지",
        "open_until_C1",
        "high",
    ),
    SegmentDefinition(
        "E1_E2",
        "open_white_container",
        "컨테이너 열기 및 손잡이 해제",
        "closed_until_O1",
        "high",
    ),
    SegmentDefinition(
        "E2_E3",
        "approach_blue_block_on_black_table",
        "검은 테이블의 파란 블록에 접근",
        "open_until_C2",
        "high",
    ),
    SegmentDefinition(
        "E3_S5",
        "acquire_and_transport_blue_block",
        "파란 블록 파지·상승·컨테이너로 운반",
        "closed",
        "high",
    ),
    SegmentDefinition(
        "S5_E4",
        "place_blue_block_in_white_container",
        "컨테이너 방출 영역 진입 및 블록 놓기",
        "closed_until_O2",
        "medium",
    ),
    SegmentDefinition(
        "E4_S6",
        "post_release_retract_and_recording_tail",
        "블록 방출 후 후퇴 및 녹화 말단",
        "open",
        "medium",
    ),
)

SEGMENT_COLORS = (
    "#4C78A8",
    "#F58518",
    "#54A24B",
    "#E45756",
    "#B279A2",
    "#72B7B2",
)


def _round_up(value: float, step: float) -> float:
    return float(math.ceil((float(value) - 1e-12) / step) * step)


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _detect_coco_events(trajectory: Trajectory) -> dict[str, int]:
    closed = trajectory.gripper_closed
    closes = np.flatnonzero((~closed[:-1]) & closed[1:]) + 1
    opens = np.flatnonzero(closed[:-1] & (~closed[1:])) + 1
    if len(closes) != 2 or len(opens) != 2:
        raise ValueError(
            f"episode {trajectory.episode} is not exactly COCO: "
            f"close={closes.tolist()} open={opens.tolist()}"
        )
    ordered = np.asarray([closes[0], opens[0], closes[1], opens[1]])
    if np.any(np.diff(ordered) <= 0):
        raise ValueError(
            f"episode {trajectory.episode} transitions are not C1-O1-C2-O2"
        )
    if bool(closed[0]) or bool(closed[-1]):
        raise ValueError(
            f"episode {trajectory.episode} does not start/end open"
        )
    return {name: int(index) for name, index in zip(EVENT_IDS, ordered)}


def _boundary_record(
    trajectories: list[Trajectory],
    episode_events: list[dict[str, int]],
    event_id: str,
    *,
    reference_commit_mm: float,
    reference_prearm_mm: float,
    radius_step_mm: float,
) -> dict[str, Any]:
    positions = np.stack(
        [
            trajectory.xyz_mm[events[event_id]]
            for trajectory, events in zip(trajectories, episode_events)
        ],
        axis=0,
    )
    center = np.median(positions, axis=0)
    mean = np.mean(positions, axis=0)
    distances = np.linalg.norm(positions - center[None, :], axis=1)
    percentiles = {
        f"p{quantile:02d}_mm": float(np.percentile(distances, quantile))
        for quantile in (50, 80, 90, 95, 100)
    }
    calibrated_commit = max(
        reference_commit_mm,
        _round_up(percentiles["p50_mm"], radius_step_mm),
    )
    calibrated_prearm = max(
        reference_prearm_mm,
        _round_up(percentiles["p95_mm"], radius_step_mm),
        calibrated_commit + radius_step_mm,
    )

    def support(radius: float) -> dict[str, Any]:
        inside = distances <= radius
        return {
            "radius_mm": float(radius),
            "episodes": int(np.sum(inside)),
            "episode_count": len(distances),
            "fraction": float(np.mean(inside)),
            "outside_episode_indices": np.flatnonzero(~inside).tolist(),
        }

    indices = np.asarray([events[event_id] for events in episode_events])
    times = np.asarray(
        [
            trajectory.timestamp_s[events[event_id]]
            for trajectory, events in zip(trajectories, episode_events)
        ],
        dtype=np.float64,
    )
    return {
        "event_id": event_id,
        "semantic_label": EVENT_SEMANTICS[event_id],
        "label_ko": EVENT_SEMANTICS_KO[event_id],
        "center_position_mm": center,
        "mean_position_mm": mean,
        "covariance_mm2": np.cov(positions, rowvar=False),
        "mad_mm": np.median(np.abs(positions - center[None, :]), axis=0),
        "event_frame": {
            "mean": float(np.mean(indices)),
            "std": float(np.std(indices)),
            "min": int(np.min(indices)),
            "max": int(np.max(indices)),
        },
        "event_time_s": {
            "mean": float(np.mean(times)),
            "std": float(np.std(times)),
            "min": float(np.min(times)),
            "max": float(np.max(times)),
        },
        "distance_from_center": percentiles,
        "reference_commit_support": support(reference_commit_mm),
        "reference_prearm_support": support(reference_prearm_mm),
        "calibrated_commit_support": support(calibrated_commit),
        "calibrated_prearm_support": support(calibrated_prearm),
        "calibration_rule": (
            "commit=max(reference_20mm,ceil5(p50)); "
            "prearm=max(reference_40mm,ceil5(p95),commit+5mm)"
        ),
    }


def _find_stable_sphere_entry(
    trajectory: Trajectory,
    *,
    first_index: int,
    last_index: int,
    center_mm: np.ndarray,
    radius_mm: float,
    stable_frames: int,
) -> int:
    distances = np.linalg.norm(trajectory.xyz_mm - center_mm[None, :], axis=1)
    valid = (distances <= radius_mm) & trajectory.gripper_closed
    stop = last_index - stable_frames + 2
    for index in range(first_index, max(first_index, stop)):
        if bool(np.all(valid[index : index + stable_frames])):
            return int(index)
    raise ValueError(
        f"episode {trajectory.episode} never enters the O2 prearm sphere "
        f"for {stable_frames} closed frames"
    )


def _segment_bounds(events: dict[str, int], s5_index: int, last: int) -> tuple[tuple[int, int], ...]:
    bounds = (
        (0, events["C1"]),
        (events["C1"], events["O1"]),
        (events["O1"], events["C2"]),
        (events["C2"], s5_index),
        (s5_index, events["O2"]),
        (events["O2"], last),
    )
    if any(start >= end for start, end in bounds):
        raise ValueError(f"invalid semantic bounds: {bounds}")
    return bounds


def _resample_arc_length(
    trajectory: Trajectory,
    start_index: int,
    end_index: int,
    phase_points: int,
) -> dict[str, np.ndarray | float]:
    xyz = np.asarray(trajectory.xyz_mm[start_index : end_index + 1], dtype=np.float64)
    time = np.asarray(
        trajectory.timestamp_s[start_index : end_index + 1], dtype=np.float64
    )
    increments = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    arc = np.concatenate(([0.0], np.cumsum(increments)))
    keep = np.concatenate(([True], np.diff(arc) > 1e-9))
    xyz = xyz[keep]
    time = time[keep]
    arc = arc[keep]
    if len(arc) < 2 or arc[-1] <= 1e-9:
        raise ValueError(
            f"episode {trajectory.episode} segment {start_index}:{end_index} "
            "has degenerate Cartesian arc length"
        )
    source_phase = arc / arc[-1]
    target_phase = np.linspace(0.0, 1.0, phase_points)
    target_xyz = np.stack(
        [
            np.interp(target_phase, source_phase, xyz[:, axis])
            for axis in range(3)
        ],
        axis=1,
    )
    target_time = np.interp(target_phase, source_phase, time - time[0])
    return {
        "phase": target_phase,
        "xyz_mm": target_xyz,
        "elapsed_s": target_time,
        "path_length_mm": float(arc[-1]),
    }


def _representative_segment(
    definition: SegmentDefinition,
    resampled: list[dict[str, np.ndarray | float]],
) -> dict[str, Any]:
    positions = np.stack([np.asarray(item["xyz_mm"]) for item in resampled], axis=0)
    elapsed = np.stack([np.asarray(item["elapsed_s"]) for item in resampled], axis=0)
    lengths = np.asarray([float(item["path_length_mm"]) for item in resampled])
    median_xyz = np.median(positions, axis=0)
    mean_xyz = np.mean(positions, axis=0)
    residual = np.linalg.norm(positions - median_xyz[None, :, :], axis=2)
    covariance = np.stack(
        [np.cov(positions[:, index, :], rowvar=False) for index in range(positions.shape[1])],
        axis=0,
    )
    return {
        "definition": definition,
        "phase": np.asarray(resampled[0]["phase"]),
        "episode_xyz_mm": positions,
        "median_xyz_mm": median_xyz,
        "mean_xyz_mm": mean_xyz,
        "covariance_mm2": covariance,
        "mad_mm": np.median(np.abs(positions - median_xyz[None, :, :]), axis=0),
        "residual_p50_mm": np.percentile(residual, 50, axis=0),
        "residual_p90_mm": np.percentile(residual, 90, axis=0),
        "residual_p95_mm": np.percentile(residual, 95, axis=0),
        "elapsed_median_s": np.median(elapsed, axis=0),
        "duration_s": {
            "mean": float(np.mean(elapsed[:, -1])),
            "std": float(np.std(elapsed[:, -1])),
            "min": float(np.min(elapsed[:, -1])),
            "max": float(np.max(elapsed[:, -1])),
        },
        "path_length_mm": {
            "median": float(np.median(lengths)),
            "mean": float(np.mean(lengths)),
            "std": float(np.std(lengths)),
            "min": float(np.min(lengths)),
            "max": float(np.max(lengths)),
        },
        "mean_residual_p90_mm": float(np.mean(np.percentile(residual, 90, axis=0))),
        "max_residual_p95_mm": float(np.max(np.percentile(residual, 95, axis=0))),
    }


def _configure_plot_font() -> None:
    candidates = (
        Path("/usr/share/fonts/truetype/nanum/NanumGothic.ttf"),
        Path("/usr/share/opentype/noto/NotoSansCJK-Regular.ttc"),
    )
    for path in candidates:
        if path.exists():
            from matplotlib import font_manager

            font_manager.fontManager.addfont(str(path))
            family = font_manager.FontProperties(fname=str(path)).get_name()
            plt.rcParams["font.family"] = family
            break
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["figure.dpi"] = 120
    plt.rcParams["savefig.dpi"] = 220


def _sphere_wireframe(
    ax: Any,
    center: np.ndarray,
    radius: float,
    color: str,
    *,
    alpha: float,
    linewidth: float,
) -> None:
    u = np.linspace(0.0, 2.0 * np.pi, 28)
    v = np.linspace(0.0, np.pi, 14)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_wireframe(
        x,
        y,
        z,
        rstride=3,
        cstride=3,
        color=color,
        alpha=alpha,
        linewidth=linewidth,
    )


def _plot_projection(
    ax: Any,
    representative_segments: list[dict[str, Any]],
    boundaries: dict[str, dict[str, Any]],
    axes: tuple[int, int],
    xlabel: str,
    ylabel: str,
    title: str,
) -> None:
    for color, segment in zip(SEGMENT_COLORS, representative_segments):
        episodes = segment["episode_xyz_mm"]
        for episode in episodes:
            ax.plot(
                episode[:, axes[0]],
                episode[:, axes[1]],
                color=color,
                alpha=0.07,
                linewidth=0.65,
            )
        median = segment["median_xyz_mm"]
        ax.plot(
            median[:, axes[0]],
            median[:, axes[1]],
            color=color,
            linewidth=3.0,
            label=segment["definition"].semantic_label,
        )
    for event_id, boundary in boundaries.items():
        center = np.asarray(boundary["center_position_mm"])
        commit = float(boundary["calibrated_commit_support"]["radius_mm"])
        prearm = float(boundary["calibrated_prearm_support"]["radius_mm"])
        ax.add_patch(
            plt.Circle(
                (center[axes[0]], center[axes[1]]),
                prearm,
                fill=False,
                color="#222222",
                alpha=0.35,
                linewidth=1.0,
                linestyle="--",
            )
        )
        ax.add_patch(
            plt.Circle(
                (center[axes[0]], center[axes[1]]),
                commit,
                fill=False,
                color="#111111",
                alpha=0.65,
                linewidth=1.3,
            )
        )
        ax.scatter(
            [center[axes[0]]],
            [center[axes[1]]],
            color="#111111",
            s=24,
            zorder=10,
        )
        ax.annotate(
            f"{event_id}\n{commit:.0f}/{prearm:.0f} mm",
            (center[axes[0]], center[axes[1]]),
            xytext=(5, 6),
            textcoords="offset points",
            fontsize=8,
        )
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title, loc="left", fontweight="bold")
    ax.grid(True, alpha=0.2)
    ax.set_aspect("equal", adjustable="datalim")


def _plot_representative_graph(
    path_png: Path,
    path_svg: Path,
    representative_segments: list[dict[str, Any]],
    boundaries: dict[str, dict[str, Any]],
    task_description: str,
) -> None:
    _configure_plot_font()
    figure = plt.figure(figsize=(18, 12), constrained_layout=True)
    grid = figure.add_gridspec(2, 2, height_ratios=(1.12, 0.88))
    ax3d = figure.add_subplot(grid[0, 0], projection="3d")
    ax_xz = figure.add_subplot(grid[0, 1])
    ax_xy = figure.add_subplot(grid[1, 0])
    ax_graph = figure.add_subplot(grid[1, 1])

    for color, segment in zip(SEGMENT_COLORS, representative_segments):
        for episode in segment["episode_xyz_mm"]:
            ax3d.plot(
                episode[:, 0],
                episode[:, 1],
                episode[:, 2],
                color=color,
                alpha=0.055,
                linewidth=0.6,
            )
        median = segment["median_xyz_mm"]
        ax3d.plot(
            median[:, 0],
            median[:, 1],
            median[:, 2],
            color=color,
            linewidth=3.2,
        )
    for event_id, boundary in boundaries.items():
        center = np.asarray(boundary["center_position_mm"])
        commit = float(boundary["calibrated_commit_support"]["radius_mm"])
        prearm = float(boundary["calibrated_prearm_support"]["radius_mm"])
        _sphere_wireframe(
            ax3d, center, prearm, "#111111", alpha=0.10, linewidth=0.45
        )
        _sphere_wireframe(
            ax3d, center, commit, "#111111", alpha=0.28, linewidth=0.6
        )
        ax3d.scatter(*center, color="#111111", s=28)
        ax3d.text(*center, f"  {event_id}", fontsize=9)
    ax3d.set_xlabel("X [mm]")
    ax3d.set_ylabel("Y [mm]")
    ax3d.set_zlabel("Z [mm]")
    ax3d.set_title(
        "A. 30개 표준화 궤적과 robust representative graph",
        loc="left",
        fontweight="bold",
    )
    ax3d.view_init(elev=24, azim=-58)

    _plot_projection(
        ax_xz,
        representative_segments,
        boundaries,
        (0, 2),
        "X [mm]",
        "Z [mm]",
        "B. 측면(X-Z): 실선=대표 경로, 원=commit/prearm sphere",
    )
    _plot_projection(
        ax_xy,
        representative_segments,
        boundaries,
        (0, 1),
        "X [mm]",
        "Y [mm]",
        "C. 평면(X-Y): 30개 source residual과 대표 경로",
    )

    ax_graph.set_xlim(-0.15, 6.15)
    ax_graph.set_ylim(-1.25, 1.5)
    ax_graph.axis("off")
    ax_graph.set_title(
        "D. T1 representative semantic state graph",
        loc="left",
        fontweight="bold",
    )
    graph_labels = (
        "손잡이 접근·파지",
        "컨테이너 열기",
        "블록 접근",
        "블록 파지·운반",
        "컨테이너에 놓기",
        "방출 후 후퇴",
    )
    node_labels = ("S0", "C1", "O1", "C2", "S5", "O2", "END")
    for index, (color, label, segment) in enumerate(
        zip(SEGMENT_COLORS, graph_labels, representative_segments)
    ):
        duration = segment["duration_s"]["mean"]
        ax_graph.annotate(
            "",
            xy=(index + 0.88, 0.35),
            xytext=(index + 0.12, 0.35),
            arrowprops={"arrowstyle": "-|>", "lw": 7, "color": color, "alpha": 0.82},
        )
        ax_graph.text(
            index + 0.5,
            0.72 if index % 2 == 0 else -0.10,
            f"{label}\n평균 {duration:.2f}s",
            ha="center",
            va="center",
            fontsize=9,
            bbox={"boxstyle": "round,pad=0.25", "fc": "white", "ec": color, "alpha": 0.95},
        )
    for index, node in enumerate(node_labels):
        if node in boundaries:
            boundary = boundaries[node]
            commit = boundary["calibrated_commit_support"]["radius_mm"]
            prearm = boundary["calibrated_prearm_support"]["radius_mm"]
            detail = f"\n{commit:.0f}/{prearm:.0f}mm"
            face = "#FFF3CD"
        elif node == "S5":
            detail = "\nO2 prearm 진입"
            face = "#E2E3FF"
        elif node == "END":
            detail = "\n녹화 종료(비계약)"
            face = "#F8D7DA"
        else:
            detail = ""
            face = "#DDEBF7"
        ax_graph.scatter(
            [index],
            [0.35],
            s=820,
            color=face,
            edgecolor="#222222",
            linewidth=1.2,
            zorder=5,
        )
        ax_graph.text(index, 0.35, node + detail, ha="center", va="center", fontsize=8, zorder=6)
    ax_graph.text(
        0.0,
        -0.92,
        "구형 경계 표기: calibrated commit/prearm 반경. S5는 O2 prearm sphere 최초 3-frame 진입.\n"
        "END는 에피소드 종료 프레임이며 semantic 완료 또는 실물 안전 경계로 검증되지 않음.",
        fontsize=9,
        color="#444444",
        va="top",
    )

    figure.suptitle(
        "T1 standardized semantic graph (30 episodes)\n"
        f'TASK_DESCRIPTION="{task_description}"',
        fontsize=17,
        fontweight="bold",
    )
    figure.savefig(path_png, bbox_inches="tight")
    figure.savefig(path_svg, bbox_inches="tight")
    plt.close(figure)


def _plot_diagnostics(
    path_png: Path,
    path_svg: Path,
    trajectories: list[Trajectory],
    episode_events: list[dict[str, int]],
    boundaries: dict[str, dict[str, Any]],
    representative_segments: list[dict[str, Any]],
    s5_indices: list[int],
    fps: int,
) -> None:
    _configure_plot_font()
    figure, axes = plt.subplots(2, 2, figsize=(17, 11), constrained_layout=True)
    ax_timing, ax_support, ax_radius, ax_segment = axes.flat

    for episode_index, events in enumerate(episode_events):
        times = [events[event] / fps for event in EVENT_IDS]
        ax_timing.plot(times, [episode_index] * 4, color="#D0D0D0", linewidth=0.8)
        for event, time, color in zip(EVENT_IDS, times, ("#D62728", "#1F77B4", "#D62728", "#1F77B4")):
            marker = "v" if event.startswith("C") else "^"
            ax_timing.scatter(time, episode_index, color=color, marker=marker, s=22)
        ax_timing.scatter(s5_indices[episode_index] / fps, episode_index, color="#9467BD", marker="|", s=36)
    ax_timing.set_xlabel("에피소드 시간 [s]")
    ax_timing.set_ylabel("episode index")
    ax_timing.set_title("A. 30개 에피소드의 C1-O1-C2-S5-O2 정렬", loc="left", fontweight="bold")
    ax_timing.grid(True, axis="x", alpha=0.2)

    x = np.arange(len(EVENT_IDS))
    commit_support = [boundaries[event]["reference_commit_support"]["fraction"] * 100 for event in EVENT_IDS]
    prearm_support = [boundaries[event]["reference_prearm_support"]["fraction"] * 100 for event in EVENT_IDS]
    width = 0.36
    ax_support.bar(x - width / 2, commit_support, width, label="20 mm commit", color="#4C78A8")
    ax_support.bar(x + width / 2, prearm_support, width, label="40 mm prearm", color="#F58518")
    ax_support.axhline(50, color="#4C78A8", linestyle="--", alpha=0.65, label="commit 기준 50%")
    ax_support.axhline(80, color="#F58518", linestyle="--", alpha=0.65, label="prearm 기준 80%")
    ax_support.set_xticks(x, EVENT_IDS)
    ax_support.set_ylim(0, 108)
    ax_support.set_ylabel("30개 중 sphere 내부 비율 [%]")
    ax_support.set_title("B. 기존 Task-C 20/40 mm 계약의 T1 지지율", loc="left", fontweight="bold")
    ax_support.legend(fontsize=8, ncols=2)
    ax_support.grid(True, axis="y", alpha=0.2)

    p50 = [boundaries[event]["distance_from_center"]["p50_mm"] for event in EVENT_IDS]
    p95 = [boundaries[event]["distance_from_center"]["p95_mm"] for event in EVENT_IDS]
    calibrated_commit = [boundaries[event]["calibrated_commit_support"]["radius_mm"] for event in EVENT_IDS]
    calibrated_prearm = [boundaries[event]["calibrated_prearm_support"]["radius_mm"] for event in EVENT_IDS]
    ax_radius.plot(x, p50, "o-", label="실측 거리 p50", color="#4C78A8")
    ax_radius.plot(x, p95, "o-", label="실측 거리 p95", color="#E45756")
    ax_radius.step(x, calibrated_commit, where="mid", label="calibrated commit", color="#4C78A8", linestyle="--")
    ax_radius.step(x, calibrated_prearm, where="mid", label="calibrated prearm", color="#E45756", linestyle="--")
    ax_radius.set_xticks(x, EVENT_IDS)
    ax_radius.set_ylabel("대표 중심으로부터 거리/반경 [mm]")
    ax_radius.set_title("C. 30개 분산으로 보정한 descriptive sphere", loc="left", fontweight="bold")
    ax_radius.legend(fontsize=8)
    ax_radius.grid(True, alpha=0.2)

    labels = [segment["definition"].segment_id for segment in representative_segments]
    means = [segment["duration_s"]["mean"] for segment in representative_segments]
    stds = [segment["duration_s"]["std"] for segment in representative_segments]
    bars = ax_segment.bar(np.arange(len(labels)), means, yerr=stds, color=SEGMENT_COLORS, capsize=4)
    ax_segment.set_xticks(np.arange(len(labels)), labels, rotation=20)
    ax_segment.set_ylabel("duration [s], mean ± std")
    ax_segment.set_title("D. semantic 구간 시간 분포", loc="left", fontweight="bold")
    ax_segment.grid(True, axis="y", alpha=0.2)
    for bar, value in zip(bars, means):
        ax_segment.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.15, f"{value:.2f}", ha="center", fontsize=8)

    figure.suptitle("T1 semantic graph diagnostics", fontsize=17, fontweight="bold")
    figure.savefig(path_png, bbox_inches="tight")
    figure.savefig(path_svg, bbox_inches="tight")
    plt.close(figure)


def _write_csv_outputs(
    output_dir: Path,
    trajectories: list[Trajectory],
    episode_events: list[dict[str, int]],
    segment_bounds: list[tuple[tuple[int, int], ...]],
    representative_segments: list[dict[str, Any]],
    boundaries: dict[str, dict[str, Any]],
    fps: int,
) -> None:
    with (output_dir / "episode_events.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = ["episode_index"]
        for event in EVENT_IDS:
            fields.extend([f"{event}_frame", f"{event}_time_s", f"{event}_x_mm", f"{event}_y_mm", f"{event}_z_mm"])
        fields.extend(["S5_frame", "S5_time_s"])
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for trajectory, events, bounds in zip(trajectories, episode_events, segment_bounds):
            row: dict[str, Any] = {"episode_index": trajectory.episode}
            for event in EVENT_IDS:
                index = events[event]
                position = trajectory.xyz_mm[index]
                row.update(
                    {
                        f"{event}_frame": int(trajectory.frame_index[index]),
                        f"{event}_time_s": float(trajectory.timestamp_s[index]),
                        f"{event}_x_mm": float(position[0]),
                        f"{event}_y_mm": float(position[1]),
                        f"{event}_z_mm": float(position[2]),
                    }
                )
            s5_index = bounds[3][1]
            row["S5_frame"] = int(trajectory.frame_index[s5_index])
            row["S5_time_s"] = float(trajectory.timestamp_s[s5_index])
            writer.writerow(row)

    with (output_dir / "semantic_segments.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = (
            "episode_index",
            "segment_index",
            "segment_id",
            "semantic_label",
            "label_ko",
            "start_frame",
            "end_frame",
            "start_time_s",
            "end_time_s",
            "duration_s",
            "path_length_mm",
            "entry_gripper_closed",
            "exit_gripper_closed",
            "confidence",
        )
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for trajectory, bounds in zip(trajectories, segment_bounds):
            for segment_index, (definition, (start, end)) in enumerate(zip(SEGMENT_DEFINITIONS, bounds)):
                path_length = float(np.linalg.norm(np.diff(trajectory.xyz_mm[start : end + 1], axis=0), axis=1).sum())
                writer.writerow(
                    {
                        "episode_index": trajectory.episode,
                        "segment_index": segment_index,
                        "segment_id": definition.segment_id,
                        "semantic_label": definition.semantic_label,
                        "label_ko": definition.label_ko,
                        "start_frame": int(trajectory.frame_index[start]),
                        "end_frame": int(trajectory.frame_index[end]),
                        "start_time_s": float(trajectory.timestamp_s[start]),
                        "end_time_s": float(trajectory.timestamp_s[end]),
                        "duration_s": float(trajectory.timestamp_s[end] - trajectory.timestamp_s[start]),
                        "path_length_mm": path_length,
                        "entry_gripper_closed": int(trajectory.gripper_closed[start]),
                        "exit_gripper_closed": int(trajectory.gripper_closed[end]),
                        "confidence": definition.confidence,
                    }
                )

    with (output_dir / "event_boundaries.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = (
            "event_id",
            "semantic_label",
            "center_x_mm",
            "center_y_mm",
            "center_z_mm",
            "time_mean_s",
            "time_std_s",
            "distance_p50_mm",
            "distance_p90_mm",
            "distance_p95_mm",
            "distance_max_mm",
            "reference_commit_radius_mm",
            "reference_commit_support",
            "reference_prearm_radius_mm",
            "reference_prearm_support",
            "calibrated_commit_radius_mm",
            "calibrated_commit_support",
            "calibrated_prearm_radius_mm",
            "calibrated_prearm_support",
        )
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for event in EVENT_IDS:
            item = boundaries[event]
            center = item["center_position_mm"]
            writer.writerow(
                {
                    "event_id": event,
                    "semantic_label": item["semantic_label"],
                    "center_x_mm": center[0],
                    "center_y_mm": center[1],
                    "center_z_mm": center[2],
                    "time_mean_s": item["event_time_s"]["mean"],
                    "time_std_s": item["event_time_s"]["std"],
                    "distance_p50_mm": item["distance_from_center"]["p50_mm"],
                    "distance_p90_mm": item["distance_from_center"]["p90_mm"],
                    "distance_p95_mm": item["distance_from_center"]["p95_mm"],
                    "distance_max_mm": item["distance_from_center"]["p100_mm"],
                    "reference_commit_radius_mm": item["reference_commit_support"]["radius_mm"],
                    "reference_commit_support": item["reference_commit_support"]["fraction"],
                    "reference_prearm_radius_mm": item["reference_prearm_support"]["radius_mm"],
                    "reference_prearm_support": item["reference_prearm_support"]["fraction"],
                    "calibrated_commit_radius_mm": item["calibrated_commit_support"]["radius_mm"],
                    "calibrated_commit_support": item["calibrated_commit_support"]["fraction"],
                    "calibrated_prearm_radius_mm": item["calibrated_prearm_support"]["radius_mm"],
                    "calibrated_prearm_support": item["calibrated_prearm_support"]["fraction"],
                }
            )

    with (output_dir / "representative_segments.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = (
            "segment_index",
            "segment_id",
            "semantic_label",
            "phase",
            "median_x_mm",
            "median_y_mm",
            "median_z_mm",
            "mean_x_mm",
            "mean_y_mm",
            "mean_z_mm",
            "residual_p50_mm",
            "residual_p90_mm",
            "residual_p95_mm",
            "elapsed_median_s",
        )
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for segment_index, segment in enumerate(representative_segments):
            definition = segment["definition"]
            for index, phase in enumerate(segment["phase"]):
                median = segment["median_xyz_mm"][index]
                mean = segment["mean_xyz_mm"][index]
                writer.writerow(
                    {
                        "segment_index": segment_index,
                        "segment_id": definition.segment_id,
                        "semantic_label": definition.semantic_label,
                        "phase": phase,
                        "median_x_mm": median[0],
                        "median_y_mm": median[1],
                        "median_z_mm": median[2],
                        "mean_x_mm": mean[0],
                        "mean_y_mm": mean[1],
                        "mean_z_mm": mean[2],
                        "residual_p50_mm": segment["residual_p50_mm"][index],
                        "residual_p90_mm": segment["residual_p90_mm"][index],
                        "residual_p95_mm": segment["residual_p95_mm"][index],
                        "elapsed_median_s": segment["elapsed_median_s"][index],
                    }
                )


def _write_npz(output_dir: Path, representative_segments: list[dict[str, Any]]) -> None:
    values: dict[str, np.ndarray] = {}
    for index, segment in enumerate(representative_segments):
        prefix = f"segment_{index}_{segment['definition'].segment_id}"
        for name in (
            "phase",
            "episode_xyz_mm",
            "median_xyz_mm",
            "mean_xyz_mm",
            "covariance_mm2",
            "mad_mm",
            "residual_p50_mm",
            "residual_p90_mm",
            "residual_p95_mm",
            "elapsed_median_s",
        ):
            values[f"{prefix}_{name}"] = np.asarray(segment[name])
    np.savez_compressed(output_dir / "standardized_semantic_trajectories.npz", **values)


def _report_markdown(
    *,
    dataset_root: Path,
    task_description: str,
    trajectories: list[Trajectory],
    boundaries: dict[str, dict[str, Any]],
    representative_segments: list[dict[str, Any]],
    s5_indices: list[int],
    episode_events: list[dict[str, int]],
    fps: int,
) -> str:
    rows = []
    for event in EVENT_IDS:
        item = boundaries[event]
        center = item["center_position_mm"]
        rows.append(
            "| {event} | {meaning} | `[{x:.1f}, {y:.1f}, {z:.1f}]` | "
            "{time:.2f}±{std:.2f} | {ref_c:.0f} mm: {ref_cs}/{count} | "
            "{ref_p:.0f} mm: {ref_ps}/{count} | {cal_c:.0f}/{cal_p:.0f} mm |".format(
                event=event,
                meaning=item["label_ko"],
                x=center[0],
                y=center[1],
                z=center[2],
                time=item["event_time_s"]["mean"],
                std=item["event_time_s"]["std"],
                ref_c=item["reference_commit_support"]["radius_mm"],
                ref_cs=item["reference_commit_support"]["episodes"],
                count=item["reference_commit_support"]["episode_count"],
                ref_p=item["reference_prearm_support"]["radius_mm"],
                ref_ps=item["reference_prearm_support"]["episodes"],
                cal_c=item["calibrated_commit_support"]["radius_mm"],
                cal_p=item["calibrated_prearm_support"]["radius_mm"],
            )
        )
    segment_rows = []
    for segment in representative_segments:
        definition = segment["definition"]
        segment_rows.append(
            f"| {definition.segment_id} | `{definition.semantic_label}` | "
            f"{definition.label_ko} | {segment['duration_s']['mean']:.2f}±"
            f"{segment['duration_s']['std']:.2f} | "
            f"{segment['path_length_mm']['median']:.1f} | "
            f"{segment['mean_residual_p90_mm']:.1f} | {definition.confidence} |"
        )
    s5_lead = np.asarray(
        [
            (events["O2"] - s5) / fps
            for events, s5 in zip(episode_events, s5_indices)
        ],
        dtype=np.float64,
    )
    o1 = boundaries["O1"]
    reference_fail = (
        o1["reference_commit_support"]["fraction"] < 0.5
        or o1["reference_prearm_support"]["fraction"] < 0.8
    )
    return f"""# T1 30-episode standardized semantic graph

생성일: 2026-08-23  
데이터셋: `{dataset_root}`  
TASK_DESCRIPTION: `{task_description}`

## 결론

T1의 {len(trajectories)}개 에피소드, 총 {sum(len(item.xyz_mm) for item in trajectories):,} frame은 모두
`C1 → O1 → C2 → O2` 순서를 만족했다. 각 semantic 구간을 물리 XYZ(mm)는
유지한 채 Cartesian arc-length `0..1`로 정렬하고, 기존 Task-C V1과 같은
component median을 대표 궤적으로 계산했다.

![T1 representative semantic graph](t1_standardized_semantic_graph.png)

![T1 boundary and timing diagnostics](t1_semantic_graph_diagnostics.png)

## Semantic subgoal

| 구간 | Semantic label | 의미 | 시간 mean±std [s] | 경로 길이 median [mm] | phase 평균 p90 residual [mm] | 신뢰도 |
|---|---|---|---:|---:|---:|---|
{chr(10).join(segment_rows)}

`S5`는 임의의 `O2-30 frame`이 아니다. 30개 O2 위치의 robust center와 p95로
구한 {boundaries['O2']['calibrated_prearm_support']['radius_mm']:.0f} mm prearm sphere에, 그리퍼가 닫힌 상태로
3 frame 연속 들어온 최초 지점이다. 30/30 에피소드에서 검출됐으며 O2보다
평균 {np.mean(s5_lead):.2f}±{np.std(s5_lead):.2f}초 앞선다.

## 구형 경계

| 이벤트 | 의미 | 대표 중심 XYZ [mm] | 이벤트 시간 [s] | 기존 commit 지지 | 기존 prearm 지지 | calibrated commit/prearm |
|---|---|---|---:|---:|---:|---:|
{chr(10).join(rows)}

기존 Task-C 계약은 `20 mm commit ≥ 50%`, `40 mm prearm ≥ 80%`를 기준으로
비교했다. C1, C2, O2는 이 기준을 만족하지만 O1은 commit
{o1['reference_commit_support']['fraction'] * 100:.1f}%, prearm
{o1['reference_prearm_support']['fraction'] * 100:.1f}%로 {'불충족한다' if reference_fail else '충족한다'}.
따라서 O1을 실물 runtime 경계로 바로 쓰면 안 된다. 이 보고서의 calibrated
sphere는 분포를 설명하기 위해 p50/p95를 5 mm 단위로 올림한 값이며, 충돌·IK·영상
상태를 검증한 실행 안전 반경이 아니다.

## 그래프 해석

- 흐린 선: 각 semantic 구간의 30개 표준화 source trajectory
- 굵은 색 선: 같은 phase에서 계산한 component-median 대표 궤적
- 안쪽 구: calibrated commit sphere
- 바깥 구: calibrated prearm sphere
- S5: O2 prearm sphere 진입점
- END: 40초 녹화의 마지막 frame이며 semantic 완료나 안전 home을 뜻하지 않음

## 직접 관측과 semantic 추론의 구분

- 직접 관측: TCP XYZ, timestamp, `gripper_commanded_state`, C/O 전환, 30개 반복성
- 메타데이터: TASK_DESCRIPTION
- 외부 semantic annotation: C1/O1을 컨테이너 조작, C2/O2를 블록 파지/방출로 해석
- `gripper_commanded_state`는 commanded state이며 물리 접촉력이나 실제 물체 보유를
  직접 측정하는 센서가 아니다.

## 산출물

- `semantic_graph.json`: 노드·edge·구형 경계·검증 결과
- `episode_events.csv`: 30개 C1/O1/C2/O2/S5 원시 경계
- `semantic_segments.csv`: 30×6개의 subgoal frame 범위
- `event_boundaries.csv`: 중심, 분산, 20/40 mm 및 calibrated sphere 지지율
- `representative_segments.csv`: phase별 median/mean/residual
- `standardized_semantic_trajectories.npz`: 30개 resample과 covariance/MAD

이 결과는 semantic dataset index를 위한 오프라인 분석물이다. 아직 ACT policy를
분할하거나 실물 로봇에서 subgoal 전환을 승인하는 실행 manifest는 아니다.
"""


def _write_checksums(output_dir: Path) -> None:
    paths = sorted(
        path
        for path in output_dir.iterdir()
        if path.is_file() and path.name != "checksums.sha256"
    )
    lines = []
    for path in paths:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        lines.append(f"{digest}  {path.name}")
    (output_dir / "checksums.sha256").write_text("\n".join(lines) + "\n", encoding="utf-8")


def analyze(
    *,
    dataset_root: Path,
    output_dir: Path,
    phase_points: int,
    reference_commit_mm: float,
    reference_prearm_mm: float,
    radius_step_mm: float,
    stable_frames: int,
) -> dict[str, Any]:
    if dataset_root.resolve() == output_dir.resolve() or dataset_root.resolve() in output_dir.resolve().parents:
        raise ValueError("output_dir must not be inside the source dataset")
    output_dir.mkdir(parents=True, exist_ok=True)
    trajectories, info = load_lerobot_trajectories(dataset_root, "t1")
    if len(trajectories) != 30:
        raise ValueError(f"T1 analysis requires 30 episodes, found {len(trajectories)}")
    task_descriptions = info.get("task_descriptions", [])
    if task_descriptions != [EXPECTED_TASK_DESCRIPTION]:
        raise ValueError(
            "unexpected T1 task description; refusing to apply the audited semantic map: "
            f"{task_descriptions}"
        )
    fps = int(info["fps"])
    episode_events = [_detect_coco_events(item) for item in trajectories]
    boundaries = {
        event: _boundary_record(
            trajectories,
            episode_events,
            event,
            reference_commit_mm=reference_commit_mm,
            reference_prearm_mm=reference_prearm_mm,
            radius_step_mm=radius_step_mm,
        )
        for event in EVENT_IDS
    }

    o2_boundary = boundaries["O2"]
    o2_center = np.asarray(o2_boundary["center_position_mm"])
    o2_prearm = float(o2_boundary["calibrated_prearm_support"]["radius_mm"])
    s5_indices = [
        _find_stable_sphere_entry(
            trajectory,
            first_index=events["C2"] + 1,
            last_index=events["O2"],
            center_mm=o2_center,
            radius_mm=o2_prearm,
            stable_frames=stable_frames,
        )
        for trajectory, events in zip(trajectories, episode_events)
    ]
    all_bounds = [
        _segment_bounds(events, s5, len(trajectory.xyz_mm) - 1)
        for trajectory, events, s5 in zip(trajectories, episode_events, s5_indices)
    ]
    representative_segments: list[dict[str, Any]] = []
    for segment_index, definition in enumerate(SEGMENT_DEFINITIONS):
        resampled = [
            _resample_arc_length(
                trajectory,
                bounds[segment_index][0],
                bounds[segment_index][1],
                phase_points,
            )
            for trajectory, bounds in zip(trajectories, all_bounds)
        ]
        representative_segments.append(_representative_segment(definition, resampled))

    manifest = {
        "schema_version": "a0509.semantic_graph.t1.v1",
        "created_date": "2026-08-23",
        "dataset_root": str(dataset_root.resolve()),
        "dataset_repo_id": "local/a0509_blue_block_t1_throw_away_inthe_container",
        "task_description": EXPECTED_TASK_DESCRIPTION,
        "episode_count": len(trajectories),
        "total_frames": int(sum(len(item.xyz_mm) for item in trajectories)),
        "fps": fps,
        "event_sequence": list(EVENT_IDS),
        "standardization": {
            "coordinate_frame": "Doosan base frame",
            "coordinate_units": "mm",
            "method": "per-semantic-segment normalized Cartesian arc length",
            "phase_points": phase_points,
            "representative": "component_median",
            "also_preserved": ["mean", "covariance", "MAD", "source residuals"],
            "explicitly_not": "coordinate z-score",
        },
        "semantic_evidence": {
            "direct": [
                "observation.state TCP XYZ",
                "observation.state gripper_commanded_state",
                "timestamps",
                "30-episode repetition",
            ],
            "metadata": ["TASK_DESCRIPTION"],
            "prior_review": ["multi-camera event-window inspection"],
            "warning": "semantic labels are external annotations, not ACT language inputs",
        },
        "boundaries": boundaries,
        "nodes": [
            {"node_id": "S0", "meaning": "recording_start_and_prep", "contractual": False},
            *[
                {
                    "node_id": event,
                    "meaning": EVENT_SEMANTICS[event],
                    "contractual": False,
                    "boundary_ref": event,
                }
                for event in EVENT_IDS[:3]
            ],
            {
                "node_id": "S5",
                "meaning": "stable_entry_into_O2_calibrated_prearm_sphere",
                "contractual": False,
                "boundary_ref": "O2",
                "stable_frames": stable_frames,
                "detected_episodes": len(s5_indices),
            },
            {
                "node_id": "O2",
                "meaning": EVENT_SEMANTICS["O2"],
                "contractual": False,
                "boundary_ref": "O2",
            },
            {
                "node_id": "END",
                "meaning": "recording_end_not_verified_semantic_completion",
                "contractual": False,
            },
        ],
        "edges": [
            {
                "edge_index": index,
                "segment_id": definition.segment_id,
                "semantic_label": definition.semantic_label,
                "label_ko": definition.label_ko,
                "expected_gripper": definition.expected_gripper,
                "confidence": definition.confidence,
                "duration_s": segment["duration_s"],
                "path_length_mm": segment["path_length_mm"],
                "mean_residual_p90_mm": segment["mean_residual_p90_mm"],
                "max_residual_p95_mm": segment["max_residual_p95_mm"],
            }
            for index, (definition, segment) in enumerate(zip(SEGMENT_DEFINITIONS, representative_segments))
        ],
        "safety": {
            "robot_executable": False,
            "dry_run_only": True,
            "original_dataset_modified": False,
            "calibrated_spheres_are_descriptive": True,
            "not_validated": ["IK", "collision", "payload", "visual predicates", "physical gripper feedback"],
        },
    }

    _write_csv_outputs(
        output_dir,
        trajectories,
        episode_events,
        all_bounds,
        representative_segments,
        boundaries,
        fps,
    )
    _write_npz(output_dir, representative_segments)
    (output_dir / "semantic_graph.json").write_text(
        json.dumps(_jsonable(manifest), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    _plot_representative_graph(
        output_dir / "t1_standardized_semantic_graph.png",
        output_dir / "t1_standardized_semantic_graph.svg",
        representative_segments,
        boundaries,
        EXPECTED_TASK_DESCRIPTION,
    )
    _plot_diagnostics(
        output_dir / "t1_semantic_graph_diagnostics.png",
        output_dir / "t1_semantic_graph_diagnostics.svg",
        trajectories,
        episode_events,
        boundaries,
        representative_segments,
        s5_indices,
        fps,
    )
    (output_dir / "README.md").write_text(
        _report_markdown(
            dataset_root=dataset_root,
            task_description=EXPECTED_TASK_DESCRIPTION,
            trajectories=trajectories,
            boundaries=boundaries,
            representative_segments=representative_segments,
            s5_indices=s5_indices,
            episode_events=episode_events,
            fps=fps,
        ),
        encoding="utf-8",
    )
    _write_checksums(output_dir)
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--phase-points", type=int, default=101)
    parser.add_argument("--reference-commit-mm", type=float, default=20.0)
    parser.add_argument("--reference-prearm-mm", type=float, default=40.0)
    parser.add_argument("--radius-step-mm", type=float, default=5.0)
    parser.add_argument("--stable-frames", type=int, default=3)
    args = parser.parse_args()
    if args.phase_points < 3:
        parser.error("--phase-points must be at least 3")
    if args.reference_commit_mm <= 0.0:
        parser.error("--reference-commit-mm must be positive")
    if args.reference_prearm_mm <= args.reference_commit_mm:
        parser.error("--reference-prearm-mm must exceed commit")
    if args.radius_step_mm <= 0.0:
        parser.error("--radius-step-mm must be positive")
    if args.stable_frames < 1:
        parser.error("--stable-frames must be positive")
    manifest = analyze(
        dataset_root=args.dataset_root,
        output_dir=args.output_dir,
        phase_points=args.phase_points,
        reference_commit_mm=args.reference_commit_mm,
        reference_prearm_mm=args.reference_prearm_mm,
        radius_step_mm=args.radius_step_mm,
        stable_frames=args.stable_frames,
    )
    print(
        json.dumps(
            {
                "output_dir": str(args.output_dir.resolve()),
                "episode_count": manifest["episode_count"],
                "total_frames": manifest["total_frames"],
                "event_sequence": manifest["event_sequence"],
                "robot_executable": manifest["safety"]["robot_executable"],
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
