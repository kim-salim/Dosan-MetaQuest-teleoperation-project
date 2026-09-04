#!/usr/bin/env python3
"""Render a command-free T7->T3 semantic-local path-length projection.

This is an analysis artifact, not a runtime planner.  It compares the latest
fixed T3-S2-phase-0 live Bridge with two offline boundary-search projections:

* semantic-support bounded: target remains in the 30-episode high-transport
  support before T3's descent/release approach;
* length-only: any T3 S2 phase up to 0.95 is admitted, illustrating the late
  entry bias of a pure path-length objective.

No ROS node, service, topic, policy checkpoint, or robot command is used.
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
T7_NPZ = ROOT / "docs/artifacts/t7_semantic_only_graph_v3_2026-08-29/standardized_semantic_phase_trajectories.npz"
T3_NPZ = ROOT / "docs/artifacts/t3_semantic_only_graph_v3_2026-08-23/standardized_semantic_phase_trajectories.npz"
SOURCE_BANK = ROOT / "docs/artifacts/t7_to_t3_s2_runtime_reference_2026-08-31/t7_s2_runtime_source_bank.json"
TRACE = Path("/home/rvlab/a0509_web_runs/20260902_170034_027437_960a0764011f/runtime_output/task_c_multi_v2_trace.jsonl")

FPS = 30.0
DURATION_S = 4.0
MAX_AXIS_STEP_MM = 7.5
MAX_AXIS_VELOCITY_MM_S = 225.0
MAX_ACCELERATION_MM_S2 = 4000.0
MAX_JERK_MM_S3 = 4000.0
SOURCE_PHASE_LOW = 0.16
SOURCE_PHASE_HIGH = 0.21
LENGTH_ONLY_B_PHASE_HIGH = 0.95

# This is not reintroduced as a live Z hard limit.  It is the existing
# 30-episode T3 high-transport support floor produced by the V1 standardizer.
# It only defines the bounded semantic initiation bank in this visualization.
T3_TRANSPORT_SUPPORT_FLOOR_MM = 412.7384063720703


def cumulative_length(xyz: np.ndarray) -> np.ndarray:
    return np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)))
    )


def bezier(
    p0: np.ndarray,
    v0: np.ndarray,
    p3: np.ndarray,
    duration_s: float,
    *,
    v3: np.ndarray | None = None,
) -> tuple[np.ndarray, dict[str, float]]:
    """Existing velocity-matched cubic; B endpoint defaults to settle."""

    v3_value = np.zeros(3, dtype=np.float64) if v3 is None else np.asarray(v3)
    p1 = p0 + duration_s * v0 / 3.0
    p2 = p3 - duration_s * v3_value / 3.0
    count = int(round(duration_s * FPS)) + 1
    u = np.linspace(0.0, 1.0, count)
    one = 1.0 - u
    position = (
        one[:, None] ** 3 * p0
        + 3.0 * one[:, None] ** 2 * u[:, None] * p1
        + 3.0 * one[:, None] * u[:, None] ** 2 * p2
        + u[:, None] ** 3 * p3
    )
    velocity = np.diff(position, axis=0) * FPS
    acceleration = np.diff(velocity, axis=0) * FPS
    jerk = np.diff(acceleration, axis=0) * FPS
    step = np.abs(np.diff(position, axis=0))
    metrics = {
        "length_mm": float(np.sum(np.linalg.norm(np.diff(position, axis=0), axis=1))),
        "max_axis_step_mm": float(np.max(step)),
        "max_axis_velocity_mm_s": float(np.max(np.abs(velocity))),
        "max_acceleration_mm_s2": float(
            np.max(np.linalg.norm(acceleration, axis=1))
        ),
        "max_jerk_mm_s3": float(np.max(np.linalg.norm(jerk, axis=1))),
    }
    metrics["hard_pass"] = bool(
        metrics["max_axis_step_mm"] <= MAX_AXIS_STEP_MM
        and metrics["max_axis_velocity_mm_s"] <= MAX_AXIS_VELOCITY_MM_S
        and metrics["max_acceleration_mm_s2"] <= MAX_ACCELERATION_MM_S2
        and metrics["max_jerk_mm_s3"] <= MAX_JERK_MM_S3
    )
    return position, metrics


def source_samples() -> list[dict[str, object]]:
    bank = json.loads(SOURCE_BANK.read_text(encoding="utf-8"))
    medoid_episode = int(bank["selection"]["medoid_episode"])
    episode = next(
        item for item in bank["episodes"] if int(item["episode"]) == medoid_episode
    )
    return [
        sample
        for sample in episode["samples"]
        if SOURCE_PHASE_LOW - 1e-9
        <= float(sample["phase"])
        <= SOURCE_PHASE_HIGH + 1e-9
    ]


def latest_bridge_xyz() -> np.ndarray:
    records: dict[int, np.ndarray] = {}
    for line in TRACE.read_text(encoding="utf-8").splitlines():
        row = json.loads(line)
        index = row.get("bridge_index")
        if (
            row.get("record_type") == "control_command"
            and index is not None
            and row.get("state") in {"RUN_BRIDGE", "HANDOFF_WINDOW"}
        ):
            records[int(index)] = np.asarray(row["action"][:3], dtype=np.float64)
    if not records:
        raise RuntimeError("latest trace contains no Bridge commands")
    return np.stack([records[index] for index in sorted(records)], axis=0)


def search(
    *,
    a_episode_xyz: np.ndarray,
    b_episode_xyz: np.ndarray,
    b_phase: np.ndarray,
    b_indices: np.ndarray,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    a_arc = cumulative_length(a_episode_xyz)
    b_arc = cumulative_length(b_episode_xyz)
    source = source_samples()
    rows: list[dict[str, object]] = []
    for sample in source:
        a_phase = float(sample["phase"])
        a_index = int(round(a_phase * 100.0))
        p0 = np.asarray(sample["position_mm"], dtype=np.float64)
        v0 = np.asarray(sample["velocity_mm_s"], dtype=np.float64)
        for b_index in b_indices:
            path, metrics = bezier(p0, v0, b_episode_xyz[b_index], DURATION_S)
            row: dict[str, object] = {
                "a_phase": a_phase,
                "b_phase": float(b_phase[b_index]),
                "a_semantic_prefix_mm": float(a_arc[a_index]),
                "bridge_length_mm": metrics["length_mm"],
                "b_semantic_suffix_mm": float(b_arc[-1] - b_arc[b_index]),
                "semantic_total_mm": float(
                    a_arc[a_index]
                    + metrics["length_mm"]
                    + b_arc[-1]
                    - b_arc[b_index]
                ),
                "source_position_mm": p0.tolist(),
                "source_velocity_mm_s": v0.tolist(),
                "target_position_mm": b_episode_xyz[b_index].tolist(),
                "path_mm": path,
                **metrics,
            }
            rows.append(row)
    feasible = [row for row in rows if bool(row["hard_pass"])]
    if not feasible:
        raise RuntimeError("semantic-local search produced no hard-pass candidate")
    selected = min(
        feasible,
        key=lambda row: (
            float(row["semantic_total_mm"]),
            float(row["max_jerk_mm_s3"]),
        ),
    )
    return selected, rows


def serializable(row: dict[str, object]) -> dict[str, object]:
    return {
        key: value.tolist() if isinstance(value, np.ndarray) else value
        for key, value in row.items()
    }


def main() -> None:
    a = np.load(T7_NPZ)
    b = np.load(T3_NPZ)
    a_phase = a["S2_phase"]
    b_phase = b["S2_phase"]
    a_median = a["S2_median_xyz_mm"]
    b_median = b["S2_median_xyz_mm"]
    a_episode_ids = a["S2_episode_ids"]
    b_episode_ids = b["S2_episode_ids"]
    a_medoid_index = int(np.flatnonzero(a_episode_ids == 15)[0])
    b_support_index = int(np.flatnonzero(b_episode_ids == 17)[0])
    a_episode = a["S2_episode_xyz_mm"][a_medoid_index]
    b_episode = b["S2_episode_xyz_mm"][b_support_index]

    support_indices = np.flatnonzero(
        (b_episode[:, 2] >= T3_TRANSPORT_SUPPORT_FLOOR_MM)
        & (b_phase <= LENGTH_ONLY_B_PHASE_HIGH)
    )
    length_only_indices = np.flatnonzero(b_phase <= LENGTH_ONLY_B_PHASE_HIGH)
    support_selected, support_rows = search(
        a_episode_xyz=a_episode,
        b_episode_xyz=b_episode,
        b_phase=b_phase,
        b_indices=support_indices,
    )
    length_selected, length_rows = search(
        a_episode_xyz=a_episode,
        b_episode_xyz=b_episode,
        b_phase=b_phase,
        b_indices=length_only_indices,
    )
    current_bridge = latest_bridge_xyz()

    # Current fixed endpoint comparison uses the same semantic S2 paths, but
    # the Bridge length is measured from the actual recorded proposal queue.
    a_arc = cumulative_length(a_episode)
    b_arc = cumulative_length(b_episode)
    current_a_phase = 0.185
    current_a_index = int(round(current_a_phase * 100.0))
    current_bridge_length = float(
        np.sum(np.linalg.norm(np.diff(current_bridge, axis=0), axis=1))
    )
    current_total = float(
        a_arc[current_a_index] + current_bridge_length + b_arc[-1]
    )

    font = "NanumSquare"
    plt.rcParams.update({"font.family": font, "axes.unicode_minus": False})
    fig = plt.figure(figsize=(17.0, 10.5), constrained_layout=True)
    grid = fig.add_gridspec(2, 3, height_ratios=[1.12, 1.0])
    ax3d = fig.add_subplot(grid[0, :2], projection="3d")
    ax_obj = fig.add_subplot(grid[0, 2])
    ax_xy = fig.add_subplot(grid[1, 0])
    ax_yz = fig.add_subplot(grid[1, 1])
    ax_bar = fig.add_subplot(grid[1, 2])

    support_path = np.asarray(support_selected["path_mm"])
    length_path = np.asarray(length_selected["path_mm"])
    colors = {
        "a": "#F59E0B",
        "b": "#2563EB",
        "current": "#A855F7",
        "support": "#10B981",
        "length": "#64748B",
    }

    for episode in a["S2_episode_xyz_mm"]:
        ax3d.plot(*episode.T, color=colors["a"], alpha=0.045, linewidth=0.7)
    for episode in b["S2_episode_xyz_mm"]:
        ax3d.plot(*episode.T, color=colors["b"], alpha=0.045, linewidth=0.7)
    ax3d.plot(*a_median.T, color=colors["a"], linewidth=2.2, label="T7 S2 median")
    ax3d.plot(*b_median.T, color=colors["b"], linewidth=2.2, label="T3 S2 median")
    ax3d.plot(*current_bridge.T, color=colors["current"], linewidth=3.0, label="현재 fixed B=0 Bridge")
    ax3d.plot(*support_path.T, color=colors["support"], linewidth=3.3, label="semantic-support 최소 경로")
    ax3d.plot(*length_path.T, color=colors["length"], linewidth=2.0, linestyle="--", label="순수 길이 최소(비교용)")
    for row, color, marker, label in (
        (support_selected, colors["support"], "o", "권장 B entry"),
        (length_selected, colors["length"], "X", "길이-only B entry"),
    ):
        target = np.asarray(row["target_position_mm"])
        ax3d.scatter(*target, color=color, marker=marker, s=85, label=label)
    ax3d.scatter(*b_episode[0], color=colors["current"], marker="s", s=70, label="현재 B phase 0")
    ax3d.set_xlabel("X [mm]")
    ax3d.set_ylabel("Y [mm]")
    ax3d.set_zlabel("Z [mm]")
    ax3d.set_title("T7→T3 예상 Bridge: semantic-local 길이 최소화")
    ax3d.view_init(elev=25, azim=-58)
    ax3d.legend(loc="upper left", fontsize=8)

    def objective_curve(rows: list[dict[str, object]]) -> tuple[np.ndarray, np.ndarray]:
        grouped: dict[float, float] = {}
        for row in rows:
            if not bool(row["hard_pass"]):
                continue
            phase = float(row["b_phase"])
            grouped[phase] = min(
                grouped.get(phase, float("inf")), float(row["semantic_total_mm"])
            )
        phase = np.asarray(sorted(grouped), dtype=np.float64)
        return phase, np.asarray([grouped[item] for item in phase])

    obj_phase, obj_value = objective_curve(length_rows)
    ax_obj.plot(obj_phase, obj_value, color="#334155", linewidth=2.2)
    ax_obj.axvspan(
        float(b_phase[support_indices[0]]),
        float(b_phase[support_indices[-1]]),
        color=colors["support"],
        alpha=0.15,
        label="30-episode transport support",
    )
    ax_obj.scatter([0.0], [current_total], color=colors["current"], marker="s", s=55, zorder=5)
    for row, color, marker in (
        (support_selected, colors["support"], "o"),
        (length_selected, colors["length"], "X"),
    ):
        ax_obj.scatter(
            [row["b_phase"]], [row["semantic_total_mm"]], color=color, marker=marker, s=75, zorder=5
        )
    ax_obj.set_xlabel("T3 S2 entry phase")
    ax_obj.set_ylabel("semantic-local total [mm]")
    ax_obj.set_title("B entry에 따른 비용\n(최소화만 하면 오른쪽 경계로 치우침)")
    ax_obj.grid(alpha=0.25)
    ax_obj.legend(fontsize=8)

    def plot_projection(ax: plt.Axes, axes: tuple[int, int], labels: tuple[str, str]) -> None:
        i, j = axes
        ax.plot(a_median[:, i], a_median[:, j], color=colors["a"], linewidth=2.0, label="T7 S2")
        ax.plot(b_median[:, i], b_median[:, j], color=colors["b"], linewidth=2.0, label="T3 S2")
        ax.plot(current_bridge[:, i], current_bridge[:, j], color=colors["current"], linewidth=2.5, label="현재")
        ax.plot(support_path[:, i], support_path[:, j], color=colors["support"], linewidth=2.8, label="support 최소")
        ax.plot(length_path[:, i], length_path[:, j], color=colors["length"], linewidth=1.8, linestyle="--", label="길이-only")
        ax.scatter(b_episode[0, i], b_episode[0, j], color=colors["current"], marker="s", s=45)
        ax.scatter(float(support_selected["target_position_mm"][i]), float(support_selected["target_position_mm"][j]), color=colors["support"], s=55)
        ax.scatter(float(length_selected["target_position_mm"][i]), float(length_selected["target_position_mm"][j]), color=colors["length"], marker="X", s=60)
        ax.set_xlabel(f"{labels[0]} [mm]")
        ax.set_ylabel(f"{labels[1]} [mm]")
        ax.grid(alpha=0.25)
        ax.axis("equal")

    plot_projection(ax_xy, (0, 1), ("X", "Y"))
    ax_xy.set_title("상면도 (XY)")
    ax_xy.legend(fontsize=8)
    plot_projection(ax_yz, (1, 2), ("Y", "Z"))
    ax_yz.set_title("측면도 (YZ)")

    labels = ["현재\nB=0", "semantic support\n최소", "순수 길이\n최소"]
    rows = [
        {
            "a": float(a_arc[current_a_index]),
            "bridge": current_bridge_length,
            "b": float(b_arc[-1]),
        },
        {
            "a": float(support_selected["a_semantic_prefix_mm"]),
            "bridge": float(support_selected["bridge_length_mm"]),
            "b": float(support_selected["b_semantic_suffix_mm"]),
        },
        {
            "a": float(length_selected["a_semantic_prefix_mm"]),
            "bridge": float(length_selected["bridge_length_mm"]),
            "b": float(length_selected["b_semantic_suffix_mm"]),
        },
    ]
    x = np.arange(3)
    bottom = np.zeros(3)
    for key, name, color in (
        ("a", "T7 semantic retained", colors["a"]),
        ("bridge", "Bridge", colors["support"]),
        ("b", "T3 semantic remaining", colors["b"]),
    ):
        values = np.asarray([row[key] for row in rows])
        ax_bar.bar(x, values, bottom=bottom, label=name, color=color, alpha=0.9)
        bottom += values
    for index, total in enumerate(bottom):
        ax_bar.text(index, total + 12.0, f"{total:.0f} mm", ha="center", fontsize=9)
    ax_bar.set_xticks(x, labels)
    ax_bar.set_ylabel("local path length [mm]")
    ax_bar.set_title("비용 구성")
    ax_bar.legend(fontsize=8)
    ax_bar.grid(axis="y", alpha=0.2)

    fig.suptitle(
        "T7.acquire_from_drawer_top → T3.deliver_to_floor\n"
        "오프라인 예상치 · 실제 명령 없음 · orientation/IK/collision 미검증",
        fontsize=15,
    )
    png = OUT / "t7_to_t3_semantic_local_length_projection.png"
    svg = OUT / "t7_to_t3_semantic_local_length_projection.svg"
    fig.savefig(png, dpi=180)
    fig.savefig(svg)
    plt.close(fig)

    report = {
        "schema_version": "a0509.t7_t3.semantic_local_length_projection.v1",
        "scope": "offline_command_free_visualization",
        "objective": "A semantic prefix + Bridge + B semantic suffix",
        "source_operator": "T7.acquire_from_drawer_top",
        "successor_operator": "T3.deliver_to_floor",
        "source_phase_window": [SOURCE_PHASE_LOW, SOURCE_PHASE_HIGH],
        "duration_s": DURATION_S,
        "current_fixed_phase_zero": {
            "a_phase": current_a_phase,
            "b_phase": 0.0,
            "a_semantic_prefix_mm": float(a_arc[current_a_index]),
            "bridge_length_mm": current_bridge_length,
            "b_semantic_suffix_mm": float(b_arc[-1]),
            "semantic_total_mm": current_total,
            "trace": str(TRACE),
        },
        "semantic_support_bounded_selection": serializable(support_selected),
        "length_only_selection_for_bias_demonstration": serializable(length_selected),
        "semantic_support_definition": {
            "role": "candidate ranking domain, not live Z safety hard limit",
            "t3_support_floor_mm": T3_TRANSPORT_SUPPORT_FLOOR_MM,
            "b_phase_low": float(b_phase[support_indices[0]]),
            "b_phase_high": float(b_phase[support_indices[-1]]),
            "provenance": "30-episode V1 T3 high-transport standardization",
        },
        "hard_checks_applied_xyz_only": {
            "max_axis_step_mm": MAX_AXIS_STEP_MM,
            "max_axis_velocity_mm_s": MAX_AXIS_VELOCITY_MM_S,
            "max_acceleration_mm_s2": MAX_ACCELERATION_MM_S2,
            "max_jerk_mm_s3": MAX_JERK_MM_S3,
        },
        "not_validated": [
            "orientation interpolation/admission",
            "fresh ACT-B prefix compatibility",
            "actual/ACK tracking during the projected path",
            "IK",
            "collision and held-object collision envelope",
            "physical robot execution",
        ],
        "outputs": {"png": str(png), "svg": str(svg)},
    }
    (OUT / "t7_to_t3_semantic_local_length_projection.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
