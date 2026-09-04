"""Dry-run 3D path and bridge dynamics plots."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from .bridge_metrics import evaluate_bridge
from .bridge_optimizer import CandidateEvaluation


COLORS = {
    "a_retained": "#007C91",
    "a_removed": "#9AC7CE",
    "bridge": "#D1495B",
    "b_removed": "#C6B8D9",
    "b_retained": "#4C956C",
}


def plot_selected_path(
    selected: CandidateEvaluation,
    *,
    sample_hz: float,
    curvature_epsilon: float,
    workspace_min_mm: np.ndarray,
    workspace_max_mm: np.ndarray,
    output: Path,
) -> None:
    _, sample = evaluate_bridge(
        selected.bridge,
        sample_hz=sample_hz,
        curvature_epsilon=curvature_epsilon,
        workspace_min_mm=workspace_min_mm,
        workspace_max_mm=workspace_max_mm,
    )
    a = selected.a
    b = selected.b
    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection="3d")
    segments = (
        (a.trajectory.xyz_mm[: a.index + 1], "a_retained", "A retained"),
        (a.trajectory.xyz_mm[a.index :], "a_removed", "A removed tail"),
        (sample["position_mm"], "bridge", "Selected velocity-matched bridge"),
        (b.trajectory.xyz_mm[: b.index + 1], "b_removed", "B removed head"),
        (b.trajectory.xyz_mm[b.index :], "b_retained", "B retained"),
    )
    for points, color, label in segments:
        ax.plot(*points.T, color=COLORS[color], linewidth=3 if color == "bridge" else 1.8, label=label)
    ax.scatter(*a.position_mm, color=COLORS["bridge"], marker="o", s=70)
    ax.scatter(*b.position_mm, color=COLORS["bridge"], marker="X", s=70)
    arrow_scale = 0.35
    ax.quiver(*a.position_mm, *(a.velocity_mm_s * arrow_scale), color="#1B1B1B", normalize=False)
    ax.quiver(*b.position_mm, *(b.velocity_mm_s * arrow_scale), color="#6A1B9A", normalize=False)
    all_points = np.concatenate([a.trajectory.xyz_mm, b.trajectory.xyz_mm, sample["position_mm"]])
    ax.set_box_aspect(np.maximum(np.ptp(all_points, axis=0), 1e-3))
    ax.set_xlabel("TCP X (mm)")
    ax.set_ylabel("TCP Y (mm)")
    ax.set_zlabel("TCP Z (mm)")
    ax.set_title(
        f"Task-C V0 dry run: A ep{a.trajectory.episode} f{a.frame} -> "
        f"B ep{b.trajectory.episode} f{b.frame}\n"
        f"T={selected.bridge.duration_s:.2f}s, total C={selected.total_c_length_mm:.1f} mm, "
        f"payload={a.holding_assumption}"
    )
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=180, facecolor="white")
    plt.close(fig)


def plot_selected_dynamics(
    selected: CandidateEvaluation,
    *,
    sample_hz: float,
    curvature_epsilon: float,
    workspace_min_mm: np.ndarray,
    workspace_max_mm: np.ndarray,
    output: Path,
) -> None:
    _, sample = evaluate_bridge(
        selected.bridge,
        sample_hz=sample_hz,
        curvature_epsilon=curvature_epsilon,
        workspace_min_mm=workspace_min_mm,
        workspace_max_mm=workspace_max_mm,
    )
    time = sample["time_s"]
    fig, axes = plt.subplots(5, 1, figsize=(12, 14), sharex=True)
    position = sample["position_mm"]
    velocity = sample["velocity_mm_s"]
    axes[0].plot(time, position)
    axes[0].set_ylabel("position (mm)")
    axes[0].legend(("x", "y", "z"), ncol=3)
    axes[1].plot(time, velocity)
    axes[1].plot(time, sample["speed_mm_s"], color="#161616", linewidth=2, label="|v|")
    axes[1].set_ylabel("velocity (mm/s)")
    axes[1].legend(("vx", "vy", "vz", "|v|"), ncol=4)
    axes[2].plot(time, sample["acceleration_magnitude_mm_s2"], color="#E4572E")
    axes[2].set_ylabel("|a| (mm/s2)")
    axes[3].plot(time, sample["curvature_per_mm"], color="#6A4C93")
    axes[3].set_ylabel("curvature (1/mm)")
    axes[4].plot(time, sample["jerk_magnitude_mm_s3"], color="#4C956C")
    axes[4].set_ylabel("|jerk| (mm/s3)")
    axes[4].set_xlabel("bridge time (s)")
    for ax in axes:
        ax.grid(alpha=0.22)
    fig.suptitle("Selected Task-C bridge dynamics (position-only dry run)")
    fig.tight_layout()
    fig.savefig(output, dpi=180, facecolor="white")
    plt.close(fig)


def plot_runtime_model_conditioned_path(
    candidate: object,
    *,
    a_history_mm: np.ndarray,
    measured_velocity_a_mm_s: np.ndarray,
    sample_hz: float,
    curvature_epsilon: float,
    workspace_min_mm: np.ndarray,
    workspace_max_mm: np.ndarray,
    output: Path,
) -> None:
    """Plot a runtime candidate whose terminal velocity came from ACT-B."""

    _, sample = evaluate_bridge(
        candidate.bridge,
        sample_hz=sample_hz,
        curvature_epsilon=curvature_epsilon,
        workspace_min_mm=workspace_min_mm,
        workspace_max_mm=workspace_max_mm,
    )
    history = np.asarray(a_history_mm, dtype=np.float64)
    velocity_a = np.asarray(measured_velocity_a_mm_s, dtype=np.float64)
    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(*history.T, color=COLORS["a_retained"], linewidth=2.0, label="A causal TCP history")
    ax.plot(
        *sample["position_mm"].T,
        color=COLORS["bridge"],
        linewidth=3.0,
        label="ACT-B velocity-conditioned bridge",
    )
    ax.scatter(*candidate.bridge.p0, color="#1B1B1B", marker="o", s=70)
    ax.scatter(*candidate.bridge.p3, color="#6A1B9A", marker="X", s=80)
    arrow_scale = 0.35
    ax.quiver(
        *candidate.bridge.p0,
        *(velocity_a * arrow_scale),
        color="#1B1B1B",
        normalize=False,
        label="measured/replayed vA",
    )
    ax.quiver(
        *candidate.bridge.p3,
        *(candidate.terminal_velocity_mm_s * arrow_scale),
        color="#6A1B9A",
        normalize=False,
        label="fresh ACT-B vB",
    )
    all_points = np.concatenate((history, sample["position_mm"]), axis=0)
    ax.set_box_aspect(np.maximum(np.ptp(all_points, axis=0), 1e-3))
    ax.set_xlabel("TCP X (mm)")
    ax.set_ylabel("TCP Y (mm)")
    ax.set_zlabel("TCP Z (mm)")
    ax.set_title(
        "Task-C model-conditioned dry run\n"
        f"B ep{candidate.entry.episode} f{candidate.entry.frame}, "
        f"T={candidate.bridge.duration_s:.2f}s, "
        f"total estimate={candidate.total_c_estimate_mm:.1f} mm"
    )
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=180, facecolor="white")
    plt.close(fig)


def plot_runtime_model_conditioned_dynamics(
    candidate: object,
    *,
    sample_hz: float,
    curvature_epsilon: float,
    workspace_min_mm: np.ndarray,
    workspace_max_mm: np.ndarray,
    output: Path,
) -> None:
    _, sample = evaluate_bridge(
        candidate.bridge,
        sample_hz=sample_hz,
        curvature_epsilon=curvature_epsilon,
        workspace_min_mm=workspace_min_mm,
        workspace_max_mm=workspace_max_mm,
    )
    time = sample["time_s"]
    fig, axes = plt.subplots(5, 1, figsize=(12, 14), sharex=True)
    axes[0].plot(time, sample["position_mm"])
    axes[0].set_ylabel("position (mm)")
    axes[0].legend(("x", "y", "z"), ncol=3)
    axes[1].plot(time, sample["velocity_mm_s"])
    axes[1].plot(time, sample["speed_mm_s"], color="#161616", linewidth=2)
    axes[1].set_ylabel("velocity (mm/s)")
    axes[1].legend(("vx", "vy", "vz", "|v|"), ncol=4)
    axes[2].plot(time, sample["acceleration_magnitude_mm_s2"], color="#E4572E")
    axes[2].set_ylabel("|a| (mm/s2)")
    axes[3].plot(time, sample["curvature_per_mm"], color="#6A4C93")
    axes[3].set_ylabel("curvature (1/mm)")
    axes[4].plot(time, sample["jerk_magnitude_mm_s3"], color="#4C956C")
    axes[4].set_ylabel("|jerk| (mm/s3)")
    axes[4].set_xlabel("bridge time (s)")
    for axis in axes:
        axis.grid(alpha=0.22)
    fig.suptitle("Fresh ACT-B velocity-conditioned bridge dynamics")
    fig.tight_layout()
    fig.savefig(output, dpi=180, facecolor="white")
    plt.close(fig)
