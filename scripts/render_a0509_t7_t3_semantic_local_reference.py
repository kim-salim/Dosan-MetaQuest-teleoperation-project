#!/usr/bin/env python3
"""Render the selected T7->T3 semantic-local FLEXIBLE_LEVEL2 reference."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
for value in (
    REPOSITORY_ROOT,
    REPOSITORY_ROOT / "src/lerobot_robot_doosan_a0509",
    REPOSITORY_ROOT / "src/quest_a0509_teleop",
):
    if str(value) not in sys.path:
        sys.path.insert(0, str(value))

from lerobot_robot_doosan_a0509.task_c_handoff.flexible_bridge import (  # noqa: E402
    FlexibleBridgeSearchConfig,
    nominal_medoid_bridge_snapshot,
    search_flexible_bridge_queue,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (  # noqa: E402
    EpisodeHandoffManifest,
    HandoffV2Config,
)
from offline_tools.cross_task_handoff.evaluate_flexible_level2 import (  # noqa: E402
    runtime_limits_from_validation,
)


DEFAULT_ARTIFACT_ROOT = (
    REPOSITORY_ROOT
    / "docs/artifacts/t7_to_t3_s2_runtime_reference_2026-08-31"
)


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--artifact-root", type=Path, default=DEFAULT_ARTIFACT_ROOT
    )
    return parser.parse_args()


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> None:
    args = _args()
    root = args.artifact_root.expanduser().resolve()
    manifest = EpisodeHandoffManifest.load(root / "flexible_reference_manifest.json")
    selection = _load(root / "semantic_local_reference_selection.json")
    source_bank = _load(root / "t7_s2_runtime_source_bank.json")
    successor_bank = _load(root / "t3_s2_entry_reference_bank.json")
    validation = _load(
        REPOSITORY_ROOT
        / "offline_tools/cross_task_handoff/validation_config_a0509_v2.json"
    )
    limits = runtime_limits_from_validation(validation)
    runtime = HandoffV2Config(
        enabled=True,
        control_hz=30.0,
        bridge_admission_mode="flexible_level2",
        adaptive_b_max_splice_index=8,
        adaptive_b_max_candidates=6,
        semantic_authority="external_planner",
    )
    snapshot = nominal_medoid_bridge_snapshot(manifest, timestamp_s=0.0)
    result = search_flexible_bridge_queue(
        manifest,
        snapshot,
        runtime,
        limits,
        FlexibleBridgeSearchConfig(max_search_time_s=0.20),
        live_mode=False,
    )
    if result.queue is None:
        raise RuntimeError("selected manifest no longer produces a safe Bridge")
    bridge = np.concatenate(
        (
            manifest.source.nominal_position_mm[None, :],
            result.queue.actions[:, :3],
        ),
        axis=0,
    )

    source_npz = np.load(source_bank["phase_support_artifact"])
    successor_npz = np.load(successor_bank["phase_support_artifact"])
    source_paths = source_npz["S2_episode_xyz_mm"]
    successor_paths = successor_npz["S2_episode_xyz_mm"]
    source_median = source_npz["S2_median_xyz_mm"]
    successor_median = successor_npz["S2_median_xyz_mm"]

    candidates = [
        item for item in selection["candidates"] if item["geometry_valid"]
    ]
    by_phase: dict[float, float] = {}
    for item in candidates:
        phase = float(item["successor_phase"])
        total = float(item["total_length_mm"])
        by_phase[phase] = min(by_phase.get(phase, float("inf")), total)
    phases = np.asarray(sorted(by_phase), dtype=np.float64)
    totals = np.asarray([by_phase[item] for item in phases])

    colors = {
        "source": "#F59E0B",
        "successor": "#2563EB",
        "bridge": "#10B981",
        "excluded": "#EF4444",
    }
    fig = plt.figure(figsize=(16, 9.5), constrained_layout=True)
    grid = fig.add_gridspec(2, 2)
    ax3d = fig.add_subplot(grid[0, 0], projection="3d")
    ax_xz = fig.add_subplot(grid[1, 0])
    ax_obj = fig.add_subplot(grid[0, 1])
    ax_bar = fig.add_subplot(grid[1, 1])

    for path in source_paths:
        ax3d.plot(*path.T, color=colors["source"], alpha=0.05, linewidth=0.7)
    for path in successor_paths:
        ax3d.plot(*path.T, color=colors["successor"], alpha=0.05, linewidth=0.7)
    ax3d.plot(*source_median.T, color=colors["source"], linewidth=2.0, label="T7 S2 median")
    ax3d.plot(*successor_median.T, color=colors["successor"], linewidth=2.0, label="T3 S2 median")
    ax3d.plot(*bridge.T, color=colors["bridge"], linewidth=3.2, label="selected flexible Bridge")
    ax3d.scatter(*bridge[0], color=colors["source"], s=70, marker="o", label=f"A ref phase {manifest.source.phase:.2f}")
    ax3d.scatter(*bridge[-1], color=colors["successor"], s=80, marker="D", label=f"B ref phase {manifest.successor.phase:.2f}")
    ax3d.set(xlabel="X [mm]", ylabel="Y [mm]", zlabel="Z [mm]")
    ax3d.set_title("T7 -> T3 semantic-local Bridge reference")
    ax3d.view_init(elev=25, azim=-58)
    ax3d.legend(fontsize=8)

    ax_xz.plot(source_median[:, 0], source_median[:, 2], color=colors["source"], label="T7 S2 median")
    ax_xz.plot(successor_median[:, 0], successor_median[:, 2], color=colors["successor"], label="T3 S2 median")
    ax_xz.plot(bridge[:, 0], bridge[:, 2], color=colors["bridge"], linewidth=3, label="Bridge")
    ax_xz.scatter(bridge[[0, -1], 0], bridge[[0, -1], 2], c=[colors["source"], colors["successor"]], s=65)
    ax_xz.set(xlabel="X [mm]", ylabel="Z [mm]", title="Height profile (no runtime Z hard gate)")
    ax_xz.grid(alpha=0.25)
    ax_xz.legend(fontsize=8)

    low, high = manifest.successor_reference_phase_window
    ax_obj.plot(phases, totals, color="#334155", linewidth=2.2, marker="o", markersize=3)
    ax_obj.axvspan(low, high, color=colors["successor"], alpha=0.08, label="30-episode support")
    ax_obj.axvspan(manifest.successor.phase, high, color=colors["excluded"], alpha=0.12, label="15 mm end-margin exclusion")
    ax_obj.scatter([manifest.successor.phase], [manifest.semantic_local_total_length_mm], color=colors["bridge"], s=90, zorder=5, label="selected")
    ax_obj.set(xlabel="T3 S2 reference phase", ylabel="A prefix + Bridge + B suffix [mm]", title="Semantic-local objective")
    ax_obj.grid(alpha=0.25)
    ax_obj.legend(fontsize=8)

    labels = ["T7 prefix", "Bridge", "T3 suffix"]
    values = [
        manifest.semantic_local_source_prefix_mm,
        manifest.semantic_local_bridge_length_mm,
        manifest.semantic_local_successor_suffix_mm,
    ]
    bars = ax_bar.bar(labels, values, color=[colors["source"], colors["bridge"], colors["successor"]])
    for bar, value in zip(bars, values, strict=True):
        ax_bar.text(bar.get_x() + bar.get_width() / 2, value + 4, f"{value:.1f}", ha="center")
    ax_bar.set_ylabel("Path length [mm]")
    ax_bar.set_title(f"Total {manifest.semantic_local_total_length_mm:.1f} mm; B-end margin {manifest.successor_interior_path_margin_mm:.0f} mm")
    ax_bar.grid(axis="y", alpha=0.25)

    fig.suptitle(
        "Dijkstra cost unchanged; only the planner-approved edge's physical reference is optimized",
        fontsize=14,
    )
    output = root / "t7_to_t3_semantic_local_selected_bridge_20260902"
    fig.savefig(output.with_suffix(".png"), dpi=180)
    fig.savefig(output.with_suffix(".svg"))
    print(f"SEMANTIC_LOCAL_BRIDGE_RENDERED output={output}")


if __name__ == "__main__":
    main()
