"""Workspace projection and ramp limiting for Doosan task poses."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence

from .pose_utils import POSE_SIZE, normalize_pose6


EPS = 1e-9


@dataclass(frozen=True)
class WorkspaceBox:
    x_min_mm: float
    x_max_mm: float
    y_min_mm: float
    y_max_mm: float
    z_min_mm: float
    z_max_mm: float
    rx_min_deg: float = -180.0
    rx_max_deg: float = 180.0
    ry_min_deg: float = -180.0
    ry_max_deg: float = 180.0
    rz_min_deg: float = -180.0
    rz_max_deg: float = 180.0

    @classmethod
    def from_mapping(cls, data: Mapping[str, float]) -> "WorkspaceBox":
        return cls(
            x_min_mm=float(data["x_min_mm"]),
            x_max_mm=float(data["x_max_mm"]),
            y_min_mm=float(data["y_min_mm"]),
            y_max_mm=float(data["y_max_mm"]),
            z_min_mm=float(data["z_min_mm"]),
            z_max_mm=float(data["z_max_mm"]),
            rx_min_deg=float(data.get("rx_min_deg", -180.0)),
            rx_max_deg=float(data.get("rx_max_deg", 180.0)),
            ry_min_deg=float(data.get("ry_min_deg", -180.0)),
            ry_max_deg=float(data.get("ry_max_deg", 180.0)),
            rz_min_deg=float(data.get("rz_min_deg", -180.0)),
            rz_max_deg=float(data.get("rz_max_deg", 180.0)),
        ).validated()

    def validated(self) -> "WorkspaceBox":
        for lo, hi, name in zip(self.lower, self.upper, ("x", "y", "z", "rx", "ry", "rz")):
            if not math.isfinite(lo) or not math.isfinite(hi):
                raise ValueError(f"workspace {name} bounds must be finite")
            if lo > hi:
                raise ValueError(f"workspace {name} min must be <= max")
        return self

    @property
    def lower(self) -> tuple[float, float, float, float, float, float]:
        return (
            self.x_min_mm,
            self.y_min_mm,
            self.z_min_mm,
            self.rx_min_deg,
            self.ry_min_deg,
            self.rz_min_deg,
        )

    @property
    def upper(self) -> tuple[float, float, float, float, float, float]:
        return (
            self.x_max_mm,
            self.y_max_mm,
            self.z_max_mm,
            self.rx_max_deg,
            self.ry_max_deg,
            self.rz_max_deg,
        )


DEFAULT_WORKSPACE = WorkspaceBox(
    x_min_mm=250.0,
    x_max_mm=750.0,
    y_min_mm=-350.0,
    y_max_mm=350.0,
    z_min_mm=120.0,
    z_max_mm=600.0,
)


def is_inside_workspace(pose: Sequence[float], workspace: WorkspaceBox | Mapping[str, float]) -> bool:
    pose = normalize_pose6(pose)
    box = _workspace_box(workspace)
    return all(box.lower[idx] - EPS <= pose[idx] <= box.upper[idx] + EPS for idx in range(POSE_SIZE))


def clamp_pose_to_workspace(
    pose: Sequence[float], workspace: WorkspaceBox | Mapping[str, float]
) -> list[float]:
    pose = normalize_pose6(pose)
    box = _workspace_box(workspace)
    return [
        min(max(pose[idx], box.lower[idx]), box.upper[idx])
        for idx in range(POSE_SIZE)
    ]


def project_to_workspace_boundary(
    last_safe_pose: Sequence[float],
    raw_pose: Sequence[float],
    workspace: WorkspaceBox | Mapping[str, float],
) -> list[float]:
    """Project raw_pose to the last reachable point on the segment from last_safe_pose.

    last_safe_pose is expected to be inside the workspace. If it is not, it is
    first clamped into the box to recover a valid segment origin.
    """

    start = normalize_pose6(last_safe_pose)
    raw = normalize_pose6(raw_pose)
    box = _workspace_box(workspace)

    if is_inside_workspace(raw, box):
        return raw[:]

    if not is_inside_workspace(start, box):
        start = clamp_pose_to_workspace(start, box)

    t_max = 1.0
    for idx in range(POSE_SIZE):
        delta = raw[idx] - start[idx]
        if abs(delta) <= EPS:
            continue
        if delta > 0.0:
            t_axis = (box.upper[idx] - start[idx]) / delta
        else:
            t_axis = (box.lower[idx] - start[idx]) / delta
        if t_axis >= -EPS:
            t_max = min(t_max, max(0.0, min(1.0, t_axis)))

    projected = [start[idx] + t_max * (raw[idx] - start[idx]) for idx in range(POSE_SIZE)]
    return clamp_pose_to_workspace(projected, box)


def limit_linear_step(
    last_pose: Sequence[float],
    target_pose: Sequence[float],
    max_step_mm: float,
) -> list[float]:
    return _limit_axis_group(last_pose, target_pose, max_step_mm, range(3))


def limit_angular_step(
    last_pose: Sequence[float],
    target_pose: Sequence[float],
    max_step_deg: float,
) -> list[float]:
    return _limit_axis_group(last_pose, target_pose, max_step_deg, range(3, 6))


def apply_ramp_limit(
    last_pose: Sequence[float],
    target_pose: Sequence[float],
    linear_step: float,
    angular_step: float,
) -> list[float]:
    pose = limit_linear_step(last_pose, target_pose, linear_step)
    return limit_angular_step(pose, target_pose, angular_step)


def _limit_axis_group(
    last_pose: Sequence[float],
    target_pose: Sequence[float],
    max_step: float,
    axes: range,
) -> list[float]:
    last = normalize_pose6(last_pose)
    target = normalize_pose6(target_pose)
    if max_step < 0.0 or not math.isfinite(max_step):
        raise ValueError("max_step must be finite and non-negative")
    output = last[:]
    for idx in axes:
        delta = target[idx] - last[idx]
        if abs(delta) <= max_step:
            output[idx] = target[idx]
        elif max_step == 0.0:
            output[idx] = last[idx]
        else:
            output[idx] = last[idx] + math.copysign(max_step, delta)
    return output


def _workspace_box(workspace: WorkspaceBox | Mapping[str, float]) -> WorkspaceBox:
    if isinstance(workspace, WorkspaceBox):
        return workspace
    return WorkspaceBox.from_mapping(workspace)
