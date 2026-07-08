"""Pure safety guard state machine for VR teleoperation."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional, Sequence

from .pose_utils import normalize_pose6
from .projection_utils import (
    WorkspaceBox,
    apply_ramp_limit,
    clamp_pose_to_workspace,
    is_inside_workspace,
    project_to_workspace_boundary,
)


@dataclass
class SafetyInput:
    raw_pose: Optional[Sequence[float]]
    tracking_ok: bool
    deadman_pressed: bool
    stamp_sec: Optional[float] = None


@dataclass
class SafetyOutput:
    target_pose: list[float]
    projected_pose: list[float]
    raw_pose: Optional[list[float]]
    hold: bool
    inside_workspace: bool
    projected: bool
    reason: str


@dataclass
class SafetyGuardCore:
    workspace: WorkspaceBox
    linear_ramp_mm_per_tick: float = 20.0
    angular_ramp_deg_per_tick: float = 3.0
    vr_timeout_sec: float = 0.3
    require_deadman: bool = True
    hold_on_tracking_lost: bool = True
    hold_on_deadman_release: bool = True
    outside_workspace_policy: str = "project_to_boundary"
    projection_method: str = "segment_boundary"
    last_safe_pose: Optional[list[float]] = None
    last_input_stamp_sec: Optional[float] = None
    last_output: Optional[SafetyOutput] = field(default=None, init=False)

    def reset(self, pose: Sequence[float]) -> None:
        self.last_safe_pose = clamp_pose_to_workspace(normalize_pose6(pose), self.workspace)
        self.last_output = None
        self.last_input_stamp_sec = None

    def update(self, sample: SafetyInput, now_sec: Optional[float] = None) -> SafetyOutput:
        now = time.monotonic() if now_sec is None else float(now_sec)
        if sample.stamp_sec is not None:
            self.last_input_stamp_sec = float(sample.stamp_sec)

        if self.last_safe_pose is None:
            seed = sample.raw_pose if sample.raw_pose is not None else [
                self.workspace.x_min_mm,
                self.workspace.y_min_mm,
                self.workspace.z_min_mm,
                0.0,
                0.0,
                0.0,
            ]
            self.last_safe_pose = clamp_pose_to_workspace(seed, self.workspace)

        hold_reason = self._hold_reason(sample, now)
        if hold_reason is not None:
            output = SafetyOutput(
                target_pose=self.last_safe_pose[:],
                projected_pose=self.last_safe_pose[:],
                raw_pose=normalize_pose6(sample.raw_pose) if sample.raw_pose is not None else None,
                hold=True,
                inside_workspace=True,
                projected=False,
                reason=hold_reason,
            )
            self.last_output = output
            return output

        if sample.raw_pose is None:
            output = SafetyOutput(
                target_pose=self.last_safe_pose[:],
                projected_pose=self.last_safe_pose[:],
                raw_pose=None,
                hold=True,
                inside_workspace=True,
                projected=False,
                reason="no_raw_pose",
            )
            self.last_output = output
            return output

        raw_pose = normalize_pose6(sample.raw_pose)
        inside = is_inside_workspace(raw_pose, self.workspace)
        projected = False
        if inside:
            projected_pose = raw_pose[:]
        elif self.outside_workspace_policy == "project_to_boundary":
            if self.projection_method == "segment_boundary":
                projected_pose = project_to_workspace_boundary(self.last_safe_pose, raw_pose, self.workspace)
            else:
                projected_pose = clamp_pose_to_workspace(raw_pose, self.workspace)
            projected = True
        else:
            projected_pose = self.last_safe_pose[:]
            projected = True

        target_pose = apply_ramp_limit(
            self.last_safe_pose,
            projected_pose,
            self.linear_ramp_mm_per_tick,
            self.angular_ramp_deg_per_tick,
        )
        self.last_safe_pose = target_pose[:]
        output = SafetyOutput(
            target_pose=target_pose,
            projected_pose=projected_pose,
            raw_pose=raw_pose,
            hold=False,
            inside_workspace=inside,
            projected=projected,
            reason="ok_projected" if projected else "ok",
        )
        self.last_output = output
        return output

    def _hold_reason(self, sample: SafetyInput, now_sec: float) -> Optional[str]:
        if self.hold_on_tracking_lost and not sample.tracking_ok:
            return "tracking_lost"
        if self.require_deadman and self.hold_on_deadman_release and not sample.deadman_pressed:
            return "deadman_released"
        if sample.stamp_sec is not None and now_sec - float(sample.stamp_sec) > self.vr_timeout_sec:
            return "vr_timeout"
        if self.last_input_stamp_sec is not None and now_sec - self.last_input_stamp_sec > self.vr_timeout_sec:
            return "vr_timeout"
        return None
