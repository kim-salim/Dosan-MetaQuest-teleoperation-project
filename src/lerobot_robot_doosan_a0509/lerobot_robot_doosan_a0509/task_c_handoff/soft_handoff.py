"""Lightweight Bridge/ACT-B soft crossfade primitives."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from quest_a0509_teleop.doosan_orientation import (
    doosan_zyz_deg_to_quaternion,
    quaternion_slerp,
    quaternion_to_doosan_zyz_deg,
)


def quintic_smoothstep(value: float) -> float:
    """C2-continuous smoothstep with zero endpoint velocity/acceleration."""

    s = min(1.0, max(0.0, float(value)))
    return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5


@dataclass(frozen=True)
class SoftHandoffPlan:
    actions: np.ndarray
    weights: np.ndarray
    b_actions_consumed: int
    bridge_actions_consumed: int

    def __post_init__(self) -> None:
        actions = np.asarray(self.actions, dtype=np.float64)
        weights = np.asarray(self.weights, dtype=np.float64)
        if actions.ndim != 2 or actions.shape[1] != 7 or len(actions) < 1:
            raise ValueError("soft handoff actions must have shape [N, 7]")
        if weights.shape != (len(actions),):
            raise ValueError("soft handoff weights must align with actions")
        if not np.all(np.isfinite(actions)) or not np.all(np.isfinite(weights)):
            raise ValueError("soft handoff contains non-finite values")
        if np.any(weights < 0.0) or np.any(weights > 1.0):
            raise ValueError("soft handoff weights must be in [0, 1]")
        object.__setattr__(self, "actions", actions.copy())
        object.__setattr__(self, "weights", weights.copy())


def build_soft_handoff(
    bridge_actions: np.ndarray,
    b_actions: np.ndarray,
    *,
    steps: int,
    held_gripper_target: float,
) -> SoftHandoffPlan:
    """Precompute a quintic XYZ/SLERP crossfade without blending gripper."""

    bridge = np.asarray(bridge_actions, dtype=np.float64)
    successor = np.asarray(b_actions, dtype=np.float64)
    if bridge.ndim != 2 or successor.ndim != 2:
        raise ValueError("crossfade inputs must have shape [steps, action_dim]")
    if bridge.shape[1] != 7 or successor.shape[1] != 7:
        raise ValueError("crossfade inputs must contain 7D A0509 actions")
    if not np.all(np.isfinite(bridge)) or not np.all(np.isfinite(successor)):
        raise ValueError("crossfade inputs must be finite")
    if not isinstance(steps, int) or isinstance(steps, bool) or steps < 1:
        raise ValueError("crossfade steps must be a positive integer")
    if len(bridge) < steps or len(successor) < steps:
        raise ValueError("crossfade inputs are shorter than requested steps")
    if held_gripper_target not in {0.0, 1.0}:
        raise ValueError("held_gripper_target must be discrete 0.0 or 1.0")

    # Include B=100% at the final crossfade command.  The command immediately
    # before this plan is the implicit s=0 Bridge endpoint for the blend.
    fractions = np.arange(1, steps + 1, dtype=np.float64) / float(steps)
    weights = np.asarray([quintic_smoothstep(value) for value in fractions])
    output = bridge[:steps].copy()
    output[:, :3] = (
        (1.0 - weights[:, None]) * bridge[:steps, :3]
        + weights[:, None] * successor[:steps, :3]
    )

    reference = bridge[0, 3:6].tolist()
    for index, weight in enumerate(weights):
        quaternion = quaternion_slerp(
            doosan_zyz_deg_to_quaternion(bridge[index, 3:6]),
            doosan_zyz_deg_to_quaternion(successor[index, 3:6]),
            float(weight),
        )
        reference = quaternion_to_doosan_zyz_deg(quaternion, reference)
        output[index, 3:6] = reference
    output[:, 6] = held_gripper_target
    return SoftHandoffPlan(
        actions=output,
        weights=weights,
        b_actions_consumed=steps,
        bridge_actions_consumed=steps,
    )
