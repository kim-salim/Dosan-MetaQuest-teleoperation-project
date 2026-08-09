"""Continuous quaternion helpers for Doosan Euler ZYZ task poses.

Doosan ``posx[3:6]`` and ``servol_rt`` use intrinsic Euler Z-Y'-Z'' angles,
not XYZ roll/pitch/yaw.  This module keeps physical orientation operations in
quaternions and only selects a ZYZ representation at the robot interface.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional


_EPSILON = 1.0e-12
_SINGULAR_SIN_EPSILON = 1.0e-8


def _finite_vector(values: Iterable[float], length: int, name: str) -> list[float]:
    output = [float(value) for value in values]
    if len(output) != length:
        raise ValueError(f"{name} must contain exactly {length} values")
    if any(not math.isfinite(value) for value in output):
        raise ValueError(f"{name} contains non-finite values: {output}")
    return output


def shortest_angle_delta_deg(value: float, reference: float) -> float:
    """Return the signed shortest ``reference -> value`` angular delta."""
    return (float(value) - float(reference) + 180.0) % 360.0 - 180.0


def angle_near_reference_deg(value: float, reference: float) -> float:
    return float(reference) + shortest_angle_delta_deg(value, reference)


def quaternion_normalized(values: Iterable[float]) -> list[float]:
    quaternion = _finite_vector(values, 4, "quaternion")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= _EPSILON:
        raise ValueError(f"quaternion has near-zero norm: {quaternion}")
    return [value / norm for value in quaternion]


def quaternion_multiply(a: Iterable[float], b: Iterable[float]) -> list[float]:
    ax, ay, az, aw = quaternion_normalized(a)
    bx, by, bz, bw = quaternion_normalized(b)
    return quaternion_normalized(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )


def quaternion_angle_deg(a: Iterable[float], b: Iterable[float]) -> float:
    qa = quaternion_normalized(a)
    qb = quaternion_normalized(b)
    dot = abs(sum(qa[index] * qb[index] for index in range(4)))
    return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))


def quaternion_from_axis_angle_deg(
    axis_xyz: Iterable[float],
    angle_deg: float,
) -> list[float]:
    axis = _finite_vector(axis_xyz, 3, "axis_xyz")
    norm = math.sqrt(sum(value * value for value in axis))
    if norm <= _EPSILON:
        raise ValueError(f"axis_xyz has near-zero norm: {axis}")
    half_angle = math.radians(float(angle_deg)) * 0.5
    scale = math.sin(half_angle) / norm
    return quaternion_normalized(
        [axis[0] * scale, axis[1] * scale, axis[2] * scale, math.cos(half_angle)]
    )


def quaternion_slerp(
    start_xyzw: Iterable[float],
    end_xyzw: Iterable[float],
    fraction: float,
) -> list[float]:
    start = quaternion_normalized(start_xyzw)
    end = quaternion_normalized(end_xyzw)
    amount = min(1.0, max(0.0, float(fraction)))
    dot = sum(start[index] * end[index] for index in range(4))
    if dot < 0.0:
        end = [-value for value in end]
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return quaternion_normalized(
            [start[index] + amount * (end[index] - start[index]) for index in range(4)]
        )
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    start_scale = math.sin((1.0 - amount) * theta) / sin_theta
    end_scale = math.sin(amount * theta) / sin_theta
    return quaternion_normalized(
        [
            start_scale * start[index] + end_scale * end[index]
            for index in range(4)
        ]
    )


def doosan_zyz_deg_to_quaternion(zyz_deg: Iterable[float]) -> list[float]:
    """Convert Doosan intrinsic Z-Y'-Z'' Euler angles to ``[x,y,z,w]``."""
    alpha_deg, beta_deg, gamma_deg = _finite_vector(
        zyz_deg, 3, "doosan_zyz_deg"
    )
    q_alpha = quaternion_from_axis_angle_deg([0.0, 0.0, 1.0], alpha_deg)
    q_beta = quaternion_from_axis_angle_deg([0.0, 1.0, 0.0], beta_deg)
    q_gamma = quaternion_from_axis_angle_deg([0.0, 0.0, 1.0], gamma_deg)
    return quaternion_multiply(quaternion_multiply(q_alpha, q_beta), q_gamma)


def _quaternion_to_matrix(quaternion_xyzw: Iterable[float]) -> list[list[float]]:
    x, y, z, w = quaternion_normalized(quaternion_xyzw)
    return [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]


def quaternion_to_doosan_zyz_deg(
    quaternion_xyzw: Iterable[float],
    reference_zyz_deg: Optional[Iterable[float]] = None,
) -> list[float]:
    """Convert a quaternion to a continuous Doosan ZYZ representation.

    When ``reference_zyz_deg`` is supplied, equivalent positive/negative-beta
    branches and 360-degree wraps are evaluated and the numerically nearest
    representation is returned.  At beta=0/180, the unobservable split between
    alpha and gamma is chosen closest to the reference.
    """
    matrix = _quaternion_to_matrix(quaternion_xyzw)
    beta = math.acos(min(1.0, max(-1.0, matrix[2][2])))
    sin_beta = math.sin(beta)
    beta_deg = math.degrees(beta)

    reference = (
        None
        if reference_zyz_deg is None
        else _finite_vector(reference_zyz_deg, 3, "reference_zyz_deg")
    )
    if abs(sin_beta) <= _SINGULAR_SIN_EPSILON:
        if beta_deg < 90.0:
            combined = math.degrees(math.atan2(matrix[1][0], matrix[0][0]))
            if reference is None:
                return [combined, 0.0, 0.0]
            error = shortest_angle_delta_deg(
                combined,
                reference[0] + reference[2],
            )
            return [
                reference[0] + 0.5 * error,
                angle_near_reference_deg(0.0, reference[1]),
                reference[2] + 0.5 * error,
            ]

        difference = math.degrees(
            math.atan2(-matrix[1][0], -matrix[0][0])
        )
        if reference is None:
            return [difference, 180.0, 0.0]
        error = shortest_angle_delta_deg(
            difference,
            reference[0] - reference[2],
        )
        return [
            reference[0] + 0.5 * error,
            angle_near_reference_deg(180.0, reference[1]),
            reference[2] - 0.5 * error,
        ]

    alpha_deg = math.degrees(math.atan2(matrix[1][2], matrix[0][2]))
    gamma_deg = math.degrees(math.atan2(matrix[2][1], -matrix[2][0]))
    canonical = [alpha_deg, beta_deg, gamma_deg]
    if reference is None:
        return canonical

    families = (
        canonical,
        [alpha_deg + 180.0, -beta_deg, gamma_deg + 180.0],
    )
    candidates = [
        [
            angle_near_reference_deg(family[index], reference[index])
            for index in range(3)
        ]
        for family in families
    ]
    return min(
        candidates,
        key=lambda candidate: sum(
            (candidate[index] - reference[index]) ** 2 for index in range(3)
        ),
    )


def doosan_zyz_b_delta_quaternion(
    anchor_zyz_deg: Iterable[float],
    delta_deg: float,
) -> list[float]:
    """Return the physical delta quaternion for the anchor's ZYZ B axis.

    The axis is ``Rz(anchor_A) * +Y`` in base coordinates.  Applying this
    quaternion on the left exactly preserves the legacy behavior of changing
    only Doosan B, while avoiding numerical Euler composition.
    """
    alpha_deg, _, _ = _finite_vector(anchor_zyz_deg, 3, "anchor_zyz_deg")
    alpha = math.radians(alpha_deg)
    return quaternion_from_axis_angle_deg(
        [-math.sin(alpha), math.cos(alpha), 0.0],
        float(delta_deg),
    )


def apply_doosan_zyz_b_delta_deg(
    anchor_zyz_deg: Iterable[float],
    delta_deg: float,
    reference_zyz_deg: Optional[Iterable[float]] = None,
) -> list[float]:
    anchor = _finite_vector(anchor_zyz_deg, 3, "anchor_zyz_deg")
    delta_quaternion = doosan_zyz_b_delta_quaternion(anchor, delta_deg)
    target_quaternion = quaternion_multiply(
        delta_quaternion,
        doosan_zyz_deg_to_quaternion(anchor),
    )
    return quaternion_to_doosan_zyz_deg(
        target_quaternion,
        anchor if reference_zyz_deg is None else reference_zyz_deg,
    )


def limit_doosan_zyz_geodesic_deg(
    anchor_zyz_deg: Iterable[float],
    target_zyz_deg: Iterable[float],
    max_delta_deg: float,
    reference_zyz_deg: Optional[Iterable[float]] = None,
) -> tuple[list[float], float, bool]:
    """Clamp a target to a physical angular envelope around an anchor."""
    if not math.isfinite(max_delta_deg) or max_delta_deg < 0.0:
        raise ValueError("max_delta_deg must be finite and non-negative")
    anchor = _finite_vector(anchor_zyz_deg, 3, "anchor_zyz_deg")
    target = _finite_vector(target_zyz_deg, 3, "target_zyz_deg")
    anchor_quaternion = doosan_zyz_deg_to_quaternion(anchor)
    target_quaternion = doosan_zyz_deg_to_quaternion(target)
    angle_deg = quaternion_angle_deg(anchor_quaternion, target_quaternion)
    if angle_deg <= max_delta_deg + 1.0e-9:
        return target, angle_deg, False
    limited_quaternion = quaternion_slerp(
        anchor_quaternion,
        target_quaternion,
        0.0 if angle_deg <= _EPSILON else max_delta_deg / angle_deg,
    )
    reference = anchor if reference_zyz_deg is None else reference_zyz_deg
    return (
        quaternion_to_doosan_zyz_deg(limited_quaternion, reference),
        angle_deg,
        True,
    )


def step_doosan_zyz_toward_deg(
    current_zyz_deg: Iterable[float],
    target_zyz_deg: Iterable[float],
    max_step_deg: float,
) -> tuple[list[float], float, bool]:
    """Move at most ``max_step_deg`` along the shortest physical rotation."""
    if not math.isfinite(max_step_deg) or max_step_deg < 0.0:
        raise ValueError("max_step_deg must be finite and non-negative")
    current = _finite_vector(current_zyz_deg, 3, "current_zyz_deg")
    target = _finite_vector(target_zyz_deg, 3, "target_zyz_deg")
    current_quaternion = doosan_zyz_deg_to_quaternion(current)
    target_quaternion = doosan_zyz_deg_to_quaternion(target)
    angle_deg = quaternion_angle_deg(current_quaternion, target_quaternion)
    if angle_deg <= max_step_deg + 1.0e-9:
        return target, angle_deg, False
    next_quaternion = quaternion_slerp(
        current_quaternion,
        target_quaternion,
        0.0 if angle_deg <= _EPSILON else max_step_deg / angle_deg,
    )
    return (
        quaternion_to_doosan_zyz_deg(next_quaternion, current),
        angle_deg,
        True,
    )
