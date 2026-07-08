"""Helpers for Doosan task poses.

Internal pose convention:
    [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]

Frame:
    robot base

Do not mix these values with ROS geometry_msgs/PoseStamped without explicit
meter/quaternion conversion.
"""

from __future__ import annotations

import json
import math
from typing import Iterable, List, Mapping, Sequence


POSE_SIZE = 6
POSE_KEYS = ("x", "y", "z", "rx", "ry", "rz")


def normalize_pose6(values: Iterable[float]) -> List[float]:
    pose = [float(value) for value in values]
    if len(pose) != POSE_SIZE:
        raise ValueError(f"pose must contain {POSE_SIZE} values, got {len(pose)}")
    if any(not math.isfinite(value) for value in pose):
        raise ValueError(f"pose contains non-finite values: {pose}")
    return pose


def pose6_from_json_dict(data: Mapping[str, object], key: str = "pose") -> List[float]:
    value = data.get(key)
    if value is None:
        value = data.get("target_pose")
    if value is None:
        value = data.get("tcp_pose")
    if value is None:
        raise ValueError(f"JSON object does not contain '{key}', 'target_pose', or 'tcp_pose'")
    if isinstance(value, Mapping):
        return normalize_pose6(value[name] for name in POSE_KEYS)
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return normalize_pose6(value)
    raise ValueError(f"pose value must be a list or object, got {type(value).__name__}")


def parse_controller_state_json(text: str) -> dict:
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid controller JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("controller JSON must be an object")
    return data


def bool_from_state(data: Mapping[str, object], key: str, default: bool = False) -> bool:
    value = data.get(key, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return default


def seq_from_pose6(pose: Sequence[float]) -> List[float]:
    return normalize_pose6(pose)


def pose6_to_dict(pose: Sequence[float]) -> dict:
    pose = normalize_pose6(pose)
    return {key: pose[idx] for idx, key in enumerate(POSE_KEYS)}


def pose6_to_json(pose: Sequence[float]) -> str:
    return json.dumps(pose6_to_dict(pose), sort_keys=True)


def add_pose6(a: Sequence[float], b: Sequence[float]) -> List[float]:
    a = normalize_pose6(a)
    b = normalize_pose6(b)
    return [a[idx] + b[idx] for idx in range(POSE_SIZE)]


def sub_pose6(a: Sequence[float], b: Sequence[float]) -> List[float]:
    a = normalize_pose6(a)
    b = normalize_pose6(b)
    return [a[idx] - b[idx] for idx in range(POSE_SIZE)]
