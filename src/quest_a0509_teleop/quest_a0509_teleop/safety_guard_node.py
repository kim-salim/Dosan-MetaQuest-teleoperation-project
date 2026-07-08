"""Clamp and ramp-limit Doosan posx targets."""

from __future__ import annotations

import json
import math
import time
from typing import Iterable, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


def _vector3(values: Iterable[float], name: str) -> list[float]:
    output = [float(value) for value in values]
    if len(output) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    if any(not math.isfinite(value) for value in output):
        raise ValueError(f"{name} contains non-finite values: {output}")
    return output


def _posx(values: Iterable[float], name: str) -> list[float]:
    output = [float(value) for value in values]
    if len(output) != 6:
        raise ValueError(f"{name} must contain exactly 6 values")
    if any(not math.isfinite(value) for value in output):
        raise ValueError(f"{name} contains non-finite values: {output}")
    return output


def _shortest_angle_delta_deg(target: float, current: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


class SafetyGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_guard_node")
        self._declare_parameters()

        self.target_posx_topic = self.get_parameter("target_posx_topic").value
        self.safe_posx_topic = self.get_parameter("safe_posx_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.workspace_min_xyz_mm = _vector3(
            self.get_parameter("workspace_min_xyz_mm").value,
            "workspace_min_xyz_mm",
        )
        self.workspace_max_xyz_mm = _vector3(
            self.get_parameter("workspace_max_xyz_mm").value,
            "workspace_max_xyz_mm",
        )
        self.robot_anchor_posx_topic = self.get_parameter("robot_anchor_posx_topic").value
        self.max_step_xyz_mm = _vector3(
            self.get_parameter("max_step_xyz_mm").value,
            "max_step_xyz_mm",
        )
        self.enable_orientation_limits = bool(
            self.get_parameter("enable_orientation_limits").value
        )
        self.max_orientation_delta_deg = _vector3(
            self.get_parameter("max_orientation_delta_deg").value,
            "max_orientation_delta_deg",
        )
        self.max_step_rpy_deg = _vector3(
            self.get_parameter("max_step_rpy_deg").value,
            "max_step_rpy_deg",
        )
        for index in range(3):
            if self.workspace_min_xyz_mm[index] > self.workspace_max_xyz_mm[index]:
                raise ValueError("workspace_min_xyz_mm must be <= workspace_max_xyz_mm")
            if self.max_step_xyz_mm[index] < 0.0:
                raise ValueError("max_step_xyz_mm values must be non-negative")
            if self.max_orientation_delta_deg[index] < 0.0:
                raise ValueError("max_orientation_delta_deg values must be non-negative")
            if self.max_step_rpy_deg[index] < 0.0:
                raise ValueError("max_step_rpy_deg values must be non-negative")

        self.latest_target: Optional[list[float]] = None
        self.last_safe: Optional[list[float]] = None
        self.orientation_anchor_rpy_deg: Optional[list[float]] = None
        self.last_event_text = ""
        self.last_safe_log_time = 0.0

        self.safe_pub = self.create_publisher(Float64MultiArray, self.safe_posx_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.target_sub = self.create_subscription(
            Float64MultiArray,
            self.target_posx_topic,
            self._on_target,
            10,
        )
        self.robot_anchor_sub = self.create_subscription(
            Float64MultiArray,
            self.robot_anchor_posx_topic,
            self._on_robot_anchor_posx,
            10,
        )
        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self._tick)
        self._publish_status(
            "safety_guard_node started: target="
            f"{self.target_posx_topic}, safe={self.safe_posx_topic}, "
            f"workspace_min_xyz_mm={self.workspace_min_xyz_mm}, "
            f"workspace_max_xyz_mm={self.workspace_max_xyz_mm}, "
            f"robot_anchor_posx_topic={self.robot_anchor_posx_topic}, "
            f"max_step_xyz_mm={self.max_step_xyz_mm}, "
            f"enable_orientation_limits={self.enable_orientation_limits}, "
            f"max_orientation_delta_deg={self.max_orientation_delta_deg}, "
            f"max_step_rpy_deg={self.max_step_rpy_deg}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("target_posx_topic", "/vr/target_posx")
        self.declare_parameter("safe_posx_topic", "/vr/safe_posx")
        self.declare_parameter("status_topic", "/vr/status")
        self.declare_parameter("robot_anchor_posx_topic", "/vr/robot_anchor_posx")
        self.declare_parameter("workspace_min_xyz_mm", [250.0, -350.0, 150.0])
        self.declare_parameter("workspace_max_xyz_mm", [650.0, 350.0, 600.0])
        self.declare_parameter("max_step_xyz_mm", [20.0, 20.0, 20.0])
        self.declare_parameter("enable_orientation_limits", True)
        self.declare_parameter("max_orientation_delta_deg", [15.0, 15.0, 20.0])
        self.declare_parameter("max_step_rpy_deg", [2.0, 2.0, 2.0])
        self.declare_parameter("publish_rate_hz", 30.0)

    def _on_target(self, msg: Float64MultiArray) -> None:
        try:
            self.latest_target = _posx(msg.data, "target_posx")
        except ValueError as exc:
            self._publish_status(f"ignored invalid target_posx: {exc}", warn=True)

    def _on_robot_anchor_posx(self, msg: Float64MultiArray) -> None:
        try:
            posx = _posx(msg.data, "robot_anchor_posx")
        except ValueError as exc:
            self._publish_status(f"ignored invalid robot anchor posx: {exc}", warn=True)
            return
        self.orientation_anchor_rpy_deg = posx[3:6]
        self._publish_status(
            "orientation safety anchor updated: "
            + json.dumps({"rpy_deg": self.orientation_anchor_rpy_deg}, sort_keys=True)
        )

    def _tick(self) -> None:
        if self.latest_target is None:
            return

        raw = self.latest_target[:]
        clamped = raw[:]
        clamped_axes = []
        for index, axis in enumerate(("x", "y", "z")):
            before = clamped[index]
            clamped[index] = min(
                max(before, self.workspace_min_xyz_mm[index]),
                self.workspace_max_xyz_mm[index],
            )
            if clamped[index] != before:
                clamped_axes.append(axis)
        if self.enable_orientation_limits:
            if self.orientation_anchor_rpy_deg is None:
                self.orientation_anchor_rpy_deg = raw[3:6]
                self._publish_status(
                    "orientation safety anchor initialized from first target: "
                    + json.dumps({"rpy_deg": self.orientation_anchor_rpy_deg}, sort_keys=True)
                )
            for offset, axis in enumerate(("rx", "ry", "rz")):
                index = offset + 3
                anchor = self.orientation_anchor_rpy_deg[offset]
                delta = _shortest_angle_delta_deg(raw[index], anchor)
                max_delta = self.max_orientation_delta_deg[offset]
                limited_delta = min(max(delta, -max_delta), max_delta)
                clamped[index] = anchor + limited_delta
                if abs(limited_delta - delta) > 1.0e-9:
                    clamped_axes.append(axis)

        if self.last_safe is None:
            safe = clamped[:]
            ramp_axes = []
            self._publish_status("safe_posx initialized: " + json.dumps({"data": safe}, sort_keys=True))
        else:
            safe = clamped[:]
            ramp_axes = []
            for index, axis in enumerate(("x", "y", "z")):
                delta = clamped[index] - self.last_safe[index]
                max_step = self.max_step_xyz_mm[index]
                if abs(delta) > max_step:
                    safe[index] = self.last_safe[index] + math.copysign(max_step, delta)
                    ramp_axes.append(axis)
            for offset, axis in enumerate(("rx", "ry", "rz")):
                index = offset + 3
                delta = _shortest_angle_delta_deg(clamped[index], self.last_safe[index])
                max_step = self.max_step_rpy_deg[offset]
                if abs(delta) > max_step:
                    safe[index] = self.last_safe[index] + math.copysign(max_step, delta)
                    ramp_axes.append(axis)
                else:
                    safe[index] = self.last_safe[index] + delta

        self.last_safe = safe[:]
        msg = Float64MultiArray()
        msg.data = safe
        self.safe_pub.publish(msg)

        event = {
            "raw": raw,
            "safe": safe,
            "clamped_axes": clamped_axes,
            "ramp_limited_axes": ramp_axes,
        }
        if clamped_axes or ramp_axes:
            self._publish_event("safety event: " + json.dumps(event, sort_keys=True), warn=bool(clamped_axes))
        else:
            now = time.monotonic()
            if now - self.last_safe_log_time >= 1.0:
                self._publish_status("safe_posx=" + json.dumps({"data": safe}, sort_keys=True))
                self.last_safe_log_time = now

    def _publish_event(self, text: str, warn: bool = False) -> None:
        now = time.monotonic()
        if text != self.last_event_text or now - self.last_safe_log_time >= 1.0:
            self._publish_status(text, warn=warn)
            self.last_event_text = text
            self.last_safe_log_time = now

    def _publish_status(self, text: str, warn: bool = False) -> None:
        self.status_pub.publish(String(data=text))
        if warn:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafetyGuardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
