"""Clamp and ramp-limit Doosan posx targets."""

from __future__ import annotations

import json
import math
import time
from typing import Iterable, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from quest_a0509_teleop.doosan_orientation import (
    limit_doosan_zyz_geodesic_deg,
)

from std_msgs.msg import Float64MultiArray, String


def _vector3(values: Iterable[float], name: str) -> list[float]:
    output = [float(value) for value in values]
    if len(output) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    if any(not math.isfinite(value) for value in output):
        raise ValueError(f"{name} contains non-finite values: {output}")
    return output


def _bool_vector3(values: Iterable[bool], name: str) -> list[bool]:
    output = list(values)
    if len(output) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    if any(type(value) is not bool for value in output):
        raise ValueError(f"{name} must contain only booleans: {output}")
    return output


def clamp_workspace_axis(
    value: float, minimum: float, maximum: float, minimum_enabled: bool
) -> float:
    output = float(value)
    if minimum_enabled:
        output = max(output, float(minimum))
    return min(output, float(maximum))


def _posx(values: Iterable[float], name: str) -> list[float]:
    output = [float(value) for value in values]
    if len(output) != 6:
        raise ValueError(f"{name} must contain exactly 6 values")
    if any(not math.isfinite(value) for value in output):
        raise ValueError(f"{name} contains non-finite values: {output}")
    return output


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
        self.workspace_min_limit_enabled = _bool_vector3(
            self.get_parameter("workspace_min_limit_enabled").value,
            "workspace_min_limit_enabled",
        )
        self.workspace_max_xyz_mm = _vector3(
            self.get_parameter("workspace_max_xyz_mm").value,
            "workspace_max_xyz_mm",
        )
        self.robot_anchor_posx_topic = self.get_parameter("robot_anchor_posx_topic").value
        self.enable_orientation_limits = bool(
            self.get_parameter("enable_orientation_limits").value
        )
        self.max_orientation_delta_deg = _vector3(
            self.get_parameter("max_orientation_delta_deg").value,
            "max_orientation_delta_deg",
        )
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0.0")
        for index in range(3):
            if (
                self.workspace_min_limit_enabled[index]
                and self.workspace_min_xyz_mm[index] > self.workspace_max_xyz_mm[index]
            ):
                raise ValueError("workspace_min_xyz_mm must be <= workspace_max_xyz_mm")
            if self.max_orientation_delta_deg[index] < 0.0:
                raise ValueError("max_orientation_delta_deg values must be non-negative")

        self.latest_target: Optional[list[float]] = None
        self.orientation_anchor_rpy_deg: Optional[list[float]] = None
        self.last_safe_orientation_zyz_deg: Optional[list[float]] = None
        self.last_event_text = ""
        self.last_status_log_time = 0.0

        self.safe_pub = self.create_publisher(Float64MultiArray, self.safe_posx_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.target_sub = self.create_subscription(
            Float64MultiArray,
            self.target_posx_topic,
            self._on_target,
            1,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.robot_anchor_sub = self.create_subscription(
            Float64MultiArray,
            self.robot_anchor_posx_topic,
            self._on_robot_anchor_posx,
            state_qos,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._tick)
        self._publish_status(
            "safety_guard_node started: target="
            f"{self.target_posx_topic}, safe={self.safe_posx_topic}, "
            f"workspace_min_xyz_mm={self.workspace_min_xyz_mm}, "
            f"workspace_max_xyz_mm={self.workspace_max_xyz_mm}, "
            f"workspace_min_limit_enabled={self.workspace_min_limit_enabled}, "
            f"robot_anchor_posx_topic={self.robot_anchor_posx_topic}, "
            "robot_anchor_qos=KEEP_LAST(depth=1, RELIABLE, TRANSIENT_LOCAL), "
            f"enable_orientation_limits={self.enable_orientation_limits}, "
            f"max_orientation_delta_deg={self.max_orientation_delta_deg}, "
            "orientation_limit_mode=quaternion_geodesic, "
            f"orientation_geodesic_limit_deg={self.max_orientation_delta_deg[1]}, "
            f"publish_rate_hz={self.publish_rate_hz}, "
            "processing_mode=fixed_rate_clamp_only, ramp_owner=streamer"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("target_posx_topic", "/vr/target_posx")
        self.declare_parameter("safe_posx_topic", "/vr/safe_posx")
        self.declare_parameter("status_topic", "/vr/status")
        self.declare_parameter("robot_anchor_posx_topic", "/vr/robot_anchor_posx")
        self.declare_parameter("workspace_min_xyz_mm", [250.0, -350.0, 150.0])
        self.declare_parameter("workspace_max_xyz_mm", [650.0, 350.0, 600.0])
        self.declare_parameter(
            "workspace_min_limit_enabled", [True, True, True]
        )
        self.declare_parameter("enable_orientation_limits", True)
        self.declare_parameter("max_orientation_delta_deg", [15.0, 15.0, 20.0])
        self.declare_parameter("publish_rate_hz", 30.0)

    def _on_target(self, msg: Float64MultiArray) -> None:
        try:
            target = _posx(msg.data, "target_posx")
        except ValueError as exc:
            self._publish_status(f"ignored invalid target_posx: {exc}", warn=True)
            return
        self.latest_target = target

    def _on_robot_anchor_posx(self, msg: Float64MultiArray) -> None:
        try:
            posx = _posx(msg.data, "robot_anchor_posx")
        except ValueError as exc:
            self._publish_status(f"ignored invalid robot anchor posx: {exc}", warn=True)
            return
        self.orientation_anchor_rpy_deg = posx[3:6]
        self.last_safe_orientation_zyz_deg = self.orientation_anchor_rpy_deg[:]
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
            clamped[index] = clamp_workspace_axis(
                before,
                self.workspace_min_xyz_mm[index],
                self.workspace_max_xyz_mm[index],
                self.workspace_min_limit_enabled[index],
            )
            if clamped[index] != before:
                clamped_axes.append(axis)
        orientation_delta_deg: Optional[float] = None
        if self.enable_orientation_limits:
            if self.orientation_anchor_rpy_deg is None:
                self.orientation_anchor_rpy_deg = raw[3:6]
                self.last_safe_orientation_zyz_deg = raw[3:6]
                self._publish_status(
                    "orientation safety anchor initialized from first target: "
                    + json.dumps(
                        {"zyz_deg": self.orientation_anchor_rpy_deg},
                        sort_keys=True,
                    )
                )
            reference_zyz = (
                self.last_safe_orientation_zyz_deg
                if self.last_safe_orientation_zyz_deg is not None
                else self.orientation_anchor_rpy_deg
            )
            limited_zyz, orientation_delta_deg, orientation_limited = (
                limit_doosan_zyz_geodesic_deg(
                    self.orientation_anchor_rpy_deg,
                    raw[3:6],
                    self.max_orientation_delta_deg[1],
                    reference_zyz,
                )
            )
            clamped[3:6] = limited_zyz
            self.last_safe_orientation_zyz_deg = limited_zyz[:]
            if orientation_limited:
                clamped_axes.append("orientation")
        else:
            self.last_safe_orientation_zyz_deg = raw[3:6]

        msg = Float64MultiArray()
        msg.data = clamped
        self.safe_pub.publish(msg)

        event = {
            "raw": raw,
            "safe": clamped,
            "clamped_axes": clamped_axes,
            "orientation_geodesic_delta_deg": orientation_delta_deg,
            "orientation_geodesic_limit_deg": self.max_orientation_delta_deg[1],
        }
        if clamped_axes:
            self._publish_event("safety event: " + json.dumps(event, sort_keys=True), warn=True)
        else:
            now = time.monotonic()
            if now - self.last_status_log_time >= 1.0:
                self._publish_status("safe_posx=" + json.dumps({"data": clamped}, sort_keys=True))
                self.last_status_log_time = now

    def _publish_event(self, text: str, warn: bool = False) -> None:
        now = time.monotonic()
        if text != self.last_event_text or now - self.last_status_log_time >= 1.0:
            self._publish_status(text, warn=warn)
            self.last_event_text = text
            self.last_status_log_time = now

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
