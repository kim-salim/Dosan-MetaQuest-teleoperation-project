"""Safety guard for projected ServoL RT target generation."""

from __future__ import annotations

import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from .pose_utils import bool_from_state, normalize_pose6, parse_controller_state_json
from .projection_utils import WorkspaceBox
from .safety_core import SafetyGuardCore, SafetyInput


class SafetyGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_guard_node")
        self._declare_parameters()
        self.raw_target_topic = self.get_parameter("raw_target_topic").value
        self.projected_target_topic = self.get_parameter("projected_target_topic").value
        self.controller_state_topic = self.get_parameter("controller_state_topic").value
        self.status_topic = self.get_parameter("safety_status_topic").value

        workspace = WorkspaceBox.from_mapping(self._workspace_params())
        self.core = SafetyGuardCore(
            workspace=workspace,
            linear_ramp_mm_per_tick=float(self.get_parameter("linear_ramp_mm_per_tick").value),
            angular_ramp_deg_per_tick=float(self.get_parameter("angular_ramp_deg_per_tick").value),
            vr_timeout_sec=float(self.get_parameter("vr_timeout_sec").value),
            require_deadman=bool(self.get_parameter("require_deadman").value),
            hold_on_tracking_lost=bool(self.get_parameter("hold_on_tracking_lost").value),
            hold_on_deadman_release=bool(self.get_parameter("hold_on_deadman_release").value),
            outside_workspace_policy=self.get_parameter("outside_workspace_policy").value,
            projection_method=self.get_parameter("projection_method").value,
        )
        self.core.reset(self.get_parameter("initial_safe_pose").value)

        self.last_raw_pose: Optional[list[float]] = None
        self.tracking_ok = False
        self.deadman_pressed = False
        self.last_controller_stamp_sec: Optional[float] = None

        self.target_pub = self.create_publisher(Float64MultiArray, self.projected_target_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.raw_sub = self.create_subscription(
            Float64MultiArray, self.raw_target_topic, self._on_raw_target, 10
        )
        self.controller_sub = self.create_subscription(
            String, self.controller_state_topic, self._on_controller_state, 10
        )
        period = 1.0 / float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"Safety guard running at {1.0 / period:.1f}Hz, publishing {self.projected_target_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("linear_ramp_mm_per_tick", 20.0)
        self.declare_parameter("angular_ramp_deg_per_tick", 3.0)
        self.declare_parameter("controller_state_topic", "/vr/controller_state")
        self.declare_parameter("raw_target_topic", "/teleop/raw_target_pose")
        self.declare_parameter("projected_target_topic", "/teleop/projected_target_pose")
        self.declare_parameter("safety_status_topic", "/teleop/safety_status")
        self.declare_parameter("initial_safe_pose", [400.0, 0.0, 350.0, 0.0, 150.0, 0.0])
        self.declare_parameter("workspace.x_min_mm", 250.0)
        self.declare_parameter("workspace.x_max_mm", 750.0)
        self.declare_parameter("workspace.y_min_mm", -350.0)
        self.declare_parameter("workspace.y_max_mm", 350.0)
        self.declare_parameter("workspace.z_min_mm", 120.0)
        self.declare_parameter("workspace.z_max_mm", 600.0)
        self.declare_parameter("workspace.rx_min_deg", -180.0)
        self.declare_parameter("workspace.rx_max_deg", 180.0)
        self.declare_parameter("workspace.ry_min_deg", -180.0)
        self.declare_parameter("workspace.ry_max_deg", 180.0)
        self.declare_parameter("workspace.rz_min_deg", -180.0)
        self.declare_parameter("workspace.rz_max_deg", 180.0)
        self.declare_parameter("vr_timeout_sec", 0.3)
        self.declare_parameter("require_deadman", True)
        self.declare_parameter("outside_workspace_policy", "project_to_boundary")
        self.declare_parameter("projection_method", "segment_boundary")
        self.declare_parameter("hold_on_tracking_lost", True)
        self.declare_parameter("hold_on_deadman_release", True)

    def _workspace_params(self) -> dict:
        keys = (
            "x_min_mm",
            "x_max_mm",
            "y_min_mm",
            "y_max_mm",
            "z_min_mm",
            "z_max_mm",
            "rx_min_deg",
            "rx_max_deg",
            "ry_min_deg",
            "ry_max_deg",
            "rz_min_deg",
            "rz_max_deg",
        )
        return {key: self.get_parameter(f"workspace.{key}").value for key in keys}

    def _on_raw_target(self, msg: Float64MultiArray) -> None:
        try:
            self.last_raw_pose = normalize_pose6(msg.data)
        except ValueError as exc:
            self.get_logger().warning(f"Invalid raw target ignored: {exc}")

    def _on_controller_state(self, msg: String) -> None:
        try:
            data = parse_controller_state_json(msg.data)
        except ValueError as exc:
            self.get_logger().warning(f"Invalid controller_state ignored: {exc}")
            return
        self.tracking_ok = bool_from_state(data, "tracking_ok", False)
        self.deadman_pressed = bool_from_state(data, "deadman", False)
        stamp = data.get("stamp")
        self.last_controller_stamp_sec = float(stamp) if isinstance(stamp, (int, float)) else time.monotonic()

    def _tick(self) -> None:
        sample = SafetyInput(
            raw_pose=self.last_raw_pose,
            tracking_ok=self.tracking_ok,
            deadman_pressed=self.deadman_pressed,
            stamp_sec=self.last_controller_stamp_sec,
        )
        result = self.core.update(sample, now_sec=time.monotonic())
        target_msg = Float64MultiArray()
        target_msg.data = result.target_pose
        self.target_pub.publish(target_msg)
        self.status_pub.publish(String(data=json.dumps(result.__dict__, default=list, sort_keys=True)))


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
