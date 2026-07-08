"""Map VR controller pose into robot-base raw target pose.

Scale is intentionally fixed to 1:1. The node uses a VR anchor and robot TCP
anchor. When clutch/recenter is pressed, both anchors are reset so relative
motion resumes from the current raw target.
"""

from __future__ import annotations

import json
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from .pose_utils import (
    add_pose6,
    bool_from_state,
    normalize_pose6,
    parse_controller_state_json,
    pose6_from_json_dict,
    sub_pose6,
)


class VrFrameMapperNode(Node):
    def __init__(self) -> None:
        super().__init__("vr_frame_mapper_node")
        self.declare_parameter("controller_state_topic", "/vr/controller_state")
        self.declare_parameter("raw_target_topic", "/teleop/raw_target_pose")
        self.declare_parameter("robot_tcp_anchor_pose", [400.0, 0.0, 350.0, 0.0, 150.0, 0.0])
        self.declare_parameter("publish_tracking_lost", False)

        self.raw_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("raw_target_topic").value, 10
        )
        self.sub = self.create_subscription(
            String,
            self.get_parameter("controller_state_topic").value,
            self._on_controller_state,
            10,
        )
        self.vr_anchor: Optional[list[float]] = None
        self.robot_anchor = normalize_pose6(self.get_parameter("robot_tcp_anchor_pose").value)
        self.current_raw = self.robot_anchor[:]
        self.last_clutch = False

    def _on_controller_state(self, msg: String) -> None:
        try:
            data = parse_controller_state_json(msg.data)
            vr_pose = pose6_from_json_dict(data)
        except ValueError as exc:
            self.get_logger().warning(f"Invalid controller_state ignored: {exc}")
            return

        tracking_ok = bool_from_state(data, "tracking_ok", True)
        if not tracking_ok and not bool(self.get_parameter("publish_tracking_lost").value):
            return

        clutch = bool_from_state(data, "clutch", False) or bool_from_state(data, "recenter", False)
        if self.vr_anchor is None or (clutch and not self.last_clutch):
            self.vr_anchor = vr_pose[:]
            self.robot_anchor = self.current_raw[:]
            self.get_logger().info(
                "VR anchor reset: "
                + json.dumps({"vr_anchor": self.vr_anchor, "robot_anchor": self.robot_anchor})
            )
        self.last_clutch = clutch

        delta = sub_pose6(vr_pose, self.vr_anchor)
        self.current_raw = add_pose6(self.robot_anchor, delta)
        out = Float64MultiArray()
        out.data = self.current_raw[:]
        self.raw_pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VrFrameMapperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
