"""Collect Doosan robot state into a simple visualization-friendly topic."""

from __future__ import annotations

import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

from dsr_msgs2.msg import RobotError


class RobotStateMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_state_monitor_node")
        self.declare_parameter("actual_tcp_topic", "/rt_topic/actual_tcp_position")
        self.declare_parameter("actual_joint_topic", "/rt_topic/actual_joint_position")
        self.declare_parameter("robot_state_topic", "/rt_topic/robot_state")
        self.declare_parameter("robot_error_topic", "/dsr01/error")
        self.declare_parameter("monitor_vis_topic", "/teleop/monitor_vis")
        self.declare_parameter("publish_rate_hz", 10.0)

        self.actual_tcp: Optional[list[float]] = None
        self.actual_joint: Optional[list[float]] = None
        self.robot_state: Optional[float] = None
        self.last_alarm: Optional[dict] = None
        self.last_update_sec = time.monotonic()

        self.vis_pub = self.create_publisher(
            String, self.get_parameter("monitor_vis_topic").value, 10
        )
        self.tcp_sub = self.create_subscription(
            Float32MultiArray,
            self.get_parameter("actual_tcp_topic").value,
            self._on_actual_tcp,
            10,
        )
        self.joint_sub = self.create_subscription(
            Float32MultiArray,
            self.get_parameter("actual_joint_topic").value,
            self._on_actual_joint,
            10,
        )
        self.robot_state_sub = self.create_subscription(
            Float32MultiArray,
            self.get_parameter("robot_state_topic").value,
            self._on_robot_state,
            10,
        )
        self.error_sub = self.create_subscription(
            RobotError,
            self.get_parameter("robot_error_topic").value,
            self._on_robot_error,
            10,
        )
        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self._publish)

    def _on_actual_tcp(self, msg: Float32MultiArray) -> None:
        self.actual_tcp = [float(v) for v in msg.data[:6]]
        self.last_update_sec = time.monotonic()

    def _on_actual_joint(self, msg: Float32MultiArray) -> None:
        self.actual_joint = [float(v) for v in msg.data[:6]]
        self.last_update_sec = time.monotonic()

    def _on_robot_state(self, msg: Float32MultiArray) -> None:
        self.robot_state = float(msg.data[0]) if msg.data else None
        self.last_update_sec = time.monotonic()

    def _on_robot_error(self, msg: RobotError) -> None:
        self.last_alarm = {
            "level": int(msg.level),
            "group": int(msg.group),
            "code": int(msg.code),
            "message": [msg.msg1, msg.msg2, msg.msg3],
        }

    def _publish(self) -> None:
        data = {
            "stamp": time.monotonic(),
            "actual_tcp": self.actual_tcp,
            "actual_joint": self.actual_joint,
            "robot_state": self.robot_state,
            "last_alarm": self.last_alarm,
            "source_age_sec": time.monotonic() - self.last_update_sec,
        }
        self.vis_pub.publish(String(data=json.dumps(data, sort_keys=True)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotStateMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
