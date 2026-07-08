"""Convert projected target poses to Doosan ServolRtStream messages."""

from __future__ import annotations

import json
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from dsr_msgs2.msg import ServolRtStream

from .pose_utils import normalize_pose6


def make_servol_rt_stream(
    pose: list[float],
    vel: list[float],
    acc: list[float],
    time_sec: float,
) -> ServolRtStream:
    msg = ServolRtStream()
    msg.pos = normalize_pose6(pose)
    msg.vel = normalize_pose6(vel)
    msg.acc = normalize_pose6(acc)
    msg.time = float(time_sec)
    return msg


class ServolRtStreamerNode(Node):
    def __init__(self) -> None:
        super().__init__("servol_rt_streamer_node")
        self._declare_parameters()
        self.enable_robot_output = bool(self.get_parameter("enable_robot_output").value)
        self.projected_target_topic = self.get_parameter("projected_target_topic").value
        self.debug_topic = self.get_parameter("debug_servol_topic").value
        self.servol_topic = self.get_parameter("servol_topic").value
        self.servol_time_sec = float(self.get_parameter("servol_time_sec").value)
        self.servol_vel = normalize_pose6(self.get_parameter("servol_vel").value)
        self.servol_acc = normalize_pose6(self.get_parameter("servol_acc").value)

        self.last_target: Optional[list[float]] = None
        self.debug_pub = self.create_publisher(ServolRtStream, self.debug_topic, 10)
        self.status_pub = self.create_publisher(String, "/teleop/servol_streamer_status", 10)
        self.robot_pub = (
            self.create_publisher(ServolRtStream, self.servol_topic, 10)
            if self.enable_robot_output
            else None
        )
        self.sub = self.create_subscription(
            Float64MultiArray,
            self.projected_target_topic,
            self._on_projected_target,
            10,
        )
        period = 1.0 / float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(period, self._tick)
        mode = "ROBOT OUTPUT ENABLED" if self.enable_robot_output else "dry-run"
        self.get_logger().warning(
            f"ServoL RT streamer started in {mode}; robot_topic={self.servol_topic}, debug={self.debug_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("servol_time_sec", 0.5)
        self.declare_parameter("enable_robot_output", False)
        self.declare_parameter("robot_namespace", "/dsr01")
        self.declare_parameter("servol_topic", "/dsr01/servol_rt_stream")
        self.declare_parameter("projected_target_topic", "/teleop/projected_target_pose")
        self.declare_parameter("debug_servol_topic", "/teleop/debug_servol_rt_stream")
        self.declare_parameter("servol_vel", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("servol_acc", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def _on_projected_target(self, msg: Float64MultiArray) -> None:
        try:
            self.last_target = normalize_pose6(msg.data)
        except ValueError as exc:
            self.get_logger().warning(f"Invalid projected target ignored: {exc}")

    def _tick(self) -> None:
        if self.last_target is None:
            return
        msg = make_servol_rt_stream(
            self.last_target,
            self.servol_vel,
            self.servol_acc,
            self.servol_time_sec,
        )
        self.debug_pub.publish(msg)
        if self.robot_pub is not None:
            self.robot_pub.publish(msg)
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "enable_robot_output": self.enable_robot_output,
                        "published_robot": self.robot_pub is not None,
                        "servol_topic": self.servol_topic,
                        "target": self.last_target,
                        "time": self.servol_time_sec,
                    },
                    sort_keys=True,
                )
            )
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ServolRtStreamerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
