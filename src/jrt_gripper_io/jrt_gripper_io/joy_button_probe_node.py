"""Print Joy button index transitions for identifying Quest A/B buttons."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyButtonProbeNode(Node):
    def __init__(self) -> None:
        super().__init__("joy_button_probe_node")
        self.declare_parameter("joy_topic", "/joy")
        self.joy_topic = str(self.get_parameter("joy_topic").value)
        self.previous_buttons: list[int] | None = None

        self.joy_sub = self.create_subscription(
            Joy,
            self.joy_topic,
            self._on_joy,
            10,
        )
        self.get_logger().info(
            f"joy_button_probe_node started: joy_topic={self.joy_topic}"
        )

    def _on_joy(self, msg: Joy) -> None:
        current = [int(value) for value in msg.buttons]
        if self.previous_buttons is None:
            self.previous_buttons = current
            self.get_logger().info(
                f"received first Joy message with {len(current)} buttons"
            )
            return

        max_len = max(len(self.previous_buttons), len(current))
        for index in range(max_len):
            before = self.previous_buttons[index] if index < len(self.previous_buttons) else 0
            after = current[index] if index < len(current) else 0
            if before != after:
                self.get_logger().info(f"button[{index}] {before} -> {after}")
        self.previous_buttons = current


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = JoyButtonProbeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
