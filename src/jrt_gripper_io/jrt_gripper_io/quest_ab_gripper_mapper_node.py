"""Map Quest Joy A/B buttons to JRT gripper command strings."""

from __future__ import annotations

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from jrt_gripper_io.gripper_logic import command_from_buttons


class QuestAbGripperMapperNode(Node):
    def __init__(self) -> None:
        super().__init__("quest_ab_gripper_mapper_node")
        self._declare_parameters()

        self.joy_topic = str(self.get_parameter("joy_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.a_button_index = int(self.get_parameter("a_button_index").value)
        self.b_button_index = int(self.get_parameter("b_button_index").value)
        self.watchdog_timeout_sec = float(
            self.get_parameter("watchdog_timeout_sec").value
        )

        self.command_pub = self.create_publisher(String, self.command_topic, 10)
        self.joy_sub = self.create_subscription(
            Joy,
            self.joy_topic,
            self._on_joy,
            10,
        )

        self._start_time = time.monotonic()
        self._last_joy_time: float | None = None
        self._last_command: str | None = None
        self._last_index_warning_time = 0.0
        timer_period = max(0.02, min(self.watchdog_timeout_sec / 2.0, 0.1))
        self.watchdog_timer = self.create_timer(timer_period, self._on_watchdog)

        self.get_logger().info(
            "quest_ab_gripper_mapper_node started: "
            f"joy_topic={self.joy_topic}, command_topic={self.command_topic}, "
            f"a_button_index={self.a_button_index}, b_button_index={self.b_button_index}, "
            f"watchdog_timeout_sec={self.watchdog_timeout_sec}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("a_button_index", 0)
        self.declare_parameter("b_button_index", 1)
        self.declare_parameter("watchdog_timeout_sec", 0.3)
        self.declare_parameter("command_topic", "/jrt_gripper/cmd")

    def _on_joy(self, msg: Joy) -> None:
        self._last_joy_time = time.monotonic()
        self._warn_if_button_indices_are_missing(msg)
        command = command_from_buttons(
            msg.buttons,
            self.a_button_index,
            self.b_button_index,
        )
        self._publish_if_changed(command, "joy")

    def _warn_if_button_indices_are_missing(self, msg: Joy) -> None:
        max_index = max(self.a_button_index, self.b_button_index)
        if max_index < len(msg.buttons):
            return
        now = time.monotonic()
        if now - self._last_index_warning_time < 2.0:
            return
        self._last_index_warning_time = now
        self.get_logger().warning(
            "Joy message has too few buttons for configured A/B indices: "
            f"button_count={len(msg.buttons)}, a_button_index={self.a_button_index}, "
            f"b_button_index={self.b_button_index}"
        )

    def _on_watchdog(self) -> None:
        now = time.monotonic()
        if self._last_joy_time is None:
            if now - self._start_time >= self.watchdog_timeout_sec:
                self._publish_if_changed("stop", "watchdog: no Joy received")
            return

        if now - self._last_joy_time >= self.watchdog_timeout_sec:
            self._publish_if_changed("stop", "watchdog: Joy timeout")

    def _publish_if_changed(self, command: str, reason: str) -> None:
        if command == self._last_command:
            return
        self._last_command = command
        self.command_pub.publish(String(data=command))
        self.get_logger().info(f"gripper command -> {command} ({reason})")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = QuestAbGripperMapperNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
