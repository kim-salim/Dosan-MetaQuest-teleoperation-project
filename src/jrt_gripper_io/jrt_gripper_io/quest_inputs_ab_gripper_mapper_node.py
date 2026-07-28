"""Map Quest2ROS right-controller A/B inputs to JRT gripper commands."""

from __future__ import annotations

import json
import time
from typing import Any, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def _field_pressed(msg: Any, field: str, threshold: float) -> bool:
    if not hasattr(msg, field):
        raise AttributeError(f"message has no field '{field}'")
    value = getattr(msg, field)
    if isinstance(value, bool):
        return value
    return float(value) >= threshold


class QuestInputsAbGripperMapperNode(Node):
    def __init__(self) -> None:
        super().__init__("quest_inputs_ab_gripper_mapper_node")
        self._declare_parameters()

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.a_button_field = str(self.get_parameter("a_button_field").value)
        self.b_button_field = str(self.get_parameter("b_button_field").value)
        self.button_threshold = float(self.get_parameter("button_threshold").value)
        self.watchdog_timeout_sec = float(
            self.get_parameter("watchdog_timeout_sec").value
        )
        self.command_pub = self.create_publisher(String, self.command_topic, 10)
        self._start_time = time.monotonic()
        self._last_input_time: float | None = None
        self._last_command: str | None = None
        self._last_missing_field_log_time = 0.0

        inputs_msg_type = self._load_inputs_msg_type()
        if inputs_msg_type is None:
            self.input_sub = None
            self.get_logger().error(
                "Quest2ROS input mapper disabled: quest2ros.msg.OVR2ROSInputs "
                "is unavailable. Build and source the integrated workspace."
            )
        else:
            self.input_sub = self.create_subscription(
                inputs_msg_type,
                self.input_topic,
                self._on_inputs,
                10,
            )

        timer_period = max(0.02, min(self.watchdog_timeout_sec / 2.0, 0.1))
        self.watchdog_timer = self.create_timer(timer_period, self._on_watchdog)

        self.get_logger().info(
            "quest_inputs_ab_gripper_mapper_node started: "
            + json.dumps(
                {
                    "input_topic": self.input_topic,
                    "command_topic": self.command_topic,
                    "a_button_field": self.a_button_field,
                    "b_button_field": self.b_button_field,
                    "button_threshold": self.button_threshold,
                    "watchdog_timeout_sec": self.watchdog_timeout_sec,
                },
                sort_keys=True,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("input_topic", "/q2r_right_hand_inputs")
        self.declare_parameter("command_topic", "/jrt_gripper/cmd")
        self.declare_parameter("a_button_field", "button_lower")
        self.declare_parameter("b_button_field", "button_upper")
        self.declare_parameter("button_threshold", 0.5)
        self.declare_parameter("watchdog_timeout_sec", 0.3)

    def _load_inputs_msg_type(self) -> Optional[type]:
        try:
            from quest2ros.msg import OVR2ROSInputs

            OVR2ROSInputs.__class__.__import_type_support__()
            return OVR2ROSInputs
        except Exception as exc:
            self.get_logger().error(f"failed to import OVR2ROSInputs: {exc}")
            return None

    def _on_inputs(self, msg: Any) -> None:
        self._last_input_time = time.monotonic()
        try:
            a_pressed = _field_pressed(
                msg,
                self.a_button_field,
                self.button_threshold,
            )
            b_pressed = _field_pressed(
                msg,
                self.b_button_field,
                self.button_threshold,
            )
        except (AttributeError, TypeError, ValueError) as exc:
            now = time.monotonic()
            if now - self._last_missing_field_log_time >= 2.0:
                self._last_missing_field_log_time = now
                self.get_logger().warning(
                    "invalid Quest input message for gripper A/B mapping: "
                    + json.dumps(
                        {
                            "error": str(exc),
                            "a_button_field": self.a_button_field,
                            "b_button_field": self.b_button_field,
                            "message_type": type(msg).__name__,
                        },
                        sort_keys=True,
                    )
                )
            return

        if a_pressed and not b_pressed:
            command = "close"
        elif b_pressed and not a_pressed:
            command = "open"
        else:
            command = "stop"
        self._publish_if_changed(command, "quest_inputs")

    def _on_watchdog(self) -> None:
        now = time.monotonic()
        if self._last_input_time is None:
            if now - self._start_time >= self.watchdog_timeout_sec:
                self._publish_if_changed("stop", "watchdog: no Quest inputs received")
            return

        if now - self._last_input_time >= self.watchdog_timeout_sec:
            self._publish_if_changed("stop", "watchdog: Quest input timeout")

    def _publish_if_changed(self, command: str, reason: str) -> None:
        if command == self._last_command:
            return
        self._last_command = command
        self.command_pub.publish(String(data=command))
        self.get_logger().info(f"gripper command -> {command} ({reason})")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = QuestInputsAbGripperMapperNode()
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
