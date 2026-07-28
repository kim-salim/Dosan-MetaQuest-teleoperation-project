"""Map Quest2ROS controller button edges to teleop services."""

from __future__ import annotations

import json
import time
from typing import Any, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import Trigger


def _string_list(value: Any) -> list[str]:
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    return [str(item).strip() for item in value if str(item).strip()]


class QuestInputButtonNode(Node):
    def __init__(self) -> None:
        super().__init__("quest_input_button_node")
        self._declare_parameters()

        self.input_topic = self.get_parameter("input_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.grip_input_field = str(self.get_parameter("grip_input_field").value).strip()
        if not self.grip_input_field:
            legacy_fields = _string_list(self.get_parameter("grip_input_fields").value)
            self.grip_input_field = legacy_fields[0] if legacy_fields else "press_middle"
        self.grip_threshold = float(self.get_parameter("grip_threshold").value)
        self.debounce_sec = float(self.get_parameter("debounce_sec").value)
        self.teleop_ready_topic = self.get_parameter("teleop_ready_topic").value
        self.require_teleop_ready = bool(self.get_parameter("require_teleop_ready").value)
        self.toggle_roll_lock_service = self.get_parameter(
            "toggle_roll_lock_service"
        ).value
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.toggle_roll_lock_client = self.create_client(
            Trigger,
            self.toggle_roll_lock_service,
        )
        self.last_grip_pressed = False
        self.last_toggle_time = 0.0
        self.pending_toggle = False
        self.last_missing_field_log_time = 0.0
        self.last_not_ready_log_time = 0.0
        self.teleop_ready = False
        self.teleop_ready_sub = self.create_subscription(
            Bool,
            self.teleop_ready_topic,
            self._on_teleop_ready,
            10,
        )

        inputs_msg_type = self._load_inputs_msg_type()
        if inputs_msg_type is None:
            self.input_sub = None
            self._publish_status(
                "Quest input button bridge disabled: quest2ros.msg.OVR2ROSInputs "
                "is unavailable. Build and source the integrated workspace.",
                warn=True,
            )
            return

        self.input_sub = self.create_subscription(
            inputs_msg_type,
            self.input_topic,
            self._on_inputs,
            10,
        )
        self._publish_status(
            "quest_input_button_node started: "
            + json.dumps(
                {
                    "input_topic": self.input_topic,
                    "grip_input_field": self.grip_input_field,
                    "grip_threshold": self.grip_threshold,
                    "debounce_sec": self.debounce_sec,
                    "teleop_ready_topic": self.teleop_ready_topic,
                    "require_teleop_ready": self.require_teleop_ready,
                    "toggle_roll_lock_service": self.toggle_roll_lock_service,
                },
                sort_keys=True,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("input_topic", "/q2r_right_hand_inputs")
        self.declare_parameter("status_topic", "/vr/status")
        self.declare_parameter("grip_input_field", "press_middle")
        self.declare_parameter(
            "grip_input_fields",
            ["press_middle"],
        )
        self.declare_parameter("grip_threshold", 0.5)
        self.declare_parameter("debounce_sec", 0.7)
        self.declare_parameter("teleop_ready_topic", "/vr/teleop_ready")
        self.declare_parameter("require_teleop_ready", True)
        self.declare_parameter("toggle_roll_lock_service", "/vr/toggle_roll_lock")

    def _load_inputs_msg_type(self) -> Optional[type]:
        try:
            from quest2ros.msg import OVR2ROSInputs

            OVR2ROSInputs.__class__.__import_type_support__()
            return OVR2ROSInputs
        except Exception as exc:
            self._publish_status(
                f"failed to import quest2ros.msg.OVR2ROSInputs: {exc}",
                warn=True,
            )
            return None

    def _on_inputs(self, msg: Any) -> None:
        now = time.monotonic()
        try:
            grip_pressed = self._grip_pressed(msg)
        except AttributeError as exc:
            if now - self.last_missing_field_log_time >= 2.0:
                self._publish_status(str(exc), warn=True)
                self.last_missing_field_log_time = now
            return

        if grip_pressed and not self.last_grip_pressed:
            if self.require_teleop_ready and not self.teleop_ready:
                if now - self.last_not_ready_log_time >= 1.0:
                    self._publish_status(
                        "ignored grip edge: teleop_ready=false; run Prepare Robot first"
                    )
                    self.last_not_ready_log_time = now
            elif now - self.last_toggle_time >= self.debounce_sec:
                self.last_toggle_time = now
                self._toggle_roll_lock()
        self.last_grip_pressed = grip_pressed

    def _grip_pressed(self, msg: Any) -> bool:
        field = self.grip_input_field
        if hasattr(msg, field):
            value = getattr(msg, field)
            if isinstance(value, bool):
                return value
            try:
                return float(value) >= self.grip_threshold
            except (TypeError, ValueError) as exc:
                raise AttributeError(
                    "Quest input message has invalid configured grip field value: "
                    + json.dumps(
                        {
                            "configured_field": field,
                            "message_type": type(msg).__name__,
                            "value": repr(value),
                        },
                        sort_keys=True,
                    )
                ) from exc
        raise AttributeError(
            "Quest input message has no configured grip field: "
            + json.dumps(
                {
                    "configured_field": field,
                    "message_type": type(msg).__name__,
                },
                sort_keys=True,
            )
        )

    def _on_teleop_ready(self, msg: Bool) -> None:
        ready = bool(msg.data)
        if ready != self.teleop_ready:
            self.teleop_ready = ready
            self._publish_status(f"teleop_ready={self.teleop_ready}")

    def _toggle_roll_lock(self) -> None:
        if self.pending_toggle:
            self._publish_status("ignored grip edge: roll-lock toggle service call pending")
            return
        if not self.toggle_roll_lock_client.service_is_ready():
            self.toggle_roll_lock_client.wait_for_service(timeout_sec=0.05)
        if not self.toggle_roll_lock_client.service_is_ready():
            self._publish_status(
                f"ignored grip edge: service unavailable: {self.toggle_roll_lock_service}",
                warn=True,
            )
            return
        self.pending_toggle = True
        future = self.toggle_roll_lock_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_toggle_response)
        self._publish_status("grip rising edge detected; requested roll-lock toggle")

    def _on_toggle_response(self, future: Any) -> None:
        self.pending_toggle = False
        try:
            response = future.result()
        except Exception as exc:
            self._publish_status(f"roll-lock toggle failed: {exc}", warn=True)
            return
        level = "ok" if response.success else "rejected"
        self._publish_status(f"roll-lock toggle {level}: {response.message}")

    def _publish_status(self, text: str, warn: bool = False) -> None:
        if hasattr(self, "status_pub"):
            self.status_pub.publish(String(data=text))
        if warn:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = QuestInputButtonNode()
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
