"""MUX-first command routing for MetaQuest and LeRobot A0509 control."""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from quest_a0509_teleop.command_mux_core import (
    CommandMuxCore,
    ControlSource,
    MuxDecision,
)


class A0509CommandMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("a0509_command_mux_node")
        self._declare_parameters()

        initial_source = str(self.get_parameter("initial_control_source").value).strip().upper()
        if initial_source != ControlSource.DISABLED.value:
            raise ValueError("initial_control_source must be DISABLED")

        self.core = CommandMuxCore(
            metaquest_timeout_sec=float(
                self.get_parameter("metaquest_timeout_sec").value
            ),
            lerobot_timeout_sec=float(
                self.get_parameter("lerobot_timeout_sec").value
            ),
            gripper_open_threshold=float(
                self.get_parameter("gripper_open_threshold").value
            ),
            gripper_close_threshold=float(
                self.get_parameter("gripper_close_threshold").value
            ),
            metaquest_gripper_min_pulse_sec=float(
                self.get_parameter(
                    "metaquest_gripper_min_pulse_sec"
                ).value
            ),
            require_metaquest_calibration=bool(
                self.get_parameter("require_metaquest_calibration").value
            ),
        )
        self.status_topic = str(self.get_parameter("mux_status_topic").value)

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.target_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("selected_target_topic").value),
            10,
        )
        self.gripper_pub = self.create_publisher(
            String,
            str(self.get_parameter("selected_gripper_topic").value),
            10,
        )
        self.source_pub = self.create_publisher(
            String,
            str(self.get_parameter("source_topic").value),
            state_qos,
        )
        self.status_pub = self.create_publisher(String, self.status_topic, state_qos)
        self.heartbeat_pub = self.create_publisher(
            Empty,
            str(self.get_parameter("selected_heartbeat_topic").value),
            10,
        )
        self.valid_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("selected_valid_topic").value),
            state_qos,
        )

        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("metaquest_target_topic").value),
            lambda msg: self._on_arm_target(ControlSource.METAQUEST, msg),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("metaquest_gripper_topic").value),
            self._on_metaquest_gripper,
            10,
        )
        self.create_subscription(
            Empty,
            str(self.get_parameter("metaquest_heartbeat_topic").value),
            self._on_metaquest_heartbeat,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("metaquest_calibration_valid_topic").value),
            self._on_metaquest_calibration_valid,
            state_qos,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("lerobot_target_topic").value),
            lambda msg: self._on_arm_target(ControlSource.LEROBOT, msg),
            10,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("lerobot_gripper_topic").value),
            self._on_lerobot_gripper,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("live_state_topic").value),
            self._on_live_state,
            state_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("teleop_ready_topic").value),
            self._on_teleop_ready,
            state_qos,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("gripper_commanded_state_topic").value),
            self._on_gripper_commanded_state,
            state_qos,
        )

        self.create_service(
            Trigger,
            "/control/select_disabled",
            lambda request, response: self._on_select(
                ControlSource.DISABLED, request, response
            ),
        )
        self.create_service(
            Trigger,
            "/control/select_metaquest",
            lambda request, response: self._on_select(
                ControlSource.METAQUEST, request, response
            ),
        )
        self.create_service(
            Trigger,
            "/control/select_lerobot",
            lambda request, response: self._on_select(
                ControlSource.LEROBOT, request, response
            ),
        )

        self.set_live_client = self.create_client(
            SetBool,
            str(self.get_parameter("set_live_service").value),
        )
        self.hold_client = self.create_client(
            Trigger,
            str(self.get_parameter("hold_service").value),
        )
        self._last_status_signature: tuple[str, str, bool] | None = None
        self._last_status_time = 0.0
        self.timer = self.create_timer(0.05, self._on_timer)

        self._publish_source()
        self.valid_pub.publish(Bool(data=False))
        self._publish_status("startup", accepted=True, reason="initial source is DISABLED")

    def _declare_parameters(self) -> None:
        self.declare_parameter("initial_control_source", "DISABLED")
        self.declare_parameter(
            "metaquest_target_topic", "/control/metaquest/target_posx"
        )
        self.declare_parameter(
            "metaquest_gripper_topic", "/control/metaquest/gripper_cmd"
        )
        self.declare_parameter(
            "metaquest_heartbeat_topic",
            "/control/metaquest/valid_pose_heartbeat",
        )
        self.declare_parameter(
            "metaquest_calibration_valid_topic",
            "/vr/metaquest_calibration/valid",
        )
        self.declare_parameter("require_metaquest_calibration", True)
        self.declare_parameter("lerobot_target_topic", "/control/lerobot/target_posx")
        self.declare_parameter(
            "lerobot_gripper_topic", "/control/lerobot/gripper_target"
        )
        self.declare_parameter("selected_target_topic", "/vr/target_posx")
        self.declare_parameter("selected_gripper_topic", "/jrt_gripper/cmd")
        self.declare_parameter("source_topic", "/control/source")
        self.declare_parameter("mux_status_topic", "/control/mux_status")
        self.declare_parameter(
            "selected_heartbeat_topic", "/control/selected_command_heartbeat"
        )
        self.declare_parameter(
            "selected_valid_topic", "/control/selected_command_valid"
        )
        self.declare_parameter(
            "live_state_topic", "/vr/live_robot_output_enabled"
        )
        self.declare_parameter("teleop_ready_topic", "/vr/teleop_ready")
        self.declare_parameter(
            "gripper_commanded_state_topic", "/jrt_gripper/commanded_state"
        )
        self.declare_parameter("set_live_service", "/vr/set_live_robot_output")
        self.declare_parameter("hold_service", "/vr/hold_servol")
        self.declare_parameter("metaquest_timeout_sec", 1.0)
        self.declare_parameter("lerobot_timeout_sec", 0.3)
        self.declare_parameter("gripper_open_threshold", 0.3)
        self.declare_parameter("gripper_close_threshold", 0.7)
        self.declare_parameter("metaquest_gripper_min_pulse_sec", 1.50)

    def _on_select(
        self,
        source: ControlSource,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        decision = self.core.select_source(source, now=time.monotonic())
        response.success = decision.accepted
        response.message = decision.reason
        self._apply_decision("source_transition", decision)
        return response

    def _on_arm_target(
        self,
        source: ControlSource,
        msg: Float64MultiArray,
    ) -> None:
        decision = self.core.receive_arm_target(
            source,
            msg.data,
            now=time.monotonic(),
        )
        self._apply_decision("arm_target", decision, input_source=source.value)

    def _on_metaquest_gripper(self, msg: String) -> None:
        decision = self.core.receive_metaquest_gripper(
            msg.data,
            now=time.monotonic(),
        )
        self._apply_decision("metaquest_gripper", decision)

    def _on_lerobot_gripper(self, msg: Float64) -> None:
        decision = self.core.receive_lerobot_gripper(
            msg.data,
            now=time.monotonic(),
        )
        self._apply_decision("lerobot_gripper", decision)

    def _on_metaquest_heartbeat(self, _msg: Empty) -> None:
        self.core.note_metaquest_valid_pose(time.monotonic())

    def _on_metaquest_calibration_valid(self, msg: Bool) -> None:
        decision = self.core.set_metaquest_calibration_valid(
            msg.data,
            now=time.monotonic(),
        )
        if decision is not None:
            self._apply_decision("metaquest_calibration_invalidated", decision)
            return
        self._publish_status(
            "metaquest_calibration",
            accepted=True,
            reason=f"valid={self.core.metaquest_calibration_valid}",
        )

    def _on_live_state(self, msg: Bool) -> None:
        decision = self.core.set_live_enabled(msg.data)
        if decision is not None:
            self._apply_decision("live_disabled", decision)

    def _on_teleop_ready(self, msg: Bool) -> None:
        self.core.set_teleop_ready(msg.data)

    def _on_gripper_commanded_state(self, msg: Float64) -> None:
        try:
            self.core.update_commanded_gripper_state(msg.data)
        except ValueError as exc:
            self._publish_status(
                "gripper_commanded_state",
                accepted=False,
                reason=str(exc),
            )

    def _on_timer(self) -> None:
        decision = self.core.check_timeout(now=time.monotonic())
        if decision is None:
            return
        self._apply_decision("source_timeout", decision)

    def _apply_decision(
        self,
        event: str,
        decision: MuxDecision,
        **details: Any,
    ) -> None:
        if decision.source_changed:
            self._publish_source()
            self.valid_pub.publish(Bool(data=False))
        if decision.gripper_command is not None:
            self.gripper_pub.publish(String(data=decision.gripper_command))
        if decision.target_posx is not None:
            target = Float64MultiArray()
            target.data = list(decision.target_posx)
            self.target_pub.publish(target)
        if decision.emit_heartbeat:
            self.heartbeat_pub.publish(Empty())
            self.valid_pub.publish(Bool(data=True))
        if decision.disable_and_hold:
            self._request_disable_and_hold()

        should_report = (
            not decision.accepted
            or decision.source_changed
            or decision.timed_out
            or decision.target_posx is None
        )
        if should_report:
            self._publish_status(
                event,
                accepted=decision.accepted,
                reason=decision.reason,
                **details,
            )

    def _request_disable_and_hold(self) -> None:
        if self.set_live_client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            future = self.set_live_client.call_async(request)
            future.add_done_callback(self._on_disable_live_done)
            self._request_hold_best_effort()
            return
        self._publish_status(
            "timeout_disable_live",
            accepted=False,
            reason=f"service unavailable: {self.set_live_client.srv_name}",
        )
        self._request_hold_best_effort()

    def _on_disable_live_done(self, future: Any) -> None:
        try:
            response = future.result()
            self._publish_status(
                "timeout_disable_live",
                accepted=bool(response.success),
                reason=str(response.message),
            )
        except Exception as exc:
            self._publish_status(
                "timeout_disable_live",
                accepted=False,
                reason=f"service call failed: {exc}",
            )

    def _request_hold_best_effort(self) -> None:
        if not self.hold_client.service_is_ready():
            self._publish_status(
                "timeout_hold",
                accepted=False,
                reason=f"service unavailable: {self.hold_client.srv_name}",
            )
            return
        future = self.hold_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_hold_done)

    def _on_hold_done(self, future: Any) -> None:
        try:
            response = future.result()
            self._publish_status(
                "timeout_hold",
                accepted=bool(response.success),
                reason=str(response.message),
            )
        except Exception as exc:
            self._publish_status(
                "timeout_hold",
                accepted=False,
                reason=f"service call failed: {exc}",
            )

    def _publish_source(self) -> None:
        self.source_pub.publish(String(data=self.core.source.value))

    def _publish_status(
        self,
        event: str,
        *,
        accepted: bool,
        reason: str,
        **details: Any,
    ) -> None:
        now = time.monotonic()
        signature = (event, reason, bool(accepted))
        if signature == self._last_status_signature and now - self._last_status_time < 1.0:
            return
        self._last_status_signature = signature
        self._last_status_time = now
        payload = {
            "event": event,
            "accepted": bool(accepted),
            "reason": reason,
            "source": self.core.source.value,
            "live_enabled": self.core.live_enabled,
            "teleop_ready": self.core.teleop_ready,
            "metaquest_calibration_required": self.core.require_metaquest_calibration,
            "metaquest_calibration_valid": self.core.metaquest_calibration_valid,
            **details,
        }
        text = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(String(data=text))
        if accepted:
            self.get_logger().info(text)
        else:
            self.get_logger().warning(text)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = A0509CommandMuxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
