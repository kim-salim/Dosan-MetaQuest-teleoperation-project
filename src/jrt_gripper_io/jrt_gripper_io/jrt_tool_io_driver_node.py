"""Drive a JRT gripper through Doosan Tool Digital Output services."""

from __future__ import annotations

import time
from collections import deque
from collections.abc import Iterable
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from jrt_gripper_io.gripper_logic import (
    ToolDoStep,
    normalize_command,
    normalize_command_mode,
    plan_tool_do_sequence,
)

try:
    from dsr_msgs2.srv import SetToolDigitalOutput
except Exception:  # pragma: no cover - only on systems without dsr_msgs2
    SetToolDigitalOutput = None


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


class JrtToolIoDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("jrt_tool_io_driver_node")
        self._declare_parameters()

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.set_tool_do_service = str(
            self.get_parameter("set_tool_do_service").value
        )
        self.close_do_index = int(self.get_parameter("close_do_index").value)
        self.open_do_index = int(self.get_parameter("open_do_index").value)
        self.active_value = int(self.get_parameter("active_value").value)
        self.inactive_value = int(self.get_parameter("inactive_value").value)
        self.service_timeout_sec = float(
            self.get_parameter("service_timeout_sec").value
        )
        self.command_mode = normalize_command_mode(
            str(self.get_parameter("command_mode").value)
        )
        self.pulse_sec = float(self.get_parameter("pulse_sec").value)
        self.interlock_sec = float(self.get_parameter("interlock_sec").value)
        self.debounce_sec = float(self.get_parameter("debounce_sec").value)
        self.startup_all_off = _as_bool(
            self.get_parameter("startup_all_off").value
        )
        self.shutdown_all_off = _as_bool(
            self.get_parameter("shutdown_all_off").value
        )
        self.dry_run = _as_bool(self.get_parameter("dry_run").value)

        self.client = None
        if not self.dry_run:
            if SetToolDigitalOutput is None:
                self.get_logger().error(
                    "dsr_msgs2.srv.SetToolDigitalOutput is unavailable; "
                    "real Tool I/O mode cannot call Doosan services."
                )
            else:
                self.client = self.create_client(
                    SetToolDigitalOutput,
                    self.set_tool_do_service,
                )

        self.command_sub = self.create_subscription(
            String,
            self.command_topic,
            self._on_command,
            10,
        )

        self._last_command: str | None = None
        self._current_command: str | None = None
        self._plan_steps: deque[ToolDoStep] = deque()
        self._step_in_flight = False
        self._delay_timer = None
        self._last_motion_command_time = 0.0

        if self.close_do_index == self.open_do_index:
            self.get_logger().error(
                "close_do_index and open_do_index are identical; "
                "close/open commands "
                "will be forced to stop."
            )
        if self.active_value == self.inactive_value:
            self.get_logger().error(
                "active_value and inactive_value are identical; "
                "Tool DO on/off states "
                "cannot be distinguished."
            )

        self.get_logger().info(
            "jrt_tool_io_driver_node started: "
            f"command_topic={self.command_topic}, "
            f"service={self.set_tool_do_service}, "
            f"close_do_index={self.close_do_index}, "
            f"open_do_index={self.open_do_index}, "
            f"active_value={self.active_value}, "
            f"inactive_value={self.inactive_value}, "
            f"command_mode={self.command_mode}, "
            f"pulse_sec={self.pulse_sec:.3f}, "
            f"interlock_sec={self.interlock_sec:.3f}, "
            f"debounce_sec={self.debounce_sec:.3f}, "
            f"startup_all_off={self.startup_all_off}, "
            f"shutdown_all_off={self.shutdown_all_off}, "
            f"dry_run={self.dry_run}"
        )
        if self.startup_all_off:
            self._start_command("stop")

    def _declare_parameters(self) -> None:
        self.declare_parameter("command_topic", "/jrt_gripper/cmd")
        self.declare_parameter(
            "set_tool_do_service",
            "/dsr01/io/set_tool_digital_output",
        )
        self.declare_parameter("close_do_index", 1)
        self.declare_parameter("open_do_index", 2)
        self.declare_parameter("active_value", 1)
        self.declare_parameter("inactive_value", 0)
        self.declare_parameter("service_timeout_sec", 1.0)
        self.declare_parameter("command_mode", "pulse")
        self.declare_parameter("pulse_sec", 0.20)
        self.declare_parameter("interlock_sec", 0.05)
        self.declare_parameter("debounce_sec", 0.30)
        self.declare_parameter("startup_all_off", True)
        self.declare_parameter("shutdown_all_off", True)
        self.declare_parameter("dry_run", False)

    def _on_command(self, msg: String) -> None:
        raw_command = msg.data.strip().lower()
        command = normalize_command(raw_command)
        if raw_command != command:
            self.get_logger().warning(
                f"unknown gripper command '{msg.data}'; sending stop"
            )

        if self._is_plan_active():
            self.get_logger().warning(
                "ignoring gripper command -> "
                f"{command} while Tool DO plan is active"
            )
            return

        if command == self._last_command:
            return

        if self._is_debounced_motion_command(command):
            return

        self._start_command(command)

    def _is_debounced_motion_command(self, command: str) -> bool:
        if command not in {"close", "open"}:
            return False
        now = time.monotonic()
        elapsed = now - self._last_motion_command_time
        if elapsed < self.debounce_sec:
            self.get_logger().warning(
                "ignoring gripper command -> "
                f"{command} during debounce window "
                f"({elapsed:.3f}s < {self.debounce_sec:.3f}s)"
            )
            return True
        self._last_motion_command_time = now
        return False

    def _start_command(self, command: str) -> None:
        safe_command = command
        try:
            steps = plan_tool_do_sequence(
                safe_command,
                self.close_do_index,
                self.open_do_index,
                self.active_value,
                self.inactive_value,
                command_mode=self.command_mode,
                pulse_sec=self.pulse_sec,
                interlock_sec=self.interlock_sec,
            )
        except ValueError as exc:
            self.get_logger().error(f"{exc}; sending stop instead")
            safe_command = "stop"
            steps = plan_tool_do_sequence(
                safe_command,
                self.close_do_index,
                self.open_do_index,
                self.active_value,
                self.inactive_value,
                command_mode="level",
            )

        if self.dry_run:
            self._log_dry_run_plan(safe_command, steps)
            self._last_command = safe_command
            return

        if self.client is None:
            self.get_logger().error(
                "Tool DO service client is unavailable; "
                "cannot send gripper command."
            )
            return

        service_ready = self.client.wait_for_service(
            timeout_sec=self.service_timeout_sec
        )
        if not service_ready:
            self.get_logger().error(
                "Tool DO service unavailable after "
                f"{self.service_timeout_sec:.3f}s: "
                f"{self.set_tool_do_service}"
            )
            return

        self._current_command = safe_command
        self._plan_steps = deque(steps)
        self.get_logger().info(f"applying gripper command -> {safe_command}")
        self._send_next_step()

    def _log_dry_run_plan(
        self,
        command: str,
        steps: Iterable[ToolDoStep],
    ) -> None:
        rendered_steps = ", ".join(
            (
                f"{self.set_tool_do_service}"
                f"(index={step.index}, value={step.value})"
                f"{self._render_delay(step.delay_after_sec)}"
            )
            for step in steps
        )
        self.get_logger().info(
            f"dry_run gripper command -> {command}: {rendered_steps}"
        )

    @staticmethod
    def _render_delay(delay_after_sec: float) -> str:
        if delay_after_sec <= 0.0:
            return ""
        return f", wait={delay_after_sec:.3f}s"

    def _send_next_step(self) -> None:
        if not self._plan_steps:
            completed = self._current_command
            self._current_command = None
            self._step_in_flight = False
            if completed is not None:
                self._last_command = completed
                self.get_logger().info(
                    f"gripper DO command applied -> {completed}"
                )
            return

        step = self._plan_steps.popleft()
        self._step_in_flight = True
        self.get_logger().info(
            "calling Tool DO: "
            f"{self.set_tool_do_service}"
            f"(index={step.index}, value={step.value})"
        )
        request = SetToolDigitalOutput.Request()
        request.index = int(step.index)
        request.value = int(step.value)
        future = self.client.call_async(request)
        future.add_done_callback(
            lambda done_future, sent_step=step: self._on_step_response(
                done_future,
                sent_step,
            )
        )

    def _on_step_response(self, future: Any, step: ToolDoStep) -> None:
        self._step_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                "Tool DO service call failed for "
                f"index={step.index}, value={step.value}: {exc}"
            )
            self._handle_service_failure()
            return

        if not bool(response.success):
            self.get_logger().error(
                "Tool DO service rejected "
                f"index={step.index}, value={step.value}; sending stop"
            )
            self._handle_service_failure()
            return

        if step.delay_after_sec > 0.0:
            self._delay_timer = self.create_timer(
                step.delay_after_sec,
                self._on_delay_elapsed,
            )
            return

        self._send_next_step()

    def _on_delay_elapsed(self) -> None:
        timer = self._delay_timer
        self._delay_timer = None
        if timer is not None:
            timer.cancel()
        self._send_next_step()

    def _handle_service_failure(self) -> None:
        failed_command = self._current_command
        if self._delay_timer is not None:
            self._delay_timer.cancel()
            self._delay_timer = None
        self._plan_steps.clear()
        self._current_command = None
        if failed_command != "stop":
            self.get_logger().error(
                "attempting failsafe stop: both Tool DO outputs OFF"
            )
            self._start_command("stop")

    def _is_plan_active(self) -> bool:
        return (
            self._step_in_flight
            or bool(self._plan_steps)
            or self._delay_timer is not None
        )

    def shutdown_stop(self) -> None:
        if not self.shutdown_all_off:
            return

        steps = plan_tool_do_sequence(
            "stop",
            self.close_do_index,
            self.open_do_index,
            self.active_value,
            self.inactive_value,
            command_mode="level",
        )
        if self.dry_run:
            self._log_dry_run_plan("shutdown stop", steps)
            return

        if not rclpy.ok() or self.client is None:
            return
        self.get_logger().info("shutdown: sending gripper stop")
        for step in steps:
            self._call_tool_do_sync(step.index, step.value)

    def _call_tool_do_sync(self, index: int, value: int) -> bool:
        if self.client is None:
            return False
        service_ready = self.client.wait_for_service(
            timeout_sec=self.service_timeout_sec
        )
        if not service_ready:
            self.get_logger().error(
                "Tool DO service unavailable during shutdown stop: "
                f"{self.set_tool_do_service}"
            )
            return False
        request = SetToolDigitalOutput.Request()
        request.index = int(index)
        request.value = int(value)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self.service_timeout_sec,
        )
        if not future.done():
            self.get_logger().error(
                "shutdown Tool DO call timed out: "
                f"index={index}, value={value}"
            )
            return False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                "shutdown Tool DO call failed: "
                f"index={index}, value={value}: {exc}"
            )
            return False
        if not bool(response.success):
            self.get_logger().error(
                f"shutdown Tool DO call rejected: index={index}, value={value}"
            )
            return False
        return True


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = JrtToolIoDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
