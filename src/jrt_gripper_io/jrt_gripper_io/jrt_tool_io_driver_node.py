"""Drive a JRT gripper through Doosan Tool Digital Output services."""

from __future__ import annotations

import time
from collections import deque
from collections.abc import Iterable
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String

from jrt_gripper_io.driver_diagnostics import GripperDriverDiagnostics

from jrt_gripper_io.gripper_logic import (
    PendingGripperCommand,
    ToolDoStep,
    normalize_command,
    normalize_command_mode,
    plan_tool_do_sequence,
)

try:
    from dsr_msgs2.srv import GetToolDigitalOutput, SetToolDigitalOutput
except Exception:  # pragma: no cover - only on systems without dsr_msgs2
    GetToolDigitalOutput = None
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
        self.get_tool_do_service = str(
            self.get_parameter("get_tool_do_service").value
        )
        self.close_do_index = int(self.get_parameter("close_do_index").value)
        self.open_do_index = int(self.get_parameter("open_do_index").value)
        self.active_value = int(self.get_parameter("active_value").value)
        self.inactive_value = int(self.get_parameter("inactive_value").value)
        self.service_timeout_sec = float(
            self.get_parameter("service_timeout_sec").value
        )
        if self.service_timeout_sec <= 0.0:
            raise ValueError("service_timeout_sec must be > 0.0")
        self.readback_timeout_sec = float(
            self.get_parameter("readback_timeout_sec").value
        )
        if self.readback_timeout_sec <= 0.0:
            raise ValueError("readback_timeout_sec must be > 0.0")
        self.readback_poll_sec = float(
            self.get_parameter("readback_poll_sec").value
        )
        if self.readback_poll_sec <= 0.0:
            raise ValueError("readback_poll_sec must be > 0.0")
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
        self.readback_client = None
        if not self.dry_run:
            if SetToolDigitalOutput is None or GetToolDigitalOutput is None:
                self.get_logger().error(
                    "Doosan Tool DO set/get services are unavailable; "
                    "real Tool I/O mode cannot call Doosan services."
                )
            else:
                self.client = self.create_client(
                    SetToolDigitalOutput,
                    self.set_tool_do_service,
                )
                self.readback_client = self.create_client(
                    GetToolDigitalOutput,
                    self.get_tool_do_service,
                )

        self.command_sub = self.create_subscription(
            String,
            self.command_topic,
            self._on_command,
            10,
        )

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.accepted_command_pub = self.create_publisher(
            String, str(self.get_parameter("accepted_command_topic").value), state_qos
        )
        self.completed_command_pub = self.create_publisher(
            String,
            str(self.get_parameter("completed_command_topic").value),
            state_qos,
        )
        self.commanded_state_pub = self.create_publisher(
            Float64, str(self.get_parameter("commanded_state_topic").value), state_qos
        )
        self.driver_busy_pub = self.create_publisher(
            Bool, str(self.get_parameter("driver_busy_topic").value), state_qos
        )
        self.last_command_ok_pub = self.create_publisher(
            Bool, str(self.get_parameter("last_command_ok_topic").value), state_qos
        )
        self.diagnostics = GripperDriverDiagnostics()

        self._last_command: str | None = None
        self._current_command: str | None = None
        self._current_is_failsafe = False
        self._plan_steps: deque[ToolDoStep] = deque()
        self._pending_command = PendingGripperCommand()
        self._step_in_flight = False
        self._step_future = None
        self._step_timeout_timer = None
        self._delay_timer = None
        self._readback_poll_timer = None
        self._readback_deadline: float | None = None
        self._readback_started_time: float | None = None
        self._last_motion_command_time = 0.0
        self._publish_diagnostics()

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
            f"set_service={self.set_tool_do_service}, "
            f"get_service={self.get_tool_do_service}, "
            f"close_do_index={self.close_do_index}, "
            f"open_do_index={self.open_do_index}, "
            f"active_value={self.active_value}, "
            f"inactive_value={self.inactive_value}, "
            f"command_mode={self.command_mode}, "
            f"pulse_sec={self.pulse_sec:.3f}, "
            f"interlock_sec={self.interlock_sec:.3f}, "
            f"debounce_sec={self.debounce_sec:.3f}, "
            f"service_timeout_sec={self.service_timeout_sec:.3f}, "
            f"readback_timeout_sec={self.readback_timeout_sec:.3f}, "
            f"readback_poll_sec={self.readback_poll_sec:.3f}, "
            f"startup_all_off={self.startup_all_off}, "
            f"shutdown_all_off={self.shutdown_all_off}, "
            f"dry_run={self.dry_run}"
        )
        if self.startup_all_off:
            self._start_command("stop")

    def _declare_parameters(self) -> None:
        self.declare_parameter("command_topic", "/jrt_gripper/cmd")
        self.declare_parameter("accepted_command_topic", "/jrt_gripper/accepted_command")
        self.declare_parameter(
            "completed_command_topic",
            "/jrt_gripper/completed_command",
        )
        self.declare_parameter("commanded_state_topic", "/jrt_gripper/commanded_state")
        self.declare_parameter("driver_busy_topic", "/jrt_gripper/driver_busy")
        self.declare_parameter("last_command_ok_topic", "/jrt_gripper/last_command_ok")
        self.declare_parameter(
            "set_tool_do_service",
            "/dsr01/dsr_controller2/io/set_tool_digital_output",
        )
        self.declare_parameter(
            "get_tool_do_service",
            "/dsr01/dsr_controller2/io/get_tool_digital_output",
        )
        self.declare_parameter("close_do_index", 2)
        self.declare_parameter("open_do_index", 1)
        self.declare_parameter("active_value", 1)
        self.declare_parameter("inactive_value", 0)
        self.declare_parameter("service_timeout_sec", 1.0)
        self.declare_parameter("readback_timeout_sec", 0.5)
        self.declare_parameter("readback_poll_sec", 0.01)
        self.declare_parameter("command_mode", "pulse")
        # The timer begins after matching Tool DO readback.  Keep the physical
        # gripper command input active with enough margin to be recognized.
        self.declare_parameter("pulse_sec", 0.50)

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
            if command != "stop" and self._is_debounced_motion_command(command):
                return
            queued = self._pending_command.offer(command)
            self.get_logger().info(
                f"queued gripper command -> {queued} while Tool DO plan is active"
            )
            if command == "stop":
                self._interrupt_active_plan_for_stop()
            return

        if command != "stop" and command == self._last_command:
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

    def _interrupt_active_plan_for_stop(self) -> None:
        self._plan_steps.clear()
        self._cancel_delay_timer()
        self._cancel_readback_poll_timer()
        if self._step_in_flight:
            self.get_logger().warning(
                "stop requested; waiting only for the in-flight Tool DO response "
                "or response deadline before forcing both outputs OFF"
            )
            return
        self._abort_current_command("preempted by stop request")
        self._start_pending_command()

    def _abort_current_command(self, reason: str) -> None:
        aborted = self._current_command
        self._plan_steps.clear()
        self._cancel_delay_timer()
        self._clear_readback_state()
        self._current_command = None
        self._current_is_failsafe = False
        if aborted is None:
            return
        self.diagnostics.fail()
        self._publish_diagnostics()
        self.get_logger().warning(f"gripper command {aborted} aborted: {reason}")

    def _start_pending_command(self, *, failsafe: bool = False) -> None:
        pending = self._pending_command.pop()
        if pending is None:
            return
        self._start_command(
            pending,
            failsafe=failsafe and pending == "stop",
        )

    def _start_command(self, command: str, *, failsafe: bool = False) -> None:
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
            self.diagnostics.accept(safe_command)
            self._publish_diagnostics(emit_accepted=True)
            self.diagnostics.complete(
                safe_command,
                update_commanded_state=False,
                preserve_failure=failsafe,
            )
            self._publish_diagnostics(completed_command=safe_command)
            self._last_command = safe_command
            return

        if self.client is None or self.readback_client is None:
            self.get_logger().error(
                "Tool DO set/get service clients are unavailable; "
                "cannot send gripper command."
            )
            self.diagnostics.fail()
            self._publish_diagnostics()
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
            self.diagnostics.fail()
            self._publish_diagnostics()
            return
        readback_service_ready = self.readback_client.wait_for_service(
            timeout_sec=self.service_timeout_sec
        )
        if not readback_service_ready:
            self.get_logger().error(
                "Tool DO readback service unavailable after "
                f"{self.service_timeout_sec:.3f}s: "
                f"{self.get_tool_do_service}"
            )
            self.diagnostics.fail()
            self._publish_diagnostics()
            return

        self._current_command = safe_command
        self._current_is_failsafe = failsafe
        self._plan_steps = deque(steps)
        self.diagnostics.accept(safe_command)
        self._publish_diagnostics(emit_accepted=True)
        self.get_logger().info(f"applying gripper command -> {safe_command}")
        self._send_next_step()

    def _publish_diagnostics(
        self,
        *,
        emit_accepted: bool = False,
        completed_command: str | None = None,
    ) -> None:
        if emit_accepted and self.diagnostics.accepted_command is not None:
            self.accepted_command_pub.publish(String(data=self.diagnostics.accepted_command))
        if completed_command is not None:
            self.completed_command_pub.publish(String(data=completed_command))
        self.driver_busy_pub.publish(Bool(data=self.diagnostics.busy))
        self.last_command_ok_pub.publish(Bool(data=self.diagnostics.last_command_ok))
        if self.diagnostics.commanded_state is not None:
            self.commanded_state_pub.publish(Float64(data=self.diagnostics.commanded_state))

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
            completed_is_failsafe = self._current_is_failsafe
            self._current_command = None
            self._current_is_failsafe = False
            self._step_in_flight = False
            self._step_future = None
            self._cancel_step_timeout()
            self._clear_readback_state()
            if completed is not None:
                self._last_command = completed
                self.diagnostics.complete(
                    completed,
                    preserve_failure=completed_is_failsafe,
                )
                self._publish_diagnostics(
                    completed_command=(
                        completed
                        if self.diagnostics.last_command_ok
                        else None
                    )
                )
                self.get_logger().info(
                    f"gripper DO command applied -> {completed}"
                )
            self._start_pending_command()
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
        self._step_future = future
        self._step_timeout_timer = self.create_timer(
            self.service_timeout_sec,
            lambda sent_future=future, sent_step=step: self._on_step_timeout(
                sent_future,
                sent_step,
            ),
        )
        future.add_done_callback(
            lambda done_future, sent_step=step: self._on_step_response(
                done_future,
                sent_step,
            )
        )

    def _on_step_response(self, future: Any, step: ToolDoStep) -> None:
        if future is not self._step_future:
            return
        self._cancel_step_timeout()
        self._step_future = None
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

        if (
            self._pending_command.command == "stop"
            and self._current_command != "stop"
        ):
            self._abort_current_command("preempted by stop request")
            self._start_pending_command()
            return

        self._readback_started_time = time.monotonic()
        self._readback_deadline = (
            self._readback_started_time + self.readback_timeout_sec
        )
        self._request_step_readback(step)

    def _request_step_readback(self, step: ToolDoStep) -> None:
        if self.readback_client is None or self._readback_deadline is None:
            self.get_logger().error(
                "Tool DO readback state is unavailable; sending stop"
            )
            self._handle_service_failure()
            return

        remaining = self._readback_deadline - time.monotonic()
        if remaining <= 0.0:
            self._handle_readback_timeout(step)
            return

        request = GetToolDigitalOutput.Request()
        request.index = int(step.index)
        future = self.readback_client.call_async(request)
        self._step_future = future
        self._step_in_flight = True
        response_timeout = min(self.service_timeout_sec, remaining)
        self._step_timeout_timer = self.create_timer(
            response_timeout,
            lambda sent_future=future, sent_step=step: self._on_readback_timeout(
                sent_future,
                sent_step,
            ),
        )
        future.add_done_callback(
            lambda done_future, sent_step=step: self._on_readback_response(
                done_future,
                sent_step,
            )
        )

    def _on_readback_response(self, future: Any, step: ToolDoStep) -> None:
        if future is not self._step_future:
            return
        self._cancel_step_timeout()
        self._step_future = None
        self._step_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                "Tool DO readback call failed for "
                f"index={step.index}: {exc}"
            )
            self._handle_service_failure()
            return

        if not bool(response.success):
            self.get_logger().error(
                f"Tool DO readback rejected for index={step.index}; sending stop"
            )
            self._handle_service_failure()
            return

        if (
            self._pending_command.command == "stop"
            and self._current_command != "stop"
        ):
            self._abort_current_command("preempted by stop request")
            self._start_pending_command()
            return

        observed = int(response.value)
        expected = int(step.value)
        if observed == expected:
            started = self._readback_started_time
            latency = 0.0 if started is None else time.monotonic() - started
            self.get_logger().info(
                "Tool DO readback confirmed: "
                f"index={step.index}, value={expected}, latency={latency:.3f}s"
            )
            self._clear_readback_state()
            self._finish_confirmed_step(step)
            return

        deadline = self._readback_deadline
        if deadline is None or time.monotonic() >= deadline:
            self._handle_readback_timeout(step, observed=observed)
            return

        poll_delay = min(
            self.readback_poll_sec,
            max(0.001, deadline - time.monotonic()),
        )
        self._readback_poll_timer = self.create_timer(
            poll_delay,
            lambda sent_step=step: self._on_readback_poll(sent_step),
        )

    def _on_readback_poll(self, step: ToolDoStep) -> None:
        self._cancel_readback_poll_timer()
        self._request_step_readback(step)

    def _on_readback_timeout(self, future: Any, step: ToolDoStep) -> None:
        if future is not self._step_future:
            return
        self._cancel_step_timeout()
        self._step_future = None
        self._step_in_flight = False
        future.cancel()
        self._handle_readback_timeout(step)

    def _handle_readback_timeout(
        self,
        step: ToolDoStep,
        *,
        observed: int | None = None,
    ) -> None:
        observed_text = "unknown" if observed is None else str(observed)
        self.get_logger().error(
            "Tool DO readback did not reach requested value after "
            f"{self.readback_timeout_sec:.3f}s: "
            f"index={step.index}, expected={step.value}, observed={observed_text}"
        )
        self._handle_service_failure()

    def _finish_confirmed_step(self, step: ToolDoStep) -> None:
        if step.delay_after_sec > 0.0:
            self._delay_timer = self.create_timer(
                step.delay_after_sec,
                self._on_delay_elapsed,
            )
            return
        self._send_next_step()

    def _on_step_timeout(self, future: Any, step: ToolDoStep) -> None:
        if future is not self._step_future:
            return
        self._cancel_step_timeout()
        self._step_future = None
        self._step_in_flight = False
        future.cancel()
        self.get_logger().error(
            "Tool DO service response timed out after "
            f"{self.service_timeout_sec:.3f}s: "
            f"index={step.index}, value={step.value}"
        )
        self._handle_service_failure()

    def _on_delay_elapsed(self) -> None:
        self._cancel_delay_timer()
        self._send_next_step()

    def _cancel_delay_timer(self) -> None:
        timer = self._delay_timer
        self._delay_timer = None
        if timer is not None:
            timer.cancel()

    def _cancel_step_timeout(self) -> None:
        timer = self._step_timeout_timer
        self._step_timeout_timer = None
        if timer is not None:
            timer.cancel()

    def _cancel_readback_poll_timer(self) -> None:
        timer = self._readback_poll_timer
        self._readback_poll_timer = None
        if timer is not None:
            timer.cancel()

    def _clear_readback_state(self) -> None:
        self._cancel_readback_poll_timer()
        self._readback_deadline = None
        self._readback_started_time = None

    def _handle_service_failure(self) -> None:
        failed_command = self._current_command
        self._cancel_delay_timer()
        self._cancel_step_timeout()
        self._clear_readback_state()
        if self._step_future is not None:
            self._step_future.cancel()
            self._step_future = None
        self._step_in_flight = False
        self._plan_steps.clear()
        self._current_command = None
        self._current_is_failsafe = False
        self._pending_command.clear_motion()
        self.diagnostics.fail()
        self._publish_diagnostics()
        if failed_command != "stop":
            self.get_logger().error(
                "attempting failsafe stop: both Tool DO outputs OFF"
            )
            self._pending_command.offer("stop")
            self._start_pending_command(failsafe=True)
        else:
            self._pending_command.clear()

    def _is_plan_active(self) -> bool:
        return (
            self._current_command is not None
            or self._step_in_flight
            or self._step_future is not None
            or self._step_timeout_timer is not None
            or self._readback_poll_timer is not None
            or self._readback_deadline is not None
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
