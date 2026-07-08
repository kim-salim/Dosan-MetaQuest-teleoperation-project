"""Publish safe position-only posx targets as gated Doosan ServolRtStream messages."""

from __future__ import annotations

import json
import math
import time
from typing import Iterable, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

try:
    from dsr_msgs2.msg import ServolRtStream
    from dsr_msgs2.srv import GetCurrentPosx, GetRobotState, ReadDataRt
except Exception:  # pragma: no cover - depends on the Doosan workspace.
    ServolRtStream = None
    GetCurrentPosx = None
    GetRobotState = None
    ReadDataRt = None


DR_BASE = 0


def _posx(values: Iterable[float], name: str) -> list[float]:
    output = [float(value) for value in values]
    if len(output) != 6:
        raise ValueError(f"{name} must contain exactly 6 values")
    if any(not math.isfinite(value) for value in output):
        raise ValueError(f"{name} contains non-finite values: {output}")
    return output


def _int_list(values: Iterable[int], name: str) -> list[int]:
    output = [int(value) for value in values]
    if not output:
        raise ValueError(f"{name} must not be empty")
    return output


def _shortest_angle_delta_deg(target: float, current: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


class ServolRtStreamerNode(Node):
    def __init__(self) -> None:
        super().__init__("servol_rt_streamer_node")
        self._declare_parameters()

        self.safe_posx_topic = self.get_parameter("safe_posx_topic").value
        self.doosan_servol_topic = self.get_parameter("doosan_servol_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.teleop_ready_topic = self.get_parameter("teleop_ready_topic").value
        self.robot_namespace = str(self.get_parameter("robot_namespace").value).strip("/")
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.live_enabled = bool(self.get_parameter("live_enabled").value)
        self.require_live_enable = bool(self.get_parameter("require_live_enable").value)
        self.require_prepare_before_live = bool(
            self.get_parameter("require_prepare_before_live").value
        )
        self.require_robot_ready = bool(self.get_parameter("require_robot_ready").value)
        self.robot_state_check_interval_sec = float(
            self.get_parameter("robot_state_check_interval_sec").value
        )
        self.safe_robot_states = _int_list(
            self.get_parameter("safe_robot_states").value,
            "safe_robot_states",
        )
        self.servol_time_sec = float(self.get_parameter("servol_time_sec").value)
        self.stream_ramp_linear_mm_per_tick = float(
            self.get_parameter("stream_ramp_linear_mm_per_tick").value
        )
        self.stream_ramp_rot_deg_per_tick = float(
            self.get_parameter("stream_ramp_rot_deg_per_tick").value
        )
        self.callback_group = ReentrantCallbackGroup()

        self.latest_safe: Optional[list[float]] = None
        self.current_command: Optional[list[float]] = None
        self.last_log_time = 0.0
        self.last_robot_state_check_time = 0.0
        self.last_robot_state: Optional[int] = None
        self.teleop_ready = False

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.safe_sub = self.create_subscription(
            Float64MultiArray,
            self.safe_posx_topic,
            self._on_safe_posx,
            10,
            callback_group=self.callback_group,
        )
        self.teleop_ready_sub = self.create_subscription(
            Bool,
            self.teleop_ready_topic,
            self._on_teleop_ready,
            10,
            callback_group=self.callback_group,
        )
        self.set_live_srv = self.create_service(
            SetBool,
            "/vr/set_live_robot_output",
            self._on_set_live_robot_output,
            callback_group=self.callback_group,
        )
        self.hold_srv = self.create_service(
            Trigger,
            "/vr/hold_servol",
            self._on_hold_servol,
            callback_group=self.callback_group,
        )

        self.robot_pub = None
        self.get_robot_state_client = None
        self.read_data_rt_client = None
        self.get_current_posx_client = None
        if self.dry_run:
            self.live_enabled = False
            self._publish_status(
                "servol_rt_streamer_node started in dry_run mode; "
                f"not publishing to {self.doosan_servol_topic}"
            )
            if ServolRtStream is None:
                self._publish_status(
                    "dsr_msgs2.msg.ServolRtStream is not available; dry_run mode will continue.",
                    warn=True,
                )
        elif ServolRtStream is None:
            self.live_enabled = False
            self._publish_status(
                "dsr_msgs2.msg.ServolRtStream is not available; robot output disabled.",
                warn=True,
            )
        else:
            self.robot_pub = self.create_publisher(ServolRtStream, self.doosan_servol_topic, 10)
            if GetRobotState is not None:
                self.get_robot_state_client = self.create_client(
                    GetRobotState,
                    f"/{self.robot_namespace}/system/get_robot_state",
                    callback_group=self.callback_group,
                )
            if ReadDataRt is not None:
                self.read_data_rt_client = self.create_client(
                    ReadDataRt,
                    f"/{self.robot_namespace}/realtime/read_data_rt",
                    callback_group=self.callback_group,
                )
            if GetCurrentPosx is not None:
                self.get_current_posx_client = self.create_client(
                    GetCurrentPosx,
                    f"/{self.robot_namespace}/aux_control/get_current_posx",
                    callback_group=self.callback_group,
                )
            if self.require_live_enable and self.live_enabled:
                self._publish_status(
                    "live_enabled parameter was true but runtime gate is required; "
                    "call /vr/set_live_robot_output to enable real output.",
                    warn=True,
                )
                self.live_enabled = False
            self._publish_status(
                "servol_rt_streamer_node armed for real robot output; "
                f"topic={self.doosan_servol_topic}, live_enabled={self.live_enabled}, "
                "call /vr/set_live_robot_output true to publish"
            )

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self._tick, callback_group=self.callback_group)

    def _declare_parameters(self) -> None:
        self.declare_parameter("safe_posx_topic", "/vr/safe_posx")
        self.declare_parameter("doosan_servol_topic", "/dsr01/servol_rt_stream")
        self.declare_parameter("status_topic", "/vr/status")
        self.declare_parameter("teleop_ready_topic", "/vr/teleop_ready")
        self.declare_parameter("robot_namespace", "/dsr01")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("live_enabled", False)
        self.declare_parameter("require_live_enable", True)
        self.declare_parameter("require_prepare_before_live", True)
        self.declare_parameter("require_robot_ready", True)
        self.declare_parameter("safe_robot_states", [1, 2])
        self.declare_parameter("robot_state_check_interval_sec", 1.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("servol_time_sec", 0.5)
        self.declare_parameter("stream_ramp_linear_mm_per_tick", 7.5)
        self.declare_parameter("stream_ramp_rot_deg_per_tick", 3.0)

    def _on_safe_posx(self, msg: Float64MultiArray) -> None:
        try:
            self.latest_safe = _posx(msg.data, "safe_posx")
        except ValueError as exc:
            self._publish_status(f"ignored invalid safe_posx: {exc}", warn=True)

    def _on_teleop_ready(self, msg: Bool) -> None:
        ready = bool(msg.data)
        if ready != self.teleop_ready:
            self.teleop_ready = ready
            self._publish_status(f"teleop_ready={self.teleop_ready}")
        if self.require_prepare_before_live and not self.teleop_ready and self.live_enabled:
            self._disable_live_robot_output("Live ServoL RT disabled because teleop_ready=false.")

    def _on_set_live_robot_output(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        try:
            if bool(request.data):
                self._enable_live_robot_output()
                response.success = True
                response.message = "Live robot output enabled."
            else:
                self._disable_live_robot_output("Live robot output disabled by service request.")
                response.success = True
                response.message = "Live robot output disabled."
        except Exception as exc:
            self.live_enabled = False
            response.success = False
            response.message = f"Failed to set live robot output: {exc}"
            self._publish_status(response.message, warn=True)
        return response

    def _on_hold_servol(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            self._publish_hold_best_effort()
            response.success = True
            response.message = "Hold command published best-effort."
        except Exception as exc:
            response.success = False
            response.message = f"Hold failed: {exc}"
            self._publish_status(response.message, warn=True)
        return response

    def _enable_live_robot_output(self) -> None:
        if self.dry_run:
            raise RuntimeError("dry_run is true; restart launch with dry_run:=false first.")
        if self.robot_pub is None or ServolRtStream is None:
            raise RuntimeError("Doosan ServolRtStream publisher is not available.")
        if self.require_prepare_before_live and not self.teleop_ready:
            raise RuntimeError(
                f"Prepare Robot must complete before live output; {self.teleop_ready_topic} is false."
            )
        if self.latest_safe is None:
            raise RuntimeError("No /vr/safe_posx target received yet.")
        if self.require_robot_ready:
            robot_state = self._read_robot_state()
            if robot_state not in self.safe_robot_states:
                raise RuntimeError(f"robot_state={robot_state} is not ready for ServoL RT.")
        actual = self._read_actual_posx_best_effort()
        if actual is not None:
            self.current_command = actual[:]
            self.robot_pub.publish(self._make_servol_msg(actual))
        else:
            self.current_command = self.latest_safe[:]
        self.last_robot_state_check_time = 0.0
        self.live_enabled = True
        self._publish_status(
            "Live robot output enabled: "
            + json.dumps(
                {
                    "start_command": self.current_command,
                    "requested_safe": self.latest_safe,
                    "ramp_linear_mm_per_tick": self.stream_ramp_linear_mm_per_tick,
                    "ramp_rot_deg_per_tick": self.stream_ramp_rot_deg_per_tick,
                },
                sort_keys=True,
            )
        )

    def _disable_live_robot_output(self, reason: str) -> None:
        was_live = self.live_enabled
        self.live_enabled = False
        self._publish_hold_best_effort()
        if was_live or reason:
            self._publish_status(reason)

    def _tick(self) -> None:
        if self.latest_safe is None:
            return

        now = time.monotonic()
        if self.dry_run or self.robot_pub is None:
            if now - self.last_log_time >= 1.0:
                self._publish_status(
                    "dry_run safe_posx target="
                    + json.dumps(
                        {"data": self.latest_safe, "servol_time_sec": self.servol_time_sec},
                        sort_keys=True,
                    )
                )
                self.last_log_time = now
            return

        if self.require_live_enable and not self.live_enabled:
            if now - self.last_log_time >= 1.0:
                self._publish_status(
                    "robot output armed but live gate is disabled; "
                    "call /vr/set_live_robot_output true after Prepare Robot completes."
                )
                self.last_log_time = now
            return

        if self.require_prepare_before_live and not self.teleop_ready:
            self._disable_live_robot_output(
                "Live ServoL RT stopped: teleop_ready=false. Run Prepare Robot first."
            )
            return

        try:
            self._maybe_check_robot_ready(now)
            target = self._ramp_command(self.latest_safe)
            self.robot_pub.publish(self._make_servol_msg(target))
        except Exception as exc:
            self._disable_live_robot_output(f"Live ServoL RT stopped: {exc}")
            return

        if now - self.last_log_time >= 1.0:
            self._publish_status(
                "published ServolRtStream target="
                + json.dumps(
                    {
                        "pos": target,
                        "requested_safe": self.latest_safe,
                        "time": self.servol_time_sec,
                        "robot_state": self.last_robot_state,
                    },
                    sort_keys=True,
                )
            )
            self.last_log_time = now

    def _maybe_check_robot_ready(self, now: float) -> None:
        if not self.require_robot_ready:
            return
        if now - self.last_robot_state_check_time < self.robot_state_check_interval_sec:
            return
        robot_state = self._read_robot_state()
        self.last_robot_state = robot_state
        self.last_robot_state_check_time = now
        if robot_state not in self.safe_robot_states:
            raise RuntimeError(f"robot_state={robot_state} is not ready for ServoL RT.")

    def _ramp_command(self, requested: list[float]) -> list[float]:
        if self.current_command is None:
            self.current_command = requested[:]
            return requested[:]
        command = self.current_command[:]
        for index, target in enumerate(requested):
            max_step = (
                self.stream_ramp_linear_mm_per_tick
                if index < 3
                else self.stream_ramp_rot_deg_per_tick
            )
            delta = (
                target - command[index]
                if index < 3
                else _shortest_angle_delta_deg(target, command[index])
            )
            if abs(delta) <= max_step:
                command[index] = target if index < 3 else command[index] + delta
            else:
                command[index] += math.copysign(max_step, delta)
        self.current_command = command[:]
        return command

    def _read_robot_state(self) -> int:
        if self.get_robot_state_client is None or GetRobotState is None:
            raise RuntimeError("GetRobotState service client is not available.")
        response = self._call_service(
            self.get_robot_state_client,
            GetRobotState.Request(),
            timeout_sec=2.0,
        )
        if not response.success:
            raise RuntimeError("GetRobotState returned success=false")
        return int(response.robot_state)

    def _read_actual_posx_best_effort(self) -> Optional[list[float]]:
        if self.read_data_rt_client is not None and ReadDataRt is not None:
            try:
                response = self._call_service(
                    self.read_data_rt_client,
                    ReadDataRt.Request(),
                    timeout_sec=1.0,
                )
                return _posx(response.data.actual_tcp_position[:6], "actual_tcp_position")
            except Exception as exc:
                self._publish_status(f"ReadDataRt actual TCP unavailable: {exc}", warn=True)
        if self.get_current_posx_client is not None and GetCurrentPosx is not None:
            request = GetCurrentPosx.Request()
            request.ref = DR_BASE
            response = self._call_service(self.get_current_posx_client, request, timeout_sec=2.0)
            if not response.success or not response.task_pos_info:
                raise RuntimeError("GetCurrentPosx returned no task position")
            return _posx(response.task_pos_info[0].data[:6], "current_posx")
        return None

    def _publish_hold_best_effort(self) -> None:
        if self.robot_pub is None:
            return
        hold = self._read_actual_posx_best_effort()
        if hold is None:
            hold = self.current_command[:] if self.current_command is not None else None
        if hold is None:
            return
        for _ in range(3):
            self.robot_pub.publish(self._make_servol_msg(hold))
            time.sleep(0.01)
        self.current_command = hold[:]

    def _call_service(self, client, request, timeout_sec: float):
        if not client.wait_for_service(timeout_sec=0.5):
            raise RuntimeError(f"service not available: {client.srv_name}")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(f"service timeout: {client.srv_name}")
            time.sleep(0.02)
        result = future.result()
        if result is None:
            raise RuntimeError(f"service failed: {client.srv_name}")
        return result

    def _make_servol_msg(self, posx: list[float]):
        if ServolRtStream is None:
            raise RuntimeError("ServolRtStream is not available")
        msg = ServolRtStream()
        msg.pos = [float(value) for value in posx]
        msg.vel = [0.0] * 6
        msg.acc = [0.0] * 6
        msg.time = float(self.servol_time_sec)
        return msg

    def _publish_status(self, text: str, warn: bool = False) -> None:
        self.status_pub.publish(String(data=text))
        if warn:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ServolRtStreamerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
