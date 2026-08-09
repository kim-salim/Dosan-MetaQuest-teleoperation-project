"""Publish safe position-only posx targets as gated Doosan ServolRtStream messages."""

from __future__ import annotations

import csv
from collections import deque
import json
import math
from pathlib import Path
import threading
import time
from typing import Iterable, Optional, Sequence

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from quest_a0509_teleop.doosan_orientation import step_doosan_zyz_toward_deg
from quest_a0509_teleop.streamer_watchdog import (
    MuxHeartbeatWatchdog,
    RobotStateWatchdog,
)

try:
    from dsr_msgs2.msg import ServolRtStream
    from dsr_msgs2.srv import GetCurrentPosx, ReadDataRt
except Exception:  # pragma: no cover - depends on the Doosan workspace.
    ServolRtStream = None
    GetCurrentPosx = None
    ReadDataRt = None


DR_BASE = 0
DR_COND_NONE = -10000.0


def _servol_motion_conditions(use_auto: bool) -> tuple[list[float], list[float]]:
    """Return Doosan ServoL endpoint velocity/acceleration conditions."""
    value = DR_COND_NONE if use_auto else 0.0
    return [value] * 6, [value] * 6


_TIMING_TRACE_HEADER = (
    "session_wall_start_ns",
    "tick_sequence",
    "live_generation",
    "tick_entry_wall_ns",
    "tick_entry_mono_ns",
    "after_checks_mono_ns",
    "after_ramp_mono_ns",
    "before_robot_publish_mono_ns",
    "after_robot_publish_mono_ns",
    "after_commanded_publish_mono_ns",
    "after_status_mono_ns",
    "tick_end_mono_ns",
    "safe_sequence",
    "safe_receive_mono_ns",
    "robot_state",
    "command_x_mm",
    "command_y_mm",
    "command_z_mm",
    "command_rx_deg",
    "command_ry_deg",
    "command_rz_deg",
    "safe_x_mm",
    "safe_y_mm",
    "safe_z_mm",
    "safe_rx_deg",
    "safe_ry_deg",
    "safe_rz_deg",
)


def _write_timing_trace_csv(
    output_path: Path,
    rows: Sequence[Sequence[object]],
) -> None:
    """Write a completed timing session outside the real-time-ish control callback."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_suffix(output_path.suffix + ".tmp")
    with temporary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(_TIMING_TRACE_HEADER)
        writer.writerows(rows)
    temporary_path.replace(output_path)


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


def _robot_state_code(values: Iterable[float]) -> int:
    data = [float(value) for value in values]
    if not data:
        raise ValueError("robot_state topic data must contain at least one value")
    value = data[0]
    if not math.isfinite(value):
        raise ValueError(f"robot_state topic value must be finite: {value}")
    state = int(round(value))
    if abs(value - state) > 1.0e-6:
        raise ValueError(f"robot_state topic value must be an integer code: {value}")
    return state


class ServolRtStreamerNode(Node):
    def __init__(self) -> None:
        super().__init__("servol_rt_streamer_node")
        self._declare_parameters()

        self.safe_posx_topic = self.get_parameter("safe_posx_topic").value
        self.doosan_servol_topic = self.get_parameter("doosan_servol_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.teleop_ready_topic = self.get_parameter("teleop_ready_topic").value
        self.commanded_posx_topic = self.get_parameter("commanded_posx_topic").value
        self.live_state_topic = self.get_parameter("live_state_topic").value
        self.selected_heartbeat_topic = self.get_parameter(
            "selected_heartbeat_topic"
        ).value
        self.robot_namespace = str(self.get_parameter("robot_namespace").value).strip("/")
        self.controller_name = str(self.get_parameter("controller_name").value).strip("/")
        if not self.robot_namespace or not self.controller_name:
            raise ValueError("robot_namespace and controller_name must not be empty")
        self.controller_prefix = f"/{self.robot_namespace}/{self.controller_name}"
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.live_enabled = bool(self.get_parameter("live_enabled").value)
        self.require_live_enable = bool(self.get_parameter("require_live_enable").value)
        self.require_prepare_before_live = bool(
            self.get_parameter("require_prepare_before_live").value
        )
        self.require_robot_ready = bool(self.get_parameter("require_robot_ready").value)
        self.robot_state_topic = self.get_parameter("robot_state_topic").value
        self.robot_state_stale_timeout_sec = float(
            self.get_parameter("robot_state_stale_timeout_sec").value
        )
        self.safe_robot_states = _int_list(
            self.get_parameter("safe_robot_states").value,
            "safe_robot_states",
        )
        self.servol_time_sec = float(self.get_parameter("servol_time_sec").value)
        self.servol_use_auto_velocity_acceleration = bool(
            self.get_parameter("servol_use_auto_velocity_acceleration").value
        )
        self.servol_motion_condition = (
            DR_COND_NONE if self.servol_use_auto_velocity_acceleration else 0.0
        )
        self.stream_ramp_linear_mm_per_tick = float(
            self.get_parameter("stream_ramp_linear_mm_per_tick").value
        )
        self.stream_ramp_rot_deg_per_tick = float(
            self.get_parameter("stream_ramp_rot_deg_per_tick").value
        )
        self.enable_timing_trace = bool(
            self.get_parameter("enable_timing_trace").value
        )
        self.timing_trace_dir = Path(
            str(self.get_parameter("timing_trace_dir").value)
        ).expanduser()
        self.timing_trace_max_samples = int(
            self.get_parameter("timing_trace_max_samples").value
        )
        if self.timing_trace_max_samples <= 0:
            raise ValueError("timing_trace_max_samples must be > 0")
        # ServoL streaming, control services, and robot-state delivery must never
        # occupy one another's callback group.
        self.state_callback_group = ReentrantCallbackGroup()
        self.control_service_callback_group = MutuallyExclusiveCallbackGroup()
        self.robot_client_callback_group = ReentrantCallbackGroup()
        self.robot_state_callback_group = MutuallyExclusiveCallbackGroup()
        self.stream_timer_callback_group = MutuallyExclusiveCallbackGroup()
        self._live_state_lock = threading.RLock()
        self._robot_state_lock = threading.RLock()
        self._live_generation = 0
        self._safe_sequence = 0
        self._safe_receive_mono_ns = 0
        self._timing_session_active = False
        self._timing_session_wall_start_ns = 0
        self._timing_tick_sequence = 0
        self._timing_records: deque[tuple[object, ...]] = deque(
            maxlen=self.timing_trace_max_samples
        )
        self._last_timing_trace_path: Optional[Path] = None

        self.latest_safe: Optional[list[float]] = None
        self.current_command: Optional[list[float]] = None
        self.last_log_time = 0.0
        self.teleop_ready = False
        self.mux_watchdog = MuxHeartbeatWatchdog(
            required=bool(self.get_parameter("require_mux_heartbeat").value),
            timeout_sec=float(self.get_parameter("mux_heartbeat_timeout_sec").value),
        )
        self.robot_state_watchdog = RobotStateWatchdog(
            required=self.require_robot_ready,
            safe_states=tuple(self.safe_robot_states),
            timeout_sec=self.robot_state_stale_timeout_sec,
        )
        self._last_robot_state_warning_time = 0.0

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.commanded_posx_pub = self.create_publisher(Float64MultiArray, self.commanded_posx_topic, 10)
        self.live_state_pub = self.create_publisher(Bool, self.live_state_topic, state_qos)
        self.safe_sub = self.create_subscription(
            Float64MultiArray,
            self.safe_posx_topic,
            self._on_safe_posx,
            10,
            callback_group=self.state_callback_group,
        )
        self.teleop_ready_sub = self.create_subscription(
            Bool,
            self.teleop_ready_topic,
            self._on_teleop_ready,
            state_qos,
            callback_group=self.state_callback_group,
        )
        self.mux_heartbeat_sub = self.create_subscription(
            Empty,
            self.selected_heartbeat_topic,
            self._on_mux_heartbeat,
            10,
            callback_group=self.state_callback_group,
        )
        self.robot_state_sub = None
        if self.require_robot_ready:
            self.robot_state_sub = self.create_subscription(
                Float64MultiArray,
                self.robot_state_topic,
                self._on_robot_state,
                state_qos,
                callback_group=self.robot_state_callback_group,
            )
        self.set_live_srv = self.create_service(
            SetBool,
            "/vr/set_live_robot_output",
            self._on_set_live_robot_output,
            callback_group=self.control_service_callback_group,
        )
        self.hold_srv = self.create_service(
            Trigger,
            "/vr/hold_servol",
            self._on_hold_servol,
            callback_group=self.control_service_callback_group,
        )

        self.robot_pub = None
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
            if ReadDataRt is not None:
                self.read_data_rt_client = self.create_client(
                    ReadDataRt,
                    f"{self.controller_prefix}/realtime/read_data_rt",
                    callback_group=self.robot_client_callback_group,
                )
            if GetCurrentPosx is not None:
                self.get_current_posx_client = self.create_client(
                    GetCurrentPosx,
                    f"{self.controller_prefix}/aux_control/get_current_posx",
                    callback_group=self.robot_client_callback_group,
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
                f"robot_state_source=topic, robot_state_topic={self.robot_state_topic}, "
                f"servol_velocity_acceleration_condition={self.servol_motion_condition}, "
                "robot_state_qos=KEEP_LAST(depth=1, RELIABLE, TRANSIENT_LOCAL), "
                f"robot_state_stale_timeout_sec={self.robot_state_stale_timeout_sec}, "
                f"timing_trace_enabled={self.enable_timing_trace}, "
                f"timing_trace_dir={self.timing_trace_dir}, "
                "call /vr/set_live_robot_output true to publish"
            )

        self._publish_live_state()
        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(
            period,
            self._tick,
            callback_group=self.stream_timer_callback_group,
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("safe_posx_topic", "/vr/safe_posx")
        self.declare_parameter(
            "doosan_servol_topic",
            "/dsr01/dsr_controller2/servol_rt_stream",
        )
        self.declare_parameter("status_topic", "/vr/status")
        self.declare_parameter("teleop_ready_topic", "/vr/teleop_ready")
        self.declare_parameter("commanded_posx_topic", "/vr/commanded_posx")
        self.declare_parameter("live_state_topic", "/vr/live_robot_output_enabled")
        self.declare_parameter("selected_heartbeat_topic", "/control/selected_command_heartbeat")
        self.declare_parameter("robot_namespace", "/dsr01")
        self.declare_parameter("controller_name", "dsr_controller2")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("live_enabled", False)
        self.declare_parameter("require_live_enable", True)
        self.declare_parameter("require_prepare_before_live", True)
        self.declare_parameter("require_robot_ready", True)
        self.declare_parameter("safe_robot_states", [1, 2])
        self.declare_parameter("robot_state_topic", "/rt_topic/robot_state")
        self.declare_parameter("robot_state_stale_timeout_sec", 1.0)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("executor_num_threads", 4)
        self.declare_parameter("executor_yield_sec", 0.001)
        self.declare_parameter("servol_time_sec", 0.1)
        self.declare_parameter("servol_use_auto_velocity_acceleration", False)
        self.declare_parameter("stream_ramp_linear_mm_per_tick", 6.67)
        self.declare_parameter("stream_ramp_rot_deg_per_tick", 1.0)
        self.declare_parameter("require_mux_heartbeat", False)
        self.declare_parameter("mux_heartbeat_timeout_sec", 1.0)
        self.declare_parameter("enable_timing_trace", False)
        self.declare_parameter("timing_trace_dir", "/tmp/teleop_diagnostics")
        self.declare_parameter("timing_trace_max_samples", 12000)

    def _on_safe_posx(self, msg: Float64MultiArray) -> None:
        try:
            self.latest_safe = _posx(msg.data, "safe_posx")
            self._safe_sequence += 1
            self._safe_receive_mono_ns = time.monotonic_ns()
        except ValueError as exc:
            self._publish_status(f"ignored invalid safe_posx: {exc}", warn=True)

    def _on_teleop_ready(self, msg: Bool) -> None:
        ready = bool(msg.data)
        if ready != self.teleop_ready:
            self.teleop_ready = ready
            self._publish_status(f"teleop_ready={self.teleop_ready}")
        if self.require_prepare_before_live and not self.teleop_ready and self.live_enabled:
            self._disable_live_robot_output("Live ServoL RT disabled because teleop_ready=false.")

    def _on_mux_heartbeat(self, _msg: Empty) -> None:
        self.mux_watchdog.mark(time.monotonic())

    def _on_robot_state(self, msg: Float64MultiArray) -> None:
        try:
            state = _robot_state_code(msg.data)
        except ValueError as exc:
            self._warn_robot_state(f"ignored invalid robot-state topic sample: {exc}")
            return
        now = time.monotonic()
        with self._robot_state_lock:
            previous_state = self.robot_state_watchdog.last_state
            self.robot_state_watchdog.mark(state, now)
        if previous_state is None:
            self._publish_status(f"robot-state topic cache updated: state={state}")
        elif state != previous_state and (
            state not in self.safe_robot_states
            or previous_state not in self.safe_robot_states
        ):
            self._publish_status(
                f"robot-state safety boundary changed: {previous_state}->{state}",
                warn=state not in self.safe_robot_states,
            )

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
            self._mark_live_disabled()
            response.success = False
            response.message = f"Failed to set live robot output: {exc}"
            self._publish_status(response.message, warn=True)
        return response

    def _on_hold_servol(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            # A hold request is a safety action. Drop the live gate before any
            # potentially blocking robot-state lookup so no stream tick can
            # overwrite the hold while that lookup is in flight.
            self._mark_live_disabled()
            self._publish_hold_best_effort()
            response.success = True
            response.message = "Live output disabled and hold command published best-effort."
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
        heartbeat_error = self.mux_watchdog.enable_reject_reason(time.monotonic())
        if heartbeat_error is not None:
            raise RuntimeError(heartbeat_error)
        if self.require_prepare_before_live and not self.teleop_ready:
            raise RuntimeError(
                f"Prepare Robot must complete before live output; {self.teleop_ready_topic} is false."
            )
        if self.latest_safe is None:
            raise RuntimeError("No /vr/safe_posx target received yet.")
        robot_state_error = self._robot_state_reject_reason(time.monotonic())
        if robot_state_error is not None:
            raise RuntimeError(robot_state_error)
        actual = self._read_actual_posx_best_effort()
        heartbeat_error = self.mux_watchdog.enable_reject_reason(time.monotonic())
        if heartbeat_error is not None:
            raise RuntimeError(heartbeat_error)
        if self.require_prepare_before_live and not self.teleop_ready:
            raise RuntimeError(
                f"Prepare Robot became invalid during live enable; "
                f"{self.teleop_ready_topic} is false."
            )
        robot_state_error = self._robot_state_reject_reason(time.monotonic())
        if robot_state_error is not None:
            raise RuntimeError(robot_state_error)
        with self._live_state_lock:
            if actual is not None:
                self.current_command = actual[:]
                self._publish_robot_command(actual)
            else:
                self.current_command = self.latest_safe[:]
            self._start_timing_session()
            self.live_enabled = True
            self._live_generation += 1
            self._publish_live_state()
        self._publish_status(
            "Live robot output enabled: "
            + json.dumps(
                {
                    "start_command": self.current_command,
                    "requested_safe": self.latest_safe,
                    "ramp_linear_mm_per_tick": self.stream_ramp_linear_mm_per_tick,
                    "ramp_rot_deg_per_tick": self.stream_ramp_rot_deg_per_tick,
                    "servol_velocity_acceleration_condition": (
                        self.servol_motion_condition
                    ),
                },
                sort_keys=True,
            )
        )

    def _disable_live_robot_output(self, reason: str) -> None:
        was_live = self._mark_live_disabled()
        self._publish_hold_best_effort()
        if was_live or reason:
            self._publish_status(reason)

    def _mark_live_disabled(self) -> bool:
        with self._live_state_lock:
            was_live = self.live_enabled
            self.live_enabled = False
            self._live_generation += 1
            self._publish_live_state()
            if was_live:
                self._finish_timing_session()
            return was_live

    def _tick(self) -> None:
        trace_tick = self.enable_timing_trace and self.live_enabled
        tick_entry_mono_ns = time.monotonic_ns() if trace_tick else 0
        tick_entry_wall_ns = time.time_ns() if trace_tick else 0
        trace_safe_sequence = self._safe_sequence if trace_tick else 0
        trace_safe_receive_mono_ns = self._safe_receive_mono_ns if trace_tick else 0
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

        if self.mux_watchdog.should_disable_live(self.live_enabled, now):
            reason = self.mux_watchdog.enable_reject_reason(now)
            self._disable_live_robot_output(
                f"Live ServoL RT stopped: {reason}."
            )
            return

        if self.require_prepare_before_live and not self.teleop_ready:
            self._disable_live_robot_output(
                "Live ServoL RT stopped: teleop_ready=false. Run Prepare Robot first."
            )
            return

        robot_state_error = self._robot_state_reject_reason(now)
        if robot_state_error is not None:
            self._disable_live_robot_output(
                f"Live ServoL RT stopped: {robot_state_error}."
            )
            return

        with self._live_state_lock:
            tick_generation = self._live_generation

        try:
            with self._live_state_lock:
                if (
                    self.require_live_enable
                    and (
                        not self.live_enabled
                        or tick_generation != self._live_generation
                    )
                ):
                    return
                after_checks_mono_ns = time.monotonic_ns()
                safe_target = self.latest_safe[:]
                target = self._ramp_command(safe_target)
                after_ramp_mono_ns = time.monotonic_ns()
                publish_timing = self._publish_robot_command(
                    target,
                    capture_timing=self.enable_timing_trace,
                )
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
                        "robot_state": self._last_robot_state(),
                    },
                    sort_keys=True,
                )
            )
            self.last_log_time = now
        after_status_mono_ns = time.monotonic_ns()

        if (
            self.enable_timing_trace
            and self._timing_session_active
            and publish_timing is not None
        ):
            before_robot_publish_ns, after_robot_publish_ns, after_commanded_publish_ns = (
                publish_timing
            )
            self._timing_tick_sequence += 1
            self._timing_records.append(
                (
                    self._timing_session_wall_start_ns,
                    self._timing_tick_sequence,
                    tick_generation,
                    tick_entry_wall_ns,
                    tick_entry_mono_ns,
                    after_checks_mono_ns,
                    after_ramp_mono_ns,
                    before_robot_publish_ns,
                    after_robot_publish_ns,
                    after_commanded_publish_ns,
                    after_status_mono_ns,
                    time.monotonic_ns(),
                    trace_safe_sequence,
                    trace_safe_receive_mono_ns,
                    self._last_robot_state(),
                    *target,
                    *safe_target,
                )
            )

    def _robot_state_reject_reason(self, now: float) -> Optional[str]:
        with self._robot_state_lock:
            return self.robot_state_watchdog.enable_reject_reason(now)

    def _last_robot_state(self) -> Optional[int]:
        with self._robot_state_lock:
            return self.robot_state_watchdog.last_state

    def _warn_robot_state(self, text: str) -> None:
        now = time.monotonic()
        if now - self._last_robot_state_warning_time < 1.0:
            return
        self._last_robot_state_warning_time = now
        self._publish_status(text, warn=True)

    def _ramp_command(self, requested: list[float]) -> list[float]:
        if self.current_command is None:
            self.current_command = requested[:]
            return requested[:]
        command = self.current_command[:]
        for index in range(3):
            delta = requested[index] - command[index]
            if abs(delta) <= self.stream_ramp_linear_mm_per_tick:
                command[index] = requested[index]
            else:
                command[index] += math.copysign(
                    self.stream_ramp_linear_mm_per_tick,
                    delta,
                )
        orientation_zyz, _, _ = step_doosan_zyz_toward_deg(
            command[3:6],
            requested[3:6],
            self.stream_ramp_rot_deg_per_tick,
        )
        command[3:6] = orientation_zyz
        self.current_command = command[:]
        return command

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
            self._publish_robot_command(hold)
            time.sleep(0.01)
        self.current_command = hold[:]

    def _publish_robot_command(
        self,
        posx: list[float],
        *,
        capture_timing: bool = False,
    ) -> Optional[tuple[int, int, int]]:
        if self.robot_pub is None:
            return None
        before_robot_publish_ns = time.monotonic_ns() if capture_timing else 0
        self.robot_pub.publish(self._make_servol_msg(posx))
        after_robot_publish_ns = time.monotonic_ns() if capture_timing else 0
        message = Float64MultiArray()
        message.data = [float(value) for value in posx]
        self.commanded_posx_pub.publish(message)
        if not capture_timing:
            return None
        after_commanded_publish_ns = time.monotonic_ns()
        return (
            before_robot_publish_ns,
            after_robot_publish_ns,
            after_commanded_publish_ns,
        )

    def _publish_live_state(self) -> None:
        self.live_state_pub.publish(Bool(data=bool(self.live_enabled)))

    def _start_timing_session(self) -> None:
        if not self.enable_timing_trace:
            return
        self._timing_records.clear()
        self._timing_tick_sequence = 0
        self._timing_session_wall_start_ns = time.time_ns()
        self._timing_session_active = True

    def _finish_timing_session(self) -> None:
        if not self.enable_timing_trace or not self._timing_session_active:
            return
        self._timing_session_active = False
        rows = tuple(self._timing_records)
        output_path = self.timing_trace_dir / (
            f"streamer_timing_{self._timing_session_wall_start_ns}.csv"
        )
        self._last_timing_trace_path = output_path
        writer = threading.Thread(
            target=_write_timing_trace_csv,
            args=(output_path, rows),
            name="streamer_timing_writer",
            daemon=True,
        )
        writer.start()
        self._publish_status(
            f"Streamer timing trace scheduled: path={output_path}, rows={len(rows)}"
        )

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
        msg.vel, msg.acc = _servol_motion_conditions(
            self.servol_use_auto_velocity_acceleration
        )
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
    executor_num_threads = int(node.get_parameter("executor_num_threads").value)
    executor_yield_sec = float(node.get_parameter("executor_yield_sec").value)
    if executor_num_threads <= 0:
        raise ValueError("executor_num_threads must be > 0")
    if executor_yield_sec < 0.0:
        raise ValueError("executor_yield_sec must be >= 0")
    executor = MultiThreadedExecutor(num_threads=executor_num_threads)
    executor.add_node(node)
    node.get_logger().info(
        "Streamer executor configured: "
        f"num_threads={executor_num_threads}, yield_sec={executor_yield_sec}"
    )
    try:
        # With continuously-ready subscriptions, MultiThreadedExecutor.spin()
        # can keep the Python dispatcher thread runnable and starve worker
        # callbacks on the GIL. A short bounded yield keeps timer and service
        # callbacks responsive while retaining separate callback groups.
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            if executor_yield_sec > 0.0:
                time.sleep(executor_yield_sec)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
