#!/usr/bin/env python3
"""Externally arm and supervise one full Task-C A -> Bridge -> B trial.

The strategy process never selects the MUX or enables Live robot output.  This
gate owns those two capabilities, verifies the running ROS parameters against
the strategy's latched downstream contract, and always forces Live OFF plus
MUX DISABLED on every exit path.  Preflight-only is the default.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import deque
from pathlib import Path
from typing import Any

import rclpy
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

try:
    from a0509_act_live_trial_gate import LiveTrialGate, pose_delta_metrics
except ModuleNotFoundError:  # Imported as ``scripts.*`` by unit tests.
    from scripts.a0509_act_live_trial_gate import LiveTrialGate, pose_delta_metrics


MOTION_AUTHORIZATION = "I_ACKNOWLEDGE_TASK_C_REAL_ROBOT_MOTION"


def _equal(expected: Any, actual: Any, *, tolerance: float = 1e-9) -> bool:
    if isinstance(expected, bool):
        return isinstance(actual, bool) and actual is expected
    if isinstance(expected, (int, float)):
        try:
            return math.isclose(
                float(actual),
                float(expected),
                rel_tol=0.0,
                abs_tol=tolerance,
            )
        except (TypeError, ValueError):
            return False
    if isinstance(expected, (list, tuple)):
        return (
            isinstance(actual, (list, tuple))
            and len(actual) == len(expected)
            and all(
                _equal(left, right, tolerance=tolerance)
                for left, right in zip(expected, actual, strict=True)
            )
        )
    return actual == expected


def contract_mismatches(
    ready_event: dict[str, Any],
    running_parameters: dict[str, Any],
    *,
    execute: bool,
) -> list[str]:
    """Return exact ROS parameter mismatches for the latched runner contract."""

    contract = ready_event.get("downstream_contract")
    if not isinstance(contract, dict):
        return ["strategy ready event has no downstream_contract"]
    orientation_limit = float(contract["orientation_limit_deg"])
    expected = {
        "guard.publish_rate_hz": contract["control_hz"],
        "guard.target_posx_topic": contract["selected_target_topic"],
        "guard.safe_posx_topic": contract["safe_posx_topic"],
        "guard.workspace_min_xyz_mm": contract["workspace_min_xyz_mm"],
        "guard.workspace_min_limit_enabled": (
            contract["workspace_min_limit_enabled"]
        ),
        "guard.workspace_max_xyz_mm": contract["workspace_max_xyz_mm"],
        "guard.enable_orientation_limits": True,
        "guard.max_orientation_delta_deg": [orientation_limit] * 3,
        "streamer.publish_rate_hz": contract["control_hz"],
        "streamer.safe_posx_topic": contract["safe_posx_topic"],
        "streamer.commanded_posx_topic": contract["commanded_posx_topic"],
        "streamer.servol_time_sec": contract["servol_time_s"],
        "streamer.servol_use_auto_velocity_acceleration": (
            contract["servol_use_auto_velocity_acceleration"]
        ),
        "streamer.stream_ramp_linear_mm_per_tick": (
            contract["linear_ramp_mm_per_tick"]
        ),
        "streamer.stream_ramp_rot_deg_per_tick": (
            contract["orientation_ramp_deg_per_tick"]
        ),
        "streamer.require_live_enable": True,
        "streamer.require_prepare_before_live": True,
        "streamer.require_robot_ready": True,
        "mux.lerobot_timeout_sec": contract["lerobot_timeout_s"],
        "mux.lerobot_target_topic": contract["lerobot_target_topic"],
        "mux.selected_target_topic": contract["selected_target_topic"],
    }
    mismatches = [
        f"{name}: expected={value!r} actual={running_parameters.get(name)!r}"
        for name, value in expected.items()
        if not _equal(value, running_parameters.get(name))
    ]
    if execute and running_parameters.get("streamer.dry_run") is not False:
        mismatches.append(
            "streamer.dry_run must be false only for an explicitly authorized trial"
        )
    return mismatches


def parameter_response_values(
    names: list[str],
    response: Any,
) -> dict[str, Any]:
    """Normalize rclpy parameter-client results across ROS distributions."""

    parameters = getattr(response, "values", response)
    if parameters is None or len(parameters) != len(names):
        raise RuntimeError("parameter query returned an unexpected result count")
    result: dict[str, Any] = {}
    for name, parameter in zip(names, parameters, strict=True):
        if hasattr(parameter, "value"):
            result[name] = parameter.value
        else:
            result[name] = parameter_value_to_python(parameter)
    return result


class TaskCLiveTrialGate(LiveTrialGate):
    _POSE_MATCH_POSITION_MM = 0.05
    _POSE_MATCH_ORIENTATION_DEG = 0.02
    _POSE_MATCH_MAX_AGE_S = 0.25
    _MISMATCH_STREAK_LIMIT = 3

    def __init__(self) -> None:
        super().__init__()
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.task_phase: str | None = None
        self.ready_event: dict[str, Any] | None = None
        self.last_task_event: dict[str, Any] | None = None
        self.task_failure_event: dict[str, Any] | None = None
        self.b_release_authorized = False
        self.b_release_command_sent = False
        self.b_release_confirmed = False
        self.b_task_complete = False
        self.b_event_order_error: str | None = None
        self.commanded: tuple[float, ...] | None = None
        self.commanded_time: float | None = None
        self.commanded_count = 0
        self._raw_history: deque[tuple[float, tuple[float, ...]]] = deque(maxlen=16)
        self._safe_history: deque[tuple[float, tuple[float, ...]]] = deque(maxlen=16)
        self.bridge_guard_exact_matches = 0
        self.bridge_stream_exact_matches = 0
        self.bridge_guard_mismatch_streak = 0
        self.bridge_stream_mismatch_streak = 0
        self.bridge_guard_interventions = 0
        self.bridge_stream_interventions = 0
        self.create_subscription(
            String,
            "/control/task_c/phase",
            self._on_task_phase,
            state_qos,
        )
        self.create_subscription(
            String,
            "/control/task_c/event",
            self._on_task_event,
            state_qos,
        )
        self.create_subscription(
            String,
            "/control/task_c/ready",
            self._on_task_ready,
            ready_qos,
        )
        self.create_subscription(
            Float64MultiArray,
            "/vr/commanded_posx",
            self._on_commanded,
            50,
        )

    def _on_task_phase(self, message: String) -> None:
        self.task_phase = str(message.data).strip()

    def _on_task_ready(self, message: String) -> None:
        try:
            value = json.loads(str(message.data))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(value, dict):
            return
        if (
            value.get("event") == "strategy_ready_external_gate_required"
            and isinstance(value.get("downstream_contract"), dict)
        ):
            self.ready_event = value

    def _on_task_event(self, message: String) -> None:
        try:
            value = json.loads(str(message.data))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(value, dict):
            return
        self.last_task_event = value
        event = str(value.get("event", ""))
        if event == "strategy_ready_external_gate_required":
            self.ready_event = value
        if event in {"fail_closed_external_gate_must_disable", "v2_fail_closed"}:
            self.task_failure_event = value
        if event == "act_b_release_authorized":
            self.b_release_authorized = True
        elif event == "act_b_release_command_sent":
            if not self.b_release_authorized:
                self.b_event_order_error = (
                    "release command preceded release authorization"
                )
            self.b_release_command_sent = True
        elif event == "act_b_release_confirmed":
            if not self.b_release_command_sent:
                self.b_event_order_error = (
                    "release confirmation preceded committed open command"
                )
            self.b_release_confirmed = True
        elif event == "act_b_task_complete":
            if not self.b_release_confirmed:
                self.b_event_order_error = (
                    "B completion preceded release confirmation"
                )
            self.b_task_complete = True

    def _on_target(self, message: Float64MultiArray) -> None:
        before = self.target_count
        super()._on_target(message)
        if self.target_count != before and self.target is not None:
            self._raw_history.append((time.monotonic(), self.target))

    def _on_safe(self, message: Float64MultiArray) -> None:
        values = self._finite_pose(message.data)
        super()._on_safe(message)
        if values is None:
            return
        now = time.monotonic()
        self._safe_history.append((now, values))
        if self.task_phase in {"BRIDGE", "RUN_BRIDGE", "HANDOFF_WINDOW", "B_SHADOW_PENDING", "B_PREFIX_ADMISSION", "SOFT_HANDOFF"}:
            if self._matches_recent(values, self._raw_history, now):
                self.bridge_guard_exact_matches += 1
                self.bridge_guard_mismatch_streak = 0
            else:
                self.bridge_guard_mismatch_streak += 1
                if self.bridge_guard_mismatch_streak == self._MISMATCH_STREAK_LIMIT:
                    self.bridge_guard_interventions += 1

    def _on_commanded(self, message: Float64MultiArray) -> None:
        values = self._finite_pose(message.data)
        if values is None:
            return
        now = time.monotonic()
        self.commanded = values
        self.commanded_time = now
        self.commanded_count += 1
        if self.task_phase in {"BRIDGE", "RUN_BRIDGE", "HANDOFF_WINDOW", "B_SHADOW_PENDING", "B_PREFIX_ADMISSION", "SOFT_HANDOFF"}:
            if self._matches_recent(values, self._safe_history, now):
                self.bridge_stream_exact_matches += 1
                self.bridge_stream_mismatch_streak = 0
            else:
                self.bridge_stream_mismatch_streak += 1
                if self.bridge_stream_mismatch_streak == self._MISMATCH_STREAK_LIMIT:
                    self.bridge_stream_interventions += 1

    @classmethod
    def _matches_recent(
        cls,
        pose: tuple[float, ...],
        history: deque[tuple[float, tuple[float, ...]]],
        now: float,
    ) -> bool:
        for timestamp, candidate in reversed(history):
            if now - timestamp > cls._POSE_MATCH_MAX_AGE_S:
                break
            position, orientation = pose_delta_metrics(candidate, pose)
            if (
                position <= cls._POSE_MATCH_POSITION_MM
                and orientation <= cls._POSE_MATCH_ORIENTATION_DEG
            ):
                return True
        return False

    def _parameter_values(
        self,
        remote_node: str,
        names: list[str],
        *,
        timeout_s: float = 5.0,
    ) -> dict[str, Any]:
        client = AsyncParameterClient(self, remote_node)
        if not client.wait_for_services(timeout_sec=timeout_s):
            raise RuntimeError(f"parameter service unavailable: {remote_node}")
        future = client.get_parameters(names)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError(f"parameter query timed out: {remote_node}")
            self.spin_callbacks(timeout_sec=0.02)
        response = future.result()
        try:
            return parameter_response_values(names, response)
        except RuntimeError as exc:
            raise RuntimeError(f"parameter query failed: {remote_node}: {exc}") from exc

    def read_running_contract(self) -> dict[str, Any]:
        guard_names = [
            "publish_rate_hz",
            "target_posx_topic",
            "safe_posx_topic",
            "workspace_min_xyz_mm",
            "workspace_min_limit_enabled",
            "workspace_max_xyz_mm",
            "enable_orientation_limits",
            "max_orientation_delta_deg",
        ]
        streamer_names = [
            "dry_run",
            "publish_rate_hz",
            "safe_posx_topic",
            "commanded_posx_topic",
            "servol_time_sec",
            "servol_use_auto_velocity_acceleration",
            "stream_ramp_linear_mm_per_tick",
            "stream_ramp_rot_deg_per_tick",
            "require_live_enable",
            "require_prepare_before_live",
            "require_robot_ready",
        ]
        mux_names = [
            "lerobot_timeout_sec",
            "lerobot_target_topic",
            "selected_target_topic",
        ]
        result: dict[str, Any] = {}
        for prefix, node_name, names in (
            ("guard", "/safety_guard_node", guard_names),
            ("streamer", "/servol_rt_streamer_node", streamer_names),
            ("mux", "/a0509_command_mux_node", mux_names),
        ):
            values = self._parameter_values(node_name, names)
            result.update({f"{prefix}.{name}": value for name, value in values.items()})
        return result

    def assert_start_envelope(self, args: argparse.Namespace) -> None:
        if self.actual is None:
            raise RuntimeError("actual TCP is unavailable")
        center = (
            *args.start_position_center_mm,
            *args.start_orientation_center_deg,
        )
        position, orientation = pose_delta_metrics(center, self.actual)
        print(
            "TASK_C_START_ENVELOPE "
            f"position_error_mm={position:.3f} "
            f"orientation_error_deg={orientation:.3f} "
            f"actual={list(self.actual)}",
            flush=True,
        )
        if position > args.start_position_radius_mm:
            raise RuntimeError("actual TCP is outside Task-A start position envelope")
        if orientation > args.start_orientation_radius_deg:
            raise RuntimeError("actual TCP is outside Task-A start orientation envelope")

    def bridge_pipeline_ok(self) -> bool:
        return (
            self.bridge_guard_interventions == 0
            and self.bridge_stream_interventions == 0
            and self.bridge_guard_mismatch_streak < self._MISMATCH_STREAK_LIMIT
            and self.bridge_stream_mismatch_streak < self._MISMATCH_STREAK_LIMIT
        )

    def summary(self) -> dict[str, Any]:
        return {
            "terminal_phase": self.task_phase,
            "last_task_event": self.last_task_event,
            "b_release_authorized": self.b_release_authorized,
            "b_release_command_sent": self.b_release_command_sent,
            "b_release_confirmed": self.b_release_confirmed,
            "b_task_complete": self.b_task_complete,
            "b_event_order_error": self.b_event_order_error,
            "bridge_guard_exact_matches": self.bridge_guard_exact_matches,
            "bridge_stream_exact_matches": self.bridge_stream_exact_matches,
            "bridge_guard_interventions": self.bridge_guard_interventions,
            "bridge_stream_interventions": self.bridge_stream_interventions,
            "target_count": self.target_count,
            "safe_count": self.safe_count,
            "commanded_count": self.commanded_count,
        }


def _fresh(timestamp: float | None, maximum_age_s: float, now: float) -> bool:
    return timestamp is not None and now - timestamp <= maximum_age_s


def gripper_open_is_violation(
    task_phase: str | None,
    commanded_state: float | None,
    *,
    release_authorized: bool,
) -> bool:
    if commanded_state is None or commanded_state >= 0.5:
        return False
    if task_phase in {"BRIDGE", "RUN_BRIDGE", "HANDOFF_WINDOW", "B_SHADOW_PENDING", "B_PREFIX_ADMISSION", "SOFT_HANDOFF"}:
        return True
    return task_phase in {"ACT_B", "RUN_B"} and not release_authorized


def run(args: argparse.Namespace) -> None:
    if args.execute and args.motion_authorization != MOTION_AUTHORIZATION:
        raise SystemExit(
            "--execute requires --motion-authorization=" + MOTION_AUTHORIZATION
        )
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TaskCLiveTrialGate()
    completed = False
    gate_report: dict[str, Any] = {}
    try:
        node.force_safe()
        node.wait_for(
            lambda: node.ready_event is not None,
            "latched Task-C strategy ready event",
            args.startup_timeout_sec,
        )
        node.wait_for(
            lambda: (
                node.actual is not None
                and _fresh(node.actual_time, args.state_max_age_sec, time.monotonic())
                and node.source == "DISABLED"
                and node.live is False
            ),
            "fresh actual TCP with MUX DISABLED and Live OFF",
            args.startup_timeout_sec,
        )
        node.assert_start_envelope(args)
        running_contract = node.read_running_contract()
        assert node.ready_event is not None
        mismatches = contract_mismatches(
            node.ready_event,
            running_contract,
            execute=args.execute,
        )
        gate_report["running_contract"] = running_contract
        gate_report["contract_mismatches"] = mismatches
        if mismatches:
            raise RuntimeError("downstream ROS contract mismatch: " + " | ".join(mismatches))
        print("TASK_C_GATE_PREFLIGHT=PASSED", flush=True)

        if not args.execute:
            completed = True
            print("TASK_C_GATE_STATE=PREFLIGHT_ONLY_COMPLETE", flush=True)
            return

        first_target_count = node.target_count
        first_safe_count = node.safe_count
        node.begin_gripper_monitoring()
        node.call(node.lerobot_client, Trigger.Request(), "select_lerobot")
        node.wait_for(
            lambda: (
                node.source == "LEROBOT"
                and node.selected_valid is True
                and node.target_count > first_target_count
                and node.safe_count > first_safe_count
                and _fresh(node.target_time, args.target_max_age_sec, time.monotonic())
                and _fresh(node.safe_time, args.target_max_age_sec, time.monotonic())
            ),
            "fresh selected LeRobot hold target",
            args.startup_timeout_sec,
        )
        node.call(node.live_client, SetBool.Request(data=True), "live_on")
        node.wait_for(
            lambda: node.live is True and node.task_phase in {"ACT_A", "RUN_A"},
            "Task-C ACT_A Live state",
            args.startup_timeout_sec,
        )
        node.wait_for(
            lambda: node.policy_queue_ready is True,
            "fresh ACT-A queue",
            args.startup_timeout_sec,
        )
        print("TASK_C_GATE_STATE=RUNNING", flush=True)

        deadline = time.monotonic() + args.transition_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            node.spin_callbacks(timeout_sec=0.02)
            if node.task_failure_event is not None or node.task_phase == "FAILED_HOLD":
                raise RuntimeError(f"Task-C strategy failed: {node.task_failure_event}")
            if node.b_event_order_error is not None:
                raise RuntimeError(
                    f"Task-C B event order failed: {node.b_event_order_error}"
                )
            if node.task_phase == "COMPLETE" and node.b_task_complete:
                break
            now = time.monotonic()
            if node.source != "LEROBOT" or node.selected_valid is not True:
                raise RuntimeError("Task-C lost valid LEROBOT MUX ownership")
            if node.live is not True:
                raise RuntimeError("Task-C Live output disabled before terminal state")
            if not _fresh(node.target_time, args.target_max_age_sec, now):
                raise RuntimeError("Task-C raw policy target became stale")
            if not _fresh(node.safe_time, args.target_max_age_sec, now):
                raise RuntimeError("Task-C safe_posx became stale")
            if not _fresh(node.actual_time, args.state_max_age_sec, now):
                raise RuntimeError("Task-C actual TCP became stale")
            if not _fresh(node.commanded_time, args.commanded_max_age_sec, now):
                raise RuntimeError("Task-C commanded_posx became stale")
            if not node.bridge_pipeline_ok():
                raise RuntimeError("Bridge was altered by safety guard or ServoL ramp")
            if gripper_open_is_violation(
                node.task_phase,
                node._last_commanded_state,
                release_authorized=node.b_release_authorized,
            ):
                raise RuntimeError(
                    "gripper commanded state opened before ACT-B release authorization"
                )
        else:
            raise TimeoutError("Task-C full semantic trial timeout")

        if not node.b_release_authorized:
            raise RuntimeError("ACT-B never authorized the demonstration release")
        if not node.b_release_command_sent:
            raise RuntimeError("ACT-B authorized release but sent no open command")
        if not node.b_release_confirmed:
            raise RuntimeError("ACT-B gripper release was not driver-confirmed")
        if not node.b_task_complete:
            raise RuntimeError("ACT-B did not reach semantic task completion")
        if node._last_commanded_state is None or node._last_commanded_state >= 0.5:
            raise RuntimeError("ACT-B completed without an open gripper command state")

        if node.bridge_guard_exact_matches < args.minimum_bridge_matches:
            raise RuntimeError("insufficient exact raw-to-safe Bridge matches")
        if node.bridge_stream_exact_matches < args.minimum_bridge_matches:
            raise RuntimeError("insufficient exact safe-to-commanded Bridge matches")
        completed = True
        print("TASK_C_GATE_STATE=COMPLETED", flush=True)
    except KeyboardInterrupt:
        print("TASK_C_GATE_INTERRUPT=SIGINT", flush=True)
    finally:
        gate_report.update(node.summary())
        gate_report["completed"] = completed
        print("TASK_C_GATE_SUMMARY=" + json.dumps(gate_report, sort_keys=True), flush=True)
        if args.report_json:
            output = Path(args.report_json).expanduser().resolve()
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(
                json.dumps(gate_report, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        node.print_gripper_summary()
        node.force_safe()
        node.destroy_node()
        rclpy.shutdown()
        if not completed:
            print("TASK_C_GATE_STATE=ABORTED", flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--motion-authorization", default="")
    parser.add_argument("--startup-timeout-sec", type=float, default=90.0)
    parser.add_argument("--transition-timeout-sec", type=float, default=90.0)
    parser.add_argument("--target-max-age-sec", type=float, default=0.30)
    parser.add_argument("--state-max-age-sec", type=float, default=0.50)
    parser.add_argument("--commanded-max-age-sec", type=float, default=0.30)
    parser.add_argument("--minimum-bridge-matches", type=int, default=5)
    parser.add_argument("--report-json", default="")
    parser.add_argument(
        "--start-position-center-mm",
        type=float,
        nargs=3,
        default=(427.3434661865234, 0.565598671634992, 458.2581522623698),
    )
    parser.add_argument("--start-position-radius-mm", type=float, default=35.0)
    parser.add_argument(
        "--start-orientation-center-deg",
        type=float,
        nargs=3,
        default=(0.10490300878882408, 149.98989868164062, 0.12113751471042633),
    )
    parser.add_argument("--start-orientation-radius-deg", type=float, default=6.0)
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
