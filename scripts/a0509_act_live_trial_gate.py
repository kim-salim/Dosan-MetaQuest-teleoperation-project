#!/usr/bin/env python3
"""Arm one bounded A0509 ACT live trial on the first fresh policy target."""

from __future__ import annotations

import argparse
import math
import time
from collections import Counter, deque
from collections.abc import Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from quest_a0509_teleop.doosan_orientation import doosan_zyz_deg_to_quaternion


def pose_delta_metrics(
    actual: tuple[float, ...], target: tuple[float, ...]
) -> tuple[float, float]:
    position_mm = math.sqrt(
        sum((target[index] - actual[index]) ** 2 for index in range(3))
    )
    actual_q = doosan_zyz_deg_to_quaternion(actual[3:6])
    target_q = doosan_zyz_deg_to_quaternion(target[3:6])
    dot = abs(sum(left * right for left, right in zip(actual_q, target_q, strict=True)))
    orientation_deg = 2.0 * math.degrees(math.acos(min(1.0, max(0.0, dot))))
    return position_mm, orientation_deg


class LiveTrialGate(Node):
    _MAX_READY_CALLBACKS_PER_CYCLE = 64

    def __init__(self) -> None:
        super().__init__("a0509_act_live_trial_gate")
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.actual: tuple[float, ...] | None = None
        self.actual_time: float | None = None
        self.target: tuple[float, ...] | None = None
        self.target_time: float | None = None
        self.target_count = 0
        self.safe_time: float | None = None
        self.safe_count = 0
        self.selected_valid: bool | None = None
        self.source: str | None = None
        self.live: bool | None = None
        self.policy_queue_ready: bool | None = None
        self.policy_queue_ready_time: float | None = None
        self.gripper_monitoring = False
        self.gripper_monitor_started_at: float | None = None
        self.gripper_targets: list[tuple[float, float]] = []
        self.raw_gripper_targets: list[tuple[float, float]] = []
        self.gripper_commands: list[tuple[float, str]] = []
        self.gripper_commanded_states: list[tuple[float, float]] = []
        self._last_gripper_region: str | None = None
        self._last_gripper_command: str | None = None
        self._last_commanded_state: float | None = None

        self.create_subscription(
            Float64MultiArray,
            "/rt_topic/actual_tcp_position",
            self._on_actual,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/control/lerobot/target_posx",
            self._on_target,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/vr/safe_posx",
            self._on_safe,
            10,
        )
        self.create_subscription(
            Bool,
            "/control/selected_command_valid",
            lambda msg: setattr(self, "selected_valid", bool(msg.data)),
            state_qos,
        )
        self.create_subscription(
            String,
            "/control/source",
            lambda msg: setattr(self, "source", str(msg.data)),
            state_qos,
        )
        self.create_subscription(
            Bool,
            "/vr/live_robot_output_enabled",
            lambda msg: setattr(self, "live", bool(msg.data)),
            state_qos,
        )
        self.create_subscription(
            Bool,
            "/control/lerobot/policy_queue_ready",
            self._on_policy_queue_ready,
            state_qos,
        )
        self.create_subscription(
            Float64,
            "/control/lerobot/gripper_target",
            self._on_gripper_target,
            50,
        )
        self.create_subscription(
            Float64,
            "/control/lerobot/diffusion_gripper_raw",
            self._on_raw_gripper_target,
            50,
        )
        self.create_subscription(
            String,
            "/jrt_gripper/cmd",
            self._on_gripper_command,
            50,
        )
        self.create_subscription(
            Float64,
            "/jrt_gripper/commanded_state",
            self._on_gripper_commanded_state,
            state_qos,
        )
        self.live_client = self.create_client(SetBool, "/vr/set_live_robot_output")
        self.disabled_client = self.create_client(Trigger, "/control/select_disabled")
        self.lerobot_client = self.create_client(Trigger, "/control/select_lerobot")

    def _on_actual(self, message: Float64MultiArray) -> None:
        values = self._finite_pose(message.data)
        if values is not None:
            self.actual = values
            self.actual_time = time.monotonic()

    def _on_target(self, message: Float64MultiArray) -> None:
        values = self._finite_pose(message.data)
        if values is not None:
            self.target = values
            self.target_time = time.monotonic()
            self.target_count += 1

    def _on_safe(self, message: Float64MultiArray) -> None:
        values = self._finite_pose(message.data)
        if values is not None:
            self.safe_time = time.monotonic()
            self.safe_count += 1

    def _on_policy_queue_ready(self, message: Bool) -> None:
        self.policy_queue_ready = bool(message.data)
        self.policy_queue_ready_time = time.monotonic()

    def _on_gripper_target(self, message: Float64) -> None:
        if not self.gripper_monitoring:
            return
        value = float(message.data)
        if not math.isfinite(value):
            return
        now = time.monotonic()
        self.gripper_targets.append((now, value))
        region = "close" if value > 0.7 else "open" if value < 0.3 else "hold"
        if region != self._last_gripper_region:
            print(
                f"LIVE_TRIAL_GRIPPER_TARGET region={region} value={value:.6f} "
                f"elapsed_s={self._gripper_elapsed(now):.3f}",
                flush=True,
            )
            self._last_gripper_region = region

    def _on_raw_gripper_target(self, message: Float64) -> None:
        if not self.gripper_monitoring:
            return
        value = float(message.data)
        if math.isfinite(value):
            self.raw_gripper_targets.append((time.monotonic(), value))

    def _on_gripper_command(self, message: String) -> None:
        if not self.gripper_monitoring:
            return
        command = str(message.data).strip().lower()
        now = time.monotonic()
        self.gripper_commands.append((now, command))
        if command != self._last_gripper_command:
            print(
                f"LIVE_TRIAL_GRIPPER_MUX command={command} "
                f"elapsed_s={self._gripper_elapsed(now):.3f}",
                flush=True,
            )
            self._last_gripper_command = command

    def _on_gripper_commanded_state(self, message: Float64) -> None:
        if not self.gripper_monitoring:
            return
        value = float(message.data)
        if not math.isfinite(value):
            return
        now = time.monotonic()
        self.gripper_commanded_states.append((now, value))
        if self._last_commanded_state is None or abs(value - self._last_commanded_state) > 1e-9:
            print(
                f"LIVE_TRIAL_GRIPPER_COMMANDED_STATE value={value:.1f} "
                f"elapsed_s={self._gripper_elapsed(now):.3f}",
                flush=True,
            )
            self._last_commanded_state = value

    def _gripper_elapsed(self, now: float) -> float:
        if self.gripper_monitor_started_at is None:
            return 0.0
        return max(0.0, now - self.gripper_monitor_started_at)

    def begin_gripper_monitoring(self) -> None:
        self.gripper_monitoring = True
        self.gripper_monitor_started_at = time.monotonic()
        self.gripper_targets.clear()
        self.raw_gripper_targets.clear()
        self.gripper_commands.clear()
        self.gripper_commanded_states.clear()
        self._last_gripper_region = None
        self._last_gripper_command = None
        self._last_commanded_state = None
        print("LIVE_TRIAL_GRIPPER_MONITOR=STARTED", flush=True)

    def print_gripper_summary(self) -> None:
        self.gripper_monitoring = False
        target_values = [value for _stamp, value in self.gripper_targets]
        raw_values = [value for _stamp, value in self.raw_gripper_targets]
        raw_ordered = sorted(raw_values)

        def raw_percentile(fraction: float) -> float | None:
            if not raw_ordered:
                return None
            index = round((len(raw_ordered) - 1) * fraction)
            return raw_ordered[index]

        raw_event_window = deque(maxlen=4)
        raw_event_window_count = 0
        raw_first_event_elapsed_s = None
        for stamp, value in self.raw_gripper_targets:
            raw_event_window.append(value)
            event_detected = (
                any(sample > 0.4 for sample in raw_event_window)
                and sum(sample >= 0.25 for sample in raw_event_window) >= 2
            )
            if event_detected:
                raw_event_window_count += 1
                if raw_first_event_elapsed_s is None:
                    raw_first_event_elapsed_s = self._gripper_elapsed(stamp)
        command_counts = Counter(command for _stamp, command in self.gripper_commands)
        state_values = [value for _stamp, value in self.gripper_commanded_states]
        first_close = next(
            (
                self._gripper_elapsed(stamp)
                for stamp, value in self.gripper_targets
                if value > 0.7
            ),
            None,
        )
        summary = {
            "raw_samples": len(raw_values),
            "raw_min": None if not raw_values else min(raw_values),
            "raw_max": None if not raw_values else max(raw_values),
            "raw_mean": (
                None if not raw_values else sum(raw_values) / len(raw_values)
            ),
            "raw_p50": raw_percentile(0.50),
            "raw_p90": raw_percentile(0.90),
            "raw_p95": raw_percentile(0.95),
            "raw_p99": raw_percentile(0.99),
            "raw_below_support_count": sum(value < 0.25 for value in raw_values),
            "raw_support_count": sum(value >= 0.25 for value in raw_values),
            "raw_peak_count": sum(value > 0.4 for value in raw_values),
            "raw_event_window_count": raw_event_window_count,
            "raw_first_event_elapsed_s": raw_first_event_elapsed_s,
            "raw_top_values": [
                round(value, 6) for value in sorted(raw_values, reverse=True)[:20]
            ],
            "target_samples": len(target_values),
            "target_min": None if not target_values else min(target_values),
            "target_max": None if not target_values else max(target_values),
            "target_open_count": sum(value < 0.3 for value in target_values),
            "target_hold_count": sum(0.3 <= value <= 0.7 for value in target_values),
            "target_close_count": sum(value > 0.7 for value in target_values),
            "first_close_target_elapsed_s": first_close,
            "mux_command_counts": dict(command_counts),
            "commanded_state_first": None if not state_values else state_values[0],
            "commanded_state_last": None if not state_values else state_values[-1],
            "commanded_state_closed_seen": any(value >= 0.5 for value in state_values),
        }
        print(f"LIVE_TRIAL_GRIPPER_SUMMARY={summary}", flush=True)

    @staticmethod
    def _finite_pose(values: object) -> tuple[float, ...] | None:
        try:
            pose = tuple(float(value) for value in values)
        except (TypeError, ValueError):
            return None
        if len(pose) != 6 or any(not math.isfinite(value) for value in pose):
            return None
        return pose

    def call(self, client: object, request: object, label: str, timeout_sec: float = 5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"{label} service unavailable")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError(label)
            self.spin_callbacks(timeout_sec=0.02)
        result = future.result()
        if result is None or not bool(result.success):
            message = "" if result is None else str(result.message)
            raise RuntimeError(f"{label} failed: {message}")
        print(f"LIVE_TRIAL_SERVICE={label} success message={result.message}", flush=True)
        return result

    def wait_for(self, predicate: Callable[[], bool], label: str, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(label)
            self.spin_callbacks(timeout_sec=0.02)

    def spin_callbacks(self, timeout_sec: float = 0.02) -> None:
        """Run one blocking callback, then drain callbacks already waiting.

        ``rclpy.spin_once`` executes at most one callback. This gate subscribes
        to several high-rate streams, so calling it only once per watchdog
        cycle can leave a fresh target queued behind state and gripper events.
        Drain a bounded batch so watchdog ages describe publisher freshness,
        not executor backlog, while still returning promptly to safety checks.
        """
        rclpy.spin_once(self, timeout_sec=timeout_sec)
        for _ in range(self._MAX_READY_CALLBACKS_PER_CYCLE):
            rclpy.spin_once(self, timeout_sec=0.0)

    def force_safe(self) -> None:
        errors: list[str] = []
        try:
            self.call(self.live_client, SetBool.Request(data=False), "live_off")
        except Exception as exc:
            errors.append(str(exc))
        # Give transient-local state publishers time to match a newly created
        # gate before deciding whether select_disabled is actually necessary.
        try:
            self.wait_for(
                lambda: self.live is False and self.source is not None,
                "observed Live OFF and current MUX source",
                3.0,
            )
        except Exception:
            pass
        # Calling select_disabled while the source is already DISABLED asks the
        # MUX to launch another asynchronous live_off request. That delayed
        # request can race a subsequent live_on. Skip only this redundant call;
        # an active source is always explicitly disabled.
        if self.source != "DISABLED":
            try:
                self.call(self.disabled_client, Trigger.Request(), "select_disabled")
            except Exception as exc:
                errors.append(str(exc))
        else:
            print(
                "LIVE_TRIAL_SERVICE=select_disabled skipped message=source already DISABLED",
                flush=True,
            )
        try:
            self.wait_for(
                lambda: self.live is False and self.source == "DISABLED",
                "verified final Live OFF and MUX DISABLED",
                3.0,
            )
        except Exception as exc:
            errors.append(str(exc))
        if errors:
            print(f"LIVE_TRIAL_SAFE_ERRORS={errors}", flush=True)


def run(args: argparse.Namespace) -> None:
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LiveTrialGate()
    completed = False
    try:
        node.force_safe()
        print("LIVE_TRIAL_STATE=WAITING_FOR_HOLD_TARGET", flush=True)
        node.wait_for(
            lambda: (
                node.actual is not None
                and node.target is not None
                and node.actual_time is not None
                and time.monotonic() - node.actual_time <= args.state_max_age_sec
            ),
            "fresh actual TCP and first policy target",
            args.startup_timeout_sec,
        )


        position_mm, orientation_deg = pose_delta_metrics(node.actual, node.target)
        print(
            "LIVE_TRIAL_HOLD_TARGET "
            f"position_delta_mm={position_mm:.3f} "
            f"orientation_delta_deg={orientation_deg:.3f} "
            f"actual={list(node.actual)} target={list(node.target)}",
            flush=True,
        )
        if position_mm > args.position_limit_mm:
            raise RuntimeError(
                f"first target position delta {position_mm:.3f} mm exceeds "
                f"{args.position_limit_mm:.3f} mm"
            )
        if orientation_deg > args.orientation_limit_deg:
            raise RuntimeError(
                f"first target orientation delta {orientation_deg:.3f} deg exceeds "
                f"{args.orientation_limit_deg:.3f} deg"
            )

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
                and node.safe_time is not None
                and time.monotonic() - node.safe_time <= args.target_max_age_sec
            ),
            "fresh selected LeRobot target and safe_posx",
            1.0,
        )
        print("LIVE_TRIAL_STATE=WAITING_FOR_FRESH_MODEL_TARGET", flush=True)
        node.call(node.live_client, SetBool.Request(data=True), "live_on")
        node.wait_for(
            lambda: node.live is True,
            "live state true",
            args.live_state_timeout_sec,
        )
        node.wait_for(
            lambda: node.policy_queue_ready is True,
            "fresh ACT queue after Live ON",
            args.startup_timeout_sec,
        )
        arming_deadline = time.monotonic() + args.arming_settle_sec
        while rclpy.ok() and time.monotonic() < arming_deadline:
            node.spin_callbacks(timeout_sec=0.02)
        node.wait_for(
            lambda: (
                node.target_time is not None
                and node.policy_queue_ready_time is not None
                and node.target_time >= node.policy_queue_ready_time - 0.20
                and time.monotonic() - node.target_time <= args.target_max_age_sec
            ),
            "fresh model target after ACT queue ready",
            args.startup_timeout_sec,
        )

        position_mm, orientation_deg = pose_delta_metrics(node.actual, node.target)
        print(
            "LIVE_TRIAL_FIRST_MODEL_TARGET "
            f"position_delta_mm={position_mm:.3f} "
            f"orientation_delta_deg={orientation_deg:.3f} "
            f"actual={list(node.actual)} target={list(node.target)}",
            flush=True,
        )
        if position_mm > args.position_limit_mm:
            raise RuntimeError(
                f"first model target position delta {position_mm:.3f} mm exceeds "
                f"{args.position_limit_mm:.3f} mm"
            )
        if orientation_deg > args.orientation_limit_deg:
            raise RuntimeError(
                f"first model target orientation delta {orientation_deg:.3f} deg exceeds "
                f"{args.orientation_limit_deg:.3f} deg"
            )
        print("LIVE_TRIAL_STATE=RUNNING", flush=True)

        deadline = time.monotonic() + args.duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            node.spin_callbacks(timeout_sec=0.02)
            now = time.monotonic()
            if node.source != "LEROBOT":
                raise RuntimeError(f"MUX source changed to {node.source!r}")
            if node.selected_valid is not True:
                raise RuntimeError("selected command became invalid")
            if node.target_time is None or now - node.target_time > args.target_max_age_sec:
                raise RuntimeError("LeRobot target stream became stale")
            if node.safe_time is None or now - node.safe_time > args.target_max_age_sec:
                raise RuntimeError("safe_posx stream became stale")
            if node.actual_time is None or now - node.actual_time > args.state_max_age_sec:
                raise RuntimeError("actual TCP stream became stale")

        completed = True
        print("LIVE_TRIAL_STATE=COMPLETED", flush=True)
    except KeyboardInterrupt:
        print("LIVE_TRIAL_INTERRUPT=SIGINT", flush=True)
    finally:
        node.print_gripper_summary()
        node.force_safe()
        node.destroy_node()
        rclpy.shutdown()
        if not completed:
            print("LIVE_TRIAL_STATE=ABORTED", flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--startup-timeout-sec", type=float, default=45.0)
    parser.add_argument("--position-limit-mm", type=float, default=50.0)
    parser.add_argument("--orientation-limit-deg", type=float, default=10.0)
    parser.add_argument("--arming-settle-sec", type=float, default=0.5)
    parser.add_argument("--live-state-timeout-sec", type=float, default=5.0)
    parser.add_argument("--target-max-age-sec", type=float, default=0.30)
    parser.add_argument("--state-max-age-sec", type=float, default=0.50)
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
