#!/usr/bin/env python3
"""Generic external safety gate for one planner-compiled Multi-V2 episode.

Unlike the historical two-policy Task-C gate, this gate does not encode a
specific release or completion semantic.  Semantic authority belongs to the
server-validated Dijkstra plan and per-stage supervisors.  This process owns
only MUX/Live authority, downstream contract verification, fresh-stream
monitoring, and fail-closed shutdown.

Preflight-only is the default.  Real motion requires the exact authorization
phrase and is never enabled by the rollout process itself.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

try:
    from a0509_act_live_trial_gate import pose_delta_metrics
    from a0509_task_c_live_trial_gate import (
        TaskCLiveTrialGate,
        _fresh,
        contract_mismatches,
    )
except ModuleNotFoundError:  # Imported as scripts.* in unit tests.
    from scripts.a0509_act_live_trial_gate import pose_delta_metrics
    from scripts.a0509_task_c_live_trial_gate import (
        TaskCLiveTrialGate,
        _fresh,
        contract_mismatches,
    )


MOTION_AUTHORIZATION = "I_ACKNOWLEDGE_A0509_REAL_ROBOT_MOTION"


class MultiStageLiveTrialGate(TaskCLiveTrialGate):
    def __init__(self) -> None:
        self.multi_ready_event: dict[str, Any] | None = None
        self.multi_complete_event: dict[str, Any] | None = None
        self.multi_failure_event: dict[str, Any] | None = None
        self.multi_stage_event_count = 0
        super().__init__()
        multi_ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            "/control/task_c/multi_ready",
            self._on_multi_ready,
            multi_ready_qos,
        )

    def _on_multi_ready(self, message: String) -> None:
        try:
            value = json.loads(str(message.data))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(value, dict):
            return
        if (
            value.get("event") == "multi_stage_v2_ready"
            and value.get("composition_id")
            and value.get("plan_path")
        ):
            self.multi_ready_event = value

    def _on_task_event(self, message: String) -> None:
        super()._on_task_event(message)
        try:
            value = json.loads(str(message.data))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(value, dict):
            return
        event = str(value.get("event", ""))
        if event.startswith("multi_"):
            self.multi_stage_event_count += 1
        if event == "multi_stage_v2_ready":
            self.multi_ready_event = value
        if event in {"multi_episode_complete", "multi_episode_control_complete"}:
            self.multi_complete_event = value
        if event in {
            "multi_multi_stage_fail_closed",
            "multi_stage_fail_closed",
            "v2_fail_closed",
            "fail_closed_external_gate_must_disable",
        }:
            self.multi_failure_event = value

    def generic_summary(self) -> dict[str, Any]:
        return {
            "terminal_phase": self.task_phase,
            "multi_ready": self.multi_ready_event is not None,
            "multi_complete": self.multi_complete_event is not None,
            "multi_failure_event": self.multi_failure_event,
            "multi_stage_event_count": self.multi_stage_event_count,
            "bridge_guard_interventions": self.bridge_guard_interventions,
            "bridge_stream_interventions": self.bridge_stream_interventions,
            "target_count": self.target_count,
            "safe_count": self.safe_count,
            "commanded_count": self.commanded_count,
        }


def _parent_alive(parent_pid: int | None) -> bool:
    if parent_pid is None:
        return True
    try:
        os.kill(parent_pid, 0)
    except (ProcessLookupError, PermissionError):
        return False
    return True


def _assert_ready_plan(
    event: dict[str, Any],
    *,
    expected_plan: Path,
    expected_composition_id: str,
) -> None:
    actual_plan = Path(str(event.get("plan_path", ""))).expanduser().resolve()
    if actual_plan != expected_plan:
        raise RuntimeError(
            f"running Multi-V2 plan mismatch: expected={expected_plan} actual={actual_plan}"
        )
    actual_composition = str(event.get("composition_id", ""))
    if actual_composition != expected_composition_id:
        raise RuntimeError(
            "running Multi-V2 composition mismatch: "
            f"expected={expected_composition_id} actual={actual_composition}"
        )
    expected_record = json.loads(expected_plan.read_text(encoding="utf-8"))
    expected_profile = expected_record.get("runtime_command_profile_id")
    actual_profile = event.get("runtime_command_profile_id")
    if actual_profile != expected_profile:
        raise RuntimeError(
            "running Multi-V2 command profile mismatch: "
            f"expected={expected_profile!r} actual={actual_profile!r}"
        )


def run(args: argparse.Namespace) -> None:
    if args.execute and args.motion_authorization != MOTION_AUTHORIZATION:
        raise SystemExit(
            "--execute requires --motion-authorization=" + MOTION_AUTHORIZATION
        )
    if args.execute and args.supervisor_pid is None:
        raise SystemExit("--execute requires --supervisor-pid")
    expected_plan = Path(args.multi_stage_plan).expanduser().resolve()
    if not expected_plan.is_file():
        raise FileNotFoundError(f"multi-stage plan not found: {expected_plan}")

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MultiStageLiveTrialGate()
    completed = False
    error: BaseException | None = None
    report: dict[str, Any] = {
        "execute": bool(args.execute),
        "expected_plan": str(expected_plan),
        "expected_composition_id": args.expected_composition_id,
        "physical_motion_authorized": bool(args.execute),
    }
    try:
        node.force_safe()
        node.wait_for(
            lambda: node.ready_event is not None and node.multi_ready_event is not None,
            "latched Task-C and Multi-V2 ready events",
            args.startup_timeout_sec,
        )
        assert node.ready_event is not None
        assert node.multi_ready_event is not None
        _assert_ready_plan(
            node.multi_ready_event,
            expected_plan=expected_plan,
            expected_composition_id=args.expected_composition_id,
        )
        node.wait_for(
            lambda: (
                node.actual is not None
                and node.target is not None
                and _fresh(
                    node.actual_time,
                    args.state_max_age_sec,
                    time.monotonic(),
                )
                and node.source == "DISABLED"
                and node.live is False
            ),
            "fresh actual/hold target with MUX DISABLED and Live OFF",
            args.startup_timeout_sec,
        )
        assert node.actual is not None and node.target is not None
        position_delta, orientation_delta = pose_delta_metrics(
            node.actual,
            node.target,
        )
        report["initial_hold"] = {
            "position_delta_mm": position_delta,
            "orientation_delta_deg": orientation_delta,
            "actual": list(node.actual),
            "target": list(node.target),
        }
        if position_delta > args.position_limit_mm:
            raise RuntimeError(
                f"initial hold target position delta {position_delta:.3f} mm exceeds "
                f"{args.position_limit_mm:.3f} mm"
            )
        if orientation_delta > args.orientation_limit_deg:
            raise RuntimeError(
                f"initial hold target orientation delta {orientation_delta:.3f} deg exceeds "
                f"{args.orientation_limit_deg:.3f} deg"
            )

        running_contract = node.read_running_contract()
        mismatches = contract_mismatches(
            node.ready_event,
            running_contract,
            execute=args.execute,
        )
        report["running_contract"] = running_contract
        report["contract_mismatches"] = mismatches
        if mismatches:
            raise RuntimeError(
                "downstream ROS contract mismatch: " + " | ".join(mismatches)
            )
        print("MULTI_WEB_GATE_PREFLIGHT=PASSED", flush=True)

        if not args.execute:
            completed = True
            print("MULTI_WEB_GATE_STATE=PREFLIGHT_ONLY_COMPLETE", flush=True)
            return

        if not _parent_alive(args.supervisor_pid):
            raise RuntimeError("web supervisor is not alive before motion arming")
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
                and _fresh(
                    node.target_time,
                    args.target_max_age_sec,
                    time.monotonic(),
                )
                and _fresh(
                    node.safe_time,
                    args.target_max_age_sec,
                    time.monotonic(),
                )
            ),
            "fresh selected LeRobot hold target",
            args.startup_timeout_sec,
        )
        node.call(node.live_client, SetBool.Request(data=True), "live_on")
        node.wait_for(
            lambda: node.live is True and node.task_phase in {"ACT_A", "RUN_A"},
            "Multi-V2 initial policy Live state",
            args.startup_timeout_sec,
        )
        node.wait_for(
            lambda: node.policy_queue_ready is True,
            "fresh initial ACT queue",
            args.startup_timeout_sec,
        )
        print("MULTI_WEB_GATE_STATE=RUNNING", flush=True)

        deadline = time.monotonic() + args.episode_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            node.spin_callbacks(timeout_sec=0.02)
            if not _parent_alive(args.supervisor_pid):
                raise RuntimeError("web supervisor disappeared during execution")
            if (
                node.task_failure_event is not None
                or node.multi_failure_event is not None
                or node.task_phase == "FAILED_HOLD"
            ):
                raise RuntimeError(
                    "Multi-V2 strategy failed: "
                    f"{node.multi_failure_event or node.task_failure_event}"
                )
            if node.task_phase == "COMPLETE" and node.multi_complete_event is not None:
                break
            now = time.monotonic()
            if node.source != "LEROBOT" or node.selected_valid is not True:
                raise RuntimeError("Multi-V2 lost valid LEROBOT MUX ownership")
            if node.live is not True:
                raise RuntimeError("Live output disabled before terminal state")
            if not _fresh(node.target_time, args.target_max_age_sec, now):
                raise RuntimeError("raw policy target became stale")
            if not _fresh(node.safe_time, args.target_max_age_sec, now):
                raise RuntimeError("safe_posx became stale")
            if not _fresh(node.actual_time, args.state_max_age_sec, now):
                raise RuntimeError("actual TCP became stale")
            if not _fresh(node.commanded_time, args.commanded_max_age_sec, now):
                raise RuntimeError("commanded_posx became stale")
            if not node.bridge_pipeline_ok():
                raise RuntimeError("Bridge was altered by guard/streamer or became stale")
        else:
            raise TimeoutError("Multi-V2 full episode timeout")

        completed = True
        print("MULTI_WEB_GATE_STATE=COMPLETED", flush=True)
    except BaseException as exc:  # cleanup must run for every failure/interrupt
        error = exc
    finally:
        report.update(node.generic_summary())
        report["completed"] = completed
        report["error"] = None if error is None else f"{type(error).__name__}: {error}"
        print("MULTI_WEB_GATE_SUMMARY=" + json.dumps(report, sort_keys=True), flush=True)
        if args.report_json:
            output = Path(args.report_json).expanduser().resolve()
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(
                json.dumps(report, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        node.print_gripper_summary()
        node.force_safe()
        node.destroy_node()
        rclpy.shutdown()
    if error is not None:
        raise error


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--multi-stage-plan", required=True)
    parser.add_argument("--expected-composition-id", required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--motion-authorization", default="")
    parser.add_argument("--supervisor-pid", type=int)
    parser.add_argument("--startup-timeout-sec", type=float, default=120.0)
    parser.add_argument("--episode-timeout-sec", type=float, default=900.0)
    parser.add_argument("--target-max-age-sec", type=float, default=0.30)
    parser.add_argument("--state-max-age-sec", type=float, default=0.50)
    parser.add_argument("--commanded-max-age-sec", type=float, default=0.30)
    parser.add_argument("--position-limit-mm", type=float, default=50.0)
    parser.add_argument("--orientation-limit-deg", type=float, default=10.0)
    parser.add_argument("--report-json", default="")
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
