"""Robot preparation and runtime services for Quest A0509 teleoperation."""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Iterable, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

try:
    from dsr_msgs2.srv import (
        CheckMotion,
        GetCurrentPosj,
        GetCurrentPosx,
        GetRobotState,
        MoveJoint,
        MoveSplineJoint,
        MoveStop,
        MoveWait,
        SetRobotControl,
        StartRtControl,
        StopRtControl,
    )
except Exception:  # pragma: no cover - depends on the Doosan workspace.
    CheckMotion = None
    GetCurrentPosj = None
    GetCurrentPosx = None
    GetRobotState = None
    MoveJoint = None
    MoveSplineJoint = None
    MoveStop = None
    MoveWait = None
    SetRobotControl = None
    StartRtControl = None
    StopRtControl = None


DR_BASE = 0
CONTROL_RESET_SAFE_OFF = 3
STOP_MODE_QSTOP = 1


def _vector6(values: Iterable[float], name: str) -> list[float]:
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


class RobotPrepNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_prep_node")
        self._declare_parameters()

        self.robot_namespace = str(self.get_parameter("robot_namespace").value).strip("/")
        self.status_topic = self.get_parameter("status_topic").value
        self.robot_anchor_posx_topic = self.get_parameter("robot_anchor_posx_topic").value
        self.teleop_ready_topic = self.get_parameter("teleop_ready_topic").value
        self.recenter_service_name = self.get_parameter("recenter_service").value
        self.set_live_service_name = self.get_parameter("set_live_service").value
        self.prepare_joint_deg = _vector6(
            self.get_parameter("prepare_joint_deg").value,
            "prepare_joint_deg",
        )
        self.prepare_joint_tolerance_deg = float(
            self.get_parameter("prepare_joint_tolerance_deg").value
        )
        self.prepare_max_step_deg = float(self.get_parameter("prepare_max_step_deg").value)
        self.prepare_j3_escape_deg = float(self.get_parameter("prepare_j3_escape_deg").value)
        self.prepare_j5_escape_deg = float(self.get_parameter("prepare_j5_escape_deg").value)
        self.prepare_j3_step_deg = float(self.get_parameter("prepare_j3_step_deg").value)
        self.prepare_j5_step_deg = float(self.get_parameter("prepare_j5_step_deg").value)
        self.prepare_vel_deg_per_sec = float(self.get_parameter("prepare_vel_deg_per_sec").value)
        self.prepare_acc_deg_per_sec2 = float(
            self.get_parameter("prepare_acc_deg_per_sec2").value
        )
        self.prepare_time_sec = float(self.get_parameter("prepare_time_sec").value)
        self.prepare_preflight_wait_sec = float(
            self.get_parameter("prepare_preflight_wait_sec").value
        )
        self.prepare_set_anchor_after_move = bool(
            self.get_parameter("prepare_set_anchor_after_move").value
        )
        self.safe_robot_states = _int_list(
            self.get_parameter("safe_robot_states").value,
            "safe_robot_states",
        )
        self.stop_mode = int(self.get_parameter("stop_mode").value)
        self.callback_group = ReentrantCallbackGroup()
        self.teleop_ready = False

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.teleop_ready_pub = self.create_publisher(Bool, self.teleop_ready_topic, 10)
        self.robot_anchor_pub = self.create_publisher(
            Float64MultiArray,
            self.robot_anchor_posx_topic,
            10,
        )
        self.recenter_client = self.create_client(
            Trigger,
            self.recenter_service_name,
            callback_group=self.callback_group,
        )
        self.set_live_client = self.create_client(
            SetBool,
            self.set_live_service_name,
            callback_group=self.callback_group,
        )

        self._make_doosan_clients()
        self.operation_lock = threading.Lock()

        self.prepare_srv = self.create_service(
            Trigger,
            "/vr/prepare_robot",
            self._on_prepare_robot,
            callback_group=self.callback_group,
        )
        self.anchor_srv = self.create_service(
            Trigger,
            "/vr/set_robot_anchor_to_current_tcp",
            self._on_set_anchor_to_current_tcp,
            callback_group=self.callback_group,
        )
        self.stop_srv = self.create_service(
            Trigger,
            "/vr/stop_robot",
            self._on_stop_robot,
            callback_group=self.callback_group,
        )
        self.reset_safe_off_srv = self.create_service(
            Trigger,
            "/vr/reset_safe_off",
            self._on_reset_safe_off,
            callback_group=self.callback_group,
        )
        self.start_rt_srv = self.create_service(
            Trigger,
            "/vr/start_rt_control",
            self._on_start_rt_control,
            callback_group=self.callback_group,
        )
        self.stop_rt_srv = self.create_service(
            Trigger,
            "/vr/stop_rt_control",
            self._on_stop_rt_control,
            callback_group=self.callback_group,
        )

        self._publish_status(
            "robot_prep_node started: "
            + json.dumps(
                {
                    "robot_namespace": self.robot_namespace,
                    "prepare_joint_deg": self.prepare_joint_deg,
                    "robot_anchor_posx_topic": self.robot_anchor_posx_topic,
                    "teleop_ready_topic": self.teleop_ready_topic,
                },
                sort_keys=True,
            )
        )
        self._publish_teleop_ready(False, "startup")

    def _declare_parameters(self) -> None:
        self.declare_parameter("robot_namespace", "/dsr01")
        self.declare_parameter("status_topic", "/vr/status")
        self.declare_parameter("robot_anchor_posx_topic", "/vr/robot_anchor_posx")
        self.declare_parameter("teleop_ready_topic", "/vr/teleop_ready")
        self.declare_parameter("recenter_service", "/vr/recenter")
        self.declare_parameter("set_live_service", "/vr/set_live_robot_output")
        self.declare_parameter("prepare_joint_deg", [0.0, 0.0, 90.0, 0.0, 30.0, 0.0])
        self.declare_parameter("prepare_joint_tolerance_deg", 2.0)
        self.declare_parameter("prepare_max_step_deg", 10.0)
        self.declare_parameter("prepare_j3_escape_deg", 20.0)
        self.declare_parameter("prepare_j5_escape_deg", 30.0)
        self.declare_parameter("prepare_j3_step_deg", 5.0)
        self.declare_parameter("prepare_j5_step_deg", 3.0)
        self.declare_parameter("prepare_vel_deg_per_sec", 30.0)
        self.declare_parameter("prepare_acc_deg_per_sec2", 30.0)
        self.declare_parameter("prepare_time_sec", 0.0)
        self.declare_parameter("prepare_preflight_wait_sec", 3.0)
        self.declare_parameter("prepare_set_anchor_after_move", True)
        self.declare_parameter("safe_robot_states", [1, 2])
        self.declare_parameter("stop_mode", STOP_MODE_QSTOP)

    def _make_doosan_clients(self) -> None:
        prefix = f"/{self.robot_namespace}"
        self.get_posj_client = (
            self.create_client(
                GetCurrentPosj,
                f"{prefix}/aux_control/get_current_posj",
                callback_group=self.callback_group,
            )
            if GetCurrentPosj is not None
            else None
        )
        self.get_posx_client = (
            self.create_client(
                GetCurrentPosx,
                f"{prefix}/aux_control/get_current_posx",
                callback_group=self.callback_group,
            )
            if GetCurrentPosx is not None
            else None
        )
        self.get_robot_state_client = (
            self.create_client(
                GetRobotState,
                f"{prefix}/system/get_robot_state",
                callback_group=self.callback_group,
            )
            if GetRobotState is not None
            else None
        )
        self.set_robot_control_client = (
            self.create_client(
                SetRobotControl,
                f"{prefix}/system/set_robot_control",
                callback_group=self.callback_group,
            )
            if SetRobotControl is not None
            else None
        )
        self.move_joint_client = (
            self.create_client(
                MoveJoint,
                f"{prefix}/motion/move_joint",
                callback_group=self.callback_group,
            )
            if MoveJoint is not None
            else None
        )
        self.move_spline_joint_client = (
            self.create_client(
                MoveSplineJoint,
                f"{prefix}/motion/move_spline_joint",
                callback_group=self.callback_group,
            )
            if MoveSplineJoint is not None
            else None
        )
        self.move_stop_client = (
            self.create_client(
                MoveStop,
                f"{prefix}/motion/move_stop",
                callback_group=self.callback_group,
            )
            if MoveStop is not None
            else None
        )
        self.move_wait_client = (
            self.create_client(
                MoveWait,
                f"{prefix}/motion/move_wait",
                callback_group=self.callback_group,
            )
            if MoveWait is not None
            else None
        )
        self.check_motion_client = (
            self.create_client(
                CheckMotion,
                f"{prefix}/motion/check_motion",
                callback_group=self.callback_group,
            )
            if CheckMotion is not None
            else None
        )
        self.start_rt_control_client = (
            self.create_client(
                StartRtControl,
                f"{prefix}/realtime/start_rt_control",
                callback_group=self.callback_group,
            )
            if StartRtControl is not None
            else None
        )
        self.stop_rt_control_client = (
            self.create_client(
                StopRtControl,
                f"{prefix}/realtime/stop_rt_control",
                callback_group=self.callback_group,
            )
            if StopRtControl is not None
            else None
        )

    def _on_prepare_robot(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        try:
            with self.operation_lock:
                self._prepare_robot()
            response.success = True
            response.message = "Robot moved to prep pose and anchor was updated."
        except Exception as exc:
            self._publish_teleop_ready(False, "prepare failed")
            response.success = False
            response.message = f"Prepare robot failed: {exc}"
            self._publish_status(response.message, warn=True)
        return response

    def _on_set_anchor_to_current_tcp(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        try:
            with self.operation_lock:
                if not self.teleop_ready:
                    raise RuntimeError("Prepare Robot must complete before anchoring current TCP.")
                posx = self._get_current_posx()
                self._publish_robot_anchor(posx)
                self._call_recenter_best_effort()
            response.success = True
            response.message = f"Robot anchor set to current TCP: {posx}"
        except Exception as exc:
            response.success = False
            response.message = f"Set robot anchor failed: {exc}"
            self._publish_status(response.message, warn=True)
        return response

    def _on_stop_robot(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        try:
            with self.operation_lock:
                self._publish_teleop_ready(False, "stop robot requested")
                self._set_live(False)
                self._move_stop()
            response.success = True
            response.message = "Live output disabled and MoveStop was sent."
        except Exception as exc:
            response.success = False
            response.message = f"Stop robot failed: {exc}"
            self._publish_status(response.message, warn=True)
        return response

    def _on_reset_safe_off(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        try:
            self._publish_teleop_ready(False, "SAFE_OFF reset requested")
            self._require_client(self.set_robot_control_client, "SetRobotControl")
            request = SetRobotControl.Request()
            request.robot_control = CONTROL_RESET_SAFE_OFF
            result = self._call_service(self.set_robot_control_client, request, timeout_sec=3.0)
            response.success = bool(result.success)
            response.message = f"SetRobotControl RESET_SAFE_OFF success={response.success}"
            self._publish_status(response.message, warn=not response.success)
        except Exception as exc:
            response.success = False
            response.message = f"Reset SAFE_OFF failed: {exc}"
            self._publish_status(response.message, warn=True)
        return response

    def _on_start_rt_control(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        try:
            self._require_client(self.start_rt_control_client, "StartRtControl")
            result = self._call_service(
                self.start_rt_control_client,
                StartRtControl.Request(),
                timeout_sec=3.0,
            )
            response.success = bool(result.success)
            response.message = f"StartRtControl success={response.success}"
            self._publish_status(response.message, warn=not response.success)
        except Exception as exc:
            response.success = False
            response.message = f"StartRtControl failed: {exc}"
            self._publish_status(response.message, warn=True)
        return response

    def _on_stop_rt_control(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        try:
            self._require_client(self.stop_rt_control_client, "StopRtControl")
            result = self._call_service(
                self.stop_rt_control_client,
                StopRtControl.Request(),
                timeout_sec=3.0,
            )
            response.success = bool(result.success)
            response.message = f"StopRtControl success={response.success}"
            self._publish_status(response.message, warn=not response.success)
        except Exception as exc:
            response.success = False
            response.message = f"StopRtControl failed: {exc}"
            self._publish_status(response.message, warn=True)
        return response

    def _prepare_robot(self) -> None:
        self._publish_teleop_ready(False, "prepare started")
        self._require_doosan_motion_clients()
        state = self._get_robot_state()
        if state not in self.safe_robot_states:
            raise RuntimeError(f"robot_state={state} is not ready for prep motion.")
        self._publish_status(
            f"prepare preflight wait {self.prepare_preflight_wait_sec:.1f}s before motion"
        )
        time.sleep(self.prepare_preflight_wait_sec)
        current = self._get_current_posj()
        phases = self._joint_escape_phases(current, self.prepare_joint_deg)
        waypoint_count = sum(len(waypoints) for _label, waypoints in phases)
        self._publish_status(
            "prepare motion plan: "
            + json.dumps(
                {
                    "current_posj": current,
                    "target_posj": self.prepare_joint_deg,
                    "phases": len(phases),
                    "waypoints": waypoint_count,
                },
                sort_keys=True,
            )
        )
        if not phases:
            self._publish_status("robot already near prep joint pose")
        for index, (label, waypoints) in enumerate(phases, start=1):
            if not waypoints:
                continue
            self._publish_status(
                f"CALL MoveSplineJoint prep phase {index}/{len(phases)} {label} "
                f"points={len(waypoints)} target={waypoints[-1]}"
            )
            self._move_spline_joint_abs(waypoints)
            reached = self._wait_for_joint_target(waypoints[-1])
            self._publish_status(
                f"prep phase {index}/{len(phases)} reached posj={reached}"
            )
        self._wait_for_motion_idle()
        final_posj = self._get_current_posj()
        final_error = max(
            abs(final_posj[index] - self.prepare_joint_deg[index])
            for index in range(6)
        )
        if final_error > self.prepare_joint_tolerance_deg:
            raise RuntimeError(
                f"prep final error {final_error:.2f} deg exceeds "
                f"{self.prepare_joint_tolerance_deg:.2f} deg"
            )
        posx = self._get_current_posx()
        self._publish_status(
            "prep pose ready: "
            + json.dumps({"posj": final_posj, "posx": posx}, sort_keys=True)
        )
        if self.prepare_set_anchor_after_move:
            self._publish_robot_anchor(posx)
            self._call_recenter_best_effort()
        self._publish_teleop_ready(True, "prepare complete")

    def _require_doosan_motion_clients(self) -> None:
        self._require_client(self.get_robot_state_client, "GetRobotState")
        self._require_client(self.get_posj_client, "GetCurrentPosj")
        self._require_client(self.get_posx_client, "GetCurrentPosx")
        self._require_client(self.move_joint_client, "MoveJoint")
        self._require_client(self.move_spline_joint_client, "MoveSplineJoint")

    def _get_robot_state(self) -> int:
        self._require_client(self.get_robot_state_client, "GetRobotState")
        response = self._call_service(
            self.get_robot_state_client,
            GetRobotState.Request(),
            timeout_sec=5.0,
        )
        if not response.success:
            raise RuntimeError("GetRobotState returned success=false")
        return int(response.robot_state)

    def _get_current_posj(self) -> list[float]:
        self._require_client(self.get_posj_client, "GetCurrentPosj")
        response = self._call_service(
            self.get_posj_client,
            GetCurrentPosj.Request(),
            timeout_sec=2.0,
        )
        if not response.success:
            raise RuntimeError("GetCurrentPosj returned success=false")
        return _vector6(response.pos, "current_posj")

    def _get_current_posx(self) -> list[float]:
        self._require_client(self.get_posx_client, "GetCurrentPosx")
        request = GetCurrentPosx.Request()
        request.ref = DR_BASE
        response = self._call_service(self.get_posx_client, request, timeout_sec=2.0)
        if not response.success or not response.task_pos_info:
            raise RuntimeError("GetCurrentPosx returned no task position")
        return _vector6(response.task_pos_info[0].data[:6], "current_posx")

    def _move_spline_joint_abs(self, waypoints: list[list[float]]) -> None:
        if len(waypoints) == 1:
            self._move_joint_abs(waypoints[0])
            return
        request = MoveSplineJoint.Request()
        request.pos = []
        for waypoint in waypoints[:100]:
            point = Float64MultiArray()
            point.data = _vector6(waypoint, "prepare waypoint")
            request.pos.append(point)
        request.pos_cnt = len(request.pos)
        request.vel = [float(self.prepare_vel_deg_per_sec)] * 6
        request.acc = [float(self.prepare_acc_deg_per_sec2)] * 6
        request.time = float(self.prepare_time_sec)
        request.mode = 0
        request.sync_type = 1
        response = self._call_service(
            self.move_spline_joint_client,
            request,
            timeout_sec=5.0,
        )
        if not response.success:
            raise RuntimeError("MoveSplineJoint returned success=false")

    def _move_joint_abs(self, target: list[float]) -> None:
        request = MoveJoint.Request()
        request.pos = _vector6(target, "prepare target")
        request.vel = float(self.prepare_vel_deg_per_sec)
        request.acc = float(self.prepare_acc_deg_per_sec2)
        request.time = float(self.prepare_time_sec)
        request.radius = 0.0
        request.mode = 0
        request.blend_type = 0
        request.sync_type = 1
        response = self._call_service(self.move_joint_client, request, timeout_sec=5.0)
        if not response.success:
            raise RuntimeError("MoveJoint returned success=false")

    def _move_stop(self) -> None:
        self._require_client(self.move_stop_client, "MoveStop")
        request = MoveStop.Request()
        request.stop_mode = int(self.stop_mode)
        response = self._call_service(self.move_stop_client, request, timeout_sec=3.0)
        if not response.success:
            raise RuntimeError("MoveStop returned success=false")
        self._publish_status(f"MoveStop sent stop_mode={self.stop_mode}")

    def _wait_for_joint_target(self, target: list[float]) -> list[float]:
        deadline = time.monotonic() + self._phase_timeout(target)
        last_pos: Optional[list[float]] = None
        while time.monotonic() < deadline:
            state = self._get_robot_state()
            if state not in self.safe_robot_states:
                raise RuntimeError(f"robot_state={state} while waiting for prep motion.")
            last_pos = self._get_current_posj()
            max_error = max(abs(last_pos[index] - target[index]) for index in range(6))
            if max_error <= self.prepare_joint_tolerance_deg:
                time.sleep(0.5)
                return last_pos
            time.sleep(0.5)
        if last_pos is None:
            raise TimeoutError("prep motion wait did not receive joint state")
        raise TimeoutError(
            f"prep motion timeout: current={last_pos}, target={target}, "
            f"max_error={max(abs(last_pos[index] - target[index]) for index in range(6)):.2f} deg"
        )

    def _wait_for_motion_idle(self) -> None:
        if self.check_motion_client is None or CheckMotion is None:
            return
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            response = self._call_service(
                self.check_motion_client,
                CheckMotion.Request(),
                timeout_sec=2.0,
            )
            if response.success and int(response.status) == 0:
                return
            time.sleep(0.5)

    def _phase_timeout(self, target: list[float]) -> float:
        if self.prepare_time_sec > 0.0:
            return max(12.0, self.prepare_time_sec + 3.0)
        current = self._get_current_posj()
        max_delta = max(abs(target[index] - current[index]) for index in range(6))
        return max(12.0, max_delta / max(abs(self.prepare_vel_deg_per_sec), 1.0) * 4.0 + 6.0)

    def _joint_escape_phases(
        self,
        current: list[float],
        target: list[float],
    ) -> list[tuple[str, list[list[float]]]]:
        pose = current[:]
        phases: list[tuple[str, list[list[float]]]] = []

        if self._needs_signed_escape(pose[4], target[4], self.prepare_j5_escape_deg):
            waypoints = self._single_axis_waypoints(
                pose,
                axis_index=4,
                target_value=self._signed_escape_value(target[4], self.prepare_j5_escape_deg),
                max_step_deg=self.prepare_j5_step_deg,
            )
            if waypoints:
                phases.append(("J5 escape", waypoints))

        if self._needs_signed_escape(pose[2], target[2], self.prepare_j3_escape_deg):
            waypoints = self._single_axis_waypoints(
                pose,
                axis_index=2,
                target_value=self._signed_escape_value(target[2], self.prepare_j3_escape_deg),
                max_step_deg=self.prepare_j3_step_deg,
            )
            if waypoints:
                phases.append(("J3 escape", waypoints))

        parallel_axes = [
            index
            for index in range(6)
            if abs(target[index] - pose[index]) > self.prepare_joint_tolerance_deg
        ]
        parallel_waypoints = []
        while parallel_axes and any(
            abs(target[index] - pose[index]) > self.prepare_joint_tolerance_deg
            for index in parallel_axes
        ):
            next_pose = pose[:]
            for index in parallel_axes:
                delta = target[index] - pose[index]
                next_pose[index] += self._clamp(
                    delta,
                    -self.prepare_max_step_deg,
                    self.prepare_max_step_deg,
                )
            pose[:] = next_pose
            parallel_waypoints.append(pose[:])

        if parallel_waypoints:
            phases.append(("joint final trim", parallel_waypoints))
        return phases

    def _single_axis_waypoints(
        self,
        pose: list[float],
        axis_index: int,
        target_value: float,
        max_step_deg: float,
    ) -> list[list[float]]:
        waypoints = []
        while abs(target_value - pose[axis_index]) > self.prepare_joint_tolerance_deg:
            delta = target_value - pose[axis_index]
            pose[axis_index] += self._clamp(delta, -max_step_deg, max_step_deg)
            waypoints.append(pose[:])
        pose[axis_index] = target_value
        return waypoints

    def _publish_robot_anchor(self, posx: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = _vector6(posx, "robot anchor posx")
        for _ in range(3):
            self.robot_anchor_pub.publish(msg)
            time.sleep(0.02)
        self._publish_status(
            "published robot anchor posx: "
            + json.dumps({"data": list(msg.data)}, sort_keys=True)
        )

    def _publish_teleop_ready(self, ready: bool, reason: str) -> None:
        self.teleop_ready = bool(ready)
        msg = Bool()
        msg.data = self.teleop_ready
        for _ in range(3):
            self.teleop_ready_pub.publish(msg)
            time.sleep(0.01)
        self._publish_status(f"teleop_ready={self.teleop_ready} reason={reason}")

    def _call_recenter_best_effort(self) -> None:
        if not self.recenter_client.wait_for_service(timeout_sec=0.5):
            self._publish_status(
                f"recenter service unavailable: {self.recenter_service_name}",
                warn=True,
            )
            return
        response = self._call_service(
            self.recenter_client,
            Trigger.Request(),
            timeout_sec=2.0,
        )
        self._publish_status(
            f"recenter response success={response.success} message={response.message}",
            warn=not response.success,
        )

    def _set_live(self, enabled: bool) -> None:
        if not self.set_live_client.wait_for_service(timeout_sec=0.5):
            self._publish_status(
                f"set live service unavailable: {self.set_live_service_name}",
                warn=True,
            )
            return
        request = SetBool.Request()
        request.data = bool(enabled)
        response = self._call_service(self.set_live_client, request, timeout_sec=3.0)
        self._publish_status(
            f"set live response success={response.success} message={response.message}",
            warn=not response.success,
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

    def _require_client(self, client, name: str) -> None:
        if client is None:
            raise RuntimeError(f"{name} service type is not available; source Doosan dsr_msgs2.")

    def _publish_status(self, text: str, warn: bool = False) -> None:
        self.status_pub.publish(String(data=text))
        if warn:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)

    @staticmethod
    def _needs_signed_escape(current: float, target: float, escape_abs_deg: float) -> bool:
        return abs(current) < escape_abs_deg and abs(target) >= escape_abs_deg

    @staticmethod
    def _signed_escape_value(target: float, escape_abs_deg: float) -> float:
        return (-1.0 if target < 0.0 else 1.0) * escape_abs_deg

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return min(max(value, low), high)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotPrepNode()
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
