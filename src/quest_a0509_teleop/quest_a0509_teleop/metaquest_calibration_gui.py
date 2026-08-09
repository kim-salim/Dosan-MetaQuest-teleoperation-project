"""Calibration-only Tk GUI for the MetaQuest-to-A0509 mapper."""

from __future__ import annotations

import json
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Callable

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def parse_calibration_status(text: str) -> dict[str, object]:
    try:
        payload = json.loads(text)
    except (TypeError, json.JSONDecodeError):
        return {"state": "UNKNOWN", "valid": False, "reason": str(text)}
    if not isinstance(payload, dict):
        return {"state": "UNKNOWN", "valid": False, "reason": str(text)}
    return payload


class MetaQuestCalibrationRos(Node):
    def __init__(self, events: "queue.Queue[tuple[str, object]]") -> None:
        super().__init__("metaquest_calibration_gui")
        self.events = events
        self.declare_parameter(
            "calibration_valid_topic", "/vr/metaquest_calibration/valid"
        )
        self.declare_parameter(
            "calibration_status_topic", "/vr/metaquest_calibration/status"
        )
        self.declare_parameter(
            "calibrate_service", "/vr/calibrate_xy_yaw_to_x_plus"
        )
        self.declare_parameter("recenter_service", "/vr/recenter")
        self.declare_parameter(
            "reset_calibration_service", "/vr/reset_xy_yaw_calibration"
        )

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("calibration_valid_topic").value),
            self._on_valid,
            state_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("calibration_status_topic").value),
            self._on_status,
            state_qos,
        )
        # Node.clients is a read-only rclpy property containing all clients.
        # Keep the action lookup under an application-specific name.
        self._service_clients = {
            "Calibrate": self.create_client(
                Trigger, str(self.get_parameter("calibrate_service").value)
            ),
            "Recenter": self.create_client(
                Trigger, str(self.get_parameter("recenter_service").value)
            ),
            "Reset": self.create_client(
                Trigger, str(self.get_parameter("reset_calibration_service").value)
            ),
        }

    def _on_valid(self, msg: Bool) -> None:
        self.events.put(("valid", bool(msg.data)))

    def _on_status(self, msg: String) -> None:
        self.events.put(("status", parse_calibration_status(msg.data)))

    def call(self, action: str, timeout_sec: float = 10.0) -> None:
        self.events.put(("action", f"{action}: running"))

        def work() -> None:
            try:
                message = self._call_trigger(action, timeout_sec)
                self.events.put(("action", f"{action}: {message}"))
            except Exception as exc:
                self.events.put(("action", f"{action}: ERROR: {exc}"))

        threading.Thread(target=work, daemon=True).start()

    def _call_trigger(self, action: str, timeout_sec: float) -> str:
        client = self._service_clients[action]
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(f"service timeout: {client.srv_name}")
            time.sleep(0.02)
        result = future.result()
        if result is None:
            raise RuntimeError(f"service call failed: {client.srv_name}")
        if not result.success:
            raise RuntimeError(str(result.message) or "service returned success=false")
        return str(result.message) or "success"


class MetaQuestCalibrationGui:
    def __init__(
        self,
        root: tk.Tk,
        ros_node: MetaQuestCalibrationRos,
        events: "queue.Queue[tuple[str, object]]",
    ) -> None:
        self.root = root
        self.ros_node = ros_node
        self.events = events
        self.values = {
            "State": tk.StringVar(value="WAITING"),
            "Valid": tk.StringVar(value="false"),
            "Correction": tk.StringVar(value="-"),
            "Observed": tk.StringVar(value="-"),
            "Distance": tk.StringVar(value="-"),
            "Mapping": tk.StringVar(value="-"),
            "Reason": tk.StringVar(value="mapper 상태 수신 대기 중"),
            "Last Action": tk.StringVar(value="idle"),
        }

        root.title("MetaQuest XY Calibration")
        root.geometry("680x430")
        root.minsize(560, 390)
        self._build()
        root.after(100, self._drain_events)

    def _build(self) -> None:
        style = ttk.Style()
        style.configure("Valid.TLabel", foreground="#16803a", font=("TkDefaultFont", 16, "bold"))
        style.configure("Invalid.TLabel", foreground="#b42318", font=("TkDefaultFont", 16, "bold"))
        style.configure("Pending.TLabel", foreground="#9a6700", font=("TkDefaultFont", 16, "bold"))

        frame = ttk.Frame(self.root, padding=14)
        frame.pack(fill=tk.BOTH, expand=True)
        ttk.Label(
            frame,
            text="MetaQuest → Robot +X 회전 보정",
            font=("TkDefaultFont", 15, "bold"),
        ).pack(anchor=tk.W)
        ttk.Label(
            frame,
            text=(
                "이 창은 로봇을 움직이지 않으며 mapper의 보정 서비스만 "
                "호출합니다."
            ),
        ).pack(anchor=tk.W, pady=(2, 12))

        state_frame = ttk.LabelFrame(frame, text="Calibration state", padding=10)
        state_frame.pack(fill=tk.X)
        self.state_label = ttk.Label(
            state_frame,
            textvariable=self.values["State"],
            style="Pending.TLabel",
        )
        self.state_label.grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=(0, 8))
        rows = ("Valid", "Correction", "Observed", "Distance", "Mapping", "Reason")
        for row, key in enumerate(rows, start=1):
            ttk.Label(state_frame, text=key, width=12).grid(
                row=row, column=0, sticky=tk.NW, pady=2
            )
            ttk.Label(
                state_frame,
                textvariable=self.values[key],
                wraplength=500 if key == "Reason" else 0,
            ).grid(row=row, column=1, sticky=tk.W, pady=2)
        state_frame.columnconfigure(1, weight=1)

        controls = ttk.LabelFrame(frame, text="Calibration controls", padding=10)
        controls.pack(fill=tk.X, pady=(12, 0))
        ttk.Button(
            controls,
            text="1. Calibrate XY +X",
            command=self._calibrate,
        ).grid(row=0, column=0, sticky=tk.EW, padx=(0, 5))
        ttk.Button(
            controls,
            text="Recenter VR",
            command=lambda: self.ros_node.call("Recenter"),
        ).grid(row=0, column=1, sticky=tk.EW, padx=5)
        ttk.Button(
            controls,
            text="Reset Calibration",
            command=self._reset,
        ).grid(row=0, column=2, sticky=tk.EW, padx=(5, 0))
        for column in range(3):
            controls.columnconfigure(column, weight=1)

        ttk.Label(
            frame,
            text=(
                "보정: 버튼을 누른 직후 약 2초 동안 Quest 컨트롤러를 실제 "
                "로봇 +X 방향으로 10 cm 이상 곧게 이동하세요. 완료 후 "
                "VALID를 확인하세요."
            ),
            wraplength=640,
        ).pack(fill=tk.X, pady=(10, 0))
        ttk.Separator(frame).pack(fill=tk.X, pady=10)
        ttk.Label(frame, text="Last Action").pack(anchor=tk.W)
        ttk.Label(
            frame,
            textvariable=self.values["Last Action"],
            wraplength=640,
        ).pack(anchor=tk.W, fill=tk.X)

    def _calibrate(self) -> None:
        if messagebox.askokcancel(
            "Calibrate XY +X",
            (
                "OK를 누른 직후 Quest 컨트롤러를 로봇 +X 방향으로 "
                "곧게 이동하세요."
            ),
        ):
            self.ros_node.call("Calibrate", timeout_sec=5.0)

    def _reset(self) -> None:
        if messagebox.askyesno(
            "Reset Calibration",
            (
                "현재 세션 보정을 무효화하고 설정 파일의 초기 회전값으로 "
                "되돌릴까요?"
            ),
        ):
            self.ros_node.call("Reset")

    def _drain_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "status":
                    self._show_status(payload)
                elif kind == "valid":
                    self.values["Valid"].set("true" if payload else "false")
                elif kind == "action":
                    self.values["Last Action"].set(str(payload))
        except queue.Empty:
            pass
        self.root.after(100, self._drain_events)

    def _show_status(self, raw_payload: object) -> None:
        payload = raw_payload if isinstance(raw_payload, dict) else {}
        state = str(payload.get("state", "UNKNOWN"))
        valid = bool(payload.get("valid", False))
        self.values["State"].set(state)
        self.values["Valid"].set("true" if valid else "false")
        self.values["Correction"].set(
            self._number(payload.get("xy_yaw_correction_deg"), " deg")
        )
        self.values["Observed"].set(
            self._number(payload.get("observed_angle_deg"), " deg")
        )
        self.values["Distance"].set(self._number(payload.get("distance_m"), " m"))
        self.values["Mapping"].set(str(payload.get("mapping_fingerprint", "-")))
        self.values["Reason"].set(str(payload.get("reason", "-")))
        if valid:
            style = "Valid.TLabel"
        elif state == "CALIBRATING":
            style = "Pending.TLabel"
        else:
            style = "Invalid.TLabel"
        self.state_label.configure(style=style)

    @staticmethod
    def _number(value: object, suffix: str) -> str:
        if value is None:
            return "-"
        try:
            return f"{float(value):.4f}{suffix}"
        except (TypeError, ValueError):
            return str(value)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    events: "queue.Queue[tuple[str, object]]" = queue.Queue()
    node = MetaQuestCalibrationRos(events)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    MetaQuestCalibrationGui(root, node, events)
    root.protocol("WM_DELETE_WINDOW", root.quit)
    try:
        root.mainloop()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
