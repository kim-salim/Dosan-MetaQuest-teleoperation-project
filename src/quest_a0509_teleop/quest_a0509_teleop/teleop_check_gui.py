"""Small Tkinter control panel for the Quest A0509 teleop check sequence."""

from __future__ import annotations

import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk
from typing import Callable

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger


class TeleopCheckRos(Node):
    def __init__(self, event_queue: "queue.Queue[tuple[str, str, object]]") -> None:
        super().__init__("teleop_check_gui")
        self.events = event_queue
        self._declare_parameters()

        self.status_topic = self.get_parameter("status_topic").value
        self.input_pose_topic = self.get_parameter("input_pose_topic").value
        self.target_posx_topic = self.get_parameter("target_posx_topic").value
        self.safe_posx_topic = self.get_parameter("safe_posx_topic").value
        self.robot_anchor_posx_topic = self.get_parameter("robot_anchor_posx_topic").value
        self.teleop_ready_topic = self.get_parameter("teleop_ready_topic").value

        self.trigger_clients = {
            "Recenter VR": self.create_client(Trigger, self.get_parameter("recenter_service").value),
            "Prepare Robot": self.create_client(Trigger, self.get_parameter("prepare_service").value),
            "Anchor = Current TCP": self.create_client(
                Trigger,
                self.get_parameter("set_anchor_service").value,
            ),
            "Calibrate XY +X": self.create_client(
                Trigger,
                self.get_parameter("calibrate_xy_yaw_service").value,
            ),
            "Start RT Control": self.create_client(
                Trigger,
                self.get_parameter("start_rt_service").value,
            ),
            "Stop RT Control": self.create_client(
                Trigger,
                self.get_parameter("stop_rt_service").value,
            ),
            "Hold ServoL": self.create_client(Trigger, self.get_parameter("hold_service").value),
            "Stop Robot": self.create_client(Trigger, self.get_parameter("stop_robot_service").value),
            "Reset SAFE_OFF": self.create_client(
                Trigger,
                self.get_parameter("reset_safe_off_service").value,
            ),
        }
        self.set_live_client = self.create_client(
            SetBool,
            self.get_parameter("set_live_service").value,
        )

        self.create_subscription(String, self.status_topic, self._on_status, 50)
        self.create_subscription(PoseStamped, self.input_pose_topic, self._on_pose, 10)
        self.create_subscription(Float64MultiArray, self.target_posx_topic, self._on_target, 10)
        self.create_subscription(Float64MultiArray, self.safe_posx_topic, self._on_safe, 10)
        self.create_subscription(
            Float64MultiArray,
            self.robot_anchor_posx_topic,
            self._on_anchor,
            10,
        )
        self.create_subscription(Bool, self.teleop_ready_topic, self._on_ready, 10)

    def _declare_parameters(self) -> None:
        self.declare_parameter("status_topic", "/vr/status")
        self.declare_parameter("input_pose_topic", "/q2r_right_hand_pose")
        self.declare_parameter("target_posx_topic", "/vr/target_posx")
        self.declare_parameter("safe_posx_topic", "/vr/safe_posx")
        self.declare_parameter("robot_anchor_posx_topic", "/vr/robot_anchor_posx")
        self.declare_parameter("teleop_ready_topic", "/vr/teleop_ready")
        self.declare_parameter("recenter_service", "/vr/recenter")
        self.declare_parameter("prepare_service", "/vr/prepare_robot")
        self.declare_parameter("set_anchor_service", "/vr/set_robot_anchor_to_current_tcp")
        self.declare_parameter("calibrate_xy_yaw_service", "/vr/calibrate_xy_yaw_to_x_plus")
        self.declare_parameter("start_rt_service", "/vr/start_rt_control")
        self.declare_parameter("stop_rt_service", "/vr/stop_rt_control")
        self.declare_parameter("hold_service", "/vr/hold_servol")
        self.declare_parameter("stop_robot_service", "/vr/stop_robot")
        self.declare_parameter("reset_safe_off_service", "/vr/reset_safe_off")
        self.declare_parameter("set_live_service", "/vr/set_live_robot_output")

    def _on_status(self, msg: String) -> None:
        self.events.put(("status", "Status", msg.data))

    def _on_pose(self, msg: PoseStamped) -> None:
        values = [
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        ]
        self.events.put(("topic", "Quest Pose", values))

    def _on_target(self, msg: Float64MultiArray) -> None:
        self.events.put(("topic", "Target PosX", list(msg.data)))

    def _on_safe(self, msg: Float64MultiArray) -> None:
        self.events.put(("topic", "Safe PosX", list(msg.data)))

    def _on_anchor(self, msg: Float64MultiArray) -> None:
        self.events.put(("topic", "Robot Anchor", list(msg.data)))

    def _on_ready(self, msg: Bool) -> None:
        self.events.put(("topic", "Teleop Ready", bool(msg.data)))

    def call_trigger(self, label: str, timeout_sec: float = 10.0) -> None:
        client = self.trigger_clients[label]
        self._run_service_thread(
            label,
            lambda: self._call_trigger(client, label, timeout_sec),
        )

    def set_live(self, enabled: bool, timeout_sec: float = 10.0) -> None:
        label = "Enable Live ServoL" if enabled else "Disable Live ServoL"
        self._run_service_thread(
            label,
            lambda: self._call_set_live(enabled, timeout_sec),
        )

    def _run_service_thread(self, label: str, target: Callable[[], str]) -> None:
        self.events.put(("action", label, "running"))

        def work() -> None:
            try:
                message = target()
                self.events.put(("action", label, message))
            except Exception as exc:
                self.events.put(("action", label, f"ERROR: {exc}"))

        thread = threading.Thread(target=work, daemon=True)
        thread.start()

    def _call_trigger(self, client, label: str, timeout_sec: float) -> str:
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        future = client.call_async(Trigger.Request())
        result = self._wait_future(future, timeout_sec)
        success = bool(result.success)
        text = str(result.message)
        if not success:
            raise RuntimeError(text or f"{label} returned success=false")
        return text or "success"

    def _call_set_live(self, enabled: bool, timeout_sec: float) -> str:
        if not self.set_live_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"service unavailable: {self.set_live_client.srv_name}")
        request = SetBool.Request()
        request.data = bool(enabled)
        future = self.set_live_client.call_async(request)
        result = self._wait_future(future, timeout_sec)
        success = bool(result.success)
        text = str(result.message)
        if not success:
            raise RuntimeError(text or "Set live output returned success=false")
        return text or "success"

    @staticmethod
    def _wait_future(future, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError("service timeout")
            time.sleep(0.02)
        result = future.result()
        if result is None:
            raise RuntimeError("service call failed")
        return result


class TeleopCheckGui:
    def __init__(self, root: tk.Tk, ros_node: TeleopCheckRos, events) -> None:
        self.root = root
        self.ros_node = ros_node
        self.events = events
        self.real_actions_enabled = tk.BooleanVar(value=False)
        self.topic_vars = {
            "Quest Pose": tk.StringVar(value="waiting..."),
            "Target PosX": tk.StringVar(value="waiting..."),
            "Safe PosX": tk.StringVar(value="waiting..."),
            "Robot Anchor": tk.StringVar(value="waiting..."),
            "Teleop Ready": tk.StringVar(value="false"),
            "Last Action": tk.StringVar(value="idle"),
        }
        self.danger_buttons: list[ttk.Button] = []

        self.root.title("Quest A0509 Teleop Check")
        self.root.geometry("920x650")
        self._build()
        self._update_danger_buttons()
        self.root.after(100, self._drain_events)

    def _build(self) -> None:
        style = ttk.Style()
        try:
            style.configure("Danger.TButton", foreground="red")
        except tk.TclError:
            pass

        top = ttk.Frame(self.root, padding=10)
        top.pack(fill=tk.BOTH, expand=True)

        monitor = ttk.LabelFrame(top, text="Monitor", padding=8)
        monitor.pack(fill=tk.X)
        for row, key in enumerate(
            ("Quest Pose", "Target PosX", "Safe PosX", "Robot Anchor", "Teleop Ready")
        ):
            ttk.Label(monitor, text=key, width=14).grid(row=row, column=0, sticky=tk.W, pady=2)
            ttk.Label(monitor, textvariable=self.topic_vars[key]).grid(
                row=row,
                column=1,
                sticky=tk.W,
                pady=2,
            )
        ttk.Label(monitor, text="Last Action", width=14).grid(row=5, column=0, sticky=tk.W, pady=2)
        ttk.Label(monitor, textvariable=self.topic_vars["Last Action"]).grid(
            row=5,
            column=1,
            sticky=tk.W,
            pady=2,
        )
        monitor.columnconfigure(1, weight=1)

        controls = ttk.Frame(top)
        controls.pack(fill=tk.X, pady=(10, 0))

        setup = ttk.LabelFrame(controls, text="Setup", padding=8)
        setup.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 6))
        self._button(setup, "Recenter VR", lambda: self.ros_node.call_trigger("Recenter VR"))
        self._button(
            setup,
            "Calibrate XY +X",
            lambda: self._calibrate_xy_yaw_action(),
        )
        self._button(
            setup,
            "Anchor = Current TCP",
            lambda: self._danger_action(
                "Anchor to current robot TCP?",
                lambda: self.ros_node.call_trigger("Anchor = Current TCP"),
            ),
            dangerous=True,
        )
        self._button(
            setup,
            "Prepare Robot",
            lambda: self._danger_action(
                "Move robot to the prep joint pose?",
                lambda: self.ros_node.call_trigger("Prepare Robot", timeout_sec=180.0),
            ),
            dangerous=True,
        )

        rt = ttk.LabelFrame(controls, text="RT / Live", padding=8)
        rt.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=6)
        self._button(
            rt,
            "Start RT Control",
            lambda: self._danger_action(
                "Call StartRtControl?",
                lambda: self.ros_node.call_trigger("Start RT Control"),
            ),
            dangerous=True,
        )
        self._button(rt, "Stop RT Control", lambda: self.ros_node.call_trigger("Stop RT Control"))
        self._button(
            rt,
            "Enable Live ServoL",
            lambda: self._danger_action(
                "Enable live ServoL RT output to the robot?",
                lambda: self.ros_node.set_live(True),
            ),
            dangerous=True,
            style="Danger.TButton",
        )
        self._button(rt, "Disable Live ServoL", lambda: self.ros_node.set_live(False))

        safety = ttk.LabelFrame(controls, text="Safety", padding=8)
        safety.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(6, 0))
        self._button(safety, "Hold ServoL", lambda: self.ros_node.call_trigger("Hold ServoL"))
        self._button(safety, "Stop Robot", lambda: self.ros_node.call_trigger("Stop Robot"), style="Danger.TButton")
        self._button(
            safety,
            "Reset SAFE_OFF",
            lambda: self._danger_action(
                "Request RESET_SAFE_OFF?",
                lambda: self.ros_node.call_trigger("Reset SAFE_OFF"),
            ),
            dangerous=True,
        )

        gate = ttk.Checkbutton(
            top,
            text="Enable real-action buttons",
            variable=self.real_actions_enabled,
            command=self._update_danger_buttons,
        )
        gate.pack(anchor=tk.W, pady=(8, 0))

        log_frame = ttk.LabelFrame(top, text="Status Log", padding=8)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        self.log_text = scrolledtext.ScrolledText(log_frame, height=16, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def _button(
        self,
        parent,
        text: str,
        command: Callable[[], None],
        dangerous: bool = False,
        style: str | None = None,
    ) -> ttk.Button:
        kwargs = {"text": text, "command": command}
        if style:
            kwargs["style"] = style
        button = ttk.Button(parent, **kwargs)
        button.pack(fill=tk.X, pady=3)
        if dangerous:
            self.danger_buttons.append(button)
        return button

    def _danger_action(self, prompt: str, action: Callable[[], None]) -> None:
        if not self.real_actions_enabled.get():
            self._append_log("Blocked: enable real-action buttons first.")
            return
        if not messagebox.askyesno("Confirm", prompt):
            self._append_log("Cancelled.")
            return
        action()

    def _calibrate_xy_yaw_action(self) -> None:
        if not messagebox.askyesno(
            "Calibrate XY +X",
            "After pressing Yes, move the Quest controller toward robot +X for 2 seconds.",
        ):
            self._append_log("Cancelled.")
            return
        self.ros_node.call_trigger("Calibrate XY +X", timeout_sec=5.0)

    def _update_danger_buttons(self) -> None:
        state = tk.NORMAL if self.real_actions_enabled.get() else tk.DISABLED
        for button in self.danger_buttons:
            button.configure(state=state)

    def _drain_events(self) -> None:
        try:
            while True:
                kind, label, payload = self.events.get_nowait()
                if kind == "topic":
                    self.topic_vars[label].set(self._format_topic(label, payload))
                elif kind == "status":
                    self._append_log(str(payload))
                elif kind == "action":
                    self.topic_vars["Last Action"].set(f"{label}: {payload}")
                    self._append_log(f"{label}: {payload}")
        except queue.Empty:
            pass
        self.root.after(100, self._drain_events)

    def _append_log(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{stamp}] {text}\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    @staticmethod
    def _format_topic(label: str, payload: object) -> str:
        if label == "Teleop Ready":
            return "true" if bool(payload) else "false"
        values = [float(value) for value in payload]  # type: ignore[arg-type]
        if label == "Quest Pose":
            return "x={:.4f} m, y={:.4f} m, z={:.4f} m".format(*values[:3])
        if len(values) >= 6:
            return (
                "x={:.1f}, y={:.1f}, z={:.1f} mm | "
                "rx={:.1f}, ry={:.1f}, rz={:.1f} deg"
            ).format(*values[:6])
        return str(values)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    events: "queue.Queue[tuple[str, str, object]]" = queue.Queue()
    node = TeleopCheckRos(events)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    TeleopCheckGui(root, node, events)

    def on_close() -> None:
        root.quit()

    root.protocol("WM_DELETE_WINDOW", on_close)
    try:
        root.mainloop()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
