"""Initialize and verify the physical gripper state before recording preflight."""

from __future__ import annotations

import time
from dataclasses import dataclass

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String

from lerobot_robot_doosan_a0509.ros_runtime import RosRuntime


GRIPPER_COMMAND_VALUES = {"open": 0.0, "close": 1.0}


@dataclass
class GripperInitializationState:
    accepted_command: str | None = None
    accepted_time: float | None = None
    completed_command: str | None = None
    completed_time: float | None = None
    commanded_state: float | None = None
    driver_busy: bool | None = None
    last_command_ok: bool | None = None
    busy_true_time: float | None = None

    def already_ready(self, expected_state: float) -> bool:
        return (
            self.commanded_state == expected_state
            and self.driver_busy is False
            and self.last_command_ok is True
        )

    def fresh_command_completed(
        self,
        command: str,
        expected_state: float,
        requested_at: float,
    ) -> bool:
        return (
            self.accepted_command == command
            and self.accepted_time is not None
            and self.accepted_time >= requested_at
            and self.busy_true_time is not None
            and self.busy_true_time >= requested_at
            and self.completed_command == command
            and self.completed_time is not None
            and self.completed_time >= requested_at
            and self.already_ready(expected_state)
        )


class RecordingGripperInitializer:
    def __init__(self) -> None:
        self._runtime = RosRuntime.acquire()
        self._node = self._runtime.create_node(
            "lerobot_a0509_recording_gripper_initializer"
        )
        self._closed = False
        self.state = GripperInitializationState()

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._command_pub = self._node.create_publisher(
            String,
            "/jrt_gripper/cmd",
            10,
        )
        self._node.create_subscription(
            String,
            "/jrt_gripper/accepted_command",
            self._on_accepted,
            state_qos,
        )
        self._node.create_subscription(
            String,
            "/jrt_gripper/completed_command",
            self._on_completed,
            state_qos,
        )
        self._node.create_subscription(
            Float64,
            "/jrt_gripper/commanded_state",
            self._on_commanded_state,
            state_qos,
        )
        self._node.create_subscription(
            Bool,
            "/jrt_gripper/driver_busy",
            self._on_driver_busy,
            state_qos,
        )
        self._node.create_subscription(
            Bool,
            "/jrt_gripper/last_command_ok",
            self._on_last_command_ok,
            state_qos,
        )

    def _on_accepted(self, message: String) -> None:
        self.state.accepted_command = str(message.data).strip().lower()
        self.state.accepted_time = time.monotonic()

    def _on_completed(self, message: String) -> None:
        self.state.completed_command = str(message.data).strip().lower()
        self.state.completed_time = time.monotonic()

    def _on_commanded_state(self, message: Float64) -> None:
        self.state.commanded_state = float(message.data)

    def _on_driver_busy(self, message: Bool) -> None:
        busy = bool(message.data)
        self.state.driver_busy = busy
        if busy:
            self.state.busy_true_time = time.monotonic()

    def _on_last_command_ok(self, message: Bool) -> None:
        self.state.last_command_ok = bool(message.data)

    def _wait_for(self, predicate, deadline: float, failure: str) -> None:
        while not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError(failure)
            time.sleep(0.02)

    def ensure(self, command: str, timeout_sec: float) -> str:
        normalized = str(command).strip().lower()
        if normalized not in GRIPPER_COMMAND_VALUES:
            raise ValueError("recording gripper state must be open or close")
        if timeout_sec <= 0.0:
            raise ValueError("gripper initialization timeout must be positive")

        expected_state = GRIPPER_COMMAND_VALUES[normalized]
        deadline = time.monotonic() + timeout_sec
        self._wait_for(
            lambda: self._command_pub.get_subscription_count() > 0,
            deadline,
            "JRT gripper command subscriber was not discovered",
        )
        self._wait_for(
            lambda: (
                self.state.driver_busy is not None
                and self.state.last_command_ok is not None
            ),
            deadline,
            "JRT gripper diagnostics were not available",
        )
        if self.state.already_ready(expected_state):
            return f"already_{normalized}"

        requested_at = time.monotonic()
        command_message = String(data=normalized)
        publish_attempts = 0
        next_publish_at = requested_at
        while True:
            accepted_fresh = (
                self.state.accepted_command == normalized
                and self.state.accepted_time is not None
                and self.state.accepted_time >= requested_at
            )
            busy_fresh = (
                self.state.busy_true_time is not None
                and self.state.busy_true_time >= requested_at
            )
            if accepted_fresh or busy_fresh:
                break
            now = time.monotonic()
            if now >= deadline:
                raise RuntimeError(
                    f"JRT gripper did not accept {normalized} after "
                    f"{publish_attempts} publish attempts"
                )
            if now >= next_publish_at:
                self._command_pub.publish(command_message)
                publish_attempts += 1
                next_publish_at = now + 0.25
            time.sleep(0.02)

        self._wait_for(
            lambda: self.state.fresh_command_completed(
                normalized,
                expected_state,
                requested_at,
            ),
            deadline,
            (
                f"JRT gripper did not complete verified {normalized}: "
                f"publish_attempts={publish_attempts}, "
                f"accepted={self.state.accepted_command!r}, "
                f"completed={self.state.completed_command!r}, "
                f"commanded_state={self.state.commanded_state!r}, "
                f"busy={self.state.driver_busy!r}, "
                f"last_command_ok={self.state.last_command_ok!r}"
            ),
        )
        return f"initialized_{normalized}"

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._runtime.release_node(self._node)


def ensure_recording_gripper_state(command: str, timeout_sec: float = 10.0) -> str:
    initializer = RecordingGripperInitializer()
    try:
        return initializer.ensure(command, timeout_sec)
    finally:
        initializer.close()
