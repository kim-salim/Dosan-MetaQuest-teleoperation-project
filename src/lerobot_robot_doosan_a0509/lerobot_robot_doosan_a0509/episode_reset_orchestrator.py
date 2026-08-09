"""Guarded physical reset between A0509 LeRobot recording episodes.

The stock LeRobot recorder implements the inter-episode reset as another timed
``record_loop`` call with ``dataset=None``. This module replaces only that
reset call while leaving recording and dataset save semantics intact.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import functools
import inspect
import logging
import math
import os
import threading
import time
from types import ModuleType
from typing import Any, Callable, Iterable, Sequence

from geometry_msgs.msg import PoseStamped
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from lerobot_robot_doosan_a0509.ros_runtime import RosRuntime


logger = logging.getLogger(__name__)


def _environment_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{name} must be true or false, got {value!r}")


def _environment_positive_float(name: str, default: float) -> float:
    value = float(os.environ.get(name, default))
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be a positive finite number, got {value!r}")
    return value


def _normalize_quaternion(values: Iterable[float]) -> tuple[float, ...]:
    quaternion = tuple(float(value) for value in values)
    if len(quaternion) != 4 or any(not math.isfinite(value) for value in quaternion):
        raise ValueError(f"invalid quaternion: {quaternion}")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-9:
        raise ValueError(f"near-zero quaternion: {quaternion}")
    return tuple(value / norm for value in quaternion)


def quaternion_angle_deg(a: Sequence[float], b: Sequence[float]) -> float:
    """Return the sign-invariant shortest angle between two quaternions."""

    qa = _normalize_quaternion(a)
    qb = _normalize_quaternion(b)
    dot = abs(sum(qa[index] * qb[index] for index in range(4)))
    return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))


@dataclass(frozen=True)
class QuestPoseSample:
    receive_time: float
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


def pose_window_is_stable(
    samples: Sequence[QuestPoseSample],
    *,
    now: float,
    window_sec: float,
    max_age_sec: float,
    max_translation_m: float,
    max_rotation_deg: float,
) -> bool:
    """Return whether fresh Quest samples stayed inside a small pose envelope."""

    if not samples:
        return False
    latest = samples[-1]
    if now - latest.receive_time > max_age_sec:
        return False
    cutoff = now - window_sec
    window = [sample for sample in samples if sample.receive_time >= cutoff]
    if len(window) < 2:
        return False
    if latest.receive_time - window[0].receive_time < window_sec * 0.9:
        return False
    for sample in window:
        translation = math.sqrt(
            sum(
                (sample.position_m[index] - latest.position_m[index]) ** 2
                for index in range(3)
            )
        )
        if translation > max_translation_m:
            return False
        if (
            quaternion_angle_deg(
                sample.quaternion_xyzw,
                latest.quaternion_xyzw,
            )
            > max_rotation_deg
        ):
            return False
    return True


def shortest_angle_delta_deg(value: float, reference: float) -> float:
    return (float(value) - float(reference) + 180.0) % 360.0 - 180.0


def pose_is_near_anchor(
    pose: Sequence[float],
    anchor: Sequence[float],
    *,
    position_limit_mm: float,
    rotation_limit_deg: float,
) -> bool:
    if len(pose) < 6 or len(anchor) < 6:
        return False
    position_norm = math.sqrt(
        sum((float(pose[index]) - float(anchor[index])) ** 2 for index in range(3))
    )
    rotation = [
        shortest_angle_delta_deg(pose[index], anchor[index])
        for index in range(3, 6)
    ]
    return (
        position_norm <= position_limit_mm
        and max(abs(value) for value in rotation) <= rotation_limit_deg
    )


class RecordingControlGate:
    """Share Enter confirmation with LeRobot's existing non-blocking key input."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._operator_ready = threading.Event()
        self._waiting = False
        self._events: dict[str, bool] | None = None
        self._listener: Any | None = None

    def init_keyboard_listener(self):
        from lerobot.utils.keyboard_input import (
            apply_recording_control,
            create_key_listener,
        )

        events = {
            "exit_early": False,
            "rerecord_episode": False,
            "stop_recording": False,
        }

        def on_key(name: str) -> None:
            key = name.lower()
            with self._lock:
                waiting = self._waiting
            if waiting:
                if key == "enter":
                    self._operator_ready.set()
                elif key in {"esc", "q"}:
                    apply_recording_control("esc", events)
                    self._operator_ready.set()
                return
            if key in {"right", "n"}:
                apply_recording_control("right", events)
            elif key in {"left", "r"}:
                apply_recording_control("left", events)
            elif key in {"esc", "q"}:
                apply_recording_control("esc", events)

        listener = create_key_listener(
            on_key,
            controls_help=(
                "recording: n=next, r=re-record, q=quit; "
                "reset prompts: Enter=confirm"
            ),
        )
        self._events = events
        self._listener = listener
        return listener, events

    def wait_for_operator(self, message: str) -> None:
        if self._events is None:
            raise RuntimeError("recording keyboard listener has not been initialized")
        if self._listener is None:
            raise RuntimeError(
                "episode reset requires an interactive keyboard; no usable TTY/display "
                "listener is available"
            )
        self._operator_ready.clear()
        with self._lock:
            self._waiting = True
        print(f"\n{message}\n확인되면 Enter를 누르세요. (중단: q 또는 Esc)", flush=True)
        try:
            while not self._operator_ready.wait(timeout=0.05):
                if self._events["stop_recording"]:
                    raise KeyboardInterrupt("episode reset cancelled")
            if self._events["stop_recording"]:
                raise KeyboardInterrupt("episode reset cancelled")
        finally:
            with self._lock:
                self._waiting = False


@dataclass(frozen=True)
class EpisodeResetConfig:
    prepare_service_timeout_sec: float = 180.0
    service_timeout_sec: float = 10.0
    state_timeout_sec: float = 8.0
    quest_stability_timeout_sec: float = 12.0
    quest_stability_window_sec: float = 0.5
    quest_max_age_sec: float = 0.3
    quest_stable_translation_m: float = 0.008
    quest_stable_rotation_deg: float = 5.0
    preflight_position_limit_mm: float = 10.0
    preflight_rotation_limit_deg: float = 3.0
    preflight_settle_sec: float = 0.35
    initial_gripper_state: str = "open"
    reset_before_first_episode: bool = True

    @classmethod
    def from_environment(cls) -> "EpisodeResetConfig":
        initial_gripper_state = os.environ.get(
            "EPISODE_INITIAL_GRIPPER_STATE", "open"
        ).strip().lower()
        if initial_gripper_state not in {"open", "close", "none"}:
            raise ValueError(
                "EPISODE_INITIAL_GRIPPER_STATE must be open, close, or none"
            )
        return cls(
            prepare_service_timeout_sec=_environment_positive_float(
                "EPISODE_PREPARE_TIMEOUT_SEC", 180.0
            ),
            service_timeout_sec=_environment_positive_float(
                "EPISODE_SERVICE_TIMEOUT_SEC", 10.0
            ),
            state_timeout_sec=_environment_positive_float(
                "EPISODE_STATE_TIMEOUT_SEC", 8.0
            ),
            quest_stability_timeout_sec=_environment_positive_float(
                "EPISODE_QUEST_STABILITY_TIMEOUT_SEC", 12.0
            ),
            quest_stability_window_sec=_environment_positive_float(
                "EPISODE_QUEST_STABILITY_WINDOW_SEC", 0.5
            ),
            quest_max_age_sec=_environment_positive_float(
                "EPISODE_QUEST_MAX_AGE_SEC", 0.3
            ),
            quest_stable_translation_m=_environment_positive_float(
                "EPISODE_QUEST_STABLE_TRANSLATION_M", 0.008
            ),
            quest_stable_rotation_deg=_environment_positive_float(
                "EPISODE_QUEST_STABLE_ROTATION_DEG", 5.0
            ),
            preflight_position_limit_mm=_environment_positive_float(
                "EPISODE_PREFLIGHT_POSITION_LIMIT_MM", 10.0
            ),
            preflight_rotation_limit_deg=_environment_positive_float(
                "EPISODE_PREFLIGHT_ROTATION_LIMIT_DEG", 3.0
            ),
            preflight_settle_sec=_environment_positive_float(
                "EPISODE_PREFLIGHT_SETTLE_SEC", 0.35
            ),
            initial_gripper_state=initial_gripper_state,
            reset_before_first_episode=_environment_bool(
                "EPISODE_RESET_BEFORE_FIRST", True
            ),
        )


class EpisodeResetOrchestrator:
    """ROS-side state machine that produces one armed, reproducible start."""

    def __init__(self, config: EpisodeResetConfig) -> None:
        self.config = config
        self._runtime = RosRuntime.acquire()
        self._node = self._runtime.create_node("lerobot_a0509_episode_reset")
        self._closed = False

        self.calibration_valid: bool | None = None
        self.teleop_ready: bool | None = None
        self.source: str | None = None
        self.selected_valid: bool | None = None
        self.live_state: bool | None = None
        self.last_pose_heartbeat: float | None = None
        self.last_selected_heartbeat: float | None = None
        self.anchor: tuple[float, ...] | None = None
        self.anchor_time: float | None = None
        self.safe: tuple[float, ...] | None = None
        self.safe_time: float | None = None
        self.gripper_busy: bool | None = None
        self.gripper_last_command_ok: bool | None = None
        self.gripper_commanded_state: float | None = None
        self.last_gripper_accepted: str | None = None
        self.last_gripper_accepted_time: float | None = None
        self.last_gripper_completed: str | None = None
        self.last_gripper_completed_time: float | None = None
        self.quest_samples: deque[QuestPoseSample] = deque(maxlen=300)

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscriptions = [
            self._node.create_subscription(
                Bool,
                "/vr/metaquest_calibration/valid",
                lambda msg: setattr(self, "calibration_valid", bool(msg.data)),
                state_qos,
            ),
            self._node.create_subscription(
                Bool,
                "/vr/teleop_ready",
                lambda msg: setattr(self, "teleop_ready", bool(msg.data)),
                state_qos,
            ),
            self._node.create_subscription(
                String,
                "/control/source",
                lambda msg: setattr(self, "source", str(msg.data)),
                state_qos,
            ),
            self._node.create_subscription(
                Bool,
                "/control/selected_command_valid",
                lambda msg: setattr(self, "selected_valid", bool(msg.data)),
                state_qos,
            ),
            self._node.create_subscription(
                Bool,
                "/vr/live_robot_output_enabled",
                lambda msg: setattr(self, "live_state", bool(msg.data)),
                state_qos,
            ),
            self._node.create_subscription(
                Empty,
                "/control/metaquest/valid_pose_heartbeat",
                lambda _msg: setattr(self, "last_pose_heartbeat", time.monotonic()),
                10,
            ),
            self._node.create_subscription(
                Empty,
                "/control/selected_command_heartbeat",
                lambda _msg: setattr(
                    self, "last_selected_heartbeat", time.monotonic()
                ),
                10,
            ),
            self._node.create_subscription(
                Float64MultiArray,
                "/vr/robot_anchor_posx",
                self._on_anchor,
                state_qos,
            ),
            self._node.create_subscription(
                Float64MultiArray,
                "/vr/safe_posx",
                self._on_safe,
                10,
            ),
            self._node.create_subscription(
                Bool,
                "/jrt_gripper/driver_busy",
                lambda msg: setattr(self, "gripper_busy", bool(msg.data)),
                state_qos,
            ),
            self._node.create_subscription(
                Bool,
                "/jrt_gripper/last_command_ok",
                lambda msg: setattr(
                    self, "gripper_last_command_ok", bool(msg.data)
                ),
                state_qos,
            ),
            self._node.create_subscription(
                Float64,
                "/jrt_gripper/commanded_state",
                lambda msg: setattr(
                    self, "gripper_commanded_state", float(msg.data)
                ),
                state_qos,
            ),
            self._node.create_subscription(
                String,
                "/jrt_gripper/accepted_command",
                self._on_gripper_accepted,
                state_qos,
            ),
            self._node.create_subscription(
                String,
                "/jrt_gripper/completed_command",
                self._on_gripper_completed,
                state_qos,
            ),
            self._node.create_subscription(
                PoseStamped,
                "/q2r_right_hand_pose",
                self._on_quest_pose,
                pose_qos,
            ),
        ]

        self._set_live_client = self._node.create_client(
            SetBool, "/vr/set_live_robot_output"
        )
        self._select_disabled_client = self._node.create_client(
            Trigger, "/control/select_disabled"
        )
        self._select_metaquest_client = self._node.create_client(
            Trigger, "/control/select_metaquest"
        )
        self._prepare_client = self._node.create_client(
            Trigger, "/vr/prepare_robot"
        )
        self._recenter_client = self._node.create_client(
            Trigger, "/vr/recenter"
        )
        self._gripper_pub = self._node.create_publisher(
            String, "/jrt_gripper/cmd", 10
        )

    def _on_anchor(self, message: Float64MultiArray) -> None:
        if len(message.data) >= 6:
            self.anchor = tuple(float(value) for value in message.data[:6])
            self.anchor_time = time.monotonic()

    def _on_safe(self, message: Float64MultiArray) -> None:
        if len(message.data) >= 6:
            self.safe = tuple(float(value) for value in message.data[:6])
            self.safe_time = time.monotonic()

    def _on_gripper_accepted(self, message: String) -> None:
        self.last_gripper_accepted = str(message.data).strip().lower()
        self.last_gripper_accepted_time = time.monotonic()

    def _on_gripper_completed(self, message: String) -> None:
        self.last_gripper_completed = str(message.data).strip().lower()
        self.last_gripper_completed_time = time.monotonic()

    def _on_quest_pose(self, message: PoseStamped) -> None:
        try:
            position = (
                float(message.pose.position.x),
                float(message.pose.position.y),
                float(message.pose.position.z),
            )
            if any(not math.isfinite(value) for value in position):
                return
            quaternion = _normalize_quaternion(
                (
                    message.pose.orientation.x,
                    message.pose.orientation.y,
                    message.pose.orientation.z,
                    message.pose.orientation.w,
                )
            )
        except (TypeError, ValueError):
            return
        self.quest_samples.append(
            QuestPoseSample(
                receive_time=time.monotonic(),
                position_m=position,
                quaternion_xyzw=quaternion,
            )
        )

    def _wait_for(
        self,
        predicate: Callable[[], bool],
        timeout_sec: float,
        label: str,
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        raise TimeoutError(label)

    def _call(self, client: Any, request: Any, *, timeout_sec: float) -> Any:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        self._wait_for(future.done, timeout_sec, f"service timeout: {client.srv_name}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"service returned no response: {client.srv_name}")
        return result

    def _trigger(self, client: Any, *, timeout_sec: float | None = None) -> str:
        result = self._call(
            client,
            Trigger.Request(),
            timeout_sec=(
                self.config.service_timeout_sec
                if timeout_sec is None
                else timeout_sec
            ),
        )
        if not result.success:
            raise RuntimeError(f"{client.srv_name}: {result.message}")
        return str(result.message)

    def _set_live(self, enabled: bool) -> str:
        request = SetBool.Request()
        request.data = bool(enabled)
        result = self._call(
            self._set_live_client,
            request,
            timeout_sec=self.config.service_timeout_sec,
        )
        if not result.success:
            raise RuntimeError(result.message)
        return str(result.message)

    def _fresh(self, timestamp: float | None, max_age_sec: float) -> bool:
        return timestamp is not None and time.monotonic() - timestamp <= max_age_sec

    def force_safe(self) -> None:
        """Synchronously establish Live OFF, DISABLED, and idle gripper output."""

        errors: list[str] = []
        try:
            self._set_live(False)
        except Exception as exc:
            errors.append(f"Live OFF failed: {exc}")
        try:
            self._trigger(self._select_disabled_client)
        except Exception as exc:
            errors.append(f"MUX DISABLED failed: {exc}")
        self._gripper_pub.publish(String(data="stop"))
        try:
            self._wait_for(
                lambda: self.live_state is False and self.source == "DISABLED",
                self.config.state_timeout_sec,
                "Live OFF / MUX DISABLED state confirmation timeout",
            )
            self._wait_for(
                lambda: (
                    self.gripper_busy is False
                    and self.gripper_last_command_ok is True
                ),
                self.config.state_timeout_sec,
                "gripper stop/idle confirmation timeout",
            )
        except Exception as exc:
            errors.append(str(exc))
        if errors:
            raise RuntimeError("; ".join(errors))
        logger.info("Episode reset state: SAFE (Live OFF, source DISABLED, gripper idle)")

    def _wait_for_stable_quest(self) -> None:
        self._wait_for(
            lambda: pose_window_is_stable(
                tuple(self.quest_samples),
                now=time.monotonic(),
                window_sec=self.config.quest_stability_window_sec,
                max_age_sec=self.config.quest_max_age_sec,
                max_translation_m=self.config.quest_stable_translation_m,
                max_rotation_deg=self.config.quest_stable_rotation_deg,
            ),
            self.config.quest_stability_timeout_sec,
            (
                "Quest controller did not remain fresh and stable for "
                f"{self.config.quest_stability_window_sec:.2f}s"
            ),
        )

    def _initialize_gripper(self) -> None:
        command = self.config.initial_gripper_state
        if command == "none":
            return
        time.sleep(0.35)
        requested_at = time.monotonic()
        self._gripper_pub.publish(String(data=command))
        expected_state = 0.0 if command == "open" else 1.0
        self._wait_for(
            lambda: (
                self.last_gripper_accepted == command
                and self.last_gripper_accepted_time is not None
                and self.last_gripper_accepted_time >= requested_at
            ),
            self.config.state_timeout_sec,
            f"gripper {command} was not accepted",
        )
        self._wait_for(
            lambda: (
                self.last_gripper_completed == command
                and self.last_gripper_completed_time is not None
                and self.last_gripper_completed_time >= requested_at
                and self.gripper_busy is False
                and self.gripper_last_command_ok is True
                and self.gripper_commanded_state == expected_state
            ),
            self.config.state_timeout_sec,
            f"gripper {command} did not complete",
        )
        logger.info("Episode reset gripper initialized: %s", command)

    def prepare_next_episode(self, gate: RecordingControlGate) -> None:
        """Move home, let the operator reset, recenter, and preflight the source."""

        print("\nEPISODE_RESET_STATE=STOPPING", flush=True)
        self.force_safe()
        gate.wait_for_operator(
            "[1/2] 작업 공간에서 손과 장애물을 치워주세요. "
            "확인 후 로봇이 준비 자세로 이동합니다."
        )

        print("EPISODE_RESET_STATE=PREPARING_ROBOT", flush=True)
        prepare_requested_at = time.monotonic()
        prepare_message = self._trigger(
            self._prepare_client,
            timeout_sec=self.config.prepare_service_timeout_sec,
        )
        self._wait_for(
            lambda: (
                self.teleop_ready is True
                and self.anchor is not None
                and self.anchor_time is not None
                and self.anchor_time >= prepare_requested_at
            ),
            self.config.state_timeout_sec,
            "fresh robot anchor / teleop_ready confirmation timeout after prepare",
        )
        logger.info("Robot preparation completed: %s", prepare_message)

        gate.wait_for_operator(
            "[2/2] 물체와 작업 환경을 초기화하세요. 완료 후 Quest 컨트롤러를 "
            "편한 중립 자세로 들고 움직이지 마세요."
        )
        print("EPISODE_RESET_STATE=RECENTERING", flush=True)
        self._wait_for_stable_quest()
        if self.calibration_valid is not True:
            raise RuntimeError(
                "MetaQuest XY/Yaw calibration is not VALID; run the calibration GUI"
            )
        recenter_requested_at = time.monotonic()
        recenter_message = self._trigger(self._recenter_client)
        logger.info("Quest recenter completed: %s", recenter_message)
        self._wait_for(
            lambda: (
                self.teleop_ready is True
                and self.calibration_valid is True
                and self._fresh(
                    self.last_pose_heartbeat,
                    self.config.quest_max_age_sec,
                )
            ),
            self.config.state_timeout_sec,
            "fresh calibrated Quest heartbeat timeout after recenter",
        )

        print("EPISODE_RESET_STATE=INITIALIZING_GRIPPER", flush=True)
        self._initialize_gripper()

        print("EPISODE_RESET_STATE=PREFLIGHT", flush=True)
        selected_at = time.monotonic()
        self._trigger(self._select_metaquest_client)
        self._wait_for(
            lambda: (
                self.source == "METAQUEST"
                and self.selected_valid is True
                and self.safe is not None
                and self.safe_time is not None
                and self.safe_time
                >= max(recenter_requested_at, selected_at)
                + self.config.preflight_settle_sec
                and self._fresh(
                    self.last_selected_heartbeat,
                    self.config.quest_max_age_sec,
                )
            ),
            self.config.state_timeout_sec,
            "fresh selected MetaQuest target timeout",
        )
        if self.anchor is None or self.safe is None:
            raise RuntimeError("robot anchor or safe target is unavailable")
        self._wait_for(
            lambda: (
                self.anchor is not None
                and self.safe is not None
                and pose_is_near_anchor(
                    self.safe,
                    self.anchor,
                    position_limit_mm=self.config.preflight_position_limit_mm,
                    rotation_limit_deg=self.config.preflight_rotation_limit_deg,
                )
            ),
            self.config.state_timeout_sec,
            "safe target did not converge to the recentered robot anchor",
        )
        self._wait_for(
            lambda: (
                self.gripper_busy is False
                and self.gripper_last_command_ok is True
            ),
            self.config.state_timeout_sec,
            "gripper did not return idle after source selection",
        )
        print("EPISODE_RESET_STATE=READY", flush=True)

    def enable_live_for_recording(self) -> None:
        if self.calibration_valid is not True:
            raise RuntimeError("refusing Live ON: MetaQuest calibration is invalid")
        if self.teleop_ready is not True:
            raise RuntimeError("refusing Live ON: teleop_ready=false")
        if self.source != "METAQUEST" or self.selected_valid is not True:
            raise RuntimeError("refusing Live ON: MetaQuest source is not fresh/selected")
        if not self._fresh(self.last_selected_heartbeat, self.config.quest_max_age_sec):
            raise RuntimeError("refusing Live ON: selected MetaQuest heartbeat is stale")
        self._set_live(True)
        self._wait_for(
            lambda: self.live_state is True,
            self.config.state_timeout_sec,
            "Live ON state confirmation timeout",
        )
        print("EPISODE_RESET_STATE=RECORDING", flush=True)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._runtime.release_node(self._node)


class EpisodeResetRecordingHook:
    """Process-local hook around LeRobot's record/reset loop calls."""

    def __init__(
        self,
        record_module: ModuleType | Any,
        *,
        config: EpisodeResetConfig,
    ) -> None:
        self.record_module = record_module
        self.config = config
        self.gate = RecordingControlGate()
        self._orchestrator: EpisodeResetOrchestrator | None = None
        self._ready_for_recording = False
        self._closed = False
        self._original_record_loop = record_module.record_loop
        self._original_init_keyboard_listener = record_module.init_keyboard_listener

    @property
    def orchestrator(self) -> EpisodeResetOrchestrator:
        if self._orchestrator is None:
            self._orchestrator = EpisodeResetOrchestrator(self.config)
        return self._orchestrator

    def install(self) -> "EpisodeResetRecordingHook":
        current = self.record_module.record_loop
        if getattr(current, "_a0509_episode_reset", False):
            raise RuntimeError("A0509 episode reset hook is already installed")
        signature = inspect.signature(current)

        @functools.wraps(current)
        def episode_aware_record_loop(*args: Any, **kwargs: Any) -> Any:
            bound = signature.bind_partial(*args, **kwargs)
            dataset = bound.arguments.get("dataset")
            if dataset is None:
                self.orchestrator.prepare_next_episode(self.gate)
                self._ready_for_recording = True
                return None

            if not self._ready_for_recording:
                if self.config.reset_before_first_episode:
                    self.orchestrator.prepare_next_episode(self.gate)
                else:
                    logger.warning(
                        "First-episode physical reset is disabled; using current state"
                    )
                self._ready_for_recording = True

            self.orchestrator.enable_live_for_recording()
            self._ready_for_recording = False
            try:
                return current(*args, **kwargs)
            finally:
                self.orchestrator.force_safe()

        episode_aware_record_loop._a0509_episode_reset = True
        episode_aware_record_loop._a0509_original_record_loop = current
        self.record_module.record_loop = episode_aware_record_loop
        self.record_module.init_keyboard_listener = self.gate.init_keyboard_listener
        logger.info(
            "Installed guarded A0509 per-episode reset; stock reset_time_s is ignored"
        )
        return self

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._orchestrator is not None:
            try:
                self._orchestrator.force_safe()
            except Exception:
                logger.exception("Failed to confirm safe state while closing episode hook")
            finally:
                self._orchestrator.close()
        self.record_module.record_loop = self._original_record_loop
        self.record_module.init_keyboard_listener = (
            self._original_init_keyboard_listener
        )


def install_episode_reset_recording(
    record_module: ModuleType | Any,
    *,
    config: EpisodeResetConfig | None = None,
) -> EpisodeResetRecordingHook:
    hook = EpisodeResetRecordingHook(
        record_module,
        config=config or EpisodeResetConfig.from_environment(),
    )
    return hook.install()
