"""LeRobot Robot adapter for the existing MUX-first Doosan A0509 ROS stack."""

from __future__ import annotations

import math
import re
import time
from typing import Any, Iterable

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.robot import Robot
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray

from lerobot_robot_doosan_a0509.config_doosan_a0509_ros import DoosanA0509RosConfig
from lerobot_robot_doosan_a0509.ros_runtime import RosRuntime
from lerobot_robot_doosan_a0509.topic_cache import (
    TopicCache,
    TopicUnavailableError,
    header_stamp_from_message,
)


JOINT_KEYS = tuple(f"joint_{index}_rad" for index in range(1, 7))
TCP_KEYS = (
    "tcp_x_mm",
    "tcp_y_mm",
    "tcp_z_mm",
    "tcp_o1_deg",
    "tcp_o2_deg",
    "tcp_o3_deg",
)
ACTION_KEYS = (
    "target_x_mm",
    "target_y_mm",
    "target_z_mm",
    "target_o1_deg",
    "target_o2_deg",
    "target_o3_deg",
    "gripper_target",
)


def _finite_vector(values: Iterable[float], length: int, name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != length:
        raise ValueError(f"{name} must contain exactly {length} values, got {len(result)}")
    if any(not math.isfinite(value) for value in result):
        raise ValueError(f"{name} contains non-finite values: {result}")
    return result


def _safe_node_suffix(value: str | None) -> str:
    rendered = "default" if value is None else str(value)
    return re.sub(r"[^a-zA-Z0-9_]", "_", rendered)


class DoosanA0509Ros(Robot):
    config_class = DoosanA0509RosConfig
    name = "doosan_a0509_ros"

    def __init__(self, config: DoosanA0509RosConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self.cache = TopicCache()
        self._runtime: RosRuntime | None = None
        self._node = None
        self._connected = False
        self._subscriptions: list[Any] = []
        self._target_pub = None
        self._gripper_pub = None
        self._debug_pub = None
        self.live_publish_count = 0
        self.debug_publish_count = 0
        self._observation_read_count = 0

    @property
    def observation_features(self) -> dict[str, type | tuple[int, int, int]]:
        features: dict[str, type | tuple[int, int, int]] = {
            **{key: float for key in JOINT_KEYS},
            **{key: float for key in TCP_KEYS},
            "gripper_commanded_state": float,
        }
        for key, camera_config in self.config.cameras.items():
            features[key] = (camera_config.height, camera_config.width, 3)
        return features

    @property
    def action_features(self) -> dict[str, type]:
        return {key: float for key in ACTION_KEYS}

    @property
    def is_connected(self) -> bool:
        return self._connected and all(camera.is_connected for camera in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        if self._connected:
            raise RuntimeError(f"{self} is already connected")
        self._runtime = RosRuntime.acquire()
        self._observation_read_count = 0
        try:
            self._node = self._runtime.create_node(
                f"lerobot_doosan_a0509_{_safe_node_suffix(self.id)}"
            )
            self._create_ros_entities()
            for camera in self.cameras.values():
                camera.connect()
            if self.config.require_fresh_state_on_connect:
                self._wait_for_required_state(self.config.connect_timeout_sec)
            self._connected = True
        except Exception:
            self._disconnect_partial()
            raise

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def get_observation(self) -> dict[str, Any]:
        self._require_connected()
        now = time.monotonic()
        state_startup_grace = (
            self._observation_read_count < self.config.state_startup_grace_reads
        )
        state_max_age_sec = (
            self.config.state_startup_max_age_sec
            if state_startup_grace
            else self.config.state_max_age_sec
        )
        joints = self.cache.require(
            "joint_positions", max_age_sec=state_max_age_sec, now=now
        ).value
        tcp = self.cache.require(
            "actual_tcp_position", max_age_sec=state_max_age_sec, now=now
        ).value
        gripper = self.cache.require_present("gripper_commanded_state").value

        observation: dict[str, Any] = {
            **dict(zip(JOINT_KEYS, joints, strict=True)),
            **dict(zip(TCP_KEYS, tcp, strict=True)),
            "gripper_commanded_state": float(gripper),
        }
        startup_grace = (
            self._observation_read_count < self.config.camera_startup_grace_reads
        )
        camera_max_age_ms = (
            self.config.camera_startup_frame_max_age_ms
            if startup_grace
            else self.config.camera_frame_max_age_ms
        )
        for key, camera in self.cameras.items():
            observation[key] = camera.read_latest(max_age_ms=camera_max_age_ms)
        self._observation_read_count += 1
        return observation

    def send_action(self, action: dict[str, Any]) -> dict[str, float]:
        self._require_connected()
        sanitized = self._validate_action(action)
        if self.config.mode == "shadow_record":
            return sanitized

        values = [sanitized[key] for key in ACTION_KEYS]
        if self.config.mode == "policy_dry_run":
            message = Float64MultiArray()
            message.data = values
            self._debug_pub.publish(message)
            self.debug_publish_count += 1
            return sanitized

        self._assert_policy_live_gate()
        target_message = Float64MultiArray()
        target_message.data = values[:6]
        self._target_pub.publish(target_message)
        self._gripper_pub.publish(Float64(data=values[6]))
        self.live_publish_count += 1
        return sanitized

    def disconnect(self) -> None:
        self._disconnect_partial()

    def _create_ros_entities(self) -> None:
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscriptions = [
            self._node.create_subscription(
                JointState, self.config.joint_states_topic, self._on_joint_state, 10
            ),
            self._node.create_subscription(
                Float64MultiArray,
                self.config.actual_tcp_position_topic,
                self._on_actual_tcp_position,
                10,
            ),
            self._node.create_subscription(
                Float64MultiArray,
                self.config.robot_state_topic,
                self._on_robot_state,
                10,
            ),
            self._node.create_subscription(
                Float64MultiArray,
                self.config.solution_space_topic,
                self._on_solution_space,
                10,
            ),
            self._node.create_subscription(
                Float64,
                self.config.gripper_commanded_state_topic,
                self._on_gripper_commanded_state,
                state_qos,
            ),
            self._node.create_subscription(
                Bool, self.config.teleop_ready_topic, self._on_teleop_ready, state_qos
            ),
            self._node.create_subscription(
                Bool, self.config.live_state_topic, self._on_live_state, state_qos
            ),
        ]
        self._target_pub = self._node.create_publisher(
            Float64MultiArray, self.config.lerobot_target_topic, 10
        )
        self._gripper_pub = self._node.create_publisher(
            Float64, self.config.lerobot_gripper_topic, 10
        )
        self._debug_pub = self._node.create_publisher(
            Float64MultiArray, self.config.debug_action_topic, 10
        )

    def _on_joint_state(self, message: JointState) -> None:
        try:
            if all(name in message.name for name in self.config.joint_names):
                positions = tuple(
                    message.position[message.name.index(name)]
                    for name in self.config.joint_names
                )
            else:
                positions = tuple(message.position[:6])
            value = _finite_vector(positions, 6, "joint_states.position")
        except (IndexError, TypeError, ValueError) as exc:
            self._node.get_logger().warning(f"ignored invalid joint state: {exc}")
            return
        self.cache.update(
            "joint_positions",
            value,
            header_stamp=header_stamp_from_message(message),
        )

    def _on_actual_tcp_position(self, message: Float64MultiArray) -> None:
        self._cache_vector("actual_tcp_position", message.data, 6)

    def _on_robot_state(self, message: Float64MultiArray) -> None:
        self._cache_scalar_array("robot_state", message.data)

    def _on_solution_space(self, message: Float64MultiArray) -> None:
        self._cache_scalar_array("solution_space", message.data)

    def _on_gripper_commanded_state(self, message: Float64) -> None:
        try:
            value = float(message.data)
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"expected [0, 1], got {value}")
        except (TypeError, ValueError) as exc:
            self._node.get_logger().warning(f"ignored invalid gripper state: {exc}")
            return
        self.cache.update("gripper_commanded_state", value)

    def _on_teleop_ready(self, message: Bool) -> None:
        self.cache.update("teleop_ready", bool(message.data))

    def _on_live_state(self, message: Bool) -> None:
        self.cache.update("live_state", bool(message.data))

    def _cache_vector(self, key: str, values: Iterable[float], length: int) -> None:
        try:
            value = _finite_vector(values, length, key)
        except (TypeError, ValueError) as exc:
            self._node.get_logger().warning(f"ignored invalid {key}: {exc}")
            return
        self.cache.update(key, value)

    def _cache_scalar_array(self, key: str, values: Iterable[float]) -> None:
        try:
            value = _finite_vector(values, 1, key)[0]
        except (TypeError, ValueError) as exc:
            self._node.get_logger().warning(f"ignored invalid {key}: {exc}")
            return
        self.cache.update(key, value)

    def _wait_for_required_state(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        if self.config.mode != "policy_live":
            self._wait_for_recording_state(deadline, timeout_sec)
            return

        required = (
            "joint_positions",
            "actual_tcp_position",
            "robot_state",
            "solution_space",
            "gripper_commanded_state",
            "teleop_ready",
            "live_state",
        )
        last_error = ""
        while True:
            now = time.monotonic()
            try:
                for key in required:
                    self.cache.require(key, max_age_sec=self.config.state_max_age_sec, now=now)
                return
            except TopicUnavailableError as exc:
                last_error = str(exc)
            if now >= deadline:
                raise RuntimeError(
                    f"required A0509 ROS state was not fresh within {timeout_sec:.2f}s: {last_error}"
                )
            time.sleep(0.02)

    def _wait_for_recording_state(self, deadline: float, timeout_sec: float) -> None:
        """Wait only for observation inputs in non-commanding modes."""
        last_error = ""
        while True:
            now = time.monotonic()
            try:
                self.cache.require(
                    "joint_positions",
                    max_age_sec=self.config.state_max_age_sec,
                    now=now,
                )
                self.cache.require(
                    "actual_tcp_position",
                    max_age_sec=self.config.state_max_age_sec,
                    now=now,
                )
                self.cache.require_present("gripper_commanded_state")
                return
            except TopicUnavailableError as exc:
                last_error = str(exc)
            if now >= deadline:
                raise RuntimeError(
                    "required A0509 recording state was not available within "
                    f"{timeout_sec:.2f}s: {last_error}"
                )
            time.sleep(0.02)

    def _assert_policy_live_gate(self) -> None:
        now = time.monotonic()
        required = (
            "joint_positions",
            "actual_tcp_position",
            "robot_state",
            "solution_space",
            "live_state",
            "gripper_commanded_state",
            "teleop_ready",
        )
        samples = {
            key: self.cache.require(key, max_age_sec=self.config.state_max_age_sec, now=now)
            for key in required
        }
        if not bool(samples["teleop_ready"].value):
            raise RuntimeError("policy_live action rejected because teleop_ready=false")
        robot_state = int(round(float(samples["robot_state"].value)))
        if robot_state not in self.config.allowed_robot_states:
            raise RuntimeError(
                f"policy_live action rejected because robot_state={robot_state} is not allowed"
            )

    def _validate_action(self, action: dict[str, Any]) -> dict[str, float]:
        missing = set(ACTION_KEYS) - set(action)
        extra = set(action) - set(ACTION_KEYS)
        if missing or extra:
            raise ValueError(
                f"action keys must exactly match {ACTION_KEYS}; missing={sorted(missing)}, extra={sorted(extra)}"
            )
        values = _finite_vector((action[key] for key in ACTION_KEYS), 7, "action")
        if not 0.0 <= values[6] <= 1.0:
            raise ValueError(f"gripper_target must be in [0, 1], got {values[6]}")
        return dict(zip(ACTION_KEYS, values, strict=True))

    def _require_connected(self) -> None:
        if not self.is_connected:
            raise RuntimeError(f"{self} is not connected")

    def _disconnect_partial(self) -> None:
        for camera in self.cameras.values():
            try:
                if camera.is_connected:
                    camera.disconnect()
            except Exception:
                pass
        runtime = self._runtime
        node = self._node
        self._runtime = None
        self._node = None
        self._subscriptions = []
        self._target_pub = None
        self._gripper_pub = None
        self._debug_pub = None
        self._connected = False
        self._observation_read_count = 0
        if runtime is not None:
            runtime.release_node(node)
