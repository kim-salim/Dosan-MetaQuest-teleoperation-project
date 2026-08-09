"""Supervise one bounded MetaQuest arm-and-gripper teleoperation session."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import time

from dsr_msgs2.srv import GetCurrentPosx
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

try:
    from quest2ros.msg import OVR2ROSInputs
except Exception:  # pragma: no cover - only on systems without Quest2ROS
    OVR2ROSInputs = None


def shortest_angle_delta_deg(target: float, reference: float) -> float:
    """Return the signed shortest angular delta in degrees."""
    return (float(target) - float(reference) + 180.0) % 360.0 - 180.0


def pose_offset_metrics(
    pose: list[float],
    anchor: list[float],
) -> tuple[list[float], list[float], float]:
    """Return XYZ delta, RPY shortest deltas, and XYZ norm."""
    position = [float(pose[index]) - float(anchor[index]) for index in range(3)]
    rotation = [
        shortest_angle_delta_deg(pose[index], anchor[index])
        for index in range(3, 6)
    ]
    position_norm = math.sqrt(sum(value * value for value in position))
    return position, rotation, position_norm


def pose_within_limits(
    pose: list[float],
    anchor: list[float],
    *,
    position_limit_mm: float,
    rotation_limit_deg: float,
) -> bool:
    """Return whether a pose has converged to the preflight anchor."""
    _, rotation, position_norm = pose_offset_metrics(pose, anchor)
    return (
        position_norm <= float(position_limit_mm)
        and max(abs(value) for value in rotation)
        <= float(rotation_limit_deg)
    )


@dataclass(frozen=True)
class SessionHealth:
    """Snapshot used by the live-session safety decision."""

    calibration_valid: bool
    teleop_ready: bool
    source: str
    selected_valid: bool
    live_state: bool
    pose_heartbeat_age_sec: float
    gripper_input_age_sec: float
    gripper_busy: bool
    gripper_last_command_ok: bool
    gripper_command_age_sec: float | None


def live_fault_reason(
    health: SessionHealth,
    *,
    heartbeat_timeout_sec: float,
    gripper_input_timeout_sec: float,
    gripper_command_timeout_sec: float,
) -> str | None:
    """Return why a live session must stop, or ``None`` while healthy."""
    if not health.calibration_valid:
        return 'MetaQuest calibration became invalid'
    if not health.teleop_ready:
        return 'teleop_ready became false'
    if health.source != 'METAQUEST':
        return f'control source changed to {health.source}'
    if not health.selected_valid:
        return 'selected MetaQuest command became invalid'
    if not health.live_state:
        return 'Live robot output became disabled'
    if health.pose_heartbeat_age_sec > heartbeat_timeout_sec:
        return (
            'MetaQuest pose heartbeat became stale: '
            f'{health.pose_heartbeat_age_sec:.3f}s'
        )
    if health.gripper_input_age_sec > gripper_input_timeout_sec:
        return (
            'MetaQuest gripper input became stale: '
            f'{health.gripper_input_age_sec:.3f}s'
        )
    if health.gripper_command_age_sec is not None:
        if (
            health.gripper_busy
            and health.gripper_command_age_sec > gripper_command_timeout_sec
        ):
            return (
                'gripper command remained busy too long: '
                f'{health.gripper_command_age_sec:.3f}s'
            )
        if (
            not health.gripper_busy
            and not health.gripper_last_command_ok
            and health.gripper_command_age_sec > gripper_command_timeout_sec
        ):
            return 'gripper driver reported a failed command'
    return None


@dataclass
class SessionMetrics:
    """Measurements collected only while the live session is active."""

    safe_samples: int = 0
    max_target_position_norm_mm: float = 0.0
    max_target_rotation_abs_deg: float = 0.0
    gripper_mapper_commands: list[str] = field(default_factory=list)
    gripper_driver_accepted_commands: list[str] = field(default_factory=list)
    gripper_driver_commands: list[str] = field(default_factory=list)

    def record_safe(self, pose: list[float], anchor: list[float]) -> None:
        _, rotation, position_norm = pose_offset_metrics(pose, anchor)
        self.safe_samples += 1
        self.max_target_position_norm_mm = max(
            self.max_target_position_norm_mm,
            position_norm,
        )
        self.max_target_rotation_abs_deg = max(
            self.max_target_rotation_abs_deg,
            max(abs(value) for value in rotation),
        )

    @property
    def gripper_validation_pass(self) -> bool:
        return {'open', 'close'}.issubset(set(self.gripper_driver_commands))


class MetaQuestFullSessionNode(Node):
    """Own preflight, live supervision, metrics, and fail-safe cleanup."""

    def __init__(self) -> None:
        super().__init__('metaquest_full_session')
        self._declare_parameters()

        self.duration_sec = float(self.get_parameter('duration_sec').value)
        self.preflight_position_limit_mm = float(
            self.get_parameter('preflight_position_limit_mm').value
        )
        self.preflight_rotation_limit_deg = float(
            self.get_parameter('preflight_rotation_limit_deg').value
        )
        self.heartbeat_timeout_sec = float(
            self.get_parameter('heartbeat_timeout_sec').value
        )
        self.gripper_input_timeout_sec = float(
            self.get_parameter('gripper_input_timeout_sec').value
        )
        self.gripper_command_timeout_sec = float(
            self.get_parameter('gripper_command_timeout_sec').value
        )
        for name, value in (
            ('duration_sec', self.duration_sec),
            ('preflight_position_limit_mm', self.preflight_position_limit_mm),
            ('preflight_rotation_limit_deg', self.preflight_rotation_limit_deg),
            ('heartbeat_timeout_sec', self.heartbeat_timeout_sec),
            ('gripper_input_timeout_sec', self.gripper_input_timeout_sec),
            ('gripper_command_timeout_sec', self.gripper_command_timeout_sec),
        ):
            if value <= 0.0:
                raise ValueError(f'{name} must be > 0.0')

        self.anchor: list[float] | None = None
        self.anchor_time: float | None = None
        self.safe: list[float] | None = None
        self.safe_time: float | None = None
        self.calibration_valid: bool | None = None
        self.teleop_ready: bool | None = None
        self.source: str | None = None
        self.selected_valid: bool | None = None
        self.live_state: bool | None = None
        self.last_pose_heartbeat: float | None = None
        self.last_selected_heartbeat: float | None = None
        self.last_gripper_input: float | None = None
        self.gripper_driver_busy: bool | None = None
        self.gripper_last_command_ok: bool | None = None
        self.gripper_commanded_state: float | None = None
        self.last_gripper_accepted_command: str | None = None
        self.last_gripper_accept_time: float | None = None
        self.last_gripper_ok_time: float | None = None
        self.command_count = 0
        self.metrics_active = False
        self.metrics = SessionMetrics()
        self.disable_requested_at: float | None = None
        self.false_seen_at: float | None = None

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Float64MultiArray,
            '/vr/robot_anchor_posx',
            self._on_anchor,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            '/vr/safe_posx',
            self._on_safe,
            10,
        )
        self.create_subscription(
            Bool,
            '/vr/metaquest_calibration/valid',
            lambda msg: setattr(self, 'calibration_valid', bool(msg.data)),
            state_qos,
        )
        self.create_subscription(
            Bool,
            '/vr/teleop_ready',
            lambda msg: setattr(self, 'teleop_ready', bool(msg.data)),
            state_qos,
        )
        self.create_subscription(
            String,
            '/control/source',
            lambda msg: setattr(self, 'source', str(msg.data)),
            state_qos,
        )
        self.create_subscription(
            Bool,
            '/control/selected_command_valid',
            lambda msg: setattr(self, 'selected_valid', bool(msg.data)),
            state_qos,
        )
        self.create_subscription(
            Bool,
            '/vr/live_robot_output_enabled',
            self._on_live_state,
            state_qos,
        )
        self.create_subscription(
            Empty,
            '/control/metaquest/valid_pose_heartbeat',
            lambda _msg: setattr(self, 'last_pose_heartbeat', time.monotonic()),
            10,
        )
        self.create_subscription(
            Empty,
            '/control/selected_command_heartbeat',
            lambda _msg: setattr(
                self,
                'last_selected_heartbeat',
                time.monotonic(),
            ),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            '/vr/commanded_posx',
            lambda _msg: setattr(self, 'command_count', self.command_count + 1),
            10,
        )
        self.create_subscription(
            String,
            '/control/metaquest/gripper_cmd',
            self._on_gripper_mapper_command,
            10,
        )
        self.create_subscription(
            String,
            '/jrt_gripper/accepted_command',
            self._on_gripper_accepted_command,
            state_qos,
        )
        self.create_subscription(
            String,
            '/jrt_gripper/completed_command',
            self._on_gripper_completed_command,
            state_qos,
        )
        self.create_subscription(
            Bool,
            '/jrt_gripper/driver_busy',
            lambda msg: setattr(self, 'gripper_driver_busy', bool(msg.data)),
            state_qos,
        )
        self.create_subscription(
            Bool,
            '/jrt_gripper/last_command_ok',
            self._on_gripper_last_command_ok,
            state_qos,
        )
        self.create_subscription(
            Float64,
            '/jrt_gripper/commanded_state',
            lambda msg: setattr(self, 'gripper_commanded_state', float(msg.data)),
            state_qos,
        )

        if OVR2ROSInputs is None:
            raise RuntimeError(
                'quest2ros.msg.OVR2ROSInputs is unavailable; '
                'cannot supervise MetaQuest A/B gripper input'
            )
        self.create_subscription(
            OVR2ROSInputs,
            '/q2r_right_hand_inputs',
            lambda _msg: setattr(self, 'last_gripper_input', time.monotonic()),
            10,
        )

        self.set_anchor_client = self.create_client(
            Trigger,
            '/vr/set_robot_anchor_to_current_tcp',
        )
        self.recenter_client = self.create_client(Trigger, '/vr/recenter')
        self.select_metaquest_client = self.create_client(
            Trigger,
            '/control/select_metaquest',
        )
        self.select_disabled_client = self.create_client(
            Trigger,
            '/control/select_disabled',
        )
        self.set_live_client = self.create_client(
            SetBool,
            '/vr/set_live_robot_output',
        )
        self.get_posx_client = self.create_client(
            GetCurrentPosx,
            '/dsr01/dsr_controller2/aux_control/get_current_posx',
        )
        self.gripper_stop_pub = self.create_publisher(
            String,
            '/jrt_gripper/cmd',
            10,
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('duration_sec', 30.0)
        self.declare_parameter('preflight_position_limit_mm', 10.0)
        self.declare_parameter('preflight_rotation_limit_deg', 3.0)
        self.declare_parameter('heartbeat_timeout_sec', 1.0)
        self.declare_parameter('gripper_input_timeout_sec', 1.0)
        self.declare_parameter('gripper_command_timeout_sec', 2.0)

    def _on_anchor(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 6:
            self.anchor = [float(value) for value in msg.data[:6]]
            self.anchor_time = time.monotonic()

    def _on_safe(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 6:
            return
        self.safe = [float(value) for value in msg.data[:6]]
        self.safe_time = time.monotonic()
        if self.metrics_active and self.anchor is not None:
            self.metrics.record_safe(self.safe, self.anchor)

    def _on_live_state(self, msg: Bool) -> None:
        self.live_state = bool(msg.data)
        if (
            self.disable_requested_at is not None
            and not self.live_state
            and self.false_seen_at is None
        ):
            self.false_seen_at = time.monotonic()

    def _on_gripper_mapper_command(self, msg: String) -> None:
        command = str(msg.data).strip().lower()
        if self.metrics_active:
            self.metrics.gripper_mapper_commands.append(command)

    def _on_gripper_accepted_command(self, msg: String) -> None:
        command = str(msg.data).strip().lower()
        self.last_gripper_accepted_command = command
        self.last_gripper_accept_time = time.monotonic()
        if self.metrics_active:
            self.metrics.gripper_driver_accepted_commands.append(command)

    def _on_gripper_completed_command(self, msg: String) -> None:
        command = str(msg.data).strip().lower()
        if self.metrics_active:
            self.metrics.gripper_driver_commands.append(command)

    def _on_gripper_last_command_ok(self, msg: Bool) -> None:
        self.gripper_last_command_ok = bool(msg.data)
        if self.gripper_last_command_ok:
            self.last_gripper_ok_time = time.monotonic()

    def spin_until(self, predicate, timeout_sec: float, label: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=0.01)
        raise TimeoutError(label)

    def call(self, client, request, timeout_sec: float = 10.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError(f'service unavailable: {client.srv_name}')
        future = client.call_async(request)
        self.spin_until(lambda: future.done(), timeout_sec, client.srv_name)
        result = future.result()
        if result is None:
            raise RuntimeError(f'service failed: {client.srv_name}')
        return result

    def trigger(self, client) -> str:
        result = self.call(client, Trigger.Request())
        if not result.success:
            raise RuntimeError(result.message)
        return str(result.message)

    def set_live_enabled(self, enabled: bool) -> str:
        request = SetBool.Request()
        request.data = bool(enabled)
        result = self.call(self.set_live_client, request)
        if not result.success:
            raise RuntimeError(result.message)
        return str(result.message)

    def current_tcp(self) -> list[float]:
        request = GetCurrentPosx.Request()
        request.ref = 0
        result = self.call(self.get_posx_client, request)
        if not result.success or not result.task_pos_info:
            raise RuntimeError('GetCurrentPosx returned no pose')
        return [float(value) for value in result.task_pos_info[0].data[:6]]

    def _fresh(self, timestamp: float | None, timeout_sec: float) -> bool:
        return (
            timestamp is not None
            and time.monotonic() - timestamp <= float(timeout_sec)
        )

    def _wait_for_initial_state(self) -> None:
        self.spin_until(
            lambda: (
                self.calibration_valid is not None
                and self.teleop_ready is not None
                and self.source is not None
                and self.live_state is not None
                and self.gripper_driver_busy is not None
                and self.gripper_last_command_ok is not None
                and self.last_pose_heartbeat is not None
                and self.last_gripper_input is not None
                and self._fresh(
                    self.last_pose_heartbeat,
                    self.heartbeat_timeout_sec,
                )
                and self._fresh(
                    self.last_gripper_input,
                    self.gripper_input_timeout_sec,
                )
            ),
            8.0,
            'initial MetaQuest/arm/gripper state timeout',
        )
        if not self.calibration_valid:
            raise RuntimeError('calibration is not VALID')
        if not self.teleop_ready:
            raise RuntimeError('teleop_ready is false')
        if not self._fresh(self.last_pose_heartbeat, self.heartbeat_timeout_sec):
            raise RuntimeError('MetaQuest pose heartbeat is stale')
        if not self._fresh(
            self.last_gripper_input,
            self.gripper_input_timeout_sec,
        ):
            raise RuntimeError('MetaQuest A/B gripper input is stale')

    def _health(self, now: float) -> SessionHealth:
        pose_age = (
            math.inf
            if self.last_pose_heartbeat is None
            else now - self.last_pose_heartbeat
        )
        gripper_input_age = (
            math.inf
            if self.last_gripper_input is None
            else now - self.last_gripper_input
        )
        gripper_command_age = (
            None
            if self.last_gripper_accept_time is None
            else now - self.last_gripper_accept_time
        )
        return SessionHealth(
            calibration_valid=bool(self.calibration_valid),
            teleop_ready=bool(self.teleop_ready),
            source=str(self.source),
            selected_valid=bool(self.selected_valid),
            live_state=bool(self.live_state),
            pose_heartbeat_age_sec=pose_age,
            gripper_input_age_sec=gripper_input_age,
            gripper_busy=bool(self.gripper_driver_busy),
            gripper_last_command_ok=bool(self.gripper_last_command_ok),
            gripper_command_age_sec=gripper_command_age,
        )

    def force_safe(self) -> dict[str, object]:
        """Disable arm output/source and directly request gripper outputs off."""
        outcome: dict[str, object] = {}
        self.metrics_active = False
        safe_requested_at = time.monotonic()
        try:
            outcome['disable_live'] = self.set_live_enabled(False)
        except Exception as exc:  # pragma: no cover - hardware/service failure
            outcome['disable_live_error'] = repr(exc)
        try:
            outcome['select_disabled'] = self.trigger(
                self.select_disabled_client
            )
        except Exception as exc:  # pragma: no cover - hardware/service failure
            outcome['select_disabled_error'] = repr(exc)

        self.gripper_stop_pub.publish(String(data='stop'))
        stop_deadline = time.monotonic() + self.gripper_command_timeout_sec + 0.5
        while rclpy.ok() and time.monotonic() < stop_deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if (
                self.last_gripper_accepted_command == 'stop'
                and self.last_gripper_accept_time is not None
                and self.last_gripper_accept_time >= safe_requested_at
                and self.gripper_driver_busy is False
                and self.gripper_last_command_ok is True
                and self.last_gripper_ok_time is not None
                and self.last_gripper_ok_time >= self.last_gripper_accept_time
            ):
                break
        outcome['final_live_state'] = self.live_state
        outcome['final_source'] = self.source
        outcome['gripper_busy'] = self.gripper_driver_busy
        outcome['gripper_last_command_ok'] = self.gripper_last_command_ok
        return outcome

    def run_session(self) -> dict[str, object]:
        result: dict[str, object] = {'duration_requested_sec': self.duration_sec}
        result['initial_safe'] = self.force_safe()
        self._wait_for_initial_state()

        print(
            'FULL_TELEOP_READY_FOR_GO\n'
            'During the 30 second session: A=close, B=open. '
            'Hold each button for about 0.5 second and test both.',
            flush=True,
        )
        input()

        self._wait_for_initial_state()
        anchor_requested_at = time.monotonic()
        result['set_anchor'] = self.trigger(self.set_anchor_client)
        self.spin_until(
            lambda: (
                self.anchor is not None
                and self.anchor_time is not None
                and self.anchor_time >= anchor_requested_at
            ),
            2.0,
            'fresh robot anchor timeout',
        )
        result['recenter'] = self.trigger(self.recenter_client)
        selected_at = time.monotonic()
        result['select_metaquest'] = self.trigger(
            self.select_metaquest_client
        )
        self.spin_until(
            lambda: (
                self.source == 'METAQUEST'
                and self.safe_time is not None
                and self.safe_time > max(anchor_requested_at, selected_at) + 0.35
                and self.selected_valid is True
                and self._fresh(
                    self.last_selected_heartbeat,
                    self.heartbeat_timeout_sec,
                )
            ),
            5.0,
            'fresh selected MetaQuest target timeout',
        )
        if self.anchor is None or self.safe is None:
            raise RuntimeError('preflight anchor or safe target missing')

        self.spin_until(
            lambda: (
                self.anchor is not None
                and self.safe is not None
                and pose_within_limits(
                    self.safe,
                    self.anchor,
                    position_limit_mm=self.preflight_position_limit_mm,
                    rotation_limit_deg=self.preflight_rotation_limit_deg,
                )
            ),
            5.0,
            'safe target did not converge to the new anchor',
        )

        position, rotation, position_norm = pose_offset_metrics(
            self.safe,
            self.anchor,
        )
        result['anchor'] = self.anchor
        result['preflight_safe'] = self.safe
        result['preflight_position_error_mm'] = position
        result['preflight_position_norm_mm'] = position_norm
        result['preflight_rotation_error_deg'] = rotation
        if position_norm > self.preflight_position_limit_mm:
            raise RuntimeError(
                'preflight position error exceeded '
                f'{self.preflight_position_limit_mm:.3f} mm: {position_norm}'
            )
        if max(abs(value) for value in rotation) > self.preflight_rotation_limit_deg:
            raise RuntimeError(
                'preflight rotation error exceeded '
                f'{self.preflight_rotation_limit_deg:.3f} deg: {rotation}'
            )
        self.spin_until(
            lambda: (
                self.last_gripper_accepted_command == 'stop'
                and self.last_gripper_accept_time is not None
                and self.last_gripper_accept_time >= selected_at
                and self.gripper_driver_busy is False
                and self.gripper_last_command_ok is True
                and self.last_gripper_ok_time is not None
                and self.last_gripper_ok_time >= self.last_gripper_accept_time
            ),
            self.gripper_command_timeout_sec + 0.5,
            'gripper preflight stop did not complete',
        )

        result['tcp_before'] = self.current_tcp()
        self.metrics = SessionMetrics()
        self.last_gripper_accept_time = None
        self.metrics_active = True
        result['enable_live'] = self.set_live_enabled(True)
        self.spin_until(
            lambda: self.live_state is True,
            1.0,
            'Live=true state timeout',
        )
        start = time.monotonic()
        deadline = start + self.duration_sec
        next_progress = start + 5.0

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            now = time.monotonic()
            fault = live_fault_reason(
                self._health(now),
                heartbeat_timeout_sec=self.heartbeat_timeout_sec,
                gripper_input_timeout_sec=self.gripper_input_timeout_sec,
                gripper_command_timeout_sec=self.gripper_command_timeout_sec,
            )
            if fault is not None:
                raise RuntimeError(fault)
            if now >= next_progress:
                print(
                    'FULL_TELEOP_PROGRESS='
                    + json.dumps(
                        {
                            'elapsed_sec': now - start,
                            'remaining_sec': max(0.0, deadline - now),
                            'max_target_position_norm_mm': (
                                self.metrics.max_target_position_norm_mm
                            ),
                            'max_target_rotation_abs_deg': (
                                self.metrics.max_target_rotation_abs_deg
                            ),
                            'gripper_driver_commands': (
                                self.metrics.gripper_driver_commands
                            ),
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                next_progress += 5.0

        self.metrics_active = False
        self.false_seen_at = None
        self.disable_requested_at = time.monotonic()
        result['disable_live'] = self.set_live_enabled(False)
        disable_response_at = time.monotonic()
        self.spin_until(
            lambda: self.false_seen_at is not None,
            1.0,
            'Live=false state timeout',
        )
        result['duration_actual_sec'] = self.disable_requested_at - start
        result['live_false_state_latency_sec'] = (
            self.false_seen_at - self.disable_requested_at
        )
        result['live_disable_response_latency_sec'] = (
            disable_response_at - self.disable_requested_at
        )

        commands_at_disable = self.command_count
        quiet_deadline = time.monotonic() + 0.75
        while time.monotonic() < quiet_deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
        result['commands_after_disable_response'] = (
            self.command_count - commands_at_disable
        )
        result['final_safe'] = self.force_safe()
        result['tcp_after'] = self.current_tcp()
        tcp_delta, tcp_rotation, tcp_delta_norm = pose_offset_metrics(
            result['tcp_after'],
            result['tcp_before'],
        )
        result['tcp_delta_mm'] = tcp_delta
        result['tcp_rotation_delta_deg'] = tcp_rotation
        result['tcp_delta_norm_mm'] = tcp_delta_norm
        result['safe_samples'] = self.metrics.safe_samples
        result['max_target_position_norm_mm'] = (
            self.metrics.max_target_position_norm_mm
        )
        result['max_target_rotation_abs_deg'] = (
            self.metrics.max_target_rotation_abs_deg
        )
        result['gripper_mapper_commands'] = self.metrics.gripper_mapper_commands
        result['gripper_driver_accepted_commands'] = (
            self.metrics.gripper_driver_accepted_commands
        )
        result['gripper_driver_commands'] = self.metrics.gripper_driver_commands
        result['gripper_validation_pass'] = (
            self.metrics.gripper_validation_pass
        )
        result['gripper_commanded_state'] = self.gripper_commanded_state
        return result


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MetaQuestFullSessionNode()
    completed = False
    try:
        result = node.run_session()
        completed = True
        print(
            'FULL_TELEOP_RESULT=' + json.dumps(result, sort_keys=True),
            flush=True,
        )
        if not result['gripper_validation_pass']:
            print(
                'FULL_TELEOP_WARNING=gripper open and close were not both '
                'accepted during the live interval',
                flush=True,
            )
    except (KeyboardInterrupt, ExternalShutdownException):
        print('FULL_TELEOP_ERROR=session interrupted', flush=True)
    except Exception as exc:
        print('FULL_TELEOP_ERROR=' + repr(exc), flush=True)
        raise
    finally:
        if not completed and rclpy.ok():
            cleanup = node.force_safe()
            print(
                'FULL_TELEOP_CLEANUP=' + json.dumps(cleanup, sort_keys=True),
                flush=True,
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
