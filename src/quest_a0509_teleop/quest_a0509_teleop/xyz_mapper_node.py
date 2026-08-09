"""Map Quest2ROS right-hand PoseStamped into Doosan posx targets.

Input PoseStamped uses ROS units: meters and quaternion. Position is mapped
from the Quest anchor pose into Doosan XYZ. The configured roll-only orientation
path preserves the legacy Quest-roll sign, scale, filters, and anchor-relative
motion, but composes the physical Doosan ZYZ B-axis rotation in quaternions.
The robot-interface ZYZ representation is selected nearest the previous command
so crossing beta=+/-180 degrees remains physically and numerically continuous.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from typing import Iterable, Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from quest_a0509_teleop.calibration_state import CalibrationState
from quest_a0509_teleop.doosan_orientation import (
    apply_doosan_zyz_b_delta_deg,
    doosan_zyz_b_delta_quaternion,
    doosan_zyz_deg_to_quaternion,
    quaternion_to_doosan_zyz_deg,
)
from quest_a0509_teleop.pose_resampler import (
    fixed_rate_alpha,
    PoseResampler,
    quaternion_slerp,
)
from std_msgs.msg import Bool, Empty, Float64MultiArray, String
from std_srvs.srv import Trigger


def _vector3(values: Iterable[float], name: str) -> list[float]:
    output = [float(value) for value in values]
    if len(output) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    if any(not math.isfinite(value) for value in output):
        raise ValueError(f"{name} contains non-finite values: {output}")
    return output


def _axis_map(values: Iterable[int], name: str) -> list[int]:
    output = [int(value) for value in values]
    if len(output) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    if sorted(output) != [0, 1, 2]:
        raise ValueError(f"{name} must be a permutation of [0, 1, 2], got {output}")
    return output


def _distance3(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((a[index] - b[index]) ** 2 for index in range(3)))


def _quat_xyzw(values: Iterable[float], name: str) -> list[float]:
    output = [float(value) for value in values]
    if len(output) != 4:
        raise ValueError(f"{name} must contain exactly 4 values")
    if any(not math.isfinite(value) for value in output):
        raise ValueError(f"{name} contains non-finite values: {output}")
    return _quat_normalized(output, name)


def _quat_normalized(values: list[float], name: str) -> list[float]:
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-9:
        raise ValueError(f"{name} has near-zero norm: {values}")
    return [value / norm for value in values]


def _quat_conjugate(q: list[float]) -> list[float]:
    return [-q[0], -q[1], -q[2], q[3]]


def _quat_multiply(a: list[float], b: list[float]) -> list[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return _quat_normalized(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        "quaternion product",
    )


def _quat_angle_deg(a: list[float], b: list[float]) -> float:
    dot = abs(sum(a[index] * b[index] for index in range(4)))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def _quat_to_euler_xyz_deg(q: list[float]) -> list[float]:
    x, y, z, w = q
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch_sin = 2.0 * (w * y - z * x)
    pitch = math.asin(min(1.0, max(-1.0, pitch_sin)))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def _euler_xyz_deg_to_quat(rpy_deg: list[float]) -> list[float]:
    roll, pitch, yaw = [math.radians(value) for value in rpy_deg]
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return _quat_normalized(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ],
        "euler xyz quaternion",
    )


def _shortest_angle_delta_deg(value: float, reference: float) -> float:
    return (value - reference + 180.0) % 360.0 - 180.0


def _angle_near_reference_deg(value: float, reference: float) -> float:
    return reference + _shortest_angle_delta_deg(value, reference)


def _rotate_xy(values: list[float], angle_deg: float) -> list[float]:
    angle_rad = math.radians(angle_deg)
    cos_angle = math.cos(angle_rad)
    sin_angle = math.sin(angle_rad)
    return [
        cos_angle * values[0] - sin_angle * values[1],
        sin_angle * values[0] + cos_angle * values[1],
    ]


def _mapping_fingerprint(mapping: dict[str, object]) -> str:
    canonical = json.dumps(mapping, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]



class XyzMapperNode(Node):
    def __init__(self) -> None:
        super().__init__("xyz_mapper_node")
        self._declare_parameters()

        self.input_pose_topic = self.get_parameter("input_pose_topic").value
        self.target_posx_topic = self.get_parameter("target_posx_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.robot_anchor_posx_topic = self.get_parameter("robot_anchor_posx_topic").value
        self.teleop_ready_topic = self.get_parameter("teleop_ready_topic").value
        self.live_state_topic = self.get_parameter("live_state_topic").value
        self.valid_pose_heartbeat_topic = self.get_parameter(
            "valid_pose_heartbeat_topic"
        ).value
        self.require_prepare_before_target = bool(
            self.get_parameter("require_prepare_before_target").value
        )
        self.robot_anchor_xyz_mm = _vector3(
            self.get_parameter("robot_anchor_xyz_mm").value,
            "robot_anchor_xyz_mm",
        )
        self.robot_anchor_rpy_deg = _vector3(
            self.get_parameter("robot_anchor_rpy_deg").value,
            "robot_anchor_rpy_deg",
        )
        self.scale_xyz = _vector3(self.get_parameter("scale_xyz").value, "scale_xyz")
        self.axis_sign = _vector3(self.get_parameter("axis_sign").value, "axis_sign")
        self.axis_map = _axis_map(self.get_parameter("axis_map").value, "axis_map")
        self.enable_xy_yaw_correction = bool(
            self.get_parameter("enable_xy_yaw_correction").value
        )
        self.xy_yaw_correction_deg = float(
            self.get_parameter("xy_yaw_correction_deg").value
        )
        if not math.isfinite(self.xy_yaw_correction_deg):
            raise ValueError("xy_yaw_correction_deg must be finite")
        self.initial_xy_yaw_correction_deg = self.xy_yaw_correction_deg
        self.require_xy_yaw_calibration = bool(
            self.get_parameter("require_xy_yaw_calibration").value
        )
        if self.require_xy_yaw_calibration and not self.enable_xy_yaw_correction:
            raise ValueError(
                "require_xy_yaw_calibration=true requires "
                "enable_xy_yaw_correction=true"
            )
        self.xy_yaw_calibration_duration_sec = float(
            self.get_parameter("xy_yaw_calibration_duration_sec").value
        )
        if self.xy_yaw_calibration_duration_sec <= 0.0:
            raise ValueError("xy_yaw_calibration_duration_sec must be > 0.0")
        self.xy_yaw_calibration_min_distance_m = float(
            self.get_parameter("xy_yaw_calibration_min_distance_m").value
        )
        if self.xy_yaw_calibration_min_distance_m <= 0.0:
            raise ValueError("xy_yaw_calibration_min_distance_m must be > 0.0")
        self.xy_yaw_calibration_service = self.get_parameter(
            "xy_yaw_calibration_service"
        ).value
        self.xy_yaw_calibration_reset_service = self.get_parameter(
            "xy_yaw_calibration_reset_service"
        ).value
        self.xy_yaw_calibration_valid_topic = self.get_parameter(
            "xy_yaw_calibration_valid_topic"
        ).value
        self.xy_yaw_calibration_status_topic = self.get_parameter(
            "xy_yaw_calibration_status_topic"
        ).value
        self.auto_recenter_on_first_pose = bool(
            self.get_parameter("auto_recenter_on_first_pose").value
        )
        self.recenter_vr_on_robot_anchor_update = bool(
            self.get_parameter("recenter_vr_on_robot_anchor_update").value
        )
        self.input_timeout_sec = float(self.get_parameter("input_timeout_sec").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if not math.isfinite(self.publish_rate_hz) or self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be finite and > 0.0")
        self.control_period_sec = 1.0 / self.publish_rate_hz
        self.enable_pose_resampling = bool(
            self.get_parameter("enable_pose_resampling").value
        )
        self.pose_interpolation_delay_sec = float(
            self.get_parameter("pose_interpolation_delay_sec").value
        )
        if (
            not math.isfinite(self.pose_interpolation_delay_sec)
            or self.pose_interpolation_delay_sec < 0.0
        ):
            raise ValueError("pose_interpolation_delay_sec must be finite and >= 0")
        self.pose_buffer_duration_sec = float(
            self.get_parameter("pose_buffer_duration_sec").value
        )
        self.pose_buffer_max_samples = int(
            self.get_parameter("pose_buffer_max_samples").value
        )
        self.pose_max_interpolation_gap_sec = float(
            self.get_parameter("pose_max_interpolation_gap_sec").value
        )
        self.pose_full_prediction_sec = float(
            self.get_parameter("pose_full_prediction_sec").value
        )
        self.pose_prediction_decay_sec = float(
            self.get_parameter("pose_prediction_decay_sec").value
        )
        self.pose_velocity_estimation_min_sec = float(
            self.get_parameter("pose_velocity_estimation_min_sec").value
        )
        self.pose_max_prediction_linear_speed_m_s = float(
            self.get_parameter("pose_max_prediction_linear_speed_m_s").value
        )
        self.pose_max_prediction_angular_speed_deg_s = float(
            self.get_parameter("pose_max_prediction_angular_speed_deg_s").value
        )
        self.pose_tracking_invalid_after_sec = float(
            self.get_parameter("pose_tracking_invalid_after_sec").value
        )
        if (
            not math.isfinite(self.pose_tracking_invalid_after_sec)
            or self.pose_tracking_invalid_after_sec <= 0.0
            or self.pose_tracking_invalid_after_sec > self.input_timeout_sec
        ):
            raise ValueError(
                "pose_tracking_invalid_after_sec must be finite, > 0, and <= "
                "input_timeout_sec"
            )
        self.pose_filter_reference_rate_hz = float(
            self.get_parameter("pose_filter_reference_rate_hz").value
        )
        self.enable_pose_low_pass_filter = bool(
            self.get_parameter("enable_pose_low_pass_filter").value
        )
        self.pose_filter_alpha = float(self.get_parameter("pose_filter_alpha").value)
        if not 0.0 < self.pose_filter_alpha <= 1.0:
            raise ValueError("pose_filter_alpha must be in the range (0.0, 1.0]")
        self.pose_filter_tick_alpha = fixed_rate_alpha(
            self.pose_filter_alpha,
            self.pose_filter_reference_rate_hz,
            self.control_period_sec,
        )
        self.max_vr_jump_m = float(self.get_parameter("max_vr_jump_m").value)
        if self.max_vr_jump_m < 0.0:
            raise ValueError("max_vr_jump_m must be >= 0.0")
        self.enable_orientation_mapping = bool(
            self.get_parameter("enable_orientation_mapping").value
        )
        self.scale_rpy = _vector3(self.get_parameter("scale_rpy").value, "scale_rpy")
        self.rot_axis_sign = _vector3(
            self.get_parameter("rot_axis_sign").value,
            "rot_axis_sign",
        )
        self.rot_axis_map = _axis_map(
            self.get_parameter("rot_axis_map").value,
            "rot_axis_map",
        )
        self.enable_orientation_low_pass_filter = bool(
            self.get_parameter("enable_orientation_low_pass_filter").value
        )
        self.orientation_filter_alpha = float(
            self.get_parameter("orientation_filter_alpha").value
        )
        if not 0.0 < self.orientation_filter_alpha <= 1.0:
            raise ValueError("orientation_filter_alpha must be in the range (0.0, 1.0]")
        self.orientation_filter_tick_alpha = fixed_rate_alpha(
            self.orientation_filter_alpha,
            self.pose_filter_reference_rate_hz,
            self.control_period_sec,
        )
        self.max_vr_rot_jump_deg = float(self.get_parameter("max_vr_rot_jump_deg").value)
        if self.max_vr_rot_jump_deg < 0.0:
            raise ValueError("max_vr_rot_jump_deg must be >= 0.0")
        self.roll_only_deadband_deg = float(
            self.get_parameter("roll_only_deadband_deg").value
        )
        if self.roll_only_deadband_deg < 0.0:
            raise ValueError("roll_only_deadband_deg must be >= 0.0")
        self.roll_only_filter_alpha = float(
            self.get_parameter("roll_only_filter_alpha").value
        )
        if not 0.0 < self.roll_only_filter_alpha <= 1.0:
            raise ValueError("roll_only_filter_alpha must be in the range (0.0, 1.0]")
        self.roll_lock_toggle_service = self.get_parameter("roll_lock_toggle_service").value
        self.pose_resampler = PoseResampler(
            max_samples=self.pose_buffer_max_samples,
            buffer_duration_sec=self.pose_buffer_duration_sec,
            max_interpolation_gap_sec=self.pose_max_interpolation_gap_sec,
            full_prediction_sec=self.pose_full_prediction_sec,
            prediction_decay_sec=self.pose_prediction_decay_sec,
            velocity_estimation_min_sec=self.pose_velocity_estimation_min_sec,
            max_prediction_linear_speed_m_s=(
                self.pose_max_prediction_linear_speed_m_s
            ),
            max_prediction_angular_speed_deg_s=(
                self.pose_max_prediction_angular_speed_deg_s
            ),
        )
        self.mapping_fingerprint = _mapping_fingerprint(
            {
                "axis_map": self.axis_map,
                "axis_sign": self.axis_sign,
                "enable_orientation_mapping": self.enable_orientation_mapping,
                "enable_xy_yaw_correction": self.enable_xy_yaw_correction,
                "initial_xy_yaw_correction_deg": self.initial_xy_yaw_correction_deg,
                "rot_axis_map": self.rot_axis_map,
                "rot_axis_sign": self.rot_axis_sign,
                "scale_rpy": self.scale_rpy,
                "scale_xyz": self.scale_xyz,
            }
        )
        self.calibration_state = CalibrationState(
            initial_correction_deg=self.initial_xy_yaw_correction_deg,
            mapping_fingerprint=self.mapping_fingerprint,
            required=self.require_xy_yaw_calibration,
        )

        self.latest_raw_vr_pose_m: Optional[list[float]] = None
        self.latest_vr_pose_m: Optional[list[float]] = None
        self.last_accepted_raw_vr_pose_m: Optional[list[float]] = None
        self.vr_anchor_m: Optional[list[float]] = None
        self.latest_raw_vr_quat_xyzw: Optional[list[float]] = None
        self.latest_vr_quat_xyzw: Optional[list[float]] = None
        self.last_accepted_raw_vr_quat_xyzw: Optional[list[float]] = None
        self.vr_anchor_quat_xyzw: Optional[list[float]] = None
        self.last_target_rpy_deg: Optional[list[float]] = self.robot_anchor_rpy_deg[:]
        self.last_orientation_debug: Optional[dict[str, object]] = None
        self.last_roll_only_delta_deg = 0.0
        self.roll_lock_enabled = False
        self.roll_lock_ry_deg: Optional[float] = None
        self.xy_yaw_calibration_active = False
        self.xy_yaw_calibration_start_time = 0.0
        self.xy_yaw_calibration_start_pose_m: Optional[list[float]] = None
        self.last_input_time: Optional[float] = None
        self.timed_out = False
        self.tracking_stale = False
        self.last_resample_mode = "waiting"
        self.last_resample_query_time: Optional[float] = None
        self.last_resample_left_sequence_id: Optional[int] = None
        self.last_resample_right_sequence_id: Optional[int] = None
        self.last_target_log_time = 0.0
        self.last_timeout_status_time = 0.0
        self.last_pose_filter_status_time = 0.0
        self.teleop_ready = False
        self.live_output_enabled = False
        self.last_waiting_ready_status_time = 0.0
        self.last_waiting_calibration_status_time = 0.0

        self.target_pub = self.create_publisher(Float64MultiArray, self.target_posx_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.xy_yaw_calibration_valid_pub = self.create_publisher(
            Bool,
            self.xy_yaw_calibration_valid_topic,
            state_qos,
        )
        self.xy_yaw_calibration_status_pub = self.create_publisher(
            String,
            self.xy_yaw_calibration_status_topic,
            state_qos,
        )
        self.valid_pose_heartbeat_pub = self.create_publisher(
            Empty,
            self.valid_pose_heartbeat_topic,
            10,
        )
        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.input_pose_topic,
            self._on_pose,
            pose_qos,
        )
        self.robot_anchor_sub = self.create_subscription(
            Float64MultiArray,
            self.robot_anchor_posx_topic,
            self._on_robot_anchor_posx,
            state_qos,
        )
        self.teleop_ready_sub = self.create_subscription(
            Bool,
            self.teleop_ready_topic,
            self._on_teleop_ready,
            state_qos,
        )
        self.live_state_sub = self.create_subscription(
            Bool,
            self.live_state_topic,
            self._on_live_state,
            state_qos,
        )
        self.recenter_srv = self.create_service(Trigger, "/vr/recenter", self._on_recenter)
        self.roll_lock_srv = self.create_service(
            Trigger,
            self.roll_lock_toggle_service,
            self._on_toggle_roll_lock,
        )
        self.xy_yaw_calibration_srv = self.create_service(
            Trigger,
            self.xy_yaw_calibration_service,
            self._on_calibrate_xy_yaw_to_x_plus,
        )
        self.xy_yaw_calibration_reset_srv = self.create_service(
            Trigger,
            self.xy_yaw_calibration_reset_service,
            self._on_reset_xy_yaw_calibration,
        )

        self.timer = self.create_timer(self.control_period_sec, self._tick)
        self._publish_status(
            "xyz_mapper_node started: input="
            f"{self.input_pose_topic}, output={self.target_posx_topic}, "
            f"robot_anchor_xyz_mm={self.robot_anchor_xyz_mm}, "
            f"robot_anchor_rpy_deg={self.robot_anchor_rpy_deg}, "
            f"scale_xyz={self.scale_xyz}, axis_sign={self.axis_sign}, "
            f"axis_map={self.axis_map}, "
            f"enable_xy_yaw_correction={self.enable_xy_yaw_correction}, "
            f"xy_yaw_correction_deg={self.xy_yaw_correction_deg}, "
            f"require_xy_yaw_calibration={self.require_xy_yaw_calibration}, "
            f"mapping_fingerprint={self.mapping_fingerprint}, "
            "xy_yaw_calibration_duration_sec="
            f"{self.xy_yaw_calibration_duration_sec}, "
            "xy_yaw_calibration_min_distance_m="
            f"{self.xy_yaw_calibration_min_distance_m}, "
            f"xy_yaw_calibration_service={self.xy_yaw_calibration_service}, "
            f"auto_recenter_on_first_pose={self.auto_recenter_on_first_pose}, "
            f"robot_anchor_posx_topic={self.robot_anchor_posx_topic}, "
            "robot_anchor_qos=KEEP_LAST(depth=1, RELIABLE, TRANSIENT_LOCAL), "
            f"teleop_ready_topic={self.teleop_ready_topic}, "
            f"live_state_topic={self.live_state_topic}, "
            f"valid_pose_heartbeat_topic={self.valid_pose_heartbeat_topic}, "
            f"require_prepare_before_target={self.require_prepare_before_target}, "
            "teleop_ready_qos=KEEP_LAST(depth=1, RELIABLE, TRANSIENT_LOCAL), "
            "pose_qos=KEEP_LAST(depth=1, BEST_EFFORT, VOLATILE), "
            f"publish_rate_hz={self.publish_rate_hz}, "
            f"enable_pose_resampling={self.enable_pose_resampling}, "
            f"pose_interpolation_delay_sec={self.pose_interpolation_delay_sec}, "
            f"pose_buffer_duration_sec={self.pose_buffer_duration_sec}, "
            f"pose_buffer_max_samples={self.pose_buffer_max_samples}, "
            "pose_max_interpolation_gap_sec="
            f"{self.pose_max_interpolation_gap_sec}, "
            f"pose_full_prediction_sec={self.pose_full_prediction_sec}, "
            f"pose_prediction_decay_sec={self.pose_prediction_decay_sec}, "
            "pose_tracking_invalid_after_sec="
            f"{self.pose_tracking_invalid_after_sec}, "
            "pose_max_prediction_linear_speed_m_s="
            f"{self.pose_max_prediction_linear_speed_m_s}, "
            "pose_max_prediction_angular_speed_deg_s="
            f"{self.pose_max_prediction_angular_speed_deg_s}, "
            f"enable_pose_low_pass_filter={self.enable_pose_low_pass_filter}, "
            f"pose_filter_alpha={self.pose_filter_alpha}, "
            f"pose_filter_tick_alpha={self.pose_filter_tick_alpha}, "
            f"pose_filter_reference_rate_hz={self.pose_filter_reference_rate_hz}, "
            f"max_vr_jump_m={self.max_vr_jump_m}, "
            f"enable_orientation_mapping={self.enable_orientation_mapping}, "
            f"scale_rpy={self.scale_rpy}, rot_axis_sign={self.rot_axis_sign}, "
            f"rot_axis_map={self.rot_axis_map}, "
            "enable_orientation_low_pass_filter="
            f"{self.enable_orientation_low_pass_filter}, "
            f"orientation_filter_alpha={self.orientation_filter_alpha}, "
            "orientation_filter_tick_alpha="
            f"{self.orientation_filter_tick_alpha}, "
            f"max_vr_rot_jump_deg={self.max_vr_rot_jump_deg}, "
            f"roll_only_deadband_deg={self.roll_only_deadband_deg}, "
            f"roll_only_filter_alpha={self.roll_only_filter_alpha}, "
            f"roll_lock_toggle_service={self.roll_lock_toggle_service}"
        )
        self._publish_calibration_state()

    def _declare_parameters(self) -> None:
        self.declare_parameter("input_pose_topic", "/q2r_right_hand_pose")
        self.declare_parameter("target_posx_topic", "/vr/target_posx")
        self.declare_parameter("status_topic", "/vr/status")
        self.declare_parameter("robot_anchor_posx_topic", "/vr/robot_anchor_posx")
        self.declare_parameter("teleop_ready_topic", "/vr/teleop_ready")
        self.declare_parameter(
            "live_state_topic",
            "/vr/live_robot_output_enabled",
        )
        self.declare_parameter(
            "valid_pose_heartbeat_topic",
            "/control/metaquest/valid_pose_heartbeat",
        )
        self.declare_parameter("require_prepare_before_target", True)
        self.declare_parameter("robot_anchor_xyz_mm", [400.0, 0.0, 350.0])
        self.declare_parameter("robot_anchor_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("scale_xyz", [0.5, 0.5, 0.5])
        self.declare_parameter("axis_sign", [1.0, 1.0, 1.0])
        self.declare_parameter("axis_map", [0, 1, 2])
        self.declare_parameter("enable_xy_yaw_correction", True)
        self.declare_parameter("xy_yaw_correction_deg", 0.0)
        self.declare_parameter("require_xy_yaw_calibration", True)
        self.declare_parameter("xy_yaw_calibration_duration_sec", 2.0)
        self.declare_parameter("xy_yaw_calibration_min_distance_m", 0.04)
        self.declare_parameter(
            "xy_yaw_calibration_service",
            "/vr/calibrate_xy_yaw_to_x_plus",
        )
        self.declare_parameter(
            "xy_yaw_calibration_reset_service",
            "/vr/reset_xy_yaw_calibration",
        )
        self.declare_parameter(
            "xy_yaw_calibration_valid_topic",
            "/vr/metaquest_calibration/valid",
        )
        self.declare_parameter(
            "xy_yaw_calibration_status_topic",
            "/vr/metaquest_calibration/status",
        )
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("auto_recenter_on_first_pose", True)
        self.declare_parameter("recenter_vr_on_robot_anchor_update", True)
        self.declare_parameter("input_timeout_sec", 0.5)
        self.declare_parameter("enable_pose_resampling", True)
        self.declare_parameter("pose_interpolation_delay_sec", 0.05)
        self.declare_parameter("pose_buffer_duration_sec", 1.0)
        self.declare_parameter("pose_buffer_max_samples", 128)
        self.declare_parameter("pose_max_interpolation_gap_sec", 0.08)
        self.declare_parameter("pose_full_prediction_sec", 0.02)
        self.declare_parameter("pose_prediction_decay_sec", 0.08)
        self.declare_parameter("pose_velocity_estimation_min_sec", 0.02)
        self.declare_parameter("pose_max_prediction_linear_speed_m_s", 0.5)
        self.declare_parameter(
            "pose_max_prediction_angular_speed_deg_s",
            90.0,
        )
        self.declare_parameter("pose_tracking_invalid_after_sec", 0.3)
        self.declare_parameter("pose_filter_reference_rate_hz", 72.0)
        self.declare_parameter("enable_pose_low_pass_filter", True)
        self.declare_parameter("pose_filter_alpha", 0.2)
        self.declare_parameter("max_vr_jump_m", 0.15)
        self.declare_parameter("enable_orientation_mapping", False)
        self.declare_parameter("scale_rpy", [0.5, 0.5, 0.5])
        self.declare_parameter("rot_axis_sign", [1.0, -1.0, -1.0])
        self.declare_parameter("rot_axis_map", [1, 0, 2])
        self.declare_parameter("enable_orientation_low_pass_filter", True)
        self.declare_parameter("orientation_filter_alpha", 0.2)
        self.declare_parameter("max_vr_rot_jump_deg", 45.0)
        self.declare_parameter("roll_only_deadband_deg", 1.0)
        self.declare_parameter("roll_only_filter_alpha", 0.35)
        self.declare_parameter("roll_lock_toggle_service", "/vr/toggle_roll_lock")

    def _on_pose(self, msg: PoseStamped) -> None:
        raw_pose_m = [
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        ]
        if any(not math.isfinite(value) for value in raw_pose_m):
            self._publish_status(f"ignored non-finite Quest pose: {raw_pose_m}", warn=True)
            return
        try:
            raw_quat_xyzw = _quat_xyzw(
                [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ],
                "Quest orientation",
            )
        except ValueError as exc:
            self._publish_status(f"ignored invalid Quest orientation: {exc}", warn=True)
            return

        now = time.monotonic()
        # Preserve the latest finite, normalized raw sample so an explicit
        # recenter can recover from a legitimate tracking-frame jump. This is
        # deliberately not a freshness update; rejected samples must not keep
        # the mapper or MUX watchdog alive.
        self.latest_raw_vr_pose_m = raw_pose_m[:]
        self.latest_raw_vr_quat_xyzw = raw_quat_xyzw[:]
        jump = self._pose_jump(raw_pose_m, raw_quat_xyzw)
        if jump is not None:
            if self.xy_yaw_calibration_active:
                self._cancel_xy_yaw_calibration("Quest tracking jump rejected")
            elif self.calibration_state.snapshot.valid:
                self._invalidate_xy_yaw_calibration(
                    "Quest tracking jump rejected; repeat XY +X calibration."
                )
            if now - self.last_pose_filter_status_time >= 1.0:
                self._publish_status(
                    "Quest pose jump rejected: "
                    + json.dumps(jump, sort_keys=True),
                    warn=True,
                )
                self.last_pose_filter_status_time = now
            return

        previous_input_time = self.last_input_time
        receive_gap_sec = (
            None if previous_input_time is None else now - previous_input_time
        )
        if (
            receive_gap_sec is not None
            and receive_gap_sec > self.pose_tracking_invalid_after_sec
        ):
            self._reset_pose_resampling(
                "Quest receive gap exceeded tracking validity",
                seed_latest=False,
                reset_filtered_state=True,
            )
            if self.xy_yaw_calibration_active:
                self._cancel_xy_yaw_calibration("Quest tracking became stale")
            # A short receive gap means that prediction must stop and the
            # resampling history is no longer continuous. It does not mean
            # that an already measured Quest-to-robot yaw correction became
            # invalid. The longer input_timeout_sec path still invalidates it
            # when the input actually disappears.

        self.last_input_time = now
        if self.timed_out:
            self.timed_out = False
            self._publish_status("Quest input resumed")
        if self.tracking_stale:
            self.tracking_stale = False
            self._publish_status(
                "Quest tracking input resumed; waiting for the resampling buffer"
            )

        self.last_accepted_raw_vr_pose_m = raw_pose_m[:]
        self.last_accepted_raw_vr_quat_xyzw = raw_quat_xyzw[:]
        self.pose_resampler.push(
            receive_time_sec=now,
            position_m=raw_pose_m,
            quaternion_xyzw=raw_quat_xyzw,
        )
        self.valid_pose_heartbeat_pub.publish(Empty())

    def _on_robot_anchor_posx(self, msg: Float64MultiArray) -> None:
        try:
            posx = [float(value) for value in msg.data]
        except (TypeError, ValueError) as exc:
            self._publish_status(f"ignored invalid robot anchor posx: {exc}", warn=True)
            return
        if len(posx) != 6 or any(not math.isfinite(value) for value in posx):
            self._publish_status(
                f"ignored invalid robot anchor posx; expected 6 finite values, got {posx}",
                warn=True,
            )
            return

        self.robot_anchor_xyz_mm = posx[:3]
        self.robot_anchor_rpy_deg = posx[3:6]
        self.last_target_rpy_deg = self.robot_anchor_rpy_deg[:]
        self.last_orientation_debug = None
        self.last_roll_only_delta_deg = 0.0
        self._clear_roll_lock("robot anchor updated")
        self._cancel_xy_yaw_calibration("robot anchor updated")
        status = {
            "robot_anchor_xyz_mm": self.robot_anchor_xyz_mm,
            "robot_anchor_rpy_deg": self.robot_anchor_rpy_deg,
        }
        if self.recenter_vr_on_robot_anchor_update and self._has_any_vr_pose():
            status.update(self._accept_latest_pose_as_anchor())
            self._reset_pose_resampling(
                "robot anchor updated",
                seed_latest=True,
                reset_filtered_state=False,
            )
        self._publish_status(
            "robot anchor updated from current TCP: " + json.dumps(status, sort_keys=True)
        )

    def _on_teleop_ready(self, msg: Bool) -> None:
        ready = bool(msg.data)
        if ready != self.teleop_ready:
            self.teleop_ready = ready
            if ready:
                self._reset_pose_resampling(
                    "teleop became ready",
                    seed_latest=True,
                    reset_filtered_state=False,
                )
            self._publish_status(f"teleop_ready={self.teleop_ready}")

    def _on_live_state(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        if enabled and not self.live_output_enabled:
            self._reset_pose_resampling(
                "Live output enabled",
                seed_latest=True,
                reset_filtered_state=False,
            )
        if enabled != self.live_output_enabled:
            self.live_output_enabled = enabled
            self._publish_status(f"live_output_enabled={enabled}")

    def _on_recenter(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if not self._has_any_vr_pose():
            response.success = False
            response.message = "Cannot recenter: no Quest pose received yet."
            self._publish_status(response.message, warn=True)
            return response
        self._cancel_xy_yaw_calibration("VR recentered")
        self._clear_roll_lock("VR recentered")
        anchors = self._accept_latest_pose_as_anchor()
        self._reset_pose_resampling(
            "VR recentered",
            seed_latest=True,
            reset_filtered_state=False,
        )
        response.success = True
        response.message = (
            "VR anchor recentered to latest raw Quest pose/orientation and jump filter reset: "
            + json.dumps(anchors, sort_keys=True)
        )
        self._publish_status(response.message)
        return response

    def _on_calibrate_xy_yaw_to_x_plus(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if not self.enable_xy_yaw_correction:
            response.success = False
            response.message = "Cannot calibrate XY yaw: enable_xy_yaw_correction=false."
            self._publish_status(response.message, warn=True)
            return response
        if self.latest_vr_pose_m is None:
            response.success = False
            response.message = "Cannot calibrate XY yaw: no Quest pose received yet."
            self._publish_status(response.message, warn=True)
            return response

        self.xy_yaw_calibration_active = True
        self.xy_yaw_calibration_start_time = time.monotonic()
        self.xy_yaw_calibration_start_pose_m = self.latest_vr_pose_m[:]
        self._reset_pose_resampling(
            "XY yaw calibration started",
            seed_latest=True,
            reset_filtered_state=False,
        )
        self._publish_calibration_state(
            self.calibration_state.start(self.xy_yaw_correction_deg)
        )
        response.success = True
        response.message = (
            "XY yaw calibration started. Move the Quest controller in desired robot +X "
            f"direction for {self.xy_yaw_calibration_duration_sec:.2f}s."
        )
        self._publish_status(
            response.message
            + " "
            + json.dumps(
                {"start_pose_m": self.xy_yaw_calibration_start_pose_m},
                sort_keys=True,
            )
        )
        return response

    def _on_reset_xy_yaw_calibration(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self.xy_yaw_calibration_active = False
        self.xy_yaw_calibration_start_pose_m = None
        self.xy_yaw_correction_deg = self.initial_xy_yaw_correction_deg
        self._reset_pose_resampling(
            "XY yaw calibration reset",
            seed_latest=True,
            reset_filtered_state=False,
        )
        snapshot = self.calibration_state.invalidate(
            "XY yaw calibration reset; repeat calibration before MetaQuest control.",
            correction_deg=self.xy_yaw_correction_deg,
            reset_correction=True,
        )
        self._publish_calibration_state(snapshot)
        response.success = True
        response.message = (
            "XY yaw calibration reset to configured correction "
            f"{self.xy_yaw_correction_deg:.3f} deg."
        )
        self._publish_status(response.message, warn=True)
        return response

    def _on_toggle_roll_lock(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if not self.roll_lock_enabled:
            self.roll_lock_ry_deg = self._current_target_ry_deg()
            self.roll_lock_enabled = True
            response.success = True
            response.message = f"Roll lock enabled at robot ry={self.roll_lock_ry_deg:.3f} deg."
            self._publish_status(response.message)
            return response

        held_ry = self._current_target_ry_deg()
        release_info = self._release_roll_lock(held_ry)
        response.success = True
        response.message = "Roll lock disabled: " + json.dumps(release_info, sort_keys=True)
        self._publish_status(response.message)
        return response

    def _tick(self) -> None:
        now = time.monotonic()
        if self.last_input_time is None:
            return
        input_age_sec = now - self.last_input_time
        if input_age_sec > self.input_timeout_sec:
            if not self.timed_out:
                if self.xy_yaw_calibration_active:
                    self._cancel_xy_yaw_calibration("Quest input timeout")
                elif self.calibration_state.snapshot.valid:
                    self._invalidate_xy_yaw_calibration(
                        "Quest input timed out; repeat XY +X calibration."
                    )
            if not self.timed_out or now - self.last_timeout_status_time >= 1.0:
                self._publish_status(
                    "Quest input timeout; holding target publication. "
                    f"age_sec={input_age_sec}",
                    warn=True,
                )
                self.last_timeout_status_time = now
            self.timed_out = True
            return

        if input_age_sec > self.pose_tracking_invalid_after_sec:
            if not self.tracking_stale:
                self.tracking_stale = True
                self._reset_pose_resampling(
                    "Quest tracking became stale",
                    seed_latest=False,
                    reset_filtered_state=True,
                )
                if self.xy_yaw_calibration_active:
                    self._cancel_xy_yaw_calibration("Quest tracking became stale")
                # Preserve completed frame calibration during a bounded
                # tracking hold. No target is published in this branch, and
                # the input-timeout path invalidates it if the gap continues.
            if now - self.last_timeout_status_time >= 1.0:
                self._publish_status(
                    "Quest tracking stale; resampled target is held and publication "
                    f"is stopped. age_sec={input_age_sec}",
                    warn=True,
                )
                self.last_timeout_status_time = now
            return

        if not self._update_control_pose(now):
            return
        if self.latest_vr_pose_m is None:
            return
        if self.vr_anchor_m is None and self.auto_recenter_on_first_pose:
            self.vr_anchor_m = self.latest_vr_pose_m[:]
            if self.latest_vr_quat_xyzw is not None:
                self.vr_anchor_quat_xyzw = self.latest_vr_quat_xyzw[:]
            self._publish_status(
                "VR anchor set from first resampled pose: "
                + json.dumps(
                    {
                        "vr_anchor_m": self.vr_anchor_m,
                        "vr_anchor_quat_xyzw": self.vr_anchor_quat_xyzw,
                    },
                    sort_keys=True,
                )
            )
        if self.vr_anchor_m is None:
            return
        if (
            self.enable_orientation_mapping
            and (self.latest_vr_quat_xyzw is None or self.vr_anchor_quat_xyzw is None)
        ):
            return

        if self.xy_yaw_calibration_active:
            self._update_xy_yaw_calibration(now)
            return

        if self.require_xy_yaw_calibration and not self.calibration_state.snapshot.valid:
            if now - self.last_waiting_calibration_status_time >= 1.0:
                self._publish_status(
                    "holding MetaQuest target publication until XY +X calibration is valid"
                )
                self.last_waiting_calibration_status_time = now
            return

        if self.require_prepare_before_target and not self.teleop_ready:
            if now - self.last_waiting_ready_status_time >= 1.0:
                self._publish_status(
                    "holding target publication until /vr/prepare_robot completes "
                    f"and {self.teleop_ready_topic} is true"
                )
                self.last_waiting_ready_status_time = now
            return

        delta_m = self._vr_delta_m()
        mapped_delta_m = self._mapped_delta_m(delta_m)
        target = self._compute_target_posx(mapped_delta_m)
        msg = Float64MultiArray()
        msg.data = target
        self.target_pub.publish(msg)
        if now - self.last_target_log_time >= 1.0:
            status = {
                "data": target,
                "latest_raw_vr_pose_m": self.latest_raw_vr_pose_m,
                "latest_vr_pose_m": self.latest_vr_pose_m,
                "mapped_delta_m": mapped_delta_m,
                "axis_map": self.axis_map,
                "axis_sign": self.axis_sign,
                "enable_xy_yaw_correction": self.enable_xy_yaw_correction,
                "xy_yaw_correction_deg": self.xy_yaw_correction_deg,
                "vr_delta_m": delta_m,
                "vr_anchor_m": self.vr_anchor_m,
                "pose_resampling": {
                    "enabled": self.enable_pose_resampling,
                    "mode": self.last_resample_mode,
                    "buffer_samples": self.pose_resampler.sample_count,
                    "query_time": self.last_resample_query_time,
                    "left_sequence_id": self.last_resample_left_sequence_id,
                    "right_sequence_id": self.last_resample_right_sequence_id,
                    "input_age_sec": now - self.last_input_time,
                    "interpolation_delay_sec": self.pose_interpolation_delay_sec,
                },
            }
            if self.enable_orientation_mapping and self.last_orientation_debug is not None:
                status.update(self.last_orientation_debug)
            self._publish_status(
                "target_posx=" + json.dumps(status, sort_keys=True)
            )
            self.last_target_log_time = now

    def _update_control_pose(self, now: float) -> bool:
        if self.latest_raw_vr_pose_m is None or self.latest_raw_vr_quat_xyzw is None:
            self.last_resample_mode = "waiting"
            return False

        if self.enable_pose_resampling:
            query_time = now - self.pose_interpolation_delay_sec
            resampled = self.pose_resampler.sample(query_time)
            if resampled is None:
                self.last_resample_mode = "buffering"
                self.last_resample_query_time = query_time
                return False
            pose_m = list(resampled.position_m)
            quaternion_xyzw = list(resampled.quaternion_xyzw)
            self.last_resample_mode = resampled.mode
            self.last_resample_query_time = resampled.query_time_sec
            self.last_resample_left_sequence_id = resampled.left_sequence_id
            self.last_resample_right_sequence_id = resampled.right_sequence_id
        else:
            pose_m = self.latest_raw_vr_pose_m[:]
            quaternion_xyzw = self.latest_raw_vr_quat_xyzw[:]
            self.last_resample_mode = "latest"
            self.last_resample_query_time = now
            self.last_resample_left_sequence_id = None
            self.last_resample_right_sequence_id = None

        self.latest_vr_pose_m = self._filtered_pose(pose_m)
        self.latest_vr_quat_xyzw = self._filtered_quat(quaternion_xyzw)
        return True

    def _reset_pose_resampling(
        self,
        reason: str,
        *,
        seed_latest: bool,
        reset_filtered_state: bool,
    ) -> None:
        self.pose_resampler.clear()
        self.last_resample_mode = f"reset:{reason}"
        self.last_resample_query_time = None
        self.last_resample_left_sequence_id = None
        self.last_resample_right_sequence_id = None
        if reset_filtered_state:
            self.latest_vr_pose_m = None
            self.latest_vr_quat_xyzw = None
            self.last_orientation_debug = None
            self.last_roll_only_delta_deg = 0.0
        if (
            seed_latest
            and self.latest_raw_vr_pose_m is not None
            and self.latest_raw_vr_quat_xyzw is not None
        ):
            self.pose_resampler.push(
                receive_time_sec=time.monotonic(),
                position_m=self.latest_raw_vr_pose_m,
                quaternion_xyzw=self.latest_raw_vr_quat_xyzw,
            )

    def _pose_jump(
        self,
        raw_pose_m: list[float],
        raw_quat_xyzw: list[float],
    ) -> Optional[dict[str, object]]:
        jump: dict[str, object] = {}
        if self.max_vr_jump_m > 0.0 and self.last_accepted_raw_vr_pose_m is not None:
            jump_m = _distance3(raw_pose_m, self.last_accepted_raw_vr_pose_m)
            if jump_m > self.max_vr_jump_m:
                jump.update(
                    {
                        "raw_vr_pose_m": raw_pose_m,
                        "last_accepted_raw_vr_pose_m": self.last_accepted_raw_vr_pose_m,
                        "jump_m": jump_m,
                        "max_vr_jump_m": self.max_vr_jump_m,
                    }
                )
        if (
            self.enable_orientation_mapping
            and self.max_vr_rot_jump_deg > 0.0
            and self.last_accepted_raw_vr_quat_xyzw is not None
        ):
            jump_deg = _quat_angle_deg(raw_quat_xyzw, self.last_accepted_raw_vr_quat_xyzw)
            if jump_deg > self.max_vr_rot_jump_deg:
                jump.update(
                    {
                        "raw_vr_quat_xyzw": raw_quat_xyzw,
                        "last_accepted_raw_vr_quat_xyzw": (
                            self.last_accepted_raw_vr_quat_xyzw
                        ),
                        "rot_jump_deg": jump_deg,
                        "max_vr_rot_jump_deg": self.max_vr_rot_jump_deg,
                    }
                )
        return jump if jump else None

    def _has_any_vr_pose(self) -> bool:
        return self.latest_raw_vr_pose_m is not None or self.latest_vr_pose_m is not None

    def _cancel_xy_yaw_calibration(self, reason: str) -> None:
        if not self.xy_yaw_calibration_active:
            return
        self.xy_yaw_calibration_active = False
        self.xy_yaw_calibration_start_pose_m = None
        self._invalidate_xy_yaw_calibration(
            f"XY yaw calibration cancelled: {reason}."
        )
        self._publish_status(f"XY yaw calibration cancelled: {reason}", warn=True)

    def _invalidate_xy_yaw_calibration(self, reason: str) -> None:
        snapshot = self.calibration_state.invalidate(
            reason,
            correction_deg=self.xy_yaw_correction_deg,
        )
        self._publish_calibration_state(snapshot)

    def _accept_latest_position_as_anchor(self) -> dict[str, list[float]]:
        anchors: dict[str, list[float]] = {}
        if self.latest_raw_vr_pose_m is not None:
            self.last_accepted_raw_vr_pose_m = self.latest_raw_vr_pose_m[:]
        if self.latest_vr_pose_m is not None:
            self.vr_anchor_m = self.latest_vr_pose_m[:]
            anchors["vr_anchor_m"] = self.vr_anchor_m[:]
        return anchors

    def _accept_latest_pose_as_anchor(self) -> dict[str, list[float]]:
        anchors: dict[str, list[float]] = {}
        if self.latest_raw_vr_pose_m is not None:
            self.last_accepted_raw_vr_pose_m = self.latest_raw_vr_pose_m[:]
            self.latest_vr_pose_m = self.latest_raw_vr_pose_m[:]
        if self.latest_vr_pose_m is not None:
            self.vr_anchor_m = self.latest_vr_pose_m[:]
            anchors["vr_anchor_m"] = self.vr_anchor_m[:]
        if self.latest_raw_vr_quat_xyzw is not None:
            self.last_accepted_raw_vr_quat_xyzw = self.latest_raw_vr_quat_xyzw[:]
            self.latest_vr_quat_xyzw = self.latest_raw_vr_quat_xyzw[:]
        if self.latest_vr_quat_xyzw is not None:
            self.vr_anchor_quat_xyzw = self.latest_vr_quat_xyzw[:]
            anchors["vr_anchor_quat_xyzw"] = self.vr_anchor_quat_xyzw[:]
        self.last_target_rpy_deg = self.robot_anchor_rpy_deg[:]
        self.last_orientation_debug = None
        self.last_roll_only_delta_deg = 0.0
        self.roll_lock_enabled = False
        self.roll_lock_ry_deg = None
        return anchors

    def _current_target_ry_deg(self) -> float:
        if self.roll_lock_ry_deg is not None:
            return float(self.roll_lock_ry_deg)
        if self.last_target_rpy_deg is not None:
            return float(self.last_target_rpy_deg[1])
        return float(self.robot_anchor_rpy_deg[1])

    def _clear_roll_lock(self, reason: str) -> None:
        if not self.roll_lock_enabled and self.roll_lock_ry_deg is None:
            return
        previous = {
            "reason": reason,
            "roll_lock_ry_deg": self.roll_lock_ry_deg,
        }
        self.roll_lock_enabled = False
        self.roll_lock_ry_deg = None
        self._publish_status("Roll lock cleared: " + json.dumps(previous, sort_keys=True))

    def _release_roll_lock(self, held_ry_deg: float) -> dict[str, object]:
        self.roll_lock_enabled = False
        self.roll_lock_ry_deg = None
        previous_anchor_rpy = self.robot_anchor_rpy_deg[:]
        release_info: dict[str, object] = {
            "held_ry_deg": held_ry_deg,
            "previous_robot_anchor_rpy_deg": previous_anchor_rpy,
            "roll_reference_reset": False,
        }
        if self._is_roll_only_ry_mode() and self.latest_vr_quat_xyzw is not None:
            self.robot_anchor_rpy_deg[1] = held_ry_deg
            self.vr_anchor_quat_xyzw = self.latest_vr_quat_xyzw[:]
            if self.latest_raw_vr_quat_xyzw is not None:
                self.last_accepted_raw_vr_quat_xyzw = self.latest_raw_vr_quat_xyzw[:]
            self.last_roll_only_delta_deg = 0.0
            self.last_target_rpy_deg = [
                self.robot_anchor_rpy_deg[0],
                held_ry_deg,
                self.robot_anchor_rpy_deg[2],
            ]
            self.last_orientation_debug = None
            release_info.update(
                {
                    "roll_reference_reset": True,
                    "new_robot_anchor_rpy_deg": self.robot_anchor_rpy_deg[:],
                    "new_vr_anchor_quat_xyzw": self.vr_anchor_quat_xyzw[:],
                }
            )
        return release_info

    def _filtered_pose(self, raw_pose_m: list[float]) -> list[float]:
        if not self.enable_pose_low_pass_filter or self.latest_vr_pose_m is None:
            return raw_pose_m[:]
        alpha = self.pose_filter_tick_alpha
        return [
            self.latest_vr_pose_m[index]
            + alpha * (raw_pose_m[index] - self.latest_vr_pose_m[index])
            for index in range(3)
        ]

    def _filtered_quat(self, raw_quat_xyzw: list[float]) -> list[float]:
        if (
            not self.enable_orientation_low_pass_filter
            or self.latest_vr_quat_xyzw is None
        ):
            return raw_quat_xyzw[:]
        return list(
            quaternion_slerp(
                self.latest_vr_quat_xyzw,
                raw_quat_xyzw,
                self.orientation_filter_tick_alpha,
            )
        )

    def _vr_delta_m(self) -> list[float]:
        assert self.latest_vr_pose_m is not None
        assert self.vr_anchor_m is not None
        return [
            self.latest_vr_pose_m[index] - self.vr_anchor_m[index]
            for index in range(3)
        ]

    def _mapped_delta_m(self, delta_m: list[float]) -> list[float]:
        return [delta_m[self.axis_map[index]] for index in range(3)]

    def _position_delta_before_yaw_mm(self, mapped_delta_m: list[float]) -> list[float]:
        return [
            self.axis_sign[index] * self.scale_xyz[index] * mapped_delta_m[index] * 1000.0
            for index in range(3)
        ]

    def _apply_xy_yaw_correction_mm(self, delta_mm: list[float]) -> list[float]:
        if not self.enable_xy_yaw_correction or abs(self.xy_yaw_correction_deg) <= 1.0e-9:
            return delta_mm[:]
        corrected_xy = _rotate_xy(delta_mm[:2], self.xy_yaw_correction_deg)
        return [corrected_xy[0], corrected_xy[1], delta_mm[2]]

    def _update_xy_yaw_calibration(self, now: float) -> None:
        if (
            self.xy_yaw_calibration_start_pose_m is None
            or self.latest_vr_pose_m is None
        ):
            self._cancel_xy_yaw_calibration("missing Quest pose")
            return
        elapsed = now - self.xy_yaw_calibration_start_time
        if elapsed < self.xy_yaw_calibration_duration_sec:
            return

        raw_delta_m = [
            self.latest_vr_pose_m[index] - self.xy_yaw_calibration_start_pose_m[index]
            for index in range(3)
        ]
        mapped_delta_m = self._mapped_delta_m(raw_delta_m)
        before_yaw_mm = self._position_delta_before_yaw_mm(mapped_delta_m)
        distance_m = math.hypot(before_yaw_mm[0], before_yaw_mm[1]) / 1000.0
        if distance_m < self.xy_yaw_calibration_min_distance_m:
            self.xy_yaw_calibration_active = False
            self.xy_yaw_calibration_start_pose_m = None
            self._invalidate_xy_yaw_calibration(
                "XY yaw calibration failed because measured planar movement was too small."
            )
            self._publish_status(
                "XY yaw calibration failed: move distance too small. "
                + json.dumps(
                    {
                        "distance_m": distance_m,
                        "min_distance_m": self.xy_yaw_calibration_min_distance_m,
                        "raw_delta_m": raw_delta_m,
                        "mapped_delta_m": mapped_delta_m,
                        "position_delta_before_yaw_mm": before_yaw_mm,
                    },
                    sort_keys=True,
                ),
                warn=True,
            )
            return

        observed_angle_deg = math.degrees(math.atan2(before_yaw_mm[1], before_yaw_mm[0]))
        self.xy_yaw_correction_deg = _shortest_angle_delta_deg(
            -observed_angle_deg,
            0.0,
        )
        corrected_mm = self._apply_xy_yaw_correction_mm(before_yaw_mm)
        anchors = self._accept_latest_position_as_anchor()
        self._reset_pose_resampling(
            "XY yaw calibration completed",
            seed_latest=True,
            reset_filtered_state=False,
        )
        self.xy_yaw_calibration_active = False
        self.xy_yaw_calibration_start_pose_m = None
        self._publish_calibration_state(
            self.calibration_state.complete(
                correction_deg=self.xy_yaw_correction_deg,
                observed_angle_deg=observed_angle_deg,
                distance_m=distance_m,
                before_yaw_mm=before_yaw_mm,
                after_yaw_mm=corrected_mm,
            )
        )
        self._publish_status(
            "XY yaw calibration complete: "
            + json.dumps(
                {
                    "observed_angle_deg": observed_angle_deg,
                    "xy_yaw_correction_deg": self.xy_yaw_correction_deg,
                    "distance_m": distance_m,
                    "raw_delta_m": raw_delta_m,
                    "mapped_delta_m": mapped_delta_m,
                    "position_delta_before_yaw_mm": before_yaw_mm,
                    "position_delta_after_yaw_mm": corrected_mm,
                    "anchors": anchors,
                },
                sort_keys=True,
            )
        )

    def _orientation_delta_quat_xyzw(self) -> list[float]:
        assert self.latest_vr_quat_xyzw is not None
        assert self.vr_anchor_quat_xyzw is not None
        return _quat_multiply(
            self.latest_vr_quat_xyzw,
            _quat_conjugate(self.vr_anchor_quat_xyzw),
        )

    def _orientation_delta_rpy_deg(self) -> list[float]:
        return _quat_to_euler_xyz_deg(self._orientation_delta_quat_xyzw())

    def _mapped_orientation_delta_rpy_deg(
        self,
        delta_rpy_deg: list[float],
    ) -> list[float]:
        return [
            self.rot_axis_sign[index]
            * self.scale_rpy[index]
            * delta_rpy_deg[self.rot_axis_map[index]]
            for index in range(3)
        ]

    def _is_roll_only_ry_mode(self) -> bool:
        return (
            abs(self.scale_rpy[0]) <= 1.0e-9
            and abs(self.scale_rpy[1]) > 1.0e-9
            and abs(self.scale_rpy[2]) <= 1.0e-9
            and self.rot_axis_map[1] == 0
        )

    def _filtered_roll_only_delta_deg(
        self,
        raw_delta_deg: float,
    ) -> tuple[float, dict[str, float]]:
        deadbanded_delta_deg = (
            0.0
            if abs(raw_delta_deg) < self.roll_only_deadband_deg
            else raw_delta_deg
        )
        filtered_delta_deg = self.last_roll_only_delta_deg + (
            self.roll_only_filter_alpha
            * (deadbanded_delta_deg - self.last_roll_only_delta_deg)
        )
        self.last_roll_only_delta_deg = filtered_delta_deg
        return filtered_delta_deg, {
            "roll_only_raw_delta_deg": raw_delta_deg,
            "roll_only_deadbanded_delta_deg": deadbanded_delta_deg,
            "roll_only_filtered_delta_deg": filtered_delta_deg,
            "roll_only_deadband_deg": self.roll_only_deadband_deg,
            "roll_only_filter_alpha": self.roll_only_filter_alpha,
        }

    def _compute_target_orientation(self) -> tuple[list[float], dict[str, object]]:
        orientation_delta_quat = self._orientation_delta_quat_xyzw()
        orientation_delta_rpy = _quat_to_euler_xyz_deg(orientation_delta_quat)
        mapped_delta_rpy = self._mapped_orientation_delta_rpy_deg(
            orientation_delta_rpy
        )
        roll_only_debug: dict[str, float] = {}
        roll_only_mode = self._is_roll_only_ry_mode()
        roll_lock_active = self.roll_lock_enabled and self.roll_lock_ry_deg is not None
        raw_roll_delta_for_debug = mapped_delta_rpy[1]
        if roll_only_mode and not roll_lock_active:
            filtered_roll_delta, roll_only_debug = self._filtered_roll_only_delta_deg(
                mapped_delta_rpy[1]
            )
            mapped_delta_rpy = [0.0, filtered_roll_delta, 0.0]
        robot_anchor_quat = doosan_zyz_deg_to_quaternion(
            self.robot_anchor_rpy_deg
        )
        reference_rpy = (
            self.last_target_rpy_deg[:]
            if self.last_target_rpy_deg is not None
            else self.robot_anchor_rpy_deg[:]
        )
        if roll_only_mode:
            if roll_lock_active:
                target_ry = _angle_near_reference_deg(
                    float(self.roll_lock_ry_deg),
                    reference_rpy[1],
                )
                mapped_delta_rpy = [
                    0.0,
                    _shortest_angle_delta_deg(target_ry, self.robot_anchor_rpy_deg[1]),
                    0.0,
                ]
                roll_only_debug = {
                    "roll_only_raw_delta_deg": raw_roll_delta_for_debug,
                    "roll_only_deadbanded_delta_deg": raw_roll_delta_for_debug,
                    "roll_only_filtered_delta_deg": self.last_roll_only_delta_deg,
                    "roll_only_deadband_deg": self.roll_only_deadband_deg,
                    "roll_only_filter_alpha": self.roll_only_filter_alpha,
                }
                orientation_mapping_mode = (
                    "roll_only_physical_zyz_b_axis_continuous_locked"
                )
            else:
                orientation_mapping_mode = (
                    "roll_only_physical_zyz_b_axis_continuous"
                )
            mapped_delta_quat = doosan_zyz_b_delta_quaternion(
                self.robot_anchor_rpy_deg,
                mapped_delta_rpy[1],
            )
            target_rpy = apply_doosan_zyz_b_delta_deg(
                self.robot_anchor_rpy_deg,
                mapped_delta_rpy[1],
                reference_rpy,
            )
            target_quat = doosan_zyz_deg_to_quaternion(target_rpy)
            canonical_target_rpy = quaternion_to_doosan_zyz_deg(target_quat)
        else:
            mapped_delta_quat = _euler_xyz_deg_to_quat(mapped_delta_rpy)
            target_quat = _quat_multiply(mapped_delta_quat, robot_anchor_quat)
            canonical_target_rpy = quaternion_to_doosan_zyz_deg(target_quat)
            target_rpy = quaternion_to_doosan_zyz_deg(
                target_quat,
                reference_rpy,
            )
            orientation_mapping_mode = (
                "quaternion_anchor_nearest_doosan_zyz"
            )
        debug = {
            "latest_raw_vr_quat_xyzw": self.latest_raw_vr_quat_xyzw,
            "latest_vr_quat_xyzw": self.latest_vr_quat_xyzw,
            "vr_anchor_quat_xyzw": self.vr_anchor_quat_xyzw,
            "orientation_mapping_mode": orientation_mapping_mode,
            "orientation_delta_quat_xyzw": orientation_delta_quat,
            "orientation_delta_rpy_deg": orientation_delta_rpy,
            "mapped_orientation_delta_rpy_deg": mapped_delta_rpy,
            "mapped_orientation_delta_quat_xyzw": mapped_delta_quat,
            "robot_anchor_quat_xyzw": robot_anchor_quat,
            "target_quat_xyzw": target_quat,
            "canonical_target_rpy_deg": canonical_target_rpy,
            "target_rpy_reference_deg": reference_rpy,
            "target_rpy_deg": target_rpy,
            "rot_axis_map": self.rot_axis_map,
            "rot_axis_sign": self.rot_axis_sign,
            "scale_rpy": self.scale_rpy,
            "roll_lock_enabled": self.roll_lock_enabled,
            "roll_lock_ry_deg": self.roll_lock_ry_deg,
        }
        debug.update(roll_only_debug)
        return target_rpy, debug

    def _compute_target_posx(self, mapped_delta_m: list[float] | None = None) -> list[float]:
        if mapped_delta_m is None:
            mapped_delta_m = self._mapped_delta_m(self._vr_delta_m())
        delta_before_yaw_mm = self._position_delta_before_yaw_mm(mapped_delta_m)
        delta_after_yaw_mm = self._apply_xy_yaw_correction_mm(delta_before_yaw_mm)
        target_xyz = [
            self.robot_anchor_xyz_mm[index] + delta_after_yaw_mm[index]
            for index in range(3)
        ]
        if self.enable_orientation_mapping:
            target_rpy, orientation_debug = self._compute_target_orientation()
            orientation_debug.update(
                {
                    "position_delta_before_yaw_mm": delta_before_yaw_mm,
                    "position_delta_after_yaw_mm": delta_after_yaw_mm,
                    "enable_xy_yaw_correction": self.enable_xy_yaw_correction,
                    "xy_yaw_correction_deg": self.xy_yaw_correction_deg,
                    "xy_yaw_calibration_active": self.xy_yaw_calibration_active,
                }
            )
            self.last_target_rpy_deg = target_rpy[:]
            self.last_orientation_debug = orientation_debug
        else:
            target_rpy = self.robot_anchor_rpy_deg[:]
            self.last_target_rpy_deg = target_rpy[:]
            self.last_orientation_debug = {
                "position_delta_before_yaw_mm": delta_before_yaw_mm,
                "position_delta_after_yaw_mm": delta_after_yaw_mm,
                "enable_xy_yaw_correction": self.enable_xy_yaw_correction,
                "xy_yaw_correction_deg": self.xy_yaw_correction_deg,
                "xy_yaw_calibration_active": self.xy_yaw_calibration_active,
            }
        return target_xyz + target_rpy

    def _publish_status(self, text: str, warn: bool = False) -> None:
        self.status_pub.publish(String(data=text))
        if warn:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)

    def _publish_calibration_state(self, snapshot=None) -> None:
        current = self.calibration_state.snapshot if snapshot is None else snapshot
        self.xy_yaw_calibration_valid_pub.publish(Bool(data=current.valid))
        self.xy_yaw_calibration_status_pub.publish(String(data=current.to_json()))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = XyzMapperNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
