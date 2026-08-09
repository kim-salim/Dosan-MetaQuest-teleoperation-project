"""Fixed-rate pose resampling helpers for bursty Quest input."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable, Optional


def _finite_tuple(
    values: Iterable[float],
    length: int,
    name: str,
) -> tuple[float, ...]:
    output = tuple(float(value) for value in values)
    if len(output) != length:
        raise ValueError(f"{name} must contain exactly {length} values")
    if any(not math.isfinite(value) for value in output):
        raise ValueError(f"{name} contains non-finite values: {output}")
    return output


def _normalized_quaternion(
    values: Iterable[float],
    name: str = "quaternion",
) -> tuple[float, float, float, float]:
    output = _finite_tuple(values, 4, name)
    norm = math.sqrt(sum(value * value for value in output))
    if norm <= 1.0e-12:
        raise ValueError(f"{name} has near-zero norm: {output}")
    return tuple(value / norm for value in output)


def quaternion_slerp(
    start: Iterable[float],
    end: Iterable[float],
    fraction: float,
) -> tuple[float, float, float, float]:
    """Interpolate or narrowly extrapolate between unit quaternions."""
    first = _normalized_quaternion(start, "start quaternion")
    second = _normalized_quaternion(end, "end quaternion")
    amount = float(fraction)
    if not math.isfinite(amount):
        raise ValueError("fraction must be finite")

    dot = sum(first[index] * second[index] for index in range(4))
    if dot < 0.0:
        second = tuple(-value for value in second)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))

    if dot > 0.9995:
        return _normalized_quaternion(
            [
                first[index] + amount * (second[index] - first[index])
                for index in range(4)
            ],
            "linearly interpolated quaternion",
        )

    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    start_weight = math.sin((1.0 - amount) * theta) / sin_theta
    end_weight = math.sin(amount * theta) / sin_theta
    return _normalized_quaternion(
        [
            start_weight * first[index] + end_weight * second[index]
            for index in range(4)
        ],
        "slerp quaternion",
    )


def fixed_rate_alpha(
    source_alpha: float,
    source_rate_hz: float,
    target_dt_sec: float,
) -> float:
    """Preserve a first-order filter time constant at a new fixed rate."""
    alpha = float(source_alpha)
    rate = float(source_rate_hz)
    dt = float(target_dt_sec)
    if not 0.0 < alpha <= 1.0:
        raise ValueError("source_alpha must be in the range (0, 1]")
    if not math.isfinite(rate) or rate <= 0.0:
        raise ValueError("source_rate_hz must be finite and > 0")
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("target_dt_sec must be finite and > 0")
    if alpha >= 1.0:
        return 1.0
    source_dt = 1.0 / rate
    time_constant = -source_dt / math.log1p(-alpha)
    return 1.0 - math.exp(-dt / time_constant)


@dataclass(frozen=True)
class PoseSample:
    sequence_id: int
    receive_time_sec: float
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class ResampledPose:
    query_time_sec: float
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    mode: str
    left_sequence_id: int
    right_sequence_id: int
    latest_sample_time_sec: float


class PoseResampler:
    """Bounded receive-time jitter buffer with LERP, SLERP, and safe prediction."""

    def __init__(
        self,
        *,
        max_samples: int = 128,
        buffer_duration_sec: float = 1.0,
        max_interpolation_gap_sec: float = 0.08,
        full_prediction_sec: float = 0.02,
        prediction_decay_sec: float = 0.08,
        velocity_estimation_min_sec: float = 0.02,
        max_prediction_linear_speed_m_s: float = 0.5,
        max_prediction_angular_speed_deg_s: float = 90.0,
    ) -> None:
        if int(max_samples) < 2:
            raise ValueError("max_samples must be >= 2")
        if not math.isfinite(buffer_duration_sec) or buffer_duration_sec <= 0.0:
            raise ValueError("buffer_duration_sec must be finite and > 0")
        if (
            not math.isfinite(max_interpolation_gap_sec)
            or max_interpolation_gap_sec <= 0.0
        ):
            raise ValueError("max_interpolation_gap_sec must be finite and > 0")
        if not math.isfinite(full_prediction_sec) or full_prediction_sec < 0.0:
            raise ValueError("full_prediction_sec must be finite and >= 0")
        if (
            not math.isfinite(prediction_decay_sec)
            or prediction_decay_sec < full_prediction_sec
        ):
            raise ValueError(
                "prediction_decay_sec must be finite and >= full_prediction_sec"
            )
        if (
            not math.isfinite(velocity_estimation_min_sec)
            or velocity_estimation_min_sec <= 0.0
        ):
            raise ValueError("velocity_estimation_min_sec must be finite and > 0")
        if (
            not math.isfinite(max_prediction_linear_speed_m_s)
            or max_prediction_linear_speed_m_s < 0.0
        ):
            raise ValueError(
                "max_prediction_linear_speed_m_s must be finite and >= 0"
            )
        if (
            not math.isfinite(max_prediction_angular_speed_deg_s)
            or max_prediction_angular_speed_deg_s < 0.0
        ):
            raise ValueError(
                "max_prediction_angular_speed_deg_s must be finite and >= 0"
            )

        self.max_samples = int(max_samples)
        self.buffer_duration_sec = float(buffer_duration_sec)
        self.max_interpolation_gap_sec = float(max_interpolation_gap_sec)
        self.full_prediction_sec = float(full_prediction_sec)
        self.prediction_decay_sec = float(prediction_decay_sec)
        self.velocity_estimation_min_sec = float(velocity_estimation_min_sec)
        self.max_prediction_linear_speed_m_s = float(
            max_prediction_linear_speed_m_s
        )
        self.max_prediction_angular_speed_rad_s = math.radians(
            float(max_prediction_angular_speed_deg_s)
        )
        self._samples: deque[PoseSample] = deque(maxlen=self.max_samples)
        self._next_sequence_id = 0

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    @property
    def latest_sample_time_sec(self) -> Optional[float]:
        if not self._samples:
            return None
        return self._samples[-1].receive_time_sec

    def clear(self) -> None:
        self._samples.clear()

    def push(
        self,
        *,
        receive_time_sec: float,
        position_m: Iterable[float],
        quaternion_xyzw: Iterable[float],
    ) -> bool:
        timestamp = float(receive_time_sec)
        if not math.isfinite(timestamp):
            raise ValueError("receive_time_sec must be finite")
        if self._samples and timestamp <= self._samples[-1].receive_time_sec:
            return False

        position = _finite_tuple(position_m, 3, "position_m")
        quaternion = _normalized_quaternion(quaternion_xyzw)
        self._next_sequence_id += 1
        self._samples.append(
            PoseSample(
                sequence_id=self._next_sequence_id,
                receive_time_sec=timestamp,
                position_m=position,
                quaternion_xyzw=quaternion,
            )
        )
        oldest_allowed = timestamp - self.buffer_duration_sec
        while (
            len(self._samples) > 2
            and self._samples[1].receive_time_sec < oldest_allowed
        ):
            self._samples.popleft()
        return True

    def sample(self, query_time_sec: float) -> Optional[ResampledPose]:
        query_time = float(query_time_sec)
        if not math.isfinite(query_time):
            raise ValueError("query_time_sec must be finite")
        if not self._samples or query_time < self._samples[0].receive_time_sec:
            return None

        samples = tuple(self._samples)
        latest = samples[-1]
        if query_time >= latest.receive_time_sec:
            return self._predict_from_index(
                samples,
                len(samples) - 1,
                query_time,
            )

        for right_index in range(1, len(samples)):
            right = samples[right_index]
            if query_time > right.receive_time_sec:
                continue
            left = samples[right_index - 1]
            gap_sec = right.receive_time_sec - left.receive_time_sec
            if gap_sec > self.max_interpolation_gap_sec:
                return self._predict_from_index(
                    samples,
                    right_index - 1,
                    query_time,
                )
            fraction = (
                0.0
                if gap_sec <= 1.0e-12
                else (query_time - left.receive_time_sec) / gap_sec
            )
            position = tuple(
                left.position_m[index]
                + fraction * (right.position_m[index] - left.position_m[index])
                for index in range(3)
            )
            quaternion = quaternion_slerp(
                left.quaternion_xyzw,
                right.quaternion_xyzw,
                fraction,
            )
            return ResampledPose(
                query_time_sec=query_time,
                position_m=position,
                quaternion_xyzw=quaternion,
                mode="interpolate",
                left_sequence_id=left.sequence_id,
                right_sequence_id=right.sequence_id,
                latest_sample_time_sec=latest.receive_time_sec,
            )
        return None

    def _prediction_motion_time(self, horizon_sec: float) -> tuple[float, str]:
        horizon = max(0.0, float(horizon_sec))
        if horizon <= self.full_prediction_sec:
            return horizon, "extrapolate"
        if self.prediction_decay_sec <= self.full_prediction_sec:
            return self.full_prediction_sec, "hold"
        decay_duration = self.prediction_decay_sec - self.full_prediction_sec
        decay_elapsed = min(horizon - self.full_prediction_sec, decay_duration)
        effective_motion_time = (
            self.full_prediction_sec
            + decay_elapsed
            - 0.5 * decay_elapsed * decay_elapsed / decay_duration
        )
        mode = "decay" if horizon < self.prediction_decay_sec else "hold"
        return effective_motion_time, mode

    def _velocity_reference_index(
        self,
        samples: tuple[PoseSample, ...],
        latest_index: int,
    ) -> Optional[int]:
        latest = samples[latest_index]
        for index in range(latest_index - 1, -1, -1):
            if (
                latest.receive_time_sec - samples[index].receive_time_sec
                >= self.velocity_estimation_min_sec
            ):
                return index
        if latest_index > 0:
            return latest_index - 1
        return None

    def _predict_from_index(
        self,
        samples: tuple[PoseSample, ...],
        latest_index: int,
        query_time_sec: float,
    ) -> ResampledPose:
        latest = samples[latest_index]
        horizon_sec = max(0.0, query_time_sec - latest.receive_time_sec)
        motion_time_sec, mode = self._prediction_motion_time(horizon_sec)
        reference_index = self._velocity_reference_index(samples, latest_index)
        if reference_index is None or motion_time_sec <= 0.0:
            return ResampledPose(
                query_time_sec=query_time_sec,
                position_m=latest.position_m,
                quaternion_xyzw=latest.quaternion_xyzw,
                mode="hold" if horizon_sec > 0.0 else "exact",
                left_sequence_id=latest.sequence_id,
                right_sequence_id=latest.sequence_id,
                latest_sample_time_sec=samples[-1].receive_time_sec,
            )

        reference = samples[reference_index]
        velocity_dt = latest.receive_time_sec - reference.receive_time_sec
        if velocity_dt <= 1.0e-12:
            velocity = (0.0, 0.0, 0.0)
        else:
            velocity = tuple(
                (latest.position_m[index] - reference.position_m[index])
                / velocity_dt
                for index in range(3)
            )
        speed = math.sqrt(sum(value * value for value in velocity))
        if speed > self.max_prediction_linear_speed_m_s and speed > 1.0e-12:
            scale = self.max_prediction_linear_speed_m_s / speed
            velocity = tuple(value * scale for value in velocity)
        position = tuple(
            latest.position_m[index] + velocity[index] * motion_time_sec
            for index in range(3)
        )

        dot = abs(
            sum(
                reference.quaternion_xyzw[index]
                * latest.quaternion_xyzw[index]
                for index in range(4)
            )
        )
        dot = min(1.0, max(-1.0, dot))
        angle_rad = 2.0 * math.acos(dot)
        extrapolation_fraction = 0.0
        if angle_rad > 1.0e-12 and velocity_dt > 1.0e-12:
            angular_speed = min(
                angle_rad / velocity_dt,
                self.max_prediction_angular_speed_rad_s,
            )
            extrapolation_fraction = angular_speed * motion_time_sec / angle_rad
        quaternion = quaternion_slerp(
            reference.quaternion_xyzw,
            latest.quaternion_xyzw,
            1.0 + extrapolation_fraction,
        )
        return ResampledPose(
            query_time_sec=query_time_sec,
            position_m=position,
            quaternion_xyzw=quaternion,
            mode=mode,
            left_sequence_id=reference.sequence_id,
            right_sequence_id=latest.sequence_id,
            latest_sample_time_sec=samples[-1].receive_time_sec,
        )
