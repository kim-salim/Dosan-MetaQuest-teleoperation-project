"""ROS-independent MetaQuest XY calibration state contract."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import json
from typing import Optional


class CalibrationPhase(str, Enum):
    UNCALIBRATED = "UNCALIBRATED"
    CALIBRATING = "CALIBRATING"
    VALID = "VALID"
    INVALID = "INVALID"


@dataclass(frozen=True)
class CalibrationSnapshot:
    state: str
    valid: bool
    reason: str
    xy_yaw_correction_deg: float
    mapping_fingerprint: str
    sequence: int
    observed_angle_deg: Optional[float] = None
    distance_m: Optional[float] = None
    position_delta_before_yaw_mm: Optional[list[float]] = None
    position_delta_after_yaw_mm: Optional[list[float]] = None

    def to_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True)


class CalibrationState:
    """Own the externally visible calibration phase and quality result."""

    def __init__(
        self,
        *,
        initial_correction_deg: float,
        mapping_fingerprint: str,
        required: bool,
    ) -> None:
        self.initial_correction_deg = float(initial_correction_deg)
        self.mapping_fingerprint = str(mapping_fingerprint)
        self.required = bool(required)
        self.sequence = 0
        if self.required:
            self.snapshot = self._make(
                CalibrationPhase.UNCALIBRATED,
                valid=False,
                reason="MetaQuest XY +X calibration is required for this tracking session.",
                correction_deg=self.initial_correction_deg,
            )
        else:
            self.snapshot = self._make(
                CalibrationPhase.VALID,
                valid=True,
                reason="Session calibration requirement is disabled by configuration.",
                correction_deg=self.initial_correction_deg,
            )

    def start(self, correction_deg: float) -> CalibrationSnapshot:
        self.snapshot = self._make(
            CalibrationPhase.CALIBRATING,
            valid=False,
            reason="Move the Quest controller toward robot +X until measurement completes.",
            correction_deg=correction_deg,
        )
        return self.snapshot

    def complete(
        self,
        *,
        correction_deg: float,
        observed_angle_deg: float,
        distance_m: float,
        before_yaw_mm: list[float],
        after_yaw_mm: list[float],
    ) -> CalibrationSnapshot:
        self.snapshot = self._make(
            CalibrationPhase.VALID,
            valid=True,
            reason="MetaQuest XY +X calibration completed.",
            correction_deg=correction_deg,
            observed_angle_deg=observed_angle_deg,
            distance_m=distance_m,
            before_yaw_mm=before_yaw_mm,
            after_yaw_mm=after_yaw_mm,
        )
        return self.snapshot

    def invalidate(
        self,
        reason: str,
        *,
        correction_deg: Optional[float] = None,
        reset_correction: bool = False,
    ) -> CalibrationSnapshot:
        correction = (
            self.initial_correction_deg
            if reset_correction
            else (
                self.snapshot.xy_yaw_correction_deg
                if correction_deg is None
                else float(correction_deg)
            )
        )
        self.snapshot = self._make(
            CalibrationPhase.INVALID,
            valid=False,
            reason=str(reason),
            correction_deg=correction,
        )
        return self.snapshot

    def _make(
        self,
        phase: CalibrationPhase,
        *,
        valid: bool,
        reason: str,
        correction_deg: float,
        observed_angle_deg: Optional[float] = None,
        distance_m: Optional[float] = None,
        before_yaw_mm: Optional[list[float]] = None,
        after_yaw_mm: Optional[list[float]] = None,
    ) -> CalibrationSnapshot:
        self.sequence += 1
        return CalibrationSnapshot(
            state=phase.value,
            valid=bool(valid),
            reason=str(reason),
            xy_yaw_correction_deg=float(correction_deg),
            mapping_fingerprint=self.mapping_fingerprint,
            sequence=self.sequence,
            observed_angle_deg=observed_angle_deg,
            distance_m=distance_m,
            position_delta_before_yaw_mm=(
                None if before_yaw_mm is None else [float(value) for value in before_yaw_mm]
            ),
            position_delta_after_yaw_mm=(
                None if after_yaw_mm is None else [float(value) for value in after_yaw_mm]
            ),
        )
