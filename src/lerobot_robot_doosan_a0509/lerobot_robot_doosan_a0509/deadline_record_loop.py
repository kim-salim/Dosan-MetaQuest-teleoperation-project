"""Absolute-deadline timing wrapper for LeRobot's recording loop."""

from __future__ import annotations

import functools
import logging
import time
from dataclasses import dataclass
from types import ModuleType
from typing import Any, Callable


logger = logging.getLogger(__name__)


@dataclass
class DeadlineMetrics:
    ticks: int = 0
    late_ticks: int = 0
    missed_deadlines: int = 0
    max_lateness_s: float = 0.0
    total_sleep_s: float = 0.0

    def as_log_dict(self) -> dict[str, int | float]:
        return {
            "ticks": self.ticks,
            "late_ticks": self.late_ticks,
            "missed_deadlines": self.missed_deadlines,
            "max_lateness_ms": round(self.max_lateness_s * 1e3, 3),
            "total_sleep_s": round(self.total_sleep_s, 6),
        }


class AbsoluteDeadlineSleeper:
    """Turn LeRobot's relative end-of-tick sleeps into an absolute timeline.

    The stock record loop passes ``period - work_time`` at the end of each
    iteration. The first call therefore reveals the first tick deadline. Later
    calls advance that deadline by exactly one period, so scheduler oversleep
    does not accumulate over the episode.
    """

    def __init__(
        self,
        period_s: float,
        sleep_fn: Callable[..., None],
        *,
        clock: Callable[[], float] = time.perf_counter,
    ) -> None:
        if period_s <= 0.0:
            raise ValueError("period_s must be positive")
        self.period_s = float(period_s)
        self._sleep_fn = sleep_fn
        self._clock = clock
        self._deadline: float | None = None
        self.metrics = DeadlineMetrics()

    def __call__(self, requested_s: float, *args: Any, **kwargs: Any) -> None:
        now = self._clock()
        requested_s = max(float(requested_s), 0.0)

        if self._deadline is None:
            # requested_s is period - work_time, so now + requested_s is the
            # first absolute end-of-tick deadline.
            self._deadline = now + requested_s
        else:
            self._deadline += self.period_s

        remaining_s = self._deadline - now
        if remaining_s < 0.0:
            lateness_s = -remaining_s
            self.metrics.late_ticks += 1
            self.metrics.max_lateness_s = max(
                self.metrics.max_lateness_s, lateness_s
            )
            # Skip complete missed periods, but permit at most one immediate
            # catch-up tick. This avoids a burst of duplicated latest frames.
            missed = int(lateness_s // self.period_s)
            if missed > 0:
                self._deadline += missed * self.period_s
                self.metrics.missed_deadlines += missed
                remaining_s = self._deadline - now

        if remaining_s > 0.0:
            before_sleep = self._clock()
            self._sleep_fn(remaining_s, *args, **kwargs)
            self.metrics.total_sleep_s += max(self._clock() - before_sleep, 0.0)

        self.metrics.ticks += 1


def install_absolute_deadline_record_loop(
    record_module: ModuleType | Any | None = None,
) -> Callable[..., Any]:
    """Install a process-local wrapper around LeRobot's stock ``record_loop``."""

    if record_module is None:
        from lerobot.scripts import lerobot_record as record_module

    current = record_module.record_loop
    if getattr(current, "_a0509_absolute_deadline", False):
        return current

    original_sleep = record_module.precise_sleep

    @functools.wraps(current)
    def deadline_record_loop(*args: Any, **kwargs: Any) -> Any:
        if "fps" in kwargs:
            fps = float(kwargs["fps"])
        elif len(args) >= 3:
            fps = float(args[2])
        else:
            raise TypeError("record_loop fps argument is required")
        if fps <= 0.0:
            raise ValueError("record_loop fps must be positive")

        sleeper = AbsoluteDeadlineSleeper(1.0 / fps, original_sleep)
        previous_sleep = record_module.precise_sleep
        record_module.precise_sleep = sleeper
        try:
            return current(*args, **kwargs)
        finally:
            record_module.precise_sleep = previous_sleep
            logger.info(
                "Absolute-deadline recorder metrics: %s", sleeper.metrics.as_log_dict()
            )

    deadline_record_loop._a0509_absolute_deadline = True
    deadline_record_loop._a0509_original_record_loop = current
    record_module.record_loop = deadline_record_loop
    logger.info("Installed A0509 absolute-deadline record loop for this process")
    return current
