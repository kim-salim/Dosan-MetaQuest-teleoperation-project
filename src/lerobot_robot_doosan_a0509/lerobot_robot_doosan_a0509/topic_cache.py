"""Thread-safe latest-value cache with local and ROS timestamps."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any


class TopicUnavailableError(RuntimeError):
    pass


@dataclass(frozen=True)
class TopicSample:
    value: Any
    receive_time: float
    header_stamp: float | None
    receive_count: int

    def age(self, now: float | None = None) -> float:
        current = time.monotonic() if now is None else float(now)
        return max(0.0, current - self.receive_time)


class TopicCache:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._samples: dict[str, TopicSample] = {}
        self._counts: dict[str, int] = {}

    def update(
        self,
        key: str,
        value: Any,
        *,
        receive_time: float | None = None,
        header_stamp: float | None = None,
    ) -> TopicSample:
        timestamp = time.monotonic() if receive_time is None else float(receive_time)
        with self._lock:
            count = self._counts.get(key, 0) + 1
            self._counts[key] = count
            sample = TopicSample(value, timestamp, header_stamp, count)
            self._samples[key] = sample
            return sample

    def sample(self, key: str) -> TopicSample | None:
        with self._lock:
            return self._samples.get(key)

    def require_present(self, key: str) -> TopicSample:
        """Return a latched state sample that remains valid until superseded."""
        sample = self.sample(key)
        if sample is None:
            raise TopicUnavailableError(f"no message received for {key}")
        return sample

    def require(
        self,
        key: str,
        *,
        max_age_sec: float,
        now: float | None = None,
    ) -> TopicSample:
        sample = self.sample(key)
        if sample is None:
            raise TopicUnavailableError(f"no message received for {key}")
        age = sample.age(now)
        if age > max_age_sec:
            raise TopicUnavailableError(
                f"stale topic {key}: age={age:.3f}s max_age={max_age_sec:.3f}s"
            )
        return sample

    def receive_count(self, key: str) -> int:
        with self._lock:
            return self._counts.get(key, 0)


def header_stamp_from_message(message: Any) -> float | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return None
