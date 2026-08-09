"""ROS-independent cancellation primitive for robot preparation motion."""

from __future__ import annotations

import threading


class PreparationCancelled(RuntimeError):
    """Raised when an operator stop request cancels robot preparation."""


class PreparationStopToken:
    """Allow a concurrent stop callback to interrupt preparation waits."""

    def __init__(self) -> None:
        self._event = threading.Event()

    @property
    def requested(self) -> bool:
        return self._event.is_set()

    def begin(self) -> None:
        self._event.clear()

    def request(self) -> None:
        self._event.set()

    def check(self) -> None:
        if self.requested:
            raise PreparationCancelled("robot preparation cancelled by stop request")

    def wait(self, timeout_sec: float) -> None:
        if self._event.wait(timeout=max(0.0, float(timeout_sec))):
            self.check()
        self.check()
