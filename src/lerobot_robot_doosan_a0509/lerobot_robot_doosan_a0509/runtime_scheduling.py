"""Small Linux scheduling helpers for the A0509 rollout process."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
import os
import threading


def parse_cpu_set(value: str) -> set[int]:
    """Parse Linux CPU-list syntax such as ``6`` or ``7-8,10``."""

    cpus: set[int] = set()
    for part in value.split(","):
        token = part.strip()
        if not token:
            raise ValueError(f"invalid empty CPU-list component in {value!r}")
        if "-" in token:
            start_text, end_text = token.split("-", 1)
            start = int(start_text)
            end = int(end_text)
            if start < 0 or end < start:
                raise ValueError(f"invalid CPU range {token!r}")
            cpus.update(range(start, end + 1))
        else:
            cpu = int(token)
            if cpu < 0:
                raise ValueError(f"CPU index must be non-negative, got {cpu}")
            cpus.add(cpu)
    if not cpus:
        raise ValueError("CPU set must not be empty")
    return cpus


def pin_current_thread(cpu_list: str) -> tuple[int, ...]:
    """Pin only the calling Linux thread and return its applied CPU tuple."""

    cpus = parse_cpu_set(cpu_list)
    os.sched_setaffinity(threading.get_native_id(), cpus)
    return tuple(sorted(cpus))


def pin_current_thread_from_env(name: str) -> tuple[int, ...] | None:
    """Pin the calling thread when *name* contains a Linux CPU list."""

    value = os.environ.get(name)
    if value is None or not value.strip():
        return None
    return pin_current_thread(value)


@contextmanager
def preserve_current_thread_affinity() -> Iterator[tuple[int, ...]]:
    """Restore the calling Linux thread's affinity after scoped work.

    Some policy adapters intentionally pin the thread that performs CUDA
    inference. Setup-time warmup runs those adapters synchronously on the
    rollout/control thread, so without this scope the warmup affinity leaks
    into the later 30 Hz control loop and into any process spawned from that
    thread.
    """

    thread_id = threading.get_native_id()
    original = set(os.sched_getaffinity(thread_id))
    try:
        yield tuple(sorted(original))
    finally:
        current = set(os.sched_getaffinity(thread_id))
        if current != original:
            os.sched_setaffinity(thread_id, original)


@contextmanager
def temporary_current_thread_affinity(
    cpus: tuple[int, ...] | set[int],
) -> Iterator[tuple[int, ...]]:
    """Temporarily pin the caller, then restore its exact prior affinity.

    Linux child processes inherit the affinity of the *spawning thread*.
    This is used only while lazily spawning a dedicated worker so its strict
    initializer can verify and retain the requested CPU without weakening the
    worker-side fail-closed check.
    """

    requested = {int(cpu) for cpu in cpus}
    if not requested or any(cpu < 0 for cpu in requested):
        raise ValueError("temporary CPU set must contain non-negative CPUs")
    thread_id = threading.get_native_id()
    original = set(os.sched_getaffinity(thread_id))
    try:
        os.sched_setaffinity(thread_id, requested)
        applied = set(os.sched_getaffinity(thread_id))
        if applied != requested:
            raise RuntimeError(
                "failed to apply temporary thread affinity: "
                f"requested={sorted(requested)} applied={sorted(applied)}"
            )
        yield tuple(sorted(original))
    finally:
        os.sched_setaffinity(thread_id, original)
