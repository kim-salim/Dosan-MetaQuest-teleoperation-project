from __future__ import annotations

import pytest

from lerobot_robot_doosan_a0509.ros_runtime import _environment_positive_int
from lerobot_robot_doosan_a0509.runtime_scheduling import (
    parse_cpu_set,
    pin_current_thread,
    preserve_current_thread_affinity,
    temporary_current_thread_affinity,
)


def test_parse_cpu_set_supports_ranges_and_lists():
    assert parse_cpu_set("6") == {6}
    assert parse_cpu_set("7-8") == {7, 8}
    assert parse_cpu_set("6,9-13") == {6, 9, 10, 11, 12, 13}


@pytest.mark.parametrize("value", ("", "8-7", "-1", "6,,7"))
def test_parse_cpu_set_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        parse_cpu_set(value)


def test_pin_current_thread_uses_native_thread_id(monkeypatch):
    calls = []
    monkeypatch.setattr(
        "lerobot_robot_doosan_a0509.runtime_scheduling.threading.get_native_id",
        lambda: 1234,
    )
    monkeypatch.setattr(
        "lerobot_robot_doosan_a0509.runtime_scheduling.os.sched_setaffinity",
        lambda thread_id, cpus: calls.append((thread_id, cpus)),
    )
    assert pin_current_thread("9-13") == (9, 10, 11, 12, 13)
    assert calls == [(1234, {9, 10, 11, 12, 13})]


def test_preserve_current_thread_affinity_restores_after_warmup_pin(monkeypatch):
    current = {7, 8}
    calls = []

    monkeypatch.setattr(
        "lerobot_robot_doosan_a0509.runtime_scheduling.threading.get_native_id",
        lambda: 1234,
    )
    monkeypatch.setattr(
        "lerobot_robot_doosan_a0509.runtime_scheduling.os.sched_getaffinity",
        lambda _thread_id: set(current),
    )

    def set_affinity(thread_id, cpus):
        calls.append((thread_id, set(cpus)))
        current.clear()
        current.update(cpus)

    monkeypatch.setattr(
        "lerobot_robot_doosan_a0509.runtime_scheduling.os.sched_setaffinity",
        set_affinity,
    )
    with preserve_current_thread_affinity():
        set_affinity(1234, {9, 10, 11, 12})
        assert current == {9, 10, 11, 12}
    assert current == {7, 8}
    assert calls[-1] == (1234, {7, 8})


def test_temporary_current_thread_affinity_restores_spawn_caller(monkeypatch):
    current = {9, 10, 11, 12}
    monkeypatch.setattr(
        "lerobot_robot_doosan_a0509.runtime_scheduling.threading.get_native_id",
        lambda: 1234,
    )
    monkeypatch.setattr(
        "lerobot_robot_doosan_a0509.runtime_scheduling.os.sched_getaffinity",
        lambda _thread_id: set(current),
    )

    def set_affinity(_thread_id, cpus):
        current.clear()
        current.update(cpus)

    monkeypatch.setattr(
        "lerobot_robot_doosan_a0509.runtime_scheduling.os.sched_setaffinity",
        set_affinity,
    )
    with temporary_current_thread_affinity((13,)):
        assert current == {13}
    assert current == {9, 10, 11, 12}


def test_ros_executor_thread_count_environment(monkeypatch):
    monkeypatch.delenv("TEST_EXECUTOR_THREADS", raising=False)
    assert _environment_positive_int("TEST_EXECUTOR_THREADS", 4) == 4
    monkeypatch.setenv("TEST_EXECUTOR_THREADS", "1")
    assert _environment_positive_int("TEST_EXECUTOR_THREADS", 4) == 1
    monkeypatch.setenv("TEST_EXECUTOR_THREADS", "0")
    with pytest.raises(ValueError, match="positive"):
        _environment_positive_int("TEST_EXECUTOR_THREADS", 4)
