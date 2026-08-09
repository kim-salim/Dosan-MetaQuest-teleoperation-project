from types import SimpleNamespace

import pytest

from lerobot_robot_doosan_a0509.deadline_record_loop import (
    AbsoluteDeadlineSleeper,
    install_absolute_deadline_record_loop,
)


class FakeClock:
    def __init__(self, oversleep_s=0.0):
        self.now = 0.0
        self.oversleep_s = oversleep_s

    def __call__(self):
        return self.now

    def work(self, duration_s):
        self.now += duration_s

    def sleep(self, duration_s, *args, **kwargs):
        del args, kwargs
        self.now += duration_s + self.oversleep_s


def test_absolute_deadline_does_not_accumulate_sleep_overshoot():
    clock = FakeClock(oversleep_s=0.001)
    period_s = 1.0 / 30.0
    work_s = 0.010
    sleeper = AbsoluteDeadlineSleeper(period_s, clock.sleep, clock=clock)

    for _ in range(300):
        clock.work(work_s)
        sleeper(period_s - work_s)

    assert clock.now == pytest.approx(10.001, abs=1e-9)
    assert sleeper.metrics.ticks == 300
    assert sleeper.metrics.missed_deadlines == 0
    # The next deadline absorbs the 1 ms oversleep without becoming late.
    assert sleeper.metrics.late_ticks == 0


def test_absolute_deadline_counts_missed_period_without_burst_sleep():
    clock = FakeClock()
    period_s = 0.1
    sleeper = AbsoluteDeadlineSleeper(period_s, clock.sleep, clock=clock)
    clock.work(0.02)
    sleeper(0.08)
    clock.work(0.25)
    sleeper(0.0)
    assert sleeper.metrics.missed_deadlines == 1
    assert sleeper.metrics.max_lateness_s == pytest.approx(0.15)


def test_installer_wraps_and_restores_precise_sleep_per_record_call():
    calls = []

    def precise_sleep(duration_s):
        calls.append(duration_s)

    module = SimpleNamespace(precise_sleep=precise_sleep)

    def record_loop(*, fps):
        module.precise_sleep(1.0 / fps)

    module.record_loop = record_loop
    previous = install_absolute_deadline_record_loop(module)
    assert previous is record_loop
    module.record_loop(fps=30)
    assert module.precise_sleep is precise_sleep
    assert len(calls) == 1
