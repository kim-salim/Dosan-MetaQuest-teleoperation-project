from types import SimpleNamespace

import pytest

from lerobot_robot_doosan_a0509.episode_reset_orchestrator import (
    EpisodeResetConfig,
    EpisodeResetRecordingHook,
    RecordingControlGate,
)


def make_events(**overrides):
    events = {
        "exit_early": False,
        "rerecord_episode": False,
        "stop_recording": False,
    }
    events.update(overrides)
    return events


@pytest.mark.parametrize(
    ("event_overrides", "expected"),
    [
        ({"exit_early": True}, "save"),
        ({"exit_early": True, "rerecord_episode": True}, "rerecord"),
        ({"exit_early": True, "stop_recording": True}, "stop"),
    ],
)
def test_episode_decision_accepts_key_event_arriving_at_timer_boundary(
    event_overrides, expected
):
    gate = RecordingControlGate()
    gate._events = make_events(**event_overrides)
    gate._listener = object()

    assert gate.wait_for_episode_decision() == expected


class FakeOrchestrator:
    def __init__(self, calls):
        self.calls = calls

    def prepare_next_episode(self, _gate):
        self.calls.append("prepare")

    def enable_live_for_recording(self):
        self.calls.append("live_on")

    def force_safe(self):
        self.calls.append("safe")

    def close(self):
        self.calls.append("close")


class FakeGate:
    def __init__(self, calls, decision):
        self.calls = calls
        self.decision = decision

    def init_keyboard_listener(self):
        return None, make_events()

    def wait_for_episode_decision(self):
        self.calls.append(f"review_{self.decision}")
        return self.decision


def make_hook(calls, decision="save"):
    def record_loop(*, events=None, dataset=None):
        calls.append("record" if dataset is not None else "stock_reset")
        return "recorded"

    original_keyboard = lambda: (None, {})
    module = SimpleNamespace(
        record_loop=record_loop,
        init_keyboard_listener=original_keyboard,
    )
    hook = EpisodeResetRecordingHook(
        module,
        config=EpisodeResetConfig(reset_before_first_episode=True),
    )
    hook._orchestrator = FakeOrchestrator(calls)
    hook.gate = FakeGate(calls, decision)
    hook.install()
    return module, hook, record_loop, original_keyboard


def test_natural_completion_waits_for_save_decision_after_safe():
    calls = []
    module, hook, _, _ = make_hook(calls)
    events = make_events()

    assert module.record_loop(events=events, dataset=object()) == "recorded"
    assert calls == ["prepare", "live_on", "record", "safe", "review_save"]
    assert events == make_events()
    hook.close()


def test_natural_completion_can_select_discard_and_rerecord():
    calls = []
    module, hook, _, _ = make_hook(calls, decision="rerecord")
    events = make_events()

    assert module.record_loop(events=events, dataset=object()) == "recorded"
    assert calls == [
        "prepare",
        "live_on",
        "record",
        "safe",
        "review_rerecord",
    ]
    assert events["exit_early"] is False
    assert events["rerecord_episode"] is True
    assert events["stop_recording"] is False
    hook.close()


def test_recording_n_keeps_early_save_behavior_without_second_prompt():
    calls = []
    module, hook, _, _ = make_hook(calls)
    events = make_events(exit_early=True)

    assert module.record_loop(events=events, dataset=object()) == "recorded"
    assert calls == ["prepare", "live_on", "record", "safe"]
    assert events == make_events()
    hook.close()


def test_recording_r_keeps_discard_and_rerecord_behavior_without_second_prompt():
    calls = []
    module, hook, _, _ = make_hook(calls)
    events = make_events(exit_early=True, rerecord_episode=True)

    assert module.record_loop(events=events, dataset=object()) == "recorded"
    assert calls == ["prepare", "live_on", "record", "safe"]
    assert events["exit_early"] is False
    assert events["rerecord_episode"] is True
    assert events["stop_recording"] is False
    hook.close()


def test_stock_reset_loop_is_noop_so_commit_precedes_next_prepare():
    calls = []
    module, hook, _, _ = make_hook(calls)

    assert module.record_loop(events=make_events(), dataset=object()) == "recorded"
    assert calls == ["prepare", "live_on", "record", "safe", "review_save"]

    assert module.record_loop(events=make_events(), dataset=None) is None
    calls.append("dataset_commit")
    assert module.record_loop(
        events=make_events(exit_early=True), dataset=object()
    ) == "recorded"
    assert calls == [
        "prepare",
        "live_on",
        "record",
        "safe",
        "review_save",
        "dataset_commit",
        "prepare",
        "live_on",
        "record",
        "safe",
    ]
    hook.close()


def test_calibration_loss_discards_partial_and_requests_rerecord():
    calls = []

    def calibration_loss(*, events=None, dataset=None):
        calls.append("record")
        raise RuntimeError(
            "MetaQuest action rejected because XY calibration is invalid"
        )

    original_keyboard = lambda: (None, {})
    module = SimpleNamespace(
        record_loop=calibration_loss,
        init_keyboard_listener=original_keyboard,
    )
    hook = EpisodeResetRecordingHook(
        module,
        config=EpisodeResetConfig(reset_before_first_episode=True),
    )
    hook._orchestrator = FakeOrchestrator(calls)
    hook.gate = FakeGate(calls, "save")
    hook.install()
    events = make_events()

    assert module.record_loop(events=events, dataset=object()) is None
    assert calls == ["prepare", "live_on", "record", "safe"]
    assert events["exit_early"] is False
    assert events["rerecord_episode"] is True
    assert events["stop_recording"] is False
    hook.close()
    assert module.record_loop is calibration_loss
    assert module.init_keyboard_listener is original_keyboard


def test_record_failure_still_forces_safe_and_close_restores_module_functions():
    calls = []

    def failing_record_loop(*, events=None, dataset=None):
        calls.append("record")
        raise RuntimeError("camera failure")

    original_keyboard = lambda: (None, {})
    module = SimpleNamespace(
        record_loop=failing_record_loop,
        init_keyboard_listener=original_keyboard,
    )
    hook = EpisodeResetRecordingHook(
        module,
        config=EpisodeResetConfig(reset_before_first_episode=True),
    )
    hook._orchestrator = FakeOrchestrator(calls)
    hook.gate = FakeGate(calls, "save")
    hook.install()

    try:
        module.record_loop(events=make_events(), dataset=object())
    except RuntimeError as exc:
        assert str(exc) == "camera failure"
    else:
        raise AssertionError("record failure was not propagated")

    assert calls == ["prepare", "live_on", "record", "safe"]
    hook.close()
    assert module.record_loop is failing_record_loop
    assert module.init_keyboard_listener is original_keyboard
