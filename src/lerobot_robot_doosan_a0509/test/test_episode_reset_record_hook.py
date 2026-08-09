from types import SimpleNamespace

from lerobot_robot_doosan_a0509.episode_reset_orchestrator import (
    EpisodeResetConfig,
    EpisodeResetRecordingHook,
)


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


def make_hook(calls):
    def record_loop(*, dataset=None):
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
    hook.install()
    return module, hook, record_loop, original_keyboard


def test_first_episode_is_prepared_armed_recorded_and_made_safe():
    calls = []
    module, hook, _, _ = make_hook(calls)

    assert module.record_loop(dataset=object()) == "recorded"
    assert calls == ["prepare", "live_on", "record", "safe"]
    hook.close()


def test_stock_reset_loop_is_replaced_and_not_repeated_before_next_episode():
    calls = []
    module, hook, _, _ = make_hook(calls)

    assert module.record_loop(dataset=None) is None
    assert module.record_loop(dataset=object()) == "recorded"
    assert calls == ["prepare", "live_on", "record", "safe"]
    hook.close()


def test_record_failure_still_forces_safe_and_close_restores_module_functions():
    calls = []

    def failing_record_loop(*, dataset=None):
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
    hook.install()

    try:
        module.record_loop(dataset=object())
    except RuntimeError as exc:
        assert str(exc) == "camera failure"
    else:
        raise AssertionError("record failure was not propagated")

    assert calls == ["prepare", "live_on", "record", "safe"]
    hook.close()
    assert module.record_loop is failing_record_loop
    assert module.init_keyboard_listener is original_keyboard
