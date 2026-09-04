import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import lerobot_robot_doosan_a0509.recording_gripper_initializer as initializer_module
from lerobot_robot_doosan_a0509.recording_gripper_initializer import (
    GripperInitializationState,
    RecordingGripperInitializer,
)


def _load_preflight_module():
    path = (
        Path(__file__).resolve().parents[3]
        / "scripts"
        / "preflight_lerobot_shadow_record.py"
    )
    spec = importlib.util.spec_from_file_location("a0509_gripper_preflight", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _FakeRobot:
    def __init__(self, config):
        self.config = config
        self.cameras = {}
        self.live_publish_count = 0
        self.debug_publish_count = 0
        self.is_connected = False

    def connect(self):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False

    def get_observation(self):
        return {"joint_1_pos": 0.0}


class _FakeTeleop:
    def __init__(self, config):
        self.config = config
        self.action_features = {"target_x_mm": float, "gripper_target": float}
        self.is_connected = False

    def connect(self):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False


def test_recording_preflight_initializes_requested_gripper_state(monkeypatch, capsys):
    module = _load_preflight_module()
    calls = []
    robot_config = SimpleNamespace(
        mode="shadow_record",
        cameras={},
        require_camera=False,
    )
    monkeypatch.setattr(module, "DoosanA0509RosConfig", lambda **_kwargs: robot_config)
    monkeypatch.setattr(
        module,
        "MetaQuestA0509Config",
        lambda **kwargs: SimpleNamespace(**kwargs),
    )
    monkeypatch.setattr(module, "DoosanA0509Ros", _FakeRobot)
    monkeypatch.setattr(module, "MetaQuestA0509", _FakeTeleop)
    monkeypatch.setattr(
        module,
        "ensure_recording_gripper_state",
        lambda command, timeout_sec: calls.append((command, timeout_sec))
        or "initialized_open",
    )

    module.main(
        allow_unprepared_teleop=True,
        initial_gripper_state="open",
        gripper_initialize_timeout_sec=7.5,
    )
    summary = json.loads(capsys.readouterr().out)

    assert calls == [("open", 7.5)]
    assert summary["gripper_initialization"] == "initialized_open"


def test_existing_verified_open_state_is_ready():
    state = GripperInitializationState(
        commanded_state=0.0,
        driver_busy=False,
        last_command_ok=True,
    )

    assert state.already_ready(0.0)
    assert not state.already_ready(1.0)


def test_stale_completion_does_not_satisfy_a_new_open_request():
    state = GripperInitializationState(
        accepted_command="open",
        accepted_time=4.0,
        completed_command="open",
        completed_time=4.5,
        commanded_state=0.0,
        driver_busy=False,
        last_command_ok=True,
        busy_true_time=4.1,
    )

    assert not state.fresh_command_completed("open", 0.0, requested_at=5.0)


def test_fresh_verified_open_completion_satisfies_initialization():
    state = GripperInitializationState(
        accepted_command="open",
        accepted_time=5.1,
        completed_command="open",
        completed_time=5.8,
        commanded_state=0.0,
        driver_busy=False,
        last_command_ok=True,
        busy_true_time=5.2,
    )

    assert state.fresh_command_completed("open", 0.0, requested_at=5.0)
def test_initializer_republishes_until_fresh_acceptance(monkeypatch):
    clock = {"now": 0.0}
    initializer = RecordingGripperInitializer.__new__(RecordingGripperInitializer)
    initializer.state = GripperInitializationState(
        driver_busy=False,
        last_command_ok=True,
    )

    class FakePublisher:
        attempts = 0

        @staticmethod
        def get_subscription_count():
            return 1

        def publish(self, message):
            assert message.data == "open"
            self.attempts += 1
            if self.attempts == 2:
                initializer.state.accepted_command = "open"
                initializer.state.accepted_time = clock["now"]
                initializer.state.busy_true_time = clock["now"]
                initializer.state.completed_command = "open"
                initializer.state.completed_time = clock["now"]
                initializer.state.commanded_state = 0.0
                initializer.state.driver_busy = False
                initializer.state.last_command_ok = True

    publisher = FakePublisher()
    initializer._command_pub = publisher
    monkeypatch.setattr(initializer_module.time, "monotonic", lambda: clock["now"])
    monkeypatch.setattr(
        initializer_module.time,
        "sleep",
        lambda seconds: clock.__setitem__(
            "now", clock["now"] + max(float(seconds), 0.13)
        ),
    )

    assert initializer.ensure("open", timeout_sec=1.0) == "initialized_open"
    assert publisher.attempts == 2
