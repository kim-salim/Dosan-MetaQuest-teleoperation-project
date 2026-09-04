import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace


def load_preflight_module():
    path = (
        Path(__file__).resolve().parents[3]
        / "scripts"
        / "preflight_lerobot_shadow_record.py"
    )
    spec = importlib.util.spec_from_file_location("a0509_shadow_preflight", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class FakeRobot:
    def __init__(self, config):
        self.config = SimpleNamespace(mode=config.mode)
        self.cameras = {}
        self.live_publish_count = 0
        self.debug_publish_count = 0
        self.is_connected = False
        self.sent_actions = []

    def connect(self):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False

    def get_observation(self):
        return {"joint_1_pos": 0.0}

    def send_action(self, action):
        self.sent_actions.append(action)
        return action


class FakeTeleop:
    def __init__(self, config):
        self.config = config
        self.action_features = {"target_x_mm": float, "gripper_target": float}
        self.is_connected = False
        self.get_action_calls = 0

    def connect(self):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False

    def get_action(self):
        self.get_action_calls += 1
        return {"target_x_mm": 400.0, "gripper_target": 0.0}


def configure_fakes(module, monkeypatch):
    robot_config = SimpleNamespace(
        mode="shadow_record",
        cameras={},
        require_camera=False,
    )
    teleop_configs = []
    robots = []
    teleops = []

    monkeypatch.setattr(module, "DoosanA0509RosConfig", lambda **_kwargs: robot_config)

    def make_teleop_config(**kwargs):
        config = SimpleNamespace(**kwargs)
        teleop_configs.append(config)
        return config

    monkeypatch.setattr(module, "MetaQuestA0509Config", make_teleop_config)

    def make_robot(config):
        robot = FakeRobot(config)
        robots.append(robot)
        return robot

    def make_teleop(config):
        teleop = FakeTeleop(config)
        teleops.append(teleop)
        return teleop

    monkeypatch.setattr(module, "DoosanA0509Ros", make_robot)
    monkeypatch.setattr(module, "MetaQuestA0509", make_teleop)
    return teleop_configs, robots, teleops


def test_unprepared_mode_defers_action_freshness_to_episode_reset(monkeypatch, capsys):
    module = load_preflight_module()
    teleop_configs, robots, teleops = configure_fakes(module, monkeypatch)

    module.main(allow_unprepared_teleop=True)
    summary = json.loads(capsys.readouterr().out)

    assert teleop_configs[0].require_fresh_action_on_connect is False
    assert teleop_configs[0].defer_calibration_on_connect is True
    assert teleops[0].get_action_calls == 0
    assert robots[0].sent_actions == []
    assert summary["status"] == "awaiting_episode_reset"


def test_legacy_preflight_still_validates_a_fresh_teacher_action(monkeypatch, capsys):
    module = load_preflight_module()
    teleop_configs, robots, teleops = configure_fakes(module, monkeypatch)

    module.main(allow_unprepared_teleop=False)
    summary = json.loads(capsys.readouterr().out)

    assert teleop_configs[0].require_fresh_action_on_connect is True
    assert teleop_configs[0].defer_calibration_on_connect is False
    assert teleops[0].get_action_calls == 1
    assert robots[0].sent_actions == [
        {"target_x_mm": 400.0, "gripper_target": 0.0}
    ]
    assert summary["status"] == "ready"
