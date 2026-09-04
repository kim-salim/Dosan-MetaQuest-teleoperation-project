from __future__ import annotations

import ast
import inspect
import textwrap
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("lerobot")
pytest.importorskip("rclpy")

from lerobot_robot_doosan_a0509.config_doosan_a0509_ros import (
    DoosanA0509RosConfig,
)
from lerobot_robot_doosan_a0509.doosan_a0509_ros import (
    ACTION_KEYS,
    ROLLOUT_ACTION_KEYS,
    ROLLOUT_OBSERVATION_KEYS,
    DoosanA0509Ros,
)
from lerobot_robot_doosan_a0509.task_c_shadow_entrypoint import (
    require_read_only_shadow_arguments,
)
from lerobot_robot_doosan_a0509.task_c_shadow_rollout import (
    TaskCShadowStrategy,
    TaskCShadowStrategyConfig,
    _policy_input_from_processed,
    _state_and_semantics,
)


def _robot_config(tmp_path) -> DoosanA0509RosConfig:
    return DoosanA0509RosConfig(
        id="task_c_shadow_test",
        calibration_dir=tmp_path,
        mode="policy_shadow",
        cameras={},
        require_camera=False,
        require_fresh_state_on_connect=False,
    )


def _populate_state(robot: DoosanA0509Ros) -> None:
    robot.cache.update("joint_positions", tuple(float(index) for index in range(6)))
    robot.cache.update(
        "actual_tcp_position", (400.0, 0.0, 350.0, 0.0, 60.0, 0.0)
    )
    robot.cache.update("gripper_commanded_state", 1.0)


def test_policy_shadow_creates_no_publishers_and_exposes_policy_aliases(tmp_path):
    robot = DoosanA0509Ros(_robot_config(tmp_path))
    try:
        robot.connect()
        _populate_state(robot)
        observation = robot.get_observation()
        assert tuple(observation) == ROLLOUT_OBSERVATION_KEYS
        assert tuple(robot.action_features) == ROLLOUT_ACTION_KEYS
        assert robot._target_pub is None
        assert robot._gripper_pub is None
        assert robot._debug_pub is None
        assert robot._policy_ready_pub is None
        action = dict(zip(ACTION_KEYS, [0.0] * 7, strict=True))
        assert robot.send_action(action) == action
        assert robot.live_publish_count == 0
        assert robot.debug_publish_count == 0
        assert robot.hold_publish_count == 0
    finally:
        robot.disconnect()


def test_entrypoint_rejects_any_non_shadow_mode_before_context_build():
    valid = [
        "--strategy.type=task_c_shadow",
        "--robot.mode=policy_shadow",
        "--return_to_initial_position=false",
    ]
    require_read_only_shadow_arguments(valid)
    with pytest.raises(ValueError, match="robot.mode=policy_shadow"):
        require_read_only_shadow_arguments(
            [argument.replace("policy_shadow", "policy_live") for argument in valid]
        )
    with pytest.raises(ValueError, match="return_to_initial_position=false"):
        require_read_only_shadow_arguments(valid[:-1])


def test_live_snapshot_conversion_matches_unbatched_act_contract():
    state_names = [f"state_{index}.pos" for index in range(13)]
    processed = {name: float(index) for index, name in enumerate(state_names)}
    processed["front"] = np.full((4, 5, 3), 255, dtype=np.uint8)
    hw_features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (13,),
            "names": state_names,
        },
        "observation.images.front": {
            "dtype": "image",
            "shape": (4, 5, 3),
            "names": ["height", "width", "channels"],
        },
    }
    expected = {
        "observation.state": SimpleNamespace(shape=(13,)),
        "observation.images.front": SimpleNamespace(shape=(3, 4, 5)),
    }
    result = _policy_input_from_processed(
        processed, hw_features=hw_features, expected_features=expected
    )
    assert tuple(result["observation.state"].shape) == (13,)
    assert tuple(result["observation.images.front"].shape) == (3, 4, 5)
    assert float(result["observation.images.front"].max()) == pytest.approx(1.0)
    tcp, semantic = _state_and_semantics(result)
    assert tcp.tolist() == [6.0, 7.0, 8.0]
    assert semantic.gripper_closed is True
    assert semantic.holding is True


def test_shadow_strategy_contains_no_send_action_call():
    tree = ast.parse(textwrap.dedent(inspect.getsource(TaskCShadowStrategy)))
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "send_action"
    ]
    assert calls == []


def test_command_guard_blocks_accidental_dispatch():
    class Robot:
        def send_action(self, action):
            return action

    strategy = TaskCShadowStrategy(TaskCShadowStrategyConfig())
    robot = Robot()
    strategy._raw_robot = robot
    strategy._install_command_guard(robot)
    with pytest.raises(RuntimeError, match="forbids every send_action"):
        robot.send_action({})
    assert strategy._command_attempts == 1
    strategy._remove_command_guard()
    assert robot.send_action({"safe": True}) == {"safe": True}
