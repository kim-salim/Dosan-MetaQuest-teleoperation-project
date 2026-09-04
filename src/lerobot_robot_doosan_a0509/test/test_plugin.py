import math
import time

import numpy as np
import pytest

pytest.importorskip("lerobot")
pytest.importorskip("rclpy")

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots import make_robot_from_config
from lerobot.robots.config import RobotConfig
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.teleoperators.config import TeleoperatorConfig

from lerobot_robot_doosan_a0509 import (
    DoosanA0509Ros,
    DoosanA0509RosConfig,
    MetaQuestA0509,
    MetaQuestA0509Config,
    ZedLeftCamera,
    ZedLeftCameraConfig,
)
from lerobot_robot_doosan_a0509.doosan_a0509_ros import (
    ACTION_KEYS,
    OBSERVATION_SCALAR_KEYS,
    ROLLOUT_ACTION_KEYS,
    ROLLOUT_ACTION_KEY_MAP,
    ROLLOUT_OBSERVATION_KEYS,
    ROLLOUT_OBSERVATION_KEY_MAP,
    TCP_KEYS,
)
from lerobot_robot_doosan_a0509.gripper_latch import GripperLatch
from lerobot_robot_doosan_a0509.ros_runtime import RosRuntime
from lerobot_robot_doosan_a0509.topic_cache import TopicCache, TopicUnavailableError


VALID_ACTION = {
    "target_x_mm": 400.0,
    "target_y_mm": 0.0,
    "target_z_mm": 350.0,
    "target_o1_deg": 0.0,
    "target_o2_deg": 60.0,
    "target_o3_deg": 0.0,
    "gripper_target": 0.0,
}


def robot_config(tmp_path, mode="shadow_record"):
    return DoosanA0509RosConfig(
        id=f"test_{mode}",
        calibration_dir=tmp_path,
        mode=mode,
        cameras={},
        require_camera=False,
        require_fresh_state_on_connect=False,
    )


def test_default_cameras_register_two_c920_and_zed_left_rgb(tmp_path):
    config = DoosanA0509RosConfig(
        id="default_camera",
        calibration_dir=tmp_path,
        require_fresh_state_on_connect=False,
    )
    assert set(config.cameras) == {"front", "side", "zed_rgb"}
    front = config.cameras["front"]
    side = config.cameras["side"]
    zed = config.cameras["zed_rgb"]
    assert front.index_or_path == (
        "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0"
    )
    assert side.index_or_path == (
        "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_947C90BF-video-index0"
    )
    assert (front.width, front.height, front.fps, front.fourcc) == (
        640,
        480,
        30,
        "MJPG",
    )
    assert (side.width, side.height, side.fps, side.fourcc) == (
        640,
        480,
        30,
        "MJPG",
    )
    assert front.warmup_s == 5
    assert side.warmup_s == 5
    assert isinstance(zed, ZedLeftCameraConfig)
    assert zed.index_or_path.endswith("ZED_2-video-index0")
    assert (zed.width, zed.height, zed.fps) == (672, 376, 30)
    assert (zed.capture_width, zed.capture_height) == (1344, 376)
    assert config.camera_frame_max_age_ms == 500
    assert config.camera_startup_frame_max_age_ms == 1000
    assert config.camera_startup_grace_reads == 30
    assert config.state_max_age_sec == 0.5
    assert config.state_startup_max_age_sec == 1.0
    assert config.state_startup_grace_reads == 30
    assert config.commanded_posx_topic == "/vr/commanded_posx"


def test_commanded_posx_callback_caches_streamer_output(tmp_path):
    robot = DoosanA0509Ros(robot_config(tmp_path))
    message = type("Message", (), {"data": [400.0, 0.0, 450.0, 0.0, 150.0, 0.0]})()
    robot._on_commanded_posx(message)
    assert robot.cache.require_present("commanded_posx").value == (
        400.0,
        0.0,
        450.0,
        0.0,
        150.0,
        0.0,
    )


def test_camera_frame_age_limit_returns_to_normal_after_startup_grace(tmp_path):
    class RecordingCamera:
        def __init__(self):
            self.is_connected = False
            self.max_ages = []

        def connect(self):
            self.is_connected = True

        def disconnect(self):
            self.is_connected = False

        def read_latest(self, max_age_ms=500):
            self.max_ages.append(max_age_ms)
            return np.zeros((2, 2, 3), dtype=np.uint8)

    config = DoosanA0509RosConfig(
        id="camera_startup_grace",
        calibration_dir=tmp_path,
        cameras={},
        require_camera=False,
        require_fresh_state_on_connect=False,
        camera_frame_max_age_ms=500,
        camera_startup_frame_max_age_ms=1000,
        camera_startup_grace_reads=1,
    )
    robot = DoosanA0509Ros(config)
    camera = RecordingCamera()
    robot.cameras = {"test": camera}
    try:
        robot.connect()
        populate_robot_state(robot)
        robot.get_observation()
        robot.get_observation()
        assert camera.max_ages == [1000, 500]
    finally:
        robot.disconnect()


def test_robot_state_age_limit_returns_to_normal_after_startup_grace(tmp_path):
    config = robot_config(tmp_path)
    config.state_max_age_sec = 0.5
    config.state_startup_max_age_sec = 1.0
    config.state_startup_grace_reads = 1
    robot = DoosanA0509Ros(config)
    try:
        robot.connect()
        populate_robot_state(robot, age=0.75)
        robot.get_observation()
        with pytest.raises(TopicUnavailableError, match="stale topic joint_positions"):
            robot.get_observation()
    finally:
        robot.disconnect()


def test_default_camera_factory_builds_custom_zed_adapter(tmp_path):
    config = DoosanA0509RosConfig(
        id="camera_factory",
        calibration_dir=tmp_path,
        require_fresh_state_on_connect=False,
    )
    cameras = make_cameras_from_configs(config.cameras)
    assert set(cameras) == {"front", "side", "zed_rgb"}
    assert isinstance(cameras["zed_rgb"], ZedLeftCamera)


def test_zed_left_camera_crops_left_half_and_converts_rgb():
    config = ZedLeftCameraConfig(
        index_or_path="/dev/null",
        width=4,
        height=2,
        capture_width=8,
        capture_height=2,
    )
    camera = ZedLeftCamera(config)
    stereo_bgr = np.zeros((2, 8, 3), dtype=np.uint8)
    stereo_bgr[:, :4] = (1, 2, 3)
    stereo_bgr[:, 4:] = (7, 8, 9)

    left_rgb = camera._postprocess_image(stereo_bgr)

    assert left_rgb.shape == (2, 4, 3)
    assert left_rgb.flags.c_contiguous
    assert np.all(left_rgb == np.array([3, 2, 1], dtype=np.uint8))


def test_config_rejects_unsupported_mode(tmp_path):
    with pytest.raises(ValueError, match="unsupported robot mode"):
        DoosanA0509RosConfig(
            id="invalid_mode",
            calibration_dir=tmp_path,
            mode="invalid",
            cameras={},
            require_camera=False,
        )


def populate_robot_state(robot, *, ready=True, live=True, state=1, age=0.0):
    now = time.monotonic() - age
    robot.cache.update("joint_positions", (0.0,) * 6, receive_time=now)
    robot.cache.update(
        "actual_tcp_position", (400.0, 0.0, 350.0, 0.0, 60.0, 0.0), receive_time=now
    )
    robot.cache.update("robot_state", float(state), receive_time=now)
    robot.cache.update("gripper_commanded_state", 0.0, receive_time=now)
    robot.cache.update("solution_space", 0.0, receive_time=now)
    robot.cache.update("teleop_ready", ready, receive_time=now)
    robot.cache.update("live_state", live, receive_time=now)


def test_config_registration_and_factory(tmp_path):
    assert "doosan_a0509_ros" in RobotConfig.get_known_choices()
    assert "metaquest_a0509" in TeleoperatorConfig.get_known_choices()
    robot = make_robot_from_config(robot_config(tmp_path))
    teleop = make_teleoperator_from_config(
        MetaQuestA0509Config(
            id="factory",
            calibration_dir=tmp_path,
            require_calibration=False,
            require_fresh_action_on_connect=False,
        )
    )
    assert isinstance(robot, DoosanA0509Ros)
    assert isinstance(teleop, MetaQuestA0509)


def test_shared_runtime_and_connect_configure_calibrate_publish_nothing(tmp_path):
    robot = DoosanA0509Ros(robot_config(tmp_path))
    teleop = MetaQuestA0509(
        MetaQuestA0509Config(
            id="runtime",
            calibration_dir=tmp_path,
            require_fresh_action_on_connect=False,
        )
    )
    try:
        robot.connect()
        teleop.connect(calibrate=False)
        assert robot._runtime is teleop._runtime
        assert RosRuntime.reference_count() == 2
        robot.configure()
        robot.calibrate()
        teleop.configure()
        teleop.cache.update("calibration_valid", True)
        teleop.calibrate()
        assert robot.live_publish_count == 0
        assert robot.debug_publish_count == 0
    finally:
        teleop.disconnect()
        robot.disconnect()
    assert RosRuntime.reference_count() == 0


def test_observation_features_match_returned_state(tmp_path):
    robot = DoosanA0509Ros(robot_config(tmp_path))
    try:
        robot.connect()
        populate_robot_state(robot)
        observation = robot.get_observation()
        assert set(observation) == set(robot.observation_features)
        assert len(observation) == 13
        assert observation["tcp_o2_deg"] == 60.0
    finally:
        robot.disconnect()


def test_recording_preserves_raw_doosan_zyz_branch(tmp_path):
    robot = DoosanA0509Ros(robot_config(tmp_path, "shadow_record"))
    raw_orientation = (177.3676605, -149.9667816, 177.7001038)
    try:
        robot.connect()
        populate_robot_state(robot)
        robot.cache.update(
            "actual_tcp_position",
            (428.72, -2.53, 456.93, *raw_orientation),
        )

        observation = robot.get_observation()

        assert tuple(observation[key] for key in TCP_KEYS[3:]) == pytest.approx(
            raw_orientation
        )
    finally:
        robot.disconnect()


@pytest.mark.parametrize("mode", ("policy_dry_run", "policy_live"))
def test_policy_observation_canonicalizes_equivalent_doosan_zyz_branch(
    tmp_path, mode
):
    robot = DoosanA0509Ros(robot_config(tmp_path, mode))
    raw_orientation = (177.3676605, -149.9667816, 177.7001038)
    expected_canonical = (-2.6323395, 149.9667816, -2.2998962)
    try:
        robot.connect()
        populate_robot_state(robot)
        robot.cache.update(
            "actual_tcp_position",
            (428.72, -2.53, 456.93, *raw_orientation),
        )

        observation = robot.get_observation()
        canonical = tuple(
            observation[ROLLOUT_OBSERVATION_KEY_MAP[key]] for key in TCP_KEYS[3:]
        )

        assert canonical == pytest.approx(expected_canonical, abs=1.0e-6)
    finally:
        robot.disconnect()


def test_recording_observation_allows_old_latched_gripper_state(tmp_path):
    robot = DoosanA0509Ros(robot_config(tmp_path, "shadow_record"))
    try:
        robot.connect()
        populate_robot_state(robot)
        robot.cache.update(
            "gripper_commanded_state",
            1.0,
            receive_time=time.monotonic() - 60.0,
        )
        observation = robot.get_observation()
        assert observation["gripper_commanded_state"] == 1.0
    finally:
        robot.disconnect()


def test_policy_live_connect_separates_dynamic_and_latched_state(tmp_path):
    config = DoosanA0509RosConfig(
        id="test_policy_live_connect_gate",
        calibration_dir=tmp_path,
        mode="policy_live",
        cameras={},
        require_camera=False,
        require_fresh_state_on_connect=False,
    )
    robot = DoosanA0509Ros(config)
    populate_robot_state(robot)
    old = time.monotonic() - 60.0
    robot.cache.update("gripper_commanded_state", 0.0, receive_time=old)
    robot.cache.update("teleop_ready", True, receive_time=old)
    robot.cache.update("live_state", False, receive_time=old)
    robot._wait_for_required_state(0.0)

    robot.cache.update("robot_state", 1.0, receive_time=old)
    with pytest.raises(RuntimeError, match="stale topic robot_state"):
        robot._wait_for_required_state(0.0)


def test_shadow_record_validates_but_never_publishes(tmp_path):
    robot = DoosanA0509Ros(robot_config(tmp_path, "shadow_record"))
    try:
        robot.connect()
        assert robot.send_action(VALID_ACTION) == VALID_ACTION
        assert robot.live_publish_count == 0
        assert robot.debug_publish_count == 0
    finally:
        robot.disconnect()


def test_policy_dry_run_only_uses_debug_publisher(tmp_path):
    robot = DoosanA0509Ros(robot_config(tmp_path, "policy_dry_run"))
    try:
        robot.connect()
        robot.send_action(VALID_ACTION)
        assert robot.debug_publish_count == 1
        assert robot.live_publish_count == 0
        assert robot.config.debug_action_topic == "/control/lerobot/debug_action"
    finally:
        robot.disconnect()


@pytest.mark.parametrize("mode", ("policy_dry_run", "policy_live"))
def test_policy_modes_expose_lerobot_rollout_position_aliases(tmp_path, mode):
    robot = DoosanA0509Ros(robot_config(tmp_path, mode))
    try:
        robot.connect()
        populate_robot_state(robot)
        observation = robot.get_observation()
        assert tuple(robot.observation_features) == ROLLOUT_OBSERVATION_KEYS
        assert tuple(observation) == ROLLOUT_OBSERVATION_KEYS
        assert tuple(robot.action_features) == ROLLOUT_ACTION_KEYS
        assert all(key.endswith(".pos") for key in ROLLOUT_OBSERVATION_KEYS)
        assert all(key.endswith(".pos") for key in ROLLOUT_ACTION_KEYS)
        assert len(ROLLOUT_OBSERVATION_KEYS) == len(OBSERVATION_SCALAR_KEYS) == 13
        assert len(ROLLOUT_ACTION_KEYS) == len(ACTION_KEYS) == 7
    finally:
        robot.disconnect()


def test_policy_rollout_action_aliases_map_back_to_canonical_contract(tmp_path):
    robot = DoosanA0509Ros(robot_config(tmp_path, "policy_dry_run"))
    alias_action = {
        ROLLOUT_ACTION_KEY_MAP[key]: value for key, value in VALID_ACTION.items()
    }
    try:
        robot.connect()
        returned = robot.send_action(alias_action)
        assert returned == VALID_ACTION
        assert robot.debug_publish_count == 1
        assert robot.live_publish_count == 0
    finally:
        robot.disconnect()


def test_policy_gripper_clips_only_small_numeric_overshoot(tmp_path):
    robot = DoosanA0509Ros(robot_config(tmp_path, "policy_dry_run"))
    try:
        robot.connect()
        below = robot.send_action({**VALID_ACTION, "gripper_target": -0.0009})
        above = robot.send_action({**VALID_ACTION, "gripper_target": 1.0009})
        assert below["gripper_target"] == 0.0
        assert above["gripper_target"] == 1.0

        with pytest.raises(ValueError, match="gripper_target"):
            robot.send_action({**VALID_ACTION, "gripper_target": -0.06})
        with pytest.raises(ValueError, match="gripper_target"):
            robot.send_action({**VALID_ACTION, "gripper_target": 1.06})
        assert robot.debug_publish_count == 2
        assert robot.live_publish_count == 0
    finally:
        robot.disconnect()


def test_policy_live_clips_bounded_gripper_regression_overshoot(tmp_path):
    robot = DoosanA0509Ros(robot_config(tmp_path, "policy_live"))
    try:
        robot.connect()
        populate_robot_state(robot)

        below = robot.send_action({**VALID_ACTION, "gripper_target": -0.010388})
        above = robot.send_action(
            {**VALID_ACTION, "gripper_target": 1.0727028846740723}
        )
        assert below["gripper_target"] == 0.0
        assert above["gripper_target"] == 1.0

        with pytest.raises(ValueError, match="gripper_target"):
            robot.send_action({**VALID_ACTION, "gripper_target": -0.251})
        with pytest.raises(ValueError, match="gripper_target"):
            robot.send_action({**VALID_ACTION, "gripper_target": 1.251})
        assert robot.live_publish_count == 2
    finally:
        robot.disconnect()


def test_policy_live_publishes_only_mux_inputs_after_gate(tmp_path):
    robot = DoosanA0509Ros(robot_config(tmp_path, "policy_live"))
    try:
        robot.connect()
        populate_robot_state(robot)
        old = time.monotonic() - 60.0
        robot.cache.update("gripper_commanded_state", 0.0, receive_time=old)
        robot.cache.update("teleop_ready", True, receive_time=old)
        robot.cache.update("live_state", True, receive_time=old)
        robot.send_action(VALID_ACTION)
        assert robot.live_publish_count == 1
        assert robot.config.lerobot_target_topic == "/control/lerobot/target_posx"
        assert robot.config.lerobot_gripper_topic == "/control/lerobot/gripper_target"
        assert "/vr/target_posx" not in {
            robot.config.lerobot_target_topic,
            robot.config.lerobot_gripper_topic,
        }
    finally:
        robot.disconnect()


def test_policy_live_rejects_stale_not_ready_and_disallowed_state(tmp_path):
    robot = DoosanA0509Ros(robot_config(tmp_path, "policy_live"))
    try:
        robot.connect()
        populate_robot_state(robot, age=1.0)
        with pytest.raises(TopicUnavailableError):
            robot.send_action(VALID_ACTION)
        populate_robot_state(robot, ready=False)
        with pytest.raises(RuntimeError, match="teleop_ready=false"):
            robot.send_action(VALID_ACTION)
        populate_robot_state(robot, live=False)
        with pytest.raises(RuntimeError, match="Live robot output is disabled"):
            robot.send_action(VALID_ACTION)
        populate_robot_state(robot, state=99)
        with pytest.raises(RuntimeError, match="robot_state=99"):
            robot.send_action(VALID_ACTION)
        assert robot.live_publish_count == 0
    finally:
        robot.disconnect()


def test_action_schema_and_numeric_validation(tmp_path):
    robot = DoosanA0509Ros(robot_config(tmp_path))
    try:
        robot.connect()
        assert set(robot.action_features) == set(VALID_ACTION)
        bad = {**VALID_ACTION, "target_x_mm": math.nan}
        with pytest.raises(ValueError, match="non-finite"):
            robot.send_action(bad)
        bad = {**VALID_ACTION, "gripper_target": 1.1}
        with pytest.raises(ValueError, match="gripper_target"):
            robot.send_action(bad)
    finally:
        robot.disconnect()


def test_metaquest_returns_mapper_target_without_coordinate_conversion(tmp_path):
    teleop = MetaQuestA0509(
        MetaQuestA0509Config(
            id="teacher",
            calibration_dir=tmp_path,
            require_fresh_action_on_connect=False,
        )
    )
    try:
        teleop.connect(calibrate=False)
        target = (410.0, -20.0, 365.0, 5.0, 42.0, -7.0)
        teleop.cache.update("target_posx", target)
        teleop.cache.update("gripper_commanded_state", 1.0)
        teleop.cache.update("valid_pose_heartbeat", True)
        teleop.cache.update("calibration_valid", True)
        teleop.cache.update("teleop_ready", True)
        action = teleop.get_action()
        assert tuple(action[key] for key in tuple(VALID_ACTION)[:6]) == target
        assert action["gripper_target"] == 1.0
        assert set(action) == set(teleop.action_features)
    finally:
        teleop.disconnect()


def test_metaquest_shadow_record_accepts_latched_state_only(tmp_path):
    teleop = MetaQuestA0509(
        MetaQuestA0509Config(
            id="teacher_shadow_state",
            calibration_dir=tmp_path,
            require_fresh_action_on_connect=False,
            allow_latched_state_in_shadow_record=True,
            max_age_sec=0.3,
        )
    )
    try:
        teleop.connect(calibrate=False)
        old = time.monotonic() - 60.0
        target = (410.0, -20.0, 365.0, 5.0, 42.0, -7.0)
        teleop.cache.update("target_posx", target)
        teleop.cache.update("valid_pose_heartbeat", True)
        teleop.cache.update("gripper_commanded_state", 1.0, receive_time=old)
        teleop.cache.update("calibration_valid", True, receive_time=old)
        teleop.cache.update("teleop_ready", True, receive_time=old)
        assert teleop.get_action()["gripper_target"] == 1.0
        teleop.cache.update("target_posx", target, receive_time=old)
        with pytest.raises(TopicUnavailableError, match="stale topic target_posx"):
            teleop.get_action()
    finally:
        teleop.disconnect()


def test_metaquest_default_still_rejects_stale_state(tmp_path):
    teleop = MetaQuestA0509(
        MetaQuestA0509Config(
            id="teacher_strict_state",
            calibration_dir=tmp_path,
            require_fresh_action_on_connect=False,
            max_age_sec=0.3,
        )
    )
    try:
        teleop.connect(calibrate=False)
        old = time.monotonic() - 60.0
        target = (410.0, -20.0, 365.0, 5.0, 42.0, -7.0)
        teleop.cache.update("target_posx", target)
        teleop.cache.update("valid_pose_heartbeat", True)
        teleop.cache.update("gripper_commanded_state", 1.0, receive_time=old)
        teleop.cache.update("calibration_valid", True)
        teleop.cache.update("teleop_ready", True, receive_time=old)
        with pytest.raises(TopicUnavailableError, match="stale topic teleop_ready"):
            teleop.get_action()
    finally:
        teleop.disconnect()


def test_metaquest_rejects_invalid_calibration_on_connect_and_action(tmp_path):
    teleop = MetaQuestA0509(
        MetaQuestA0509Config(
            id="uncalibrated_teacher",
            calibration_dir=tmp_path,
            connect_timeout_sec=0.0,
            require_fresh_action_on_connect=False,
        )
    )
    with pytest.raises(RuntimeError, match="calibration is missing or invalid"):
        teleop.connect()
    assert not teleop.is_connected

    teleop.connect(calibrate=False)
    try:
        with pytest.raises(RuntimeError, match="calibration is invalid"):
            teleop.get_action()
        assert teleop.config.heartbeat_topic == (
            "/control/metaquest/valid_pose_heartbeat"
        )
    finally:
        teleop.disconnect()


def test_metaquest_can_defer_connect_calibration_but_still_rejects_action(tmp_path):
    teleop = MetaQuestA0509(
        MetaQuestA0509Config(
            id="deferred_calibration_teacher",
            calibration_dir=tmp_path,
            connect_timeout_sec=0.0,
            require_fresh_action_on_connect=False,
            defer_calibration_on_connect=True,
        )
    )
    teleop.connect()
    try:
        assert teleop.is_connected
        with pytest.raises(RuntimeError, match="calibration is invalid"):
            teleop.get_action()
    finally:
        teleop.disconnect()


def test_gripper_stop_preserves_latched_state():
    latch = GripperLatch(1.0)
    assert latch.update_command("stop") == 1.0
    assert latch.update_command("open") == 0.0
    assert latch.update_command("stop") == 0.0


def test_topic_cache_tracks_count_header_and_freshness():
    cache = TopicCache()
    first = cache.update("x", 1, receive_time=1.0, header_stamp=10.0)
    second = cache.update("x", 2, receive_time=2.0, header_stamp=11.0)
    assert first.receive_count == 1
    assert second.receive_count == 2
    assert cache.require("x", max_age_sec=0.5, now=2.4).value == 2
    with pytest.raises(TopicUnavailableError):
        cache.require("x", max_age_sec=0.5, now=2.6)
    assert cache.require_present("x").value == 2
    with pytest.raises(TopicUnavailableError):
        cache.require_present("missing")
