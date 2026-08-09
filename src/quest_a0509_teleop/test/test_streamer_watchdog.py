import pytest

from quest_a0509_teleop.servol_rt_streamer_node import _robot_state_code
from quest_a0509_teleop.streamer_watchdog import (
    MuxHeartbeatWatchdog,
    RobotStateWatchdog,
)


def test_default_timeout_allows_one_second_of_jitter():
    watchdog = MuxHeartbeatWatchdog(required=True)
    watchdog.mark(1.0)
    assert watchdog.is_fresh(2.0)
    assert not watchdog.is_fresh(2.01)


def test_direct_mode_preserves_existing_behavior_without_heartbeat():
    watchdog = MuxHeartbeatWatchdog(required=False)
    assert watchdog.is_fresh(100.0)
    assert not watchdog.should_disable_live(True, 100.0)


def test_required_mode_rejects_live_without_heartbeat():
    watchdog = MuxHeartbeatWatchdog(required=True, timeout_sec=0.35)
    assert not watchdog.is_fresh(0.0)
    assert "no selected-command heartbeat" in watchdog.enable_reject_reason(0.0)


def test_fresh_heartbeat_allows_live():
    watchdog = MuxHeartbeatWatchdog(required=True, timeout_sec=0.35)
    watchdog.mark(1.0)
    assert watchdog.is_fresh(1.34)
    assert watchdog.enable_reject_reason(1.34) is None


def test_stale_heartbeat_rejects_live_enable():
    watchdog = MuxHeartbeatWatchdog(required=True, timeout_sec=0.35)
    watchdog.mark(1.0)
    assert not watchdog.is_fresh(1.36)
    assert "stale" in watchdog.enable_reject_reason(1.36)


def test_live_stream_is_disabled_when_mux_process_stops_heartbeating():
    watchdog = MuxHeartbeatWatchdog(required=True, timeout_sec=0.35)
    watchdog.mark(1.0)
    assert not watchdog.should_disable_live(True, 1.34)
    assert watchdog.should_disable_live(True, 1.36)


def test_watchdog_only_requests_disable_for_live_stream():
    watchdog = MuxHeartbeatWatchdog(required=True, timeout_sec=0.35)
    assert not watchdog.should_disable_live(False, 10.0)


def test_robot_state_watchdog_rejects_live_before_first_async_sample():
    watchdog = RobotStateWatchdog(required=True, timeout_sec=1.0)
    assert "no robot-state topic sample" in watchdog.enable_reject_reason(0.0)


def test_robot_state_topic_parser_accepts_integer_valued_float():
    assert _robot_state_code([1.0]) == 1
    assert _robot_state_code([2.0, 999.0]) == 2


@pytest.mark.parametrize("data", [[], [float("nan")], [1.25]])
def test_robot_state_topic_parser_rejects_invalid_sample(data):
    with pytest.raises(ValueError):
        _robot_state_code(data)


def test_robot_state_watchdog_accepts_fresh_safe_state():
    watchdog = RobotStateWatchdog(required=True, safe_states=(1, 2), timeout_sec=1.0)
    watchdog.mark(1, 10.0)
    assert watchdog.enable_reject_reason(10.99) is None


def test_robot_state_watchdog_rejects_unsafe_state():
    watchdog = RobotStateWatchdog(required=True, safe_states=(1, 2), timeout_sec=1.0)
    watchdog.mark(3, 10.0)
    assert "robot_state=3" in watchdog.enable_reject_reason(10.1)


def test_robot_state_watchdog_rejects_stale_cache():
    watchdog = RobotStateWatchdog(required=True, timeout_sec=1.0)
    watchdog.mark(1, 10.0)
    assert "stale" in watchdog.enable_reject_reason(11.01)
    assert watchdog.should_disable_live(True, 11.01)


def test_robot_state_watchdog_can_be_disabled_for_dry_run():
    watchdog = RobotStateWatchdog(required=False)
    assert watchdog.enable_reject_reason(100.0) is None
    assert not watchdog.should_disable_live(True, 100.0)
