from pathlib import Path

import pytest

from quest_a0509_teleop.prep_safety import (
    PreparationCancelled,
    PreparationStopToken,
)


def test_stop_token_starts_clear_and_can_be_reused():
    token = PreparationStopToken()
    assert not token.requested
    token.request()
    assert token.requested
    token.begin()
    assert not token.requested


def test_stop_token_interrupts_check_and_wait():
    token = PreparationStopToken()
    token.request()
    with pytest.raises(PreparationCancelled):
        token.check()
    with pytest.raises(PreparationCancelled):
        token.wait(10.0)


def test_stop_callback_is_not_blocked_by_prepare_operation_lock():
    package_root = Path(__file__).resolve().parents[1]
    source = (
        package_root / "quest_a0509_teleop" / "robot_prep_node.py"
    ).read_text()
    stop_handler = source.split("    def _on_stop_robot(", 1)[1].split(
        "    def _on_reset_safe_off(", 1
    )[0]
    assert "with self.operation_lock" not in stop_handler
    assert stop_handler.index("self.prepare_stop.request()") < stop_handler.index(
        "self._move_stop()"
    )
