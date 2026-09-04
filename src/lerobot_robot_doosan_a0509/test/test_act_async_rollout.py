from __future__ import annotations

import pytest
import torch

from lerobot_robot_doosan_a0509.act_async_rollout import (
    _blend_processed_action_overlap,
    _environment_nonnegative_int,
    _reset_policy_live_state_for_test,
    configure_policy_live_queue,
    get_act_gpu_arbiter,
    install_act_async_chunk_compat,
    install_act_async_overlap_blending,
    merge_fifo_action_chunks,
    notify_policy_live_state,
    policy_live_hold_required,
    require_async_fifo_arguments,
)

from quest_a0509_teleop.doosan_orientation import (
    doosan_zyz_deg_to_quaternion,
    quaternion_angle_deg,
)


def test_gpu_arbiter_is_process_wide_and_reentrant():
    first = get_act_gpu_arbiter()
    second = get_act_gpu_arbiter()
    assert first is second
    with first:
        with second:
            pass


def test_async_fifo_arguments_require_disabled_rtc_and_100_steps():
    require_async_fifo_arguments(
        [
            "--inference.type=rtc",
            "--inference.rtc.enabled=false",
            "--policy.n_action_steps=100",
        ]
    )

    with pytest.raises(ValueError, match="requires --inference.type=rtc"):
        require_async_fifo_arguments(
            ["--inference.type=sync", "--policy.n_action_steps=100"]
        )
    with pytest.raises(ValueError, match="prefix-guided RTC"):
        require_async_fifo_arguments(
            ["--inference.type=rtc", "--policy.n_action_steps=100"]
        )
    with pytest.raises(ValueError, match="n_action_steps=100"):
        require_async_fifo_arguments(
            [
                "--inference.type=rtc",
                "--inference.rtc.enabled=false",
                "--policy.n_action_steps=10",
            ]
        )


def test_environment_warmup_count(monkeypatch):
    monkeypatch.setenv("TEST_WARMUP", "3")
    assert _environment_nonnegative_int("TEST_WARMUP", 2) == 3
    monkeypatch.setenv("TEST_WARMUP", "-1")
    with pytest.raises(ValueError, match="non-negative"):
        _environment_nonnegative_int("TEST_WARMUP", 2)


def test_act_async_adapter_accepts_worker_keywords_and_warms_once(monkeypatch):
    from lerobot.policies.act.modeling_act import ACTPolicy

    calls = []

    def fake_predict(self, batch):
        calls.append((self, batch))
        return torch.zeros((1, 100, 7))

    monkeypatch.setattr(ACTPolicy, "predict_action_chunk", fake_predict)
    assert install_act_async_chunk_compat(warmup_inferences=2) is True
    patched = ACTPolicy.predict_action_chunk
    assert install_act_async_chunk_compat(warmup_inferences=2) is False

    class Owner:
        pass

    owner = Owner()
    batch = {"observation.state": torch.zeros((1, 13))}
    result = patched(
        owner,
        batch,
        inference_delay=1,
        prev_chunk_left_over=torch.zeros((5, 7)),
    )
    assert result.shape == (1, 100, 7)
    assert len(calls) == 3

    patched(owner, batch, inference_delay=0, prev_chunk_left_over=None)
    assert len(calls) == 4


def test_act_async_adapter_pins_each_native_inference_thread(monkeypatch):
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot_robot_doosan_a0509 import act_async_rollout as module

    current_native_id = [1001]
    pin_calls = []

    def fake_predict(_self, _batch):
        return torch.zeros((1, 100, 7))

    def fake_pin(name):
        pin_calls.append((current_native_id[0], name))
        return (9, 10, 11, 12)

    monkeypatch.setattr(ACTPolicy, "predict_action_chunk", fake_predict)
    monkeypatch.setattr(module, "get_native_id", lambda: current_native_id[0])
    monkeypatch.setattr(module, "pin_current_thread_from_env", fake_pin)
    assert install_act_async_chunk_compat(warmup_inferences=0) is True
    patched = ACTPolicy.predict_action_chunk

    class Owner:
        pass

    owner = Owner()
    batch = {"observation.state": torch.zeros((1, 13))}
    patched(owner, batch)
    current_native_id[0] = 2002
    patched(owner, batch)

    assert [item[0] for item in pin_calls] == [1001, 2002]
    assert getattr(owner, module._INFERENCE_PIN_MARKER) == frozenset({1001, 2002})


def test_fifo_merge_delay_aligns_and_smoothly_replaces_old_tail():
    old = torch.tensor(
        [
            [0.0, 0.0, 0.0, 10.0, 40.0, 20.0, 0.0],
            [10.0, 0.0, 0.0, 10.0, 40.0, 20.0, 0.0],
            [20.0, 0.0, 0.0, 10.0, 40.0, 20.0, 0.0],
            [30.0, 0.0, 0.0, 10.0, 40.0, 20.0, 0.0],
            [40.0, 0.0, 0.0, 10.0, 40.0, 20.0, 0.0],
        ]
    )
    new = torch.tensor(
        [
            [100.0, 0.0, 0.0, 20.0, 50.0, 30.0, 1.0],
            [110.0, 0.0, 0.0, 20.0, 50.0, 30.0, 1.0],
            [120.0, 0.0, 0.0, 20.0, 50.0, 30.0, 1.0],
            [130.0, 0.0, 0.0, 20.0, 50.0, 30.0, 1.0],
            [140.0, 0.0, 0.0, 20.0, 50.0, 30.0, 1.0],
            [150.0, 0.0, 0.0, 20.0, 50.0, 30.0, 1.0],
        ]
    )

    merged_original, merged_processed = merge_fifo_action_chunks(
        old,
        old,
        new,
        new,
        consumed_during_inference=2,
        overlap_steps=3,
    )

    assert merged_processed.shape == (4, 7)
    assert merged_original.shape == (4, 7)
    assert merged_processed[:, 0].tolist() == pytest.approx(
        [18.75, 70.0, 121.25, 150.0]
    )
    assert merged_processed[:, 6].tolist() == [0.0, 0.0, 0.0, 1.0]


def test_processed_overlap_uses_physical_zyz_slerp_and_holds_gripper():
    old_zyz = [5.0, 179.0, 10.0]
    new_zyz = [190.0, -175.0, 195.0]
    old = torch.tensor([[0.0, 0.0, 0.0, *old_zyz, 0.0]]).repeat(3, 1)
    new = torch.tensor([[0.0, 0.0, 0.0, *new_zyz, 1.0]]).repeat(3, 1)

    blended = _blend_processed_action_overlap(old, new, overlap_steps=3)
    start_quaternion = doosan_zyz_deg_to_quaternion(old_zyz)
    end_quaternion = doosan_zyz_deg_to_quaternion(new_zyz)
    total_angle = quaternion_angle_deg(start_quaternion, end_quaternion)
    progressed = [
        quaternion_angle_deg(
            start_quaternion,
            doosan_zyz_deg_to_quaternion(row[3:6].tolist()),
        )
        for row in blended
    ]

    assert progressed == sorted(progressed)
    assert 0.0 < progressed[0] < progressed[-1] < total_angle
    assert blended[:, 6].tolist() == [0.0, 0.0, 0.0]


def test_action_queue_discards_pre_live_inference_and_waits_for_fresh_chunk():
    from lerobot.policies.rtc import ActionQueue
    from lerobot.policies.rtc.configuration_rtc import RTCConfig

    _reset_policy_live_state_for_test()
    configure_policy_live_queue(True)
    install_act_async_overlap_blending(overlap_steps=3)
    queue = ActionQueue(RTCConfig(enabled=False))
    chunk = torch.arange(28, dtype=torch.float32).reshape(4, 7)

    # A completed pre-Live chunk is present, as it is after policy warmup.
    queue.merge(chunk, chunk, real_delay=0, action_index_before_inference=0)
    assert queue.qsize() == 4

    queue.get_action_index()
    notify_policy_live_state(True)
    # The ROS Live callback clears immediately; it does not depend on the
    # rollout thread reaching its next get().
    assert queue.qsize() == 0
    queue.merge(chunk, chunk, real_delay=0, action_index_before_inference=0)
    assert queue.qsize() == 0
    assert queue.get() is None
    assert policy_live_hold_required() is True

    fresh_index = queue.get_action_index()
    queue.merge(
        chunk + 100.0,
        chunk + 100.0,
        real_delay=0,
        action_index_before_inference=fresh_index,
    )
    assert policy_live_hold_required() is False
    assert queue.get().tolist() == (chunk[0] + 100.0).tolist()

    notify_policy_live_state(False)
    assert queue.get() is None
    assert policy_live_hold_required() is True
    _reset_policy_live_state_for_test()


def test_action_queue_consumes_normally_when_live_gate_is_disabled():
    from lerobot.policies.rtc import ActionQueue
    from lerobot.policies.rtc.configuration_rtc import RTCConfig

    _reset_policy_live_state_for_test()
    configure_policy_live_queue(False)
    install_act_async_overlap_blending(overlap_steps=3)
    queue = ActionQueue(RTCConfig(enabled=False))
    chunk = torch.arange(28, dtype=torch.float32).reshape(4, 7)

    index = queue.get_action_index()
    queue.merge(chunk, chunk, real_delay=0, action_index_before_inference=index)
    assert queue.get().tolist() == chunk[0].tolist()
    assert policy_live_hold_required() is False
    _reset_policy_live_state_for_test()
