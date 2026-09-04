from __future__ import annotations

import time
from types import SimpleNamespace

import pytest
import torch

pytest.importorskip("lerobot")

from lerobot.policies.rtc.configuration_rtc import RTCConfig
from lerobot_robot_doosan_a0509.diffusion_async_rollout import (
    DiffusionActionQueue,
    DiffusionLiveNoiseCache,
    TimeAlignedDiffusionChunk,
    _reset_diffusion_policy_live_state_for_test,
    blend_diffusion_processed_overlap,
    configure_diffusion_policy_live_queue,
    diffusion_policy_live_queue_ready,
    ensemble_time_aligned_diffusion_chunks,
    merge_diffusion_action_chunks,
    notify_diffusion_policy_live_state,
    require_diffusion_async_fifo_arguments,
    stack_temporal_observation_batches,
)
from lerobot_robot_doosan_a0509.doosan_a0509_diffusion_ros import (
    DiffusionGripperDecisionFilter,
    DoosanA0509DiffusionRos,
)
from lerobot_robot_doosan_a0509.doosan_a0509_ros import DoosanA0509Ros


VALID_ARGUMENTS = [
    "--inference.type=rtc",
    "--inference.rtc.enabled=false",
    "--inference.queue_threshold=9",
    "--policy.n_action_steps=15",
    "--robot.type=doosan_a0509_diffusion_ros",
    "--fps=30",
]


@pytest.fixture(autouse=True)
def reset_live_gate():
    _reset_diffusion_policy_live_state_for_test()
    yield
    _reset_diffusion_policy_live_state_for_test()


def _chunk(value: float, *, steps: int = 15, gripper: float = 0.0) -> torch.Tensor:
    actions = torch.full((steps, 7), value, dtype=torch.float32)
    actions[:, 3:6] = torch.tensor([0.0, 60.0, 0.0])
    actions[:, 6] = gripper
    return actions


def test_async_arguments_accept_only_measured_diffusion_baseline():
    require_diffusion_async_fifo_arguments(VALID_ARGUMENTS)

    replacements = {
        "--inference.type=rtc": "--inference.type=sync",
        "--inference.rtc.enabled=false": "--inference.rtc.enabled=true",
        "--inference.queue_threshold=9": "--inference.queue_threshold=8",
        "--policy.n_action_steps=15": "--policy.n_action_steps=8",
        "--robot.type=doosan_a0509_diffusion_ros": "--robot.type=doosan_a0509_ros",
        "--fps=30": "--fps=10",
    }
    for accepted, rejected in replacements.items():
        arguments = [rejected if value == accepted else value for value in VALID_ARGUMENTS]
        with pytest.raises(ValueError, match="requires"):
            require_diffusion_async_fifo_arguments(arguments)


def test_temporal_stack_builds_two_frame_policy_batch():
    first = {
        "observation.state": torch.zeros((1, 13)),
        "observation.images.front": torch.zeros((1, 3, 480, 640)),
        "robot_type": "doosan_a0509_diffusion_ros",
    }
    second = {
        "observation.state": torch.ones((1, 13)),
        "observation.images.front": torch.ones((1, 3, 480, 640)),
        "robot_type": "doosan_a0509_diffusion_ros",
    }

    batch = stack_temporal_observation_batches(
        [first, second],
        task="pick blue block",
    )

    assert batch["observation.state"].shape == (1, 2, 13)
    assert batch["observation.images.front"].shape == (1, 2, 3, 480, 640)
    assert torch.equal(batch["observation.state"][:, 0], first["observation.state"])
    assert torch.equal(batch["observation.state"][:, 1], second["observation.state"])
    assert batch["task"] == ["pick blue block"]


def test_live_noise_cache_reuses_prior_then_resets_on_generation_change():
    policy = torch.nn.Linear(1, 1, bias=False)
    policy.config = SimpleNamespace(
        horizon=16,
        action_feature=SimpleNamespace(shape=(7,)),
    )
    batch = {"observation.state": torch.zeros((1, 2, 13))}
    cache = DiffusionLiveNoiseCache(correlation=1.0)

    first, first_regenerated = cache.get(policy, batch, generation=1)
    second, second_regenerated = cache.get(policy, batch, generation=1)
    third, third_regenerated = cache.get(policy, batch, generation=2)

    assert first.shape == (1, 16, 7)
    assert first_regenerated
    assert not second_regenerated
    assert torch.equal(first, second)
    assert third_regenerated
    assert not torch.equal(second, third)


def test_time_aligned_ensemble_uses_three_chunks_and_newest_gripper():
    oldest = _chunk(0.0, steps=9, gripper=0.0)
    previous = _chunk(10.0, steps=9, gripper=0.0)
    newest = _chunk(20.0, steps=9, gripper=1.0)
    for actions in (oldest, previous, newest):
        actions[:, 3:6] = torch.tensor([0.0, 60.0, 0.0])
    history = [
        TimeAlignedDiffusionChunk(0, oldest, oldest),
        TimeAlignedDiffusionChunk(3, previous, previous),
    ]

    original, processed, ensemble_steps, max_contributors = (
        ensemble_time_aligned_diffusion_chunks(
            newest,
            newest,
            start_tick=6,
            history=history,
            weight_decay=0.5,
        )
    )

    expected = (20.0 * 1.0 + 10.0 * 0.5 + 0.0 * 0.25) / 1.75
    assert processed[0, 0].item() == pytest.approx(expected)
    assert original[0, 0].item() == pytest.approx(expected)
    assert processed[0, 6].item() == 1.0
    assert ensemble_steps == 6
    assert max_contributors == 3


def test_queue_temporal_history_is_cleared_on_live_generation_change():
    configure_diffusion_policy_live_queue(True)
    notify_diffusion_policy_live_state(False)
    queue = DiffusionActionQueue(
        RTCConfig(enabled=False),
        max_action_age_sec=10.0,
        ensemble_history_size=3,
    )

    generation = notify_diffusion_policy_live_state(True)
    queue.get_action_index()
    queue.set_pending_chunk_context(
        source_observation_time=time.monotonic(),
        inference_latency_sec=0.17,
    )
    queue.merge(_chunk(0.0), _chunk(0.0), real_delay=6)
    assert queue.metrics()["ensemble_history_entries"] == 1

    notify_diffusion_policy_live_state(False)
    next_generation = notify_diffusion_policy_live_state(True)

    assert next_generation == generation + 1
    assert queue.metrics()["ensemble_history_entries"] == 0
    assert queue.qsize() == 0


def test_processed_overlap_blends_pose_and_holds_discrete_gripper():
    old = _chunk(0.0, steps=2, gripper=0.0)
    old[:, :3] = 0.0
    old[:, 3:6] = torch.tensor([170.0, 20.0, -170.0])
    new = _chunk(30.0, steps=8, gripper=1.0)
    new[:, :3] = 30.0
    new[:, 3:6] = torch.tensor([-170.0, 20.0, 170.0])

    blended = blend_diffusion_processed_overlap(old, new, overlap_steps=2)

    assert blended.shape == (8, 7)
    assert torch.all(blended[:2, :3] > 0.0)
    assert torch.all(blended[:2, :3] < 30.0)
    assert blended[:2, 6].tolist() == [0.0, 0.0]
    assert blended[2:, 6].tolist() == [1.0] * 6
    assert torch.isfinite(blended).all()


def test_latest_chunk_merge_trims_delay_and_blends_matching_future_ticks():
    old_original = _chunk(100.0, steps=3, gripper=0.0)
    old_processed = _chunk(100.0, steps=3, gripper=0.0)
    new_original = _chunk(0.0, gripper=1.0)
    new_processed = _chunk(0.0, gripper=1.0)
    ramp = torch.arange(15, dtype=torch.float32).unsqueeze(1).repeat(1, 3) * 10.0
    new_original[:, :3] = ramp
    new_processed[:, :3] = ramp

    original, processed = merge_diffusion_action_chunks(
        old_original,
        old_processed,
        new_original,
        new_processed,
        overlap_steps=3,
        delay_steps=6,
    )

    assert original.shape == (9, 7)
    assert processed.shape == (9, 7)
    assert torch.all(processed[:3, :3] > new_processed[6:9, :3])
    assert torch.all(processed[:3, :3] < old_processed[:3, :3])
    assert torch.equal(processed[3:, :3], new_processed[9:, :3])
    assert processed[:3, 6].tolist() == [0.0, 0.0, 0.0]
    assert processed[3:, 6].tolist() == [1.0] * 6


def test_delay_expired_chunk_preserves_the_remaining_old_tail():
    queue = DiffusionActionQueue(
        RTCConfig(enabled=False),
        max_action_age_sec=10.0,
    )
    queue.set_pending_chunk_context(
        source_observation_time=time.monotonic(),
        inference_latency_sec=0.17,
    )
    queue.merge(_chunk(0.0), _chunk(0.0), real_delay=6)
    assert queue.qsize() == 9

    assert queue.get() is not None
    assert queue.get() is not None
    queue.set_pending_chunk_context(
        source_observation_time=time.monotonic(),
        inference_latency_sec=0.60,
    )
    queue.merge(_chunk(1.0), _chunk(1.0), real_delay=18)

    assert queue.qsize() == 7
    assert queue.metrics()["delay_expired_chunk_count"] == 1


def test_live_rising_edge_discards_pre_live_inference():
    configure_diffusion_policy_live_queue(True)
    notify_diffusion_policy_live_state(False)
    queue = DiffusionActionQueue(
        RTCConfig(enabled=False),
        max_action_age_sec=10.0,
    )

    queue.get_action_index()
    generation = notify_diffusion_policy_live_state(True)
    queue.set_pending_chunk_context(
        source_observation_time=time.monotonic(),
        inference_latency_sec=0.17,
    )
    queue.merge(_chunk(0.0), _chunk(0.0), real_delay=6)

    assert generation == 1
    assert queue.qsize() == 0
    assert not diffusion_policy_live_queue_ready()
    assert queue.metrics()["stale_inference_drop_count"] == 1

    queue.get_action_index()
    queue.set_pending_chunk_context(
        source_observation_time=time.monotonic(),
        inference_latency_sec=0.17,
    )
    queue.merge(_chunk(1.0), _chunk(1.0), real_delay=6)

    assert queue.qsize() == 9
    assert queue.metrics()["delay_trimmed_action_count"] == 6
    assert diffusion_policy_live_queue_ready()


def test_stale_chunk_is_dropped_instead_of_executed():
    queue = DiffusionActionQueue(
        RTCConfig(enabled=False),
        max_action_age_sec=0.10,
    )
    queue.set_pending_chunk_context(
        source_observation_time=time.monotonic() - 1.0,
        inference_latency_sec=0.17,
    )
    queue.merge(_chunk(0.0), _chunk(0.0), real_delay=6)

    assert queue.get() is None
    assert queue.metrics()["stale_action_drop_count"] == 1
    assert queue.qsize() == 0


def test_diffusion_gripper_filter_detects_windowed_event_and_latches_closed():
    gripper = DiffusionGripperDecisionFilter()
    gripper.reset(0.0)

    assert gripper.update(0.1) == (0.0, False)
    assert gripper.update(0.41) == (0.0, False)
    assert gripper.update(0.26) == (1.0, True)
    assert gripper.update(0.0) == (1.0, False)

    gripper.reset(0.0)
    assert gripper.update(0.0) == (0.0, False)


def test_diffusion_gripper_filter_rejects_an_isolated_peak():
    gripper = DiffusionGripperDecisionFilter()
    gripper.reset(0.0)

    for value in (0.41, 0.1, 0.1, 0.1):
        assert gripper.update(value) == (0.0, False)
    for value in (0.3, 0.3):
        assert gripper.update(value) == (0.0, False)
    assert gripper.update(0.45) == (1.0, True)


def test_diffusion_robot_wraps_only_its_own_gripper_send_action():
    assert DoosanA0509DiffusionRos.send_action is not DoosanA0509Ros.send_action
    assert DoosanA0509Ros.send_action.__qualname__ == "DoosanA0509Ros.send_action"
