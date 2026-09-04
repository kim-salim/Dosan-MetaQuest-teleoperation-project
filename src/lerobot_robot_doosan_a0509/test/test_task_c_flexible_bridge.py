from __future__ import annotations

import json
import os
import threading
import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("lerobot")

from offline_tools.task_c_bridge_v0.runtime_policy import AsyncPolicySession, PolicyChunk
from offline_tools.task_c_bridge_v0.bezier_bridge import build_velocity_matched_bezier
from offline_tools.cross_task_handoff.evaluate_flexible_level2 import (
    runtime_limits_from_validation,
)
from lerobot_robot_doosan_a0509.task_c_handoff.bridge_runtime import (
    BridgeGenerationError,
    BridgeRuntimeLimits,
    BridgeRuntimeSnapshot,
    instantiate_bridge_queue,
)
from lerobot_robot_doosan_a0509.task_c_handoff.async_successor import (
    AsyncSuccessorController,
)
from lerobot_robot_doosan_a0509.task_c_handoff.compatibility import (
    HandoffCompatibilityEvaluator,
    HandoffSpliceSelector,
)
from lerobot_robot_doosan_a0509.task_c_handoff.flexible_bridge import (
    AsyncFlexibleBridgeResult,
    AsyncFlexibleBridgePlanner,
    FlexibleBridgeTemplate,
    FlexibleBridgeSearchConfig,
    FlexibleBridgeSearchResult,
    _has_cusp,
    _severe_reversal,
    _candidate_specs,
    rebase_cached_flexible_bridge,
    search_flexible_bridge_queue,
    nominal_medoid_bridge_snapshot,
    validate_flexible_entry_rebase,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    BridgeAdmissionMode,
    EpisodeHandoffManifest,
    HandoffCompatibilityConfig,
    HandoffV2Config,
)
from lerobot_robot_doosan_a0509.task_c_handoff.coordinator import (
    TaskCHandoffV2Coordinator,
)
from lerobot_robot_doosan_a0509.task_c_handoff.soft_handoff import (
    build_soft_handoff,
)
from lerobot_robot_doosan_a0509.task_c_live_v2_rollout import (
    TaskCLiveV2Strategy,
)
from quest_a0509_teleop.doosan_orientation import (
    quaternion_to_doosan_zyz_deg,
)


class _CapturedPublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def test_v2_ready_contract_uses_dedicated_latched_publisher():
    strategy = object.__new__(TaskCLiveV2Strategy)
    strategy.phase = SimpleNamespace(value="WAITING_FOR_LIVE")
    strategy._v2_coordinator = None
    strategy._v2_manifest = None
    strategy._v2_trace = None
    strategy._v2_pretrace_events = []
    strategy._event_pub = _CapturedPublisher()
    strategy._ready_pub = _CapturedPublisher()
    strategy._phase_pub = None

    strategy._event(
        "strategy_ready_external_gate_required",
        downstream_contract={"control_hz": 30.0},
    )
    for index in range(32):
        strategy._event("later_setup_event", index=index)

    assert len(strategy._event_pub.messages) == 33
    assert len(strategy._ready_pub.messages) == 1
    ready = json.loads(strategy._ready_pub.messages[0].data)
    assert ready["event"] == "strategy_ready_external_gate_required"
    assert ready["downstream_contract"] == {"control_hz": 30.0}


def _manifest(*, flexible: bool) -> EpisodeHandoffManifest:
    semantic = {
        "gripper_state": "closed",
        "held_object": "blue_block",
        "contact_mode": "free_transport_assumed",
        "entry_preconditions": ["grasp_complete"],
    }
    return EpisodeHandoffManifest.from_mapping(
        {
            "composition_id": "flex_test",
            "handoff_id": "h_flex_test",
            "source": {
                "task": "T7",
                "segment": "acquire_from_drawer_top",
                "phase": 1.0,
                "support_episode": 12,
                "support_frame": 100,
                "nominal_pose_mm_deg": [100, 0, 400, 0, 150, 0],
                "nominal_velocity_mm_s": [3.249, 0, 0],
                "support_radius_mm": 90,
                "semantic": semantic,
            },
            "successor": {
                "task": "T3",
                "segment": "deliver_to_floor",
                "phase": 0.0,
                "support_episode": 22,
                "support_frame": 200,
                "nominal_pose_mm_deg": [150, 10, 410, 1, 150, 0],
                "nominal_velocity_mm_s": [2, 0, 0],
                "support_radius_mm": 90,
                "semantic": semantic,
            },
            "bridge": {
                "duration_s": 2.8,
                "nominal_length_mm": 52,
                "transport_floor_mm": 350,
            },
            "generation": {
                "method": "flexible_reference" if flexible else "corridor_diverse",
                "bridge_admission_mode": (
                    "flexible_level2" if flexible else "strict_level2"
                ),
                "reference_only": flexible,
            },
            "validation": {
                "hard_filter_passed": not flexible,
                "robot_executable": False,
                "dry_run_only": True,
                "ik_checked": False,
                "collision_checked": False,
            },
        }
    )


def _snapshot(*, offset: np.ndarray | None = None) -> BridgeRuntimeSnapshot:
    pose = np.asarray([100, 0, 400, 0, 150, 0], dtype=np.float64)
    if offset is not None:
        pose[:3] += np.asarray(offset, dtype=np.float64)
    return BridgeRuntimeSnapshot(
        timestamp_s=time.monotonic(),
        actual_pose_mm_deg=pose,
        acknowledged_pose_mm_deg=pose,
        actual_velocity_mm_s=np.asarray([3.249, 0, 0], dtype=np.float64),
        gripper_target=1.0,
    )


def _limits() -> BridgeRuntimeLimits:
    return BridgeRuntimeLimits(
        workspace_min_mm=np.asarray([0, -500, 0]),
        workspace_max_mm=np.asarray([700, 500, 700]),
        velocity_limit_mm_s=1000,
        axis_velocity_limit_mm_s=1000,
        acceleration_limit_mm_s2=1000,
        curvature_limit_per_mm=0.25,
        jerk_limit_mm_s3=10000,
        integrated_squared_jerk_limit=1.0e9,
        backtracking_ratio_limit=10,
        linear_ramp_mm_per_tick=20,
        orientation_ramp_deg_per_tick=2,
    )


def _runtime_config() -> HandoffV2Config:
    return HandoffV2Config(
        enabled=True,
        handoff_window_steps=24,
        b_prefix_steps=5,
        crossfade_steps=5,
        bridge_admission_mode=BridgeAdmissionMode.FLEXIBLE_LEVEL2,
        adaptive_b_max_splice_index=5,
        adaptive_b_max_candidates=6,
        compatibility=HandoffCompatibilityConfig(),
    )


def test_nominal_medoid_snapshot_uses_reviewed_manifest_source():
    manifest = _manifest(flexible=True)
    snapshot = nominal_medoid_bridge_snapshot(
        manifest,
        timestamp_s=123.0,
    )
    expected_orientation = quaternion_to_doosan_zyz_deg(
        manifest.source.nominal_orientation_quat_xyzw
    )
    np.testing.assert_allclose(
        snapshot.actual_pose_mm_deg,
        np.concatenate(
            (manifest.source.nominal_position_mm, expected_orientation)
        ),
    )
    np.testing.assert_allclose(
        snapshot.acknowledged_pose_mm_deg,
        snapshot.actual_pose_mm_deg,
    )
    np.testing.assert_allclose(
        snapshot.actual_velocity_mm_s,
        manifest.source.nominal_velocity_mm_s,
    )
    assert snapshot.gripper_target == manifest.source.semantic.gripper_target
    assert snapshot.timestamp_s == pytest.approx(123.0)


def test_setup_nominal_template_bounded_retry_and_cache_restore():
    manifest = _manifest(flexible=True)
    runtime_config = _runtime_config()
    limits = _limits()
    search_config = FlexibleBridgeSearchConfig()
    ready_search = search_flexible_bridge_queue(
        manifest,
        nominal_medoid_bridge_snapshot(
            manifest,
            timestamp_s=time.monotonic(),
        ),
        runtime_config,
        limits,
        search_config,
        live_mode=False,
    )
    assert ready_search.valid

    class Worker:
        def __init__(self):
            self.requests = []
            self.invalidations = 0

        def request_template(
            self,
            _manifest_value,
            snapshot,
            _runtime_config_value,
            _limits_value,
            _search_config_value,
            *,
            live_mode,
        ):
            assert live_mode
            self.requests.append(snapshot)
            return len(self.requests)

        def wait_for_result(self, *, timeout_s):
            assert timeout_s == pytest.approx(1.0)
            generation = len(self.requests)
            if generation < 3:
                return AsyncFlexibleBridgeResult(
                    "rejected",
                    generation,
                    failure_reason="no_command_safe_candidate",
                    request_kind="template_search",
                )
            return AsyncFlexibleBridgeResult(
                "ready",
                generation,
                search=ready_search,
                request_kind="template_search",
            )

        def invalidate(self):
            self.invalidations += 1

    worker = Worker()
    strategy = object.__new__(TaskCLiveV2Strategy)
    strategy.v2_config = SimpleNamespace(
        flexible_bridge_template_max_attempts=3,
        flexible_bridge_template_setup_timeout_s=1.0,
    )
    strategy._v2_runtime_config = runtime_config
    strategy._v2_manifest = manifest
    strategy._flexible_bridge_worker = worker
    strategy._flexible_bridge_search_config = search_config
    strategy._flexible_bridge_nominal_template_cache = {}
    strategy._flexible_bridge_template = None
    strategy._flexible_bridge_template_requested = False
    strategy._flexible_bridge_template_request_attempts = 0
    strategy._flexible_bridge_template_retry_exhausted_logged = False
    strategy._flexible_bridge_next_template_request_s = 0.0
    strategy._flexible_bridge_ready = ready_search
    strategy._flexible_bridge_next_rebase_request_s = 1.0
    strategy._bridge_runtime_limits = lambda: limits
    events = []
    strategy._event = lambda name, **details: events.append((name, details))

    template = strategy._cache_nominal_flexible_bridge_template(
        manifest,
        startup_phase="unit_test_before_live",
    )

    assert len(worker.requests) == 3
    assert template is strategy._flexible_bridge_template
    assert (
        strategy._flexible_bridge_nominal_template_cache[manifest.handoff_id]
        is template
    )
    for snapshot in worker.requests:
        np.testing.assert_allclose(
            snapshot.actual_pose_mm_deg[:3],
            manifest.source.nominal_position_mm,
        )
    assert events[-1][0] == "flexible_bridge_nominal_template_cached"
    assert events[-1][1]["before_live_authorization"] is True

    strategy._flexible_bridge_template = None
    strategy._reset_flexible_bridge_runtime()
    assert strategy._flexible_bridge_template is template
    assert strategy._flexible_bridge_template_requested
    assert strategy._flexible_bridge_ready is None
    assert worker.invalidations == 1



def test_strict_rejects_global_curvature_while_flexible_uses_soft_curvature():
    strict = _manifest(flexible=False)
    strict_config = replace(
        _runtime_config(),
        bridge_admission_mode=BridgeAdmissionMode.STRICT_LEVEL2,
        adaptive_b_max_splice_index=0,
        adaptive_b_max_candidates=1,
    )
    with pytest.raises(BridgeGenerationError, match="curvature_limit"):
        instantiate_bridge_queue(
            strict,
            _snapshot(),
            strict_config,
            _limits(),
            clock=time.perf_counter,
        )

    result = search_flexible_bridge_queue(
        _manifest(flexible=True),
        _snapshot(),
        _runtime_config(),
        _limits(),
        FlexibleBridgeSearchConfig(),
        live_mode=False,
    )
    assert result.valid
    assert result.selected is not None
    assert result.selected.max_curvature_per_mm > _limits().curvature_limit_per_mm
    assert result.selected.max_normal_acceleration_mm_s2 < _limits().acceleration_limit_mm_s2
    assert "curvature_limit" not in result.selected.hard_rejection_reasons


def test_flexible_transport_floor_is_a_hard_constraint():
    manifest = replace(
        _manifest(flexible=True),
        transport_floor_mm=405.0,
    )
    result = search_flexible_bridge_queue(
        manifest,
        _snapshot(),
        _runtime_config(),
        _limits(),
        FlexibleBridgeSearchConfig(),
        live_mode=False,
    )
    assert not result.valid
    assert result.queue is None
    assert result.rejected_reason_counts["transport_floor_violation"] > 0


def test_actual_offset_adaptive_entry_rejoins_reference_middle():
    snapshot = _snapshot(offset=np.asarray([8.0, -5.0, 3.0]))
    result = search_flexible_bridge_queue(
        _manifest(flexible=True),
        snapshot,
        _runtime_config(),
        _limits(),
        FlexibleBridgeSearchConfig(),
        live_mode=False,
    )
    assert result.queue is not None
    queue = result.queue
    np.testing.assert_allclose(queue.bridge.p0, snapshot.acknowledged_pose_mm_deg[:3])
    nominal = _manifest(flexible=True).source.nominal_position_mm
    start_error = np.linalg.norm(queue.bridge.p0 - nominal)
    manifest = _manifest(flexible=True)
    reference = build_velocity_matched_bezier(
        manifest.source.nominal_position_mm,
        manifest.source.nominal_velocity_mm_s,
        manifest.successor.nominal_position_mm,
        manifest.successor.nominal_velocity_mm_s,
        queue.bridge.duration_s,
    )
    reference_mid = reference.position(0.5)
    middle_error = np.linalg.norm(queue.bridge.position(0.5) - reference_mid)
    assert middle_error < start_error
    valid, reasons, _ = validate_flexible_entry_rebase(queue, snapshot, _limits())
    assert valid, reasons


def test_cached_template_rebases_only_fresh_entry_and_preserves_middle_tail():
    manifest = _manifest(flexible=True)
    runtime_config = _runtime_config()
    limits = _limits()
    search_config = FlexibleBridgeSearchConfig()
    initial = _snapshot()
    searched = search_flexible_bridge_queue(
        manifest,
        initial,
        runtime_config,
        limits,
        search_config,
        live_mode=False,
    )
    assert searched.queue is not None
    assert searched.selected is not None
    template = FlexibleBridgeTemplate(
        handoff_id=manifest.handoff_id,
        search=searched,
        prepared_from_snapshot_timestamp_s=initial.timestamp_s,
    )
    latest = _snapshot(offset=np.asarray([0.5, -0.25, 0.1]))
    rebased = rebase_cached_flexible_bridge(
        template,
        manifest,
        latest,
        runtime_config,
        limits,
        search_config,
    )
    assert rebased.queue is not None, rebased.rejected_reason_counts
    assert 1 <= rebased.candidates_evaluated <= (
        search_config.adaptive_entry_max_candidates
    )
    assert rebased.queue.bridge_algorithm == (
        "cached_reference_dynamic_future_join_c2_v2"
    )
    diagnostics = rebased.queue.candidate_diagnostics
    assert diagnostics["reference_projection_fraction"] is not None
    assert diagnostics["future_join_lookahead_fraction"] > 0.0
    assert diagnostics["entry_fraction"] > (
        diagnostics["reference_projection_fraction"]
    )
    np.testing.assert_allclose(
        rebased.queue.bridge.p0,
        latest.acknowledged_pose_mm_deg[:3],
    )
    join_index = int(
        round(rebased.queue.candidate_diagnostics["entry_fraction"] * rebased.queue.steps)
    ) - 1
    np.testing.assert_allclose(
        rebased.queue.actions[join_index + 1 :, :3],
        searched.queue.actions[join_index + 1 :, :3],
        atol=1.0e-9,
    )
    valid, reasons, _ = validate_flexible_entry_rebase(
        rebased.queue,
        latest,
        limits,
    )
    assert valid, reasons


def test_t1_t2_floor400_replays_latest_predictive_source_splice():
    repository_root = Path(__file__).resolve().parents[3]
    manifest = EpisodeHandoffManifest.load(
        repository_root
        / "docs/artifacts/"
        "t1_t8_spatial_floor_level2_semantic_local_all_2026-09-03/"
        "edges/t1_open_white_container__to__t2_acquire_from_floor_floor400/"
        "flexible_reference_manifest.json"
    )
    validation = json.loads(
        (
            repository_root
            / "offline_tools/cross_task_handoff/validation_config_a0509_v2.json"
        ).read_text(encoding="utf-8")
    )
    limits = runtime_limits_from_validation(validation)
    runtime_config = replace(
        _runtime_config(),
        b_prefix_steps=15,
        crossfade_steps=15,
        adaptive_b_max_splice_index=8,
        adaptive_b_max_candidates=6,
    )
    search_config = FlexibleBridgeSearchConfig()
    nominal_snapshot = nominal_medoid_bridge_snapshot(
        manifest,
        timestamp_s=1.0,
    )
    nominal = search_flexible_bridge_queue(
        manifest,
        nominal_snapshot,
        runtime_config,
        limits,
        search_config,
        live_mode=False,
    )
    assert nominal.queue is not None
    template = FlexibleBridgeTemplate(
        handoff_id=manifest.handoff_id,
        search=nominal,
        prepared_from_snapshot_timestamp_s=1.0,
    )
    predicted_pose = np.asarray(
        [
            317.110107421875,
            12.747716903686523,
            483.76324462890625,
            0.1308889389038086,
            146.8383331298828,
            0.22866232693195343,
        ],
        dtype=np.float64,
    )
    predicted_snapshot = BridgeRuntimeSnapshot(
        timestamp_s=2.0,
        actual_pose_mm_deg=predicted_pose,
        acknowledged_pose_mm_deg=predicted_pose,
        actual_velocity_mm_s=np.asarray(
            [6.190887451170255, 3.406711578369069, -43.66744995117486],
            dtype=np.float64,
        ),
        gripper_target=manifest.source.semantic.gripper_target,
    )
    rebased = rebase_cached_flexible_bridge(
        template,
        manifest,
        predicted_snapshot,
        runtime_config,
        limits,
        search_config,
    )
    assert rebased.queue is not None, rebased.rejected_reason_counts
    assert rebased.selected is not None
    assert rebased.candidates_hard_passed == rebased.candidates_evaluated
    assert rebased.selected.actual_reference_start_error_mm == pytest.approx(
        38.445906561855914
    )
    assert rebased.selected.minimum_position_z_mm == pytest.approx(
        430.5010070800781
    )
    assert rebased.queue.transport_floor_mm == pytest.approx(400.0)


def test_t7_t3_s2_reference_replays_recorded_actual_ack_to_future_join():
    # Command-free replay of generation 6 from the 2026-08-31 trace. The old
    # fixed 18% join rejected this snapshot; the bounded future search admits
    # a later join while preserving the cached S2 reference tail.
    repository_root = Path(__file__).resolve().parents[3]
    manifest = EpisodeHandoffManifest.load(
        repository_root
        / "docs/artifacts/t7_to_t3_s2_runtime_reference_2026-08-31"
        / "flexible_reference_manifest.json"
    )
    validation = json.loads(
        (
            repository_root
            / "offline_tools/cross_task_handoff/validation_config_a0509_v2.json"
        ).read_text(encoding="utf-8")
    )
    limits = runtime_limits_from_validation(validation)
    runtime_config = _runtime_config()
    search_config = FlexibleBridgeSearchConfig()
    nominal_orientation = quaternion_to_doosan_zyz_deg(
        manifest.source.nominal_orientation_quat_xyzw,
        [0.0, 150.0, 0.0],
    )
    nominal_pose = np.concatenate(
        (manifest.source.nominal_position_mm, nominal_orientation)
    )
    nominal_snapshot = BridgeRuntimeSnapshot(
        timestamp_s=1.0,
        actual_pose_mm_deg=nominal_pose,
        acknowledged_pose_mm_deg=nominal_pose,
        actual_velocity_mm_s=manifest.source.nominal_velocity_mm_s,
        gripper_target=1.0,
    )
    searched = search_flexible_bridge_queue(
        manifest,
        nominal_snapshot,
        runtime_config,
        limits,
        search_config,
        live_mode=False,
    )
    assert searched.queue is not None
    assert searched.selected is not None
    legacy_join_fraction = searched.selected.entry_fraction
    template = FlexibleBridgeTemplate(
        handoff_id=manifest.handoff_id,
        search=searched,
        prepared_from_snapshot_timestamp_s=1.0,
    )
    recorded_snapshot = BridgeRuntimeSnapshot(
        timestamp_s=2.0,
        actual_pose_mm_deg=np.asarray(
            [
                567.8320922851562,
                -166.2964630126953,
                486.21240234375,
                0.9432923793792725,
                140.42344665527344,
                0.7513822317123413,
            ],
            dtype=np.float64,
        ),
        acknowledged_pose_mm_deg=np.asarray(
            [
                564.2105712890625,
                -164.8671875,
                491.5675048828125,
                0.8208118081092834,
                141.1183624267578,
                0.7654126286506653,
            ],
            dtype=np.float64,
        ),
        actual_velocity_mm_s=np.asarray(
            [-44.02293556187153, 14.243237242382207, 94.19497153936601],
            dtype=np.float64,
        ),
        gripper_target=1.0,
    )
    rebased = rebase_cached_flexible_bridge(
        template,
        manifest,
        recorded_snapshot,
        runtime_config,
        limits,
        search_config,
    )
    assert rebased.queue is not None, rebased.rejected_reason_counts
    assert rebased.selected is not None
    assert rebased.candidates_evaluated > 1
    assert rebased.candidates_hard_passed >= 1
    assert rebased.selected.reference_projection_fraction is not None
    assert rebased.selected.future_join_lookahead_fraction > 0.0
    assert rebased.selected.entry_fraction > (
        rebased.selected.reference_projection_fraction
    )
    assert rebased.selected.entry_fraction > legacy_join_fraction
    assert (
        rebased.selected.max_ack_span_axis_step_mm
        <= limits.linear_ramp_mm_per_tick
    )
    assert rebased.queue.bridge_algorithm == (
        "cached_reference_dynamic_future_join_c2_v2"
    )


def test_cusp_and_severe_reversal_remain_hard_signals():
    config = FlexibleBridgeSearchConfig()
    sampled = {
        "position_mm": np.asarray(
            [[0, 0, 0], [1, 0, 0], [2, 0, 0], [1, 0, 0]],
            dtype=np.float64,
        ),
        "velocity_mm_s": np.asarray(
            [[10, 0, 0], [10, 0, 0], [0, 0, 0], [-10, 0, 0]],
            dtype=np.float64,
        ),
        "speed_mm_s": np.asarray([10, 10, 0, 10], dtype=np.float64),
    }
    assert _has_cusp(sampled, config)
    sampled["velocity_mm_s"][2] = [-10, 0, 0]
    sampled["speed_mm_s"][2] = 10
    assert _severe_reversal(sampled, config)


def test_candidate_feature_flags_control_bounded_generator_groups():
    disabled = FlexibleBridgeSearchConfig(
        enable_reference_deformation=False,
        enable_tangent_regularization=False,
        enable_settle_connector=False,
        enable_alignment_connector=True,
        enable_lift_transport_connector=False,
    )
    specs = list(_candidate_specs(disabled))
    assert specs
    assert {item[0] for item in specs} == {"alignment_reference_connector"}
    assert len(specs) == len(disabled.duration_multipliers)


def test_flexible_still_rejects_actual_command_dynamics_violation():
    tight_limits = replace(_limits(), jerk_limit_mm_s3=1.0)
    result = search_flexible_bridge_queue(
        _manifest(flexible=True),
        _snapshot(),
        _runtime_config(),
        tight_limits,
        FlexibleBridgeSearchConfig(),
        live_mode=False,
    )
    assert not result.valid
    assert result.rejected_reason_counts["command_jerk_limit"] > 0


def test_stale_actual_ack_rebase_is_fail_closed():
    initial = _snapshot()
    result = search_flexible_bridge_queue(
        _manifest(flexible=True),
        initial,
        _runtime_config(),
        _limits(),
        FlexibleBridgeSearchConfig(),
        live_mode=False,
    )
    assert result.queue is not None
    moved = _snapshot(offset=np.asarray([50.0, 0.0, 0.0]))
    valid, reasons, metrics = validate_flexible_entry_rebase(
        result.queue, moved, _limits()
    )
    assert not valid
    assert "stale_entry_linear_ramp" in reasons
    assert metrics["max_entry_axis_delta_mm"] > _limits().linear_ramp_mm_per_tick


def test_entry_revalidation_rejects_stale_gripper_command():
    snapshot = _snapshot()
    result = search_flexible_bridge_queue(
        _manifest(flexible=True),
        snapshot,
        _runtime_config(),
        _limits(),
        FlexibleBridgeSearchConfig(),
        live_mode=False,
    )
    assert result.queue is not None

    current = replace(snapshot, gripper_target=0.0)
    valid, reasons, metrics = validate_flexible_entry_rebase(
        result.queue,
        current,
        _limits(),
    )

    assert not valid
    assert "stale_entry_gripper" in reasons
    assert metrics["entry_gripper_delta"] == pytest.approx(1.0)


def test_entry_revalidation_uses_shared_4000_acceleration_bound():
    snapshot = _snapshot()
    result = search_flexible_bridge_queue(
        _manifest(flexible=True),
        snapshot,
        _runtime_config(),
        _limits(),
        FlexibleBridgeSearchConfig(),
        live_mode=False,
    )
    assert result.queue is not None
    queue = result.queue
    dt = queue.bridge.duration_s / float(queue.steps)
    first_velocity = (
        queue.actions[0, :3] - snapshot.acknowledged_pose_mm_deg[:3]
    ) / dt
    current = replace(
        snapshot,
        actual_velocity_mm_s=(
            first_velocity - np.asarray([33.0, 0.0, 0.0], dtype=np.float64)
        ),
    )

    strict_valid, strict_reasons, strict_metrics = validate_flexible_entry_rebase(
        queue, current, replace(_limits(), acceleration_limit_mm_s2=300.0)
    )
    common_valid, common_reasons, common_metrics = validate_flexible_entry_rebase(
        queue, current, replace(_limits(), acceleration_limit_mm_s2=4000.0)
    )

    assert not strict_valid
    assert "stale_entry_acceleration" in strict_reasons
    assert common_valid, common_reasons
    assert strict_metrics["entry_acceleration_mm_s2"] == pytest.approx(
        common_metrics["entry_acceleration_mm_s2"]
    )
    assert 300.0 < common_metrics["entry_acceleration_mm_s2"] < 4000.0


def _compatibility() -> HandoffCompatibilityConfig:
    return HandoffCompatibilityConfig(
        max_first_xyz_axis_delta_mm=2.0,
        max_first_rotation_delta_deg=5.0,
        max_prefix_velocity_mm_s=1000.0,
        max_bridge_prefix_velocity_mismatch_mm_s=1000.0,
        max_crossfade_xyz_axis_step_mm=20.0,
        max_crossfade_rotation_step_deg=5.0,
        max_crossfade_command_acceleration_mm_s2=100000.0,
    )


def test_b0_can_fail_while_later_fresh_bj_is_admitted():
    actions = np.zeros((12, 7), dtype=np.float64)
    actions[:, 3:6] = [0, 150, 0]
    actions[:, 6] = 1.0
    actions[:3, 0] = 30.0
    actions[3:, 0] = np.arange(9, dtype=np.float64) * 0.1
    actions[:, 2] = 400.0
    chunk = PolicyChunk("B", 1, 0.0, 0.01, 0.01, 30.0, actions)
    selector = HandoffSpliceSelector(max_splice_index=5, max_candidates=6)
    evaluator = HandoffCompatibilityEvaluator(
        _compatibility(),
        prefix_steps=5,
        action_hz=30.0,
        splice_selector=selector,
    )
    bridge = np.tile(np.asarray([0, 0, 400, 0, 150, 0, 1.0]), (5, 1))
    history = np.tile(bridge[0, :6], (2, 1))

    admissions = {}
    for index in selector.candidate_indices(chunk, prefix_steps=5):
        plan = build_soft_handoff(
            bridge,
            chunk.actions[index:],
            steps=5,
            held_gripper_target=1.0,
        )
        admissions[index] = evaluator.evaluate_prefix(
            chunk,
            bridge_reference_action=bridge[0],
            bridge_velocity_mm_s=np.zeros(3),
            expected_semantic=_manifest(flexible=True).successor.semantic,
            soft_handoff_plan=plan,
            command_history_mm_deg=history,
            post_crossfade_action=chunk.actions[index + 5],
            splice_index=index,
        )
    assert not admissions[0].valid
    assert admissions[3].valid


def test_bj_cannot_skip_across_gripper_event():
    actions = np.zeros((10, 7), dtype=np.float64)
    actions[:, 2] = 400.0
    actions[:, 3:6] = [0, 150, 0]
    actions[3:, 6] = 1.0
    chunk = PolicyChunk("B", 1, 0.0, 0.01, 0.01, 30.0, actions)
    evaluator = HandoffCompatibilityEvaluator(
        _compatibility(), prefix_steps=5, action_hz=30.0
    )
    admission = evaluator.evaluate_prefix(
        chunk,
        bridge_reference_action=np.asarray([0, 0, 400, 0, 150, 0, 1.0]),
        bridge_velocity_mm_s=np.zeros(3),
        expected_semantic=_manifest(flexible=True).successor.semantic,
        splice_index=3,
    )
    assert not admission.gripper_compatible
    assert "prefix_gripper_semantic_mismatch" in admission.failure_reasons


def test_async_bridge_worker_request_and_pending_poll_do_not_block(monkeypatch):
    from lerobot_robot_doosan_a0509.task_c_handoff import flexible_bridge as module

    expected = FlexibleBridgeSearchResult(
        queue=None,
        candidates_evaluated=0,
        candidates_hard_passed=0,
        search_latency_s=0.1,
        selected=None,
        rejected_reason_counts={"test": 1},
        timed_out=False,
    )

    def slow_search(*_args, **_kwargs):
        time.sleep(0.10)
        return expected

    monkeypatch.setattr(module, "search_flexible_bridge_queue", slow_search)
    worker = AsyncFlexibleBridgePlanner(max_result_age_s=1.0)
    started = time.perf_counter()
    generation = worker.request(
        _manifest(flexible=True),
        _snapshot(),
        _runtime_config(),
        _limits(),
        FlexibleBridgeSearchConfig(),
        live_mode=False,
    )
    request_latency = time.perf_counter() - started
    assert generation == 1
    assert request_latency < 0.05
    assert worker.poll(now_s=time.monotonic()).status == "pending"
    time.sleep(0.12)
    assert worker.poll(now_s=time.monotonic()).status == "rejected"
    worker.close()


def test_async_bridge_worker_exception_and_stale_result_fail_closed(monkeypatch):
    from lerobot_robot_doosan_a0509.task_c_handoff import flexible_bridge as module

    def failed_search(*_args, **_kwargs):
        raise RuntimeError("synthetic worker failure")

    monkeypatch.setattr(module, "search_flexible_bridge_queue", failed_search)
    failed_worker = AsyncFlexibleBridgePlanner(max_result_age_s=1.0)
    failed_worker.request(
        _manifest(flexible=True),
        _snapshot(),
        _runtime_config(),
        _limits(),
        FlexibleBridgeSearchConfig(),
        live_mode=False,
    )
    deadline = time.monotonic() + 1.0
    failed = failed_worker.poll(now_s=time.monotonic())
    while failed.status == "pending" and time.monotonic() < deadline:
        time.sleep(0.002)
        failed = failed_worker.poll(now_s=time.monotonic())
    assert failed.status == "failed"
    assert "synthetic worker failure" in str(failed.failure_reason)
    failed_worker.close()

    rejected = FlexibleBridgeSearchResult(
        queue=None,
        candidates_evaluated=1,
        candidates_hard_passed=0,
        search_latency_s=0.02,
        selected=None,
        rejected_reason_counts={"synthetic": 1},
        timed_out=False,
    )

    def slow_search(*_args, **_kwargs):
        time.sleep(0.03)
        return rejected

    monkeypatch.setattr(module, "search_flexible_bridge_queue", slow_search)
    stale_worker = AsyncFlexibleBridgePlanner(max_result_age_s=0.01)
    stale_worker.request(
        _manifest(flexible=True),
        _snapshot(),
        _runtime_config(),
        _limits(),
        FlexibleBridgeSearchConfig(),
        live_mode=False,
    )
    time.sleep(0.04)
    stale = stale_worker.poll(now_s=time.monotonic())
    assert stale.status == "rejected"
    assert stale.stale
    assert stale.failure_reason == "stale_bridge_result"
    stale_worker.close()


def test_cached_template_age_is_not_command_freshness(monkeypatch):
    from lerobot_robot_doosan_a0509.task_c_handoff import flexible_bridge as module

    rejected = FlexibleBridgeSearchResult(
        queue=None,
        candidates_evaluated=1,
        candidates_hard_passed=0,
        search_latency_s=0.03,
        selected=None,
        rejected_reason_counts={"synthetic": 1},
        timed_out=False,
    )

    def slow_search(*_args, **_kwargs):
        time.sleep(0.03)
        return rejected

    monkeypatch.setattr(module, "search_flexible_bridge_queue", slow_search)
    worker = AsyncFlexibleBridgePlanner(max_result_age_s=0.01)
    worker.request_template(
        _manifest(flexible=True),
        _snapshot(),
        _runtime_config(),
        _limits(),
        FlexibleBridgeSearchConfig(),
        live_mode=False,
    )
    time.sleep(0.04)
    result = worker.poll(now_s=time.monotonic())
    assert result.status == "rejected"
    assert not result.stale
    assert result.request_kind == "template_search"
    assert result.failure_reason == "no_command_safe_candidate"
    worker.close()


@pytest.mark.skipif(not hasattr(os, "sched_getaffinity"), reason="Linux affinity required")
def test_process_bridge_worker_isolated_to_requested_cpu():
    requested_cpu = max(os.sched_getaffinity(0))
    expected_nice = max(os.getpriority(os.PRIO_PROCESS, 0), 10)
    worker = AsyncFlexibleBridgePlanner(
        max_result_age_s=1.0,
        backend="process",
        cpu_set=(requested_cpu,),
        nice_value=10,
    )
    try:
        info = worker.start(timeout_s=15.0)
        assert info["backend"] == "process"
        assert info["pid"] != os.getpid()
        assert info["cpu_set"] == [requested_cpu]
        assert info["nice"] == expected_nice
        manifest = _manifest(flexible=True)
        snapshot = _snapshot()
        generation = worker.request_template(
            manifest,
            snapshot,
            _runtime_config(),
            _limits(),
            FlexibleBridgeSearchConfig(),
            live_mode=False,
        )
        assert generation is not None
        deadline = time.monotonic() + 5.0
        template_result = worker.poll(now_s=time.monotonic())
        while template_result.status == "pending" and time.monotonic() < deadline:
            time.sleep(0.002)
            template_result = worker.poll(now_s=time.monotonic())
        assert template_result.status == "ready", template_result.failure_reason
        assert template_result.search is not None
        template = FlexibleBridgeTemplate(
            handoff_id=manifest.handoff_id,
            search=template_result.search,
            prepared_from_snapshot_timestamp_s=snapshot.timestamp_s,
        )
        latest = _snapshot(offset=np.asarray([0.25, 0.0, 0.0]))
        assert worker.request_rebase(
            template,
            manifest,
            latest,
            _runtime_config(),
            _limits(),
            FlexibleBridgeSearchConfig(),
        ) is not None
        deadline = time.monotonic() + 5.0
        rebase_result = worker.poll(now_s=time.monotonic())
        while rebase_result.status == "pending" and time.monotonic() < deadline:
            time.sleep(0.002)
            rebase_result = worker.poll(now_s=time.monotonic())
        assert rebase_result.status == "ready", rebase_result.failure_reason
        assert rebase_result.search is not None
        assert 1 <= rebase_result.search.candidates_evaluated <= (
            FlexibleBridgeSearchConfig().adaptive_entry_max_candidates
        )
        assert rebase_result.search.selected is not None
        assert (
            rebase_result.search.selected.future_join_lookahead_fraction
            > 0.0
        )
    finally:
        worker.close()


@pytest.mark.skipif(not hasattr(os, "sched_getaffinity"), reason="Linux affinity required")
def test_process_bridge_worker_uses_cpu_outside_warmup_thread_affinity():
    thread_id = threading.get_native_id()
    original = set(os.sched_getaffinity(thread_id))
    if len(original) < 2:
        pytest.skip("test requires at least two available CPUs")
    if {9, 10, 11, 12, 13}.issubset(original):
        inherited_cpus = {9, 10, 11, 12}
        requested_cpu = 13
    else:
        inherited_cpus = {min(original)}
        requested_cpu = max(original)
    worker = None
    os.sched_setaffinity(thread_id, inherited_cpus)
    try:
        worker = AsyncFlexibleBridgePlanner(
            max_result_age_s=1.0,
            backend="process",
            cpu_set=(requested_cpu,),
            nice_value=10,
        )
        info = worker.start(timeout_s=15.0)
        assert info["cpu_set"] == [requested_cpu]
        assert set(os.sched_getaffinity(thread_id)) == inherited_cpus
    finally:
        if worker is not None:
            worker.close()
        os.sched_setaffinity(thread_id, original)


class _AdaptivePrefixBackend:
    def reset(self) -> None:
        pass

    def infer(self, _observation):
        actions = np.tile(
            np.asarray([150, 10, 410, 1, 150, 0, 1.0]),
            (100, 1),
        )
        actions[:3, 0] = 300.0
        return actions


def test_coordinator_selects_bj_and_activates_after_splice_plus_crossfade():
    manifest = _manifest(flexible=True)
    config = replace(
        _runtime_config(),
        compatibility=replace(
            _compatibility(),
            max_first_xyz_axis_delta_mm=25.0,
            max_crossfade_xyz_axis_step_mm=20.0,
        ),
    )
    bridge_result = search_flexible_bridge_queue(
        manifest,
        _snapshot(),
        config,
        _limits(),
        FlexibleBridgeSearchConfig(),
        live_mode=False,
    )
    assert bridge_result.queue is not None
    session = AsyncPolicySession("B", _AdaptivePrefixBackend(), action_hz=30.0)
    successor = AsyncSuccessorController(session, max_result_age_s=10.0)
    selector = HandoffSpliceSelector(max_splice_index=5, max_candidates=6)
    evaluator = HandoffCompatibilityEvaluator(
        config.compatibility,
        prefix_steps=config.b_prefix_steps,
        action_hz=30.0,
        splice_selector=selector,
    )
    coordinator = TaskCHandoffV2Coordinator(
        manifest=manifest,
        config=config,
        successor=successor,
        evaluator=evaluator,
    )
    coordinator.start_bridge(bridge_result.queue, timestamp_s=time.monotonic())
    history = [
        _snapshot().acknowledged_pose_mm_deg.copy(),
        _snapshot().acknowledged_pose_mm_deg.copy(),
    ]
    try:
        for _ in range(300):
            now = time.monotonic()
            command = coordinator.tick(
                timestamp_s=now,
                actual_pose_mm_deg=np.asarray([150, 10, 410, 1, 150, 0]),
                command_history_mm_deg=np.stack(history[-2:]),
                actual_semantic=manifest.successor.semantic,
                policy_input={"state": np.zeros(13)},
                observation_timestamp_s=now,
            )
            if command.action is not None:
                history.append(command.action[:6].copy())
            if coordinator.state.value == "RUN_B":
                break
            time.sleep(0.002)
        assert coordinator.state.value == "RUN_B"
        selected_events = [
            item for item in coordinator.events
            if item.event == "act_b_prefix_admission"
            and item.details.get("compatibility") == "PASS"
        ]
        assert selected_events
        assert selected_events[-1].details["selected_splice_index"] == 3
        assert session.queue_size == 100 - 3 - config.crossfade_steps
    finally:
        session.close()
