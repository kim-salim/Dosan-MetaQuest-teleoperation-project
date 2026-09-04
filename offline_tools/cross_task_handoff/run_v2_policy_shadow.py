#!/usr/bin/env python3
"""Command-free V2 Bridge/ACT-B shadow using a recorded successor frame."""

from __future__ import annotations

import argparse
from collections import deque
import json
import threading
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

from lerobot_robot_doosan_a0509.task_c_handoff.async_successor import (
    AsyncSuccessorController,
    TimedACTBackend,
)
from lerobot_robot_doosan_a0509.task_c_handoff.bridge_runtime import (
    BridgeRuntimeLimits,
    BridgeRuntimeSnapshot,
    instantiate_bridge_queue,
)
from lerobot_robot_doosan_a0509.task_c_handoff.compatibility import (
    HandoffCompatibilityEvaluator,
    HandoffSpliceSelector,
)
from lerobot_robot_doosan_a0509.task_c_handoff.coordinator import (
    TaskCHandoffV2Coordinator,
)
from lerobot_robot_doosan_a0509.task_c_handoff.handoff_trace import (
    ControlTimingMonitor,
    json_default,
)
from lerobot_robot_doosan_a0509.task_c_handoff.flexible_bridge import (
    FlexibleBridgeSearchConfig,
    require_flexible_queue,
    search_flexible_bridge_queue,
)
from lerobot_robot_doosan_a0509.task_c_handoff.models import (
    BridgeAdmissionMode,
    EpisodeHandoffManifest,
    HandoffCompatibilityConfig,
    HandoffV2Config,
    HandoffV2State,
)
from offline_tools.cross_task_handoff.validate_handoff_candidates import (
    CandidateValidationConfig,
)
from offline_tools.task_c_bridge_v0.lerobot_act_backend import (
    LeRobotACTBackend,
    load_recorded_observation,
)
from offline_tools.task_c_bridge_v0.runtime_policy import AsyncPolicySession
from quest_a0509_teleop.doosan_orientation import (
    quaternion_to_doosan_zyz_deg,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode-manifest", type=Path, required=True)
    parser.add_argument("--checkpoint-b", type=Path, required=True)
    parser.add_argument("--dataset-b", type=Path, required=True)
    parser.add_argument("--validation-config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--warmup-inferences", type=int, default=2)
    parser.add_argument("--handoff-window-steps", type=int, default=24)
    parser.add_argument("--prefix-steps", type=int, default=15)
    parser.add_argument("--crossfade-steps", type=int, default=15)
    parser.add_argument("--max-result-age-s", type=float, default=0.30)
    parser.add_argument("--max-first-xyz-axis-delta-mm", type=float, default=75.0)
    parser.add_argument("--max-first-rotation-delta-deg", type=float)
    parser.add_argument("--max-prefix-velocity-mm-s", type=float, default=300.0)
    parser.add_argument("--max-prefix-acceleration-mm-s2", type=float)
    parser.add_argument(
        "--max-crossfade-command-acceleration-mm-s2",
        type=float,
        required=True,
    )
    parser.add_argument("--max-crossfade-xyz-axis-step-mm", type=float)
    parser.add_argument("--max-crossfade-rotation-step-deg", type=float)
    parser.add_argument("--max-velocity-mismatch-mm-s", type=float, default=75.0)
    parser.add_argument(
        "--source-orientation-reference-deg",
        type=float,
        nargs=3,
        default=(0.0, 150.0, 0.0),
    )
    parser.add_argument("--max-wall-time-s", type=float, default=15.0)
    parser.add_argument("--adaptive-b-max-splice-index", type=int, default=8)
    parser.add_argument("--adaptive-b-max-candidates", type=int, default=6)
    parser.add_argument("--flexible-bridge-max-candidates", type=int, default=64)
    parser.add_argument("--flexible-bridge-max-search-time-s", type=float, default=0.50)
    return parser.parse_args()


def _bridge_limits(value: CandidateValidationConfig) -> BridgeRuntimeLimits:
    return BridgeRuntimeLimits(
        workspace_min_mm=value.workspace_min_mm,
        workspace_max_mm=value.workspace_max_mm,
        velocity_limit_mm_s=value.velocity_limit_mm_s,
        axis_velocity_limit_mm_s=value.axis_velocity_limit_mm_s,
        acceleration_limit_mm_s2=value.acceleration_limit_mm_s2,
        curvature_limit_per_mm=value.curvature_limit_per_mm,
        jerk_limit_mm_s3=value.jerk_limit_mm_s3,
        integrated_squared_jerk_limit=value.integrated_squared_jerk_limit,
        backtracking_ratio_limit=value.backtracking_ratio_limit,
        linear_ramp_mm_per_tick=value.linear_ramp_mm_per_tick,
        orientation_ramp_deg_per_tick=value.orientation_ramp_deg_per_tick,
        sample_hz=value.sample_hz,
        curvature_epsilon=value.curvature_epsilon,
        ack_pipeline_max_lag_steps=1,
        workspace_min_limit_enabled=(
            value.workspace_min_limit_enabled
        ),
    )


def _event_record(value: Any) -> dict[str, Any]:
    return {
        "timestamp_s": value.timestamp_s,
        "state": value.state,
        "event": value.event,
        "details": value.details,
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite shadow report: {args.output}")
    if args.warmup_inferences < 0 or args.max_wall_time_s <= 0.0:
        raise ValueError("invalid V2 shadow timing arguments")

    manifest = EpisodeHandoffManifest.load(args.episode_manifest)
    validation = CandidateValidationConfig.from_mapping(
        json.loads(args.validation_config.read_text(encoding="utf-8"))
    )
    observation = load_recorded_observation(
        args.dataset_b,
        episode=manifest.successor.support_episode,
        frame=manifest.successor.support_frame,
        repo_id="local/task_c_handoff_v2_policy_shadow",
    )

    backend = TimedACTBackend(
        LeRobotACTBackend(args.checkpoint_b, device=args.device)
    )
    session = AsyncPolicySession(
        "ACT-B-V2-SHADOW",
        backend,
        action_hz=30.0,
        inference_lock=threading.Lock(),
    )
    compatibility = HandoffCompatibilityConfig(
        max_first_xyz_axis_delta_mm=args.max_first_xyz_axis_delta_mm,
        max_first_rotation_delta_deg=(
            args.max_first_rotation_delta_deg
            if args.max_first_rotation_delta_deg is not None
            else validation.orientation_ramp_deg_per_tick * args.crossfade_steps
        ),
        max_prefix_velocity_mm_s=args.max_prefix_velocity_mm_s,
        max_prefix_acceleration_mm_s2=args.max_prefix_acceleration_mm_s2,
        max_bridge_prefix_velocity_mismatch_mm_s=(
            args.max_velocity_mismatch_mm_s
        ),
        max_crossfade_xyz_axis_step_mm=(
            args.max_crossfade_xyz_axis_step_mm
            if args.max_crossfade_xyz_axis_step_mm is not None
            else validation.linear_ramp_mm_per_tick
        ),
        max_crossfade_rotation_step_deg=(
            args.max_crossfade_rotation_step_deg
            if args.max_crossfade_rotation_step_deg is not None
            else validation.orientation_ramp_deg_per_tick
        ),
        max_crossfade_command_acceleration_mm_s2=(
            args.max_crossfade_command_acceleration_mm_s2
        ),
    )
    admission_mode = manifest.bridge_admission_mode
    if admission_mode is BridgeAdmissionMode.FLEXIBLE_LEVEL2:
        max_splice_index = args.adaptive_b_max_splice_index
        max_splice_candidates = args.adaptive_b_max_candidates
    else:
        max_splice_index = 0
        max_splice_candidates = 1
    config = HandoffV2Config(
        enabled=True,
        handoff_window_steps=args.handoff_window_steps,
        b_prefix_steps=args.prefix_steps,
        crossfade_steps=args.crossfade_steps,
        max_b_result_age_sec=args.max_result_age_s,
        max_inflight_b_requests=1,
        request_retry_interval_steps=3,
        enable_soft_handoff=True,
        enable_endpoint_fallback=True,
        control_hz=30.0,
        bridge_admission_mode=admission_mode,
        adaptive_b_max_splice_index=max_splice_index,
        adaptive_b_max_candidates=max_splice_candidates,
        semantic_authority=manifest.semantic_authority,
        compatibility=compatibility,
    )
    successor = AsyncSuccessorController(
        session,
        max_result_age_s=args.max_result_age_s,
    )
    coordinator = TaskCHandoffV2Coordinator(
        manifest=manifest,
        config=config,
        successor=successor,
        evaluator=HandoffCompatibilityEvaluator(
            compatibility,
            prefix_steps=args.prefix_steps,
            action_hz=30.0,
            semantic_authority=manifest.semantic_authority,
            splice_selector=HandoffSpliceSelector(
                max_splice_index=max_splice_index,
                max_candidates=max_splice_candidates,
            ),
        ),
    )

    source_orientation = quaternion_to_doosan_zyz_deg(
        manifest.source.nominal_orientation_quat_xyzw,
        list(args.source_orientation_reference_deg),
    )
    source_pose = np.concatenate(
        (manifest.source.nominal_position_mm, source_orientation)
    )
    snapshot = BridgeRuntimeSnapshot(
        timestamp_s=time.monotonic(),
        actual_pose_mm_deg=source_pose,
        acknowledged_pose_mm_deg=source_pose,
        actual_velocity_mm_s=manifest.source.nominal_velocity_mm_s,
        gripper_target=manifest.source.semantic.gripper_target,
    )
    bridge_limits = _bridge_limits(validation)
    flexible_search = None
    if admission_mode is BridgeAdmissionMode.FLEXIBLE_LEVEL2:
        flexible_search = search_flexible_bridge_queue(
            manifest,
            snapshot,
            config,
            bridge_limits,
            FlexibleBridgeSearchConfig(
                max_candidates=args.flexible_bridge_max_candidates,
                max_search_time_s=args.flexible_bridge_max_search_time_s,
            ),
            live_mode=False,
        )
        bridge = require_flexible_queue(flexible_search)
    else:
        bridge = instantiate_bridge_queue(
            manifest,
            snapshot,
            config,
            bridge_limits,
            clock=time.perf_counter,
        )
    timing = ControlTimingMonitor(control_hz=30.0)
    commands: list[dict[str, Any]] = []
    warmup_latency_s: list[float] = []
    started_s = 0.0
    actual_pose = source_pose.copy()
    command_history: deque[np.ndarray] = deque(
        (source_pose.copy(), source_pose.copy()), maxlen=2
    )

    try:
        warmup_latency_s = session.warmup(
            observation.policy_input,
            inferences=args.warmup_inferences,
        )
        started_s = time.monotonic()
        coordinator.mark_policies_loaded(timestamp_s=time.monotonic())
        coordinator.mark_run_a(timestamp_s=time.monotonic())
        coordinator.mark_exit_commit(timestamp_s=time.monotonic())
        coordinator.mark_prepare_bridge(timestamp_s=time.monotonic())
        coordinator.start_bridge(bridge, timestamp_s=time.monotonic())

        period_s = 1.0 / 30.0
        while time.monotonic() - started_s < args.max_wall_time_s:
            tick_started = time.perf_counter()
            now_s = time.monotonic()
            command = coordinator.tick(
                timestamp_s=now_s,
                actual_pose_mm_deg=actual_pose,
                actual_semantic=manifest.source.semantic,
                policy_input=observation.policy_input,
                command_history_mm_deg=np.stack(tuple(command_history), axis=0),
                observation_timestamp_s=now_s,
            )
            tick_duration_s = time.perf_counter() - tick_started
            timing.add(tick_duration_s)
            if command.action is not None:
                command_history.append(command.action[:6].copy())
                actual_pose = command.action[:6].copy()
                commands.append(
                    {
                        "timestamp_s": now_s,
                        "state": command.state.value,
                        "source": command.source,
                        "bridge_index": command.bridge_index,
                        "bridge_progress": command.bridge_progress,
                        "crossfade_weight": command.crossfade_weight,
                        "generation": command.successor_generation,
                        "action": command.action.tolist(),
                        "tick_duration_ms": tick_duration_s * 1000.0,
                    }
                )
            if coordinator.state in {
                HandoffV2State.RUN_B,
                HandoffV2State.ENDPOINT_FALLBACK,
                HandoffV2State.FAILED_HOLD,
            }:
                break
            remaining = period_s - (time.perf_counter() - tick_started)
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        session.close()

    report = {
        "schema_version": "a0509.task_c_handoff_v2_policy_shadow.v1",
        "mode": "command_free_recorded_successor_observation",
        "robot_commands_published": 0,
        "live_enabled": False,
        "mux_selected": False,
        "handoff_id": manifest.handoff_id,
        "bridge_admission_mode": admission_mode.value,
        "checkpoint_b": str(Path(args.checkpoint_b).expanduser().resolve()),
        "successor_observation": {
            "dataset_root": observation.dataset_root,
            "episode": observation.episode,
            "frame": observation.frame,
            "timestamp_s": observation.timestamp_s,
        },
        "bridge": bridge.record(),
        "flexible_bridge_search": (
            None if flexible_search is None else flexible_search.record()
        ),
        "compatibility_thresholds": asdict(compatibility),
        "warmup_latency_ms": [value * 1000.0 for value in warmup_latency_s],
        "terminal_state": coordinator.state.value,
        "takeover_success": coordinator.state is HandoffV2State.RUN_B,
        "fallback_required": coordinator.state is HandoffV2State.ENDPOINT_FALLBACK,
        "control_timing": asdict(timing.summary()),
        "session_stats": asdict(session.stats()),
        "prefix_admission": (
            None
            if coordinator.last_admission is None
            else coordinator.last_admission.record()
        ),
        "commands": commands,
        "events": [_event_record(event) for event in coordinator.events],
        "limitations": {
            "perfect_tracking_bridge": True,
            "recorded_successor_RGB_state_snapshot": True,
            "environment_collision_checked": False,
            "IK_checked": False,
            "physical_success_claimed": False,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True, default=json_default) + "\n",
        encoding="utf-8",
    )
    return report


def main() -> None:
    report = run(_parse_args())
    print(
        "TASK_C_V2_POLICY_SHADOW_COMPLETE "
        f"state={report['terminal_state']} "
        f"takeover={report['takeover_success']} "
        f"commands_published={report['robot_commands_published']}"
    )


if __name__ == "__main__":
    main()
