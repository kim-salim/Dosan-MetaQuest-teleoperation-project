#!/usr/bin/env python3
"""Load both ACT policies and validate real inference without robot commands."""

from __future__ import annotations

import argparse
import json
import statistics
import threading
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

from .lerobot_act_backend import LeRobotACTBackend, load_recorded_observation
from .runtime_policy import AsyncPolicySession, assess_policy_chunk
from .shadow_latency import policy_chunk_timing_record


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint-a", type=Path, required=True)
    parser.add_argument("--checkpoint-b", type=Path, required=True)
    parser.add_argument("--dataset-a", type=Path, required=True)
    parser.add_argument("--dataset-b", type=Path, required=True)
    parser.add_argument("--a-episode", type=int, required=True)
    parser.add_argument("--a-frame", type=int, required=True)
    parser.add_argument("--b-episode", type=int, required=True)
    parser.add_argument("--b-frame", type=int, required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--action-hz", type=float, default=30.0)
    parser.add_argument("--warmup-inferences", type=int, default=2)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--velocity-window-steps", type=int, default=15)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def _latency_summary(values_s: list[float]) -> dict[str, Any]:
    if not values_s:
        return {"count": 0, "milliseconds": []}
    values_ms = [float(value * 1000.0) for value in values_s]
    return {
        "count": len(values_ms),
        "milliseconds": values_ms,
        "mean_ms": statistics.fmean(values_ms),
        "maximum_ms": max(values_ms),
    }


def _assessment_record(value: Any) -> dict[str, Any]:
    return {
        "valid": value.valid,
        "failure_reasons": list(value.failure_reasons),
        "intended_velocity_mm_s": value.intended_velocity_mm_s.tolist(),
        "first_position_jump_mm": value.first_position_jump_mm,
        "max_predicted_velocity_mm_s": value.max_predicted_velocity_mm_s,
        "velocity_window_steps": value.velocity_window_steps,
        "velocity_method": value.velocity_method,
        "single_frame_difference_used": False,
    }


def main() -> None:
    args = _parse_args()
    if args.action_hz <= 0.0:
        raise SystemExit("--action-hz must be positive")
    if args.warmup_inferences < 0:
        raise SystemExit("--warmup-inferences must be non-negative")
    if args.timeout_s <= 0.0:
        raise SystemExit("--timeout-s must be positive")

    import torch

    observation_a = load_recorded_observation(
        args.dataset_a,
        episode=args.a_episode,
        frame=args.a_frame,
        repo_id="local/task_c_act_a_dry_run",
    )
    observation_b = load_recorded_observation(
        args.dataset_b,
        episode=args.b_episode,
        frame=args.b_frame,
        repo_id="local/task_c_act_b_dry_run",
    )

    load_started = time.perf_counter()
    backend_a = LeRobotACTBackend(args.checkpoint_a, device=args.device)
    loaded_a = time.perf_counter()
    backend_b = LeRobotACTBackend(args.checkpoint_b, device=args.device)
    loaded_b = time.perf_counter()

    gpu_lock = threading.Lock()
    session_a = AsyncPolicySession(
        "ACT-A",
        backend_a,
        action_hz=args.action_hz,
        inference_lock=gpu_lock,
    )
    session_b = AsyncPolicySession(
        "ACT-B",
        backend_b,
        action_hz=args.action_hz,
        inference_lock=gpu_lock,
    )
    try:
        warm_a = session_a.warmup(
            observation_a.policy_input,
            inferences=args.warmup_inferences,
        )
        warm_b = session_b.warmup(
            observation_b.policy_input,
            inferences=args.warmup_inferences,
        )
        if session_a.ready_chunk is not None or session_b.ready_chunk is not None:
            raise RuntimeError("warmup output leaked into an executable queue")

        capture_a = time.monotonic()
        generation_a = session_a.prime(
            observation_a.policy_input,
            observation_timestamp_s=capture_a,
        )
        chunk_a = session_a.wait_for_chunk(generation_a, args.timeout_s)
        assessment_a = assess_policy_chunk(
            chunk_a,
            observation_a.tcp_position_mm,
            velocity_window_steps=args.velocity_window_steps,
            velocity_method="linear_regression",
            velocity_epsilon=1e-9,
            first_position_jump_limit_mm=1e9,
            predicted_velocity_limit_mm_s=1e9,
        )

        capture_b = time.monotonic()
        generation_b = session_b.prime(
            observation_b.policy_input,
            observation_timestamp_s=capture_b,
        )
        chunk_b = session_b.wait_for_chunk(generation_b, args.timeout_s)
        assessment_b = assess_policy_chunk(
            chunk_b,
            observation_b.tcp_position_mm,
            velocity_window_steps=args.velocity_window_steps,
            velocity_method="linear_regression",
            velocity_epsilon=1e-9,
            first_position_jump_limit_mm=75.0,
            predicted_velocity_limit_mm_s=300.0,
        )

        # Prove activation and invalidation are session-local.
        session_a.activate(generation_a)
        first_a = session_a.pop_action()
        a_queue_before_clear = session_a.queue_size
        b_ready_before_a_clear = session_b.ready_chunk is not None
        session_a.deactivate_and_clear()
        b_ready_after_a_clear = session_b.ready_chunk is not None
        session_b.activate(generation_b)
        first_b = session_b.pop_action()
        if first_a is None or first_b is None:
            raise RuntimeError("fresh ACT chunk unexpectedly contained no action")
        if not b_ready_before_a_clear or not b_ready_after_a_clear:
            raise RuntimeError("clearing ACT-A mutated ACT-B's separate queue")

        cuda_memory = None
        if args.device.startswith("cuda"):
            device = torch.device(args.device)
            cuda_memory = {
                "device_name": torch.cuda.get_device_name(device),
                "allocated_bytes": int(torch.cuda.memory_allocated(device)),
                "reserved_bytes": int(torch.cuda.memory_reserved(device)),
                "both_models_resident": True,
            }

        report = {
            "status": "dry_run_complete",
            "mode": "dual_act_resident_inference_only",
            "model_load": {
                "ACT-A_seconds": loaded_a - load_started,
                "ACT-B_seconds": loaded_b - loaded_a,
                "total_seconds": loaded_b - load_started,
                "ACT-A_checkpoint": str(backend_a.checkpoint),
                "ACT-B_checkpoint": str(backend_b.checkpoint),
            },
            "observations": {
                "ACT-A": {
                    "dataset_root": observation_a.dataset_root,
                    "episode": observation_a.episode,
                    "frame": observation_a.frame,
                    "global_index": observation_a.global_index,
                    "dataset_timestamp_s": observation_a.timestamp_s,
                    "tcp_position_mm": observation_a.tcp_position_mm.tolist(),
                },
                "ACT-B": {
                    "dataset_root": observation_b.dataset_root,
                    "episode": observation_b.episode,
                    "frame": observation_b.frame,
                    "global_index": observation_b.global_index,
                    "dataset_timestamp_s": observation_b.timestamp_s,
                    "tcp_position_mm": observation_b.tcp_position_mm.tolist(),
                },
            },
            "warmup": {
                "outputs_discarded": True,
                "ACT-A": _latency_summary(warm_a),
                "ACT-B": _latency_summary(warm_b),
            },
            "fresh_chunks": {
                "ACT-A": {
                    "generation": chunk_a.generation,
                    "shape": list(chunk_a.actions.shape),
                    "latency_ms": chunk_a.inference_latency_s * 1000.0,
                    "timing": policy_chunk_timing_record(chunk_a),
                    "first_action": first_a.tolist(),
                    "intent_assessment": _assessment_record(assessment_a),
                },
                "ACT-B": {
                    "generation": chunk_b.generation,
                    "shape": list(chunk_b.actions.shape),
                    "latency_ms": chunk_b.inference_latency_s * 1000.0,
                    "timing": policy_chunk_timing_record(chunk_b),
                    "first_action": first_b.tolist(),
                    "entry_velocity_assessment": _assessment_record(assessment_b),
                },
            },
            "queue_isolation": {
                "ACT-A_queue_after_one_pop_before_clear": a_queue_before_clear,
                "ACT-B_ready_before_ACT-A_clear": b_ready_before_a_clear,
                "ACT-B_ready_after_ACT-A_clear": b_ready_after_a_clear,
                "shared_temporal_ensemble": False,
                "shared_action_queue": False,
                "shared_gpu_lock_only": True,
            },
            "session_stats": {
                "ACT-A": asdict(session_a.stats()),
                "ACT-B": asdict(session_b.stats()),
            },
            "cuda_memory": cuda_memory,
            "velocity_contract": {
                "runtime_a_boundary": "measured_tcp_history_not_policy_target",
                "runtime_b_boundary": "fresh_postprocessed_ACT-B_chunk",
                "offline_b_velocity": "planning_seed_and_fallback_only",
            },
            "orientation_status": "pending",
            "orientation_bridge_generated": False,
            "collision_status": "NOT_CHECKED",
            "ik_status": "NOT_CHECKED",
            "robot_executable": False,
            "dry_run_only": True,
            "robot_commands_published": False,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(report, indent=2, sort_keys=True))
    finally:
        session_a.close()
        session_b.close()


if __name__ == "__main__":
    main()
