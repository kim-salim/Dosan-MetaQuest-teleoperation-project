#!/usr/bin/env python3
"""Command-free timing benchmark for preloaded Multi-V2 phase supervisors."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "src/lerobot_robot_doosan_a0509"
for value in (str(REPOSITORY_ROOT), str(PACKAGE_ROOT)):
    if value not in sys.path:
        sys.path.insert(0, value)

from lerobot_robot_doosan_a0509.task_c_handoff.multi_stage import (  # noqa: E402
    MultiStagePlan,
)
from lerobot_robot_doosan_a0509.task_c_handoff.stage_supervisor import (  # noqa: E402
    GripperEventMode,
    StagePhaseSupervisor,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--iterations", type=int, default=10000)
    return parser.parse_args()


def _summary(values: list[float]) -> dict[str, float | int]:
    array = np.asarray(values, dtype=np.float64)
    return {
        "samples": int(array.size),
        "p50_ms": float(np.percentile(array, 50)),
        "p95_ms": float(np.percentile(array, 95)),
        "p99_ms": float(np.percentile(array, 99)),
        "max_ms": float(np.max(array)),
    }


def _semantic_updates(
    supervisor: StagePhaseSupervisor,
    position: np.ndarray,
) -> None:
    mode = supervisor.config.gripper_event_mode
    if mode is GripperEventMode.CLOSED_THEN_OPEN:
        supervisor.update(
            tcp_position_mm=position,
            tcp_velocity_mm_s=np.zeros(3),
            gripper_closed=True,
        )
        closed = False
    elif mode is GripperEventMode.OPEN_THEN_CLOSED:
        supervisor.update(
            tcp_position_mm=position,
            tcp_velocity_mm_s=np.zeros(3),
            gripper_closed=False,
        )
        closed = True
    else:
        closed = supervisor.boundary.semantic.gripper_state == "closed"
    supervisor.update(
        tcp_position_mm=position,
        tcp_velocity_mm_s=np.zeros(3),
        gripper_closed=closed,
    )


def run(plan: MultiStagePlan, *, iterations: int) -> dict[str, object]:
    if iterations < 100:
        raise ValueError("iterations must be at least 100")
    stages: list[dict[str, object]] = []
    for index, stage in enumerate(plan.stages[:-1]):
        if stage.phase_supervisor is None:
            continue
        transition = plan.transitions[index]
        supervisor = StagePhaseSupervisor(
            transition.handoff_manifest.source,
            stage.phase_supervisor,
        )
        bank = supervisor.tracker.bank
        phase_index = int(
            np.argmin(
                np.abs(
                    bank.phase - supervisor.tracker.config.nominal_phase
                )
            )
        )
        position = bank.episode_xyz_mm[0, phase_index].copy()
        global_ms: list[float] = []
        for _ in range(iterations):
            supervisor.reset()
            _semantic_updates(supervisor, position)
            global_ms.append(
                float(supervisor.tracker.last_status.tracker_latency_ms)
            )
        supervisor.reset()
        _semantic_updates(supervisor, position)
        local_ms: list[float] = []
        closed = transition.handoff_manifest.source.semantic.gripper_state == "closed"
        for _ in range(iterations):
            status = supervisor.update(
                tcp_position_mm=position,
                tcp_velocity_mm_s=np.zeros(3),
                gripper_closed=closed,
            )
            local_ms.append(float(status.tracker_latency_ms))
        stages.append(
            {
                "stage_id": stage.stage_id,
                "policy_id": stage.policy_id,
                "semantic_exit_segment": (
                    transition.handoff_manifest.source.segment
                ),
                "semantic_exit_phase": (
                    transition.handoff_manifest.source.phase
                ),
                "tracking_segment": bank.segment,
                "tracking_nominal_phase": (
                    supervisor.tracker.config.nominal_phase
                ),
                "zero_cost_execution_tail": (
                    stage.phase_supervisor.execution_tail is not None
                ),
                "episode_support_count": int(bank.episode_ids.size),
                "phase_points": int(bank.phase.size),
                "gripper_event": (
                    stage.phase_supervisor.gripper_event_mode.value
                ),
                "global_search": _summary(global_ms),
                "local_search": _summary(local_ms),
            }
        )
    return {
        "schema_version": "a0509.multi_v2_phase_supervisor_benchmark.v1",
        "mode": "command_free_preloaded_numpy_support",
        "plan": str(plan.source_path),
        "iterations_per_mode_per_stage": iterations,
        "robot_commands_published": 0,
        "ros_used": False,
        "gpu_inference_used": False,
        "stages": stages,
    }


def main() -> None:
    args = _parse_args()
    result = run(MultiStagePlan.load(args.plan), iterations=args.iterations)
    output = args.output.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite benchmark: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print("PHASE_SUPERVISOR_BENCHMARK_SCOPE=OFFLINE_COMMAND_FREE")
    for stage in result["stages"]:
        local = stage["local_search"]
        print(
            f"STAGE={stage['stage_id']} local_p99_ms={local['p99_ms']:.6f} "
            f"local_max_ms={local['max_ms']:.6f}"
        )
    print(f"OUTPUT={output}")


if __name__ == "__main__":
    main()
