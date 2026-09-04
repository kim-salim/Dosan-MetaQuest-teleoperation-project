# Task-C Full Live Validation — 2026-08-14

## Status

**PASS: two consecutive full physical executions completed.**

Both trials executed the complete chain:

`ACT-A → semantic cut → endpoint-stop Bridge → atomic ACT-B handoff → rolling ACT-B → driver-confirmed release → home settle → COMPLETE`.

This validates the current checkpoints, runtime manifest, control parameters,
camera placement, and observed work-cell arrangement used for these runs. It is
not a blanket certification for arbitrary scenes, payloads, poses, or paths.

## Validated configuration

- Control rate: 30 Hz
- Robot: Doosan A0509, real controller
- GPU: NVIDIA Thor
- ACT-A:
  `/home/rvlab/lerobot_models/act_a0509_blue_block_bs16_30k_20260810_181114/030000/pretrained_model`
- ACT-B:
  `/home/rvlab/lerobot_models/act_a0509_blue_block_v2_bs16_30k_20260810_200240/030000/pretrained_model`
- Runtime manifest:
  `/home/rvlab/lerobot_datasets/task_c_bridge_v0_closed_holding_20260813/runtime_transition_manifest.json`
- Bridge mode: endpoint stop, fresh ACT-B inference, exact-pose tail, atomic handoff
- ACT-B mode: fresh-observation rolling inference with 15-step overlap
- Release completion evidence:
  `completed_command=open`, `driver_busy=false`,
  `last_command_ok=true`, open observation stability, final home-envelope
  settle

## Physical trial results

| Metric | Trial 1 | Trial 2 |
|---|---:|---:|
| Artifact directory | `task_c_full_live_20260814_0043` | `task_c_full_live_repeat_20260814_0054` |
| Start position error | 0.956 mm | 7.015 mm |
| Start orientation error | 0.657° | 0.130° |
| Live edge to COMPLETE | 34.371 s | 34.721 s |
| ACT-A commands | 417 | 424 |
| Bridge commands | 163 | 176 |
| ACT-B commands | 410 | 408 |
| ACT-B policy steps | 407 | 406 |
| Rolling ACT-B refreshes | 5 | 5 |
| Refresh queue-hold cycles | 0 | 0 |
| Final settle speed | 4.256 mm/s | 4.148 mm/s |
| Terminal phase | COMPLETE | COMPLETE |

The 15-frame completion threshold and 15 mm/s settle-speed limit were met in
both trials.

## Timing and semantic milestones

| Metric from Live edge | Trial 1 | Trial 2 |
|---|---:|---:|
| ACT-A close latch | 11.678 s | 11.300 s |
| ACT-B open command committed | 27.453 s | 27.891 s |
| ACT-B open driver-confirmed | 28.252 s | 28.763 s |

Trial 1 release pose:

`[434.140, -176.155, 303.781, 0.123, 160.881, 0.133]` mm/deg

Trial 2 release pose:

`[428.169, -169.549, 304.324, 0.129, 160.058, 0.157]` mm/deg

Trial 1 final pose:

`[393.178, -4.962, 472.569, 0.112, 144.085, 0.105]` mm/deg

Trial 2 final pose:

`[389.264, 4.686, 470.530, 0.101, 143.956, 0.110]` mm/deg

## Rolling ACT-B timing

| Metric | Trial 1 | Trial 2 |
|---|---:|---:|
| Request-to-completion min/mean/max | 75.088 / 81.194 / 90.014 ms | 70.310 / 78.880 / 84.736 ms |
| Observation age min/mean/max | 100.165 / 100.714 / 102.197 ms | 100.003 / 108.199 / 135.029 ms |
| Maximum GPU arbiter wait | 0.0032 ms | 0.0025 ms |
| Delay compensation | 2–3 steps | 2–3 steps |
| Overlap | 15 steps | 15 steps |
| Queue exhaustion hold | 0 | 0 |

All observation ages remained below the configured 500 ms rolling-refresh
limit.

## Control and fail-closed evidence

- Gate contract mismatches: 0 in both trials
- Gate event-order errors: 0 in both trials
- Failure or rejected events: 0 in both trials
- Raw-to-safe exact Bridge matches: 193 and 200
- Safe-to-commanded exact Bridge matches: 193 and 200
- Guard interventions: 0 in both trials
- Streamer interventions: 0 in both trials
- Gripper command sequence in each trial: one `stop`, one `close`, one
  `open`
- Driver-confirmed release: true in both trials
- Gripper open latch at completion: true in both trials

Tracking backpressure activated 28 and 29 times. The largest uncommitted
candidate position errors were 16.702 mm and 16.119 mm. Those candidates were
not sent: the runner held the previous full action until the candidate entered
the 14 mm admission limit. Every backpressure event was subsequently released,
and no hard tracking failure occurred.

After each trial, the gate forced:

- MUX source: `DISABLED`
- Live robot output: `false`
- Gripper commanded state: open
- Gripper driver: `busy=false`, `last_command_ok=true`

The policy runner exited and released both cameras. The physical control
bringup remained running with MUX disabled and Live output off.

## Evidence

Trial 1:

- `/home/rvlab/lerobot_datasets/task_c_full_live_20260814_0043/task_c_live_events.jsonl`
- `/home/rvlab/lerobot_datasets/task_c_full_live_20260814_0043/gate_preflight.json`
- `/home/rvlab/lerobot_datasets/task_c_full_live_20260814_0043/gate_live.json`
- `/home/rvlab/lerobot_datasets/task_c_full_live_20260814_0043/before_scene.jpg`
- `/home/rvlab/lerobot_datasets/task_c_full_live_20260814_0043/after_scene.jpg`

Trial 2:

- `/home/rvlab/lerobot_datasets/task_c_full_live_repeat_20260814_0054/task_c_live_events.jsonl`
- `/home/rvlab/lerobot_datasets/task_c_full_live_repeat_20260814_0054/gate_preflight.json`
- `/home/rvlab/lerobot_datasets/task_c_full_live_repeat_20260814_0054/gate_live.json`
- `/home/rvlab/lerobot_datasets/task_c_full_live_repeat_20260814_0054/before_scene.jpg`
- `/home/rvlab/lerobot_datasets/task_c_full_live_repeat_20260814_0054/after_scene.jpg`

The operator visually confirmed the second run as completely successful.

## Interpretation and next gate

The primary objective—natural, functional A/Bridge/B continuity through ACT-B
completion—is achieved for two consecutive physical trials. The next evidence
step is repeated-trial reliability measurement. Two successes establish the
working path but are not enough to estimate a low failure rate or tail
probability.

Recommended next gate:

1. Run 10 reset-and-repeat physical trials with the same scene contract.
2. Require `COMPLETE`, zero intervention, driver-confirmed release, and
   cleanup-confirmed `DISABLED`/Live-off in every trial.
3. Review timing distributions and failure taxonomy before expanding the
   workspace, payload, orientation, or scene variation.
