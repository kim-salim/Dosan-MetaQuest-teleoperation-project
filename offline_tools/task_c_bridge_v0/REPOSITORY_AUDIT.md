# Repository Audit for Task-C V0

Audit date: 2026-08-13 (Asia/Seoul).

Repository:

```text
path: /home/rvlab/Dosan-MetaQuest-teleoperation-project
branch: main
HEAD: fe9d75d0308df78f4656eedfda2e6b723ae732e6
```

The working tree was already dirty. Unrelated tracked and untracked work was
preserved. Implementation changes in this task are confined to
`offline_tools/task_c_bridge_v0`; generated artifacts are under
`/home/rvlab/lerobot_datasets/task_c_bridge_v0_closed_holding_20260813`.

## 1. ACT-A / ACT-B policy execution

Project entry:

```text
src/lerobot_robot_doosan_a0509/
  lerobot_robot_doosan_a0509/rollout_entrypoint.py
```

It installs the project ACT compatibility patch and invokes installed LeRobot
`lerobot_rollout`. The local untracked
`lerobot_robot_doosan_a0509/act_async_rollout.py` patches ACT chunk inference
for the existing single-policy live path.

Verified checkpoints:

```text
ACT-A:
/home/rvlab/lerobot_models/
act_a0509_blue_block_bs16_30k_20260810_181114/
030000/pretrained_model

ACT-B:
/home/rvlab/lerobot_models/
act_a0509_blue_block_v2_bs16_30k_20260810_200240/
030000/pretrained_model
```

Both configs are ACT/ResNet-18, FP32, `n_obs_steps=1`, 13-D state, three
cameras, 7-D action, `chunk_size=n_action_steps=100`, and
`temporal_ensemble_coeff=null`.

## 2. Inference loop, queue, and temporal state

Installed LeRobot 0.6 `BaseStrategy` runs at the rollout control rate. The
main loop captures an observation, notifies `RTCInferenceEngine`, receives a
background `predict_action_chunk` result, merges it into `ActionQueue`, pops
an action, and calls the robot send path.

The project `act_async_rollout.py` adds:

- asynchronous non-RTC FIFO behavior;
- configurable CUDA warmup with discarded outputs;
- inference-delay alignment;
- 15-step overlap blending;
- orientation SLERP only for overlap of existing policy chunks;
- discrete gripper hold;
- a live-generation queue invalidation/refill mechanism.

That implementation has one global live-policy generation and is not an A/B
switcher. The Task-C runtime therefore creates separate
`AsyncPolicySession` objects, generations, resets, ready chunks, activation
states, and queue indices for A and B. Only a CUDA inference mutex is shared.
Checkpoint temporal ensembling is disabled; the two policy action queues are
also never shared.

## 3. Dataset episode/frame/timestamp contract

A dataset:

```text
root: /home/rvlab/lerobot_datasets/a0509_blue_block_v1_20260810_170226
task: Pick up the blue block on the table
episodes: 30
frames: 31,500
frames per episode: exactly 1,050
episode frame_index: 0 ... 1,049
episode timestamp: 0 ... 34.966667 s
nominal FPS from metadata: 30
```

B dataset:

```text
root: /home/rvlab/lerobot_datasets/a0509_blue_block_v2_20260810_191044
task: Pick up the blue block under the table
episodes: 30
frames: 31,494
frames per episode: 1,044 or 1,050
nominal FPS from metadata: 30
```

The LeRobot v3 Parquet identity fields are `episode_index`, `frame_index`,
`timestamp`, `index`, and `task_index`. No FPS, frame layout, or field name
is inferred when metadata supplies it.

## 4. State/action fields and units

Exact `observation.state` names from `meta/info.json`:

```text
[0:6]   joint_1_rad ... joint_6_rad
[6:9]   tcp_x_mm, tcp_y_mm, tcp_z_mm
[9:12]  tcp_o1_deg, tcp_o2_deg, tcp_o3_deg
[12]    gripper_commanded_state
```

Exact `action` is 7-D absolute Cartesian target XYZ mm, O1/O2/O3 degrees, and
gripper command. TCP XYZ units are millimetres. Dataset gripper values are
observed as binary 0/1 with one close and one open transition per episode.
No non-finite state/action values were found in the audited tables.

Quaternion is not present as a named quaternion field in this dataset; the
orientation payload is Doosan O1/O2/O3 degrees. Whatever orientation payload
is loaded is copied and preserved. V0 never reads it for semantic selection,
bridge generation, feasibility, or ranking.

## 5. Semantic/subgoal/holding/contact availability

The raw schema has no explicit subgoal, boundary, holding, contact, or object
state label. Current closed-transport windows are derived from gripper-close to
the first following gripper-open transition, with configured offsets/stride and
a payload-clearance heuristic.

Within those windows, `holding=true` and
`contact_mode=free_transport_assumed` are explicit assumptions, not measured
labels. Missing object state remains unchecked. If future datasets provide real
holding/contact/object/subgoal labels, semantic compatibility already treats
known mismatches as hard filters.

Low-speed local minima are not used anywhere in Task-C candidate generation.
An exhaustive repository search did not find the previously documented
`visualize_subgoals.py` in the current checkout. That prior audit statement
was incorrect. No existing low-speed implementation was deleted.

## 6. Existing trajectory analysis and command routing

Current trajectory analysis is
`offline_tools/task_c_bridge_v0`: strict dataset load, semantic window
generation, local/representative velocity estimation, exact cubic Bezier
derivatives, metric/feasibility evaluation, exhaustive total-C optimization,
serialization, plots, and tests.

Existing command MUX supports source ownership
`DISABLED/METAQUEST/LEROBOT`; it does not distinguish ACT-A from ACT-B. It
rejects unsafe source changes while live and enforces freshness/live gates.
The Doosan adapter's live policy path publishes Cartesian targets to
`/control/lerobot/target_posx` and handles gripper separately. Policy dry-run
publishes debug data only.

No Task-C module imports or calls command MUX, Tool I/O, ServoL, or the Doosan
adapter. No launch file or robot command was run during this task.

## 7. Implemented Task-C runtime layer

- `runtime_policy.py`: resident-model warmup, discarded warmup outputs,
  async fresh inference, generation invalidation, stale-result drop, isolated
  A/B queues, multi-step postprocessed action velocity.
- `runtime_bridge.py`: all manifest B entries by all configured durations,
  semantic hard filtering, model-target position consistency, exact endpoint
  velocity override, dynamic/geometric feasibility, total-C ranking.
- `runtime_orchestrator.py`: measured TCP A boundary, initial representative
  B velocity, ACT-B priming during bridge, model-conditioned tail replanning,
  optional final near-entry refresh, atomic handoff, fail-closed no-XYZ.
- `lerobot_act_backend.py`: strict local ACT load and preprocessing without
  ROS/robot imports.
- `run_dual_act_dry_run.py`: two models resident on GPU, two warmups each,
  fresh recorded-observation inference, queue-isolation validation.
- `run_model_conditioned_bridge.py`: actual ACT-B inferred velocity injected
  into a feasible velocity-matched bridge.

Tail replanning retains the already executed Bezier arc length as
`committed_bridge_prefix_length_mm`; its runtime objective is therefore
`A retained + committed bridge prefix + new bridge tail + B retained`.

Verified inference-only run on 2026-08-13:

```text
GPU: NVIDIA Thor
both models resident: true
fresh ACT-A latency: 68.52 ms
fresh ACT-B latency: 63.98 ms
ACT-A model intent: [-9.120, -21.975, 101.181] mm/s
A physical boundary replay: [-22.679, -17.269, 93.102] mm/s
ACT-B model terminal boundary: [-7.010, -24.829, -37.492] mm/s
model-conditioned bridge/total: 366.742 / 1304.921 mm
endpoint position error: 0.0 mm
endpoint velocity error: <= 3.52e-14 mm/s
robot commands published: false
```


## 8. Working-tree snapshot before Task-C edits

Pre-existing tracked changes included three docs, A0509 live-trial/preflight/
recording scripts, multiple Doosan/MetaQuest/episode-reset runtime files,
package metadata/tests, and the accepted MetaQuest YAML.

Pre-existing untracked work included Diffusion training/runtime files and
scripts, ACT/Diffusion policy launch/benchmark files, rollout and scheduling
modules/tests, several `.orig` files, and the original untracked
`offline_tools` tree. None of those unrelated changes was reset, deleted, or
overwritten. No destructive git command was used.

## 9. Safety status

```text
orientation_status = pending
orientation_bridge_generated = false
collision_status = NOT_CHECKED_WITH_PAYLOAD
ik_status = NOT_CHECKED
robot_executable = false
dry_run_only = true
publish_robot_commands = false
```

Workspace bounds are the observed dataset envelope plus a margin, not a
robot-certified workspace. Position-only results are not robot-executable.
