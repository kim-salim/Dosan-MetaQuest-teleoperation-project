# Cross-task handoff V2 offline library

This directory builds Task-C boundary-condition diversity at offline/episode
frequency. It never publishes ROS commands and never searches candidates in the
30 Hz control loop.

## Pipeline

```text
LeRobot dataset + semantic_graph_v3.json
  -> build_episode_phase_index
  -> enumerate_handoff_candidates (hard validation included)
  -> validate_handoff_candidates (optional independent re-check)
  -> select_diverse_handoffs (normalized farthest-point sampling)
  -> one immutable episode manifest per selected handoff
```

Geometric/dynamic hard rejection always precedes diversity selection. Semantic
authority is explicit and travels with the library, candidate, selected manifest,
shadow report, and live trace:

- `runtime_guarded` (default): semantic/gripper/held-object/contact/precondition
  mismatches are hard rejection conditions, preserving the reviewed V2 behavior.
- `external_planner`: those mismatches and successor support-radius deviations are
  retained as diagnostics, while the high-level planner owns their authorization.

Both modes still hard-reject non-finite state, workspace or transport-floor
violations, infeasible cubic-Bezier dynamics, downstream per-tick XYZ/orientation
ramp violations, stale generations, incompatible fresh ACT-B motion prefixes, ACK
or tracking errors, and retain fail-closed handling. `external_planner` also holds
the actual discrete gripper state through Bridge/crossfade, then delegates later
gripper events to the active ACT policy and downstream MUX hysteresis.

The repository has no trusted robot IK or environment collision checker, so every
generated record explicitly says:

```text
ik_checked=false
collision_checked=false
robot_executable=false
dry_run_only=true
```

The runtime requires an exact selected episode manifest. It does not choose or
cluster handoffs during command execution. A manifest must also contain
`validation.hard_filter_passed=true`; a generated dry-run manifest is not an
implicit authorization for physical execution.

## Environment

Run the tools as Python modules from the repository root. Directly invoking a
`.py` path is unsupported because the tools import the existing offline and ROS
packages.

```bash
cd ~/Dosan-MetaQuest-teleoperation-project
source /opt/ros/jazzy/setup.bash
source ~/venvs/lerobot/bin/activate
source install/setup.bash

export PYTHONPATH="$PWD/src/quest_a0509_teleop:$PWD/src/lerobot_robot_doosan_a0509:$PWD:${PYTHONPATH:-}"
```

## 1. Build phase indices

Build one index for the source task and one for the successor task. Each index
contains episode/segment/phase, pose, centered offline velocity, gripper,
semantic state, held-object state, contact mode, and symbolic preconditions.

```bash
python -m offline_tools.cross_task_handoff.build_episode_phase_index \
  --dataset-root "$SOURCE_DATASET" \
  --semantic-graph "$SOURCE_SEMANTIC_GRAPH" \
  --task-id "$SOURCE_TASK" \
  --phase-step 0.02 \
  --velocity-window-frames 5 \
  --output "$WORK_DIR/source_phase_index.json"

python -m offline_tools.cross_task_handoff.build_episode_phase_index \
  --dataset-root "$SUCCESSOR_DATASET" \
  --semantic-graph "$SUCCESSOR_SEMANTIC_GRAPH" \
  --task-id "$SUCCESSOR_TASK" \
  --phase-step 0.02 \
  --velocity-window-frames 5 \
  --output "$WORK_DIR/successor_phase_index.json"
```

The centered offline velocity is only a planning feature. Runtime Bridge
generation always estimates a causal velocity from the latest actual TCP
history.

## 2. Enumerate and hard-filter candidates

Only explicitly selected semantic segments are paired. Candidate generation is
bounded with `--max-pairs`; it is not performed by the live runtime.

```bash
python -m offline_tools.cross_task_handoff.enumerate_handoff_candidates \
  --source-index "$WORK_DIR/source_phase_index.json" \
  --source-task "$SOURCE_TASK" \
  --source-segment "$SOURCE_SEGMENT" \
  --successor-index "$WORK_DIR/successor_phase_index.json" \
  --successor-task "$SUCCESSOR_TASK" \
  --successor-segment "$SUCCESSOR_SEGMENT" \
  --duration-s 2.8 \
  --validation-config offline_tools/cross_task_handoff/validation_config_a0509_v2.json \
  --transport-floor-mm "$TRANSPORT_FLOOR_MM" \
  --max-pairs 50000 \
  --semantic-authority external_planner \
  --composition-id "$COMPOSITION_ID" \
  --output "$WORK_DIR/candidate_library.json"
```

An independent validation pass can be run before selection:

```bash
python -m offline_tools.cross_task_handoff.validate_handoff_candidates \
  --input "$WORK_DIR/candidate_library.json" \
  --config offline_tools/cross_task_handoff/validation_config_a0509_v2.json \
  --semantic-authority external_planner \
  --output "$WORK_DIR/candidate_library_revalidated.json"
```

The validator never silently drops a mismatch. In `runtime_guarded` it becomes
a rejection reason. In `external_planner`, the same item is preserved under
`semantic_diagnostics`, while geometric/dynamic violations remain rejection
reasons.

## 3. Select diverse episode handoffs

Selection uses normalized farthest-point sampling only after hard filtering.
This clustering is a reproducible diversity mechanism, not a safety check.

```bash
python -m offline_tools.cross_task_handoff.select_diverse_handoffs \
  --library "$WORK_DIR/candidate_library_revalidated.json" \
  --count 30 \
  --output-dir "$WORK_DIR/episode_manifests" \
  --support-radius-mm 40 \
  --support-orientation-radius-deg 8 \
  --prearm-radius-mm 50 \
  --commit-radius-mm 25 \
  --direction-cosine-minimum 0.25
```

One immutable manifest is selected before each Task-C episode. It contains the
source/successor support episode and phase, semantic boundary state, nominal
pose/velocity, Bridge duration, and validation provenance.

## 4. Command-free ACT-B policy shadow

This test reads an observation from the recorded successor dataset, executes
real ACT-B inference asynchronously, advances a virtual Bridge at 30 Hz, checks
the returned prefix, and evaluates the crossfade. It publishes no ROS command.

```bash
python -m offline_tools.cross_task_handoff.run_v2_policy_shadow \
  --episode-manifest "$EPISODE_MANIFEST" \
  --checkpoint-b "$CHECKPOINT_B" \
  --dataset-b "$SUCCESSOR_DATASET" \
  --validation-config offline_tools/cross_task_handoff/validation_config_a0509_v2.json \
  --handoff-window-steps 24 \
  --prefix-steps 15 \
  --crossfade-steps 15 \
  --max-result-age-s 0.30 \
  --output "$WORK_DIR/v2_policy_shadow.json"
```

Expected report invariant:

```text
robot_commands_published = 0
```

Passing this shadow test does not establish IK feasibility, collision safety,
object retention, or physical robot success.
