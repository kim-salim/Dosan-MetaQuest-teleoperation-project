# Task-C representative Bridge V1 — 2026-08-18

## Outcome

The existing exhaustive V0 implementation and its two successful physical
validation records were preserved. A separate V1 implementation now builds one
phase-aligned representative trajectory for A and one for B, optimizes the
shortest complete C path with a 30 Hz coarse / 60 Hz final pipeline, and exposes
an opt-in 40/20 mm live boundary.

The original V1 artifact generation was command-free. Subsequent bounded
physical trials validated the representative A boundary and initial Bridge,
then failed closed before ACT-B. The endpoint-boundary update described below
was first regression-tested command-free and then completed one bounded
physical A -> Bridge -> B trial on 2026-08-19.

## Inputs

- A: `a0509_blue_block_v1_20260810_170226`, 30 episodes, 30 Hz
- B: `a0509_blue_block_v2_20260810_191044`, 30 episodes, 30 Hz
- config:
  `offline_tools/task_c_bridge_v1/config_representative_closed_holding.json`
- final output:
  `/home/rvlab/lerobot_datasets/task_c_bridge_v1_representative_20260818_v2`

The generated `checksums.sha256` covers every core artifact.

## Search result

| Metric | Result |
|---|---:|
| Representative A/B source episodes | 30 / 30 |
| Phase samples per representative | 101 |
| Coarse evaluations | 5,733 |
| Refined evaluations | 4,160 |
| Feasible search candidates | 1,806 |
| 60 Hz final/robust validations | 84 / 84 |
| Standardization time | 2.252 s |
| Optimization time | 3.783 s |
| Total before serialization | 6.036 s |

Selected contract:

| Field | Value |
|---|---|
| A phase | 0.02 |
| A boundary center, mm | [435.753, 204.728, 415.287] |
| A representative velocity, mm/s | [-7.845, -16.700, 88.029] |
| B phase | 0.94 |
| B entry, mm | [414.468, -156.555, 418.762] |
| Bridge duration | 3.5 s |
| Bridge length | 385.317 mm |
| Estimated total C length | 1501.716 mm |
| Maximum speed | 152.231 mm/s |
| Maximum acceleration | 186.297 mm/s² |
| Robust residual pass fraction | 43/60 = 71.7% |
| 40 mm training support | 26/30 = 86.7% |
| 20 mm training support | 15/30 = 50.0% |

The runtime B payload floor is the B-episode p95 value, 409.693 mm. Offline
representative optimization still checks the more conservative
max(A-max, B-p95) floor. This split matches V0 semantics: offline has both
candidate endpoint floors, while live starts from an already observed A state
and carries the selected B-entry floor.

## Prior-success no-command replay

The two 2026-08-14 successful physical `act_a_to_bridge` events were replayed
through the V1 boundary and pure runtime planner. This is compatibility
evidence, not a new physical V1 success claim.

| Prior run | Boundary distance | Direction cosine | Ack Z | V1 plan | Duration | Planner latency |
|---|---:|---:|---:|---|---:|---:|
| `task_c_full_live_20260814_0043` | 16.521 mm | 0.9919 | 410.245 mm | feasible | 3.5 s | 3.331 ms |
| `task_c_full_live_repeat_20260814_0054` | 17.792 mm | 0.9992 | 412.091 mm | feasible | 3.5 s | 2.922 ms |

Both observations are inside the 20 mm commit sphere and exceed the 0.7
direction-cosine requirement. Each runtime plan evaluated 13 durations and
preserved the endpoint-stop contract.

## Regression evidence

- Task-C V0/V1 offline/runtime/shadow/live-gate suite: 109 passed
- A0509 plugin suite, including live runner invariants: 127 passed
- Runtime coordinator suite alone: 36 passed
- Live rollout suite alone: 25 passed
- `colcon build --packages-select lerobot_robot_doosan_a0509 --symlink-install`:
  passed
- Generated artifact checksum verification: all files OK
- New live pre-plan source test confirms no send, pause, reset, or queue mutation
  occurs at the 40 mm boundary.

## Bounded physical evidence and endpoint-boundary update

The bounded run
`task_c_representative_v1_direct_physical_20260818_2300` produced the following
physical evidence before its fail-closed result:

- ACT-A grasp and close latch succeeded;
- representative prearm occurred at 33.716 mm with direction cosine 0.9981;
- commit occurred at 11.061 mm with direction cosine 0.9996;
- 393 ACT-A and 124 Bridge commands were sent;
- all 248 guard/stream comparisons were exact, with zero intervention;
- the stopped Bridge endpoint error was 0.032 mm;
- fresh ACT-B inference was valid, but zero ACT-B commands were sent;
- Live returned false and the MUX returned to `DISABLED`.

That run used the conservative endpoint-only transition. Its fresh ACT-B first
target differed from the stopped endpoint by
`[-3.036, -2.612, -6.366] mm`. The Euclidean jump was 7.521 mm, but the actual
ServoL contract is per Cartesian axis and its maximum axis step was only
6.366 mm against the unchanged 6.67 mm/tick limit. The coordinator's direct
fallback therefore produced a false rejection by using an L2 metric where the
downstream owner uses an L-infinity metric.

The later bounded run
`task_c_representative_v1_moving_overlap_physical_20260818_2353` provided the
decisive endpoint evidence:

- ACT-A grasp and representative-boundary commit succeeded;
- 426 ACT-A and 138 Bridge commands were sent, and zero ACT-B commands;
- all 212 guard and 212 streamer comparisons were exact, with zero
  intervention;
- the robot settled at `[414.496, -156.587, 418.766] mm` with zero TCP
  velocity, Live false, and MUX `DISABLED` after fail-closed;
- the planning-only mid-Bridge B seed was outside the representative B entry
  and all suffixes failed `act_b_entry_position_inconsistent`;
- endpoint fresh ACT-B inference was valid, with maximum predicted velocity
  83.354 mm/s;
- its first target delta was `[-3.100, -3.935, -7.512] mm`, so the maximum
  axis step 7.512 mm exceeded the unchanged 6.67 mm/tick direct limit by
  0.842 mm.

The B checkpoint was already CUDA resident and warmed. Warmup outputs remained
discarded; only a new generation from the settled endpoint cameras, joints,
TCP, and gripper observation was considered executable.

Command-free replay of that exact endpoint and target found a zero-to-zero
alignment Bridge that passes every existing limit. The shortest feasible
duration is 0.6 s:

| Metric | 0.6 s alignment result |
|---|---:|
| Length | 9.029 mm |
| Maximum vector speed | 22.555 mm/s |
| Maximum axis speed | 18.766 mm/s |
| Maximum acceleration | 150.478 mm/s² |
| Maximum jerk | 501.595 mm/s³ |
| Maximum curvature | approximately 0 /mm |
| Maximum 30 Hz per-axis step | 0.592 mm/tick (limit 6.67) |
| Minimum TCP Z | 411.250 mm |
| Runtime payload floor | 409.693 mm |

No workspace, payload-floor, ramp, velocity, acceleration, jerk, curvature, or
backtracking threshold was increased.

Representative V1 now disables the mid-Bridge seed and uses the settled B
boundary as its primary transition point. V0 and the generic live runner retain
all new flags as false by default. The V1 transition is:

```text
40 mm A prearm -> 20 mm A commit
  -> complete the validated zero-terminal Bridge to representative B entry
  -> hold until actual/acknowledged position and causal speed settle
  -> request one fresh ACT-B execution_refresh from the endpoint observation
  -> if a velocity-matched connector is feasible, use it
  -> else if every XYZ axis fits 6.67 mm, use stopped direct handoff
  -> else plan the shortest feasible zero-to-zero alignment Bridge
  -> validate full XYZ/orientation/workspace/ramp/dynamic contract
  -> reach the exact ACT-B first target and atomically activate its queue
```

The endpoint execution generation is never a warmup output and is not inferred
from a mid-Bridge observation. If its chunk is stale/invalid, if direct entry
exceeds the per-axis ramp, or if no alignment Bridge passes all existing
constraints, the coordinator remains fail-closed.

The stopped direct branch uses
`max(abs(delta_xyz)) <= 6.67 mm`, exactly matching the streamer's per-axis ramp.
Euclidean jump remains recorded as a diagnostic. Full-pose, orientation,
workspace, tracking, vector-speed, per-axis-speed, generation, freshness, and
gripper-holding checks are unchanged. Alignment duration selection is an
explicit `shortest_feasible` opt-in used only by this stopped endpoint fallback;
the original near-shortest/smoothest selection remains unchanged elsewhere.

## 2026-08-19 endpoint-boundary physical validation: PASS

The bounded run
`task_c_representative_v1_endpoint_alignment_physical_20260819_003139`
completed the full semantic task. The run used the endpoint-boundary wrapper
with mid-Bridge `tail_seed` disabled, fresh endpoint ACT-B inference enabled,
stopped direct enabled, and the zero-to-zero position Bridge fallback enabled.

- The external preflight passed with zero ROS contract mismatches. The Task-A
  start-envelope error was 14.083 mm and 2.442 degrees.
- ACT-A prearmed the representative boundary at 31.891 mm and committed at
  14.618 mm with direction cosine 0.9916.
- The initial 3.5 s Bridge started from the acknowledged command and measured
  causal TCP velocity, retained a zero terminal velocity, and passed the full
  downstream stream assessment before the ACT-A queue was cleared.
- The endpoint actual error was 0.054 mm. After a 0.268 s hold, causal measured
  speed was 13.023 mm/s and one `execution_refresh` was requested from the
  settled endpoint observation.
- The velocity-matched tail was infeasible under the existing dynamics checks.
  The fresh first target delta was `[-1.963, -3.242, -3.055] mm`; its maximum
  axis step was 3.242 mm, below the unchanged 6.67 mm/tick direct limit.
- Full-pose direct validation passed: first orientation step 0.525 degrees
  against the 1.0 degree/tick limit, maximum predicted Cartesian speed
  88.571 mm/s, and no workspace or tracking-admission failure. The handoff was
  atomic with `generation_role=execution_refresh` and no skipped B actions.
- Commands by phase were ACT-A 368, Bridge 133, and ACT-B 372. The gate observed
  207 exact raw-to-safe Bridge matches and 208 exact safe-to-commanded matches,
  with zero guard and zero streamer interventions.
- ACT-B authorized release before the open command, the gripper driver confirmed
  `completed_command=open`, `busy=false`, and `last_command_ok=true`, and the
  task completed after 15 stable home frames. Four rolling B refreshes completed
  without a hold cycle.
- Terminal state was `COMPLETE`, with no coordinator failure reason. Gate cleanup
  restored Live OFF and MUX `DISABLED`; the final observed TCP velocity was zero
  and the gripper commanded state was open.

This sample exercised the stopped-direct branch, not the zero-to-zero alignment
fallback, because the naturally inferred first target already fit one streamer
tick. The exact prior `7.512 mm > 6.67 mm` endpoint sample remains command-free
evidence for the 0.6 s alignment branch; that fallback has not yet been forced
or separately claimed as a physical execution success.

## Activation and remaining gate

V1 remains disabled by default. It requires a V1 manifest and
`--strategy.representative_boundary_enabled=true`; a V1 manifest is rejected
without that explicit flag, and a V0 manifest cannot enable the V1 boundary.

The installed-module preflight and one operator-authorized bounded physical
rerun have now passed. V1 still remains explicit opt-in, and the alignment
fallback should be observed only when a natural fresh endpoint chunk exceeds
the direct per-axis limit; it should not be forced by relaxing or bypassing a
safety threshold. Orientation, IK, and collision-with-payload are not newly
certified by this artifact, and the manifest intentionally remains
`robot_executable=false`, `dry_run_only=true`.
