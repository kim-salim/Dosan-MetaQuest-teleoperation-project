# Task-C V2 asymmetric transition patch

Date: 2026-08-26 (Asia/Seoul)

## Outcome

The existing Task-C V2 runtime remains the only V2 runtime. This patch does
not create a second coordinator and does not change the downstream command
path.

```text
ACT-A
  -> source phase/support admission
  -> actual-state Bridge validation
  -> invalidate ACT-A generation
  -> precomputed Bridge at 30 Hz
  -> handoff window + asynchronous ACT-B inference
  -> fresh ACT-B prefix admission
  -> quintic Bridge/B crossfade
  -> ACT-B rolling refresh
```

The authority contract is intentionally asymmetric:

```text
A phase:
  When may ACT-A be interrupted?

B fresh policy output:
  When may ACT-B take control?
```

`successor.phase` remains in the episode manifest and trace as research
metadata. It is not an ACT-B runtime gate.

## Preserved V2 behavior

The following implementations were not redesigned:

- `TaskCHandoffV2Coordinator`
- ACT-B asynchronous inference worker and generation isolation
- Bridge queue consumption and handoff window
- B outer semantic/support check
- B fresh-prefix velocity/acceleration admission
- quintic position blend and quaternion orientation interpolation
- discrete gripper hold during crossfade
- endpoint V1 fallback
- MUX, Live, guard, ServoL, and `/vr/commanded_posx` acknowledgement
- normal ACT-B rolling refresh

V0/V1 use the same default representative-boundary behavior as before. The
base strategy only gained an overridable A-exit decision hook; its default
implementation is the original 40/20 mm logic.

## New source trigger modes

```text
median_sphere  existing 40/20 mm representative boundary (default)
phase_support new semantic-segment phase plus real-episode support
```

The default remains `median_sphere` for reproducibility. The experiment
wrapper exposes `SOURCE_TRIGGER_MODE=phase_support` as an explicit opt-in.

## Definition of A phase

Phase is not wall-clock time and not whole-episode progress. It is normalized
Cartesian arc length within the source semantic segment selected by the
handoff manifest, for example T2 `S2`.

The tracker loads the selected segment arrays once during setup:

```text
S2_phase
S2_episode_ids
S2_episode_xyz_mm
S2_median_xyz_mm
```

No artifact file is read in the 30 Hz loop.

The first semantically valid query is vectorized over every episode and phase.
After lock, search is limited to the previous phase index plus/minus the
configured local radius. A backward phase tolerance prevents large causal
regressions. Observations before the semantic predecessor completes cannot
seed this phase lock.

## Source commit contract

The primary phase window is:

```text
[clip(phi_A* - half_width), clip(phi_A* + half_width)]
```

With the default `half_width=0.05`, a nominal phase of 0.50 yields
`[0.45, 0.55]`; a nominal phase of 0.02 yields `[0.00, 0.07]`.

The runtime keeps two distinct distance metrics:

```text
distance_to_component_median
distance_to_nearest_actual_support
```

The component median is diagnostic only in `phase_support` mode. Admission
uses the nearest actual demonstration support at the estimated phase.

If no explicit support distance is configured, the threshold is calibrated
from the leave-one-episode-out nearest-support distribution inside the source
phase window. Median residual p95 is not reused as a nearest-support radius.

An A exit becomes eligible only after all of the following persist:

```text
phase_ready
support_ready
semantic_ready
persistence_ready
```

The actual/acknowledged pose, causal actual TCP velocity, semantic state, and
complete Bridge queue are then validated. Only a feasible Bridge causes ACT-A
pause/reset. A `BridgeGenerationError` inside the primary window leaves ACT-A
running so the next actual state may be tried. Missing/stale acknowledgement,
non-finite input, tracking error, or semantic corruption propagates to the
existing fail-closed path.

If the estimated phase passes the configured deadline without a feasible
commit, the runtime fails closed instead of creating a late Bridge.

## ACT-B authority

There is no B phase admission branch. `successor_runtime_phase_gate=true` is
rejected at configuration validation.

ACT-B takeover still requires:

```text
semantic compatibility
reasonable distance/orientation to selected real B support
fresh asynchronous B result
generation and age validity
B prefix dynamic compatibility
prospective crossfade stream compatibility
```

Therefore a B state with an equivalent offline phase far from the nominal
phase may take control when its fresh output is compatible. Conversely, a B
state near the nominal metadata phase cannot take control when its fresh
prefix is incompatible.

## Runtime options

The V2 strategy adds these flat Draccus-compatible options:

```text
source_trigger_mode
source_phase_artifact_path
source_phase_half_width
source_phase_prearm_extra
source_phase_persistence_ticks
source_phase_local_search_radius_indices
source_phase_backward_tolerance
source_phase_deadline_extra
source_support_distance_threshold_mm
source_support_loo_quantile
successor_runtime_phase_gate   # must remain false
```

The wrapper maps the same settings from environment variables prefixed with
`SOURCE_`.

## T2 -> T3 offline replay result

Input trace:

```text
/home/rvlab/lerobot_datasets/
  task_c_t2_t3_v2_full_20260825_h_d4/task_c_v2_trace.jsonl
```

Source support:

```text
docs/artifacts/t2_semantic_only_graph_v3_2026-08-23/
  standardized_semantic_phase_trajectories.npz
segment = S2
episodes = 31
phase points = 101
```

Results using nominal phase `0.50 +/- 0.05` and three-tick persistence:

```text
auto LOO-p95 support threshold: 34.151 mm
commit ACT-A command index:     455
elapsed time:                   15.244 s
estimated phase:                0.50
component median distance:      26.942 mm
nearest actual support:          8.093 mm
nearest support episode:        23
```

This is the intended repaired case: the legacy 20 mm representative sphere
does not admit it, while the actual episode support is close and the source
phase is valid.

## Timing result

On the local machine with the real T2 S2 support bank:

```text
artifact setup load: 2.494 ms (outside control loop)

saved-trace tracker latency:
  p50 0.086 ms
  p95 0.091 ms
  p99 0.109 ms
  max 0.289 ms

10,000 local-search updates:
  p50 0.028 ms
  p95 0.029 ms
  p99 0.031 ms
  max 0.089 ms
```

These are phase-tracker microbenchmarks, not physical robot end-to-end timing.
The existing V2 trace continues to report control-loop and ACT-B inference
timing separately.

## Test result

Command-free regression suites completed on 2026-08-26:

```text
new source/B-contract subset: 32 passed
Task-C V0/V1/V2 + ACT async:  80 passed
```

The tests cover clipped phase windows, alternate real support, support OOD,
phase-window rejection, semantic persistence reset, pre-semantic lock
isolation, leave-one-out calibration, Bridge infeasibility, B phase
non-authority, B prefix incompatibility, and all existing async/crossfade/
fallback behavior.

## Not validated by this patch

No Live ON, MUX LEROBOT selection, Tool DO, or ServoL physical command was
executed. The following remain physical-validation items:

- actual object retention
- collision safety and IK (the current manifest explicitly marks them
  unchecked)
- physical Bridge feasibility for each selected boundary
- end-to-end control tick p95/p99 during camera and CUDA load
- physical acceleration reduction at crossfade
- full Task-C success rate

