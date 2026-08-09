# LeRobot A0509 episode reset workflow — 2026-08-09

## Purpose

Every recorded episode now begins from the same intentional physical state:

- Doosan A0509 at the configured preparation joint pose.
- Robot TCP anchor refreshed from the pose actually reached.
- The existing session-level MetaQuest XY/Yaw calibration remains `VALID`.
- The current Quest controller pose is accepted as the new relative origin.
- JRT gripper is explicitly opened and its driver completion is confirmed.
- MUX has selected `METAQUEST`, but Live remains OFF until recording begins.

The GUI XY/Yaw calibration is not repeated between episodes. It is a
session/tracking-frame calibration. `/vr/recenter` is the per-episode relative
zero operation.

## State sequence

```text
episode recording ends
  -> Live OFF
  -> MUX DISABLED
  -> gripper STOP and driver idle confirmation
  -> operator prompt 1: clear hands and obstacles
  -> /vr/prepare_robot
  -> fresh robot anchor and teleop_ready confirmation
  -> operator prompt 2: reset object, hold Quest controller neutral
  -> Quest raw pose stable for 0.5 seconds
  -> calibration VALID check
  -> /vr/recenter (result checked independently)
  -> fresh valid-pose heartbeat check
  -> gripper OPEN accepted/completed check
  -> MUX METAQUEST
  -> selected target fresh and within 10 mm / 3 deg of anchor
  -> Live ON confirmation
  -> next episode timer and frame recording start
```

The first episode uses this same sequence by default. The arm never starts its
preparation move until the operator confirms prompt 1 with Enter.

## LeRobot integration

The implementation is in
`lerobot_robot_doosan_a0509/episode_reset_orchestrator.py`. The project record
entry point installs it after the existing shared-memory encoder and absolute
deadline scheduler.

LeRobot's stock reset is a fixed-duration `record_loop(dataset=None)`. With
`EPISODE_RESET_ORCHESTRATION=true`, that unrecorded timed loop is replaced by
the guarded state sequence above. `RESET_TIME_S` is still supplied to the
LeRobot configuration for compatibility, but it is not used as the reset gate.

No dataset frames are added during the reset. The completed episode buffer is
left frozen while reset runs, and LeRobot then preserves its normal behavior:

- `n` or Right: accept the current episode early, reset, then continue.
- `r` or Left: stop the current attempt, reset, clear its buffer, and record the
  same episode index again.
- `q` or Esc during recording: stop the session.
- Enter during a reset prompt: confirm only the displayed reset step.
- `q` or Esc during a reset prompt: cancel while the system is in the safe
  Live-OFF/DISABLED state.

## Default settings

```bash
EPISODE_RESET_ORCHESTRATION=true
EPISODE_RESET_BEFORE_FIRST=true
EPISODE_INITIAL_GRIPPER_STATE=open
```

Advanced thresholds may be overridden for diagnosis:

```bash
EPISODE_PREPARE_TIMEOUT_SEC=180
EPISODE_SERVICE_TIMEOUT_SEC=10
EPISODE_STATE_TIMEOUT_SEC=8
EPISODE_QUEST_STABILITY_TIMEOUT_SEC=12
EPISODE_QUEST_STABILITY_WINDOW_SEC=0.5
EPISODE_QUEST_MAX_AGE_SEC=0.3
EPISODE_QUEST_STABLE_TRANSLATION_M=0.008
EPISODE_QUEST_STABLE_ROTATION_DEG=5
EPISODE_PREFLIGHT_POSITION_LIMIT_MM=10
EPISODE_PREFLIGHT_ROTATION_LIMIT_DEG=3
EPISODE_PREFLIGHT_SETTLE_SEC=0.35
```

The initial gripper state can be `open`, `close`, or `none`. `open` is the
accepted data-collection default. The driver must publish a fresh matching
`accepted_command` and `completed_command`; a stale commanded-state latch alone
does not pass the reset.

## Failure behavior

The next episode does not start if any of these checks fail:

- Live OFF or MUX DISABLED cannot be confirmed.
- The gripper cannot reach an idle successful state.
- Preparation motion/service fails or a fresh anchor is not published.
- Quest input is stale or cannot remain stable.
- XY/Yaw calibration is not `VALID`.
- Explicit recenter fails.
- Initial gripper command is not accepted and completed.
- Selected MetaQuest target is stale or does not converge to the anchor.
- Live ON cannot be confirmed.

Recording-loop exceptions also execute the safe transition in `finally`.

## Verification performed

The change was verified without commanding physical hardware:

- Bash syntax check passed for `record_lerobot_shadow_pilot.sh`.
- Python byte compilation passed for the entry point and orchestrator.
- All 37 `lerobot_robot_doosan_a0509` tests passed.
- Preflight tests confirm that episode-reset mode defers `teleop_ready` and
  action freshness until after preparation while still requiring calibration.
- The installed editable plugin loaded and rendered `record_entrypoint --help`.

Physical verification is intentionally still pending. The first real run must
be treated as a supervised reset/one-episode validation before collecting a
multi-episode dataset.
