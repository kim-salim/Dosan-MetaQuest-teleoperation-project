# A0509 MetaQuest accepted teleoperation baseline — 2026-08-07

This document freezes the parameters used by the physical arm-and-gripper
configuration accepted after the final one-minute tests. It is the reference
for LeRobot teacher recording. It does not enable robot motion by itself.

## Canonical parameter snapshot

The immutable snapshot is:

```text
src/quest_a0509_teleop/config/a0509_metaquest_accepted_2026-08-07.yaml
SHA-256: 46dc5d022db4221009a13848b0b514d4b366aecae72158c0db362815ae02866e
```

At freeze time this checksum matched both the editable source configuration
`xyz_position_only.yaml` and its installed package copy. For a new experiment,
copy this file to a new name rather than editing the accepted snapshot.

## Accepted motion mapping

| Item | Accepted value |
|---|---|
| Position scale | Quest 1.0 m -> robot 850 mm on X/Y/Z (`scale_xyz=0.85`) |
| Position axes | robot X <- Quest Y, robot Y <- -Quest X, robot Z <- Quest Z |
| Orientation | only robot Ry enabled, scale `0.7`; existing tested sign retained |
| Orientation representation | Doosan `posx` orientation; not generic RPY |
| Mapper rate | 30 Hz |
| Position filter | receive-time resampling plus fixed-tick low-pass |
| Interpolation delay | 50 ms |
| Prediction | full to 20 ms, decay to 80 ms, then hold |
| Tracking invalid | 300 ms |
| Raw input timeout | 500 ms |
| Position jump guard | 200 mm |
| Rotation jump guard | 45 deg |

Pose resampling remains enabled with a one-second, 128-sample bounded buffer.
The 72 Hz reference and `alpha=0.4` preserve the tested filter time constant
when filtering on the 30 Hz control tick.

## Accepted safety and stream settings

| Item | Accepted value |
|---|---|
| Workspace minimum | X=50 mm, Y=-350 mm, Z minimum disabled |
| Workspace maximum | X=650 mm, Y=350 mm, Z=600 mm |
| Orientation delta limit | 90 deg on each Doosan orientation component |
| Safety output rate | 30 Hz |
| ServoL output rate | 30 Hz |
| ServoL time | 0.1 s |
| Velocity/acceleration conditions | automatic, all fields `DR_COND_NONE=-10000` |
| Position ramp | 6.67 mm/tick, approximately 200 mm/s at 30 Hz |
| Rotation ramp | 1.0 deg/tick, 30 deg/s at 30 Hz |
| Robot state stale timeout | 1.0 s |
| Selected-source heartbeat timeout | 1.0 s |
| Streamer executor | 4 workers with 1 ms bounded yield |
| Doosan controller update rate | 100 Hz |

The safety guard owns workspace/orientation clamping. The Streamer owns the
final rate limit, robot-state gate, MUX-heartbeat gate, Live gate, and ServoL
publication. The source MUX always starts `DISABLED`.

## Accepted preparation and calibration

- Preparation joint pose: `[0, 0, 90, 0, 60, 0]` deg.
- Preparation velocity/acceleration: `30 deg/s`, `30 deg/s^2`.
- Preparation completion updates the TCP anchor and recenters the Quest anchor.
- XY +X calibration is mandatory for the MetaQuest source.
- Calibration window: two seconds; minimum accepted displacement: 40 mm.
- Calibration, anchor, source selection, and Live state are session state and
  are deliberately not persisted in this snapshot.
- Each new physical session must prepare, recenter/calibrate, confirm VALID,
  select METAQUEST, and explicitly enable Live.

## Accepted MUX and gripper settings

| Item | Accepted value |
|---|---|
| Initial source | `DISABLED` |
| MetaQuest timeout | 1.0 s |
| LeRobot timeout | 0.3 s |
| MetaQuest calibration required | true |
| A / `button_lower` | close |
| B / `button_upper` | open |
| Quest input watchdog | 0.3 s |
| Physical Tool DO | DO1=open, DO2=close |
| Driver mode | pulse |
| Tool DO pulse | 0.50 s |
| Interlock | 0.05 s |
| Driver debounce | 0.30 s |
| Readback poll / timeout | 0.01 s / 0.50 s |
| MetaQuest minimum motion pulse at MUX | 1.50 s |

Open/close is accepted only when the selected source, preparation,
calibration, fresh arm target, and Live gates all permit output. Live OFF emits
stop and rejects later open/close requests.

## Reproducible physical launch

Run this only after the physical workcell safety check. It starts with the MUX
disabled and Live false; the operator still performs preparation and calibration.

```bash
cd ~/Dosan-MetaQuest-teleoperation-project
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch quest_a0509_teleop \
  a0509_full_bringup_with_gripper.launch.py \
  config_file:="$PWD/src/quest_a0509_teleop/config/a0509_metaquest_accepted_2026-08-07.yaml" \
  start_robot_bringup:=true \
  start_endpoint:=true \
  start_gui:=false \
  start_calibration_gui:=true \
  start_rviz:=false \
  dry_run:=false \
  start_gripper:=true \
  use_command_mux:=true \
  initial_control_source:=DISABLED \
  metaquest_timeout_sec:=1.0 \
  mux_heartbeat_timeout_sec:=1.0 \
  metaquest_gripper_min_pulse_sec:=1.50 \
  gripper_watchdog_timeout_sec:=0.3 \
  gripper_close_do_index:=2 \
  gripper_open_do_index:=1 \
  gripper_command_mode:=pulse \
  gripper_pulse_sec:=0.50 \
  host:=192.168.137.100 \
  rt_host:=192.168.137.100 \
  port:=12345 \
  model:=a0509 \
  name:=dsr01 \
  controller_name:=dsr_controller2 \
  update_rate:=100
```

After the launch is fully started, isolate the control process tree on CPUs
0-5 and raise all existing ROS/DDS threads to normal priority (`nice 0`):

```bash
./scripts/apply_a0509_control_scheduling.sh
```

The command validates that exactly one physical A0509 full bringup is running
and opens one administrator authentication dialog. It does not enable Live or
change the MUX source.

For software-only validation, use the same command with `dry_run:=true`,
`start_robot_bringup:=false`, and `start_gripper:=false`.

## Freeze verification

```bash
sha256sum -c <(printf '%s  %s\n' \
  46dc5d022db4221009a13848b0b514d4b366aecae72158c0db362815ae02866e \
  src/quest_a0509_teleop/config/a0509_metaquest_accepted_2026-08-07.yaml)
```

At freeze time the running Mapper, Safety Guard, Streamer, MUX, preparation,
Quest A/B mapper, and Tool I/O driver parameter dumps matched this document.
No ROS node was restarted and no physical output was enabled while creating the
snapshot.

## LeRobot recording contract

Teacher recording must use this physical baseline with the robot adapter in
`shadow_record`. MetaQuest remains the selected execution source. The recorded
teacher action is the mapper-accepted 7D target pose plus gripper target. The
robot observation contains the 13D joint/TCP/gripper state and three RGB images:

- `front`: C920, 640x480 at 30 FPS
- `side`: C920 serial `947C90BF`, 640x480 at 30 FPS
- `zed_rgb`: the left 672x376 RGB half of the ZED2 1344x376 UVC stereo frame,
  retained at 30 FPS

ZED2 depth, disparity, point clouds, and the right RGB view are intentionally
not recorded. `shadow_record` must never publish an additional robot command.

## Pilot recording entry point

After preparation and GUI calibration report VALID, run:

```bash
cd ~/Dosan-MetaQuest-teleoperation-project
./scripts/record_lerobot_shadow_pilot.sh
```

The script verifies this baseline checksum and runs a guarded shadow preflight
before creating a dataset directory. With the accepted default
`EPISODE_INITIAL_GRIPPER_STATE=open`, the preflight first establishes a
driver-verified physical OPEN state; existing verified OPEN is reused, otherwise
a fresh OPEN must be accepted, completed, and confirmed by Tool DO readback.
Its remaining defaults are one 30-second episode, a 30 FPS dataset target,
real-time H.264 NVENC streaming with preset 12 (`p1`), no PNG image-writer
processes or threads, no display, no sound, and no Hub upload.
The default dataset root is a new timestamped directory below
`~/lerobot_datasets/`. At a natural episode time limit, robot output is forced
safe before an operator review: `n`/Right commits and advances, `r`/Left clears
and repeats the same index, and `q`/Esc commits and ends the session. Keys used
during recording keep their existing early-next, re-record, and stop behavior.
An accepted buffer is committed before the next physical reset starts, so
cancelling a reset cannot lose the previous episode.

The recorder main process and camera threads are confined to CPUs 6-9. A single
`spawn`-started encoder worker is confined to CPUs 10-13. Both run at
`nice 10`; the robot control tree remains on CPUs 0-5 at `nice 0`. If the
desktop session starts at a lower priority, one administrator authentication
dialog raises the recorder to `nice 10`.

The recorder uses 32 owned shared-memory ring slots per camera (about 83 MB for
the accepted three-camera shapes). The worker acknowledges a slot only after
the stock LeRobot encoder has copied it. A full ring, command-queue saturation,
worker failure, frame-count mismatch, or any inner-encoder drop aborts the
episode instead of silently producing misaligned video and Parquet rows.

Diagnostic overrides are `LEROBOT_RECORD_CPUSET`, `LEROBOT_ENCODER_CPUSET`, and
`ENCODER_SHM_SLOTS`. Set `MULTIPROCESS_ENCODER=false` to return to the stock
same-process threaded encoder without changing the rest of the record command.

The record loop also uses an absolute-deadline 30 Hz scheduler by default.
Unlike relative sleep, each tick is timed against the original monotonic clock
epoch, so occasional oversleep does not accumulate as episode-long drift. If a
complete period is missed, the scheduler skips that deadline and permits at
most one immediate catch-up tick; it never emits an unbounded burst. Set
`ABSOLUTE_DEADLINE_SCHEDULER=false` to restore the stock LeRobot relative-sleep
behavior for diagnosis.

NVENC startup can temporarily delay Python camera and ROS callback threads.
For the first 30 successful observations only, camera frames may be up to
1000 ms old and joint/TCP samples up to 1.0 s old. The adapter then
automatically returns to the normal 500 ms camera and 0.5 s state limits.

The stock same-process Live-OFF 10-second validation on 2026-08-08 completed
with 269 synchronized rows (26.9 FPS). Process-isolated encoding raised the
equivalent result to 292 synchronized rows (29.2 FPS). Enabling the absolute
deadline scheduler then produced 299 synchronized rows (29.9 FPS): all three
H.264 videos contained exactly 299 frames and reported 30/1 FPS with a
9.966667-second encoded duration. Front and side remained 640x480 and
`zed_rgb` remained 672x376.

The final run submitted and processed 299 frames per camera with zero drops.
Peak shared-memory occupancy was 18, 19, and 19 slots out of 32. Scheduler
metrics reported 299 ticks, four late ticks, one missed deadline, 46.326 ms
maximum lateness, and 8.626741 seconds of actual sleep. The worker reported
`spawn`, CPUs 10-13, and `nice 10`, then exited without leaving a worker process
or shared-memory object. The plugin regression result was 27 passed.

This result localizes the former steady 0.8 FPS loss to relative-sleep drift.
The remaining single frame in this run corresponds to one measured missed
deadline during the initial recording transient, rather than encoder backlog or
silent frame loss. One-off tests with a 1 ms Python thread switch interval and
a smaller streaming-statistics sample were slower and were not retained.

The first physical Live pilot with this configuration was completed on
2026-08-08. One 35-second episode contained exactly 1050 synchronized rows and
1050 frames in each of the three videos. The scheduler reported zero late ticks
and zero missed deadlines; every encoder stream submitted and processed 1050
frames with zero drops. MetaQuest controlled the robot for a timed 30-second
window inside the episode. Recorded TCP ranges were 232.482 mm in X, 151.022 mm
in Y, and 326.195 mm in Z. Both teacher gripper targets and robot commanded
gripper states contained 0 and 1, confirming open/close recording. The run
ended with Live false, the MUX source DISABLED, no encoder worker, and no
remaining shared-memory object. The local validation dataset is
`~/lerobot_datasets/a0509_metaquest_live_pilot_20260808_210041`.

Override the initial task when the exact object and target are known:

```bash
TASK_DESCRIPTION="Pick up the red block and place it in the tray." \
DATASET_REPO_ID="local/a0509_red_block_pilot" \
./scripts/record_lerobot_shadow_pilot.sh
```

The preflight fails before creating any data if the robot observation, any
configured camera, calibration, accepted mapper target, heartbeat, teleop-ready
state, or gripper state is unavailable.

## Gripper mapping revalidation — 2026-08-08

The accepted mapping was revalidated after an apparent no-motion incident:

```text
Quest A / button_lower = CLOSE = Tool DO2
Quest B / button_upper = OPEN  = Tool DO1
active / inactive      = 1 / 0
driver pulse           = 0.50 s after matching ON readback
```

The incident was not a mapping change. A same-direction command at an already
reached finger limit produced no visible motion, and one diagnostic
`ros2 topic pub --once` discovered a temporary rosbag subscriber before the
driver. A state-aware direct signal sequence then confirmed DO2 CLOSE from a
known open state and DO1 OPEN from a known closed state. No accepted YAML value
or checksum changed.

The authoritative recovery procedure and the distinction between Tool DO
readback and physical finger feedback are recorded in
`docs/jrt_gripper_mapping_revalidation_2026-08-08.md`. Read that file before
changing DO indices or diagnosing an A/B no-motion report.
