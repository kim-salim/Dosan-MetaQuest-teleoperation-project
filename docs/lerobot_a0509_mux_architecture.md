# LeRobot ACT + MetaQuest A0509 MUX-first architecture

This document describes the ROS 2 Jazzy implementation used on Jetson Thor. The
current Python nodes, launch files, runtime YAML, and `dsr_controller2` C++ source
are the source of truth. The MUX does not replace the existing coordinate mapper,
robot preparation, safety guard, ServoL ramp, RT interlock, or JRT Tool I/O plan.

## Command graph

```text
/q2r_right_hand_pose
  -> xyz_mapper_node
  |-> /control/metaquest/valid_pose_heartbeat -> MUX freshness
  |-> /vr/metaquest_calibration/valid -> MUX + LeRobot teacher gate
  -> /control/metaquest/target_posx -------------------                                                         A0509 Command MUX
/control/lerobot/target_posx --------------------------/       |
                                                                v
                                                        /vr/target_posx
                                                                |
                                                        safety_guard_node
                                                                v
                                                        /vr/safe_posx
                                                                |
                                                 servol_rt_streamer_node
                                                                v
                                                  /dsr01/dsr_controller2/servol_rt_stream

/q2r_right_hand_inputs
  -> quest_inputs_ab_gripper_mapper_node
  -> /control/metaquest/gripper_cmd --------------------                                                         same selected source
/control/lerobot/gripper_target -----------------------/       |
                                                                v
                                                        /jrt_gripper/cmd
                                                                |
                                                   jrt_tool_io_driver_node
                                                                v
                                                   Doosan Tool Digital I/O
```

LeRobot never publishes `/vr/target_posx`, `/vr/safe_posx`, or
`/dsr01/dsr_controller2/servol_rt_stream`. It only publishes source input topics when the Robot
mode is `policy_live`.

The Doosan controller publishes the configured RT diagnostics as absolute topics:

- `/rt_topic/actual_tcp_position` (6 values, mm/deg)
- `/rt_topic/actual_tcp_velocity` (6 values)
- `/rt_topic/robot_state` (one value)
- `/rt_topic/solution_space` (one value)

Joint state comes from `/dsr01/joint_states` and is already expressed in radians.

## MetaQuest calibration ownership

`xyz_mapper_node` is the single owner of the session XY-yaw correction. The
calibration-only GUI never calculates or applies a transform; it only calls the
mapper services and displays the mapper's transient-local state.

The static transform remains versioned in `xyz_position_only.yaml`:

```text
Robot X = +0.5 * Quest delta Y
Robot Y = -0.5 * Quest delta X
Robot Z = +0.5 * Quest delta Z
Robot Ry = -0.7 * Quest relative roll
Robot Rx and Rz are locked
```

The session state is one of `UNCALIBRATED`, `CALIBRATING`, `VALID`, or
`INVALID`. The status JSON contains the correction, observed direction, mapped
travel distance, before/after vectors, sequence number, reason, and a
fingerprint of the static mapping. It is intentionally not persisted across
mapper restarts.

Interfaces:

```text
/vr/metaquest_calibration/valid   std_msgs/Bool
/vr/metaquest_calibration/status  std_msgs/String JSON
/vr/calibrate_xy_yaw_to_x_plus    std_srvs/Trigger
/vr/reset_xy_yaw_calibration      std_srvs/Trigger
/vr/recenter                      std_srvs/Trigger
```

`Recenter` changes the current Quest position/orientation anchor but preserves a
completed XY-yaw correction. `Reset` restores the configured initial correction
and marks the session invalid. A rejected tracking jump or input timeout also
invalidates the calibration. Starting a new calibration immediately publishes
`valid=false`.

The separate `metaquest_calibration_gui` contains only Calibrate, Recenter, and
Reset. It has no preparation, RT, live-enable, hold, stop, or Tool I/O controls.
It may be launched with the full stack using `start_calibration_gui:=true`, or
against an already running mapper with:

```bash
ros2 launch quest_a0509_teleop metaquest_calibration_gui.launch.py
```

## Source state machine

The state is one of `DISABLED`, `METAQUEST`, or `LEROBOT`. Startup is always
`DISABLED`; input traffic never selects a source automatically.

| Current | Request | Live output | Result |
|---|---|---:|---|
| any | `DISABLED` | either | accepted immediately |
| `DISABLED` | `METAQUEST` | false | accepted only when calibration is valid |
| `DISABLED` | `LEROBOT` | false | accepted; MetaQuest calibration is irrelevant |
| `METAQUEST` | `LEROBOT` | false | accepted |
| `LEROBOT` | `METAQUEST` | false | accepted |
| any | active source | true | rejected |

Every real source change publishes one gripper `stop`, clears the previous target
cache, sets selected-command validity false, and requires a new arm target before
arm or gripper output is allowed. Requesting `DISABLED` again still publishes
`stop` and immediately requests live-disable plus hold. Arm and gripper sources
cannot be selected independently.

When MetaQuest calibration becomes invalid while `METAQUEST` is selected, the
MUX immediately changes to `DISABLED`, publishes gripper `stop`, requests local
live-disable, and requests ServoL hold. It never automatically reselects
MetaQuest after recalibration.

Source selection services are:

```bash
ros2 service call /control/select_disabled std_srvs/srv/Trigger '{}'
ros2 service call /control/select_metaquest std_srvs/srv/Trigger '{}'
ros2 service call /control/select_lerobot std_srvs/srv/Trigger '{}'
```

Select `METAQUEST` or `LEROBOT` only while
`/vr/live_robot_output_enabled` is false. Source selection never prepares the
robot, starts RT, moves the arm, changes Tool I/O, or enables live output.

## Freshness and independent shutdown

| Check | Default | Owner | Failure action |
|---|---:|---|---|
| mapper-accepted MetaQuest pose | 0.30 s | MUX | source -> disabled, gripper stop, live false and hold |
| LeRobot target input | 0.30 s | MUX | same |
| selected command heartbeat | 0.35 s | streamer | local live false and existing TCP/last-command hold |
| LeRobot robot state | 0.50 s | Robot plugin | reject `policy_live` action |
| MetaQuest teacher input | 0.30 s | Teleoperator plugin | reject teacher action |
| MetaQuest XY-yaw calibration | latched | mapper/MUX/Teleoperator | block MetaQuest target, selection, and teacher action |

The MetaQuest timeout is based on
`/control/metaquest/valid_pose_heartbeat`, emitted by `xyz_mapper_node` only
after finite-position, quaternion, translation-jump, and rotation-jump checks
accept a `/q2r_right_hand_pose` sample. Rejected tracking jumps do not refresh
the mapper input timeout or the MUX watchdog. The LeRobot timeout is based on
actual `/control/lerobot/target_posx` reception. The MUX emits
`/control/selected_command_heartbeat` only when it forwards a new selected arm
command. Consequently, terminating the MUX also trips the independent streamer
watchdog. There is no automatic source recovery and no automatic live-enable.

The streamer publishes:

- `/vr/commanded_posx`: the exact final ramp/hold `pos` used in each ServoL message
- `/vr/live_robot_output_enabled`: the current local live gate, transient-local

## Gripper contract

MetaQuest provides `open`, `close`, or `stop`. LeRobot provides a scalar in
`[0, 1]`:

- target greater than `0.7`: close
- target less than `0.3`: open
- target from `0.3` through `0.7`: retain the previous commanded state

Duplicate output is suppressed. On source selection, hysteresis starts from the
latest `/jrt_gripper/commanded_state` when one is available.

JRT diagnostic topics are transient-local:

- `/jrt_gripper/accepted_command`
- `/jrt_gripper/completed_command`
- `/jrt_gripper/commanded_state` (`open=0.0`, `close=1.0`)
- `/jrt_gripper/driver_busy`
- `/jrt_gripper/last_command_ok`

`commanded_state` changes only after a complete successful open/close Tool I/O
plan. `stop` retains the latch, and service failure does not claim the requested
state. While a Tool I/O plan is active, one pending command is coalesced; `stop`
has priority and cannot be overwritten by a later `open` or `close`. Every
asynchronous Tool I/O call has a `service_timeout_sec` response deadline. A
failure or timeout cancels the remaining plan and attempts both outputs OFF.
After every SET call, the driver polls
`/dsr01/dsr_controller2/io/get_tool_digital_output`. The pulse timer starts
only after the requested ON readback is observed, and completion is emitted
only after the final OFF readback. `readback_timeout_sec` and
`readback_poll_sec` bound this confirmation. This confirms the flange output,
not measured finger position or grasp force.

## LeRobot data schemas

The flat 7D action schema is identical for the Robot and Teleoperator:

1. `target_x_mm`
2. `target_y_mm`
3. `target_z_mm`
4. `target_o1_deg`
5. `target_o2_deg`
6. `target_o3_deg`
7. `gripper_target`

The three Doosan orientation values remain the mapper/`posx` representation.
They are not reinterpreted as a quaternion or generic RPY convention.

The 13D scalar observation state is:

1. `joint_1_rad` through `joint_6_rad`
2. `tcp_x_mm`, `tcp_y_mm`, `tcp_z_mm`
3. `tcp_o1_deg`, `tcp_o2_deg`, `tcp_o3_deg`
4. `gripper_commanded_state`

A configurable `front` camera is included separately as
`observation.images.front` by LeRobot. On the Jetson workcell the default is the
Logitech C920 at 640x480, 30 FPS, addressed through the persistent path
`/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0`. Override
`robot.cameras` for a different installed camera or explicitly set
`require_camera=false` only for software tests. The ZED 2 remains available as
an independent UVC stereo source; depth is not part of this policy observation.

`robot_state`, `solution_space`, `teleop_ready`, and live state are rollout gates
or diagnostics, not policy observation inputs.

## LeRobot modes and procedures

Install the plugin in the active LeRobot environment after building the ROS
workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/venvs/lerobot/bin/activate
source install/setup.bash
python -m pip install -e src/lerobot_robot_doosan_a0509
```

### MUX-first teacher recording

1. Start the integrated launch. It starts disabled.
2. Complete robot preparation and confirm `teleop_ready=true` while live remains
   false.
3. Open the calibration-only GUI, recenter if needed, press Calibrate, and move
   the Quest controller at least 10 cm toward physical robot +X over the default
   two-second window.
4. Confirm the GUI and `/vr/metaquest_calibration/valid` report `VALID/true`.
5. Verify fresh source topics and all diagnostics.
6. While live output is false, explicitly select `METAQUEST`.
7. Run `lerobot-record` with:
   `--robot.type=doosan_a0509_ros --robot.mode=shadow_record
   --teleop.type=metaquest_a0509` plus the dataset and camera options.
8. `MetaQuestA0509.get_action()` stores the mapper target and successfully
   commanded gripper latch as the teacher action. `Robot.send_action()` validates
   it and performs no publish in `shadow_record`.

The teacher action is not `/vr/target_posx`, `/vr/safe_posx`,
`/vr/commanded_posx`, or actual TCP. Those topics are execution diagnostics and
must not replace the source-specific teacher action.

The teacher adapter subscribes to mapper-accepted
`/control/metaquest/valid_pose_heartbeat`, not raw PoseStamped traffic. With its
default `require_calibration=true`, connection and `get_action()` reject a
missing or invalid calibration. This requirement does not apply to LeRobot
policy inference because policy actions enter through the independent
`/control/lerobot/*` source.

### Policy shadow

Keep source `METAQUEST`. Use Robot mode `policy_dry_run`; policy actions appear
only on `/control/lerobot/debug_action`. They do not enter the MUX. Compare policy
output with teacher and execution diagnostics offline.

### Policy live

1. Disable live output and confirm `/vr/live_robot_output_enabled: false`.
2. Explicitly select `LEROBOT`.
3. Start the policy with Robot mode `policy_live` and verify fresh policy input,
   MUX source, heartbeat, robot state, and safety output.
4. The user may then enable live output as a separate deliberate operation.

The LeRobot plugin never selects the source and never calls live-enable. Enabling
live output can move the physical A0509 and must only be done after the workcell
checklist below.

## Dry-run verification

The following launch must not connect to the Doosan bringup or send Tool I/O:

```bash
source /opt/ros/jazzy/setup.bash
source ~/venvs/lerobot/bin/activate
source install/setup.bash
ros2 launch quest_a0509_teleop a0509_full_bringup_with_gripper.launch.py   start_robot_bringup:=false   start_endpoint:=false   start_gui:=false   start_rviz:=false   dry_run:=true
```

Inspect:

```bash
ros2 topic echo --once /control/source
ros2 topic echo /control/mux_status
ros2 topic echo /control/selected_command_valid
ros2 topic echo /vr/live_robot_output_enabled
ros2 topic echo /jrt_gripper/driver_busy
ros2 topic echo --qos-durability transient_local /vr/metaquest_calibration/status
```

The configured preparation joint pose is
`[0.0, 0.0, 90.0, 0.0, 60.0, 0.0]` degrees. This work does not change the
preparation sequence and does not call `/vr/prepare_robot`.

## Rollback

For temporary diagnosis, start the integrated launch with
`use_command_mux:=false`. The mapper returns directly to `/vr/target_posx`, the
Quest gripper mapper returns directly to `/jrt_gripper/cmd`, and the streamer
disables its MUX-heartbeat requirement. Do not use LeRobot `policy_live` in this
mode; its source topics are intentionally disconnected from robot execution.

## Manual checklist before physical operation

- Verify physical E-stop, safety zones, payload, TCP, and Tool I/O wiring.
- Confirm the controller/DRCF version and A0509 namespace are correct.
- Confirm the runtime preparation pose, including J5=60 degrees, is safe in the
  actual workcell at reduced speed.
- Confirm MUX starts `DISABLED` and source changes are rejected while live.
- Confirm MetaQuest selection is rejected before calibration and accepted only
  while `/vr/metaquest_calibration/valid` is true.
- Reset calibration while MetaQuest is selected and confirm immediate
  `DISABLED`, gripper stop, live-disable, and hold.
- Confirm MetaQuest and LeRobot timeouts each force `DISABLED` and gripper stop.
- Kill the MUX in a software test and confirm the streamer heartbeat watchdog
  disables its local live gate and holds.
- Confirm `/jrt_gripper/commanded_state` is understood as commanded, not measured.
- Validate camera identity, frame shape, FPS, and timestamp freshness.
- Confirm joint state is radians and TCP/action pose is mm/Doosan degrees.
- Inspect target, safe, commanded, and actual TCP values without moving the robot.
- Keep `dry_run=true` until every check passes. Live-enable remains a separate
  operator action and is never performed by LeRobot.
