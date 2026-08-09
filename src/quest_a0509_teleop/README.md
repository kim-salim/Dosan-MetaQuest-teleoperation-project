# quest_a0509_teleop

Meta Quest2ROS to Doosan A0509 task-space teleoperation for ROS 2 Jazzy.

The full bringup launches a disabled-by-default arm/gripper command MUX. The
MetaQuest mapper publishes `/control/metaquest/target_posx`; only the MUX owns
`/vr/target_posx` when `use_command_mux:=true`. The complete LeRobot ACT design
is documented in `../../docs/lerobot_a0509_mux_architecture.md`.

This package maps the Meta Quest controller position into Doosan task-space
XYZ. It can also map controller orientation as an anchor-relative quaternion
delta into Doosan `rx`, `ry`, and `rz`.

## Safety Defaults

Real robot motion is disabled by default.

- `dry_run` defaults to `true`.
- The launch file defaults to `dry_run:=true`.
- `servol_rt_streamer_node` does not publish to `/dsr01/dsr_controller2/servol_rt_stream` while
  dry-run is enabled.
- Even when `dry_run:=false`, real ServoL RT streaming is blocked until
  `/vr/set_live_robot_output` is explicitly set to `true`.
- Workspace clamping and per-tick ramp limiting run before ServoL RT output.
- `/vr/target_posx` is not published until `/vr/prepare_robot` completes and
  `/vr/teleop_ready` becomes `true`.
- MetaQuest targets are also blocked until the current mapper session reports a
  valid XY +X calibration. Mapper restart, reset, tracking jump, or input timeout
  requires calibration again.
- `/vr/stop_robot` can cancel an in-progress preparation wait without waiting for
  the preparation operation lock, then sends `MoveStop` and disables live output.
- The MetaQuest MUX watchdog uses only mapper-accepted pose heartbeats; invalid
  quaternions, non-finite samples, and rejected pose jumps do not refresh it.
- Live ServoL RT output is rejected while `/vr/teleop_ready` is `false`.
- Verify `/vr/target_posx`, `/vr/safe_posx`, and `/vr/status` before enabling
  robot output.

This software does not replace the physical emergency stop, Doosan safety
controller, or a human operator's safety checks.

## Units

Input:

```text
/q2r_right_hand_pose: geometry_msgs/PoseStamped
position: meters
orientation: quaternion
```

Internal and Doosan target:

```text
[x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
```

Mapping:

```text
p_vr_delta = p_vr_now - p_vr_anchor
mapped_delta[i] = p_vr_delta[axis_map[i]]
target_x = anchor_x + axis_sign[0] * scale_xyz[0] * mapped_delta[0] * 1000.0
target_y = anchor_y + axis_sign[1] * scale_xyz[1] * mapped_delta[1] * 1000.0
target_z = anchor_z + axis_sign[2] * scale_xyz[2] * mapped_delta[2] * 1000.0

q_relative = q_vr_now * inverse(q_vr_anchor)
orientation_delta_rpy = euler_xyz(q_relative)
mapped_orientation_delta_rpy[i] =
  rot_axis_sign[i] * scale_rpy[i] * orientation_delta_rpy[rot_axis_map[i]]

q_delta = quaternion_from_euler_xyz(mapped_orientation_delta_rpy)
q_target = q_delta * quaternion_from_euler_xyz(robot_anchor_rpy_deg)
target_rx, target_ry, target_rz = nearest_equivalent_euler_xyz(q_target)
```

The default config swaps the current Quest x/y relative motion:

```yaml
axis_map: [1, 0, 2]
```

This means:

```text
Robot x <- Quest y
Robot y <- -Quest x
Robot z <- Quest z
```

The default orientation mapping starts conservatively:

```yaml
enable_orientation_mapping: true
scale_rpy: [0.0, 0.7, 0.0]
rot_axis_sign: [-1.0, -1.0, -1.0]
rot_axis_map: [1, 0, 2]
max_vr_rot_jump_deg: 45.0
```

In this roll-only profile, positive Quest roll still produces the configured
negative robot rotation at 0.7 gain. The Mapper applies that delta about the
physical axis represented by the anchor's Doosan ZYZ B component, then chooses
the equivalent ZYZ branch nearest the previous target. This preserves the
operator gesture while crossing beta=+/-180 degrees continuously.

The safety guard clamps the physical quaternion angle around the robot TCP
anchor before ServoL RT output:

```yaml
max_orientation_delta_deg: [90.0, 90.0, 90.0]
publish_rate_hz: 30.0
```

`p_vr_now` is the fixed-rate, filtered Quest position. Accepted Quest callbacks
only validate and append receive-time poses to a bounded jitter buffer. The
existing Mapper 30 Hz timer queries an older point on that timeline, uses linear
interpolation for position and quaternion SLERP for orientation, and then runs
the low-pass filters exactly once per control tick:

```text
callback: buffer.push(receive_monotonic_time, position, quaternion)
tick:     pose = buffer.sample(now - pose_interpolation_delay_sec)
tick:     filtered = filtered + fixed_rate_alpha * (pose - filtered)
```

Default resampling and filter parameters:

```yaml
enable_pose_resampling: true
pose_interpolation_delay_sec: 0.05
pose_buffer_duration_sec: 1.0
pose_buffer_max_samples: 128
pose_max_interpolation_gap_sec: 0.08
pose_full_prediction_sec: 0.02
pose_prediction_decay_sec: 0.08
pose_tracking_invalid_after_sec: 0.3
enable_pose_low_pass_filter: true
pose_filter_alpha: 0.4
pose_filter_reference_rate_hz: 72.0
max_vr_jump_m: 0.15
```

`pose_filter_alpha` remains the response at the nominal Quest input rate. The
Mapper converts it to a 30 Hz tick alpha from the configured reference rate, so
moving the filter out of the bursty callback does not silently change its time
constant. Values closer to `1.0` follow faster; lower values are smoother but add
lag.

When a query is newer than the latest buffered pose, prediction is fully enabled
for at most 20 ms and its velocity then decays to zero by 80 ms. Wider gaps are
held, not interpolated or indefinitely extrapolated. A 300 ms receive gap marks
tracking stale, clears the buffer, invalidates calibration, and stops target
publication; the existing 500 ms input timeout remains the final watchdog.

`max_vr_jump_m` rejects a single Quest position sample when it jumps farther
than the threshold from the last accepted raw pose; set it to `0.0` to disable
jump rejection. Accepted samples publish
`/control/metaquest/valid_pose_heartbeat`; rejected samples do not update either
the mapper input freshness or the MUX freshness.

The Quest pose subscription uses `KEEP_LAST(depth=1)`, `BEST_EFFORT`, and
`VOLATILE` QoS. The callback is intentionally lightweight so delivered burst
samples can enter the local buffer; DDS still discards queued historical poses if
the subscriber falls behind. The current independent Quest app does not provide
a preserved source timestamp or tracking-state flag, so the implementation uses
Jetson monotonic receive time. Source-time resampling would require a future app
protocol change and is not assumed here.

The safety guard stores the newest selected target and publishes finite,
workspace-clamped output with an anchor-relative quaternion geodesic orientation
limit from an independent 30 Hz timer. In the current roll-only configuration,
the second value of `max_orientation_delta_deg` is the physical angular envelope.
The guard intentionally has no ramp limiter; the Streamer fixed-rate loop owns
linear ramp limiting and applies rotational ramp limiting with quaternion SLERP,
then selects the continuous Doosan ZYZ branch nearest the previous command.

The current workspace profile is:

```yaml
workspace_min_xyz_mm: [50.0, -350.0, 0.0]
workspace_min_limit_enabled: [true, true, false]
workspace_max_xyz_mm: [650.0, 350.0, 600.0]
```

The `false` Z entry disables only the Safety Guard's Z lower clamp. Finite-value
validation and the X/Y lower limits, all upper limits, and the robot controller's
own physical limits remain active.

Mapper preparation-state and robot-anchor inputs, and the safety guard
robot-anchor input, use reliable transient-local QoS. Restarting these nodes
therefore restores the last state retained by `robot_prep_node` instead of using
default anchors or requiring a second preparation move.

When `Recenter VR` or `/vr/recenter` is used, the mapper accepts the latest raw
Quest pose as the new anchor, resets the jump-filter baseline, and flushes the
resampling history. Robot-anchor updates, calibration start/completion/reset,
the transition to teleop-ready, and Live enable also flush the buffer so
pre-transition poses can never be replayed under a new anchor. Recenter preserves
an already completed XY-yaw correction unless a calibration measurement is
currently active.

## Calibration-only GUI

The optional small GUI only calls mapper calibration services. It has no robot
preparation, RT, live-output, stop, or gripper controls. Start it with the full
bringup:

```bash
ros2 launch quest_a0509_teleop a0509_full_bringup_with_gripper.launch.py \
  start_gui:=false \
  start_calibration_gui:=true \
  dry_run:=true
```

If the mapper is already running, start only the GUI:

```bash
ros2 launch quest_a0509_teleop metaquest_calibration_gui.launch.py
```

Recommended operator order:

1. Complete `/vr/prepare_robot` and keep live output false.
2. Confirm fresh Quest pose, then use Recenter if the tracking frame moved.
3. Press `Calibrate XY +X` and move the Quest controller at least 10 cm in the
   physical robot +X direction over the default two-second measurement.
4. Confirm `VALID` in the GUI before selecting the `METAQUEST` MUX source.

State and service interfaces:

```text
/vr/metaquest_calibration/valid
/vr/metaquest_calibration/status
/vr/calibrate_xy_yaw_to_x_plus
/vr/reset_xy_yaw_calibration
/vr/recenter
```

The status and validity topics are transient-local. Calibration remains in the
mapper when the GUI is closed, but is intentionally not persisted across mapper
restarts. `Reset Calibration` returns to the configured
`xy_yaw_correction_deg` and marks the session invalid.

## Supervised 30-second full teleoperation

After real bringup, robot preparation, Quest recenter, and a `VALID` XY +X
calibration, start the supervised arm-and-gripper session:

```bash
ros2 run quest_a0509_teleop metaquest_full_session --ros-args \
  -p duration_sec:=30.0
```

The runner first forces Live false, selects `DISABLED`, and sends a gripper
`stop`. It then waits at `FULL_TELEOP_READY_FOR_GO`. Press Enter only after the
workspace is clear and the physical emergency stop is available.

During the live interval:

- Quest A (`button_lower`) closes the gripper.
- Quest B (`button_upper`) opens the gripper.
- Releasing both buttons requests gripper `stop`.
- Hold A and B separately for about 0.5 second so the pulse-mode Tool DO plan
  can complete.

This session intentionally has no temporary anchor-relative position envelope,
rotation envelope, or early test duration other than `duration_sec`. The normal
workspace clamp, orientation clamp, command ramps, calibration/prepare gates,
MUX source ownership, robot-state check, and pose/input heartbeats remain
mandatory. A dropped gate or failed gripper command immediately ends the
session.

At normal completion, exception, or Ctrl-C, the runner disables Live output,
selects `DISABLED`, directly requests gripper `stop`, and waits for the driver to
confirm the stop. `gripper_validation_pass=true` requires both physical `open`
and `close` commands to have been accepted during the live interval. The robot
holds its final TCP and does not automatically return to the preparation pose.

## Topics

Input:

```text
/q2r_right_hand_pose
```

Internal:

```text
/vr/target_posx
/vr/safe_posx
/vr/robot_anchor_posx
/vr/teleop_ready
/vr/status
```

Robot output, only when `dry_run:=false`:

```text
/dsr01/dsr_controller2/servol_rt_stream
```

## Quest2ROS TCP Endpoint

Start the bundled Quest2ROS TCP endpoint from this workspace:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.py
```

The endpoint converts Quest2ROS app data into ROS topics such as
`/q2r_right_hand_pose`.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to quest_a0509_teleop
source install/setup.bash
```

## Run Dry-Run

```bash
ros2 launch quest_a0509_teleop xyz_position_only.launch.py
```

Useful checks:

```bash
ros2 topic echo /vr/status
ros2 topic echo /vr/teleop_ready
ros2 topic echo /vr/target_posx
ros2 topic echo /vr/safe_posx
```

Recenter the VR anchor to the latest Quest pose:

```bash
ros2 service call /vr/recenter std_srvs/srv/Trigger {}
```

## Real Robot Check

Full launch with Doosan A0509 bringup, teleop pipeline, and GUI:

```bash
ros2 launch quest_a0509_teleop a0509_full_bringup.launch.py
```

This wraps the same Doosan bringup settings used by
`~/doosan_a0509_tools/launch_a0509_real_rviz.sh`:

```text
mode:=real
host:=192.168.137.100
rt_host:=192.168.137.100
port:=12345
model:=a0509
name:=dsr01
color:=white
```

The full launch still defaults to `dry_run:=true`, so `/dsr01/dsr_controller2/servol_rt_stream`
is not published. For an armed real-robot check:

```bash
ros2 launch quest_a0509_teleop a0509_full_bringup.launch.py dry_run:=false
```

If Doosan bringup is already running in another terminal, avoid launching a
second controller stack:

```bash
ros2 launch quest_a0509_teleop a0509_full_bringup.launch.py \
  start_robot_bringup:=false \
  dry_run:=false
```

Use the real-robot launch when you want the preparation services as well as the
position pipeline:

```bash
ros2 launch quest_a0509_teleop xyz_position_only_real.launch.py
```

GUI version:

```bash
ros2 launch quest_a0509_teleop xyz_position_only_gui.launch.py
```

For an armed real-robot check with the GUI, keep the GUI's live output disabled
until after prep/anchor checks:

```bash
ros2 launch quest_a0509_teleop xyz_position_only_gui.launch.py dry_run:=false
```

The GUI shows:

```text
Quest Pose
Target PosX
Safe PosX
Robot Anchor
Teleop Ready
/vr/status log
```

The GUI controls:

```text
Recenter VR
Anchor = Current TCP
Prepare Robot
Start RT Control
Stop RT Control
Enable Live ServoL
Disable Live ServoL
Hold ServoL
Stop Robot
Reset SAFE_OFF
```

Real-motion actions are disabled until `Enable real-action buttons` is checked,
and each high-risk action opens a confirmation dialog.

This still starts in dry-run mode. To arm the streamer but keep live output
disabled:

```bash
ros2 launch quest_a0509_teleop xyz_position_only_real.launch.py dry_run:=false
```

Preparation service copied from the ServoL RT GUI workflow:

```bash
ros2 service call /vr/prepare_robot std_srvs/srv/Trigger {}
```

This moves to the Cartesian prep joint pose:

```text
[0.0, 0.0, 90.0, 0.0, 60.0, 0.0] deg
```

After the prep move, the node reads the current TCP pose, publishes it to
`/vr/robot_anchor_posx`, and asks `/vr/recenter` to use the latest Quest pose as
the VR anchor. This makes the first live command start from the actual robot TCP
instead of the static config anchor.

Until this finishes successfully, `/vr/teleop_ready` stays `false`,
`xyz_mapper_node` does not publish `/vr/target_posx`, and
`servol_rt_streamer_node` refuses `Enable Live ServoL`.

If the robot is already in the intended pose and you only want to anchor to the
current TCP:

```bash
ros2 service call /vr/set_robot_anchor_to_current_tcp std_srvs/srv/Trigger {}
```

Enable live ServoL RT output only after the prep/anchor step and topic checks:

```bash
ros2 service call /vr/set_live_robot_output std_srvs/srv/SetBool "{data: true}"
```

Disable live output:

```bash
ros2 service call /vr/set_live_robot_output std_srvs/srv/SetBool "{data: false}"
```

Emergency software stop helper:

```bash
ros2 service call /vr/stop_robot std_srvs/srv/Trigger {}
```

SAFE_OFF reset helper, matching the GUI's explicit reset button:

```bash
ros2 service call /vr/reset_safe_off std_srvs/srv/Trigger {}
```

RT control helpers are exposed for controller bringup checks:

```bash
ros2 service call /vr/start_rt_control std_srvs/srv/Trigger {}
ros2 service call /vr/stop_rt_control std_srvs/srv/Trigger {}
```

## ServoL RT Message

Only after dry-run verification and with the robot in a safe state:

```bash
ros2 launch quest_a0509_teleop xyz_position_only_real.launch.py dry_run:=false
```

When `dry_run:=false` and `/vr/set_live_robot_output` is true,
`servol_rt_streamer_node` publishes `dsr_msgs2/msg/ServolRtStream` with:

```text
pos = /vr/safe_posx data
vel = [-10000, -10000, -10000, -10000, -10000, -10000]
acc = [-10000, -10000, -10000, -10000, -10000, -10000]
time = servol_time_sec
```

The experiment profile sets `servol_use_auto_velocity_acceleration: true`.
Doosan interprets `-10000` as `DR_COND_NONE` and derives endpoint velocity and
acceleration from the streamed target pose. Set the parameter to `false` to
restore the previous explicit `vel=0`, `acc=0` behavior.

The default real-output streaming profile is:

```text
publish rate        = 30 Hz (33.3 ms)
ServoL time         = 0.1 s
linear ramp         = 6.67 mm/tick (about 200 mm/s at 30 Hz)
rotational ramp     = 1.0 deg/tick (30 deg/s at 30 Hz)
```

During Live operation, robot readiness comes from the Doosan controller's
existing `/rt_topic/robot_state` `Float64MultiArray` topic. The configured
controller publishes this topic from its 10 ms RT-data timer (observed at about
100 Hz); this does not change the 30 Hz ServoL control rate. The Streamer uses a
`KEEP_LAST(depth=1)`, reliable, transient-local subscription and only caches the
latest integer state code and receive time. Repeated `GetRobotState` service
polling is not used in the Live path.

The accepted state codes remain `1` (`STANDBY`) and `2` (`MOVING`), and the
cached sample must remain fresher than `robot_state_stale_timeout_sec` (default
1.0 second). Invalid, missing, unsafe, or stale samples reject Live enable or
immediately stop an active Live session. `robot_prep_node` still performs its
separate one-shot state check before a preparation move.
