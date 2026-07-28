# quest_a0509_teleop

Meta Quest2ROS to Doosan A0509 task-space teleoperation for ROS 2 Humble.

This package maps the Meta Quest controller position into Doosan task-space
XYZ. It can also map controller orientation as an anchor-relative quaternion
delta into Doosan `rx`, `ry`, and `rz`.

## Safety Defaults

Real robot motion is disabled by default.

- `dry_run` defaults to `true`.
- The launch file defaults to `dry_run:=true`.
- `servol_rt_streamer_node` does not publish to `/dsr01/servol_rt_stream` while
  dry-run is enabled.
- Even when `dry_run:=false`, real ServoL RT streaming is blocked until
  `/vr/set_live_robot_output` is explicitly set to `true`.
- Workspace clamping and per-tick ramp limiting run before ServoL RT output.
- `/vr/target_posx` is not published until `/vr/prepare_robot` completes and
  `/vr/teleop_ready` becomes `true`.
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
orientation_delta_rotvec = rotvec(q_relative)
mapped_orientation_delta_rotvec[i] =
  rot_axis_sign[i] * scale_rpy[i] * orientation_delta_rotvec[rot_axis_map[i]]

R_delta = matrix_from_rotvec(mapped_orientation_delta_rotvec)
R_anchor = matrix_from_euler_xyz(robot_anchor_rpy_deg)
R_target = R_delta * R_anchor
target_rx, target_ry, target_rz = euler_xyz(R_target)
```

The default config swaps the current Quest x/y relative motion:

```yaml
axis_map: [1, 0, 2]
```

This means:

```text
Robot x <- Quest y
Robot y <- Quest x
Robot z <- Quest z
```

The default orientation mapping starts conservatively:

```yaml
enable_orientation_mapping: true
scale_rpy: [0.5, 0.5, 0.5]
rot_axis_sign: [1.0, -1.0, -1.0]
rot_axis_map: [1, 0, 2]
max_vr_rot_jump_deg: 45.0
```

The safety guard clamps orientation around the robot TCP anchor before ServoL
RT output:

```yaml
max_orientation_delta_deg: [15.0, 15.0, 20.0]
max_step_rpy_deg: [2.0, 2.0, 2.0]
```

`p_vr_now` is the filtered Quest position when
`enable_pose_low_pass_filter` is enabled. The filter runs in
`xyz_mapper_node` as each Quest pose arrives:

```text
filtered = filtered + pose_filter_alpha * (raw - filtered)
```

Default filter parameters:

```yaml
enable_pose_low_pass_filter: true
pose_filter_alpha: 0.2
max_vr_jump_m: 0.15
```

`pose_filter_alpha` closer to `1.0` follows the controller faster. Lower values
are smoother but add lag. `max_vr_jump_m` rejects a single Quest position sample
when it jumps farther than the threshold from the last accepted raw pose; set it
to `0.0` to disable jump rejection.

When `Recenter VR` or `/vr/recenter` is used, the mapper accepts the latest raw
Quest pose as the new anchor and resets the jump-filter baseline. This lets the
operator recover safely if Quest tracking jumps and target values stop changing.

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
/dsr01/servol_rt_stream
```

## Quest2ROS TCP Endpoint

Start the bundled Quest2ROS TCP endpoint from this workspace:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.py
```

The endpoint converts Quest2ROS app data into ROS topics such as
`/q2r_right_hand_pose`.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
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
rt_host:=192.168.137.10
port:=12345
model:=a0509
name:=dsr01
color:=white
```

The full launch still defaults to `dry_run:=true`, so `/dsr01/servol_rt_stream`
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
vel = [0, 0, 0, 0, 0, 0]
acc = [0, 0, 0, 0, 0, 0]
time = servol_time_sec
```
