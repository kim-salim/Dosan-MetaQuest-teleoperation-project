# a0509_vr_teleop

ROS2 Humble package for a MetaQuest-to-Doosan A0509 teleoperation pipeline.

The package publishes Doosan ServoL RT commands only when
`enable_robot_output:=true`. The default is `false`, so the first run is a
dry-run that publishes `/teleop/debug_servol_rt_stream` only.

## Pose Convention

Internal target pose format:

```text
[x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
frame: robot base
position: millimeters
rotation: degrees
```

No scale is applied to VR motion. The frame mapper uses a VR anchor and robot
TCP anchor for 1:1 relative motion.

## Safety Policy

The safety guard runs at 10 Hz:

- inside workspace: raw target passes through
- outside workspace: target is projected to the workspace boundary along the
  segment from `last_safe_pose` to `raw_pose`
- ramp limit after projection: 20 mm/tick and 3 deg/tick
- tracking loss, deadman release, or watchdog timeout: hold last safe target

Initial conservative box workspace is configured in
`config/a0509_vr_teleop.yaml`.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select a0509_vr_teleop
source install/setup.bash
```

## Test

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select a0509_vr_teleop
colcon test-result --verbose
```

## Dry Run

This starts mock Quest input, projection, ServoL debug stream, monitor, and RViz
markers. It does not publish to `/dsr01/servol_rt_stream`.

```bash
ros2 launch a0509_vr_teleop a0509_vr_teleop.launch.py
```

Useful topics:

```text
/vr/controller_state
/teleop/raw_target_pose
/teleop/projected_target_pose
/teleop/safety_status
/teleop/debug_servol_rt_stream
/teleop/markers
```

## Real Robot Output

Only after dry-run and RViz verification:

```bash
ros2 launch a0509_vr_teleop a0509_vr_teleop.launch.py \
  use_mock_quest_input:=false \
  enable_robot_output:=true \
  robot_namespace:=/dsr01
```

This package does not servo on/off the robot and does not replace the physical
emergency stop or Doosan safety controller.
