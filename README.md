# Doosan A0509 Meta Quest Teleoperation

ROS 2 Humble workspace for controlling a Doosan A0509 robot and a JRT gripper
from Meta Quest inputs.

## System Flow

Robot arm:

```text
/q2r_right_hand_pose
  -> Quest pose mapper
  -> workspace and motion safety guard
  -> gated ServoL RT streamer
  -> /dsr01/servol_rt_stream
  -> Doosan controller
```

Gripper:

```text
/q2r_right_hand_inputs
  -> Quest A/B mapper
  -> /jrt_gripper/cmd
  -> Doosan Tool Digital Output
  -> JRT gripper
```

Real robot output is disabled by default. The arm requires a successful robot
preparation step and an explicit live-output service call. The integrated
gripper launch inherits the same dry-run default.

## Repository Layout

```text
src/
├── quest_a0509_teleop/  Primary Quest-to-A0509 teleoperation pipeline
├── jrt_gripper_io/      Quest/Joy-to-JRT Tool I/O control
├── a0509_vr_teleop/     Earlier UDP/mock VR pipeline and RViz safety tools
├── dh_robot_rviz/       Standalone DH-model RViz visualization
└── doosan-robot2/       Pinned Doosan ROS 2 driver submodule
```

The complete architecture, parameters, interfaces, test status, and known
safety gaps are documented in
[ROS2_WS_ANALYSIS_REPORT_KO.md](ROS2_WS_ANALYSIS_REPORT_KO.md).

## Clone

```bash
git clone --recurse-submodules \
  https://github.com/kim-salim/Dosan-MetaQuest-teleoperation-project.git ros2_ws
cd ros2_ws
```

For an existing clone:

```bash
git submodule update --init --recursive
```

## Build

Prerequisites:

- Ubuntu 22.04
- ROS 2 Humble
- Doosan controller dependencies described by `src/doosan-robot2/README.md`
- A separately built Quest2ROS workspace for `quest2ros/msg/OVR2ROSInputs`

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
source install/setup.bash
```

## Test

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select a0509_vr_teleop jrt_gripper_io quest_a0509_teleop
colcon test-result --verbose
```

## Dry Run

Start the integrated A0509, Quest teleoperation, GUI, and gripper launch without
publishing real ServoL or Tool I/O commands:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch quest_a0509_teleop a0509_full_bringup_with_gripper.launch.py \
  dry_run:=true
```

If Doosan bringup is already running:

```bash
ros2 launch quest_a0509_teleop a0509_full_bringup_with_gripper.launch.py \
  start_robot_bringup:=false \
  dry_run:=true
```

## Safety

This software does not replace the robot safety controller, physical emergency
stop, verified Tool I/O wiring, or an operator safety procedure.

Before real operation:

1. Verify Quest input freshness and the arm-output gate.
2. Verify JRT Tool DO polarity and wiring with the gripper disconnected.
3. Inspect `/vr/target_posx`, `/vr/safe_posx`, and `/vr/status`.
4. Validate the robot preparation path in the actual workcell.
5. Keep `dry_run:=true` until all checks pass.
