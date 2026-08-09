# Doosan A0509 Meta Quest Teleoperation

ROS 2 Jazzy workspace for controlling a Doosan A0509 robot and a JRT gripper
from Meta Quest inputs on an arm64 Jetson Thor.


The current Jazzy integration uses a disabled-by-default command MUX for both
MetaQuest and LeRobot ACT. See
[docs/lerobot_a0509_mux_architecture.md](docs/lerobot_a0509_mux_architecture.md)
The accepted physical teleoperation parameters are frozen in
[docs/a0509_metaquest_accepted_baseline_2026-08-07.md](docs/a0509_metaquest_accepted_baseline_2026-08-07.md).

## System flow

```text
/q2r_right_hand_pose
  -> Quest pose mapper
  -> session XY +X calibration gate
  -> disabled-by-default source MUX
  -> workspace and motion safety guard
  -> gated ServoL RT streamer
  -> /dsr01/dsr_controller2/servol_rt_stream
  -> Doosan Jazzy controller

/q2r_right_hand_inputs
  -> Quest A/B mapper
  -> /jrt_gripper/cmd
  -> Doosan Tool Digital Output
  -> JRT gripper
```

Real robot output is disabled by default. The arm requires a successful robot
preparation step and an explicit live-output service call. The integrated
gripper launch inherits the same `dry_run=true` default.

## Active packages

```text
src/
├── quest2ros/            Quest message interfaces
├── ros_tcp_communication/ Unity/Quest TCP endpoint
├── quest_a0509_teleop/   Primary Quest-to-A0509 teleoperation pipeline
├── jrt_gripper_io/       Quest-to-JRT Tool I/O control
├── lerobot_robot_doosan_a0509/ LeRobot 0.6 robot/teleoperator plugin
└── doosan-robot2/        Pinned official Doosan ROS 2 Jazzy submodule
```

The LeRobot plugin runs from its Python 3.12 virtual environment. The earlier
`a0509_vr_teleop` UDP implementation and `dh_robot_rviz` model are
not part of the Jetson real-robot build.

## Clone

```bash
git clone --recurse-submodules \
  https://github.com/kim-salim/Dosan-MetaQuest-teleoperation-project.git
cd Dosan-MetaQuest-teleoperation-project
```

For an existing clone:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

## Jetson build

Prerequisites are Ubuntu 24.04, arm64, and ROS 2 Jazzy. The detailed native and
optional container procedures are in [README.jetson.md](README.jetson.md).

```bash
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update --rosdistro jazzy

rosdep install --ignore-src --rosdistro jazzy -r -y \
  --from-paths \
    src/quest2ros src/ros_tcp_communication \
    src/quest_a0509_teleop src/jrt_gripper_io \
    src/doosan-robot2/dsr_msgs2 src/doosan-robot2/dsr_common2 \
    src/doosan-robot2/dsr_hardware2 src/doosan-robot2/dsr_controller2 \
    src/doosan-robot2/dsr_description2 \
  --skip-keys "ament_python joint_state_publisher_gui ros_gz rviz2"

colcon build --symlink-install \
  --packages-select \
    quest2ros ros_tcp_endpoint \
    dsr_msgs2 dsr_common2 dsr_hardware2 dsr_controller2 dsr_description2 \
    quest_a0509_teleop jrt_gripper_io \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DDRCF_VER=2

source install/setup.bash
```

Use `-DDRCF_VER=3` for DRCF 3.x controllers.

## Test

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test \
  --packages-select quest2ros ros_tcp_endpoint quest_a0509_teleop jrt_gripper_io
colcon test-result --verbose
```

## Software-only dry run

This starts the Quest endpoint and teleoperation nodes without connecting to the
Doosan controller:

```bash
ros2 launch quest_a0509_teleop \
  a0509_full_bringup_with_gripper.launch.py \
  start_robot_bringup:=false \
  start_gui:=false \
  start_calibration_gui:=false \
  start_rviz:=false \
  dry_run:=true
```

For the real controller, omit `start_robot_bringup:=false` and provide `host`,
`rt_host`, and `port`. Keep `dry_run:=true` until all workcell checks pass.

## Safety

This software does not replace the robot safety controller, physical emergency
stop, verified Tool I/O wiring, or an operator safety procedure.

Before real operation:

1. Confirm MUX source `DISABLED`, live output false, and the Quest freshness
   watchdog active.
2. Complete robot preparation, then use the calibration-only GUI to establish a
   valid MetaQuest +X direction before selecting `METAQUEST`.
3. Verify JRT Tool DO polarity and wiring with the gripper disconnected.
4. Inspect calibration state, `/vr/target_posx`, `/vr/safe_posx`, and
   `/vr/status`.
5. Validate the robot preparation path at reduced speed in the actual workcell.
6. Keep `dry_run:=true` until all checks pass.

The prior architecture report remains available in
[ROS2_WS_ANALYSIS_REPORT_KO.md](ROS2_WS_ANALYSIS_REPORT_KO.md); its Humble Docker
section is superseded by `README.jetson.md`.
