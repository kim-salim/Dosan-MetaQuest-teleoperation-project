# Jetson Thor / ROS 2 Jazzy

This workspace targets the Jetson Thor host directly:

- Ubuntu 24.04
- arm64
- ROS 2 Jazzy
- Doosan's official `doosan-robot2` Jazzy branch
- Doosan Jazzy arm64 DRFL library

The Quest mapping, safety limits, robot preparation, ServoL streaming, and JRT
control logic are unchanged. Only the ROS distribution, Doosan source, package
dependencies, and bringup path are adapted for the Thor.

## Native build (recommended)

Docker is not required when ROS 2 Jazzy is already installed on the Thor.
Initialize the submodule and install only the real-robot dependencies:

```bash
cd ~/Dosan-MetaQuest-teleoperation-project
git submodule sync --recursive
git submodule update --init --recursive

source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update --rosdistro jazzy

rosdep install --ignore-src --rosdistro jazzy -r -y \
  --from-paths \
    src/quest2ros \
    src/ros_tcp_communication \
    src/quest_a0509_teleop \
    src/jrt_gripper_io \
    src/doosan-robot2/dsr_msgs2 \
    src/doosan-robot2/dsr_common2 \
    src/doosan-robot2/dsr_hardware2 \
    src/doosan-robot2/dsr_controller2 \
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

Use `-DDRCF_VER=3` for a Doosan controller running DRCF 3.x.

## Native verification

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

test "$(dpkg --print-architecture)" = arm64
test -f src/doosan-robot2/dsr_common2/lib/jazzy/arm64/libDRFL.a
ros2 pkg prefix dsr_controller2
ros2 interface show dsr_msgs2/msg/ServolRtStream
ros2 launch quest_a0509_teleop \
  a0509_full_bringup_with_gripper.launch.py --show-args

colcon test \
  --packages-select quest2ros ros_tcp_endpoint quest_a0509_teleop jrt_gripper_io
colcon test-result --verbose
```

## Optional container

The container is useful for reproducibility, but it is not required on a native
Jazzy host. It intentionally excludes Gazebo, MoveIt, and GUI packages.

```bash
docker build \
  -f Dockerfile.jetson \
  --build-arg DRCF_VER=2 \
  -t dosan-metaquest:jazzy-thor-arm64 .
```

The Dockerfile rejects non-arm64 builds. To include Tk and RViz, add
`--build-arg INSTALL_GUI=1`.

Verify the image:

```bash
docker image inspect \
  --format '{{.Architecture}}' \
  dosan-metaquest:jazzy-thor-arm64

docker run --rm dosan-metaquest:jazzy-thor-arm64 \
  ros2 pkg prefix dsr_controller2
```

## Endpoint-only smoke test

This does not connect to the robot or publish robot commands:

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=29 \
  dosan-metaquest:jazzy-thor-arm64 \
  ros2 launch quest_a0509_teleop \
    a0509_full_bringup_with_gripper.launch.py \
    start_endpoint:=true \
    start_robot_bringup:=false \
    start_teleop:=false \
    start_gui:=false \
    start_rviz:=false \
    start_gripper:=false \
    dry_run:=true
```

The Quest application must connect to the Jetson IP address on TCP port `10000`.

## Software-only dry run

This starts the endpoint and teleoperation nodes without a Doosan connection.
The preparation gate remains false, so no robot-ready target is produced.

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=29 \
  dosan-metaquest:jazzy-thor-arm64 \
  ros2 launch quest_a0509_teleop \
    a0509_full_bringup_with_gripper.launch.py \
    start_robot_bringup:=false \
    start_gui:=false \
    start_rviz:=false \
    dry_run:=true
```

## Robot Ethernet on this Thor

The Doosan controller is connected to `enP2p1s0`. Keep Wi-Fi as the default
route for Quest and Internet traffic, and configure only the directly connected
robot subnet on Ethernet:

```bash
sudo nmcli connection modify "Wired connection 1" \
  connection.interface-name enP2p1s0 \
  connection.autoconnect yes \
  ipv4.method manual \
  ipv4.addresses 192.168.137.20/24 \
  ipv4.gateway "" ipv4.dns "" ipv4.never-default yes \
  ipv6.method disabled
sudo nmcli connection up "Wired connection 1"

ping -c 3 -I enP2p1s0 192.168.137.100
nc -vz -w 2 192.168.137.100 12345
```

Here `192.168.137.20` is the Jetson address. The launch arguments `host` and
`rt_host` are robot-side addresses, not the Jetson address. Confirm the DRCF
major version on the Teach Pendant before changing the build-time `DRCF_VER`.

## Realtime scheduling permission

`ros2_control` attempts to run its controller-manager thread with
`SCHED_FIFO` priority 50. Configure the native Jetson user once:

```bash
cd ~/Dosan-MetaQuest-teleoperation-project
sudo bash scripts/setup_realtime_permissions.sh
```

The script creates the `realtime` group if needed, adds the invoking non-root
user to it, and installs `/etc/security/limits.d/99-realtime.conf`. Completely
log out and back in, or reboot the Jetson, before verifying the new login
session:

```bash
cd ~/Dosan-MetaQuest-teleoperation-project
bash scripts/verify_realtime_permissions.sh
```

All checks must report `PASS`, including the `SCHED_FIFO` priority 50 test.
Do not test with `sudo`, because the robot driver runs as the normal user.

## Real robot bringup

The launch uses a real-robot-only wrapper around Doosan's official Jazzy
`dsr_controller2`, `dsr_hardware2`, and `dsr_description2` packages. It does not
start an emulator, Gazebo, MoveIt, or RViz by default.

```bash
ros2 launch quest_a0509_teleop \
  a0509_full_bringup_with_gripper.launch.py \
  name:=dsr01 \
  host:=192.168.137.100 \
  rt_host:=192.168.137.100 \
  port:=12345 \
  model:=a0509 \
  start_endpoint:=true \
  start_gui:=false \
  start_rviz:=false \
  dry_run:=true
```

Keep `dry_run:=true` until the robot connection, Quest input freshness, robot
preparation, workspace limits, emergency stop, and Tool I/O polarity have been
verified. Enabling live output is a separate runtime service operation.
