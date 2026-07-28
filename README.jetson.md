# Jetson Thor Docker

The Jetson Thor host can remain on Ubuntu 24.04 and ROS 2 Jazzy. This image
uses an isolated Ubuntu 22.04 / ROS 2 Humble arm64 userspace because the
Doosan driver and bundled DRFL library in this repository target Humble.

The image contains:

- Doosan A0509 ROS 2 driver and arm64 DRFL library
- `quest2ros` messages
- `ros_tcp_endpoint` on TCP port 10000
- Quest-to-A0509 teleoperation and JRT gripper nodes
- RViz, MoveIt, ros2_control, and the existing Humble simulation dependencies

## Build on Jetson Thor

```bash
cd ~/ros2_ws
docker build \
  -f Dockerfile.jetson \
  --build-arg DRCF_VER=2 \
  -t kimsalim/ros2-a0509:humble-thor-arm64 .
```

For a Doosan controller using DRCF 3.x, use `--build-arg DRCF_VER=3`.

## Cross-build on an amd64 workstation

Create a Buildx builder once:

```bash
docker buildx create --name thor-builder --use
docker buildx inspect --bootstrap
```

Build and push the arm64 image:

```bash
cd ~/ros2_ws
docker buildx build \
  --platform linux/arm64 \
  -f Dockerfile.jetson \
  --build-arg DRCF_VER=2 \
  -t kimsalim/ros2-a0509:humble-thor-arm64 \
  --push .
```

Native Jetson builds do not need the `--platform` option.

## Verify on Jetson

```bash
docker image inspect \
  --format '{{.Architecture}}' \
  kimsalim/ros2-a0509:humble-thor-arm64
```

The result must be `arm64`.

```bash
docker run --rm \
  kimsalim/ros2-a0509:humble-thor-arm64 \
  bash -lc '
    dpkg --print-architecture
    ros2 pkg prefix quest2ros
    ros2 pkg prefix ros_tcp_endpoint
    python3 -c "from quest2ros.msg import OVR2ROSInputs; print(OVR2ROSInputs())"
  '
```

## Quest endpoint smoke test

This starts only the Quest TCP endpoint. It does not start the robot driver or
publish robot commands.

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=29 \
  kimsalim/ros2-a0509:humble-thor-arm64 \
  ros2 launch quest_a0509_teleop \
    a0509_full_bringup_with_gripper.launch.py \
    start_endpoint:=true \
    start_robot_bringup:=false \
    start_teleop:=false \
    start_gui:=false \
    start_gripper:=false \
    dry_run:=true
```

Configure the Quest application to use the Jetson IP address and TCP port
`10000`.

## Full dry-run

Give the Jetson Ethernet interface an address that can reach the Doosan
controller. The launch defaults use controller `192.168.137.100` and local
real-time host address `192.168.137.10`.

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=29 \
  kimsalim/ros2-a0509:humble-thor-arm64 \
  ros2 launch quest_a0509_teleop \
    a0509_full_bringup_with_gripper.launch.py \
    start_endpoint:=true \
    start_gui:=false \
    dry_run:=true \
    mode:=real \
    host:=192.168.137.100 \
    rt_host:=192.168.137.10
```

Keep `dry_run:=true` until the Quest watchdog, robot preparation, workspace
limits, emergency stop, and gripper I/O polarity have been verified.

## RViz / GUI

NVIDIA Container Runtime should be installed on the Thor host. If it is not
already configured as Docker's default runtime, add `--runtime nvidia`.

For an X11 desktop session:

```bash
xhost +si:localuser:root

docker run --rm -it \
  --network host \
  --ipc host \
  --runtime nvidia \
  -e DISPLAY \
  -e ROS_DOMAIN_ID=29 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  kimsalim/ros2-a0509:humble-thor-arm64 \
  ros2 launch quest_a0509_teleop \
    a0509_full_bringup_with_gripper.launch.py \
    start_endpoint:=true \
    start_gui:=true \
    dry_run:=true
```
