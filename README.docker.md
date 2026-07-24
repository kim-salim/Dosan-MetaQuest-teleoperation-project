# Docker Build and Publish

This image builds the local ROS 2 Humble workspace from `src/` and excludes
local `build/`, `install/`, and `log/` artifacts from the Docker context.

## Build

```bash
cd /home/salim2001/ros2_ws
docker build -t <dockerhub-id>/ros2-a0509:humble .
```

For Doosan controller firmware 3.x:

```bash
docker build --build-arg DRCF_VER=3 -t <dockerhub-id>/ros2-a0509:humble-drcf3 .
```

## Run

```bash
docker run --rm -it \
  --net=host \
  --ipc=host \
  -e ROS_DOMAIN_ID=29 \
  <dockerhub-id>/ros2-a0509:humble
```

GUI tools such as RViz need X11 or another display bridge from the host.

## Publish

```bash
docker login
docker push <dockerhub-id>/ros2-a0509:humble
```

The Doosan virtual emulator launches a separate Docker container. To use that
from inside this image, run against the host Docker daemon and ensure the
`doosanrobot/dsr_emulator:3.0.1` image is available on the host.
