# Docker build

The legacy amd64/Humble image is no longer the deployment target. Jetson Thor
uses the arm64 ROS 2 Jazzy image defined in `Dockerfile.jetson`.

See [README.jetson.md](README.jetson.md) for the native Jazzy workflow and the
optional container build, verification, and launch commands.

```bash
docker build \
  -f Dockerfile.jetson \
  --build-arg DRCF_VER=2 \
  -t dosan-metaquest:jazzy-thor-arm64 .
```
