FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ARG ROS_DISTRO=humble
ARG DRCF_VER=2

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_DOMAIN_ID=29

WORKDIR /ros2_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    dbus-x11 \
    git \
    libpoco-dev \
    libyaml-cpp-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    wget \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-example-interfaces \
    ros-${ROS_DISTRO}-force-torque-sensor-broadcaster \
    ros-${ROS_DISTRO}-forward-command-controller \
    ros-${ROS_DISTRO}-gazebo-msgs \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-ign-ros2-control \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-kdl-parser \
    ros-${ROS_DISTRO}-moveit-configs-utils \
    ros-${ROS_DISTRO}-moveit-msgs \
    ros-${ROS_DISTRO}-moveit-ros-move-group \
    ros-${ROS_DISTRO}-position-controllers \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ros2controlcli \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-transmission-interface \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true

COPY src ./src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -r -y \
      -t buildtool -t build -t build_export -t buildtool_export -t exec \
      --skip-keys ament_python \
    && rm -rf /var/lib/apt/lists/* \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DDRCF_VER=${DRCF_VER}

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
