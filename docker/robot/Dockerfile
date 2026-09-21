# syntax=docker/dockerfile:1
#
# The image that runs ON an Andino robot: the hardware layer and nothing else.
#   robot_state_publisher + ros2_control (andino_base -> the Arduino) + diff_drive + RPLIDAR
# SLAM, Nav2 and perception are deliberately NOT here. They run off-robot, on machines with
# GPUs and mains power, as part of the same ROS graph — a different image.
#
# Build context is the REPOSITORY ROOT:
#   docker build -f docker/robot/Dockerfile -t andino-robot .
# Multi-arch by construction (ros:*-ros-base is published for amd64 and arm64); the robot is arm64.
#
# ROS_DISTRO: jazzy by default. The workspace builds for humble too (upstream CI covers both).

ARG ROS_DISTRO=jazzy

# ---------------------------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS deps
ARG ROS_DISTRO
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /ws

# package.xml files ONLY, so the slow dependency layer below is rebuilt when dependencies change
# and not every time a source file does.
COPY andino_description/package.xml src/andino_description/package.xml
COPY andino_base/package.xml        src/andino_base/package.xml
COPY andino_control/package.xml     src/andino_control/package.xml
COPY andino_bringup/package.xml     src/andino_bringup/package.xml

# xterm is skipped: andino_bringup wants it for the keyboard-teleop launch, and a robot has no
# display. rmw_zenoh is added explicitly — see the ENV block at the bottom for why.
RUN apt-get update \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}" --skip-keys "xterm" \
 && apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-rmw-zenoh-cpp" \
 && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------------------------
FROM deps AS build
ARG ROS_DISTRO
COPY andino_description src/andino_description
COPY andino_base        src/andino_base
COPY andino_control     src/andino_control
COPY andino_bringup     src/andino_bringup
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && colcon build --merge-install --install-base /opt/andino \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# ---------------------------------------------------------------------------------------------
# Runtime = the dependency layer + the built install tree. The build tree is left behind.
FROM deps AS runtime
ARG ROS_DISTRO
# An on-robot gamepad (a wireless pad whose receiver rides on the robot: teleop that neither fades
# with distance nor crosses the Wi-Fi). Launch file and button map live with the deployment, not
# here. A LATE layer on purpose, not part of `deps`: the robot pulls images over Wi-Fi onto an SD
# card, and this way two small packages cost a small layer instead of a new 1 GB one.
RUN apt-get update \
 && apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-joy-linux" "ros-${ROS_DISTRO}-teleop-twist-joy" \
 && rm -rf /var/lib/apt/lists/*
COPY --from=build /opt/andino /opt/andino
COPY docker/robot/entrypoint.sh /entrypoint.sh
# For liveness probes: a node that is running is not necessarily a node that is publishing.
COPY docker/robot/topic_alive.py /usr/local/bin/topic-alive
# For the lidar container: an A1 left streaming by a killed driver hangs the next one's handshake.
COPY docker/robot/rplidar_stop.py /usr/local/bin/rplidar-stop

# rmw_zenoh rather than the default DDS, because this robot's ROS graph is meant to extend onto
# other machines (GPU perception, SLAM) across a routed network. DDS discovery is multicast, which
# neither a Kubernetes pod network nor a WireGuard mesh carries; zenoh discovers through a router
# the nodes dial out to. It needs `ros2 run rmw_zenoh_cpp rmw_zenohd` reachable on localhost:7447.
# To run on plain DDS instead (single host, no router): -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DISTRO=${ROS_DISTRO} \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp

ENTRYPOINT ["/entrypoint.sh"]
# Default: upstream's single-process bringup, un-namespaced. A fleet deployment runs
# robot_base.launch.py and robot_lidar.launch.py instead, in separate containers, with a namespace.
# The camera is off by default: on a libcamera-era Raspberry Pi OS the sensor is not a plain V4L2
# capture device, so upstream's v4l2_camera launch does not apply. Enable it per deployment.
CMD ["ros2", "launch", "andino_bringup", "andino_robot.launch.py", "include_camera:=false"]
