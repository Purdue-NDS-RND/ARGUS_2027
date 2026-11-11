# ARGUS mission simulator: the club ArduPilot + Gazebo Harmonic image, plus
# ROS 2 Humble, MAVROS and the ROS-Gazebo bridge, all in one container.
#
# Build from the repo root (sim/run_sim.sh --build does this):
#   docker compose -f sim/compose.yaml build
#
# The base image comes from the club repo (Drone_Sim_Docker, `make build`).
# Our code is not copied in: sim/run_sim.sh mounts the repo, and
# sim/container/start.sh builds it with colcon at start-up.

ARG BASE_IMAGE=ardupilot-gz:4.6.3
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
# https://github.com/ros-infrastructure/ros-apt-source/releases
ARG ROS_APT_SOURCE_VERSION=1.3.0

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
USER root

# ROS 2 Humble apt source. The OSRF Gazebo source is already in the base image
# and also carries ros-humble-ros-gzharmonic (ros_gz built for Harmonic).
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates curl \
    && curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb" \
    && apt-get install -y --no-install-recommends /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-humble-cv-bridge \
        ros-humble-vision-msgs \
        ros-humble-image-transport \
        ros-humble-mavros \
        ros-humble-mavros-extras \
        ros-humble-ros-gzharmonic \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-yaml \
    && /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh \
    && rm -rf /var/lib/apt/lists/*

# Resolve our packages' dependencies with rosdep. Only the package.xml files
# are in the build context (see sim/Dockerfile.dockerignore), so editing code
# does not invalidate this layer.
# ament_python is skipped: four of our package.xml files list it as a
# buildtool_depend, but it is a build type, not a rosdep key. colcon does not
# need it.
COPY src /tmp/argus-src/src
RUN rosdep init \
    && rosdep update --rosdistro humble \
    && apt-get update \
    && rosdep install --rosdistro humble --from-paths /tmp/argus-src/src \
        --ignore-src -y --skip-keys ament_python \
    && rm -rf /var/lib/apt/lists/* /tmp/argus-src

ENV ROS_DISTRO=humble \
    GZ_VERSION=harmonic

LABEL org.opencontainers.image.title="ARGUS mission simulator" \
      org.opencontainers.image.description="ArduCopter 4.6.3 SITL, Gazebo Harmonic, ROS 2 Humble, MAVROS and ros_gz"

# The entrypoint lives in the mounted repo; sim/compose.yaml sets it.
ENTRYPOINT []
CMD ["bash"]
