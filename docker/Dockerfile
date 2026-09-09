# Drone_Sim — ROS 2 Jazzy + Gazebo Harmonic
# Base image is Ubuntu 24.04 (noble); the host stays on 22.04/Humble untouched.
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# ---- simulation + tooling -------------------------------------------------
# ros-jazzy-ros-gz pulls in Gazebo Harmonic (gz-sim8) from packages.ros.org.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      ros-jazzy-ros-gz-bridge \
      ros-jazzy-ros-gz-sim \
      ros-jazzy-ros-gz-image \
      ros-jazzy-actuator-msgs \
      ros-jazzy-rviz2 \
      ros-jazzy-tf2-eigen \
      ros-jazzy-tf2-ros \
      ros-jazzy-xacro \
      ros-dev-tools \
      python3-colcon-common-extensions \
      libeigen3-dev \
      mesa-utils \
      gdb \
      less vim tmux \
    && rm -rf /var/lib/apt/lists/*

# ---- non-root user matching the host, so bind-mounted files stay yours ----
ARG UID=1000
ARG GID=1000
ARG USERNAME=dev
RUN if getent group ${GID} >/dev/null; then groupmod -n ${USERNAME} "$(getent group ${GID} | cut -d: -f1)"; \
    else groupadd -g ${GID} ${USERNAME}; fi \
 && if id -u ${UID} >/dev/null 2>&1; then usermod -l ${USERNAME} -d /home/${USERNAME} -m "$(id -nu ${UID})"; \
    else useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME}; fi \
 && apt-get update && apt-get install -y --no-install-recommends sudo && rm -rf /var/lib/apt/lists/* \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# build/ and install/ live in a named volume (see compose.yaml) so the slow
# NTFS bind mount never sees thousands of small build artifacts.
RUN mkdir -p /ws/src /ws/build /ws/install /ws/log \
 && chown -R ${UID}:${GID} /ws

# Remote visualisation: rosbridge exposes ROS topics as a JSON WebSocket, which
# a browser can read directly. Deliberately a SEPARATE apt layer so adding it
# does not invalidate the big ROS/Gazebo layer above on rebuild.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

# Source ROS for EVERY shell, not just the entrypoint.
#
# `docker exec` bypasses ENTRYPOINT, and `bash -lc` (what the Makefile uses)
# reads /etc/profile but NOT ~/.bashrc -- Ubuntu's default .bashrc returns
# early for non-interactive shells. So without this, `docker compose exec sim
# bash -lc "ros2 ..."` fails with "command not found" even though everything is
# installed. profile.d is read by login shells regardless of interactivity.
#
# Note: Jazzy's ros_gz pulls VENDORED Gazebo (gz-sim-vendor), so the `gz` CLI
# lives inside /opt/ros/jazzy, not /usr/bin. It does not exist until sourced.
RUN printf '%s\n' \
      'source /opt/ros/jazzy/setup.bash' \
      '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' \
      'export GZ_SIM_RESOURCE_PATH="/ws/src/dsim_description/models:/ws/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"' \
      > /etc/profile.d/10-ros.sh \
 && chmod 0644 /etc/profile.d/10-ros.sh \
 && cat /etc/profile.d/10-ros.sh >> /home/${USERNAME}/.bashrc \
 && chown ${UID}:${GID} /home/${USERNAME}/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
WORKDIR /ws
ENV QDRONE_WS=/ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
