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

# ---- planner toolchain: Rust + maturin, so bezier_opt builds in here -------
# The space-time planner (banana99ou/bezier-trajectory) has no Python solver
# path: its optimiser is a Rust extension (pyo3, clarabel) and without it
# optimize_spacetime() raises. Building it needs cargo and maturin, and they
# need the network at `make planner` time -- through whatever CA the local
# network insists on (docker/ca/README.md). Separate layer, same reason as
# the one above: the big Gazebo layer must not rebuild for this.
COPY ca/ /usr/local/share/ca-certificates/extra/
RUN update-ca-certificates \
 && apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-venv build-essential curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*
ENV SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt \
    PIP_CERT=/etc/ssl/certs/ca-certificates.crt \
    RUSTUP_HOME=/opt/rustup \
    CARGO_HOME=/opt/cargo
RUN curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --no-modify-path \
 && chmod -R a+rwX /opt/rustup /opt/cargo \
 && pip install --break-system-packages maturin==1.8.3
ENV PATH="/opt/cargo/bin:${PATH}"

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
      'export PYTHONPATH="/ws/install/planner_py:/ws/planner_src${PYTHONPATH:+:$PYTHONPATH}"' \
      'export CARGO_HOME=/ws/build/cargo' \
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
