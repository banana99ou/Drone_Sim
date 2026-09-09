#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
# The workspace overlay only exists after the first build.
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
# Make our models and worlds discoverable by Gazebo.
export GZ_SIM_RESOURCE_PATH="/ws/src/dsim_description/models:/ws/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
exec "$@"
