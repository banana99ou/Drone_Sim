#!/usr/bin/env bash
# Teleport the drone back to a start pose, for running repeated planner trials.
#
#   scripts/reset_pose.sh                    # back to origin at 0.1 m
#   scripts/reset_pose.sh -2 0 1.5           # to x=-2, y=0, z=1.5
#   WORLD=drone_fence3d scripts/reset_pose.sh
#
# This talks to Gazebo directly rather than through ROS, because the pose
# service is a gz-transport service with no ROS bridge by default.
#
# THIS IS THE NARROW TOOL. It moves the vehicle and nothing else: the clock
# keeps running, the referee keeps accumulating, and the trajectory keeps
# playing. That is what you want when placing a vehicle for a trial.
#
# To start a whole run again -- vehicle home, clock to zero, metrics cleared --
# use the simulator's reset instead, which every node notices on its own:
#   ros2 service call /sim/control dsim_msgs/srv/SimControl "{command: 3}"
# or press "reset run" in the viewer.
set -euo pipefail

X="${1:-0}"; Y="${2:-0}"; Z="${3:-0.1}"
WORLD="${WORLD:-}"

if [ -z "$WORLD" ]; then
  # Discover the running world rather than assuming which one is up.
  WORLD="$(gz topic -l 2>/dev/null | sed -n 's|^/world/\([^/]*\)/.*|\1|p' | head -1)"
  if [ -z "$WORLD" ]; then
    echo "could not find a running Gazebo world. Is the sim up?" >&2
    exit 1
  fi
fi

echo "resetting drone in world '$WORLD' to ($X, $Y, $Z)"
gz service -s "/world/${WORLD}/set_pose" \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 \
  --req "name: \"drone\", position: {x: $X, y: $Y, z: $Z}, orientation: {w: 1}"
