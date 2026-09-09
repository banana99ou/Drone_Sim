#!/usr/bin/env bash
# Is the sim actually up, and is it publishing? Run on the HOST.
set -o pipefail
cd "$(dirname "$0")/.."
COMPOSE="docker compose -f docker/compose.yaml"

if ! $COMPOSE ps --status running 2>/dev/null | grep -q drone_sim; then
  echo "container:  not running"
  exit 1
fi
echo "container:  up"

echo -n "processes:  "
$COMPOSE exec -T sim bash -lc \
  'ps -eo args | grep -cE "controller_node|eval_node|reference_generator|parameter_bridge|viz_server"' 2>/dev/null

echo "publishing:"
$COMPOSE exec -T sim bash -lc \
  'timeout 8 ros2 topic echo /drone/eval/status dsim_msgs/msg/FlightStatus --once 2>/dev/null \
   | grep -E "collided|tracking_rmse_m|elapsed_s" | sed "s/^/  /"' 2>&1 \
  || echo "  NOT publishing -- the nodes are gone even though the container is up"
