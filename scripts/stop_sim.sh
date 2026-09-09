#!/usr/bin/env bash
# Stop the simulator and the viewer. Run on the HOST.
set -o pipefail
cd "$(dirname "$0")/.."

COMPOSE="docker compose -f docker/compose.yaml"
PIDFILE="logs/.sim.pid"

if [ -f "$PIDFILE" ]; then
  kill "$(cat "$PIDFILE")" 2>/dev/null && echo "==> signalled the host-side launch"
  rm -f "$PIDFILE"
fi

# The container-side teardown is the one that matters: ros2 launch orphans the
# gz server, and rosbridge/http.server outlive it too.
if $COMPOSE ps --status running 2>/dev/null | grep -q drone_sim; then
  $COMPOSE exec -T sim bash -lc 'cd /ws && bash scripts/kill_sim.sh'
else
  echo "container is not running; nothing to stop"
fi
