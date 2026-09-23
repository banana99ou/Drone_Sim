#!/usr/bin/env bash
# Stop the simulator and the viewer. Run on the HOST.
set -o pipefail
cd "$(dirname "$0")/.."

COMPOSE="docker compose -f docker/compose.yaml"
# Two host-side launches to signal now, not one: scripts/run_sim.sh starts the
# viewer (viz.launch.py) separately from the simulator, so that the viewer can
# restart the simulator without killing itself. Stopping means stopping both.
for PIDFILE in logs/.sim.pid logs/.viz.pid; do
  if [ -f "$PIDFILE" ]; then
    kill "$(cat "$PIDFILE")" 2>/dev/null && echo "==> signalled $PIDFILE"
    rm -f "$PIDFILE"
  fi
done

# The container-side teardown is the one that matters: ros2 launch orphans the
# gz server, and the viewer is a launch of its own that nothing else signals.
# kill_sim.sh derives its pattern list from the install tree, so viz_server is
# in it automatically -- the only path that spares the viewer is the teardown
# the viewer itself runs, where kill_sim.sh's ancestor exclusion covers it.
if $COMPOSE ps --status running 2>/dev/null | grep -q drone_sim; then
  $COMPOSE exec -T sim bash -lc 'cd /ws && bash scripts/kill_sim.sh'
else
  echo "container is not running; nothing to stop"
fi
