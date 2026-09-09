#!/usr/bin/env bash
# Start the simulator and the remote viewer. Run on the HOST.
#
#   scripts/run_sim.sh                                  # pillars, 1 m circle
#   WORLD=empty REFERENCE=lemniscate scripts/run_sim.sh
#   RADIUS=2.0 scripts/run_sim.sh
#
# Why it is shaped like this: the launch is backgrounded HOST-side with nohup
# around `docker compose exec`, NOT with `docker compose exec -d`. The detached
# form looked fine and then silently lost its ROS nodes minutes later, leaving
# gz and rosbridge alive so the viewer kept serving a frozen drone. Backgrounding
# on the host keeps one owning process we can see and signal.
#
# It also WAITS for the sim to actually publish before claiming success, so
# "started" means observed, not hoped.
set -o pipefail
cd "$(dirname "$0")/.."

WORLD="${WORLD:-pillars}"
REFERENCE="${REFERENCE:-circle}"
RADIUS="${RADIUS:-1.0}"
ALTITUDE="${ALTITUDE:-1.5}"
GUI="${GUI:-false}"
WEB_PORT="${WEB_PORT:-8080}"
COMPOSE="docker compose -f docker/compose.yaml"
LOG="logs/sim.log"
PIDFILE="logs/.sim.pid"

mkdir -p logs

if ! $COMPOSE ps --status running 2>/dev/null | grep -q drone_sim; then
  echo "==> container not running, starting it"
  $COMPOSE up -d >/dev/null || exit 1
fi

echo "==> clearing anything left over"
$COMPOSE exec -T sim bash -lc 'cd /ws && bash scripts/kill_sim.sh' || true
[ -f "$PIDFILE" ] && kill "$(cat "$PIDFILE")" 2>/dev/null
rm -f "$PIDFILE"

echo "==> launching  world=$WORLD reference=$REFERENCE radius=$RADIUS gui=$GUI"
nohup $COMPOSE exec -T sim bash -lc \
  "cd /ws && exec ros2 launch dsim_bringup sim.launch.py \
     world:=$WORLD reference:=$REFERENCE radius:=$RADIUS altitude:=$ALTITUDE \
     gui:=$GUI viz:=true web_port:=$WEB_PORT" \
  >"$LOG" 2>&1 &
echo $! > "$PIDFILE"

echo "==> waiting for the sim to publish (this is the actual proof it is up)"
UP=0
for i in $(seq 1 40); do
  sleep 2
  if ! kill -0 "$(cat "$PIDFILE" 2>/dev/null)" 2>/dev/null; then
    echo "FAIL: the launch exited. Last lines of $LOG:"
    tail -25 "$LOG"
    exit 1
  fi
  if $COMPOSE exec -T sim bash -lc \
       'timeout 4 ros2 topic echo /drone/eval/status dsim_msgs/msg/FlightStatus --once' \
       >/dev/null 2>&1; then
    UP=1; break
  fi
done

if [ "$UP" -ne 1 ]; then
  echo "FAIL: no /drone/eval/status after ~80 s. Last lines of $LOG:"
  tail -25 "$LOG"
  exit 1
fi

# Prefer the Tailscale address: that is the one reachable from the MacBook.
IP="$(tailscale ip -4 2>/dev/null | head -1)"
[ -z "$IP" ] && IP="$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -m1 '^100\.')"
[ -z "$IP" ] && IP="$(hostname -I 2>/dev/null | awk '{print $1}')"

echo
echo "  viewer:  http://${IP}:${WEB_PORT}"
echo "  log:     $LOG          (tail -f to watch)"
echo "  stop:    make stop"
