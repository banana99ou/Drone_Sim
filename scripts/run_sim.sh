#!/usr/bin/env bash
# Start the simulator and the remote viewer. Run on the HOST.
#
#   scripts/run_sim.sh                                  # empty world, 1 m circle
#   WORLD=empty REFERENCE=lemniscate scripts/run_sim.sh
#   RADIUS=2.0 PERIOD=6 scripts/run_sim.sh
#   WORLD=fence3d PLAN=plans/fence3d_seed.json scripts/run_sim.sh   # fly a plan
#   WORLD=loiter PLAN=plans/loiter_N8_seg16.json STATE=truth scripts/run_sim.sh
#
# PERIOD is the knob that decides whether there is anything to watch. A circle
# needs bank = atan(4*pi^2*r / (T^2*g)), so the old 12 s lap on a 1 m circle
# was 1.6 degrees of bank: the control stack was idling, and the overlay
# arrows correctly showed almost nothing happening. The default below is a
# 3.5 s lap -- 18 degrees of bank, thrust visibly split across the diagonal,
# and still well inside the 40 degree tilt clamp.
#
# Why it is shaped like this: the launch is backgrounded HOST-side with nohup
# around `docker compose exec`, NOT with `docker compose exec -d`. The detached
# form looked fine and then silently lost its ROS nodes minutes later, leaving
# gz and rosbridge alive so the viewer kept serving a frozen drone. Backgrounding
# on the host keeps one owning process we can see and signal.
#
# It also WAITS for the sim to actually publish before claiming success, so
# "started" means observed, not hoped.
#
# TWO launches, not one. The viewer is viz.launch.py and outlives the
# simulator; sim.launch.py is the part that gets cycled. That split is what
# lets the viewer's scenario dropdown restart the simulator without killing
# the server that was asked to do it -- see viz.launch.py's docstring. So this
# script starts the viewer first, waits for it to answer, and only then brings
# a run up underneath it.
set -o pipefail
cd "$(dirname "$0")/.."

WORLD="${WORLD:-empty}"
REFERENCE="${REFERENCE:-circle}"
RADIUS="${RADIUS:-1.0}"
PERIOD="${PERIOD:-3.5}"
ALTITUDE="${ALTITUDE:-1.5}"
GUI="${GUI:-false}"
# A space-time plan to fly (path relative to the repo, which is /ws inside).
# The plan names its scenario and WORLD must be that scenario; the launch
# refuses anything else.
PLAN="${PLAN:-}"
# What the controller flies on: est (the sensor-based estimate) or truth.
# loiter needs truth -- it flies at 62.5 m and the optical flow aids velocity
# only below 3 m. The viewer's dropdown works this out per run from the plan's
# own altitude; from here you have to say. See docs/PLANNER.md.
STATE="${STATE:-est}"
WEB_PORT="${WEB_PORT:-8080}"
# Interface the viewer binds to. Set it to your Tailscale IP to keep the port
# off the local LAN.
BIND="${BIND:-0.0.0.0}"
COMPOSE="docker compose -f docker/compose.yaml"
LOG="logs/sim.log"
PIDFILE="logs/.sim.pid"
VIZ_LOG="logs/viz.log"
VIZ_PIDFILE="logs/.viz.pid"

mkdir -p logs

if ! $COMPOSE ps --status running 2>/dev/null | grep -q drone_sim; then
  echo "==> container not running, starting it"
  $COMPOSE up -d >/dev/null || exit 1
fi

echo "==> clearing anything left over"
$COMPOSE exec -T sim bash -lc 'cd /ws && bash scripts/kill_sim.sh' || true
for f in "$PIDFILE" "$VIZ_PIDFILE"; do
  [ -f "$f" ] && kill "$(cat "$f")" 2>/dev/null
  rm -f "$f"
done

# ---- the viewer, which outlives every run it shows -------------------------
echo "==> starting the viewer on :$WEB_PORT"
nohup $COMPOSE exec -T sim bash -lc \
  "cd /ws && exec ros2 launch dsim_bringup viz.launch.py \
     web_port:=$WEB_PORT bind:=$BIND" \
  >"$VIZ_LOG" 2>&1 &
echo $! > "$VIZ_PIDFILE"

# Answering on the port is the proof, not "the process exists": the previous
# viewer may still be releasing the socket, and a viewer that lost the bind
# race fails in exactly the way that looks like a healthy start from outside.
VIZ_UP=0
for i in $(seq 1 30); do
  sleep 1
  if ! kill -0 "$(cat "$VIZ_PIDFILE" 2>/dev/null)" 2>/dev/null; then
    echo "FAIL: the viewer exited. Last lines of $VIZ_LOG:"; tail -20 "$VIZ_LOG"; exit 1
  fi
  if $COMPOSE exec -T sim bash -lc \
       "curl -sf http://127.0.0.1:${WEB_PORT}/snapshot >/dev/null"; then
    VIZ_UP=1; break
  fi
done
if [ "$VIZ_UP" -ne 1 ]; then
  echo "FAIL: the viewer never answered on :$WEB_PORT. Last lines of $VIZ_LOG:"
  tail -20 "$VIZ_LOG"; exit 1
fi

# Report the bank angle the chosen lap implies, so "nothing is happening on
# screen" can be checked against what was actually asked for before anyone
# goes looking for a bug in the controller.
BANK="$(python3 -c "
import math
r, t = $RADIUS, $PERIOD
print(f'{math.degrees(math.atan2(4*math.pi**2*r/t**2, 9.80665)):.0f}')" 2>/dev/null || echo '?')"

PLAN_ARG=""
if [ -n "$PLAN" ]; then
  PLAN_ARG="plan:=/ws/${PLAN#/ws/}"
  echo "==> launching  world=$WORLD plan=$PLAN"
else
  echo "==> launching  world=$WORLD reference=$REFERENCE radius=$RADIUS period=${PERIOD}s"
  echo "    (that lap needs about ${BANK} deg of bank; the tilt clamp is 40)"
fi
nohup $COMPOSE exec -T sim bash -lc \
  "cd /ws && exec ros2 launch dsim_bringup sim.launch.py \
     world:=$WORLD reference:=$REFERENCE radius:=$RADIUS altitude:=$ALTITUDE \
     period:=$PERIOD $PLAN_ARG \
     gui:=$GUI viz:=true state:=$STATE" \
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
echo "           the scenario dropdown switches runs from there -- it restarts"
echo "           the simulator underneath the page, which stays up."
echo "  log:     $LOG          (tail -f to watch)"
echo "  viewer log: $VIZ_LOG"
echo "  stop:    make stop"
