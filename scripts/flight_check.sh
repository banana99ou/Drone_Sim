#!/usr/bin/env bash
# Headless flight check. Run INSIDE the container.
#
# Flies the built-in circle and asserts the vehicle actually tracked it. This
# is the check that separates "the sim launched" from "the sim flies" -- it
# FAILS if the drone never takes off, drifts, crashes, or oscillates, because
# all of those show up as tracking error or a collision.
#
#   bash scripts/flight_check.sh            # default thresholds
#   RMSE_LIMIT=0.15 bash scripts/flight_check.sh
# NOT `set -u`: ROS 2's setup.bash references unbound variables and dies under it.
set -o pipefail

RMSE_LIMIT="${RMSE_LIMIT:-0.15}"     # metres, measured after the warmup window
MIN_ALT="${MIN_ALT:-1.0}"            # must actually leave the ground
SETTLE_S="${SETTLE_S:-35}"           # 4 s takeoff + 6 s RMSE warmup + ~2 laps
WORLD="${WORLD:-empty}"
REFERENCE="${REFERENCE:-circle}"

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

SIM_PROCS='gz sim|controller_node|eval_node|reference_generator_node|parameter_bridge'

kill_leftovers() {
  # Delegated to a script file on purpose -- see the comment in kill_sim.sh
  # about pkill -f matching its own invoking shell.
  bash "$(dirname "$0")/kill_sim.sh" >/dev/null 2>&1
}

# ---- preflight -------------------------------------------------------------
# A leftover sim from a previous run is not harmless: its nodes publish on the
# same topics, `ros2 topic echo --once` returns whichever arrives first, and the
# check silently grades the WRONG vehicle. That actually happened -- a zombie
# sim whose controller had been killed was lying on the ground, and this script
# reported its 7.5 m tracking error as if it were the new run's.
if pgrep -f "$SIM_PROCS" >/dev/null 2>&1; then
  echo "== found a running sim, clearing it =="
  kill_leftovers
fi
if pgrep -f "$SIM_PROCS" >/dev/null 2>&1; then
  echo "FAIL: could not clear existing sim processes:"
  pgrep -af "$SIM_PROCS" | head
  exit 1
fi

echo "== launching ($WORLD, $REFERENCE, headless) =="
# setsid so the whole launch tree is one process group we can signal as a unit.
setsid ros2 launch dsim_bringup sim.launch.py \
    world:="$WORLD" reference:="$REFERENCE" gui:=false \
    >/tmp/flight_check.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
  # `ros2 launch` does not reliably exit on SIGINT without a tty, and a bare
  # `wait` on it hangs forever -- that is exactly how the first version of this
  # script wedged. Signal the group, give it a bounded grace period, then KILL.
  kill -INT -"$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null
  for _ in $(seq 1 8); do
    kill -0 "$LAUNCH_PID" 2>/dev/null || return 0
    sleep 1
  done
  kill -KILL -"$LAUNCH_PID" 2>/dev/null
  kill_leftovers
  return 0
}
trap cleanup EXIT

echo "== flying for ${SETTLE_S}s =="
END=$((SECONDS + SETTLE_S))
while [ $SECONDS -lt $END ]; do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "FAIL: launch died early. Last lines:"
    tail -30 /tmp/flight_check.log
    exit 1
  fi
  sleep 2
done

# ---- verify we are about to measure exactly one sim ----------------------
PUBS="$(timeout 20 ros2 topic info /drone/eval/status 2>/dev/null \
        | sed -n 's/^Publisher count: //p')"
if [ "${PUBS:-0}" != "1" ]; then
  echo "FAIL: expected exactly 1 publisher on /drone/eval/status, found ${PUBS:-0}."
  echo "      More than one means a stale sim is still alive and any number"
  echo "      sampled here could come from the wrong vehicle."
  pgrep -af "$SIM_PROCS" | head
  exit 1
fi

echo "== sampling (1 publisher confirmed) =="
STATUS="$(timeout 15 ros2 topic echo /drone/eval/status dsim_msgs/msg/FlightStatus --once 2>/dev/null)"
ODOM="$(timeout 15 ros2 topic echo /drone/odom nav_msgs/msg/Odometry --once 2>/dev/null)"

if [ -z "$STATUS" ]; then
  echo "FAIL: no /drone/eval/status published. Last lines:"
  tail -40 /tmp/flight_check.log
  exit 1
fi

get() { echo "$STATUS" | grep -m1 "^$1:" | awk '{print $2}'; }
RMSE="$(get tracking_rmse_m)"
MAXE="$(get tracking_max_m)"
COLLIDED="$(get collided)"
ELAPSED="$(get elapsed_s)"
ALT="$(echo "$ODOM" | grep -A4 '^  pose:' | grep -m1 'z:' | awk '{print $2}')"

echo
echo "  elapsed         ${ELAPSED:-?} s"
echo "  altitude        ${ALT:-?} m"
echo "  tracking rmse   ${RMSE:-?} m   (limit $RMSE_LIMIT)"
echo "  tracking max    ${MAXE:-?} m"
echo "  collided        ${COLLIDED:-?}"
echo

fail=0
awk -v v="${RMSE:-999}" -v l="$RMSE_LIMIT" 'BEGIN{exit !(v+0 <= l+0)}' \
  || { echo "FAIL: tracking RMSE ${RMSE} exceeds ${RMSE_LIMIT} m"; fail=1; }
awk -v v="${ALT:-0}" -v l="$MIN_ALT" 'BEGIN{exit !(v+0 >= l+0)}' \
  || { echo "FAIL: altitude ${ALT} m below ${MIN_ALT} m -- it never took off"; fail=1; }
[ "$COLLIDED" = "false" ] || { echo "FAIL: collision detected"; fail=1; }
# Sanity: elapsed must be consistent with how long we actually flew. A wildly
# larger value means we sampled a sim that started before this script did.
awk -v v="${ELAPSED:-0}" -v l="$((SETTLE_S + 30))" 'BEGIN{exit !(v+0 <= l+0)}' \
  || { echo "FAIL: elapsed ${ELAPSED}s implies a stale sim (flew only ${SETTLE_S}s)"; fail=1; }

if [ "$fail" -eq 0 ]; then
  echo "PASS: took off, tracked the reference, no collision."
  echo "  (this would have failed on: no takeoff, drift, oscillation, or a crash)"
fi
exit "$fail"
