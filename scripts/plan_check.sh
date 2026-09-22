#!/usr/bin/env bash
# Fly a space-time plan headless and grade it. Run INSIDE the container.
#
#   bash scripts/plan_check.sh                          # plans/fence3d_seed.json
#   PLAN=plans/fence3d.json bash scripts/plan_check.sh  # an optimised plan
#
# The world is the plan's scenario (the launch refuses anything else). The
# grading is scripts/check_planner.py, which is started BEFORE the scenario
# clock reaches zero so it can confirm the vehicle was at the start point, then
# watches the whole window. The seed plan is expected to collide, and says
# where; a plan that declares no expectation must not.
# NOT `set -u`: ROS 2's setup.bash references unbound variables and dies under it.
set -o pipefail

PLAN="${PLAN:-plans/fence3d_seed.json}"
STATE="${STATE:-est}"
WEB_PORT="${WEB_PORT:-8080}"

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
cd /ws

SCENARIO="$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['scenario'])" "$PLAN")" \
  || { echo "FAIL: cannot read scenario from $PLAN"; exit 1; }
START_S="$(python3 -c "import json,sys; print(json.load(open(f'scenarios/{sys.argv[1]}.json'))['sim']['start_s'])" "$SCENARIO")"

SIM_PROCS='gz sim|controller_node|eval_node|reference_generator_node|parameter_bridge|bridge_node'
kill_leftovers() { bash "$(dirname "$0")/kill_sim.sh" >/dev/null 2>&1; }

if pgrep -f "$SIM_PROCS" >/dev/null 2>&1; then
  echo "== found a running sim, clearing it =="
  kill_leftovers
fi
if pgrep -f "$SIM_PROCS" >/dev/null 2>&1; then
  echo "FAIL: could not clear existing sim processes:"; pgrep -af "$SIM_PROCS" | head; exit 1
fi

echo "== launching ($SCENARIO, plan $PLAN, state=$STATE, headless; scenario t=0 at sim ${START_S}s) =="
setsid ros2 launch dsim_bringup sim.launch.py \
    world:="$SCENARIO" plan:="/ws/$PLAN" gui:=false web_port:="$WEB_PORT" state:="$STATE" \
    >/tmp/plan_check.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -INT -"$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null
  for _ in $(seq 1 8); do kill -0 "$LAUNCH_PID" 2>/dev/null || return 0; sleep 1; done
  kill -KILL -"$LAUNCH_PID" 2>/dev/null
  kill_leftovers
  return 0
}
trap cleanup EXIT

# Wait for the referee's report to appear, then hand over to the grader while
# the scenario clock is still negative. The grader refuses to start late.
echo "== waiting for the referee =="
for _ in $(seq 1 60); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "FAIL: launch died early. Last lines:"; tail -30 /tmp/plan_check.log; exit 1
  fi
  if curl -sf "http://127.0.0.1:${WEB_PORT}/snapshot" 2>/dev/null \
       | python3 -c 'import json,sys; s=json.load(sys.stdin); sys.exit(0 if s.get("clearance") and s.get("plan_path") else 1)' 2>/dev/null; then
    break
  fi
  sleep 1
done

# Exactly one publisher, so a stale sim cannot be the thing we grade. Retried:
# a fresh `ros2` process has to complete DDS discovery before it can answer,
# and asking once reported 0 publishers on a launch that was perfectly healthy
# -- the referee was already streaming to the viewer at the time.
PUBS=0
for _ in $(seq 1 10); do
  PUBS="$(timeout 20 ros2 topic info /drone/eval/clearance 2>/dev/null | sed -n 's/^Publisher count: //p')"
  [ "${PUBS:-0}" -ge 1 ] 2>/dev/null && break
  sleep 2
done
if [ "${PUBS:-0}" != "1" ]; then
  echo "FAIL: expected exactly 1 publisher on /drone/eval/clearance, found ${PUBS:-0}."
  echo "      0 means the referee never came up; more than 1 means a stale sim is"
  echo "      alive and any number sampled here could come from the wrong vehicle."
  pgrep -af "$SIM_PROCS" | head
  exit 1
fi

echo "== grading =="
python3 scripts/check_planner.py "$PLAN" "http://127.0.0.1:${WEB_PORT}"
rc=$?
echo
if [ "$rc" -eq 0 ]; then
  echo "PASS: the plan flew as its file says it should."
  echo "  (this would have failed on: obstacles not where the referee thinks, the"
  echo "   vehicle not at the start on time, feedforward not reaching the controller,"
  echo "   tracking loss, or hits that differ from the plan's own prediction)"
else
  echo "FAIL: $rc check(s) failed (see above). Launch log: /tmp/plan_check.log"
fi
exit "$rc"
