#!/usr/bin/env bash
# Kill every simulator process. Run INSIDE the container.
#
# Two traps this avoids, both of which bit during development:
#
# 1. The patterns live in this FILE, not in a shell one-liner. `pkill -f`
#    matches whole command lines, so typing the pattern into a shell means that
#    shell's own command line contains it and pkill kills the shell (exit 137).
#
# 2. It excludes its own ancestry. Even from a file, the CALLER's command line
#    may legitimately mention "ros2 launch dsim_bringup" -- and pkill would
#    then kill the caller. So ancestor PIDs are computed and skipped.
set -o pipefail

# rosbridge_websocket and http.server belong here too: they were missed at
# first, survived a 'cleared' teardown, and the next run's rosbridge then
# died with 'Address already in use' while the viewer silently showed data
# from the previous sim.
PATTERNS='gz sim|gz_tools_vendor|controller_node|eval_node|reference_generator_node|parameter_bridge|rosbridge_websocket|http.server|viz_server|viz_relay_node|ros2 launch dsim_bringup'

# self, parent, grandparent, ... up to init
ancestors() {
  local p=$$
  while [ -n "$p" ] && [ "$p" -gt 1 ] 2>/dev/null; do
    printf '%s\n' "$p"
    p="$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')"
  done
}

EXCLUDE=" $(ancestors | tr '\n' ' ') "

kill_pass() {
  local sig="$1" n=0
  for pid in $(pgrep -f "$PATTERNS" 2>/dev/null); do
    case "$EXCLUDE" in
      *" $pid "*) continue ;;          # never kill ourselves or our caller
    esac
    kill "$sig" "$pid" 2>/dev/null && n=$((n + 1))
  done
  printf '%s\n' "$n"
}

killed="$(kill_pass -TERM)"
sleep 2
kill_pass -KILL >/dev/null
sleep 1

remaining=0
for pid in $(pgrep -f "$PATTERNS" 2>/dev/null); do
  case "$EXCLUDE" in *" $pid "*) continue ;; esac
  remaining=$((remaining + 1))
done

if [ "$remaining" -gt 0 ]; then
  echo "still alive after SIGKILL:"
  pgrep -af "$PATTERNS"
  exit 1
fi
echo "cleared ${killed} simulator process(es)"
