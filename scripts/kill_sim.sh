#!/usr/bin/env bash
# Kill every simulator process. Run INSIDE the container.
#
# Three traps this avoids, all of which bit during development:
#
# 1. The patterns live in this FILE, not in a shell one-liner. `pkill -f`
#    matches whole command lines, so typing the pattern into a shell means that
#    shell's own command line contains it and pkill kills the shell (exit 137).
#
# 2. It excludes its own ancestry. Even from a file, the CALLER's command line
#    may legitimately mention "ros2 launch dsim_bringup" -- and pkill would
#    then kill the caller. So ancestor PIDs are computed and skipped.
set -o pipefail

# 3. The list of THIS workspace's nodes is DERIVED, not typed.
#
#    It used to be a hand-written list of executable names, and the failure was
#    exactly what a hand-written list does: a new node was added to the launch
#    file and not to the list, so every teardown left it running. Three copies
#    of the simulation-control node accumulated, all pacing the same world at
#    once, and the world ran at two and three times the requested speed. The
#    measurements taken to explain that were all wrong, because the thing
#    causing it was not in any of them.
#
#    So the workspace's own install tree is the source of truth: whatever
#    colcon installed as an executable is a simulator process. Adding a package
#    can no longer be forgotten here, because nothing here has to be edited.
#    (-type l as well as -type f: colcon --symlink-install installs them as
#    symlinks into the build tree, and a -type f test finds nothing at all --
#    which would silently restore the original bug in a quieter form.)
ws_executables() {
  find /ws/install -mindepth 4 -maxdepth 4 -path '*/lib/*' \
       \( -type f -o -type l \) -perm -u+x -printf '%f\n' 2>/dev/null | sort -u
}

# Everything that is NOT ours, and so cannot be derived: Gazebo, its tools, the
# ros_gz bridge, and the launch process that started them all.
EXTERNAL='gz sim|gz_tools_vendor|parameter_bridge|ros2 launch dsim_bringup'
PATTERNS="$EXTERNAL$(ws_executables | sed 's/^/|/' | tr -d '\n')"

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
