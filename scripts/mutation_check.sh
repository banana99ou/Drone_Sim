#!/usr/bin/env bash
# Mutation check: deliberately inject known bugs and confirm the test suite
# catches each one.
#
# A passing test suite is only evidence if it would have failed on a wrong
# implementation. This script proves that by breaking the code on purpose. Each
# mutation below is a REAL bug someone could plausibly introduce -- a flipped
# sign, a swapped axis, a dropped feedforward term.
#
# Expected result: every mutation is CAUGHT. A SURVIVED mutation means that bug
# could be merged with a green test run.
set -uo pipefail
cd "$(dirname "$0")/.."

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
INC="-I src/dsim_control/include -I /usr/include/eigen3"

build_and_run() {   # $1 = source dir
  g++ -std=c++17 -O1 -I "$1/include" -I /usr/include/eigen3 \
      "$1/src/mixer.cpp" "$1/src/se3_controller.cpp" "$1/test/test_control.cpp" \
      -lgtest -pthread -o "$WORK/t" 2>"$WORK/build.log" || { echo "BUILD_FAIL"; return; }
  "$WORK/t" >"$WORK/run.log" 2>&1 && echo "ALL_PASS" || echo "SOME_FAIL"
}

mutate() {          # $1 = name, $2 = file, $3 = from, $4 = to
  local name="$1" file="$2" from="$3" to="$4"
  rm -rf "$WORK/src"; cp -r src/dsim_control "$WORK/src"
  if ! grep -qF -- "$from" "$WORK/src/$file"; then
    echo "  [SKIP]     $name  (pattern not found -- code changed, update this script)"
    return 1
  fi
  python3 - "$WORK/src/$file" "$from" "$to" <<'PY'
import sys
p, a, b = sys.argv[1], sys.argv[2], sys.argv[3]
t = open(p).read()
open(p, "w").write(t.replace(a, b, 1))
PY
  local result; result="$(build_and_run "$WORK/src")"
  if [ "$result" = "SOME_FAIL" ]; then
    local caught; caught="$(grep '^\[  FAILED  \] [A-Za-z]' "$WORK/run.log" | sed 's/ ([0-9]* ms)$//' | sort -u | wc -l)"
    echo "  [CAUGHT]   $name  -> $caught test(s) failed"
    grep '^\[  FAILED  \] [A-Za-z]' "$WORK/run.log" | sed 's/ ([0-9]* ms)$//' | sort -u | sed 's/^/               /'
    return 0
  elif [ "$result" = "BUILD_FAIL" ]; then
    echo "  [CAUGHT]   $name  -> did not compile"
    return 0
  else
    echo "  [SURVIVED] $name  <-- BUG: this error would pass the test suite"
    return 2
  fi
}

echo "baseline (unmutated):"
echo "  $(build_and_run src/dsim_control)  (expected: ALL_PASS)"
echo
echo "injected bugs:"
survivors=0

run_mutation() { mutate "$@"; [ $? -eq 2 ] && survivors=$((survivors+1)); return 0; }

run_mutation "yaw reaction-torque sign flipped" \
  "src/mixer.cpp" \
  "            -c,   -c,    c,    c;" \
  "             c,    c,   -c,   -c;"

run_mutation "roll and pitch rows swapped" \
  "src/mixer.cpp" \
  "             d,   -d,   -d,    d,        // tau_x = sum  y_i f_i
            -d,    d,   -d,    d,        // tau_y = -sum x_i f_i" \
  "            -d,    d,   -d,    d,
             d,   -d,   -d,    d,"

run_mutation "acceleration feedforward dropped" \
  "src/se3_controller.cpp" \
  "+ gains_.mass * ref.acceleration;" \
  "+ 0.0 * gains_.mass * ref.acceleration;"

run_mutation "position error sign flipped" \
  "src/se3_controller.cpp" \
  "const Eigen::Vector3d e_p = state.position - ref.position;" \
  "const Eigen::Vector3d e_p = ref.position - state.position;"

run_mutation "gravity compensation removed" \
  "src/se3_controller.cpp" \
  "+ gains_.mass * gains_.gravity * e3" \
  "+ 0.0 * gains_.mass * gains_.gravity * e3"

run_mutation "arm length ignored in allocation" \
  "src/mixer.cpp" \
  "const double d = arm_length_m / std::sqrt(2.0);" \
  "const double d = 0.1;"

run_mutation "moment constant ignored in allocation" \
  "src/mixer.cpp" \
  "  const double c = moment_constant_m;" \
  "  const double c = 0.05;"

echo
if [ "$survivors" -eq 0 ]; then
  echo "RESULT: every injected bug was caught. The suite has teeth."
else
  echo "RESULT: $survivors mutation(s) SURVIVED -- the suite does not cover them."
  exit 1
fi
