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
      "$1/src/mixer.cpp" "$1/src/se3_controller.cpp" \
      "$1/src/body_rate_source.cpp" "$1/test/test_control.cpp" \
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

skipped=0

echo "baseline (unmutated):"
baseline="$(build_and_run src/dsim_control)"
echo "  $baseline  (expected: ALL_PASS)"
if [ "$baseline" != "ALL_PASS" ]; then
  # Without this guard every mutation below is "caught" by the same build
  # failure, and the script reports teeth it does not have. That happened the
  # moment a new source file was added to the package and not to the compile
  # line above.
  echo "  BASELINE IS NOT GREEN -- every result below would be meaningless."
  tail -20 "$WORK/build.log" 2>/dev/null
  exit 1
fi
echo
echo "injected bugs:"
survivors=0

# A mutation whose pattern no longer matches is NOT a pass: nothing was tested.
# Counting it as success would make this script report a green run for code it
# never touched -- a check that cannot fail.
run_mutation() {
  mutate "$@"
  local rc=$?
  [ $rc -eq 2 ] && survivors=$((survivors+1))
  [ $rc -eq 1 ] && skipped=$((skipped+1))
  return 0
}

run_mutation "yaw reaction-torque sign flipped" \
  "src/mixer.cpp" \
  "alloc_(3, i) = -kLayout[i].spin * c;" \
  "alloc_(3, i) = kLayout[i].spin * c;"

run_mutation "roll and pitch rows swapped" \
  "src/mixer.cpp" \
  "    alloc_(1, i) = r.y();                      // tau_x = sum  y_i f_i
    alloc_(2, i) = -r.x();                     // tau_y = -sum x_i f_i" \
  "    alloc_(1, i) = -r.x();
    alloc_(2, i) = r.y();"

run_mutation "two rotors swapped in the layout table" \
  "include/dsim_control/mixer.hpp" \
  "    {-1.0, -1.0, +1.0},        // 1  back-right,  ccw
    {+1.0, -1.0, -1.0},        // 2  front-right, cw" \
  "    {+1.0, -1.0, -1.0},
    {-1.0, -1.0, +1.0},"

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
  "arm_offset_ = arm_length_m / std::sqrt(2.0);" \
  "arm_offset_ = 0.1;"

run_mutation "rotor position reported without its y offset" \
  "src/mixer.cpp" \
  "return Eigen::Vector3d(g.sx * arm_offset_, g.sy * arm_offset_, 0.0);" \
  "return Eigen::Vector3d(g.sx * arm_offset_, 0.0, 0.0);"

run_mutation "moment constant ignored in allocation" \
  "src/mixer.cpp" \
  "  const double c = moment_constant_m;" \
  "  const double c = 0.05;"

# ---------------------------------------------------------------------------
# The overlay geometry the browser draws lives on the ROS side (all logic does),
# so it gets the same treatment. An arrow pointing the wrong way is worse than a
# blank screen: it is confidently wrong, and it sends you looking for the bug in
# the controller instead of in the picture.
# ---------------------------------------------------------------------------
mutate_py() {       # $1 = name, $2 = from, $3 = to
  local name="$1" from="$2" to="$3"
  rm -rf "$WORK/viz"; cp -r src/dsim_viz "$WORK/viz"
  rm -rf "$WORK/viz/dsim_viz/__pycache__"
  local target="$WORK/viz/dsim_viz/overlay.py"
  if ! grep -qF -- "$from" "$target"; then
    echo "  [SKIP]     $name  (pattern not found -- code changed, update this script)"
    return 1
  fi
  python3 -c '
import sys
p, a, b = sys.argv[1], sys.argv[2], sys.argv[3]
t = open(p).read()
open(p, "w").write(t.replace(a, b, 1))
' "$target" "$from" "$to"
  if PYTHONPATH="$WORK/viz" python3 -m pytest -x -q "$WORK/viz/test/test_overlay.py" \
       >"$WORK/py.log" 2>&1; then
    echo "  [SURVIVED] $name  <-- BUG: this error would pass the test suite"
    return 2
  fi
  echo "  [CAUGHT]   $name  -> $(grep -oE '[0-9]+ failed' "$WORK/py.log" | head -1)"
  return 0
}

run_py_mutation() {
  mutate_py "$@"
  local rc=$?
  [ $rc -eq 2 ] && survivors=$((survivors+1))
  [ $rc -eq 1 ] && skipped=$((skipped+1))
  return 0
}

echo
echo "baseline (unmutated overlay geometry):"
rm -rf "$WORK/viz"; cp -r src/dsim_viz "$WORK/viz"
rm -rf "$WORK/viz/dsim_viz/__pycache__"
if PYTHONPATH="$WORK/viz" python3 -m pytest -q "$WORK/viz/test/test_overlay.py" \
     >"$WORK/py.log" 2>&1; then
  echo "  ALL_PASS  (expected)"
else
  echo "  BASELINE FAILS -- fix that before trusting anything below"
  tail -5 "$WORK/py.log"
  exit 1
fi

echo
echo "injected overlay bugs:"

run_py_mutation "body vectors not rotated into the world" \
  "base = _add(p, rotate(m, hub))" \
  "base = _add(p, hub)"

run_py_mutation "arrow index decoupled from its rotor" \
  "    for i, hub in enumerate(hubs):" \
  "    for i, hub in enumerate(list(reversed(hubs))):"

run_py_mutation "aerodynamic residual sign flipped" \
  "return _sub(measured, [0.0, 0.0, control['realised_thrust_n']])" \
  "return _sub([0.0, 0.0, control['realised_thrust_n']], measured)"

run_py_mutation "mass dropped from the measured force" \
  "measured = _mul(imu['accel_body'], control['mass_kg'])" \
  "measured = _mul(imu['accel_body'], 1.0)"

run_py_mutation "weight arrow points up" \
  "'forces', 'weight', p, [0.0, 0.0, -weight * scale_f]," \
  "'forces', 'weight', p, [0.0, 0.0, weight * scale_f],"

run_py_mutation "world-frame velocity rotated a second time" \
  "'velocity', 'velocity', p, _mul(v, SCALE['vel_m_per_mps'])," \
  "'velocity', 'velocity', p, _mul(rotate(m, v), SCALE['vel_m_per_mps']),"

run_py_mutation "hover deviation loses its sign" \
  "norm=round(max(-1.0, min(1.0, (f - hover_each) / hover_each))," \
  "norm=round(max(-1.0, min(1.0, abs(f - hover_each) / hover_each)),"

run_py_mutation "quaternion transposed (body and world swapped)" \
  "        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w))," \
  "        (1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)),"

echo

# ---------------------------------------------------------------------------
# The sensor models get the same treatment, and for a specific reason: a cold
# review found that the optical-flow model generated its reading from the
# MEASURED range and gyro and then reported those same values, so the
# consumer's documented reconstruction cancelled them algebraically. A
# rangefinder reading 2.5 m instead of 1.5 m still recovered the velocity to
# twelve decimal places. Every test passed, because the tests asserted an
# algebraic identity of the model rather than a property of a sensor.
#
# The first two mutations below reintroduce exactly that bug. If either
# survives, the tests have gone back to testing nothing.
# ---------------------------------------------------------------------------
build_and_run_sensors() {   # $1 = source dir
  g++ -std=c++17 -O1 -I "$1/include" -I /usr/include/eigen3 \
      "$1/test/test_sensors.cpp" \
      -lgtest -lgtest_main -pthread -o "$WORK/ts" 2>"$WORK/build.log" \
      || { echo "BUILD_FAIL"; return; }
  "$WORK/ts" >"$WORK/run.log" 2>&1 && echo "ALL_PASS" || echo "SOME_FAIL"
}

mutate_sensors() {  # $1 = name, $2 = file (relative), $3 = from, $4 = to
  local name="$1" file="$2" from="$3" to="$4"
  rm -rf "$WORK/sens"; cp -r src/dsim_sensors "$WORK/sens"
  if ! grep -qF -- "$from" "$WORK/sens/$file"; then
    echo "  [SKIP]     $name  (pattern not found -- code changed, update this script)"
    return 1
  fi
  python3 -c '
import sys
p, a, b = sys.argv[1], sys.argv[2], sys.argv[3]
t = open(p).read()
open(p, "w").write(t.replace(a, b, 1))
' "$WORK/sens/$file" "$from" "$to"
  local result; result="$(build_and_run_sensors "$WORK/sens")"
  if [ "$result" = "SOME_FAIL" ]; then
    local caught; caught="$(grep -c '^\[  FAILED  \] [A-Za-z]' "$WORK/run.log")"
    echo "  [CAUGHT]   $name  -> $caught test(s) failed"
    return 0
  elif [ "$result" = "BUILD_FAIL" ]; then
    echo "  [CAUGHT]   $name  -> did not compile"
    return 0
  fi
  echo "  [SURVIVED] $name  <-- BUG: this error would pass the test suite"
  return 2
}

run_sensor_mutation() {
  mutate_sensors "$@"
  local rc=$?
  [ $rc -eq 2 ] && survivors=$((survivors+1))
  [ $rc -eq 1 ] && skipped=$((skipped+1))
  return 0
}

echo
echo "baseline (unmutated sensor models):"
echo "  $(build_and_run_sensors src/dsim_sensors)  (expected: ALL_PASS)"
echo
echo "injected sensor bugs:"

run_sensor_mutation "flow driven by the MEASURED range (the original bug)" \
  "include/dsim_sensors/optical_flow.hpp" \
  "  const double flow_rate_x = -in.v_body.y() / in.true_range_m + in.omega_true.x();
  const double flow_rate_y = in.v_body.x() / in.true_range_m + in.omega_true.y();" \
  "  const double flow_rate_x = -in.v_body.y() / in.measured_range_m + in.omega_true.x();
  const double flow_rate_y = in.v_body.x() / in.measured_range_m + in.omega_true.y();"

run_sensor_mutation "flow driven by the MEASURED gyro, x axis" \
  "include/dsim_sensors/optical_flow.hpp" \
  "+ in.omega_true.x();" \
  "+ in.omega_measured.x();"

run_sensor_mutation "flow driven by the MEASURED gyro, y axis" \
  "include/dsim_sensors/optical_flow.hpp" \
  "+ in.omega_true.y();" \
  "+ in.omega_measured.y();"

run_sensor_mutation "flow x sign flipped" \
  "include/dsim_sensors/optical_flow.hpp" \
  "const double flow_rate_x = -in.v_body.y() / in.true_range_m" \
  "const double flow_rate_x = in.v_body.y() / in.true_range_m"

run_sensor_mutation "flow not divided by height" \
  "include/dsim_sensors/optical_flow.hpp" \
  "const double flow_rate_y = in.v_body.x() / in.true_range_m" \
  "const double flow_rate_y = in.v_body.x()"

run_sensor_mutation "unusable reading keeps a confident quality" \
  "include/dsim_sensors/optical_flow.hpp" \
  "    s.quality = 0;
  }
  if (s.quality == 0) {" \
  "    s.quality = s.quality;
  }
  if (s.quality == 0) {"

run_sensor_mutation "rangefinder reports altitude, not slant range" \
  "include/dsim_sensors/tof.hpp" \
  "  return altitude_m / cos_tilt;" \
  "  return altitude_m;"

run_sensor_mutation "rangefinder error stops growing with distance" \
  "include/dsim_sensors/tof.hpp" \
  "  const double sigma = c.noise_m + c.noise_frac * true_range;" \
  "  const double sigma = c.noise_m;"

run_sensor_mutation "rangefinder saturation clamped instead of infinite" \
  "include/dsim_sensors/tof.hpp" \
  "  if (measured > c.max_range_m) {return kTooFar;}" \
  "  if (measured > c.max_range_m) {return c.max_range_m;}"

run_sensor_mutation "body rate uses the naive, sheet-sensitive formulation" \
  "include/dsim_sensors/rates.hpp" \
  "  const Eigen::AngleAxisd aa(dq);
  return aa.axis() * (aa.angle() / dt);" \
  "  return 2.0 * dq.vec() / dt;"

echo
if [ "$skipped" -ne 0 ]; then
  echo "RESULT: $skipped mutation(s) could not be applied -- their patterns no"
  echo "        longer match the source, so those bugs went UNTESTED. Update the"
  echo "        patterns in this script; do not read the rest as a pass."
  exit 1
fi
if [ "$survivors" -eq 0 ]; then
  echo "RESULT: every injected bug was caught. The suite has teeth."
else
  echo "RESULT: $survivors mutation(s) SURVIVED -- the suite does not cover them."
  exit 1
fi
