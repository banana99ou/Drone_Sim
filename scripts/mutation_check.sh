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

# What each package contributes to a HOST build, with no ROS and no container.
#
# Listed, not globbed -- some library sources include generated ROS message
# headers that do not exist here -- but the lists are CHECKED against what is
# on disk by audit_sources() below. A source file that is neither compiled nor
# declared untestable is a hard error, because the alternative is what happened
# before: a new file was added to the package and not to this line, the build
# broke, and all 27 mutations were reported "caught" by the same build failure.
sources_dsim_control="src/mixer.cpp src/se3_controller.cpp src/body_rate_source.cpp"
untestable_dsim_control="src/trajectory_buffer.cpp"      # needs dsim_msgs headers
sources_dsim_simctl=""                                   # header-only
untestable_dsim_simctl=""
sources_dsim_time=""                                     # header-only
untestable_dsim_time=""
sources_dsim_estimation="src/attitude_filter.cpp src/velocity_filter.cpp"
untestable_dsim_estimation=""
sources_dsim_eval=""                                     # header-only geometry
untestable_dsim_eval=""                                  # (the nodes are *_node.cpp)

audit_sources() {   # $1 = package
  local pkg="$1" listed found missing=0
  listed=" $(eval echo "\$sources_$pkg") $(eval echo "\$untestable_$pkg") "
  for found in $(cd "src/$pkg" 2>/dev/null && \
                 find src -maxdepth 1 -name '*.cpp' ! -name '*_node.cpp' 2>/dev/null | sort); do
    case "$listed" in
      *" $found "*) ;;
      *) echo "  $pkg/$found is neither compiled nor declared untestable in $0"
         missing=1 ;;
    esac
  done
  return $missing
}

build_and_run() {   # $1 = source dir, $2 = test file relative to it, $3 = package
  local srcs main_lib=""
  srcs="$(eval echo "\$sources_$3")"
  # gtest_main only where the test does not bring its own main(). Linking both
  # is a duplicate-symbol error, and linking neither is an undefined one.
  grep -q "int main" "$1/$2" || main_lib="-lgtest_main"
  local expanded=""
  for f in $srcs; do expanded="$expanded $1/$f"; done
  # shellcheck disable=SC2086
  g++ -std=c++17 -O1 -I "$1/include" -I /usr/include/eigen3 \
      $expanded "$1/$2" \
      $main_lib -lgtest -pthread -o "$WORK/t" 2>"$WORK/build.log" || { echo "BUILD_FAIL"; return; }
  "$WORK/t" >"$WORK/run.log" 2>&1 && echo "ALL_PASS" || echo "SOME_FAIL"
}

# PKG and TEST select which package the following mutations apply to. They
# default to the control package, so the existing calls read unchanged.
PKG=dsim_control
TEST=test/test_control.cpp

mutate() {          # $1 = name, $2 = file, $3 = from, $4 = to
  local name="$1" file="$2" from="$3" to="$4"
  rm -rf "$WORK/src"; cp -r "src/$PKG" "$WORK/src"
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
  local result; result="$(build_and_run "$WORK/src" "$TEST" "$PKG")"
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

check_baseline() {  # $1 = package, $2 = test file
  local b ok=0
  audit_sources "$1" || ok=1
  b="$(build_and_run "src/$1" "$2" "$1")"
  echo "  $1: $b  (expected: ALL_PASS)"
  [ "$b" = "ALL_PASS" ] && [ "$ok" -eq 0 ]
}

echo "baseline (unmutated):"
baseline=ALL_PASS
check_baseline dsim_control test/test_control.cpp || baseline=BAD
check_baseline dsim_simctl test/test_pacer.cpp    || baseline=BAD
check_baseline dsim_time   test/test_sim_epoch.cpp || baseline=BAD
check_baseline dsim_estimation test/test_estimation.cpp || baseline=BAD
check_baseline dsim_eval   test/test_obstacle.cpp || baseline=BAD
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
# The pacer decides how fast the world runs, and its failures are quiet: a
# speed that is wrong by a factor of two looks like a calibration quirk, not
# like a bug. Every mutation here is one that actually happened during its
# development.
# ---------------------------------------------------------------------------
# The integral term. Three ways an integrator goes wrong, and the term is new
# enough that none of them would be caught by anything above.
run_mutation "integral term added instead of subtracted" \
  "src/se3_controller.cpp" \
  "    - integral_" \
  "    + integral_"

run_mutation "integral accumulates while the tilt is clamped (windup)" \
  "src/se3_controller.cpp" \
  "  if (dt > 0.0 && !hold) {" \
  "  if (dt > 0.0) {"

run_mutation "anti-windup clamp removed" \
  "src/se3_controller.cpp" \
  "    integral_ = integral_.cwiseMax(-gains_.max_integral_n)
      .cwiseMin(gains_.max_integral_n);" \
  "    integral_ = integral_;"

run_mutation "integral accumulates without regard to the step length" \
  "src/se3_controller.cpp" \
  "    integral_ += gains_.ki.cwiseProduct(e_p) * dt;" \
  "    integral_ += gains_.ki.cwiseProduct(e_p) * 0.004;"

run_mutation "integral gain raised past the Routh-Hurwitz bound" \
  "include/dsim_control/se3_controller.hpp" \
  "  Eigen::Vector3d ki     {1.5, 1.5, 2.0};" \
  "  Eigen::Vector3d ki     {20.0, 20.0, 20.0};"

PKG=dsim_simctl
TEST=test/test_pacer.cpp

run_mutation "playback speed ignored" \
  "include/dsim_simctl/pacer.hpp" \
  "target_ += speed_ * wall_dt;" \
  "target_ += wall_dt;"

run_mutation "a world that overran is forgiven (the real 0.63x bug)" \
  "include/dsim_simctl/pacer.hpp" \
  "if (sim_now - target_ > cfg_.max_slip_s) {target_ = sim_now - cfg_.max_slip_s;}" \
  "if (sim_now > target_) {target_ = sim_now;}"

run_mutation "backlog allowed to grow without bound" \
  "include/dsim_simctl/pacer.hpp" \
  "if (target_ - sim_now > cfg_.max_slip_s) {target_ = sim_now + cfg_.max_slip_s;}" \
  "" 

run_mutation "target not quantised to the physics step" \
  "include/dsim_simctl/pacer.hpp" \
  "return std::round(target_ / cfg_.step_s) * cfg_.step_s;" \
  "return target_;"

run_mutation "resync keeps the old target" \
  "include/dsim_simctl/pacer.hpp" \
  "    target_ = sim_now;
    have_target_ = true;" \
  "    have_target_ = true;"

run_mutation "slip tolerance tighter than the statistics interval" \
  "include/dsim_simctl/pacer.hpp" \
  "double max_slip_s {0.5};" \
  "double max_slip_s {0.05};"

# ---------------------------------------------------------------------------
# "Simulated time went backwards" is how every accumulating node learns that
# the run restarted. A detector that misses it leaves the referee reporting a
# previous run's numbers; one that fires spuriously throws away a good run.
# ---------------------------------------------------------------------------
PKG=dsim_time
TEST=test/test_sim_epoch.cpp

run_mutation "restart detected in the wrong direction" \
  "include/dsim_time/sim_epoch.hpp" \
  "const bool jumped_back = (last_s_ - now_s) > threshold_s_;" \
  "const bool jumped_back = (now_s - last_s_) > threshold_s_;"

run_mutation "first sample reported as a restart" \
  "include/dsim_time/sim_epoch.hpp" \
  "      last_s_ = now_s;
      return false;" \
  "      last_s_ = now_s;
      return true;"

run_mutation "restart reported on every sample after one" \
  "include/dsim_time/sim_epoch.hpp" \
  "    last_s_ = now_s;
    if (jumped_back) {" \
  "    if (jumped_back || epochs_ > 0) {"

run_mutation "restarts not counted" \
  "include/dsim_time/sim_epoch.hpp" \
  "      ++epochs_;" \
  "      epochs_ += 0;"

run_mutation "forget() does not forget" \
  "include/dsim_time/sim_epoch.hpp" \
  "    have_ = false;
  }" \
  "  }"

PKG=dsim_control
TEST=test/test_control.cpp

# ---------------------------------------------------------------------------
# The referee's clearance. This is the number that says whether a plan hit
# something, so a bug here is a plan graded wrong in whichever direction the
# bug points: a clamp that hides penetration, a velocity that is ignored so
# the fence is scored where it was ten seconds ago, a hit counted five hundred
# times or never cleared.
# ---------------------------------------------------------------------------
PKG=dsim_eval
TEST=test/test_obstacle.cpp

run_mutation "obstacle motion ignored (scored where it started)" \
  "include/dsim_eval/obstacle.hpp" \
  "      ? std::max(0.0, std::min(1.0, (t - tStart()) / span))" \
  "      ? 0.0"

run_mutation "de Casteljau stops one step early (a cubic read as its hull)" \
  "include/dsim_eval/obstacle.hpp" \
  "    while (pts.size() > 1) {" \
  "    while (pts.size() > 2) {"

run_mutation "the active window ignored (an absent obstacle still scored)" \
  "include/dsim_eval/obstacle.hpp" \
  "    if (!activeAt(t)) {return std::numeric_limits<double>::infinity();}" \
  "    if (false) {return std::numeric_limits<double>::infinity();}"

run_mutation "a column treated as a ball (an over-the-top route the 2D plan never had)" \
  "include/dsim_eval/obstacle.hpp" \
  "    if (type == \"column\") {
      return std::sqrt(dx * dx + dy * dy) - radius;
    }" \
  "    if (false) {
      return std::sqrt(dx * dx + dy * dy) - radius;
    }"

run_mutation "sphere distance clamped at zero (penetration reads as a graze)" \
  "include/dsim_eval/obstacle.hpp" \
  "    return std::sqrt(dx * dx + dy * dy + dz * dz) - radius;" \
  "    return std::max(0.0, std::sqrt(dx * dx + dy * dy + dz * dz) - radius);"

run_mutation "vehicle radius not subtracted from the clearance" \
  "include/dsim_eval/obstacle.hpp" \
  "      const double c = obstacles_[i].signedDistance(px, py, pz, t) - radius_;" \
  "      const double c = obstacles_[i].signedDistance(px, py, pz, t);"

run_mutation "a hit recorded on every sample inside, not once per entry" \
  "include/dsim_eval/obstacle.hpp" \
  "        if (!inside_[i]) {" \
  "        if (true) {"

run_mutation "leaving an obstacle never clears the inside flag (no re-entry)" \
  "include/dsim_eval/obstacle.hpp" \
  "      } else {
        inside_[i] = false;
      }" \
  "      } else {
      }"

run_mutation "reset keeps the old hits" \
  "include/dsim_eval/obstacle.hpp" \
  "    hits_.clear();
    std::fill(inside_.begin(), inside_.end(), false);" \
  "    std::fill(inside_.begin(), inside_.end(), false);"

PKG=dsim_control
TEST=test/test_control.cpp

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

# Same treatment for the planner bridge's conversion (src/dsim_planner). The
# tests there compare analytic derivatives against finite differences in
# time, which is the one comparison a chain-rule slip cannot pass.
mutate_planner() {  # $1 = name, $2 = from, $3 = to  (spacetime.py)
  local name="$1" from="$2" to="$3"
  rm -rf "$WORK/planner"; cp -r src/dsim_planner "$WORK/planner"
  find "$WORK/planner" -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null
  local target="$WORK/planner/dsim_planner/spacetime.py"
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
  if PYTHONPATH="$WORK/planner" python3 -m pytest -x -q "$WORK/planner/test/test_spacetime.py" \
       >"$WORK/py.log" 2>&1; then
    echo "  [SURVIVED] $name  <-- BUG: this error would pass the test suite"
    return 2
  fi
  echo "  [CAUGHT]   $name  -> $(grep -oE '[0-9]+ failed' "$WORK/py.log" | head -1)"
  return 0
}

run_planner_mutation() {
  mutate_planner "$@"
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

# The cascade's command arrows. Each of these is a plausible slip that draws a
# command which looks reasonable and is wrong about the one thing it shows.
run_py_mutation "position command points along the error, not at the reference" \
  "            _mul(_sub(ref_p, p), TRACK_MAGNIFY)," \
  "            _mul(_sub(p, ref_p), TRACK_MAGNIFY),"

run_py_mutation "reference position recovered by adding the error" \
  "    ref_p = _sub(p, control['position_error'])" \
  "    ref_p = _add(p, control['position_error'])"

run_py_mutation "position command drawn true to scale (below the draw floor)" \
  "TRACK_MAGNIFY = 10.0" \
  "TRACK_MAGNIFY = 1.0"

run_py_mutation "velocity command drawn as the error instead of the reference" \
  "    ref_v = _sub(v, control['velocity_error'])" \
  "    ref_v = list(control['velocity_error'])"

run_py_mutation "velocity command on its own scale" \
  "            _mul(ref_v, SCALE['vel_m_per_mps'])," \
  "            _mul(ref_v, SCALE['vel_m_per_mps'] * 2.0),"

run_py_mutation "attitude command left out of the command group" \
  "            'command', 'cmd_axis', p, _mul(cmd, axis * 1.2)," \
  "            'attitude', 'cmd_axis', p, _mul(cmd, axis * 1.2),"

run_py_mutation "commands survive disarm" \
  "    if not control['armed']:
        return {'arrows': [], 'ticks': [], 'readout': readout, 'scale': SCALE}" \
  "    if not control['armed'] and False:
        return {'arrows': [], 'ticks': [], 'readout': readout, 'scale': SCALE}"

run_py_mutation "arrow labels drop back to one decimal" \
  "LABEL_DECIMALS = 2" \
  "LABEL_DECIMALS = 1"

run_py_mutation "torque labelled in newton-metres at two decimals" \
  "            f'{1e3 * _norm(tau):.{LABEL_DECIMALS}f} mN.m'))" \
  "            f'{_norm(tau):.{LABEL_DECIMALS}f} N.m'))"

echo

# ---------------------------------------------------------------------------
# The space-time Bezier conversion. tau is not time; every one of these is
# the mistake of forgetting that in one place.
# ---------------------------------------------------------------------------
echo
echo "baseline (unmutated planner conversion):"
rm -rf "$WORK/planner"; cp -r src/dsim_planner "$WORK/planner"
find "$WORK/planner" -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null
if PYTHONPATH="$WORK/planner" python3 -m pytest -q "$WORK/planner/test/test_spacetime.py" \
     >"$WORK/py.log" 2>&1; then
  echo "  ALL_PASS  (expected)"
else
  echo "  BASELINE FAILS -- fix that before trusting anything below"
  tail -5 "$WORK/py.log"
  exit 1
fi

echo
echo "injected planner-conversion bugs:"

run_planner_mutation "dp/dtau reported as the velocity" \
  "    vel = [dp[i] / tdot for i in range(3)]" \
  "    vel = [dp[i] for i in range(3)]"

run_planner_mutation "acceleration drops the d2t/dtau2 term" \
  "    acc = [(ddp[i] * tdot - dp[i] * tddot) / tdot ** 3 for i in range(3)]" \
  "    acc = [ddp[i] / tdot ** 2 for i in range(3)]"

run_planner_mutation "time inverted as if it were linear in tau" \
  "    lo, hi = 0.0, 1.0
    for _ in range(iters):" \
  "    return (t - t0) / (t1 - t0)
    for _ in range(iters):"

run_planner_mutation "derivative control points without the degree factor" \
  "    return [[n * (b - a) for a, b in zip(P[i], P[i + 1])] for i in range(n)]" \
  "    return [[(b - a) for a, b in zip(P[i], P[i + 1])] for i in range(n)]"

run_planner_mutation "monotonic time no longer required" \
  "        if not P[i + 1][3] > P[i][3]:" \
  "        if False:"

run_planner_mutation "the arrival sample dropped" \
  "    out.append((t1, *sample_at_time(P, t1)))
    return out" \
  "    return out"

run_planner_mutation "samples spaced in tau instead of time" \
  "        out.append((t, *sample_at_time(P, t)))" \
  "        out.append((de_casteljau(P, k * dt / (t1 - t0))[3], *sample_at_time(P, de_casteljau(P, k * dt / (t1 - t0))[3])))"

# ---------------------------------------------------------------------------
# The solve endpoint. It is the second write path this project exposes on a
# port reachable across the whole tailnet, so every bound on what a request may
# say gets broken on purpose. (It is no longer the only one that starts a
# process -- /launch starts a whole simulator, and gets the same treatment
# further down.)
# ---------------------------------------------------------------------------
mutate_solver() {   # $1 = name, $2 = from, $3 = to
  local name="$1" from="$2" to="$3"
  rm -rf "$WORK/viz"; cp -r src/dsim_viz "$WORK/viz"
  find "$WORK/viz" -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null
  local target="$WORK/viz/dsim_viz/solver.py"
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
  if PYTHONPATH="$WORK/viz" python3 -m pytest -x -q "$WORK/viz/test/test_solver.py" \
       >"$WORK/py.log" 2>&1; then
    echo "  [SURVIVED] $name  <-- BUG: this error would pass the test suite"
    return 2
  fi
  echo "  [CAUGHT]   $name  -> $(grep -oE '[0-9]+ failed' "$WORK/py.log" | head -1)"
  return 0
}

run_solver_mutation() {
  mutate_solver "$@"
  local rc=$?
  [ $rc -eq 2 ] && survivors=$((survivors+1))
  [ $rc -eq 1 ] && skipped=$((skipped+1))
  return 0
}

echo
echo "baseline (unmutated solve endpoint):"
rm -rf "$WORK/viz"; cp -r src/dsim_viz "$WORK/viz"
find "$WORK/viz" -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null
if PYTHONPATH="$WORK/viz" python3 -m pytest -q "$WORK/viz/test/test_solver.py" \
     >"$WORK/py.log" 2>&1; then
  echo "  ALL_PASS  (expected)"
else
  echo "  BASELINE FAILS -- fix that before trusting anything below"
  tail -5 "$WORK/py.log"
  exit 1
fi

echo
echo "injected solve-endpoint bugs:"

run_solver_mutation "range check dropped" \
  "    if not lo <= v <= hi:" \
  "    if False:"

run_solver_mutation "the scenario taken from the request" \
  "                \"--scenario\", self.scenario," \
  "                \"--scenario\", body.get(\"scenario\", self.scenario),"

run_solver_mutation "the output path taken from the request" \
  "                \"--out\", self.plan_file]" \
  "                \"--out\", body.get(\"out\", self.plan_file)]"

run_solver_mutation "a missing field defaults instead of refusing" \
  "    if key not in body:
        raise SolveError(f\"missing {key}\")" \
  "    if key not in body:
        return lo"

run_solver_mutation "run through a shell" \
  "            proc = subprocess.run(argv, cwd=self.ws, capture_output=True," \
  "            proc = subprocess.run(\" \".join(argv), shell=True, cwd=self.ws, capture_output=True,"

run_solver_mutation "a zero speed cap passed through as a cap of zero" \
  "        if vmax > 0.0:" \
  "        if vmax >= 0.0:"

run_solver_mutation "enabled without a plan file to write" \
  "        self.enabled = bool(self.allowed and scenario and plan_file)" \
  "        self.enabled = bool(self.allowed)"

run_solver_mutation "the plan summary cached on mtime alone" \
  "        key = (self.plan_file, st.st_mtime_ns, st.st_size, st.st_ino)" \
  "        key = (self.plan_file, st.st_mtime_ns)"

run_solver_mutation "retarget keeps the previous run's solve log" \
  "            self.last = None
        self.scenario = scenario" \
  "            pass
        self.scenario = scenario"

run_solver_mutation "an unreadable plan crashes the snapshot" \
  "        except (OSError, ValueError):
            return None" \
  "        except OSError:
            return None"

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
    local caught; caught="$(grep '^\[  FAILED  \] [A-Za-z]' "$WORK/run.log" | sed 's/ ([0-9]* ms)$//' | sort -u | wc -l)"
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
sensors_baseline="$(build_and_run_sensors src/dsim_sensors)"
echo "  $sensors_baseline  (expected: ALL_PASS)"
if [ "$sensors_baseline" != "ALL_PASS" ]; then
  # Same guard as the C++ and overlay blocks, added after it was seen to
  # matter: with magnetometer.hpp deliberately broken, every one of the 17
  # sensor mutations below reported CAUGHT against the same pre-existing
  # failure. This block printed its baseline and carried on regardless.
  echo "  BASELINE IS NOT GREEN -- every sensor result below would be meaningless."
  tail -20 "$WORK/build.log" "$WORK/run.log" 2>/dev/null
  exit 1
fi
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

# The magnetometer is one transpose, and every wrong version of it -- the
# world field reported unrotated, R for R^T, swapped or dropped components --
# still yields a plausible field on every sample. A compass that turns the
# wrong way is invisible to everything but a test that knows the true yaw.
run_sensor_mutation "magnetometer reports the world field unrotated" \
  "include/dsim_sensors/magnetometer.hpp" \
  "  return R_wb.transpose() * field_world;" \
  "  return field_world;"

run_sensor_mutation "magnetometer rotated the wrong way (R for R^T)" \
  "include/dsim_sensors/magnetometer.hpp" \
  "  return R_wb.transpose() * field_world;" \
  "  return R_wb * field_world;"

run_sensor_mutation "magnetometer x and y components swapped" \
  "include/dsim_sensors/magnetometer.hpp" \
  "  return R_wb.transpose() * field_world;" \
  "  const Eigen::Vector3d b = R_wb.transpose() * field_world;
  return Eigen::Vector3d(b.y(), b.x(), b.z());"

run_sensor_mutation "magnetometer drops the vertical component" \
  "include/dsim_sensors/magnetometer.hpp" \
  "  return R_wb.transpose() * field_world;" \
  "  return R_wb.transpose() * Eigen::Vector3d(field_world.x(), field_world.y(), 0.0);"

run_sensor_mutation "magnetometer noise not applied" \
  "include/dsim_sensors/magnetometer.hpp" \
  "  return true_body + hard_iron_body + c.noise_t * gauss_unit;" \
  "  return true_body + hard_iron_body + 0.0 * c.noise_t * gauss_unit;"

run_sensor_mutation "hard iron dropped from the reading" \
  "include/dsim_sensors/magnetometer.hpp" \
  "  return true_body + hard_iron_body + c.noise_t * gauss_unit;" \
  "  return true_body + c.noise_t * gauss_unit;"

run_sensor_mutation "hard iron not scaled to the configured magnitude" \
  "include/dsim_sensors/magnetometer.hpp" \
  "  return gauss_unit * (hard_iron_t / n);" \
  "  return gauss_unit;"

# ---------------------------------------------------------------------------
# The state estimator. Every mutation here is a bug that flies: the vehicle
# takes off on the estimate, and each of these produces a plausible-looking
# state that is wrong in one way -- a phantom climb in every turn, a velocity
# on the wrong axes once yawed, a compass that reads 30 deg off when banked.
# None of them is visible at a level hover, which is why the tests fly a
# tilted, yawed, rotating vehicle.
# ---------------------------------------------------------------------------
PKG=dsim_estimation
TEST=test/test_estimation.cpp

run_mutation "gravity added with the wrong sign (hover reads as a 2 g climb)" \
  "src/velocity_filter.cpp" \
  "return R_est * accel_body + Eigen::Vector3d(0.0, 0.0, -cfg_.gravity);" \
  "return R_est * accel_body + Eigen::Vector3d(0.0, 0.0, cfg_.gravity);"

run_mutation "accelerometer rotated with R^T instead of R" \
  "src/velocity_filter.cpp" \
  "return R_est * accel_body + Eigen::Vector3d(0.0, 0.0, -cfg_.gravity);" \
  "return R_est.transpose() * accel_body + Eigen::Vector3d(0.0, 0.0, -cfg_.gravity);"

run_mutation "flow y sign flipped against OpticalFlow.msg" \
  "src/velocity_filter.cpp" \
  "const double vy = -(f.integrated_x - f.integrated_xgyro) / f.integration_time_s * h;" \
  "const double vy = (f.integrated_x - f.integrated_xgyro) / f.integration_time_s * h;"

run_mutation "gyro term dropped from the flow (rotation becomes velocity)" \
  "src/velocity_filter.cpp" \
  "const double vx = (f.integrated_y - f.integrated_ygyro) / f.integration_time_s * h;" \
  "const double vx = f.integrated_y / f.integration_time_s * h;"

run_mutation "slant range used as altitude" \
  "src/velocity_filter.cpp" \
  "  return range_m * R_est(2, 2);" \
  "  return range_m;"

run_mutation "mag heading read without tilt compensation" \
  "src/attitude_filter.cpp" \
  "  const Eigen::Vector3d m_world = q_ * mag_body;" \
  "  const Eigen::Vector3d m_world = mag_body;"

run_mutation "flow innovation not rotated into the world" \
  "src/velocity_filter.cpp" \
  "  v_ += alpha * (R_est * innovation_body);" \
  "  v_ += alpha * innovation_body;"

run_mutation "flow innovation compares against R v instead of R^T v" \
  "src/velocity_filter.cpp" \
  "  const Eigen::Vector3d v_body_est = R_est.transpose() * v_;" \
  "  const Eigen::Vector3d v_body_est = R_est * v_;"

run_mutation "gyro bias feedback sign flipped" \
  "src/attitude_filter.cpp" \
  "  bias_ -= (cfg_.ki_accel * e_acc + cfg_.ki_mag * e_mag) * dt;" \
  "  bias_ += (cfg_.ki_accel * e_acc + cfg_.ki_mag * e_mag) * dt;"

run_mutation "accelerometer correction sign flipped" \
  "src/attitude_filter.cpp" \
  "  accel_error_ = v_meas.cross(v_pred);" \
  "  accel_error_ = v_pred.cross(v_meas);"

run_mutation "gyro integrated in the world frame" \
  "src/attitude_filter.cpp" \
  "    q_ = (q_ * dq).normalized();" \
  "    q_ = (dq * q_).normalized();"

run_mutation "mag correction computed but never applied" \
  "src/attitude_filter.cpp" \
  "  mag_error_z_ = std::sin(err);" \
  "  mag_error_z_ = 0.0;"

run_mutation "quality-0 flow readings fused anyway" \
  "src/velocity_filter.cpp" \
  "  if (f.quality == 0 || !std::isfinite(f.ground_distance_m)" \
  "  if (!std::isfinite(f.ground_distance_m)"

run_mutation "out-of-range (infinite) ranges accepted" \
  "src/velocity_filter.cpp" \
  "  if (!std::isfinite(range_m) || range_m <= 0.0 || cos_tilt <= 0.0) {" \
  "  if (range_m <= 0.0 || cos_tilt <= 0.0) {"

run_mutation "range residual not fed into the climb rate" \
  "src/velocity_filter.cpp" \
  "  v_.z() += k_vz * residual;" \
  "  v_.z() += 0.0 * residual;"

run_mutation "accel integral gain re-enabled (winds up in every turn)" \
  "include/dsim_estimation/attitude_filter.hpp" \
  "  double ki_accel {0.0};" \
  "  double ki_accel {0.01};"

PKG=dsim_control
TEST=test/test_control.cpp

# ---------------------------------------------------------------------------
# The launch endpoint. The biggest hammer on the port: it tears the simulator
# down and starts a different one. Two classes of bug are broken on purpose
# here -- what a request is allowed to reach, and whether the endpoint can
# still say a launch FAILED. The second matters as much as the first: a
# launcher that reports success unconditionally turns a dead simulator into a
# blank page with a green label on it.
# ---------------------------------------------------------------------------
mutate_launcher() { # $1 = name, $2 = from, $3 = to
  local name="$1" from="$2" to="$3"
  rm -rf "$WORK/viz"; cp -r src/dsim_viz "$WORK/viz"
  find "$WORK/viz" -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null
  local target="$WORK/viz/dsim_viz/launcher.py"
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
  if PYTHONPATH="$WORK/viz" python3 -m pytest -x -q "$WORK/viz/test/test_launcher.py" \
       >"$WORK/py.log" 2>&1; then
    echo "  [SURVIVED] $name  <-- BUG: this error would pass the test suite"
    return 2
  fi
  echo "  [CAUGHT]   $name  -> $(grep -oE '[0-9]+ failed' "$WORK/py.log" | head -1)"
  return 0
}

run_launcher_mutation() {
  mutate_launcher "$@"
  local rc=$?
  [ $rc -eq 2 ] && survivors=$((survivors+1))
  [ $rc -eq 1 ] && skipped=$((skipped+1))
  return 0
}

echo
echo "baseline (unmutated launch endpoint):"
rm -rf "$WORK/viz"; cp -r src/dsim_viz "$WORK/viz"
find "$WORK/viz" -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null
if PYTHONPATH="$WORK/viz" python3 -m pytest -q "$WORK/viz/test/test_launcher.py" \
     >"$WORK/py.log" 2>&1; then
  echo "  ALL_PASS  (expected)"
else
  echo "  BASELINE FAILS -- fix that before trusting anything below"
  tail -5 "$WORK/py.log"
  exit 1
fi

echo
echo "injected launch-endpoint bugs:"

run_launcher_mutation "an unknown name fabricates a run instead of being refused" \
  "        run = self.catalogue.by_name(name)
        if run is None:" \
  "        run = self.catalogue.by_name(name) or {\"name\": name, \"world\": name}
        if run is None:"

run_launcher_mutation "the disabled switch ignored" \
  "        if not self.enabled:
            raise LaunchError(\"switching runs is disabled on this server\")" \
  "        if False:
            raise LaunchError(\"switching runs is disabled on this server\")"

run_launcher_mutation "teardown points at something harmless, so the old sim lives" \
  "                [\"bash\", os.path.join(self.ws, \"scripts\", \"kill_sim.sh\")]," \
  "                [\"true\"],"

run_launcher_mutation "a failed teardown ignored" \
  "        if killed.returncode != 0:" \
  "        if False:"

run_launcher_mutation "run through a shell" \
  "                proc = subprocess.Popen(argv, cwd=self.ws, stdout=log," \
  "                proc = subprocess.Popen(\" \".join(argv), shell=True, cwd=self.ws, stdout=log,"

run_launcher_mutation "a scenario offered that has no scenario file" \
  "            if not os.path.exists(os.path.join(self.scenario_dir, f\"{scenario}.json\")):" \
  "            if False:"

run_launcher_mutation "the plan's filename trusted over its declared scenario" \
  "            scenario = doc.get(\"scenario\")" \
  "            scenario = fname.split(\"_\")[0]"

run_launcher_mutation "the seed plan preferred over the solved one" \
  "        solved = sorted(f for f, _ in plans if not f.endswith(\"_seed.json\"))" \
  "        solved = sorted(f for f, _ in plans if f.endswith(\"_seed.json\"))"

run_launcher_mutation "the sensing envelope ignored, so loiter flies on the estimate" \
  "            if top is not None and top > ceiling:" \
  "            if False:"

run_launcher_mutation "stale telemetry counts as the new run coming up" \
  "            if started is not None and started > self._at:" \
  "            if started is not None:"

run_launcher_mutation "a launch that exits is never reported as failed" \
  "                if code is not None:" \
  "                if False:"

run_launcher_mutation "a launch that never publishes starts for ever" \
  "                elif (self._clock() - self._at) > START_TIMEOUT_S:" \
  "                elif False:"

run_launcher_mutation "a second switch allowed while one is still starting" \
  "        if self.status()[0] == \"starting\":" \
  "        if False:"

run_launcher_mutation "a reference run named after its world, so hover and circle collide" \
  "        name = doc.get(\"world\") if doc.get(\"plan\") else doc.get(\"reference\")" \
  "        name = doc.get(\"world\")"

run_launcher_mutation "a run outside the catalogue claims a catalogue entry" \
  "        return name if name and self.catalogue.by_name(name) else None" \
  "        return name or None"

run_launcher_mutation "the catalogue keyed on modification time alone" \
  "            out.append((e.name, st.st_mtime_ns, st.st_size))" \
  "            out.append((e.name, st.st_mtime_ns))"

run_launcher_mutation "the catalogue keyed on the directory, which misses a rewrite" \
  "    try:
        entries = os.scandir(path)" \
  "    try:
        return os.stat(path).st_mtime_ns
        entries = os.scandir(path)"

run_launcher_mutation "the catalogue cached for the life of the process" \
  "        if key != self._key:
            self._runs = tuple(REFERENCE_RUNS) + tuple(self._scenario_runs())" \
  "        if self._runs == ():
            self._runs = tuple(REFERENCE_RUNS) + tuple(self._scenario_runs())"

run_launcher_mutation "current.json cached, so the page reports the previous run" \
  "        return doc if isinstance(doc, dict) else {}" \
  "        self._memo = getattr(self, \"_memo\", None) or (doc if isinstance(doc, dict) else {})
        return self._memo"

run_launcher_mutation "a half-written current.json taken as fact" \
  "        except (OSError, ValueError):
            # Missing, or caught mid-write. Either way there is no run to
            # report, and reporting none is better than reporting half of one.
            return {}" \
  "        except (OSError, ValueError):
            raise"

run_launcher_mutation "colour codes left in the log shown to the browser" \
  "        text = \"\\n\".join(_ANSI.sub(\"\", line).rstrip() for line in raw).strip()" \
  "        text = \"\\n\".join(line.rstrip() for line in raw).strip()"

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
