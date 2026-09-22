#!/usr/bin/env python3
"""Watch a RUNNING sim fly a space-time plan, and grade what happened.

    python3 scripts/check_planner.py plans/fence3d_seed.json [http://host:8080]

Polls /snapshot from before the plan starts until after it ends and checks,
in order, the things that have to hold for the run to mean anything:

  1. the world being flown is the world being scored -- Gazebo's own poses for
     the moving obstacles agree with the referee's pos0 + vel*t to within
     WORLD_MISMATCH_M, and at least one pose was actually heard (None fails);
  2. the vehicle reached the plan's start point before the scenario clock hit
     zero, so the plan was flown from where the planner assumed;
  3. the velocity feedforward reached the controller: during the cruise the
     reference speed the controller reports equals the plan's speed. With the
     feedforward zeroed the vehicle still tracks a gentle plan, late, and
     nothing else here would notice;
  4. tracking stayed inside TRACK_MAX_M for the whole window, and inside the
     tighter CRUISE_MAX_M once the start transient has died away;
  5. the hits are the hits the plan predicts. A plan file may declare
     `expect` -- the seed does, because it is a straight line through the
     fence and a referee that lets it through is broken; an optimised plan
     declares none and must produce none.

Exit code is the number of failures. Every check prints what would have
made it fail.
"""
import json
import math
import sys
import time
import urllib.request

WORLD_MISMATCH_M = 0.02    # 20 mm; gravity left on drifts a body 10 mm/s, caught in 2 s
START_TOL_M = 0.15         # must be at the start point, not merely near it
CRUISE_SPEED_TOL = 0.15    # fraction: |v_ref| against the plan's own speed
TRACK_MAX_M = 0.30         # the seed opens with a 0.9 m/s velocity step; measured
                           # 29 cm at t=0 and again at the stop, decaying over ~3 s
CRUISE_FROM_S = 3.0        # after the step transient has died away...
CRUISE_MAX_M = 0.10        # ...the loop holds the seed to 4 cm; 10 cm is a real loss
HIT_TIME_TOL_S = 0.30      # entry time vs the planned-trajectory prediction
HIT_DEPTH_TOL_M = 0.10


def fetch(url):
    with urllib.request.urlopen(url + "/snapshot", timeout=10) as r:
        return json.loads(r.read().decode())


class Checks:
    def __init__(self):
        self.failures = 0

    def ok(self, name, detail):
        print(f"  ok    {name}: {detail}")

    def bad(self, name, detail, would_catch):
        print(f"  FAIL  {name}: {detail}")
        print(f"        would catch: {would_catch}")
        self.failures += 1


def scenario_time(snap):
    c = snap.get("clearance")
    return None if not c else c["scenario_time_s"]


def wait_until(url, pred, timeout_s, what):
    """Poll until pred(snapshot) is true; return the last snapshot or None."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        snap = fetch(url)
        if pred(snap):
            return snap
        time.sleep(0.1)
    print(f"  timed out after {timeout_s:.0f} s waiting for {what}")
    return None


def main(plan_file, url):
    c = Checks()
    plan = json.load(open(plan_file))
    scenario = json.load(open(f"scenarios/{plan['scenario']}.json"))
    T = float(scenario["T"])
    start_p = scenario["start"][:3]
    expect = plan.get("expect", {})
    # The plan's own speed profile, for check 3, from the same sampler the
    # bridge uses -- an independent path from the controller's report.
    sys.path.insert(0, "src/dsim_planner")
    from dsim_planner import spacetime
    P = spacetime.validate(plan["control_points"])

    snap = fetch(url)
    if not snap.get("clearance"):
        c.bad("referee report present", "no /drone/eval/clearance in /snapshot",
              "a referee that never started, or a viewer not subscribed to it")
        return c.failures
    if not snap.get("plan_path"):
        c.bad("plan path present", "no /drone/plan_path in /snapshot",
              "a bridge that never started, or a latched topic the viewer missed")
        return c.failures

    t_now = scenario_time(snap)
    if t_now > 0.0:
        c.bad("caught the run before it started", f"scenario t is already {t_now:.1f} s",
              "nothing; this script must be started before start_s -- see plan_check.sh")
        return c.failures
    print(f"  scenario t = {t_now:.1f} s; plan runs 0 .. {T:.0f} s")

    # ---- 2. at the start point before the clock starts ---------------------
    snap = wait_until(url, lambda s: scenario_time(s) >= -0.3, abs(t_now) + 5, "t = -0.3 s")
    if snap is None:
        c.bad("reached t=-0.3 s", "no progress", "a paused or dead sim")
        return c.failures
    d_start = math.dist(snap["pose"]["p"], start_p)
    if d_start <= START_TOL_M:
        c.ok("at the start point before t=0", f"{d_start * 100:.1f} cm from {start_p}")
    else:
        c.bad("at the start point before t=0", f"{d_start:.2f} m from {start_p}",
              "a start_s too short for the transit from the pad, or a plan whose first "
              "point is not where the scenario says")

    # ---- run the window, sampling -----------------------------------------
    max_err = 0.0
    max_err_cruise = 0.0
    speed_samples = []          # (t, |v_ref| reported, |v| planned)
    last_t = -1.0
    deadline = time.time() + T + 15
    while time.time() < deadline:
        snap = fetch(url)
        t = scenario_time(snap)
        if t > T + 1.0:
            break
        if t != last_t and 0.0 <= t <= T:
            last_t = t
            st = snap.get("status") or {}
            max_err = max(max_err, st.get("tracking_error_m", 0.0))
            if t >= CRUISE_FROM_S:
                max_err_cruise = max(max_err_cruise, st.get("tracking_error_m", 0.0))
            ctl = snap.get("control")
            if ctl and 1.0 <= t <= T - 1.0:
                v = ctl["velocity_world"]
                e = ctl["velocity_error"]
                v_ref = math.dist([v[i] - e[i] for i in range(3)], [0, 0, 0])
                _, v_plan, _ = spacetime.sample_at_time(P, t)
                speed_samples.append((t, v_ref, math.dist(v_plan, [0, 0, 0])))
        time.sleep(0.1)
    cl = snap["clearance"]
    print(f"  window done at scenario t = {cl['scenario_time_s']:.1f} s, "
          f"{len(speed_samples)} cruise samples")

    # ---- 1. world vs referee ----------------------------------------------
    mm = cl["world_mismatch_m"]
    moving = any(any(v != 0 for v in o.get("vel", [0, 0, 0]))
                 for o in scenario["obstacles"])
    if not moving:
        c.ok("world vs referee", "no moving obstacles; nothing to disagree about")
    elif mm is None or cl["world_poses_seen"] == 0:
        c.bad("world vs referee", "no obstacle pose was ever heard from Gazebo",
              "a missing PosePublisher or bridge entry -- the check is off, not passing")
    elif mm <= WORLD_MISMATCH_M:
        c.ok("world vs referee", f"max |gazebo - analytic| {mm * 1000:.2f} mm over "
             f"{cl['world_poses_seen']} poses (limit {WORLD_MISMATCH_M * 1000:.0f} mm)")
    else:
        c.bad("world vs referee", f"max |gazebo - analytic| {mm * 1000:.1f} mm",
              "a world generated with a different start_s than the referee uses, "
              "or gravity acting on a body that is supposed to move at constant velocity")

    # ---- 3. feedforward reached the controller ----------------------------
    if not speed_samples:
        c.bad("velocity feedforward reached the controller", "no cruise samples",
              "a run too short to sample, or a viewer without control telemetry")
    else:
        worst = max(abs(a - b) / max(b, 1e-6) for _, a, b in speed_samples)
        t_w, a_w, b_w = max(speed_samples, key=lambda s: abs(s[1] - s[2]) / max(s[2], 1e-6))
        if worst <= CRUISE_SPEED_TOL:
            c.ok("velocity feedforward reached the controller",
                 f"|v_ref| tracks the plan's speed within {worst * 100:.1f}% "
                 f"(worst at t={t_w:.1f} s: {a_w:.3f} vs {b_w:.3f} m/s)")
        else:
            c.bad("velocity feedforward reached the controller",
                  f"|v_ref| {a_w:.3f} vs planned {b_w:.3f} m/s at t={t_w:.1f} s "
                  f"({worst * 100:.0f}% off)",
                  "a bridge publishing positions with zero velocity, or a sampler that "
                  "reports dp/dtau as a velocity")

    # ---- 4. tracking --------------------------------------------------------
    if max_err <= TRACK_MAX_M:
        c.ok("tracking through the window", f"max error {max_err * 100:.1f} cm "
             f"(limit {TRACK_MAX_M * 100:.0f} cm)")
    else:
        c.bad("tracking through the window", f"max error {max_err * 100:.1f} cm",
              "a plan the vehicle cannot follow, or feedforward with the wrong sign")
    if max_err_cruise <= CRUISE_MAX_M:
        c.ok("tracking in the cruise", f"max error {max_err_cruise * 100:.1f} cm after "
             f"t={CRUISE_FROM_S:.0f} s (limit {CRUISE_MAX_M * 100:.0f} cm)")
    else:
        c.bad("tracking in the cruise", f"max error {max_err_cruise * 100:.1f} cm after "
              f"t={CRUISE_FROM_S:.0f} s",
              "acceleration feedforward missing or wrong on a plan that turns, or a "
              "loop that never settles after the start")

    # ---- 5. the hits ----------------------------------------------------------
    hits = cl["hits"]
    summary = ", ".join(f"{h['with']}@t={h['t']:.2f}s depth {h['depth_m'] * 100:.0f}cm"
                        for h in hits) or "none"
    if "first_hit" in expect:
        # The plan says it collides, and says with what and when. Predicted on
        # the PLANNED trajectory; the flown one is late or early by the
        # tracking error, hence the tolerance.
        e = expect["first_hit"]
        if not hits:
            c.bad("expected collision", "no hit recorded",
                  "a referee that cannot see the obstacles it was given, or a world "
                  "with the obstacles somewhere else -- the seed goes THROUGH the fence")
        else:
            h = hits[0]
            good = (h["with"] == e["with"] and abs(h["t"] - e["t"]) <= HIT_TIME_TOL_S
                    and abs(h["depth_m"] - e["depth_m"]) <= HIT_DEPTH_TOL_M)
            if good:
                c.ok("expected collision", f"first hit {h['with']} at t={h['t']:.2f} s, "
                     f"{h['depth_m'] * 100:.0f} cm deep (predicted {e['with']} at "
                     f"t={e['t']:.2f} s, {e['depth_m'] * 100:.0f} cm); all: {summary}")
            else:
                c.bad("expected collision", f"first hit {h['with']} at t={h['t']:.2f} s, "
                      f"{h['depth_m'] * 100:.0f} cm deep; predicted {e['with']} at "
                      f"t={e['t']:.2f} s, {e['depth_m'] * 100:.0f} cm",
                      "obstacles moving at the wrong speed or from the wrong place, "
                      "or a scenario clock offset between world, referee and bridge")
    else:
        if not hits:
            c.ok("no collision", f"min clearance {cl['min_clearance_m'] * 100:.1f} cm")
        else:
            c.bad("no collision", f"hits: {summary}",
                  "a plan that does not clear the obstacles once the vehicle's own "
                  "radius is counted, or one the vehicle could not follow closely enough")
        cert = plan.get("certified_clearance_m")
        if cert is not None:
            # The planner's own certificate is for a point; the referee
            # subtracts the vehicle radius. Report the comparison honestly.
            print(f"  note  planner certified {cert:+.3f} m for a POINT; the referee's "
                  f"{cl['min_clearance_m']:+.3f} m counts the vehicle radius")

    return c.failures


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "http://127.0.0.1:8080"))
