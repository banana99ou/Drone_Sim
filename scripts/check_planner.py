#!/usr/bin/env python3
"""Watch a RUNNING sim fly a space-time plan, and grade what happened.

    python3 scripts/check_planner.py plans/fence3d_seed.json [http://host:8080]

Polls /snapshot from before the plan starts until after it ends and checks,
in order, the things that have to hold for the run to mean anything:

  1. the referee's obstacle field agrees with the PLANNER's own
     obstacle_positions_at() -- two independent implementations, in two
     languages, of the same Bezier motion, sampled across the whole window.
     That is what "the world being scored is the world that was planned"
     reduces to now that the obstacles are not Gazebo bodies at all;
  2. the vehicle reached the plan's start point before the scenario clock hit
     zero, so the plan was flown from where the planner assumed;
  3. the velocity feedforward reached the controller: during the cruise the
     reference speed the controller reports equals the plan's speed. With the
     feedforward zeroed the vehicle still tracks a gentle plan, late, and
     nothing else here would notice;
  4. tracking stayed inside TRACK_MAX_M for the whole window, and inside the
     tighter cruise allowance over the last 40% of the plan, which is after
     the opening transient and before the stop;
  5. the hits are the hits the plan predicts. A plan file may declare
     `expect` -- the seed does, because it is a straight line through the
     obstacles and a referee that lets it through is broken. An optimised plan
     declares none, and then whether hits are allowed is decided by its own
     `predicted_clearance_m`: a plan the optimiser could not certify says so,
     and a hit is agreement rather than a surprise;
  6. and if the plan carries `predicted_clearance_m` -- what the optimiser's
     certificate implies for THIS vehicle on the real obstacles -- the
     referee's measured minimum agrees with it. Those are two independent
     computations of the same number: a convex certificate over the whole
     curve on one side, and sampled ground truth on the other. They can
     differ by the tracking error, and by nothing else.

Exit code is the number of failures. Every check prints what would have
made it fail.
"""
import json
import math
import sys
import time
import urllib.request

#: The referee (C++ de Casteljau) against the planner (Python, via numpy).
#: Two implementations of the same curve: they should agree to floating point,
#: so this is a round-off bound and not a tolerance for being approximately
#: right. A frame offset, a clock offset or a dropped control point all land
#: orders of magnitude above it.
OBSTACLE_AGREEMENT_M = 1e-6
START_TOL_M = 0.15         # must be at the start point, not merely near it
CRUISE_SPEED_TOL = 0.15    # fraction: |v_ref| against the plan's own speed
#: The cruise window is the last CRUISE_FROM_FRAC of the plan, never starting
#: before CRUISE_MIN_S. Defined as a FRACTION rather than a fixed second
#: because the opening transient scales with the plan's start speed, and a
#: constant calibrated on the seed's 0.9 m/s start sat right on top of a 3 m/s
#: plan's decay: measured 92 cm at t=0.5, 23 cm at t=2, 8-10 cm at t=3, 7 cm
#: at t=4 -- so a 3 s boundary straddled the 10 cm limit run to run while
#: nothing about the tracking had changed.
#:
#: This is not a threshold that cannot fail. A plan whose transient still eats
#: 40% of its own window IS a plan the vehicle cannot fly, and a feedforward
#: error shows up across the whole window, not just at its start.
CRUISE_FROM_FRAC = 0.4
CRUISE_MIN_S = 3.0
#: Cruise tracking allowance, as a fraction of the PLAN's top speed times a
#: lag. The loop's steady error is velocity-proportional -- it flies on an
#: estimated velocity that lags the true one -- so a fixed number is calibrated
#: at one speed and wrong at every other: measured 2.8 cm on a 0.9 m/s plan,
#: 8.0 on a 3.0 m/s one and 10.3 on a 2.5 m/s plan that does its fastest work
#: at the end. 50 ms of lag covers all of those with room, and a feedforward
#: that stopped arriving would double or triple them.
CRUISE_LAG_S = 0.05
CRUISE_MIN_M = 0.05        # ...and a floor, so a slow plan is not held to millimetres
#: Peak tracking error allowed in the window for a plan that starts from rest.
TRACK_MAX_M = 0.30
#: For a plan that opens at speed, the peak scales with that speed: the
#: optimiser pins endpoint positions and not velocities, so the vehicle
#: hovering at the start point is handed a velocity STEP, and the loop is
#: bandwidth-limited rather than acceleration-limited in answering it.
#: Measured peak / v0 across six flown plans from 0.70 to 3.00 m/s: 0.322 s,
#: with residuals under 5 cm and no trend. This is that, with margin.
#:
#: The first version of this used the full-authority catch-up distance
#: v0^2/(2 a_max), which is quadratic and under-predicted by 3x at the low end
#: -- it passed fence3d and failed diverse and loiter for no reason to do with
#: either plan.
OPENING_LAG_S = 0.5
HIT_TIME_TOL_S = 0.30      # entry time vs the planned-trajectory prediction
HIT_DEPTH_TOL_M = 0.10
#: How far the measured minimum clearance may fall below the certificate. The
#: certificate is about the PLANNED curve; the vehicle flies the tracking error
#: away from it, so the measured margin is smaller by up to that error. Sized
#: against the tracking the loop actually achieves, not picked round.
CLEARANCE_SLACK_M = 0.10


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


def obstacle_agreement(url, scenario, samples=41):
    """Compare the referee's live obstacle field against the planner's own.

    The referee is polled at whatever scenario time it happens to be at, and
    the planner's obstacle_positions_at() -- an independent implementation of
    the same lifted-control-point motion -- is evaluated at that same instant.
    Repeated across the window so a disagreement that only appears mid-curve
    (a cubic read as a straight line, say) cannot hide.

    Returns (worst distance, which obstacle, at what time, window mismatches).
    """
    from spacetime_bezier.geometry import obstacle_positions_at
    import numpy as np

    by_name = {o["name"]: o for o in scenario["obstacles"]}
    worst, worst_name, worst_t, window = 0.0, "", 0.0, []
    seen = 0
    for _ in range(samples):
        cl = fetch(url).get("clearance")
        if not cl or not cl["obstacles"]:
            time.sleep(0.05)
            continue
        seen += 1
        t = cl["scenario_time_s"]
        for o in cl["obstacles"]:
            ref = by_name.get(o["name"])
            if ref is None:
                window.append(f"{o['name']}: not in the scenario file")
                continue
            cps = np.asarray(ref["control_points"], dtype=float)
            # The scenario file is always 4-wide; the planner's routine works
            # in whatever dimension the scenario was planned in, so a 2D
            # obstacle's z column is dropped before the comparison and its
            # z is not compared at all (a column has no meaningful z).
            spatial = int(scenario["spatial_dim"])
            if spatial == 2:
                cps = cps[:, [0, 1, 3]]
            active = cps[0, -1] <= t <= cps[-1, -1]
            if active != bool(o["active"]):
                window.append(f"{o['name']} at t={t:.2f}: referee says "
                              f"{'active' if o['active'] else 'absent'}, planner says "
                              f"{'active' if active else 'absent'}")
                continue
            if not active:
                continue
            want = obstacle_positions_at(cps, np.array([t]))[0]
            got = o["pos"][:spatial]
            d = float(np.linalg.norm(np.asarray(got) - want))
            if d > worst:
                worst, worst_name, worst_t = d, o["name"], t
        time.sleep(0.05)
    return (worst if seen else None), worst_name, worst_t, window


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
    sys.path.insert(0, "src/dsim_planner")
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

    # ---- 0. is this plan even flyable? ------------------------------------
    # Asked first because it decides what every number below MEANS. The
    # optimiser has no acceleration constraint, so an uncapped solve happily
    # returns a curve demanding 108 degrees of tilt; grading the tracking of
    # such a plan measures the clamp, not the plan. scripts/solve_plan.py
    # records the demand in the file.
    # Is the plan inside the vehicle's SENSING envelope? The estimator's only
    # velocity aiding is optical flow, which needs the ground in a height band,
    # and a rangefinder with a few metres of range. Above that band there is
    # nothing correcting the accelerometer and the state estimate walks away:
    # measured on `loiter`, which flies at 62.5 m -- 48 m of tracking error on
    # state:=est, 2.2 cm on state:=truth, same plan. Said here, before the
    # tracking checks, so four failures read as one fact.
    try:
        import yaml
        sensors = yaml.safe_load(open("config/drone.yaml"))["drone"]["sensors"]
        ceiling = float(sensors["optical_flow"]["max_height_m"])
        alt = [row[2] for row in P]
        if max(alt) > ceiling:
            print(f"  note  this plan reaches {max(alt):.1f} m and the optical flow "
                  f"works to {ceiling:.1f} m. Above that the estimator has no velocity "
                  f"aiding at all, so state:=est cannot fly it; use state:=truth and "
                  f"read the result as a statement about the PLAN, not the vehicle.")
    except (OSError, KeyError, ValueError, ImportError):
        pass

    demands = plan.get("demands")
    if demands is None:
        print("  note  this plan carries no feasibility record (it predates "
              "scripts/solve_plan.py); its demands were never checked")
    elif not demands.get("flyable", True):
        c.bad("the plan is flyable",
              f"it demands {demands['max_tilt_deg']:.1f} deg of tilt "
              f"({demands['max_accel_mps2']:.1f} m/s^2) against a "
              f"{demands['tilt_limit_deg']:.0f} deg clamp",
              "a plan solved with no speed cap -- the optimiser has no acceleration "
              "constraint at all. Re-solve with --v-max. Flying it anyway is a fine "
              "thing to watch, but nothing below would be a measurement of the plan.")
        return c.failures
    else:
        c.ok("the plan is flyable", f"demands {demands['max_speed_mps']:.2f} m/s, "
             f"{demands['max_accel_mps2']:.2f} m/s^2, {demands['max_tilt_deg']:.1f} deg "
             f"of tilt (clamp {demands['tilt_limit_deg']:.0f})")

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
    cruise_from = max(CRUISE_MIN_S, CRUISE_FROM_FRAC * T)
    max_err = 0.0
    max_err_cruise = 0.0
    max_err_stop = 0.0          # after the plan ends: the vehicle is still moving
    speed_samples = []          # (t, |v_ref| reported, |v| planned)
    last_t = -1.0
    deadline = time.time() + T + 15
    while time.time() < deadline:
        snap = fetch(url)
        t = scenario_time(snap)
        if t > T + 1.5:
            break
        if t != last_t and T < t <= T + 1.5:
            # Past the horizon the controller holds the final point with zero
            # velocity while the vehicle is still moving at whatever speed the
            # plan ended at -- the closing half of the same formulation gap as
            # the standing start. Measured, reported, not gated: it is the
            # planner's boundary condition, not the simulator's tracking.
            last_t = t
            max_err_stop = max(max_err_stop, (snap.get("status") or {})
                               .get("tracking_error_m", 0.0))
        if t != last_t and 0.0 <= t <= T:
            last_t = t
            st = snap.get("status") or {}
            max_err = max(max_err, st.get("tracking_error_m", 0.0))
            if t >= cruise_from:
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

    # ---- 1. referee vs planner --------------------------------------------
    worst, worst_name, worst_t, active_disagreements = obstacle_agreement(
        url, scenario, samples=41)
    if worst is None:
        c.bad("referee agrees with the planner about the obstacles",
              "the referee published no obstacle field",
              "a referee that was given no obstacle list -- the scenario's "
              "config/obstacles_<name>.yaml is missing or empty, and every "
              "clearance number below would be vacuously infinite")
    elif active_disagreements:
        c.bad("referee agrees with the planner about which obstacles exist",
              f"{len(active_disagreements)} disagree: {active_disagreements[:4]}",
              "an active window read differently on the two sides -- the referee "
              "would score against an obstacle the planner solved without, or "
              "ignore one it solved around")
    elif worst <= OBSTACLE_AGREEMENT_M:
        c.ok("referee agrees with the planner about the obstacles",
             f"max |referee - planner| {worst:.2e} m over {len(scenario['obstacles'])} "
             f"obstacles x 41 instants (limit {OBSTACLE_AGREEMENT_M:g})")
    else:
        c.bad("referee agrees with the planner about the obstacles",
              f"max |referee - planner| {worst:.3e} m on {worst_name} at t={worst_t:.2f} s",
              "a scenario clock offset between the referee and the plan, a control "
              "point dropped in the parameter flattening, or de Casteljau implemented "
              "differently on the two sides")

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
    # What the peak is allowed to be depends on whether the plan starts from
    # rest. Holding a standing-start plan to the rest-start number would fail
    # every optimised plan for a reason that is the planner's formulation, not
    # the simulator's tracking -- and holding a rest-start plan to the loose
    # number would stop catching real tracking loss. So the bound comes from
    # the plan's own declared opening speed.
    # From the plan's own record when it has one, and computed from the control
    # points when it does not. Defaulting v0 to zero made the seed -- which
    # opens at 0.9 m/s -- report "the plan starts from rest", which is a
    # statement the grader had no business making.
    if demands and "start_speed_mps" in demands:
        v0 = demands["start_speed_mps"]
    else:
        v0 = math.dist(spacetime.sample_at_time(P, P[0][3])[1], [0, 0, 0])
    limit = max(TRACK_MAX_M, OPENING_LAG_S * v0)
    why = (f"a {v0:.2f} m/s standing start, at {OPENING_LAG_S:.2f} s of closed-loop lag"
           if OPENING_LAG_S * v0 > TRACK_MAX_M
           else "the plan barely moves at t=0, so nothing excuses a large peak")
    if max_err <= limit:
        c.ok("tracking through the window", f"max error {max_err * 100:.1f} cm "
             f"(limit {limit * 100:.0f} cm -- {why})")
    else:
        c.bad("tracking through the window",
              f"max error {max_err * 100:.1f} cm against {limit * 100:.0f} cm ({why})",
              "a plan the vehicle cannot follow, or feedforward with the wrong sign")
    v_top = (demands or {}).get("max_speed_mps") or max(
        (s[2] for s in speed_samples), default=1.0)
    cruise_limit = max(CRUISE_MIN_M, CRUISE_LAG_S * v_top)
    if max_err_cruise <= cruise_limit:
        c.ok("tracking in the cruise", f"max error {max_err_cruise * 100:.1f} cm over "
             f"t={cruise_from:.1f}..{T:.0f} s (limit {cruise_limit * 100:.1f} cm = "
             f"{CRUISE_LAG_S * 1000:.0f} ms of lag at the plan's {v_top:.2f} m/s)")
    else:
        c.bad("tracking in the cruise", f"max error {max_err_cruise * 100:.1f} cm over "
              f"t={cruise_from:.1f}..{T:.0f} s against {cruise_limit * 100:.1f} cm",
              "acceleration feedforward missing or wrong on a plan that turns, or a "
              "loop that never settles after the start")
    v_end = ((demands or {}).get("end_speed_mps")
             if demands and "end_speed_mps" in demands
             else math.dist(spacetime.sample_at_time(P, P[-1][3])[1], [0, 0, 0]))
    if v_end > 0.05:
        print(f"  note  the plan ends at {v_end:.2f} m/s and the controller then holds "
              f"the final point: {max_err_stop * 100:.1f} cm of overshoot past t={T:.0f} s. "
              f"That is the planner's boundary condition, not tracking.")

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
        # An optimised plan declares no expected hit, so what decides whether
        # one is a failure is the plan's OWN certificate. A solve that came
        # back uncertified with a negative margin is a plan that says it
        # collides; the referee agreeing with it is the system working.
        promised = plan.get("predicted_clearance_m")
        certified = (plan.get("solver_info") or {}).get("certified")
        if promised is not None and promised < 0.0:
            if hits:
                c.ok("hits only where the plan admits it will",
                     f"the optimiser promised {promised:+.4f} m "
                     f"(certified={certified}) and the run hit: {summary}")
            else:
                c.bad("hits only where the plan admits it will",
                      f"the optimiser promised {promised:+.4f} m and nothing was hit",
                      "a referee that cannot see the obstacles it was given -- a plan "
                      "with a negative margin must show up as a collision somewhere")
        elif not hits:
            c.ok("no collision", f"min clearance {cl['min_clearance_m'] * 100:.1f} cm")
        else:
            c.bad("no collision", f"hits: {summary}",
                  "a plan that does not clear the obstacles once the vehicle's own "
                  "radius is counted, or one the vehicle could not follow closely enough")
    # ---- 6. the certificate against the measurement -----------------------
    pred = plan.get("predicted_clearance_m")
    measured = cl["min_clearance_m"]
    if pred is None:
        print("  note  this plan carries no predicted_clearance_m "
              "(scripts/solve_plan.py writes one); nothing to hold the referee to")
    elif measured >= pred - CLEARANCE_SLACK_M:
        c.ok("certificate matches the measurement",
             f"optimiser predicted {pred:+.4f} m, referee measured {measured:+.4f} m "
             f"(short by {pred - measured:+.4f}, allowed {CLEARANCE_SLACK_M:.2f})")
    else:
        c.bad("certificate matches the measurement",
              f"optimiser predicted {pred:+.4f} m, referee measured {measured:+.4f} m",
              "obstacles inflated by the wrong amount before solving, a scenario that "
              "means different things to the solver and the referee, or tracking error "
              "larger than the margin the plan was solved for")

    return c.failures


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "http://127.0.0.1:8080"))
