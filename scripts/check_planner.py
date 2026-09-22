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
     tighter CRUISE_MAX_M over the cruise -- the last 40% of the plan, which
     is after the opening transient and before the stop;
  5. the hits are the hits the plan predicts. A plan file may declare
     `expect` -- the seed does, because it is a straight line through the
     fence and a referee that lets it through is broken; an optimised plan
     declares none and must produce none;
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

WORLD_MISMATCH_M = 0.02    # 20 mm; gravity left on drifts a body 10 mm/s, caught in 2 s
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
CRUISE_MAX_M = 0.10        # the loop holds a plan to a few cm; 10 cm is a real loss
#: The vehicle's lateral acceleration at the tilt clamp, g*tan(0.7) -- used
#: only to size the standing-start allowance for a plan that carries no
#: feasibility record of its own. solve_plan.py computes the same number from
#: config/drone.yaml and gains.yaml.
LATERAL_ACCEL_LIMIT = 8.26
#: Peak tracking error allowed in the window for a plan that STARTS FROM REST.
#: A plan that opens at speed is a different question -- see below.
TRACK_MAX_M = 0.30
#: For a plan with a standing start, the peak is bounded by the catch-up
#: distance the plan's own opening speed implies, times this. The optimiser
#: pins endpoint positions and not velocities, so the vehicle hovering at the
#: start point is handed a velocity step; v0^2/(2 a_max) is the distance the
#: reference gains while the vehicle reaches v0 using every bit of its lateral
#: authority, and the loop does not use all of it immediately. Measured: 0.92 m
#: against a 0.545 m floor on a 3.00 m/s start, i.e. 1.7x.
STANDING_START_FACTOR = 2.5
HIT_TIME_TOL_S = 0.30      # entry time vs the planned-trajectory prediction
HIT_DEPTH_TOL_M = 0.10
#: How far the measured minimum clearance may fall below the certificate. The
#: certificate is about the PLANNED curve; the vehicle flies the tracking error
#: away from it, so the measured margin is smaller by up to that error. Sized
#: against the cruise tracking limit above, not picked round.
CLEARANCE_SLACK_M = CRUISE_MAX_M


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

    # ---- 0. is this plan even flyable? ------------------------------------
    # Asked first because it decides what every number below MEANS. The
    # optimiser has no acceleration constraint, so an uncapped solve happily
    # returns a curve demanding 108 degrees of tilt; grading the tracking of
    # such a plan measures the clamp, not the plan. scripts/solve_plan.py
    # records the demand in the file.
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
        deficit = demands.get("standing_start_deficit_m", 0.0)
    else:
        v0 = math.dist(spacetime.sample_at_time(P, P[0][3])[1], [0, 0, 0])
        deficit = v0 ** 2 / (2 * LATERAL_ACCEL_LIMIT)
    if v0 > 0.05:
        limit = max(TRACK_MAX_M, STANDING_START_FACTOR * deficit)
        why = (f"a {v0:.2f} m/s standing start implies at least {deficit * 100:.0f} cm "
               f"of catch-up even at full authority")
    else:
        limit = TRACK_MAX_M
        why = "the plan starts from rest, so nothing excuses a large peak"
    if max_err <= limit:
        c.ok("tracking through the window", f"max error {max_err * 100:.1f} cm "
             f"(limit {limit * 100:.0f} cm -- {why})")
    else:
        c.bad("tracking through the window",
              f"max error {max_err * 100:.1f} cm against {limit * 100:.0f} cm ({why})",
              "a plan the vehicle cannot follow, or feedforward with the wrong sign")
    if max_err_cruise <= CRUISE_MAX_M:
        c.ok("tracking in the cruise", f"max error {max_err_cruise * 100:.1f} cm over "
             f"t={cruise_from:.1f}..{T:.0f} s (limit {CRUISE_MAX_M * 100:.0f} cm)")
    else:
        c.bad("tracking in the cruise", f"max error {max_err_cruise * 100:.1f} cm over "
              f"t={cruise_from:.1f}..{T:.0f} s",
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
        if not hits:
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
