#!/usr/bin/env python3
"""Solve a scenario with the space-time Bezier optimiser and write a plan.

Run INSIDE the container, where the Rust extension is built (make planner):

    python3 scripts/solve_plan.py --scenario fence3d -N 8 --n-seg 2
    python3 scripts/solve_plan.py --scenario fence3d --sweep       # every config
    python3 scripts/solve_plan.py --scenario fence3d -N 8 --n-seg 8 --v-max 3.0

Writes plans/<scenario>_N<N>_seg<n_seg>.json: the control points, the solver's
own verdict, and the numbers a run should be judged against. Then:

    make plan PLAN=plans/fence3d_N8_seg2.json

The scenario handed to the optimiser is the SIM's scenarios/<name>.json, which
scripts/import_scenarios.py generates from the planner's own SCENARIO_MAP and
`--check` holds to it. One source, so the world Gazebo builds, the obstacles
the referee scores against and the problem the solver reads cannot drift
apart. (An earlier version solved the planner's copy too and compared; once
the file became generated that was solving the same input twice.)

Four of the seven scenarios are planned in TWO spatial dimensions plus time.
They are solved that way -- lifting them into 3D would hand the solver an
escape over the top that the 2D problem never had -- and the resulting curve
is lifted to the scenario's flight altitude afterwards, which is exact
because the altitude is constant.

Nothing here is a ROS node. The plan is a file; dsim_planner's bridge reads it.
"""
import argparse
import json
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent

#: Seconds of closed-loop lag: the peak position error a velocity STEP of v0
#: produces is about this times v0. Fitted from six flown plans between 0.70
#: and 3.00 m/s (0.322 s, residuals under 5 cm). It matters because the
#: optimiser pins endpoint positions and not velocities, so every plan opens
#: with such a step.
OPENING_LAG_S = 0.322


def vehicle():
    """What the vehicle can do, from the same config the controller reads.

    max_lateral_accel is the honest limit on a plan: a multirotor turns by
    tilting, so a horizontal acceleration of a needs a tilt of atan(a/g), and
    the controller clamps tilt at max_tilt_rad. Past that the demand is simply
    not executed -- the loop saturates and the vehicle falls behind, which is
    what "not feasible" looks like from inside the simulator.
    """
    import yaml
    cfg = yaml.safe_load((ROOT / "config" / "drone.yaml").read_text())["drone"]
    gains = yaml.safe_load(
        (ROOT / "src" / "dsim_bringup" / "config" / "gains.yaml").read_text())["controller"]
    import math
    g = float(cfg["gravity_m_s2"])
    mass = float(cfg["mass_kg"])
    rotor = cfg["rotor"]
    max_thrust = (float(rotor["count"]) * float(rotor["motor_constant"])
                  * float(rotor["max_rot_velocity"]) ** 2)
    tilt = float(gains["gains"]["max_tilt_rad"])
    return {
        "radius_m": float(cfg["radius_m"]),
        "gravity": g,
        "max_tilt_rad": tilt,
        "max_lateral_accel": g * math.tan(tilt),
        # Straight up, everything the rotors have minus what holds it up.
        "max_vertical_accel": max_thrust / mass - g,
        "thrust_to_weight": max_thrust / (mass * g),
    }


def feasibility(control_points, veh, dt=0.02):
    """What the plan demands of the vehicle, sampled in time.

    Uses the SAME conversion the bridge will use to fly it
    (dsim_planner.spacetime), so this is a statement about the trajectory the
    controller will actually be handed, not about the control polygon.
    """
    sys.path.insert(0, str(ROOT / "src" / "dsim_planner"))
    from dsim_planner import spacetime

    import math

    P = spacetime.validate(control_points)
    samples = spacetime.sample_uniform_in_time(P, dt)
    s = spacetime.summarize(samples)
    s["tilt_limit_deg"] = math.degrees(veh["max_tilt_rad"])
    s["accel_limit_mps2"] = veh["max_lateral_accel"]
    s["flyable"] = s["max_tilt_deg"] <= s["tilt_limit_deg"]
    # The optimiser pins endpoint POSITIONS and nothing else, so a plan is free
    # to begin at full speed and end still moving -- it is a segment of a
    # flight, not a flight. The vehicle reaches the start point and hovers
    # there, so a standing start is a step the loop has to chase: measured 92 cm
    # of tracking error on a 3.00 m/s start. Recorded here so the number is
    # attached to the plan instead of being rediscovered as a mystery overshoot.
    s["start_speed_mps"] = math.dist(samples[0][2], [0, 0, 0])
    s["end_speed_mps"] = math.dist(samples[-1][2], [0, 0, 0])
    # The opening error that step implies. MEASURED, not derived from the
    # acceleration limit: the loop is not acceleration-limited here, it is
    # bandwidth-limited, and the peak position error of a velocity step turns
    # out to be almost exactly proportional to the step. Fitted across six
    # runs spanning 0.70 to 3.00 m/s: 0.322 s, residuals under 5 cm. (The
    # full-authority catch-up distance v0^2/(2a) is not it -- it is quadratic,
    # and it under-predicted by 3x at the low end.)
    s["opening_error_m"] = OPENING_LAG_S * s["start_speed_mps"]
    return s


def load_scenario(name):
    """The sim's scenario as the solver wants it, at the solver's own dimension.

    scenarios/*.json always carries four coordinates per control point, because
    the referee and the viewer work in three dimensions whatever the plan was
    solved in. For a 2D scenario the z column is the flight altitude and is not
    part of the problem, so it is dropped here and put back on the answer.
    """
    path = ROOT / "scenarios" / f"{name}.json"
    if not path.exists():
        raise SystemExit(f"no such scenario: {path}\n"
                         f"have: {sorted(p.stem for p in (ROOT / 'scenarios').glob('*.json'))}")
    sc = json.loads(path.read_text())
    spatial = int(sc["spatial_dim"])

    def drop(point):
        p = [float(v) for v in point]
        return p if spatial == 3 else [p[0], p[1], p[3]]

    solved = dict(sc)
    solved["start"] = drop(sc["start"])
    solved["end"] = drop(sc["end"])
    solved["obstacles"] = [
        {"control_points": [drop(row) for row in o["control_points"]],
         "radius": float(o["radius"]),
         **{k: o[k] for k in ("name", "color") if o.get(k) is not None}}
        for o in sc["obstacles"]]
    return sc, solved


def lift_plan(control_points, sc):
    """Put the flight altitude back into a plan solved in two dimensions.

    Exact: the altitude is a constant, so a Bezier in (x, y, t) with every
    control point given the same z is the same curve flown level.
    """
    if int(sc["spatial_dim"]) == 3:
        return [[float(c) for c in row] for row in control_points]
    z = float(sc["lift_z"])
    return [[float(row[0]), float(row[1]), z, float(row[2])] for row in control_points]


def inflate(scenario, by):
    """Grow every obstacle by `by` metres.

    The optimiser solves for a POINT: its certificate says the curve stays
    `min_clearance` away from the obstacle surfaces, with no vehicle in the
    picture. The referee flies a 0.3 m envelope and subtracts it. Measured on
    fence3d N8_seg2: certified +0.1623 for a point is -0.1377 for this
    vehicle -- a plan that the solver certifies and the referee scores as a
    collision, and neither of them is wrong.

    Inflating first is the standard fix and it is exact for a sphere: a point
    clearing a radius r + 0.3 ball is a 0.3 ball clearing the radius r one.
    """
    if by <= 0.0:
        return scenario
    out = dict(scenario)
    out["obstacles"] = [dict(o, radius=float(o["radius"]) + by) for o in scenario["obstacles"]]
    return out


def solve(scenario, configs, **kw):
    from spacetime_bezier import optimize_scenario
    t0 = time.time()
    out = optimize_scenario(scenario, configs, verbose=False, **kw)
    return out, time.time() - t0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", help="which scenario to solve; see --list")
    ap.add_argument("-N", type=int, default=8, help="Bezier degree (default 8)")
    ap.add_argument("--n-seg", type=int, default=2, help="keep-out segments (default 2)")
    ap.add_argument("--sweep", action="store_true",
                    help="solve every (N, n_seg) the planner records for this "
                         "scenario and keep the best, instead of one config")
    ap.add_argument("--list", action="store_true",
                    help="list the scenarios and their recorded configs, and stop")
    ap.add_argument("--v-max", type=float, default=None,
                    help="hard speed cap, m/s. The optimiser has NO acceleration "
                         "cap, so this is the only physical bound available; the "
                         "vehicle's own limit is about 8.3 m/s^2 of lateral "
                         "acceleration at the tilt clamp and nothing enforces it.")
    ap.add_argument("--inflate", type=float, default=None,
                    help="grow every obstacle by this many metres before solving, "
                         "so the certificate is about the VEHICLE and not a point. "
                         "Defaults to the vehicle radius in config/drone.yaml; "
                         "pass 0 to solve for a point and see the difference.")
    ap.add_argument("--min-dt", type=float, default=0.1)
    ap.add_argument("--max-iter", type=int, default=200)
    ap.add_argument("--time-weight", type=float, default=0.0)
    ap.add_argument("--out", default=None, help="output path (default plans/<name>_N<N>_seg<s>.json)")
    args = ap.parse_args(argv)

    if not args.list and not args.scenario:
        ap.error("--scenario is required (or --list to see what there is)")

    if args.list:
        for path in sorted((ROOT / "scenarios").glob("*.json")):
            sc = json.loads(path.read_text())
            print(f"  {sc['name']:9} {sc['spatial_dim']}D+t  T={sc['T']:>5g}s  "
                  f"{len(sc['obstacles']):>2} obstacles  "
                  f"configs {[f'N{a}_seg{b}' for a, b in sc['configs']]}   {sc['title']}")
        return 0

    try:
        import spacetime_bezier  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            f"the space-time planner is not importable ({exc}).\n"
            f"Run this INSIDE the container after `make planner`, which builds\n"
            f"the Rust extension from /ws/spacetime. See docs/PLANNER.md.")

    veh = vehicle()
    radius = veh["radius_m"]
    grow = radius if args.inflate is None else args.inflate
    sc, scenario = load_scenario(args.scenario)
    configs = ([(args.N, args.n_seg)] if not args.sweep
               else [(int(a), int(b)) for a, b in sc["configs"]])

    kw = dict(min_dt=args.min_dt, max_iter=args.max_iter, time_weight=args.time_weight)
    if args.v_max is not None:
        kw["v_max"] = args.v_max

    print(f"solving {args.scenario} ({sc['spatial_dim']}D+t): "
          f"{len(scenario['obstacles'])} obstacles, "
          f"{scenario['start']} -> {scenario['end']}, configs {configs}")
    print(f"  obstacles inflated by {grow:.2f} m"
          + (f" (the vehicle radius)" if args.inflate is None else "")
          + ("; the certificate below is about the VEHICLE" if grow > 0 else
             "; the certificate below is about a POINT"))
    out, elapsed = solve(inflate(scenario, grow), configs, **kw)
    print(f"  {elapsed:.2f} s for {len(configs)} config(s)")

    best = out["best"]
    if best is None:
        print("\nNO CONFIG PRODUCED A USABLE PLAN. Per config:")
        for key, r in out["results"].items():
            print(f"  {key:10} converged={r['converged']} certified={r['certified']} "
                  f"min_clearance={r['min_clearance']:+.4f} "
                  f"reasons={r.get('figure_grade_reasons')}")
        return 1

    print(f"\n  {'config':10} {'conv':5} {'cert':5} {'clear':>9} {'arrival':>8} {'iters':>5}")
    for key, r in out["results"].items():
        print(f"  {key:10} {str(r['converged']):5} {str(r['certified']):5} "
              f"{r['min_clearance']:+9.4f} {r['arrival_time']:8.2f} {r['iterations']:5d}"
              + ("   <- best" if key == best else ""))

    r = out["results"][best]
    P = lift_plan(r["control_points"], sc)
    # What the referee should report, stated BEFORE the run so the run can
    # disagree with it. The optimiser certifies a point against the INFLATED
    # obstacles; the referee measures the real ones and subtracts the vehicle
    # radius. The two agree exactly when grow == radius, and differ by
    # (grow - radius) otherwise.
    predicted = r["min_clearance"] + grow - radius
    print(f"\n  certified {r['min_clearance']:+.4f} m against obstacles grown by "
          f"{grow:.2f} m; on the REAL obstacles a {radius:.2f} m vehicle has "
          f"{predicted:+.4f} m, which is what the referee should report"
          + ("" if predicted > 0 else
             "\n     <-- NEGATIVE: this plan collides for this vehicle"))

    # --- can this vehicle fly it? ------------------------------------------
    # Asked here, before the plan is written, because the optimiser has no
    # acceleration constraint at all: its only physical bound is the speed cap
    # (--v-max), and with no cap it is free to loiter and then dart. Measured
    # on fence3d N8_seg2 with no cap: 15.05 m/s and 82.61 m/s^2, needing 108.6
    # degrees of tilt against a 40 degree clamp. The plan is still written --
    # flying it and watching the loop saturate is a legitimate thing to want --
    # but it is labelled, and check_planner.py reads the label.
    fz = feasibility(P, veh)
    verdict = "flyable" if fz["flyable"] else "NOT FLYABLE BY THIS VEHICLE"
    print(f"\n  demands: {fz['max_speed_mps']:.2f} m/s, {fz['max_accel_mps2']:.2f} m/s^2, "
          f"{fz['max_tilt_deg']:.1f} deg of tilt")
    print(f"  vehicle: {veh['max_lateral_accel']:.2f} m/s^2 lateral at the "
          f"{fz['tilt_limit_deg']:.0f} deg clamp, "
          f"{veh['max_vertical_accel']:.2f} m/s^2 up  ->  {verdict}")
    if not fz["flyable"]:
        print(f"  the optimiser has NO acceleration cap. Re-solve with a speed cap:")
        print(f"      --v-max {max(0.5, fz['max_speed_mps'] / 4):.1f}    "
              f"(or lower; there is no formula, the two are only indirectly linked)")
    if fz["start_speed_mps"] > 0.05 or fz["end_speed_mps"] > 0.05:
        print(f"  standing start: the plan opens at {fz['start_speed_mps']:.2f} m/s and "
              f"ends at {fz['end_speed_mps']:.2f} m/s.")
        print(f"     The optimiser pins endpoint POSITIONS only, so this is a segment "
              f"of a flight, not a flight.")
        print(f"     The vehicle hovers at the start point and must chase that step: "
              f"about {fz['opening_error_m']:.2f} m of opening error, which is "
              f"{OPENING_LAG_S:.3f} s of closed-loop lag at that speed.")

    out_path = pathlib.Path(args.out) if args.out else (
        ROOT / "plans" / f"{args.scenario}_{best}.json")
    doc = {
        "_generated_by": f"scripts/solve_plan.py --scenario {args.scenario} "
                         f"-N {r['N']} --n-seg {r['n_seg']}"
                         + (f" --v-max {args.v_max}" if args.v_max is not None else ""),
        "scenario": args.scenario,
        "spatial_dim": int(sc["spatial_dim"]),
        "solver": r["backend"],
        "config": best,
        # The solver's own verdict, carried so a run can be read against what
        # the optimiser claimed rather than against a memory of it.
        "certified_clearance_m": r["min_clearance"],
        "inflated_by_m": grow,
        # What the referee is expected to measure on the real obstacles with
        # this vehicle. scripts/check_planner.py holds the live run to it.
        "predicted_clearance_m": round(predicted, 6),
        "solver_info": {k: r[k] for k in (
            "N", "n_seg", "converged", "certified", "feasible", "figure_grade",
            "figure_grade_reasons", "iterations", "arrival_time", "elastic_weight",
            "weight_raises", "total_slack", "max_koz_dual", "speed_cap_violation",
            "certificate_violation", "stop_label", "trust_radius",
            # Line of sight. Three keys and not one, because "certified" alone
            # is not the claim: the solver DROPS occlusion planes when a
            # station lies inside the convex hull it is building them from, and
            # a dropped plane is not a satisfied one. A run with planes dropped
            # certified nothing about those instants, and the referee is the
            # only thing that then knows whether sight was held.
            "occlusion_certified", "occlusion_violation",
            "occlusion_planes_dropped") if k in r},
        "solve_seconds": round(elapsed, 3),
        # What the plan asks of the vehicle, and whether it can. Written into
        # the file so a plan cannot be flown without the question having been
        # asked; scripts/check_planner.py refuses to grade an unflyable plan
        # as if a tracking failure were a surprise.
        "demands": {k: (round(v, 4) if isinstance(v, float) else v) for k, v in fz.items()},
        "control_points": P,
    }
    text = json.dumps({k: v for k, v in doc.items() if k != "control_points"}, indent=1)
    rows = ",\n".join("  " + json.dumps([round(c, 9) for c in row]) for row in P)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    # Written aside and renamed: the bridge reloads its plan when the file
    # changes, and a reader that catches a half-written file gets a JSON error
    # for a plan that is perfectly fine. rename(2) is atomic within a
    # filesystem, so the bridge sees either the old plan or the new one.
    tmp = out_path.with_suffix(".json.tmp")
    tmp.write_text(text[:-2] + ',\n "control_points": [\n' + rows + "\n ]\n}\n")
    tmp.replace(out_path)
    # relative_to raises for a path outside the workspace, and raising AFTER
    # the file is written reports failure for a solve that succeeded.
    try:
        shown = out_path.relative_to(ROOT)
    except ValueError:
        shown = out_path
    print(f"\nwrote {shown}")
    print(f"  make plan PLAN={out_path.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
