#!/usr/bin/env python3
"""Import the planner's scenarios into scenarios/*.json.

Run INSIDE the container, where spacetime_bezier imports:

    python3 scripts/import_scenarios.py            # regenerate
    python3 scripts/import_scenarios.py --check    # fail if any drifted

These files were hand-copied once and that was a standing drift risk: a
scenario that means one thing to the solver and another to the referee is the
whole failure the generator chain exists to prevent. Now they are GENERATED
from SCENARIO_MAP, in the planner's own canonical form -- obstacles as lifted
control points in (x, y, [z], t), with the active window intrinsic to the
first and last of them -- so nothing is transcribed and `--check` catches a
planner change that the sim has not been told about.

Two things this adds on top of the planner's dict, both under keys the planner
ignores:

  spatial_dim / lift_z
      Four of the seven scenarios are planned in TWO spatial dimensions plus
      time. They are NOT lifted into 3D by making their obstacles spheres:
      that would hand the solver an escape route over the top that the 2D
      problem never had, and the scenario would stop being the scenario. They
      stay 2D for the solver, and the sim flies the resulting plan at a fixed
      altitude, with each obstacle treated as the vertical COLUMN it is -- a
      disc extruded through every height, which is what a 2D obstacle means.

  sim.start_s / sim.spawn
      When scenario t=0 happens in simulated seconds, and where the vehicle
      starts. It spawns AT the plan's first point and settles there; the
      transit from a pad is not part of any plan and nothing certifies it, so
      there is no reason to fly one. (loiter's start is 62.5 m up; a climb to
      it would take a minute of nothing.)
"""
import argparse
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_DIR = ROOT / "scenarios"

#: Altitude a 2-spatial-dimension scenario is flown at, in metres.
LIFT_Z = 1.5
#: Simulated seconds before scenario t=0: the vehicle spawns at the plan's
#: first point and holds there. Long enough for the attitude estimator to
#: settle (it initialises from the first mag and accel samples) and for the
#: viewer to be connected before anything moves.
START_S = 5.0


def scenario_doc(name, factory, configs):
    sc = factory()
    spatial = len(sc["start"]) - 1
    if spatial not in (2, 3):
        raise SystemExit(f"{name}: {spatial} spatial dimensions; this sim flies in 3")

    def lift(point):
        """(x, y, t) -> (x, y, LIFT_Z, t), or pass a 3D point through."""
        p = [float(v) for v in point]
        return p if spatial == 3 else [p[0], p[1], LIFT_Z, p[-1]]

    obstacles = []
    for o in sc["obstacles"]:
        cps = [lift(row) for row in o["control_points"]]
        entry = {
            "name": o.get("name") or f"O{len(obstacles)}",
            # A 2D obstacle is a disc at every altitude, not a ball at one.
            "type": "sphere" if spatial == 3 else "column",
            "radius": float(o.get("radius", o.get("r"))),
            "control_points": cps,
        }
        if o.get("color"):
            entry["color"] = o["color"]
        obstacles.append(entry)

    doc = {
        "_generated_by": "scripts/import_scenarios.py from the planner's SCENARIO_MAP "
                         "-- do not edit; edit the planner and re-import",
        "name": name,
        "title": sc.get("title", name),
        "spatial_dim": spatial,
        "start": lift(sc["start"]),
        "end": lift(sc["end"]),
        "T": float(sc["T"]),
        # The (degree, segments) pairs the planner records as worth solving for
        # this scene. `make solve SOLVE_ARGS=--sweep` walks them.
        "configs": [[int(a), int(b)] for a, b in configs],
        "obstacles": obstacles,
        "sim": {"start_s": START_S, "spawn": lift(sc["start"])[:3]},
    }
    if spatial == 2:
        doc["lift_z"] = LIFT_Z
    for key in ("init_curve", "stations", "coord_bounds", "trust_radius"):
        if sc.get(key) is not None:
            doc[key] = sc[key]
    return doc


def render(doc):
    """Stable text: control points one per line, everything else indented."""
    obstacles = doc.pop("obstacles")
    body = json.dumps(doc, indent=1)
    doc["obstacles"] = obstacles
    rows = []
    for o in obstacles:
        cps = ",\n".join("    " + json.dumps([round(c, 9) for c in row])
                         for row in o["control_points"])
        head = {k: v for k, v in o.items() if k != "control_points"}
        rows.append("  " + json.dumps(head)[:-1] + ', "control_points": [\n'
                    + cps + "\n  ]}")
    return body[:-2] + ',\n "obstacles": [\n' + ",\n".join(rows) + "\n ]\n}\n"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args(argv)

    try:
        from spacetime_bezier import SCENARIO_MAP
    except ImportError as exc:
        raise SystemExit(
            f"the space-time planner is not importable ({exc}).\n"
            f"Run this INSIDE the container after `make planner`. See docs/PLANNER.md.")

    OUT_DIR.mkdir(exist_ok=True)
    stale, seen = [], set()
    for name, (factory, configs) in sorted(SCENARIO_MAP.items()):
        path = OUT_DIR / f"{name}.json"
        seen.add(path.name)
        new = render(scenario_doc(name, factory, configs))
        old = path.read_text() if path.exists() else None
        if args.check:
            if old != new:
                stale.append(path.name)
        else:
            path.write_text(new)
            doc = json.loads(new)
            print(f"{'unchanged' if old == new else 'wrote    '}  scenarios/{path.name}"
                  f"  ({doc['spatial_dim']}D+t, {len(doc['obstacles'])} obstacles, "
                  f"T={doc['T']:g}s)")

    orphans = sorted(p.name for p in OUT_DIR.glob("*.json") if p.name not in seen)
    if orphans:
        # A scenario file with no scenario behind it would still generate a
        # world and a seed, and would be flyable and ungradeable.
        print(f"\nscenarios/ has files the planner does not define: {orphans}")
        print("  delete them, or add them to the planner's SCENARIO_MAP")
        return 1

    if args.check:
        if stale:
            print("STALE (run scripts/import_scenarios.py):")
            for s in stale:
                print("  scenarios/" + s)
            return 1
        print("every scenario matches the planner")
        return 0
    print("\nnow run: python3 scripts/gen_assets.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
