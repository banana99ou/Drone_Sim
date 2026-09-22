"""Re-solve the running scenario from the viewer.

This is the SECOND write path this server exposes, and it is deliberately not
part of the first. /control is documented as being unable to run a command;
widening it to spawn a solver would have made that sentence false while leaving
it on the page. So solving lives here, with its own endpoint, its own audit
line and its own switch.

What it can express, and nothing else:

  * the scenario is FIXED, taken from the launch. It is not a request field,
    because the Gazebo world is generated per scenario -- solving a different
    one would produce a plan for obstacles that are not in the running world.
  * three numbers, each range-checked against a bound with a reason;
  * the output path is FIXED: the plan file the running bridge already reads.

The solver runs as a subprocess with an argument LIST -- never a shell, never
a string -- of a fixed script at a fixed path. No field of the request reaches
a filename, a flag name, or an interpreter.

The bridge notices the new file by its mtime and reloads it, so the effect of
a solve is a new trajectory on the running vehicle within about a second. No
node is restarted and nothing is respawned here.
"""
import json
import os
import subprocess
import sys

#: Bézier degree. Below 2 there is no interior control point to move and the
#: curve is the straight seed; above 20 the QP grows without the scenes here
#: needing it, and a browser field should not be able to ask for a ten-minute
#: solve.
N_RANGE = (2, 20)
#: Keep-out segments: how finely the obstacle tube is cut for the convex rows.
#: 1 is one plane for the whole curve; 16 is far past the point where fence3d's
#: certificate stops moving (measured: +0.0144 at both seg8 and N10_seg8).
NSEG_RANGE = (1, 16)
#: Speed cap in m/s. 0 means "no cap", which is the optimiser's own default and
#: is the setting that produces unflyable plans -- allowed, because seeing that
#: happen is the point of having the knob.
VMAX_RANGE = (0.0, 20.0)
#: A solve of the shipped scenarios takes 0.2-1.4 s. A minute is not a
#: performance budget, it is a stuck-process bound.
TIMEOUT_S = 60.0


class SolveError(ValueError):
    """The request was refused. The message is safe to show the user."""


def _number(body, key, lo, hi, kind):
    if key not in body:
        raise SolveError(f"missing {key}")
    try:
        v = kind(body[key])
    except (TypeError, ValueError):
        raise SolveError(f"{key} must be a {kind.__name__}")
    if not lo <= v <= hi:
        raise SolveError(f"{key} must be between {lo} and {hi}, got {v}")
    return v


class Solver:
    """Runs scripts/solve_plan.py for the ONE scenario this sim is flying."""

    def __init__(self, ws, scenario, plan_file, enabled):
        self.ws = ws
        self.scenario = scenario
        self.plan_file = plan_file
        # Enabled only when this launch actually has a plan and a scenario.
        # Without both there is nothing to solve and nowhere to put it, and a
        # button that quietly does nothing is worse than one that is not there.
        self.enabled = bool(enabled and scenario and plan_file)
        self.last = None

    @property
    def state(self):
        """What the page needs to draw the panel and label the plan."""
        return {
            "enabled": self.enabled,
            "scenario": self.scenario,
            "plan_file": os.path.basename(self.plan_file) if self.plan_file else None,
            "n_range": list(N_RANGE),
            "nseg_range": list(NSEG_RANGE),
            "vmax_range": list(VMAX_RANGE),
            "plan": self.plan_summary(),
            "last": self.last,
        }

    def plan_summary(self):
        """The plan on disk right now, as the page should label it."""
        if not self.plan_file:
            return None
        try:
            with open(self.plan_file) as fh:
                doc = json.load(fh)
        except (OSError, ValueError):
            return None
        d = doc.get("demands") or {}
        return {
            "solver": doc.get("solver"),
            "config": doc.get("config"),
            "certified_clearance_m": doc.get("certified_clearance_m"),
            "predicted_clearance_m": doc.get("predicted_clearance_m"),
            "max_speed_mps": d.get("max_speed_mps"),
            "max_accel_mps2": d.get("max_accel_mps2"),
            "max_tilt_deg": d.get("max_tilt_deg"),
            "tilt_limit_deg": d.get("tilt_limit_deg"),
            "flyable": d.get("flyable"),
            "start_speed_mps": d.get("start_speed_mps"),
        }

    def solve(self, body):
        if not self.enabled:
            raise SolveError("this run has no plan to re-solve "
                             "(launch with plan:=<file>)")
        n = _number(body, "N", *N_RANGE, int)
        nseg = _number(body, "n_seg", *NSEG_RANGE, int)
        vmax = _number(body, "v_max", *VMAX_RANGE, float)

        argv = [sys.executable, "scripts/solve_plan.py",
                "--scenario", self.scenario,
                "-N", str(n), "--n-seg", str(nseg),
                "--out", self.plan_file]
        if vmax > 0.0:
            argv += ["--v-max", str(vmax)]

        try:
            proc = subprocess.run(argv, cwd=self.ws, capture_output=True,
                                  text=True, timeout=TIMEOUT_S)
        except subprocess.TimeoutExpired:
            raise SolveError(f"the solver did not finish in {TIMEOUT_S:.0f} s")
        except OSError as exc:
            raise SolveError(f"could not run the solver: {exc}")

        out = (proc.stdout or "") + (proc.stderr or "")
        if proc.returncode != 0:
            # The script's own refusals are informative -- a scenario that
            # drifted, a config that produced nothing usable -- so they are
            # passed through rather than replaced with a status code.
            tail = "\n".join(out.strip().splitlines()[-6:])
            raise SolveError(tail or f"the solver exited {proc.returncode}")

        self.last = {"N": n, "n_seg": nseg, "v_max": vmax,
                     "log": "\n".join(out.strip().splitlines()[-14:])}
        return dict(self.state, message=f"solved N={n} n_seg={nseg} "
                    + (f"v_max={vmax:g}" if vmax > 0 else "uncapped"))
