#!/usr/bin/env python3
"""Assert that the live telemetry is physically consistent.

Run against a flying sim:

    python3 scripts/check_telemetry.py                  # http://127.0.0.1:8080
    python3 scripts/check_telemetry.py http://host:8080

Every check here compares numbers that were produced by DIFFERENT paths, so
agreement is evidence and disagreement localises the fault. A unit test can
only prove the code does what it says; these prove the running system's
separate parts still describe the same vehicle.

What each check would catch is spelled out in its message. Exit code is the
number of failures, so this is usable as a gate.
"""
import json
import math
import sys
import urllib.request

# Tolerances, each with a reason rather than a round number.
EXACT = 1e-9          # an algebraic identity computed in one place: no slack
SAMPLE_SKEW = 2e-3    # two topics sampled at different instants at ~2 m/s
TRIG = 0.04           # 4%: the steady-turn relation ignores vertical accel
AERO_MIN_N = 0.005    # below this the residual path is dead, not quiet
AERO_MAX_N = 3.0      # a fifth of the weight; beyond that it is a bug
SPLIT_MIN_N = 0.02    # a turning quadrotor cannot have four equal rotors


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

    def near(self, name, a, b, tol, detail, would_catch):
        if abs(a - b) <= tol:
            self.ok(name, f"{detail} (|diff| {abs(a - b):.2e} <= {tol:g})")
        else:
            self.bad(name, f"{detail} (|diff| {abs(a - b):.2e} > {tol:g})",
                     would_catch)

    def between(self, name, v, lo, hi, detail, would_catch):
        if lo <= v <= hi:
            self.ok(name, f"{detail} ({lo:g} <= {v:.4g} <= {hi:g})")
        else:
            self.bad(name, f"{detail} ({v:.4g} outside [{lo:g}, {hi:g}])",
                     would_catch)


def main(url):
    snap = fetch(url)
    c = Checks()

    for key in ("pose", "control", "imu", "overlay", "state_vel"):
        if not snap.get(key):
            c.bad(f"{key} present", "missing from /snapshot",
                  "a dead subscription, or a node that never started")
            return c.failures
    ctl, imu, ov = snap["control"], snap["imu"], snap["overlay"]

    if not ctl["armed"]:
        c.bad("armed", "the controller is not commanding",
              "a disarmed or failed controller, which makes every number below "
              "meaningless")
        return c.failures

    thrust = ctl["realised_thrust_n"]
    weight = ctl["mass_kg"] * ctl["gravity_m_s2"]
    tilt = ctl["commanded_tilt_rad"]
    speed = math.dist(ctl["velocity_world"], [0, 0, 0])

    # 1. An algebraic identity the message publishes twice on purpose. The two
    #    sides come from the mixer's inverse and its forward map.
    c.near("rotor thrusts sum to the realised thrust",
           sum(ctl["rotor_thrust_n"]), thrust, EXACT,
           f"sum {sum(ctl['rotor_thrust_n']):.6f} N vs {thrust:.6f} N",
           "an allocation matrix that has stopped inverting")

    # 2. Same vector, two frames. A rotation cannot change a length, so the
    #    world-frame velocity the controller resolved must match the magnitude
    #    of the body-frame twist it was GIVEN -- state_vel, the Odometry the
    #    controller subscribes to, which is /drone/state_est by default and
    #    /drone/truth under state:=truth. Comparing against truth instead would
    #    fail by exactly the estimation error (26 mm/s on an 18 degree lap,
    #    against a 4 mm/s tolerance): a measurement of the estimator, not an
    #    identity about the controller.
    given = math.dist(snap["state_vel"], [0, 0, 0])
    c.near("velocity magnitude survives the frame change",
           speed, given, max(SAMPLE_SKEW, SAMPLE_SKEW * speed),
           f"|v_world| {speed:.4f} vs |twist given| {given:.4f} m/s",
           "a body/world frame mix-up in the odometry handling")

    # 3. Physics, not code: holding altitude at a bank angle needs
    #    weight / cos(tilt) of thrust. Neither number was computed from the
    #    other -- thrust comes from the rotor speeds, tilt from the commanded
    #    force direction -- so agreement is a real cross-check on both.
    if math.cos(tilt) > 0.1:
        expected = weight / math.cos(tilt)
        c.near("thrust matches weight / cos(tilt) in a steady turn",
               thrust, expected, TRIG * expected,
               f"{thrust:.3f} N vs {expected:.3f} N at {math.degrees(tilt):.1f} deg",
               "a wrong thrust scale, a wrong tilt, or a vehicle that is not "
               "actually holding altitude")

    # 4. The measured residual. Zero means the IMU path is dead or the
    #    subtraction cancelled something it should not have; huge means the
    #    force decomposition is wrong.
    aero = ov["readout"]["aero_n"]
    if aero is None:
        c.bad("aero force measured", "no IMU sample reached the overlay",
              "an unbridged or renamed IMU topic")
    else:
        c.between("aerodynamic residual is small but real", aero,
                  AERO_MIN_N, AERO_MAX_N, "measured minus rotor thrust",
                  "a dead IMU path (exactly zero) or a broken decomposition")

    # 5. A quadrotor turns by splitting thrust across a diagonal. While it is
    #    moving at speed, four EQUAL rotor thrusts are not a quiet vehicle --
    #    they are impossible.
    split = max(ctl["rotor_thrust_n"]) - min(ctl["rotor_thrust_n"])
    if speed > 0.5:
        c.between("rotor thrusts are split while manoeuvring", split,
                  SPLIT_MIN_N, 4 * ctl["max_rotor_thrust_n"],
                  f"spread across the four rotors at {speed:.2f} m/s",
                  "a mixer that sends every rotor the same command, which "
                  "would make the vehicle uncontrollable in attitude")
    else:
        print(f"  skip  rotor split: only {speed:.2f} m/s, nothing to split for")

    # 6. The overlay the browser draws must actually contain the arrows the
    #    page expects, or the picture is silently incomplete.
    kinds = [a["kind"] for a in ov["arrows"]]
    expect = {"rotor": 4, "thrust": 1, "weight": 1, "aero": 1,
              "velocity": 1, "body_axis": 1, "cmd_axis": 1}
    for kind, n in expect.items():
        got = kinds.count(kind)
        if got == n:
            c.ok(f"overlay has {n} {kind} arrow(s)", "as the page expects")
        else:
            c.bad(f"overlay has {n} {kind} arrow(s)", f"found {got}",
                  "an overlay the viewer would draw incompletely, with no error")
    if len(ov["ticks"]) == 4:
        c.ok("overlay has 4 hover ticks", "one per rotor")
    else:
        c.bad("overlay has 4 hover ticks", f"found {len(ov['ticks'])}",
              "missing hover references, so above/below hover is unreadable")

    # 7. Nothing on the wire may be NaN or infinite: JSON has no NaN, so a
    #    non-finite number arrives as invalid JSON or as a string and the page
    #    silently stops drawing.
    def finite(v):
        return all(isinstance(x, (int, float)) and math.isfinite(x) for x in v)
    bad_arrow = next((a for a in ov["arrows"]
                      if not (finite(a["from"]) and finite(a["to"]))), None)
    if bad_arrow is None:
        c.ok("every arrow endpoint is finite", f"{len(ov['arrows'])} arrows")
    else:
        c.bad("every arrow endpoint is finite", f"{bad_arrow['kind']} is not",
              "a NaN from a zero-length normalisation, which blanks the overlay")

    print()
    if c.failures == 0:
        print(f"TELEMETRY OK: {len(ov['arrows'])} arrows, all cross-checks agree "
              f"(thrust {thrust:.2f} N, tilt {math.degrees(tilt):.1f} deg, "
              f"{speed:.2f} m/s)")
    else:
        print(f"TELEMETRY FAILED: {c.failures} check(s)")
    return c.failures


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8080"))
