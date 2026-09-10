#!/usr/bin/env python3
"""Exercise the simulator's write path against a running sim, and prove the
world survives it.

    python3 scripts/check_simcontrol.py                 # http://127.0.0.1:8080
    python3 scripts/check_simcontrol.py http://host:8080

WHY THIS SCRIPT EXISTS.

The viewer once had a speed slider that deleted the world's gravity. Moving it
called `/world/<name>/set_physics`, gz.msgs.Physics has no gravity field,
gz-sim assigned gravity from the message anyway, and proto3 reads an absent
field as (0, 0, 0). The vehicle became weightless and left. It was found at
41.5 km, eleven hours later.

Nothing caught it, and the reason is worth stating plainly: there were fifty
unit tests around that code and not one of them touched a simulator. The
closest was an assertion that the command STRING contained "max_step_size:
0.001" -- a test of punctuation that passed happily throughout.

So the checks here are about the running world, and the central one is
IMPOSSIBLE TO PASS WITHOUT GRAVITY: a 1.5 kg vehicle in steady flight must be
producing about 14.7 N of thrust. Weightless, the controller finds it needs
none, hits its minimum-thrust floor, and produces about 0.5 N. There is a
factor of thirty between the two states; no tolerance choice can confuse them.

WHAT DOES NOT WORK AS A CHECK, and this is the trap that hid the bug for eleven
hours: the IMU. A weightless vehicle coasting at constant velocity reads
exactly 1 g of specific force, the same as a hovering one. Every health check
that consulted the accelerometer stayed green while the drone climbed past
30 km. Weight has to be inferred from what the ROTORS are doing.

Exit code is the number of failures.
"""
import json
import math
import sys
import time
import urllib.request

BASE = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8080"

# --- tolerances, each with a reason rather than a round number --------------

#: Steady-flight thrust must match weight/cos(tilt) this closely. Loose enough
#: for a banked turn's vertical acceleration, which the relation ignores;
#: nowhere near loose enough to admit a weightless vehicle (0.5 N vs 15.4 N).
WEIGHT_FRAC = 0.10
#: Tracking error above which the vehicle is not flying the plan any more. The
#: demo lap holds a few centimetres; a weightless one passes this within two
#: seconds.
TRACK_MAX_M = 1.0
#: Measured speed must land within this of the requested one. The world runs
#: in bursts a tick long and the measurement window is a second of wall time,
#: so a few percent is expected; a factor of two is not -- and a factor of two
#: is exactly what three control nodes pacing one world produced, so this is
#: the check that would catch that happening again.
SPEED_FRAC = 0.25
#: Simulated seconds a paused world may advance over two wall seconds, once it
#: has settled. A step already in flight when the request lands is legitimate;
#: anything more means the pause did not take. A world that ignored the request
#: would move 2 s, a hundred times this.
PAUSED_DRIFT_S = 0.02
#: How far simulated time must go BACKWARDS for a reset to have happened. The
#: check runs after the vehicle has been flying for a while, so a real reset
#: drops the clock by a minute or more; anything under this is the clock
#: carrying on.
RESET_DROP_S = 10.0


def get(path="/snapshot"):
    with urllib.request.urlopen(BASE + path, timeout=10) as r:
        return json.loads(r.read().decode())


def post(body):
    req = urllib.request.Request(
        BASE + "/control", data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            return r.status, json.loads(r.read().decode())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode() or "{}")


class Checks:
    def __init__(self):
        self.failures = 0

    def ok(self, name, detail):
        print(f"  ok    {name}: {detail}")

    def bad(self, name, detail, would_catch):
        print(f"  FAIL  {name}: {detail}")
        print(f"        would catch: {would_catch}")
        self.failures += 1

    def that(self, cond, name, detail, would_catch):
        if cond:
            self.ok(name, detail)
        else:
            self.bad(name, detail, would_catch)
        return cond


def has_weight(snap):
    """Is this vehicle being held up against gravity?

    Steady flight at a bank angle needs thrust = m*g / cos(tilt). Returns
    (verdict, measured_N, expected_N). A weightless vehicle sits at its
    minimum-thrust floor, roughly thirty times below expected.
    """
    c = snap["control"]
    q = snap["pose"]["q"]
    # cos(tilt) is the body z axis projected on world z, straight from the
    # quaternion: 1 - 2(x^2 + y^2).
    cos_tilt = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
    expected = c["mass_kg"] * c["gravity_m_s2"] / max(cos_tilt, 1e-6)
    measured = c["realised_thrust_n"]
    return abs(measured - expected) <= WEIGHT_FRAC * expected, measured, expected


def self_test(ck):
    """Prove the weight predicate can fail, on numbers rather than on trust.

    Required by the rule that a check which cannot fail is not evidence: the
    predicate above is the entire regression test for the bug this script
    exists for, so it has to be shown rejecting the failure it is looking for.
    """
    hovering = {"pose": {"q": [1.0, 0.0, 0.0, 0.0]},
                "control": {"mass_kg": 1.5, "gravity_m_s2": 9.80665,
                            "realised_thrust_n": 14.71}}
    weightless = json.loads(json.dumps(hovering))
    weightless["control"]["realised_thrust_n"] = 0.5      # the min-thrust floor
    ck.that(has_weight(hovering)[0], "self-test/accepts a flying vehicle",
            "14.71 N against 14.71 N expected",
            "a predicate so tight it rejects healthy flight, making the real "
            "check useless noise")
    ck.that(not has_weight(weightless)[0], "self-test/rejects a weightless one",
            "0.50 N against 14.71 N expected",
            "a predicate that passes for the exact failure it exists to "
            "detect -- which is what the previous test suite did")


def settle(seconds=4.0):
    time.sleep(seconds)
    return get()


def wait_flying(ck, limit=90.0):
    """Wait for the vehicle to be established on its trajectory."""
    end = time.time() + limit
    while time.time() < end:
        snap = get()
        if snap.get("status") and snap["t"] > 12.0 and \
                snap["status"]["tracking_error_m"] < TRACK_MAX_M:
            return snap
        time.sleep(1.0)
    ck.bad("setup/flying", f"no settled flight within {limit:g} s",
           "a sim that is not actually flying, which would make every check "
           "below meaningless rather than failing honestly")
    return None


def main():
    ck = Checks()
    print(f"simulation control check against {BASE}")

    print("\nself-test (no simulator involved)")
    self_test(ck)

    print("\nbaseline")
    snap = wait_flying(ck)
    if snap is None:
        return 1
    good, measured, expected = has_weight(snap)
    ck.that(good, "baseline/weight",
            f"thrust {measured:.2f} N vs weight/cos(tilt) {expected:.2f} N",
            "a world that was already weightless before this script touched it")

    sim = snap["sim"]
    if not ck.that(sim.get("enabled"), "baseline/control enabled",
                   f"error: {sim.get('error')!r}",
                   "a sim launched with control:=false, where nothing below "
                   "can be exercised"):
        return ck.failures

    # --- the regression test for the bug this script exists for -------------
    print("\nplayback speed")
    for speed in (0.25, 0.5, 1.0):
        code, res = post({"speed": speed})
        if not ck.that(code == 200, f"speed {speed}x/accepted",
                       f"HTTP {code}: {res.get('error')}",
                       "a speed the node refuses but the page offers"):
            continue
        snap = settle(6.0)
        got = snap["sim"]["achieved_speed"]
        ck.that(abs(got - speed) <= SPEED_FRAC * speed,
                f"speed {speed}x/achieved",
                f"measured {got:.3f}x (requested {speed:g}x)",
                "a slider that reports a speed the world is not running at")
        good, measured, expected = has_weight(snap)
        ck.that(good, f"speed {speed}x/still has weight",
                f"thrust {measured:.2f} N vs {expected:.2f} N expected",
                "THE BUG: a speed change that leaves the world weightless. "
                "This is the check the old suite did not have")
        err = snap["status"]["tracking_error_m"]
        ck.that(err < TRACK_MAX_M, f"speed {speed}x/still tracking",
                f"tracking error {err * 100:.1f} cm",
                "a vehicle that stopped flying the plan when the speed changed")

    # --- pause really stops time --------------------------------------------
    print("\npause")
    code, _ = post({"paused": True})
    ck.that(code == 200, "pause/accepted", f"HTTP {code}", "a pause that errors")
    # Settle before sampling. sim_time_s is whatever the world last said in a
    # statistics message, and those come five times a second, so a sample taken
    # the instant the request returns can be up to 200 ms old -- and the
    # difference against a later sample then reads as the world having run on
    # after being paused. Measured 25 ms of phantom drift that way, which is
    # the staleness of the sample and not the behaviour of the world.
    time.sleep(0.5)
    t0 = get()["sim"]["sim_time_s"]
    time.sleep(2.0)
    t1 = get()["sim"]["sim_time_s"]
    ck.that(abs(t1 - t0) <= PAUSED_DRIFT_S, "pause/time stops",
            f"simulated time moved {abs(t1 - t0) * 1000:.1f} ms over 2 s of wall time",
            "a pause button that reports paused while the world keeps running")

    code, res = post({"toggle_pause": True})
    ck.that(code == 200 and not res.get("paused"), "pause/toggle resumes",
            f"HTTP {code}, paused={res.get('paused')}",
            "a toggle that does not flip, which is what a page computing the "
            "new state from a stale cache would produce")
    snap = settle(5.0)
    t2 = snap["sim"]["sim_time_s"]
    ck.that(t2 > t1 + 1.0, "pause/time restarts",
            f"simulated time advanced {t2 - t1:.2f} s after resuming",
            "a world that never starts again once paced -- the failure mode of "
            "a control node timed by the clock it stops")

    # --- reset ---------------------------------------------------------------
    print("\nreset")
    before = get()["status"]
    t_before = get()["sim"]["sim_time_s"]
    code, res = post({"reset": True})
    ck.that(code == 200, "reset/accepted", f"HTTP {code}: {res.get('error')}",
            "a reset button that errors")
    time.sleep(1.0)
    snap = get()
    # Measured as a DROP, not as an absolute value. The reset now blocks until
    # the clock has actually restarted, so by the time this reads it the new
    # run is already several seconds old -- and a threshold on the absolute
    # value fails for a reset that worked perfectly.
    dropped = t_before - snap["sim"]["sim_time_s"]
    ck.that(dropped > RESET_DROP_S, "reset/clock restarts",
            f"simulated time went from {t_before:.1f} s back to "
            f"{snap['sim']['sim_time_s']:.2f} s",
            "a reset that does not reset the world clock")
    ck.that(snap["sim"]["resets"] >= 1, "reset/counted",
            f"{snap['sim']['resets']} reset(s) reported",
            "a reset the control node did not record, so a viewer cannot tell "
            "a young run from a wrong one")

    st = snap["status"]
    ck.that(st["path_length_m"] < before["path_length_m"], "reset/referee restarts",
            f"path length {before['path_length_m']:.1f} m -> {st['path_length_m']:.1f} m",
            "metrics carried across a reset. The referee once reported an RMSE "
            "of 1118 m for a vehicle tracking a circle to 5 cm, because "
            "elapsed_s followed the clock back to zero and the sum of squares "
            "did not")
    ck.that(st["tracking_max_m"] < before["tracking_max_m"] + 1e-9,
            "reset/worst case restarts",
            f"max error {before['tracking_max_m']:.2f} m -> {st['tracking_max_m']:.2f} m",
            "a peak error from a previous run being reported as this one's")

    snap = wait_flying(ck)
    if snap is not None:
        good, measured, expected = has_weight(snap)
        ck.that(good, "reset/flies again",
                f"thrust {measured:.2f} N vs {expected:.2f} N expected",
                "a reset that leaves the vehicle unable to fly")

    # Gazebo's own reset resumes the world. If that is not put back, the
    # simulator is running while this stack reports it paused -- the viewer
    # showed PAUSED over a flying vehicle before this was fixed.
    post({"paused": True})
    time.sleep(0.5)
    post({"reset": True})
    time.sleep(1.5)
    t0 = get()["sim"]["sim_time_s"]
    time.sleep(2.0)
    after = get()
    ck.that(after["sim"]["paused"], "reset/keeps the world paused",
            f"reports paused={after['sim']['paused']}",
            "a reset that silently resumes a paused simulator")
    ck.that(abs(after["sim"]["sim_time_s"] - t0) <= PAUSED_DRIFT_S,
            "reset/paused really means stopped",
            f"simulated time moved {abs(after['sim']['sim_time_s'] - t0) * 1000:.1f} ms",
            "a stack that REPORTS paused after a reset while the world runs on. "
            "The report and the world have to be checked separately, or one "
            "covers for the other")
    post({"paused": False})

    # The speed must survive a pause, or the slider silently reverts to 1x
    # every time someone pauses to look at something.
    post({"speed": 0.25})
    post({"paused": True})
    res = post({"paused": False})[1]
    ck.that(abs(res.get("requested_speed", 0) - 0.25) < 1e-9,
            "speed survives a pause",
            f"resumed at {res.get('requested_speed')}x",
            "a pause that forgets the playback speed")
    post({"speed": 1.0})

    # --- the endpoint must not accept the old, dangerous vocabulary ---------
    print("\nrefusals")
    code, res = post({"rtf": 0.5})
    ck.that(code == 400 and "speed" in res.get("error", ""),
            "refuses the old rtf key",
            f"HTTP {code}: {res.get('error')}",
            "a re-introduced real-time-factor path, which is the exact call "
            "that deletes the world's gravity")
    code, res = post({})
    ck.that(code == 400, "refuses an empty command", f"HTTP {code}",
            "a POST that means nothing being reported as applied")
    code, res = post({"speed": 99})
    ck.that(code == 400, "refuses an out-of-range speed", f"HTTP {code}",
            "a speed the simulator cannot honour being accepted anyway")
    # The ceiling is a measured fact about this world, not a preference: the
    # SDF throttles it to real time and the only service that lifts that is the
    # one that deletes gravity. A build that offers more than 1x is offering
    # something it cannot deliver.
    code, res = post({"speed": 2.0})
    ck.that(code == 400, "refuses faster than real time", f"HTTP {code}",
            "a slider that promises fast-forward. Stepping does not bypass "
            "Gazebo's real-time throttle -- 10 s of simulated time takes "
            "10.38 s of wall clock -- so 2x would silently be 1x")

    print()
    if ck.failures:
        print(f"SIMCONTROL FAIL: {ck.failures} check(s) failed")
    else:
        print("SIMCONTROL OK")
    return ck.failures


if __name__ == "__main__":
    sys.exit(main())
