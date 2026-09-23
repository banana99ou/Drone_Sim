"""Switch which run the simulator is flying, from the viewer.

This is the THIRD write path on this port, and like /solve it is kept apart
from /control rather than folded into it: /control is documented as being
unable to run a command, and widening it to restart the simulator would have
made that sentence false while leaving it on the page.

WHY THIS ENDPOINT CAN EXIST AT ALL.  The viewer server used to be a node inside
sim.launch.py, alongside Gazebo and the controller.  A server that restarts the
launch it is part of kills itself mid-reply, and the page is left staring at a
closed socket with nothing to say about what happened -- which is the one
moment you most want a message.  So the server moved out into viz.launch.py,
and only the simulator half is cycled here.  The page stays up across a switch
and reports the outcome, including a launch that died on its first second.

WHAT A REQUEST MAY EXPRESS, and nothing else: the NAME of one run out of a
catalogue built from disk.  Not a world file, not a plan path, not a launch
argument, not a number.  The catalogue is the same object the page draws its
dropdown from, so the server cannot offer a run it would then refuse, and
cannot be asked for one it never offered.

TEARDOWN uses scripts/kill_sim.sh, run as a CHILD of this process.  That is
load-bearing: the script excludes its own ancestors from the kill (see its
header), so the viewer is spared by the mechanism that is already there rather
than by a hand-written exception to a pattern list whose entire design is that
it is derived and never typed.
"""
import json
import os
import re
import subprocess
import time

#: Built-in reference trajectories: no planner, no obstacles, the empty world.
#: `period` is the knob that decides whether there is anything to watch.  A
#: circle needs bank = atan(4*pi^2*r / (T^2*g)), so 2 m at 3.5 s is 33 degrees
#: with the rotor thrusts visibly split, and 2 m at 12 s is 3 degrees, which
#: looks like a hover with extra steps.  These are the numbers scripts/
#: run_sim.sh defaults to, for the same reason.
REFERENCE_RUNS = (
    {"name": "hover", "title": "hover (1.5 m)", "world": "empty",
     "reference": "hover", "radius": 1.0, "period": 12.0, "altitude": 1.5},
    {"name": "circle", "title": "circle (2 m, 3.5 s lap)", "world": "empty",
     "reference": "circle", "radius": 2.0, "period": 3.5, "altitude": 1.5,
     "state_reason": "a 2 m circle at 3.5 s a lap needs 33 deg of bank; "
                     "the tilt clamp is 40"},
)

#: How long kill_sim.sh gets.  It sends TERM, sleeps 2, sends KILL, sleeps 1,
#: then verifies -- so its own floor is about 3 s and anything past 30 means it
#: is stuck, not slow.
KILL_TIMEOUT_S = 30.0

#: How long a launch gets to produce telemetry before it is called failed.
#: scripts/run_sim.sh waits 80 s for the same signal from the same launch; this
#: is that bound with room for a slower first start after a rebuild.
START_TIMEOUT_S = 90.0


#: Gazebo and ros2 launch colour their output, and the log is read back into a
#: <pre> in the page, where the escape sequences show up as literal "[1;32m"
#: in front of every word. Stripped on the way out, not on the way in: the file
#: on disk stays exactly what the launch wrote, for `tail -f` on a terminal
#: that can render it.
_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


class LaunchError(ValueError):
    """The request was refused. The message is safe to show the user."""


def _dir_key(path):
    """A cache key covering every .json file in a directory.

    It stats the FILES, not the directory. A directory's own mtime moves on
    create, delete and rename but NOT on a write into an existing file -- so a
    key built from it would go on quoting an old plan's certified clearance
    after the plan had been re-solved in place, which is the kind of stale
    number that gets believed.

    Size is in the key as well as mtime because mtime granularity is a
    property of the filesystem, not of Python: two writes inside one tick are
    indistinguishable by time alone, and a cache that cannot tell them apart
    fails silently and in the direction of showing the older value.
    """
    try:
        entries = os.scandir(path)
    except OSError:
        return None
    out = []
    with entries:
        for e in entries:
            if not e.name.endswith(".json"):
                continue
            try:
                st = e.stat()
            except OSError:
                continue
            out.append((e.name, st.st_mtime_ns, st.st_size))
    return tuple(sorted(out))


def _file_key(path):
    """Same reasoning as _dir_key: never mtime alone."""
    try:
        st = os.stat(path)
    except OSError:
        return None
    return (st.st_mtime_ns, st.st_size, st.st_ino)


class CurrentRun:
    """What sim.launch.py says it is running, read from web/current.json.

    This file is the handoff between the two launches.  It is written by the
    simulator's launch, not by this process, so it reports what actually came
    up rather than what was asked for -- a launch that refused its arguments
    never writes one.

    Read fresh every time, with no cache.  It holds about eighty bytes and the
    stat a cache would need costs nearly as much as the read; caching it buys
    nothing and would put the one fact this server cannot afford to be wrong
    about -- which scenario is on screen -- behind a timestamp comparison the
    filesystem is allowed to blur.  The cost this module does avoid is in the
    catalogue, which reads every plan.
    """

    def __init__(self, path, ws):
        self.path = path
        self.ws = ws

    def read(self):
        try:
            with open(self.path) as fh:
                doc = json.load(fh)
        except (OSError, ValueError):
            # Missing, or caught mid-write. Either way there is no run to
            # report, and reporting none is better than reporting half of one.
            return {}
        return doc if isinstance(doc, dict) else {}

    @property
    def scenario(self):
        """The scenario name, or "" for a run with no plan."""
        doc = self.read()
        return doc.get("world", "") if doc.get("plan") else ""

    @property
    def plan_file(self):
        doc = self.read()
        plan = doc.get("plan")
        return os.path.join(self.ws, "plans", plan) if plan else ""


class Catalogue:
    """The runs this server will launch, derived from what is on disk.

    Derived, not typed: adding a scenario to scenarios/ and solving it is
    enough to make it appear in the dropdown.  The alternative -- a list in
    this file -- is the failure mode scripts/kill_sim.sh documents at length,
    where the list and the thing it describes drift apart silently.

    A scenario earns an entry only if some plan in plans/ DECLARES it.  The
    declaration is read from the plan, never inferred from its filename, which
    is the same rule sim.launch.py enforces before it will fly one: a plan
    whose name says wall and whose contents say fence3d is a plan for fence3d.
    """

    def __init__(self, ws):
        self.ws = ws
        self.scenario_dir = os.path.join(ws, "scenarios")
        self.plan_dir = os.path.join(ws, "plans")
        self.config = os.path.join(ws, "config", "drone.yaml")
        self._key = object()
        self._runs = ()

    def runs(self):
        key = (_dir_key(self.scenario_dir), _dir_key(self.plan_dir),
               _file_key(self.config))
        if key != self._key:
            self._runs = tuple(REFERENCE_RUNS) + tuple(self._scenario_runs())
            self._key = key
        return self._runs

    def by_name(self, name):
        for run in self.runs():
            if run["name"] == name:
                return run
        return None

    # ---- how a scenario becomes an entry ---------------------------------

    def _flow_ceiling_m(self):
        """The height above which the velocity estimate has no aiding.

        Read from the sensor config rather than written here.  It decides
        whether a run is flown on the estimate or on ground truth, and the
        measured consequence of getting it wrong is not subtle: the loiter
        scenario flies at 62.5 m, and on state:=est it tracks its plan to 48 m
        of error against 2.2 cm on truth.  That is the sensors being honest
        about their envelope, not the controller failing.
        """
        try:
            import yaml
            with open(self.config) as fh:
                cfg = yaml.safe_load(fh)
            return float(cfg["drone"]["sensors"]["optical_flow"]["max_height_m"])
        except (OSError, ValueError, KeyError, TypeError, ImportError):
            # No ceiling known means no claim made: fly on the default.
            return float("inf")

    def _plans_by_scenario(self):
        out = {}
        try:
            names = sorted(os.listdir(self.plan_dir))
        except OSError:
            return out
        for fname in names:
            if not fname.endswith(".json"):
                continue
            path = os.path.join(self.plan_dir, fname)
            try:
                with open(path) as fh:
                    doc = json.load(fh)
            except (OSError, ValueError):
                continue
            scenario = doc.get("scenario")
            if isinstance(scenario, str) and scenario:
                out.setdefault(scenario, []).append((fname, doc))
        return out

    def _scenario_runs(self):
        ceiling = self._flow_ceiling_m()
        for scenario, plans in sorted(self._plans_by_scenario().items()):
            if not os.path.exists(os.path.join(self.scenario_dir, f"{scenario}.json")):
                # A plan for a scenario this workspace no longer has. Offering
                # it would produce a launch that refuses its own arguments.
                continue
            fname, doc = self._pick(plans)
            run = {"name": scenario, "title": scenario, "world": scenario,
                   "plan": os.path.join("plans", fname)}
            top = _max_altitude(doc)
            if top is not None and top > ceiling:
                run["state"] = "truth"
                run["state_reason"] = (
                    f"flies to {top:.0f} m; optical flow aids velocity only "
                    f"below {ceiling:.0f} m, so this one is flown on truth")
            demands = doc.get("demands") or {}
            if demands.get("flyable") is False:
                run["warning"] = (
                    f"this plan demands {demands.get('max_tilt_deg', float('nan')):.0f} deg "
                    f"of tilt and the clamp is "
                    f"{demands.get('tilt_limit_deg', float('nan')):.0f}")
            yield run

    @staticmethod
    def _pick(plans):
        """Which plan to fly for a scenario, when several solve it.

        A solved plan over the straight seed.  The seed exists to be flown into
        an obstacle -- it is the fixture that proves the referee reports a hit
        -- and a dropdown whose entries collide by default would be reporting
        the fixture rather than the planner.  Ties break on name so the choice
        is the same on every machine.
        """
        solved = sorted(f for f, _ in plans if not f.endswith("_seed.json"))
        if solved:
            return next((f, d) for f, d in plans if f == solved[0])
        return min(plans, key=lambda fd: fd[0])


def _max_altitude(doc):
    """Highest z in a plan's control points, or None if it has none.

    The control polygon BOUNDS the curve for each coordinate independently
    (convex hull property), so this is an upper bound on the flown altitude and
    never under-reports the one thing it is used to decide.
    """
    top = None
    for cp in doc.get("control_points") or ():
        if isinstance(cp, (list, tuple)) and len(cp) >= 3:
            try:
                z = float(cp[2])
            except (TypeError, ValueError):
                continue
            top = z if top is None else max(top, z)
    return top


class Launcher:
    """Cycles sim.launch.py underneath a viewer that stays up.

    `status_at` is a callable returning the monotonic time the last piece of
    simulator telemetry arrived, or None.  A launch is not called "running"
    because it was spawned, or because a process still exists, but because
    telemetry arrived AFTER it was asked for -- a message from the run that was
    just torn down cannot satisfy it.
    """

    def __init__(self, ws, current, enabled, status_at=None, clock=time.monotonic):
        self.ws = ws
        self.current = current
        self.enabled = bool(enabled)
        self.catalogue = Catalogue(ws)
        self._status_at = status_at or (lambda: None)
        self._clock = clock
        self._proc = None
        self._requested = None
        self._at = 0.0
        # A finished launch leaves a verdict here, so "failed" survives long
        # enough for a page polling at 30 Hz to show it instead of a blank.
        self._outcome = None

    # ---- what the page draws ---------------------------------------------

    def sim_live(self):
        """Is the simulator publishing right now?

        Two seconds of slack: /drone/eval/status is published continuously
        while a run exists, so a gap this long means the run is gone, not slow.
        """
        at = self._status_at()
        return at is not None and (self._clock() - at) < 2.0

    def status(self):
        """One of idle | starting | running | failed, and why.

        This is the part that has to be able to say "no", so it is written to
        fail rather than to reassure: a spawned process that exits, or one that
        never produces telemetry inside START_TIMEOUT_S, both end as `failed`
        with the tail of the launch log attached.

        A launch stops being JUDGED the moment its telemetry arrives.  It has
        to: /drone/eval/status runs on sim time, so pausing the simulator stops
        it, and a rule that watched forever would report a paused run as a dead
        one -- an alarm raised by the user pressing pause.
        """
        if self._proc is not None:
            started = self._status_at()
            if started is not None and started > self._at:
                self._outcome, self._proc = None, None       # it came up
            else:
                code = self._proc.poll()
                if code is not None:
                    self._outcome = ("failed",
                                     f"the launch exited {code}" + self._tail())
                    self._proc = None
                elif (self._clock() - self._at) > START_TIMEOUT_S:
                    self._outcome = ("failed",
                                     f"no telemetry {START_TIMEOUT_S:.0f} s "
                                     f"after launch" + self._tail())
                    self._proc = None
                else:
                    return "starting", ""
        if self.sim_live():
            return "running", ""
        return self._outcome or ("idle", "")

    def current_run_name(self):
        """Which catalogue entry the running simulator corresponds to.

        A plan run is named after its scenario, which is also its world. A
        reference run is named after the REFERENCE, because hover and circle
        share the empty world -- so the world alone cannot tell those two
        apart, and a dropdown set from it would sit on neither of them while
        one of them was flying.

        None when the running sim is not in the catalogue at all, which is a
        real case: it can be started from the command line with any
        combination of arguments this list does not offer.
        """
        doc = self.current.read()
        name = doc.get("world") if doc.get("plan") else doc.get("reference")
        return name if name and self.catalogue.by_name(name) else None

    @property
    def state(self):
        status, detail = self.status()
        return {
            "enabled": self.enabled,
            "runs": [dict(r) for r in self.catalogue.runs()],
            "current": self.current.read(),
            "current_run": self.current_run_name(),
            "sim_live": self.sim_live(),
            "status": status,
            "detail": detail,
            "requested": self._requested,
            "elapsed_s": (round(self._clock() - self._at, 1)
                          if status == "starting" else None),
        }

    def _tail(self, lines=12):
        try:
            with open(os.path.join(self.ws, "logs", "sim.log"), errors="replace") as fh:
                raw = fh.read().strip().splitlines()[-lines:]
        except OSError:
            return ""
        text = "\n".join(_ANSI.sub("", line).rstrip() for line in raw).strip()
        return f":\n{text}" if text else ""

    # ---- the write --------------------------------------------------------

    def argv(self, run):
        """The launch command for a run. Pure, so a test can read it."""
        argv = ["ros2", "launch", "dsim_bringup", "sim.launch.py",
                f"world:={run['world']}", "gui:=false", "viz:=true"]
        if run.get("plan"):
            argv.append(f"plan:={os.path.join(self.ws, run['plan'])}")
        else:
            argv += [f"reference:={run['reference']}",
                     f"radius:={run['radius']:g}",
                     f"period:={run['period']:g}",
                     f"altitude:={run['altitude']:g}"]
        if run.get("state"):
            argv.append(f"state:={run['state']}")
        return argv

    def launch(self, body):
        if not self.enabled:
            raise LaunchError("switching runs is disabled on this server")
        name = body.get("run")
        if not isinstance(name, str) or not name:
            raise LaunchError("missing run")
        run = self.catalogue.by_name(name)
        if run is None:
            offered = ", ".join(r["name"] for r in self.catalogue.runs())
            raise LaunchError(f"unknown run {name!r}; this server offers: {offered}")
        if self.status()[0] == "starting":
            raise LaunchError(f"{self._requested} is still starting")

        # Teardown first, and synchronously: two Gazebo servers pacing one
        # world is not a hypothetical -- see the third trap in kill_sim.sh,
        # where duplicates made the world run at two and three times the
        # requested speed and every measurement taken to explain it was wrong.
        try:
            killed = subprocess.run(
                ["bash", os.path.join(self.ws, "scripts", "kill_sim.sh")],
                cwd=self.ws, capture_output=True, text=True, timeout=KILL_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            raise LaunchError(f"teardown did not finish in {KILL_TIMEOUT_S:.0f} s")
        except OSError as exc:
            raise LaunchError(f"could not run the teardown: {exc}")
        if killed.returncode != 0:
            tail = "\n".join(((killed.stdout or "") + (killed.stderr or ""))
                             .strip().splitlines()[-4:])
            raise LaunchError(f"teardown failed, refusing to start a second "
                              f"simulator:\n{tail}")

        argv = self.argv(run)
        log_path = os.path.join(self.ws, "logs", "sim.log")
        try:
            os.makedirs(os.path.dirname(log_path), exist_ok=True)
            # The parent's handle is closed on the way out of the `with`; the
            # child keeps its own. Leaving it open here would leak one file
            # descriptor per switch in a process that never restarts.
            with open(log_path, "w") as log:
                proc = subprocess.Popen(argv, cwd=self.ws, stdout=log,
                                        stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        except OSError as exc:
            raise LaunchError(f"could not start the launch: {exc}")

        self._proc = proc
        self._requested = name
        self._at = self._clock()
        self._outcome = None
        return dict(self.state, message=f"starting {name}")
