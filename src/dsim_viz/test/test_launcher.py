"""The /launch endpoint: what it offers, what it refuses, and when it says a
run came up.

Nothing here starts a simulator. These are about the two claims the endpoint
makes -- "this is what I will launch" and "this is what happened" -- and each
test says what it would catch.
"""
import json
import os
import subprocess

import pytest

from dsim_viz.launcher import (REFERENCE_RUNS, START_TIMEOUT_S, Catalogue,
                               CurrentRun, LaunchError, Launcher)


# ---- a workspace on disk, because the catalogue is derived from one --------

def write(path, doc):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc))


def plan(scenario, top=1.5, flyable=True):
    return {"scenario": scenario, "demands": {"flyable": flyable,
                                              "max_tilt_deg": 108.6,
                                              "tilt_limit_deg": 40.0},
            "control_points": [[0, 0, 1.0, 0], [1, 0, top, 1], [2, 0, 1.0, 2]]}


@pytest.fixture
def ws(tmp_path):
    (tmp_path / "scenarios").mkdir()
    (tmp_path / "plans").mkdir()
    (tmp_path / "web").mkdir()
    (tmp_path / "logs").mkdir()
    for name in ("wall", "fence3d", "loiter"):
        write(tmp_path / "scenarios" / f"{name}.json", {"name": name})
    write(tmp_path / "plans" / "wall_seed.json", plan("wall"))
    write(tmp_path / "plans" / "wall_N10_seg16.json", plan("wall"))
    write(tmp_path / "plans" / "fence3d_seed.json", plan("fence3d"))
    write(tmp_path / "plans" / "loiter_N8_seg16.json", plan("loiter", top=62.5))
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "drone.yaml").write_text(
        "drone:\n  sensors:\n    optical_flow:\n      max_height_m: 3.0\n")
    return tmp_path


@pytest.fixture
def current(ws):
    return CurrentRun(str(ws / "web" / "current.json"), str(ws))


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class FakeProc:
    """A spawned launch. `code` None means still running."""

    def __init__(self, code=None):
        self.code = code

    def poll(self):
        return self.code


@pytest.fixture
def spawned(monkeypatch):
    """Capture the teardown and the launch instead of running either."""
    seen = {"kill": None, "argv": None, "proc": FakeProc()}

    def fake_run(argv, **kw):
        seen["kill"] = list(argv)
        return subprocess.CompletedProcess(argv, seen.get("kill_rc", 0), "cleared 3", "")

    def fake_popen(argv, **kw):
        seen["argv"] = list(argv)
        seen["kw"] = kw
        return seen["proc"]

    monkeypatch.setattr("subprocess.run", fake_run)
    monkeypatch.setattr("subprocess.Popen", fake_popen)
    return seen


def mk(ws, current, clock, status_at=lambda: None, enabled=True):
    return Launcher(str(ws), current, enabled=enabled,
                    status_at=status_at, clock=clock)


# ---- what the catalogue contains, and why ---------------------------------

def test_every_scenario_with_a_plan_is_offered(ws):
    # FAILS IF: the catalogue is a hand-written list. Adding a scenario and
    # solving it must be enough to make it appear, or the list and the disk
    # drift apart the way scripts/kill_sim.sh documents at length.
    names = [r["name"] for r in Catalogue(str(ws)).runs()]
    assert "wall" in names and "fence3d" in names and "loiter" in names


def test_hover_and_circle_are_always_offered(ws):
    # FAILS IF: the built-in trajectories are dropped when no scenario solves.
    # They need no plan and no obstacles, so an empty plans/ must not remove
    # them -- they are how you check the vehicle flies at all.
    for f in (ws / "plans").iterdir():
        f.unlink()
    names = [r["name"] for r in Catalogue(str(ws)).runs()]
    assert names == [r["name"] for r in REFERENCE_RUNS]


def test_a_plan_for_a_missing_scenario_is_not_offered(ws):
    # FAILS IF: the catalogue offers a run the launch would refuse. plan.json
    # names a scenario that has no scenarios/*.json, and sim.launch.py raises
    # on exactly that -- so offering it produces a launch that kills the
    # running sim and then dies.
    write(ws / "plans" / "ghost_N2_seg1.json", plan("ghost"))
    assert Catalogue(str(ws)).by_name("ghost") is None


def test_a_scenario_with_no_plan_is_not_offered(ws):
    # FAILS IF: the catalogue is built from scenarios/ rather than from plans
    # that declare them. There would be nothing to fly.
    write(ws / "scenarios" / "unsolved.json", {"name": "unsolved"})
    assert Catalogue(str(ws)).by_name("unsolved") is None


def test_the_scenario_is_read_from_the_plan_not_its_filename(ws):
    # FAILS IF: the catalogue infers the scenario from the file name. A plan
    # called wall_*.json whose contents say fence3d is a plan for fence3d, and
    # sim.launch.py enforces exactly that before it will fly one.
    write(ws / "plans" / "wall_lies.json", plan("fence3d"))
    run = Catalogue(str(ws)).by_name("fence3d")
    assert run["plan"].endswith("wall_lies.json")       # sorts before _seed


def test_a_solved_plan_beats_the_seed(ws):
    # FAILS IF: the dropdown defaults to the straight seed. The seed exists to
    # be flown INTO an obstacle -- it is the fixture that proves the referee
    # reports a hit -- so defaulting to it would report the fixture.
    assert Catalogue(str(ws)).by_name("wall")["plan"].endswith("wall_N10_seg16.json")


def test_the_seed_is_used_when_it_is_all_there_is(ws):
    # FAILS IF: a scenario with only a seed silently vanishes from the list.
    assert Catalogue(str(ws)).by_name("fence3d")["plan"].endswith("fence3d_seed.json")


def test_a_run_above_the_flow_ceiling_is_flown_on_truth(ws):
    # FAILS IF: loiter is offered on the estimate. Measured: it flies at 62.5 m,
    # the optical flow aids velocity only below 3 m, and the tracking error is
    # 48 m on est against 2.2 cm on truth. The ceiling is READ from the sensor
    # config, so changing the sensor changes this and nothing has to be edited.
    run = Catalogue(str(ws)).by_name("loiter")
    assert run["state"] == "truth"
    assert "3 m" in run["state_reason"] and "62" in run["state_reason"]


def test_a_run_inside_the_envelope_says_nothing_about_state(ws):
    # FAILS IF: every run is forced onto truth, which would quietly stop the
    # estimator from ever being exercised.
    assert "state" not in Catalogue(str(ws)).by_name("wall")


def test_an_unflyable_plan_is_offered_with_a_warning(ws):
    # FAILS IF: a plan the vehicle cannot fly is either hidden (you could not
    # reproduce it) or offered silently (you would discover it as mysterious
    # tracking error).
    target = ws / "plans" / "wall_N10_seg16.json"
    stamp = os.stat(target).st_mtime_ns
    cat = Catalogue(str(ws))
    assert cat.by_name("wall").get("warning") is None
    write(target, plan("wall", flyable=False))
    # Pinned, so this also proves the key is not the modification time alone:
    # a re-solve inside one filesystem tick must still be noticed.
    os.utime(target, ns=(stamp, stamp))
    warning = cat.by_name("wall")["warning"]
    assert "109 deg" in warning and "clamp is 40" in warning


def test_the_catalogue_notices_a_new_plan(ws):
    # FAILS IF: the mtime cache never invalidates. scripts/solve_plan.py writes
    # through a temporary file and renames it, which moves the directory mtime.
    cat = Catalogue(str(ws))
    assert cat.by_name("door3d") is None
    write(ws / "scenarios" / "door3d.json", {"name": "door3d"})
    write(ws / "plans" / "door3d_N8_seg4.json", plan("door3d"))
    assert cat.by_name("door3d") is not None


# ---- what a request may say ------------------------------------------------

def test_refuses_an_unknown_run(ws, current, spawned):
    lc = mk(ws, current, Clock())
    with pytest.raises(LaunchError, match="unknown run"):
        lc.launch({"run": "does_not_exist"})
    # FAILS IF: it tore the simulator down before validating. A refused request
    # must leave the running sim alone.
    assert spawned["kill"] is None and spawned["argv"] is None


def test_refuses_a_missing_or_non_string_run(ws, current, spawned):
    lc = mk(ws, current, Clock())
    for body in ({}, {"run": 5}, {"run": ""}, {"run": None}, {"run": ["wall"]}):
        with pytest.raises(LaunchError):
            lc.launch(body)
    assert spawned["argv"] is None


def test_refuses_when_disabled(ws, current, spawned):
    # FAILS IF: control:=false still lets the whole tailnet restart the sim.
    lc = mk(ws, current, Clock(), enabled=False)
    with pytest.raises(LaunchError, match="disabled"):
        lc.launch({"run": "wall"})
    assert spawned["argv"] is None


def test_no_field_of_the_request_reaches_the_command(ws, current, spawned):
    # FAILS IF: a name is interpolated into the launch. The request selects a
    # catalogue ENTRY; every string in the command comes from that entry.
    lc = mk(ws, current, Clock())
    lc.launch({"run": "wall"})
    assert spawned["argv"][:4] == ["ros2", "launch", "dsim_bringup", "sim.launch.py"]
    assert "shell" not in spawned["kw"] or spawned["kw"]["shell"] is False


def test_a_plan_run_launches_its_world_and_its_plan(ws, current, spawned):
    lc = mk(ws, current, Clock())
    lc.launch({"run": "wall"})
    argv = spawned["argv"]
    assert "world:=wall" in argv
    assert f"plan:={ws}/plans/wall_N10_seg16.json" in argv
    # FAILS IF: a plan run also starts a built-in reference. Two publishers on
    # /drone/trajectory would take turns owning the vehicle.
    assert not any(a.startswith("reference:=") for a in argv)


def test_a_reference_run_launches_the_empty_world_with_no_plan(ws, current, spawned):
    lc = mk(ws, current, Clock())
    lc.launch({"run": "circle"})
    argv = spawned["argv"]
    assert "world:=empty" in argv and "reference:=circle" in argv
    assert "period:=3.5" in argv and "radius:=2" in argv
    assert not any(a.startswith("plan:=") for a in argv)


def test_a_run_above_the_ceiling_launches_on_truth(ws, current, spawned):
    # FAILS IF: the catalogue says truth and the command says otherwise.
    lc = mk(ws, current, Clock())
    lc.launch({"run": "loiter"})
    assert "state:=truth" in spawned["argv"]


def test_teardown_runs_before_the_launch(ws, current, spawned):
    # FAILS IF: a second simulator is started on top of the first. Duplicates
    # pacing one world made it run at two and three times the requested speed,
    # and every measurement taken to explain that was wrong.
    lc = mk(ws, current, Clock())
    lc.launch({"run": "wall"})
    assert spawned["kill"] == ["bash", str(ws / "scripts" / "kill_sim.sh")]


def test_a_failed_teardown_refuses_to_start_anything(ws, current, spawned, monkeypatch):
    # FAILS IF: a kill that could not clear the old sim is ignored and a new
    # one is started anyway -- the exact duplicate-simulator state above.
    def fake_run(argv, **kw):
        return subprocess.CompletedProcess(argv, 1, "still alive after SIGKILL", "")
    monkeypatch.setattr("subprocess.run", fake_run)
    lc = mk(ws, current, Clock())
    with pytest.raises(LaunchError, match="teardown failed"):
        lc.launch({"run": "wall"})
    assert spawned["argv"] is None


def test_refuses_while_a_run_is_still_starting(ws, current, spawned):
    # FAILS IF: an impatient second click tears down the sim that is coming up.
    clock = Clock()
    lc = mk(ws, current, clock)
    lc.launch({"run": "wall"})
    with pytest.raises(LaunchError, match="still starting"):
        lc.launch({"run": "circle"})


# ---- when it says a run came up -------------------------------------------

def test_starting_until_telemetry_arrives(ws, current, spawned):
    clock = Clock()
    at = [None]
    lc = mk(ws, current, clock, status_at=lambda: at[0])
    lc.launch({"run": "wall"})
    assert lc.status()[0] == "starting"
    clock.t += 5
    at[0] = clock.t
    assert lc.status()[0] == "running"


def test_telemetry_from_before_the_launch_does_not_count(ws, current, spawned):
    # FAILS IF: "running" means any message ever arrived. The previous run was
    # publishing right up to the teardown, so a rule that looked only at
    # freshness would call every launch successful the instant it was asked
    # for -- including one that never started.
    clock = Clock()
    at = [clock.t - 0.1]                      # the OLD run, still fresh
    lc = mk(ws, current, clock, status_at=lambda: at[0])
    lc.launch({"run": "wall"})
    assert lc.status()[0] == "starting"


def test_a_launch_that_exits_is_failed_with_its_log(ws, current, spawned):
    # FAILS IF: a launch that refused its arguments leaves the page saying
    # "starting" forever, with the reason only in a file on the host.
    clock = Clock()
    lc = mk(ws, current, clock)
    lc.launch({"run": "wall"})
    # After the launch: launching TRUNCATES the log, and what matters is what
    # the child wrote into it afterwards.
    (ws / "logs" / "sim.log").write_text("RuntimeError: world file not found\n")
    spawned["proc"].code = 1
    status, detail = lc.status()
    assert status == "failed"
    assert "exited 1" in detail and "world file not found" in detail


def test_the_log_tail_is_readable_in_a_browser(ws, current, spawned):
    # FAILS IF: the colour codes ros2 launch and Gazebo write reach the page.
    # They are invisible in a terminal and literal "[1;32m" noise in a <pre>,
    # in front of every word of the one message you are trying to read.
    clock = Clock()
    lc = mk(ws, current, clock)
    lc.launch({"run": "wall"})
    (ws / "logs" / "sim.log").write_text(
        "\x1b[1;31m[ERROR]\x1b[0m world file not found\n")
    spawned["proc"].code = 1
    detail = lc.status()[1]
    assert "\x1b" not in detail and "[1;31m" not in detail
    assert "[ERROR] world file not found" in detail


def test_a_launch_that_never_publishes_is_failed(ws, current, spawned):
    # FAILS IF: a launch whose processes stay alive but produce nothing is
    # reported as starting for ever. It is the quietest failure there is.
    clock = Clock()
    lc = mk(ws, current, clock)
    lc.launch({"run": "wall"})
    clock.t += START_TIMEOUT_S + 1
    assert lc.status()[0] == "failed"


def test_a_confirmed_run_stops_being_judged(ws, current, spawned):
    # FAILS IF: pausing the simulator is reported as a dead launch.
    # /drone/eval/status runs on SIM time, so pause stops it -- a rule that
    # kept watching would raise an alarm the user caused by pressing pause.
    clock = Clock()
    at = [None]
    lc = mk(ws, current, clock, status_at=lambda: at[0])
    lc.launch({"run": "wall"})
    clock.t += 5
    at[0] = clock.t
    assert lc.status()[0] == "running"
    clock.t += 300                              # paused for five minutes
    assert lc.status()[0] == "idle"
    assert lc.status()[0] != "failed"


def test_idle_when_nothing_was_ever_launched(ws, current):
    # FAILS IF: a viewer started on its own claims a simulator is running.
    assert mk(ws, current, Clock()).status() == ("idle", "")


def test_running_without_this_launcher_having_started_it(ws, current):
    # FAILS IF: a sim started from the command line shows as "no simulator" on
    # a page that is visibly receiving its telemetry.
    clock = Clock()
    lc = mk(ws, current, clock, status_at=lambda: clock.t - 0.1)
    assert lc.status() == ("running", "")


# ---- what the page is told -------------------------------------------------

def test_state_offers_the_same_names_it_will_accept(ws, current, spawned):
    # FAILS IF: the dropdown is built from a different list than the one the
    # endpoint validates against, so the page can offer a run that is refused.
    lc = mk(ws, current, Clock())
    for run in lc.state["runs"]:
        lc.launch({"run": run["name"]})
        lc._proc = None                       # accept the next one


def test_state_reports_the_running_world_not_the_requested_one(ws, current, spawned):
    # FAILS IF: the page labels itself from the request. current.json is
    # written by the simulator's launch AFTER it has accepted its arguments,
    # so a launch that refused them leaves the previous run on screen -- which
    # is true, where "now flying wall" would have been a lie.
    write(ws / "web" / "current.json", {"world": "fence3d", "plan": "fence3d_seed.json"})
    lc = mk(ws, current, Clock())
    lc.launch({"run": "wall"})
    assert lc.state["current"]["world"] == "fence3d"
    assert lc.state["requested"] == "wall"


def test_a_reference_run_is_named_after_its_reference(ws, current):
    # FAILS IF: the dropdown is set from the world. hover and circle BOTH run
    # in the empty world, so the world cannot tell them apart and the selector
    # would sit on neither while one of them was flying.
    write(ws / "web" / "current.json",
          {"world": "empty", "reference": "circle", "plan": None})
    assert mk(ws, current, Clock()).current_run_name() == "circle"


def test_a_plan_run_is_named_after_its_scenario(ws, current):
    write(ws / "web" / "current.json",
          {"world": "wall", "reference": "none", "plan": "wall_N10_seg16.json"})
    assert mk(ws, current, Clock()).current_run_name() == "wall"


def test_a_run_outside_the_catalogue_names_nothing(ws, current):
    # FAILS IF: a sim started from the command line with arguments this list
    # does not offer makes the dropdown claim one of its entries is flying.
    write(ws / "web" / "current.json",
          {"world": "empty", "reference": "lemniscate", "plan": None})
    assert mk(ws, current, Clock()).current_run_name() is None


def test_nothing_running_names_nothing(ws, current):
    assert mk(ws, current, Clock()).current_run_name() is None


# ---- current.json, the handoff between the two launches --------------------

def test_current_run_is_empty_when_nothing_has_run(ws, current):
    assert current.read() == {}
    assert current.scenario == "" and current.plan_file == ""


def test_current_run_survives_a_truncated_file(ws, current):
    # FAILS IF: a half-written current.json takes the viewer down. It is
    # written by another process while this one is serving.
    (ws / "web" / "current.json").write_text('{"world": "wa')
    assert current.read() == {}


def test_a_run_with_no_plan_has_no_scenario_to_solve(ws, current):
    # FAILS IF: the solve panel appears for hover or circle, where there is no
    # plan to re-solve and nowhere to write one.
    write(ws / "web" / "current.json",
          {"world": "empty", "reference": "circle", "plan": None})
    assert current.scenario == "" and current.plan_file == ""


def test_current_run_follows_the_file(ws, current):
    # FAILS IF: the mtime cache never invalidates and the viewer reports the
    # first run it ever saw for the life of the process.
    target = ws / "web" / "current.json"
    write(target, {"world": "wall", "plan": "wall_seed.json"})
    assert current.scenario == "wall"
    stamp = os.stat(target).st_mtime_ns
    write(target, {"world": "fence3d", "plan": "fence3d_seed.json"})
    os.utime(target, ns=(stamp, stamp))      # the timestamp is no help here
    assert current.scenario == "fence3d"
    assert current.plan_file == str(ws / "plans" / "fence3d_seed.json")
