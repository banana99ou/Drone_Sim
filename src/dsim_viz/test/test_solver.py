"""The solve endpoint's validation and its view of the plan on disk.

Nothing here runs the optimiser: these are about what the endpoint will and
will not pass through to it. Each test says what it would catch.
"""
import json

import pytest

from dsim_viz.solver import N_RANGE, NSEG_RANGE, VMAX_RANGE, SolveError, Solver


@pytest.fixture
def spy(monkeypatch):
    """Capture the argv the solver would run, and stop before running it."""
    seen = {}

    def fake_run(argv, **kw):
        seen["argv"] = list(argv)
        seen["kw"] = kw
        raise SolveError("stopped before running the optimiser")

    monkeypatch.setattr("subprocess.run", fake_run)
    return seen


@pytest.fixture
def plan(tmp_path):
    p = tmp_path / "plan.json"
    p.write_text(json.dumps({
        "scenario": "fence3d", "solver": "rust", "config": "N8_seg2",
        "certified_clearance_m": 0.2209, "predicted_clearance_m": 0.2209,
        "demands": {"max_speed_mps": 3.0, "max_accel_mps2": 1.85,
                    "max_tilt_deg": 10.3, "tilt_limit_deg": 40.0,
                    "flyable": True, "start_speed_mps": 3.0},
        "control_points": [[0, 0, 0, 0], [1, 0, 0, 1]],
    }))
    return p


def sv(plan, enabled=True, scenario="fence3d"):
    return Solver("/ws", scenario, str(plan), enabled)


# ---- when the endpoint exists at all --------------------------------------

def test_disabled_without_a_plan(tmp_path):
    # FAILS IF: a run with no plan offers a solve button. There is nowhere to
    # write and nothing to reload; the page must not show the panel.
    assert Solver("/ws", "fence3d", "", True).enabled is False
    assert Solver("/ws", "", "/ws/plans/x.json", True).enabled is False
    assert Solver("/ws", "fence3d", "/ws/plans/x.json", False).enabled is False


def test_refuses_when_disabled(plan):
    with pytest.raises(SolveError, match="no plan"):
        sv(plan, enabled=False).solve({"N": 8, "n_seg": 2, "v_max": 3.0})


# ---- what a request may say -------------------------------------------------

@pytest.mark.parametrize("body,msg", [
    ({"n_seg": 2, "v_max": 1.0}, "missing N"),
    ({"N": 8, "v_max": 1.0}, "missing n_seg"),
    ({"N": 8, "n_seg": 2}, "missing v_max"),
    ({"N": N_RANGE[0] - 1, "n_seg": 2, "v_max": 1.0}, "N must be between"),
    ({"N": N_RANGE[1] + 1, "n_seg": 2, "v_max": 1.0}, "N must be between"),
    ({"N": 8, "n_seg": NSEG_RANGE[1] + 1, "v_max": 1.0}, "n_seg must be between"),
    ({"N": 8, "n_seg": NSEG_RANGE[0] - 1, "v_max": 1.0}, "n_seg must be between"),
    ({"N": 8, "n_seg": 2, "v_max": VMAX_RANGE[1] + 1}, "v_max must be between"),
    ({"N": 8, "n_seg": 2, "v_max": -1.0}, "v_max must be between"),
    ({"N": "eight", "n_seg": 2, "v_max": 1.0}, "N must be a int"),
    ({"N": None, "n_seg": 2, "v_max": 1.0}, "N must be a int"),
])
def test_rejects(plan, body, msg):
    # FAILS IF: a field stops being range-checked. Every one of these would
    # otherwise reach a subprocess argument list.
    with pytest.raises(SolveError, match=msg):
        sv(plan).solve(body)


def test_the_scenario_is_not_a_request_field(plan, spy):
    # FAILS IF: a request can choose which scenario to solve. The Gazebo world
    # is generated per scenario, so solving another one would produce a plan
    # for obstacles that are not in the running world -- and a clearance number
    # about a fence nobody can see.
    with pytest.raises(SolveError):
        sv(plan).solve({"N": 8, "n_seg": 2, "v_max": 3.0, "scenario": "door3d"})
    assert "door3d" not in spy["argv"]
    assert spy["argv"][spy["argv"].index("--scenario") + 1] == "fence3d"


def test_the_output_path_is_not_a_request_field(plan, spy):
    # FAILS IF: a request can choose where the plan is written. That is a
    # filesystem write from a port reachable across the whole tailnet.
    with pytest.raises(SolveError):
        sv(plan).solve({"N": 8, "n_seg": 2, "v_max": 1.0,
                        "out": "/etc/passwd", "--out": "/etc/passwd"})
    argv = spy["argv"]
    assert argv.count("--out") == 1
    assert argv[argv.index("--out") + 1] == str(plan)
    assert "/etc/passwd" not in argv


def test_no_cap_is_expressible_and_omits_the_flag(plan, spy):
    # FAILS IF: v_max=0 sends --v-max 0, which the optimiser reads as a cap of
    # zero and refuses. Zero means "no cap" here, and that is the setting that
    # produces the unflyable plans worth seeing.
    with pytest.raises(SolveError):
        sv(plan).solve({"N": 8, "n_seg": 2, "v_max": 0.0})
    assert "--v-max" not in spy["argv"]


def test_runs_a_list_never_a_shell(plan, spy):
    # FAILS IF: the solver is ever invoked through a shell. Every number is
    # range-checked, but an argument LIST is what makes that sufficient.
    with pytest.raises(SolveError):
        sv(plan).solve({"N": 8, "n_seg": 2, "v_max": 3.0})
    assert isinstance(spy["argv"], list)
    assert spy["kw"].get("shell") is not True
    assert spy["argv"][1].endswith("solve_plan.py")


# ---- what the page is told --------------------------------------------------

def test_summarises_the_plan_on_disk(plan):
    s = sv(plan).state
    assert s["enabled"] is True
    assert s["scenario"] == "fence3d"
    assert s["plan_file"] == "plan.json"
    assert s["plan"]["config"] == "N8_seg2"
    assert s["plan"]["flyable"] is True
    assert s["plan"]["predicted_clearance_m"] == pytest.approx(0.2209)


def test_a_missing_or_broken_plan_summarises_as_none(tmp_path):
    # FAILS IF: an unreadable plan crashes the snapshot. The stream is shared
    # by every browser watching; one bad file must not stop all of them.
    assert sv(tmp_path / "gone.json").plan_summary() is None
    bad = tmp_path / "bad.json"
    bad.write_text("{not json")
    assert sv(bad).plan_summary() is None
