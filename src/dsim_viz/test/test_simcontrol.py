"""Invariants for the one write path in the viewer stack.

This is the only code that can change the running simulation, so a mistake here
is not a cosmetic bug. Each test says what it would catch.

A NOTE ON WHAT THE PREVIOUS VERSION OF THIS FILE PROVED, WHICH WAS NOTHING.

It had a test called `test_speed_request_preserves_the_physics_step`, which
asserted that the string handed to `subprocess` contained "max_step_size:
0.001". It passed for the entire life of the bug it was supposed to be
guarding. It could not have failed for any behaviour of Gazebo whatsoever --
only for someone editing the text of a command -- and the command it was
carefully checking the punctuation of was the one that deleted the world's
gravity and threw the vehicle four kilometres into the air.

The lesson is in what is tested here now. These tests cover the parsing, which
is real logic with real failure modes, and they say so. What they cannot cover
-- that the command has the intended effect on a running simulator -- is not
faked with a mock that agrees with its author. It is measured against a live
sim by scripts/check_simcontrol.py, which flies the vehicle, moves the speed,
resets, and asserts the thing that actually matters: that it still has weight.
"""
import pytest

from dsim_viz import simcontrol

# The range the control node publishes. Every test that involves a speed uses
# this rather than a constant from the module, because the module deliberately
# has no opinion about the range: it validates against what the simulator said.
LIMITS = {"min_speed": 0.05, "max_speed": 4.0}
NO_LIMITS = dict(simcontrol.UNKNOWN)


# FAILS IF: a value from the network reaches the simulator unchecked. A string,
# a null or a NaN is much harder to notice once it is inside the simulator than
# as a refused request.
@pytest.mark.parametrize("bad", ["fast", None, [], {}])
def test_non_numeric_speed_is_refused(bad):
    with pytest.raises(simcontrol.ControlError):
        simcontrol.parse_command({"speed": bad}, LIMITS)


# FAILS IF: NaN is refused for the wrong reason. It IS caught by the range test
# (every comparison with NaN is false), so the explicit branch only changes the
# message -- and a user told "must be between 0.05 and 4" about a NaN learns
# nothing. This pins which path ran.
def test_nan_speed_is_refused_as_not_a_real_number():
    with pytest.raises(simcontrol.ControlError, match="real number"):
        simcontrol.parse_command({"speed": float("nan")}, LIMITS)


# FAILS IF: the range check is missing or one-sided. Zero would stop time while
# reporting itself as running; a huge factor makes the simulator silently fail
# to keep up, so the number becomes a lie rather than a speed.
@pytest.mark.parametrize("bad", [0, -1, 0.049, 4.01, 1000])
def test_out_of_range_speed_is_refused(bad):
    with pytest.raises(simcontrol.ControlError):
        simcontrol.parse_command({"speed": bad}, LIMITS)


# FAILS IF: the bounds themselves are rejected, which would make the slider's
# own end stops unusable.
@pytest.mark.parametrize("good", [0.05, 0.5, 1, "2.0", 4.0])
def test_valid_speeds_are_accepted(good):
    cmd = simcontrol.parse_command({"speed": good}, LIMITS)
    assert cmd.verb == "speed"
    assert LIMITS["min_speed"] <= cmd.speed <= LIMITS["max_speed"]


# FAILS IF: the limits are hard-coded here instead of coming from the
# simulator. This is the whole point of passing them in: if this file owned the
# range, the page, this module and the control node would be three copies, and
# raising the ceiling in one of them would produce a slider that sends requests
# the node refuses.
def test_the_accepted_range_comes_from_the_simulator():
    wider = {"min_speed": 0.01, "max_speed": 10.0}
    assert simcontrol.parse_command({"speed": 8.0}, wider).speed == 8.0
    with pytest.raises(simcontrol.ControlError):
        simcontrol.parse_command({"speed": 8.0}, LIMITS)


# FAILS IF: a speed is accepted before the control node has said what it will
# accept. Guessing a range here means the first request after startup can be
# one the node rejects, for reasons the page cannot explain.
def test_speed_is_refused_until_the_range_is_known():
    with pytest.raises(simcontrol.ControlError, match="has not reported"):
        simcontrol.parse_command({"speed": 1.0}, NO_LIMITS)


# FAILS IF: the old key is quietly translated. `rtf` named a Gazebo real-time
# factor, and the mechanism behind it deletes the world's gravity. Anyone still
# sending that key is following instructions that are dangerous in their other
# half too, and should be told so rather than silently accommodated.
def test_the_old_rtf_key_is_refused_with_an_explanation():
    with pytest.raises(simcontrol.ControlError, match="speed"):
        simcontrol.parse_command({"rtf": 0.5}, LIMITS)


# FAILS IF: an empty or unrecognised body is treated as a command. A POST that
# means nothing must not be reported as applied.
@pytest.mark.parametrize("body", [{}, {"nonsense": 1}, {"gravity": 0}])
def test_a_body_with_no_verb_is_refused(body):
    with pytest.raises(simcontrol.ControlError, match="nothing to do"):
        simcontrol.parse_command(body, LIMITS)


# FAILS IF: two verbs in one body are silently ordered by the implementation.
# "set the speed and unpause" has two possible meanings -- a paused world that
# resumes at the new speed, or a running world that changes speed -- and
# whichever the code happened to do would be untested.
def test_two_verbs_in_one_body_are_refused():
    with pytest.raises(simcontrol.ControlError, match="one command at a time"):
        simcontrol.parse_command({"speed": 2.0, "paused": True}, LIMITS)


# FAILS IF: a non-object body reaches the field lookups. JSON allows a bare
# list or number at the top level.
@pytest.mark.parametrize("body", [[], "pause", 3, None])
def test_a_non_object_body_is_refused(body):
    with pytest.raises(simcontrol.ControlError):
        simcontrol.parse_command(body, LIMITS)


# FAILS IF: the pause toggle starts carrying a state computed somewhere else.
# The whole reason TOGGLE_PAUSED exists is that the page must not decide the
# new value from a cached copy -- two tabs would fight over it.
def test_toggle_carries_no_state():
    cmd = simcontrol.parse_command({"toggle_pause": True}, LIMITS)
    assert cmd.verb == "toggle_pause"
    assert cmd.paused is False and cmd.speed == 0.0


# FAILS IF: an explicit pause state is dropped or inverted. This is the path a
# script uses; the button uses the toggle.
@pytest.mark.parametrize("want", [True, False])
def test_explicit_pause_is_carried_through(want):
    cmd = simcontrol.parse_command({"paused": want}, LIMITS)
    assert cmd.verb == "paused" and cmd.paused is want


# FAILS IF: reset needs a speed range to be known. Reset is the recovery path
# -- it is what a user reaches for when the simulator is in a state nobody
# understands -- so it must not depend on the control node having successfully
# reported anything first.
def test_reset_works_before_anything_is_known():
    assert simcontrol.parse_command({"reset": True}, NO_LIMITS).verb == "reset"


# FAILS IF: the disabled server still forwards. With control:=false the
# endpoint must refuse rather than reach for a client that was never created.
def test_disabled_control_refuses_every_command():
    client = simcontrol.SimControlClient(node=None, enabled=False)
    assert client.enabled is False
    assert client.state["enabled"] is False
    with pytest.raises(simcontrol.ControlError, match="disabled"):
        client.command({"reset": True})


# FAILS IF: the unknown state reports plausible numbers. A viewer that cannot
# tell "not connected yet" from "running at 1.00x" will show a confident speed
# for a simulator that is not there.
def test_unknown_state_has_no_invented_numbers():
    for key in ("requested_speed", "achieved_speed", "min_speed", "max_speed",
                "sim_time_s", "step_size_s", "backlog_s", "target_sim_time_s"):
        assert simcontrol.UNKNOWN[key] is None, key
    assert simcontrol.UNKNOWN["enabled"] is False


# FAILS IF: state_to_dict starts computing rather than copying. Every field in
# SimState is measured by the control node; a conversion here would be a second
# opinion about the simulator held by something not connected to it.
def test_state_to_dict_copies_field_for_field():
    class FakeMsg:
        enabled, paused, pacing = True, False, True
        requested_speed, achieved_speed = 0.25, 0.2431
        min_speed, max_speed = 0.05, 4.0
        sim_time_s, step_size_s, backlog_s = 96.132, 0.001, 0.004
        target_sim_time_s = 96.136
        resets, step_errors = 3, 0
        ticks, run_requests = 4021, 4019
        error = ""

    got = simcontrol.state_to_dict(FakeMsg())
    assert got["achieved_speed"] == 0.2431          # not rounded, not scaled
    assert got["requested_speed"] == 0.25
    assert got["sim_time_s"] == 96.132
    assert got["resets"] == 3
    assert set(got) == set(simcontrol.UNKNOWN)      # no field added or lost
