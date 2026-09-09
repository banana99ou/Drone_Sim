"""Invariants for the one write path in the viewer stack.

This is the only code that can change the running simulation, so a mistake
here is not a cosmetic bug. Each test says what it would catch.
"""
import subprocess

import pytest

from dsim_viz import simcontrol


# FAILS IF: a value from the network reaches the simulator unchecked. A string,
# a null or a NaN handed to set_physics is much harder to notice than a
# refused request -- the sim would just behave oddly from then on.
@pytest.mark.parametrize('bad', ['fast', None, [], {}, float('nan')])
def test_non_numeric_speed_is_refused(bad):
    with pytest.raises(simcontrol.ControlError):
        simcontrol.validate_rtf(bad)


# FAILS IF: the range check is missing or one-sided. Zero would stop time while
# reporting itself as running, and a huge factor makes the simulator silently
# fail to keep up -- the number would be a lie rather than a speed.
@pytest.mark.parametrize('bad', [0, -1, 0.049, 4.01, 1000])
def test_out_of_range_speed_is_refused(bad):
    with pytest.raises(simcontrol.ControlError):
        simcontrol.validate_rtf(bad)


# FAILS IF: the bounds themselves are rejected, which would make the slider's
# own end stops unusable.
@pytest.mark.parametrize('good', [simcontrol.MIN_RTF, 0.5, 1, '2.0',
                                  simcontrol.MAX_RTF])
def test_valid_speeds_are_accepted(good):
    assert simcontrol.MIN_RTF <= simcontrol.validate_rtf(good) <= simcontrol.MAX_RTF


# FAILS IF: the command is ever built as a shell string. Passing a list means
# nothing from a request can be reinterpreted as shell syntax, which is the
# whole reason this server is safe to expose on a tailnet.
def test_commands_are_argument_lists_not_shell_strings():
    for cmd in (simcontrol.pause_command('/gz', 'w', True),
                simcontrol.rtf_command('/gz', 'w', 1.0),
                simcontrol.stats_command('/gz', 'w')):
        assert isinstance(cmd, list)
        assert all(isinstance(a, str) for a in cmd)


# FAILS IF: pause and resume send the same request. They differ by one word, so
# a copy-paste error here gives a pause button that does nothing on the second
# press -- and looks like a UI bug for a long time.
def test_pause_and_resume_differ():
    on = simcontrol.pause_command('/gz', 'drone_pillars', True)
    off = simcontrol.pause_command('/gz', 'drone_pillars', False)
    assert 'pause: true' in on
    assert 'pause: false' in off
    assert '/world/drone_pillars/control' in on


# FAILS IF: the physics step is dropped from the speed request. gz.msgs.Physics
# replaces the whole profile, so omitting max_step_size resets the integration
# step -- changing accuracy while the user thinks they changed only speed.
def test_speed_request_preserves_the_physics_step():
    cmd = simcontrol.rtf_command('/gz', 'w', 0.25)
    req = cmd[cmd.index('--req') + 1]
    assert 'real_time_factor: 0.25' in req
    assert 'max_step_size: 0.001' in req


# FAILS IF: a missing `paused` field is read as unknown or as paused. Protobuf
# text format omits false booleans, so a running world says nothing at all
# about pause -- and a viewer that showed "paused" for every normal frame would
# be useless.
def test_absent_paused_field_means_running():
    running = """real_time_factor: 0.9998
step_size {
  nsec: 1000000
}
"""
    assert simcontrol.parse_stats(running) == {'paused': False,
                                               'measured_rtf': 0.9998}


# FAILS IF: an explicit pause is missed. This is the output shape gz actually
# produces, indented inside the message.
def test_explicit_paused_field_is_read():
    out = "real_time_factor: 0\n  paused: true\n"
    parsed = simcontrol.parse_stats(out)
    assert parsed['paused'] is True
    assert parsed['measured_rtf'] == 0.0


# FAILS IF: garbage output crashes the parser. gz can print warnings or nothing
# at all, and a viewer must degrade to "unknown", never to a traceback in an
# HTTP handler.
def test_unparseable_output_yields_no_factor_rather_than_raising():
    assert simcontrol.parse_stats('') == {'paused': False, 'measured_rtf': None}
    assert simcontrol.parse_stats('Error: no such topic')['measured_rtf'] is None


# FAILS IF: control can be exercised on a server where it was switched off.
# The launch file can disable it, and that must actually hold.
def test_disabled_control_refuses_everything():
    sc = simcontrol.SimControl('w', enabled=False)
    with pytest.raises(simcontrol.ControlError):
        sc.set_paused(True)
    with pytest.raises(simcontrol.ControlError):
        sc.set_rtf(1.0)
    assert sc.state['enabled'] is False


# FAILS IF: reported state is the REQUEST rather than the observation. If a
# command fails, or someone pauses from the Gazebo GUI, the viewer must show
# what is true, not what it asked for.
def test_state_reports_what_was_observed_not_what_was_asked():
    calls = []

    class Done:
        stdout = "real_time_factor: 0\n  paused: true\n"
        stderr = ''
        returncode = 0

    def fake_run(cmd, **kwargs):
        calls.append(cmd)
        return Done()

    sc = simcontrol.SimControl('w', enabled=True, runner=fake_run)
    if not sc.state['enabled']:
        pytest.skip('gz CLI not present in this environment')
    # Ask to RESUME, while the simulator keeps reporting itself paused.
    state = sc.set_paused(False)
    assert state['paused'] is True, 'reported the request instead of the truth'
    assert any('pause: false' in c for c in calls)


# FAILS IF: a hung simulator hangs the HTTP handler. Every gz call must be
# bounded, or one unresponsive service call blocks a viewer thread forever.
def test_a_hanging_simulator_becomes_an_error_not_a_hang():
    def fake_run(cmd, **kwargs):
        raise subprocess.TimeoutExpired(cmd, simcontrol.TIMEOUT_S)

    sc = simcontrol.SimControl('w', enabled=True, runner=fake_run)
    if not sc.state['enabled']:
        pytest.skip('gz CLI not present in this environment')
    with pytest.raises(simcontrol.ControlError):
        sc.set_paused(True)
