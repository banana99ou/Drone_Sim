"""Invariants for the overlay geometry the browser draws.

Each test says what would make it fail. The point of testing this on the ROS
side is that it is testable at all: the same arithmetic done in the page would
need a headless browser to inspect, and an arrow pointing the wrong way looks
plausible on a screen -- it is confidently wrong, which is worse than blank.
"""
import math

import pytest

from dsim_viz import overlay

# Geometry is asserted to the precision overlay.py promises on the wire, not to
# machine epsilon. Coordinates are rounded before they are sent (see
# WIRE_DECIMALS), so a tighter tolerance here would be testing float noise, and
# a looser one would let a real error through.
TOL = overlay.WIRE_TOLERANCE

MASS = 1.5
G = 9.80665
WEIGHT = MASS * G
HOVER_EACH = WEIGHT / 4.0
D = 0.2 / math.sqrt(2.0)

# The four hubs, in mixer index order, as ControlDebug reports them.
HUBS = [
    {'x': D, 'y': D, 'z': 0.0},
    {'x': -D, 'y': -D, 'z': 0.0},
    {'x': D, 'y': -D, 'z': 0.0},
    {'x': -D, 'y': D, 'z': 0.0},
]


def control(**over):
    """A level hover control step, overridable field by field."""
    c = {
        'armed': True,
        'mass_kg': MASS,
        'gravity_m_s2': G,
        'max_rotor_thrust_n': 9.0,
        'thrust_n': WEIGHT,
        'torque_nm': [0.0, 0.0, 0.0],
        'realised_thrust_n': WEIGHT,
        'realised_torque_nm': [0.0, 0.0, 0.0],
        'saturated': False,
        'rotor_position': [[h['x'], h['y'], h['z']] for h in HUBS],
        'rotor_thrust_n': [HOVER_EACH] * 4,
        'rotor_speed_rad_s': [639.0] * 4,
        'velocity_world': [0.0, 0.0, 0.0],
        'position_error': [0.0, 0.0, 0.0],
        'velocity_error': [0.0, 0.0, 0.0],
        'attitude_error': [0.0, 0.0, 0.0],
        'body_rate_error': [0.0, 0.0, 0.0],
        'desired_force': [0.0, 0.0, WEIGHT],
        'commanded_tilt_rad': 0.0,
        'tilt_clamped': False,
    }
    c.update(over)
    return c


def pose(p=(0.0, 0.0, 1.5), q=(1.0, 0.0, 0.0, 0.0)):
    return {'p': list(p), 'q': list(q)}


def imu(accel):
    return {'accel_body': list(accel), 'rate_body': [0.0, 0.0, 0.0]}


def hover_imu():
    """What the accelerometer reads in a true hover: the reaction to gravity."""
    return imu([0.0, 0.0, WEIGHT / MASS])


def arrows_of(result, kind):
    return [a for a in result['arrows'] if a['kind'] == kind]


def length(a):
    return math.dist(a['from'], a['to'])


# FAILS IF: quat_matrix stops being a rotation. Two independent properties that
# cannot both hold for a wrong matrix: it must be orthonormal, and it must
# preserve orientation (det +1, not -1, which would mirror the whole vehicle).
@pytest.mark.parametrize('q', [
    (1.0, 0.0, 0.0, 0.0),
    (math.cos(0.3), 0.0, 0.0, math.sin(0.3)),
    (math.cos(0.4), math.sin(0.4) * 0.6, math.sin(0.4) * 0.8, 0.0),
])
def test_quat_matrix_is_a_rotation(q):
    m = overlay.quat_matrix(q)
    for i in range(3):
        for j in range(3):
            expect = 1.0 if i == j else 0.0
            got = sum(m[i][k] * m[j][k] for k in range(3))
            assert got == pytest.approx(expect, abs=1e-12)
    det = (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    assert det == pytest.approx(1.0, abs=1e-12)


# FAILS IF: there is nothing to draw and we draw something anyway. A viewer
# that renders arrows before the first control message would be showing a
# vehicle state that has never existed.
def test_no_control_means_no_overlay():
    assert overlay.build(None, control(), None) is None
    assert overlay.build(pose(), None, None) is None


# FAILS IF: a disarmed vehicle still gets force arrows. All demands are zero,
# so the arrows would be a pile of zero-length lines claiming to be a picture.
def test_disarmed_draws_nothing_but_still_reports():
    r = overlay.build(pose(), control(armed=False), hover_imu())
    assert r['arrows'] == []
    assert r['ticks'] == []
    assert r['readout']['hover_thrust_n'] == pytest.approx(WEIGHT)


# FAILS IF: an arrow lands on the wrong arm. Arrow i must start at rotor i's
# hub, transformed by the pose being drawn. This is the check that a rotor's
# thrust is never attributed to a different rotor -- the failure that makes a
# correct controller look broken.
def test_rotor_arrows_start_at_their_own_hub():
    p = (1.0, 2.0, 3.0)
    r = overlay.build(pose(p=p), control(), hover_imu())
    rotors = arrows_of(r, 'rotor')
    assert len(rotors) == 4
    for i, a in enumerate(rotors):
        assert a['rotor'] == i
        hub = HUBS[i]
        assert a['from'] == pytest.approx(
            [p[0] + hub['x'], p[1] + hub['y'], p[2] + hub['z']], abs=TOL)


# FAILS IF: body-frame quantities are not rotated by the pose. Yawed 90 deg,
# rotor 0's hub (+d, +d) must appear at world (-d, +d): if this passes with the
# rotation dropped, the arrows would stay axis-aligned while the drone turns.
def test_yaw_rotates_the_hubs():
    q = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))   # +90 deg yaw
    r = overlay.build(pose(p=(0, 0, 0), q=q), control(), hover_imu())
    first = arrows_of(r, 'rotor')[0]
    assert first['from'] == pytest.approx([-D, D, 0.0], abs=TOL)


# FAILS IF: rotor arrow length stops tracking rotor thrust. At hover all four
# must be identical, and the hover tick must sit exactly at the tip -- that is
# what makes "above or below hover" readable at a glance.
def test_hover_gives_four_equal_arrows_with_ticks_at_the_tip():
    r = overlay.build(pose(), control(), hover_imu())
    rotors = arrows_of(r, 'rotor')
    lengths = [length(a) for a in rotors]
    assert lengths == pytest.approx(
        [HOVER_EACH * overlay.SCALE['force_m_per_n']] * 4, abs=TOL)
    for a, t in zip(rotors, r['ticks']):
        mid = [(t['a'][k] + t['b'][k]) / 2 for k in range(3)]
        assert mid == pytest.approx(a['to'], abs=TOL)
        assert a['norm'] == pytest.approx(0.0, abs=TOL)


# FAILS IF: the deviation from hover loses its sign or its clamp. A rotor
# working harder than hover must read positive so the page can colour it, and a
# demand beyond the clamp must not run off the end of the colour scale.
def test_deviation_from_hover_is_signed_and_clamped():
    thrusts = [HOVER_EACH * 1.5, HOVER_EACH * 0.5, 0.0, HOVER_EACH * 9.0]
    r = overlay.build(pose(), control(rotor_thrust_n=thrusts), hover_imu())
    norms = [a['norm'] for a in arrows_of(r, 'rotor')]
    assert norms[0] == pytest.approx(0.5)
    assert norms[1] == pytest.approx(-0.5)
    assert norms[2] == pytest.approx(-1.0)
    assert norms[3] == pytest.approx(1.0)


# FAILS IF: thrust and weight stop cancelling in a true hover. Two arrows
# computed from different numbers -- the realised rotor thrust and mass * g --
# must be exactly opposite when the vehicle is holding altitude. If they are
# not, one of them is wrong, and the picture would show a hovering drone being
# pushed somewhere.
def test_hover_thrust_and_weight_cancel():
    r = overlay.build(pose(), control(), hover_imu())
    thrust = arrows_of(r, 'thrust')[0]
    weight = arrows_of(r, 'weight')[0]
    total = [(thrust['to'][k] - thrust['from'][k])
             + (weight['to'][k] - weight['from'][k]) for k in range(3)]
    assert total == pytest.approx([0.0, 0.0, 0.0], abs=TOL)


# FAILS IF: the aero arrow is not the measured residual. Feeding an IMU that
# reads exactly the thrust the rotors are producing means there is nothing else
# acting on the airframe, so the aero force must be zero. A non-zero answer
# here means the subtraction, the mass, or the frame is wrong.
def test_aero_is_zero_when_the_imu_sees_only_thrust():
    r = overlay.build(pose(), control(), hover_imu())
    assert r['readout']['aero_n'] == pytest.approx(0.0, abs=TOL)
    aero = arrows_of(r, 'aero')[0]
    assert length(aero) == pytest.approx(0.0, abs=TOL)


# FAILS IF: the residual is scaled or signed wrongly. An extra 0.2 m/s^2 of
# measured acceleration on body x, at 1.5 kg, is 0.3 N of aerodynamic force
# along body x -- computed by hand, not by the code under test.
#
# The READOUT must carry the true newtons: the drawn arrow is magnified for
# legibility (see AERO_MAGNIFY), and if that exaggeration leaked into the
# number the HUD shows, the viewer would be reporting a force five times
# larger than the one being measured.
def test_aero_equals_mass_times_the_unexplained_acceleration():
    extra = 0.2
    r = overlay.build(pose(), control(), imu([extra, 0.0, WEIGHT / MASS]))
    assert r['readout']['aero_n'] == pytest.approx(MASS * extra)
    aero = arrows_of(r, 'aero')[0]
    delta = [aero['to'][k] - aero['from'][k] for k in range(3)]
    drawn = MASS * extra * overlay.SCALE['force_m_per_n'] * overlay.AERO_MAGNIFY
    assert delta == pytest.approx([drawn, 0.0, 0.0], abs=TOL)
    assert delta[0] > 0, 'drag along +x must be drawn along +x'


# FAILS IF: "no IMU yet" is reported as "no drag". Those are different claims
# and a viewer must be able to tell them apart.
def test_missing_imu_yields_no_aero_arrow_rather_than_a_zero_one():
    r = overlay.build(pose(), control(), None)
    assert arrows_of(r, 'aero') == []
    assert r['readout']['aero_n'] is None


# FAILS IF: the velocity arrow gets rotated. velocity_world is already in the
# world frame; rotating it again would look correct at hover and be wrong in
# every turn -- the exact silent error the field exists to prevent.
def test_velocity_arrow_is_not_rotated_again():
    q = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
    v = [2.0, 0.0, 0.0]
    r = overlay.build(pose(p=(0, 0, 0), q=q), control(velocity_world=v), hover_imu())
    a = arrows_of(r, 'velocity')[0]
    delta = [a['to'][k] - a['from'][k] for k in range(3)]
    assert delta == pytest.approx(
        [2.0 * overlay.SCALE['vel_m_per_mps'], 0.0, 0.0], abs=TOL)


# FAILS IF: a tilted demand does not separate the two attitude axes. That gap
# is the attitude error the inner loop is working on; if the commanded axis
# tracked the body axis regardless of the demand, the overlay would show a
# controller that never disagrees with the vehicle.
def test_tilted_demand_separates_the_attitude_axes():
    tilted = control(desired_force=[WEIGHT * 0.3, 0.0, WEIGHT],
                     commanded_tilt_rad=math.atan2(0.3, 1.0))
    r = overlay.build(pose(p=(0, 0, 0)), tilted, hover_imu())
    body = arrows_of(r, 'body_axis')[0]
    cmd = arrows_of(r, 'cmd_axis')[0]
    assert cmd['dashed'] is True
    # Level vehicle: body z is straight up, the demand is not.
    assert body['to'] == pytest.approx([0.0, 0.0, overlay.SCALE['axis_m']], abs=TOL)
    assert cmd['to'][0] > 0.05


# FAILS IF: a zero torque still draws an arrow. At steady hover the torque is
# genuinely nothing, and a stub arrow of arbitrary length would be an invented
# reading.
def test_zero_torque_draws_no_torque_arrow():
    r = overlay.build(pose(), control(), hover_imu())
    assert arrows_of(r, 'torque') == []
    r = overlay.build(pose(), control(realised_torque_nm=[0.02, 0.0, 0.0]), hover_imu())
    assert len(arrows_of(r, 'torque')) == 1


# FAILS IF: an arrow is emitted with a group the page has no checkbox for --
# it would be undrawable, or worse, drawn unconditionally.
def test_every_arrow_belongs_to_a_known_group():
    r = overlay.build(pose(), control(realised_torque_nm=[0.02, 0.0, 0.0]), hover_imu())
    assert r['arrows']
    for a in r['arrows']:
        assert a['group'] in overlay.GROUPS


# FAILS IF: the aero arrow is drawn at the shared force scale again. At 0.35 N
# that is 17 mm, which the renderer discards as shorter than its own 2 px
# floor -- the arrow silently does not exist. It must be magnified, and by
# exactly the factor its label advertises, or the label is a lie.
def test_aero_arrow_is_magnified_by_the_factor_it_advertises():
    r = overlay.build(pose(), control(), imu([0.2, 0.0, WEIGHT / MASS]))
    aero = arrows_of(r, 'aero')[0]
    plain = MASS * 0.2 * overlay.SCALE['force_m_per_n']
    assert length(aero) == pytest.approx(plain * overlay.AERO_MAGNIFY, abs=TOL)
    assert f'x{overlay.AERO_MAGNIFY:.0f}' in aero['label']
    # The true value stays in the label; only the drawn length is exaggerated.
    assert f'{MASS * 0.2:.2f} N' in aero['label']


# FAILS IF: an arrow can grow without bound. One bad gyro sample used to demand
# 2.55 N.m, which at the torque scale is a 51 m arrow -- 170x the collision
# envelope, flashing across the whole scene. A clamp is what stops a broken
# input from taking over the picture.
def test_absurd_torque_is_clamped_and_says_so():
    r = overlay.build(pose(), control(realised_torque_nm=[2.5456, 0.0, 0.0]),
                      hover_imu())
    tau = arrows_of(r, 'torque')[0]
    assert length(tau) == pytest.approx(overlay.MAX_ARROW_M, abs=TOL)
    assert tau['clamped'] is True
    # ...and the label still reports what was really demanded, unclamped.
    assert '2.546' in tau['label']


# FAILS IF: ordinary arrows are marked clamped. The flag drives a different
# stroke in the page, so a false positive would make every normal frame look
# like a fault.
def test_normal_arrows_are_not_clamped():
    r = overlay.build(pose(), control(realised_torque_nm=[0.0175, 0.0, 0.0]),
                      imu([0.2, 0.0, WEIGHT / MASS]))
    assert r['arrows']
    for a in r['arrows']:
        assert a['clamped'] is False, f"{a['kind']} was clamped unexpectedly"


# FAILS IF: the torque scale drifts back to something that draws a 4 px stub.
# A steady turn on this airframe is about 0.017 N.m; at the default camera
# distance the viewer needs roughly 0.3 m of arrow for that to be legible and
# carry a label.
def test_typical_torque_is_long_enough_to_read():
    r = overlay.build(pose(), control(realised_torque_nm=[0.0, 0.0, 0.0175]),
                      hover_imu())
    tau = arrows_of(r, 'torque')[0]
    assert 0.25 < length(tau) < overlay.MAX_ARROW_M
