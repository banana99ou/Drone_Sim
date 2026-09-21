"""Ready-to-draw overlay geometry for the browser viewer.

THE RULE THIS FILE EXISTS TO HONOUR: all logic lives in the ROS simulator and
its scripts. The browser only draws.

So every physical decision is made here -- what the aerodynamic force is, which
frame a vector is in, how many metres of arrow one newton is worth -- and what
crosses the wire is a list of line segments in world coordinates with a
semantic label. The page projects them and strokes them. It never multiplies a
mass by an acceleration, never rotates a body vector into the world, and never
decides what a force is.

That split is not tidiness. A browser doing physics is physics you cannot test:
it needs a headless browser harness to inspect, it duplicates constants that
live in config/drone.yaml, and when a viewer disagrees with the simulator you
have two candidate culprits instead of one. Here it is a pure function of three
messages, and test_overlay.py asserts its output against hand-computed values.

Every number used comes off the wire (ControlDebug carries the vehicle's mass,
gravity and rotor positions for exactly this reason). Nothing in this file
knows anything about the vehicle that the simulator did not tell it.
"""
import math

# Arrow lengths. ONE scale for every force, so lengths are directly
# comparable: a thrust arrow twice as long as the weight arrow means twice the
# weight. Deliberately not auto-normalised to the biggest value on screen --
# that would draw hover and a hard bank identically, destroying the one thing
# these overlays exist to show. The viewer may multiply everything by a display
# gain, which changes all lengths together and never the ratios.
SCALE = {
    'force_m_per_n': 0.05,      # hover is 14.7 N total -> a 0.74 m arrow
    'vel_m_per_mps': 0.25,
    # Torques on this airframe run about 0.017 N.m in a steady turn, so 3.0
    # drew a 5 cm stub -- 4 pixels at the default camera, too short even to
    # earn a label. 20 puts it at 0.35 m / 30 px.
    'torque_m_per_nm': 20.0,
    'axis_m': 0.5,              # length of the unit attitude axes
}

# Aerodynamic force is around 0.35 N at cruise against a 14.7 N weight -- a 44x
# range. On the shared force scale that is 17 mm, which the renderer discards as
# shorter than its own 2 px floor: the arrow was never drawn at all. Rather than
# give it a private scale and pretend it is comparable to the others, it is
# magnified by a declared factor and the factor is written into its label. An
# exaggeration you can read is honest; a silent one is not.
AERO_MAGNIFY = 20.0

# No arrow may exceed this, however large the quantity behind it.
#
# Before the body-rate fix, one bad gyro sample per lap produced a 2.55 N.m
# torque demand -- a 51 m arrow at the scale above, 170x the collision
# envelope, flashing across the scene. Clamping keeps a broken input from
# taking over the picture, and `clamped` lets the page draw it differently so
# the clamp is visible rather than a quiet lie about magnitude. The label
# always carries the true value.
MAX_ARROW_M = 2.0

# Which toggle each arrow belongs to. Sent with the arrow so the page's
# checkboxes are a pure filter that needs no idea what a torque is.
GROUPS = ('motors', 'forces', 'velocity', 'attitude', 'torque', 'command')

# Decimal places on every arrow's numeric label. The wire carries full
# precision (WIRE_DECIMALS); this is only what the page prints next to the
# head, and it is one number rather than a per-arrow choice so two arrows can
# be compared digit by digit.
#
# Torque is the exception, and it is a units exception rather than a precision
# one: a steady turn on this airframe demands about 0.017 N.m, which at two
# decimals prints as "0.02" and throws away most of what there is to see. It
# is labelled in millinewton-metres instead, so the same two decimals carry
# three significant figures of a quantity that never reaches 0.1 N.m.
LABEL_DECIMALS = 2

# The position command is a tracking error -- 1 to 3 cm in normal flight,
# against a thrust arrow of 0.74 m. Drawn true to scale it is 1 mm, below the
# renderer's own 2 px floor, so it would silently not exist. Magnified, with
# the factor in the label and the true distance next to it.
TRACK_MAGNIFY = 10.0


def quat_matrix(q):
    """Hamilton quaternion [w, x, y, z] -> rotation matrix as three rows.

    Rotates BODY vectors into WORLD, matching the odometry convention.
    """
    w, x, y, z = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def rotate(m, v):
    """m * v, with m as rows."""
    return [sum(row[i] * v[i] for i in range(3)) for row in m]


def body_z(m):
    """The body +z axis in world coordinates: the third column of m."""
    return [m[0][2], m[1][2], m[2][2]]


def body_x(m):
    return [m[0][0], m[1][0], m[2][0]]


def _add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def _sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _mul(a, s):
    return [a[0] * s, a[1] * s, a[2] * s]


def _norm(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _unit(a):
    n = _norm(a)
    # A legitimately zero vector (no torque demanded while hovering) must not
    # become a NaN: one NaN endpoint poisons a whole projected polygon.
    return [0.0, 0.0, 0.0] if n < 1e-12 else _mul(a, 1.0 / n)


# Wire precision for every coordinate this module emits. One micrometre is
# absurdly finer than anything a screen can show, and rounding still strips the
# float noise that bloats the JSON -- repr(0.14000000000000001) is 19 characters
# of nothing. It is stated as a constant, and tested to, because "how exact is
# this number" is part of the contract with the page: test_overlay.py asserts
# geometry to exactly this tolerance.
WIRE_DECIMALS = 6
WIRE_TOLERANCE = 10.0 ** -WIRE_DECIMALS


def _round(v):
    return [round(x, WIRE_DECIMALS) for x in v]


def _arrow(group, kind, origin, vector, label, **extra):
    clamped = False
    length = _norm(vector)
    if length > MAX_ARROW_M:
        vector = _mul(vector, MAX_ARROW_M / length)
        clamped = True
    a = {
        'group': group,
        'kind': kind,
        'clamped': clamped,
        # `from` doubles as the anchor the page scales the arrow about when the
        # user turns up the display gain: to' = from + (to - from) * gain. That
        # is a display zoom, the same kind of operation as the camera's, and it
        # cannot change a ratio between two arrows.
        'from': _round(origin),
        'to': _round(_add(origin, vector)),
        'label': label,
    }
    a.update(extra)
    return a


def aero_force_body(control, imu):
    """MEASURED aerodynamic force on the airframe, in the body frame.

    An accelerometer reads proper acceleration: total non-gravitational force
    over mass, in the body frame. So mass * accel is thrust plus everything
    else acting on the airframe, and subtracting the thrust the rotors are
    actually producing leaves rotor drag, rolling moments, and any effect the
    plant has that the controller does not model.

    This is a measurement, not a model. The alternative was to re-evaluate
    Gazebo's drag coefficient, which would have meant holding a second opinion
    about the vehicle's physics and drawing a guess in the same style as a
    measurement.

    Returns None when there is no IMU sample yet, so a consumer can tell
    "no data" from "zero drag".
    """
    if not imu or not imu.get('accel_body'):
        return None
    measured = _mul(imu['accel_body'], control['mass_kg'])
    return _sub(measured, [0.0, 0.0, control['realised_thrust_n']])


def build(pose, control, imu):
    """Overlay for one frame, or None when there is nothing to draw yet.

    Returns {'arrows': [...], 'ticks': [...], 'readout': {...}, 'scale': {...}}
    with every position already in WORLD metres.
    """
    if not pose or not control:
        return None

    m = quat_matrix(pose['q'])
    p = list(pose['p'])
    scale_f = SCALE['force_m_per_n']
    arrows = []
    ticks = []

    thrust_body = [0.0, 0.0, control['realised_thrust_n']]
    aero_body = aero_force_body(control, imu)

    readout = {
        'speed_mps': _norm(control['velocity_world']),
        # What each loop is asking for, as one number per level, so the HUD
        # can show the commands even when an arrow is too short to label.
        'cmd_track_m': _norm(control['position_error']),
        'cmd_speed_mps': _norm(_sub(control['velocity_world'],
                                    control['velocity_error'])),
        'saturated': bool(control['saturated']),
        'tilt_clamped': bool(control['tilt_clamped']),
        'tilt_deg': math.degrees(control['commanded_tilt_rad']),
        'torque_nm': _norm(control['realised_torque_nm']),
        'aero_n': None if aero_body is None else _norm(aero_body),
        'hover_thrust_n': control['mass_kg'] * control['gravity_m_s2'],
    }

    # Disarmed means every demand is zero. Four zero-length arrows and a torque
    # of nothing is not information; the readout still says why.
    if not control['armed']:
        return {'arrows': [], 'ticks': [], 'readout': readout, 'scale': SCALE}

    # ---- one arrow per rotor, along body +z, length proportional to thrust --
    hubs = control['rotor_position']
    thrusts = control['rotor_thrust_n']
    hover_each = readout['hover_thrust_n'] / max(len(hubs), 1)
    up = body_z(m)
    across = _mul(body_x(m), 0.05)
    for i, hub in enumerate(hubs):
        base = _add(p, rotate(m, hub))
        f = thrusts[i]
        arrows.append(_arrow(
            'motors', 'rotor', base, _mul(up, f * scale_f),
            f'{f:.{LABEL_DECIMALS}f}',
            # Deviation from hover in [-1, 1]. A quadrotor holds a bank by
            # splitting thrust across a diagonal, so this is the number that
            # makes the control action readable: two rotors go one way, two the
            # other, and the split grows with the manoeuvre. The page maps it
            # to a colour; it does not compute it.
            norm=round(max(-1.0, min(1.0, (f - hover_each) / hover_each)),
                      WIRE_DECIMALS)
            if hover_each > 0 else 0.0,
            rotor=i))
        # Reference mark at exactly hover thrust, so an arrow is not merely a
        # length: you can see which rotors are above weight and which below.
        at = _add(base, _mul(up, hover_each * scale_f))
        ticks.append({'anchor': _round(base),
                      'a': _round(_sub(at, across)),
                      'b': _round(_add(at, across))})

    # ---- forces at the centre of mass --------------------------------------
    arrows.append(_arrow(
        'forces', 'thrust', p, _mul(rotate(m, thrust_body), scale_f),
        f'thrust {control["realised_thrust_n"]:.{LABEL_DECIMALS}f} N'))
    weight = readout['hover_thrust_n']
    arrows.append(_arrow(
        'forces', 'weight', p, [0.0, 0.0, -weight * scale_f],
        f'weight {weight:.{LABEL_DECIMALS}f} N'))
    if aero_body is not None:
        # Magnified, and the label says so -- see AERO_MAGNIFY.
        arrows.append(_arrow(
            'forces', 'aero', p,
            _mul(rotate(m, aero_body), scale_f * AERO_MAGNIFY),
            f'aero {_norm(aero_body):.{LABEL_DECIMALS}f} N '
            f'x{AERO_MAGNIFY:.0f}'))

    # ---- velocity ----------------------------------------------------------
    # control['velocity_world'] is already resolved into the world frame by the
    # controller. Using the raw odometry twist would be wrong: it is body-frame
    # by REP-145, and drawing it as world-frame looks right at hover and is
    # quietly wrong in every turn.
    v = control['velocity_world']
    arrows.append(_arrow(
        'velocity', 'velocity', p, _mul(v, SCALE['vel_m_per_mps']),
        f'{_norm(v):.{LABEL_DECIMALS}f} m/s'))

    # ---- attitude: what the vehicle is doing vs what was demanded ----------
    # Unit directions, so these carry no magnitude to exaggerate. The gap
    # between them IS the attitude error the inner loop is working on -- if
    # they separate visibly and the torque arrow is zero, something is broken.
    axis = SCALE['axis_m']
    arrows.append(_arrow('attitude', 'body_axis', p, _mul(up, axis), 'body z'))

    # ---- torque ------------------------------------------------------------
    tau = control['realised_torque_nm']
    if _norm(tau) > 1e-9:
        arrows.append(_arrow(
            'torque', 'torque', p,
            _mul(rotate(m, tau), SCALE['torque_m_per_nm']),
            f'{1e3 * _norm(tau):.{LABEL_DECIMALS}f} mN.m'))

    # ---- what each loop of the cascade is COMMANDING -----------------------
    # One arrow per level of the controller, in the order the cascade runs, so
    # what the stack is asking for can be read next to what the vehicle is
    # doing. Every actual has its counterpart already on screen: the vehicle
    # itself for position, the velocity arrow for velocity, body z for
    # attitude. These three are the other half of each of those pairs, and
    # they share one toggle because reading one without the others tells you
    # which loop is unhappy but not why.
    #
    # ControlDebug publishes ERRORS, defined as state - reference, so each
    # reference is recovered by subtraction rather than re-derived here. That
    # keeps the arrow tied to the number the controller actually used: a
    # trajectory re-sampled in this file could disagree with the one the loop
    # ran on, and the disagreement would look like a tracking failure.
    #
    #   cmd_position  where the position loop wants the vehicle to BE, drawn
    #                 from where it is. This is the tracking error vector,
    #                 pointing at the target.
    #   cmd_velocity  the velocity the loop is asking for, on the same scale
    #                 as the actual velocity arrow; the gap between them is
    #                 the velocity error the loop is reacting to.
    #   cmd_axis      the attitude demand: the direction the thrust axis is
    #                 being asked to point, against body z.
    ref_p = _sub(p, control['position_error'])
    track = _norm(control['position_error'])
    if track > 1e-9:
        # Tracking runs at a few centimetres while the arrows around it are
        # hundreds of times longer, so at true scale this one is shorter than
        # the renderer's 2 px floor and never gets drawn at all. It is
        # magnified by a declared factor, with the true distance in the label
        # -- the same bargain the aero arrow makes. An exaggeration you can
        # read is honest; a silent one is not.
        arrows.append(_arrow(
            'command', 'cmd_position', p,
            _mul(_sub(ref_p, p), TRACK_MAGNIFY),
            f'cmd pos {track:.{LABEL_DECIMALS}f} m x{TRACK_MAGNIFY:.0f}',
            dashed=True))

    ref_v = _sub(v, control['velocity_error'])
    if _norm(ref_v) > 1e-9:
        arrows.append(_arrow(
            'command', 'cmd_velocity', p,
            _mul(ref_v, SCALE['vel_m_per_mps']),
            f'cmd vel {_norm(ref_v):.{LABEL_DECIMALS}f} m/s', dashed=True))

    cmd = _unit(control['desired_force'])
    if _norm(cmd) > 0:
        arrows.append(_arrow(
            'command', 'cmd_axis', p, _mul(cmd, axis * 1.2),
            f'cmd tilt {readout["tilt_deg"]:.{LABEL_DECIMALS}f} deg',
            dashed=True))

    return {'arrows': arrows, 'ticks': ticks, 'readout': readout, 'scale': SCALE}
