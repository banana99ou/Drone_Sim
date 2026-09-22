"""A space-time Bezier curve, turned into a time-parameterised trajectory.

The planner (banana99ou/bezier-trajectory, spacetime_bezier) optimises control
points in (x, y, z, t): the curve's parameter tau runs over [0, 1] and TIME is
just another coordinate of the curve. That is what lets it treat a moving
obstacle as a static one in four dimensions, and it is also why the output
cannot be handed to a controller directly -- dp/dtau is not a velocity.

This module is the conversion, and nothing else. Pure Python, no numpy, no
ROS: the whole point is that it can be unit tested and mutation-checked on the
host against finite differences, which is the one check that cannot be fooled
by an algebra slip.

Two facts about the conversion that decide how it is written:

  * Velocity and acceleration come from the chain rule with tau eliminated:
        v = p' / t'                     (' is d/dtau)
        a = (p'' t' - p' t'') / t'^3
    The second term of a is not optional. With uniform time knots t'' = 0 and
    dropping it changes nothing, which is exactly why it gets dropped; on an
    optimised plan the vehicle waits and then hurries, t'' is large, and the
    acceleration feedforward would be wrong by that whole term.

  * Samples are taken uniform in TIME, by inverting t(tau) with bisection --
    never uniform in tau. The planner's own geometry.py records measuring a
    72.7x variation in time spacing between neighbouring uniform-tau samples on
    a clustered curve. The controller interpolates linearly between samples,
    so a sample spacing that balloons where the curve waits is a feedforward
    that is stale for most of a second exactly where it matters least, and
    packed to no purpose where the curve moves.

The inversion is safe because the planner's optimiser emits a hard row
t[i+1] - t[i] >= min_dt on the control points; a Bezier's derivative control
points are N times those differences, all positive, and a convex combination
of positives is positive, so dt/dtau > 0 everywhere on the curve. `validate`
checks the row rather than trusting it: a plan that violates it is refused,
because a curve whose time runs backwards has two positions at one instant and
no trajectory to speak of.
"""
from __future__ import annotations

import math
from typing import Sequence

Point = Sequence[float]


class PlanError(ValueError):
    """The control points do not describe a flyable space-time curve."""


def validate(control_points: Sequence[Point]) -> list[list[float]]:
    """Return the control points as floats, or raise PlanError.

    Requires (n >= 2, 4) with time strictly increasing between consecutive
    control points -- the planner's monotonicity row, checked here because
    everything after it divides by dt/dtau.
    """
    P = [[float(c) for c in row] for row in control_points]
    if len(P) < 2:
        raise PlanError(f"need at least 2 control points, got {len(P)}")
    dims = {len(row) for row in P}
    if dims != {4}:
        raise PlanError(f"control points must be (x, y, z, t); got widths {sorted(dims)}")
    for i in range(len(P) - 1):
        if not P[i + 1][3] > P[i][3]:
            raise PlanError(
                f"time is not increasing between control points {i} and {i + 1} "
                f"({P[i][3]} -> {P[i + 1][3]}); dt/dtau would not be positive")
    return P


def derivative_control_points(P: Sequence[Point]) -> list[list[float]]:
    """Control points of dP/dtau: N * (P[i+1] - P[i])."""
    n = len(P) - 1
    return [[n * (b - a) for a, b in zip(P[i], P[i + 1])] for i in range(n)]


def de_casteljau(P: Sequence[Point], tau: float) -> list[float]:
    """Evaluate a Bezier curve at tau by repeated linear interpolation."""
    pts = [list(row) for row in P]
    while len(pts) > 1:
        pts = [[(1.0 - tau) * a + tau * b for a, b in zip(pts[i], pts[i + 1])]
               for i in range(len(pts) - 1)]
    return pts[0]


def evaluate(P: Sequence[Point], tau: float):
    """(position, first, second) derivatives with respect to TAU, each 4-wide."""
    d1 = derivative_control_points(P)
    p = de_casteljau(P, tau)
    dp = de_casteljau(d1, tau)
    if len(d1) >= 2:
        ddp = de_casteljau(derivative_control_points(d1), tau)
    else:
        ddp = [0.0] * len(p)
    return p, dp, ddp


def time_to_tau(P: Sequence[Point], t: float, iters: int = 60) -> float:
    """Invert t(tau) by bisection. t(tau) is monotone (see module docstring)."""
    t0, t1 = P[0][3], P[-1][3]
    if t <= t0:
        return 0.0
    if t >= t1:
        return 1.0
    lo, hi = 0.0, 1.0
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        if de_casteljau(P, mid)[3] < t:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def sample_at_time(P: Sequence[Point], t: float):
    """Position, velocity and acceleration (each xyz) at scenario time t."""
    tau = time_to_tau(P, t)
    p, dp, ddp = evaluate(P, tau)
    tdot, tddot = dp[3], ddp[3]
    if not tdot > 0.0:
        raise PlanError(f"dt/dtau = {tdot} at tau = {tau}; the curve is not a trajectory here")
    vel = [dp[i] / tdot for i in range(3)]
    acc = [(ddp[i] * tdot - dp[i] * tddot) / tdot ** 3 for i in range(3)]
    return p[:3], vel, acc


def sample_uniform_in_time(P: Sequence[Point], dt: float):
    """[(t, position, velocity, acceleration), ...] from the curve's start to its end.

    Times are t0 + k*dt for k = 0.. and the exact end time last, so the final
    sample lands on the planned arrival rather than up to dt short of it.
    """
    if not dt > 0.0:
        raise PlanError(f"sample dt must be positive, got {dt}")
    t0, t1 = P[0][3], P[-1][3]
    out = []
    k = 0
    while True:
        t = t0 + k * dt
        if t >= t1 - 1e-9:
            break
        out.append((t, *sample_at_time(P, t)))
        k += 1
    out.append((t1, *sample_at_time(P, t1)))
    return out


def summarize(samples) -> dict:
    """The numbers worth logging about a trajectory before flying it."""
    g = 9.80665
    max_v = max(math.sqrt(sum(c * c for c in s[2])) for s in samples)
    max_a = max(math.sqrt(sum(c * c for c in s[3])) for s in samples)
    # Tilt a quadrotor needs to produce that horizontal acceleration while
    # holding the vertical one: atan(|a_xy| / (g + a_z)).
    max_tilt = max(
        math.atan2(math.hypot(s[3][0], s[3][1]), g + s[3][2]) for s in samples)
    return {
        "points": len(samples),
        "duration_s": samples[-1][0] - samples[0][0],
        "max_speed_mps": max_v,
        "max_accel_mps2": max_a,
        "max_tilt_deg": math.degrees(max_tilt),
    }
