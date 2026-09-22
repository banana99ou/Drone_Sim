"""The conversion from a space-time Bezier to a trajectory.

The load-bearing tests compare the analytic derivatives against finite
differences of the position IN TIME, on a curve whose time coordinate is
deliberately non-uniform. That is the one comparison an algebra slip cannot
pass: dp/dtau reported as velocity is off by dt/dtau, and an acceleration
missing its t'' term is off by exactly that term, and both are zero-cost
mistakes on a uniform-time curve -- which is why the curve here is not one.
"""
import math

import pytest

from dsim_planner import spacetime as st

# The straight seed for fence3d: 9 m in 10 s, one line, uniform time.
SEED = [[0.5 + 1.125 * i, 5.0, 0.5, 1.25 * i] for i in range(9)]

# A curve that waits, climbs, then hurries: time knots bunch at the start
# (t' small) and stretch at the end, and the spatial polygon bends in all
# three axes so no component is trivially zero.
CLUSTERED = [
    [0.0, 0.0, 1.0, 0.0],
    [0.5, 0.2, 1.0, 0.4],
    [1.0, 0.9, 1.4, 0.9],
    [3.0, 1.5, 2.2, 1.6],
    [6.0, 1.2, 1.5, 4.0],
    [8.0, 0.4, 1.0, 7.5],
    [9.0, 0.0, 1.0, 10.0],
]


def norm(v):
    return math.sqrt(sum(c * c for c in v))


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


# ---- validation ----------------------------------------------------------

def test_rejects_time_running_backwards():
    P = [[0, 0, 0, 0.0], [1, 0, 0, 2.0], [2, 0, 0, 1.5], [3, 0, 0, 3.0]]
    with pytest.raises(st.PlanError, match="not increasing"):
        st.validate(P)


def test_rejects_equal_times_too():
    # Equal is not "increasing": t' would be zero there and v = p'/0.
    P = [[0, 0, 0, 0.0], [1, 0, 0, 1.0], [2, 0, 0, 1.0]]
    with pytest.raises(st.PlanError):
        st.validate(P)


def test_rejects_the_wrong_width():
    with pytest.raises(st.PlanError, match=r"\(x, y, z, t\)"):
        st.validate([[0, 0, 0], [1, 1, 1]])


def test_accepts_the_seed_and_the_clustered_curve():
    assert st.validate(SEED) == SEED
    st.validate(CLUSTERED)


# ---- the curve itself ------------------------------------------------------

def test_de_casteljau_matches_bernstein_on_a_cubic():
    P = [[0, 0, 0, 0], [1, 2, 0, 1], [3, 2, 1, 2], [4, 0, 1, 3]]
    for tau in (0.0, 0.13, 0.5, 0.77, 1.0):
        b = [(1 - tau) ** 3, 3 * tau * (1 - tau) ** 2, 3 * tau ** 2 * (1 - tau), tau ** 3]
        expect = [sum(b[i] * P[i][k] for i in range(4)) for k in range(4)]
        got = st.de_casteljau(P, tau)
        assert got == pytest.approx(expect, abs=1e-12)


def test_endpoints_are_the_first_and_last_control_points():
    for P in (SEED, CLUSTERED):
        assert st.de_casteljau(P, 0.0) == pytest.approx(P[0])
        assert st.de_casteljau(P, 1.0) == pytest.approx(P[-1])


def test_time_inversion_round_trips():
    for t in (0.0, 0.05, 0.37, 1.0, 2.9, 5.5, 9.99, 10.0):
        tau = st.time_to_tau(CLUSTERED, t)
        assert st.de_casteljau(CLUSTERED, tau)[3] == pytest.approx(t, abs=1e-9)


def test_time_is_genuinely_non_uniform_on_the_clustered_curve():
    # Guard on the test fixture: if this ever became uniform, the derivative
    # tests below would stop being able to fail on the mistakes they exist for.
    taus = [st.time_to_tau(CLUSTERED, t) for t in (0.0, 2.5, 5.0, 7.5, 10.0)]
    gaps = [b - a for a, b in zip(taus, taus[1:])]
    assert max(gaps) / min(gaps) > 1.8


# ---- the conversion: derivatives against finite differences ----------------

@pytest.mark.parametrize("t", [0.3, 1.2, 2.0, 3.3, 5.0, 6.6, 8.8, 9.7])
def test_velocity_is_the_time_derivative_of_position(t):
    h = 1e-4
    _, v, _ = st.sample_at_time(CLUSTERED, t)
    p_plus, _, _ = st.sample_at_time(CLUSTERED, t + h)
    p_minus, _, _ = st.sample_at_time(CLUSTERED, t - h)
    fd = [(a - b) / (2 * h) for a, b in zip(p_plus, p_minus)]
    assert v == pytest.approx(fd, abs=1e-5), f"v {v} vs finite difference {fd}"
    # And it is NOT dp/dtau: the two differ by the factor dt/dtau, which on
    # this curve is far from one.
    tau = st.time_to_tau(CLUSTERED, t)
    _, dp, _ = st.evaluate(CLUSTERED, tau)
    assert abs(dp[3] - 1.0) > 0.5
    assert norm(sub(v, dp[:3])) > 0.1 * norm(v)


@pytest.mark.parametrize("t", [0.3, 1.2, 2.0, 3.3, 5.0, 6.6, 8.8, 9.7])
def test_acceleration_is_the_time_derivative_of_velocity(t):
    h = 1e-4
    _, _, a = st.sample_at_time(CLUSTERED, t)
    _, v_plus, _ = st.sample_at_time(CLUSTERED, t + h)
    _, v_minus, _ = st.sample_at_time(CLUSTERED, t - h)
    fd = [(x - y) / (2 * h) for x, y in zip(v_plus, v_minus)]
    assert a == pytest.approx(fd, abs=1e-4), f"a {a} vs finite difference {fd}"


def test_dropping_the_second_time_derivative_would_be_caught():
    # The term this test exists for. On CLUSTERED, t'' is not small anywhere
    # interesting; the wrong formula p''/t'^2 differs from the right one by
    # more than the finite-difference tolerance above.
    t = 2.0
    tau = st.time_to_tau(CLUSTERED, t)
    _, dp, ddp = st.evaluate(CLUSTERED, tau)
    wrong = [ddp[i] / dp[3] ** 2 for i in range(3)]
    _, _, right = st.sample_at_time(CLUSTERED, t)
    assert norm(sub(wrong, right)) > 1e-2


def test_the_seed_is_constant_velocity_and_no_acceleration():
    for t in (0.0, 2.5, 7.0, 10.0):
        p, v, a = st.sample_at_time(SEED, t)
        assert v == pytest.approx([0.9, 0.0, 0.0], abs=1e-9)
        assert a == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)
        assert p == pytest.approx([0.5 + 0.9 * t, 5.0, 0.5], abs=1e-9)


# ---- sampling ---------------------------------------------------------------

def test_samples_are_uniform_in_time_not_in_tau():
    samples = st.sample_uniform_in_time(CLUSTERED, 0.02)
    times = [s[0] for s in samples]
    gaps = [b - a for a, b in zip(times, times[1:])]
    # Every interior gap is dt; the last one may be shorter (it lands on t1).
    assert all(g == pytest.approx(0.02, abs=1e-9) for g in gaps[:-1])
    assert 0.0 < gaps[-1] <= 0.02 + 1e-9
    assert times[0] == 0.0 and times[-1] == 10.0
    # A uniform-in-tau sampler would show time gaps that vary by a large
    # factor on this curve; measured here so the claim in the docstring is
    # tied to a number.
    taus = [i / 500 for i in range(501)]
    tau_times = [st.de_casteljau(CLUSTERED, tau)[3] for tau in taus]
    tau_gaps = [b - a for a, b in zip(tau_times, tau_times[1:])]
    assert max(tau_gaps) / min(tau_gaps) > 3.0


def test_samples_carry_matching_position_velocity_acceleration():
    samples = st.sample_uniform_in_time(CLUSTERED, 0.01)
    # Trapezoidal integration of the sampled velocity must recover the
    # displacement between endpoints: samples that mix positions from one
    # instant with velocities from another would not integrate back.
    disp = [0.0, 0.0, 0.0]
    for (t0, _, v0, _), (t1, _, v1, _) in zip(samples, samples[1:]):
        for k in range(3):
            disp[k] += 0.5 * (v0[k] + v1[k]) * (t1 - t0)
    assert disp == pytest.approx(sub(CLUSTERED[-1][:3], CLUSTERED[0][:3]), abs=2e-3)


def test_rejects_a_non_positive_sample_step():
    with pytest.raises(st.PlanError):
        st.sample_uniform_in_time(SEED, 0.0)


def test_summary_reports_the_seed_honestly():
    s = st.summarize(st.sample_uniform_in_time(SEED, 0.02))
    assert s["duration_s"] == pytest.approx(10.0)
    assert s["max_speed_mps"] == pytest.approx(0.9, abs=1e-9)
    assert s["max_accel_mps2"] == pytest.approx(0.0, abs=1e-9)
    assert s["max_tilt_deg"] == pytest.approx(0.0, abs=1e-9)
