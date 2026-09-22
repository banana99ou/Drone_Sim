"""The message the bridge builds. Needs dsim_msgs, so this runs in the
container (colcon test) and is skipped on the host."""
import math

import pytest

pytest.importorskip("dsim_msgs")

from dsim_planner.bridge_node import build_trajectory  # noqa: E402

SEED = [[0.5 + 1.125 * i, 5.0, 0.5, 1.25 * i] for i in range(9)]


def to_s(d):
    return d.sec + 1e-9 * d.nanosec


def test_stamp_is_the_absolute_start_time():
    msg, _ = build_trajectory(SEED, 10.25, 0.02, 0.0)
    assert to_s(msg.header.stamp) == pytest.approx(10.25, abs=1e-9)


def test_time_from_start_counts_from_the_plans_first_point():
    # A plan whose own clock starts at t=3 still has points[0] at offset 0:
    # the header stamp is where scenario time is anchored, not the plan's t0.
    shifted = [[x, y, z, t + 3.0] for x, y, z, t in SEED]
    msg, _ = build_trajectory(shifted, 10.0, 0.5, 0.0)
    offsets = [to_s(p.time_from_start) for p in msg.points]
    assert offsets[0] == 0.0
    assert offsets[-1] == pytest.approx(10.0)
    assert all(b - a == pytest.approx(0.5, abs=1e-9) for a, b in zip(offsets, offsets[1:]))


def test_nanoseconds_never_overflow_a_second():
    # 0.02 s steps accumulate float error; a rounding that produced
    # nanosec == 1e9 is a message the controller's binary search cannot sort.
    msg, _ = build_trajectory(SEED, 10.0, 0.02, 0.0)
    assert all(0 <= p.time_from_start.nanosec < 1_000_000_000 for p in msg.points)
    assert 0 <= msg.header.stamp.nanosec < 1_000_000_000


def test_points_carry_the_feedforward():
    msg, samples = build_trajectory(SEED, 10.0, 0.02, 0.3)
    assert len(msg.points) == len(samples) == 501
    assert msg.points[100].velocity.x == pytest.approx(0.9)
    assert msg.points[100].acceleration.x == pytest.approx(0.0, abs=1e-9)
    assert msg.points[100].yaw == 0.3
    assert msg.points[-1].position.x == pytest.approx(9.5)
