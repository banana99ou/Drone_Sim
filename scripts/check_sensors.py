#!/usr/bin/env python3
"""Check the simulated sensors against their config and against ground truth.

Run INSIDE the container against a running sim:

    python3 scripts/check_sensors.py            # 20 s of samples
    python3 scripts/check_sensors.py 40

Why this exists: config/drone.yaml and model.sdf agree about the IMU noise
because both are generated from one source. Neither is evidence that the
SIMULATOR applies it. A typo in the SDF path, an sdformat version that ignored
the element, or a plugin that dropped the noise would leave both files looking
perfect while the data stayed clean.

The noise checks run in EVERY regime, not just hover. They estimate the
standard deviation from consecutive-sample DIFFERENCES: for white noise
std(diff) = sigma * sqrt(2), while the vehicle's real motion is smooth at
250 Hz and contributes almost nothing between adjacent samples. An earlier
version of this script only ran them when the vehicle was nearly still, which
silently removed the IMU gate from `make fly` and `make sensors` -- both of
which fly a circle. A check that quietly does nothing is worse than no check.

The ground-truth cross-checks need motion to have any signal, so those are
skipped -- loudly -- when the vehicle is hovering.

Exit code is the number of failed checks, so this works as a gate.
"""
import math
import statistics
import sys
import time

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Range

from dsim_msgs.msg import OpticalFlow

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
CONFIG = "/ws/config/drone.yaml"

# Below this the ground-truth cross-checks have no signal to work with.
MOVING_SPEED = 0.40

# A standard deviation from a few thousand differences is itself noisy, and
# real motion leaks in a little, so the band is wide. This checks that noise of
# the right ORDER is present, not that an instrument is calibrated -- and it
# still catches zero noise, an order-of-magnitude error, or noise applied to
# the wrong quantity. Verified: zeroing the stddev fields in the SDF makes
# every one of these fail at ratio ~0.01.
SIGMA_LOW, SIGMA_HIGH = 0.4, 2.5


def yaw_free_cos_tilt(q):
    """World-z component of the body z axis, from a quaternion."""
    return 1.0 - 2.0 * (q.x * q.x + q.y * q.y)


def sigma_from_differences(series):
    """Estimate white-noise sigma from consecutive differences.

    For independent samples, var(x[n] - x[n-1]) = 2 * var(noise), so
    sigma = std(diff) / sqrt(2). Any smooth real signal underneath contributes
    only its change over one sample interval, which at 250 Hz is negligible
    next to the sensor noise -- and whatever it does contribute makes the
    estimate LARGER, so this cannot fake a pass on a clean sensor.
    """
    if len(series) < 3:
        return 0.0
    diffs = [b - a for a, b in zip(series, series[1:])]
    return statistics.stdev(diffs) / math.sqrt(2.0)


class Probe(Node):
    def __init__(self):
        super().__init__("sensor_probe")
        self.gyro = [[], [], []]
        self.accel = [[], [], []]
        self.speeds = []
        self.truth = None
        self.tof = []      # (measured, altitude, cos_tilt)
        self.flow = []     # (vx, vy, true_vx, true_vy, quality, range)
        self.create_subscription(Imu, "/drone/imu", self.on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "/drone/truth", self.on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(Range, "/drone/tof", self.on_tof,
                                 qos_profile_sensor_data)
        self.create_subscription(OpticalFlow, "/drone/optical_flow",
                                 self.on_flow, qos_profile_sensor_data)

    def on_imu(self, m):
        g, a = m.angular_velocity, m.linear_acceleration
        for i, v in enumerate((g.x, g.y, g.z)):
            self.gyro[i].append(v)
        for i, v in enumerate((a.x, a.y, a.z)):
            self.accel[i].append(v)

    def on_truth(self, m):
        q = m.pose.pose.orientation
        v = m.twist.twist.linear          # body frame, REP-145
        self.truth = (m.pose.pose.position.z, yaw_free_cos_tilt(q),
                      (v.x, v.y, v.z))
        self.speeds.append(math.dist((v.x, v.y, v.z), (0, 0, 0)))

    def on_tof(self, m):
        if self.truth and math.isfinite(m.range):
            self.tof.append((m.range, self.truth[0], self.truth[1]))

    def on_flow(self, m):
        if not self.truth or m.quality == 0 or m.integration_time_s <= 0:
            return
        if not math.isfinite(m.ground_distance_m):
            return
        # Exactly the compensation OpticalFlow.msg tells a consumer to do.
        h, dt = m.ground_distance_m, m.integration_time_s
        vx = (m.integrated_y - m.integrated_ygyro) / dt * h
        vy = -(m.integrated_x - m.integrated_xgyro) / dt * h
        tv = self.truth[2]
        self.flow.append((vx, vy, tv[0], tv[1], m.quality, h))


class Checks:
    def __init__(self):
        self.failures = 0

    def ok(self, name, detail):
        print(f"  ok   {name:<36} {detail}")

    def bad(self, name, detail, would_catch):
        print(f"  FAIL {name:<36} {detail}")
        print(f"       would catch: {would_catch}")
        self.failures += 1

    def within(self, name, measured, limit, detail, would_catch):
        if measured <= limit:
            self.ok(name, detail)
        else:
            self.bad(name, detail, would_catch)

    def ratio(self, name, measured, expected, would_catch):
        r = measured / expected if expected else float("inf")
        detail = f"measured {measured:.5f}  configured {expected:.5f}  ratio {r:.2f}"
        if SIGMA_LOW <= r <= SIGMA_HIGH:
            self.ok(name, detail)
        else:
            self.bad(name, detail, would_catch)


def check_imu_noise(p, cfg, c):
    """Runs in every regime -- see the note on sigma_from_differences()."""
    imu = cfg["imu"]
    for i, axis in enumerate("xyz"):
        c.ratio(f"gyro {axis} noise (rad/s)",
                sigma_from_differences(p.gyro[i]), imu["gyro_noise_rad_s"],
                "noise configured but not applied by the simulator")
    for i, axis in enumerate("xyz"):
        c.ratio(f"accel {axis} noise (m/s^2)",
                sigma_from_differences(p.accel[i]), imu["accel_noise_m_s2"],
                "noise configured but not applied by the simulator")

    worst = max(sigma_from_differences(p.gyro[i]) for i in range(3))
    c.within("gyro noise is loop-safe", worst, 0.05, f"{worst:.4f} rad/s",
             "noise large enough to degrade the rotational loop, which reads "
             "this sensor directly")


def check_rangefinder(p, cfg, c):
    """The rangefinder must report SLANT range, and its error must fit budget.

    Two separate claims, because the interesting one is easy to miss. A sensor
    that returned plain altitude passes an error-budget test at every
    configuration this repo ships: at 12.6 degrees of bank the slant/altitude
    difference is 36 mm against a 3-sigma budget of 151 mm. So the slant
    relationship is asserted directly, as a correlation across the window that
    grows with tilt, rather than hidden inside a tolerance.
    """
    if not p.tof:
        c.bad("rangefinder has returns", "no finite ranges received",
              "a sensor that never publishes, or is stuck out of range")
        return

    tof = cfg["tof"]
    usable = [(r, alt, ct) for r, alt, ct in p.tof if ct > 0.1 and alt > 0.05]
    if not usable:
        c.bad("rangefinder has usable samples", f"{len(p.tof)} received, none usable",
              "a vehicle on the ground, or attitudes past the beam's reach")
        return

    # Error budget evaluated at the range actually OBSERVED, not at max_range.
    # Using max_range inflated the budget to 6 sigma at the flown altitude.
    errors = [(r - alt / ct, alt, ct, r) for r, alt, ct in usable]
    worst, w_alt, w_ct, w_r = max(errors, key=lambda e: abs(e[0]))
    sigma_at = tof["noise_m"] + tof["noise_frac"] * w_r
    # E[max] over a few hundred samples is already past 3 sigma, so 4 is the
    # honest threshold for a worst-of-N statistic, plus one quantisation step.
    budget = 4.0 * sigma_at + tof["resolution_m"]
    c.within("range error fits the configured budget", abs(worst), budget,
             f"worst {worst:+.4f} m at range {w_r:.2f} m "
             f"(budget {budget:.4f} = 4 sigma of {sigma_at:.4f})",
             "an error model larger than configured, or a sensor reporting "
             "something other than range")

    # The slant relationship itself. Compare the measured excess over altitude
    # with the geometric prediction alt*(sec(tilt) - 1) on the most tilted
    # samples in the window; a sensor reporting altitude gives ~0 excess.
    tilted = sorted(usable, key=lambda e: e[2])[:max(20, len(usable) // 10)]
    predicted = statistics.fmean(alt * (1.0 / ct - 1.0) for _, alt, ct in tilted)
    measured = statistics.fmean(r - alt for r, alt, _ in tilted)
    worst_tilt = math.degrees(math.acos(min(1.0, tilted[-1][2])))
    if predicted < 0.010:
        # Not enough bank in this window for the test to mean anything. Say so
        # rather than printing a pass: this is exactly the case where the old
        # version claimed to be checking something it could not see.
        print(f"  skip slant-range shape                  only {worst_tilt:.1f} deg "
              f"of bank here (predicts {predicted * 1000:.1f} mm); fly a "
              f"shorter period to exercise it")
    else:
        c.within("range follows altitude/cos(tilt)",
                 abs(measured - predicted), 0.5 * predicted + tof["noise_m"],
                 f"excess {measured * 1000:.1f} mm vs predicted "
                 f"{predicted * 1000:.1f} mm at up to {worst_tilt:.1f} deg",
                 "a sensor reporting altitude instead of slant range, which "
                 "would give an excess of ~0")


def check_optical_flow(p, cfg, c, speed):
    """The flow must reconstruct the true velocity, and quality must be real.

    Tolerance is ABSOLUTE, derived from the error model: the reconstruction
    error is noise_rad_s * height * N(0,1), which does not depend on speed.
    Scaling the tolerance with speed (as an earlier version did) made the gate
    26x looser at the fastest shipped configuration than the physics warrants.
    """
    if not p.flow:
        c.bad("optical flow is usable", "no readings with quality > 0",
              "a flow sensor stuck at quality 0, so nothing downstream could "
              "ever use it")
        return

    flow_cfg = cfg["optical_flow"]
    heights = [h for *_, h in p.flow]
    # One-sigma of the reconstruction, from the documented error model.
    sigma = flow_cfg["noise_rad_s"] * statistics.fmean(heights)
    # Median error of a zero-mean normal is 0.674 sigma; allow 3x that for the
    # estimator's own spread. This also transitively checks the RANGE, because
    # a wrong ground_distance_m now scales the reconstruction proportionally.
    limit = 3.0 * 0.674 * sigma

    errs_x = statistics.median([abs(vx - tx) for vx, _, tx, _, _, _ in p.flow])
    errs_y = statistics.median([abs(vy - ty) for _, vy, _, ty, _, _ in p.flow])
    for name, err in (("body x", errs_x), ("body y", errs_y)):
        c.within(f"flow reconstructs velocity ({name})", err, limit,
                 f"median error {err:.4f} m/s at {speed:.2f} m/s "
                 f"(limit {limit:.4f} from noise_rad_s x height)",
                 "a flipped sign, a missing division by height, rotation "
                 "compensation that does not cancel, or a wrong reported range")

    # Quality must actually be good in the middle of the envelope, not merely
    # non-zero -- on_flow() already discards zeros, so 'min > 0' would be
    # guaranteed by the filter and could never fail.
    worst_q = min(q for *_, q, _ in p.flow)
    median_q = statistics.median([q for *_, q, _ in p.flow])
    if median_q >= 128:
        c.ok("flow quality in normal flight",
             f"median {median_q:.0f}/255, lowest {worst_q}/255")
    else:
        c.bad("flow quality in normal flight",
              f"median {median_q:.0f}/255, lowest {worst_q}/255",
              "a quality model that degrades in ordinary flight, so a "
              "consumer would reject usable readings")


def main():
    with open(CONFIG) as fh:
        cfg = yaml.safe_load(fh)["drone"]["sensors"]

    rclpy.init()
    p = Probe()
    end = time.time() + SECONDS
    while time.time() < end:
        rclpy.spin_once(p, timeout_sec=0.2)

    n = len(p.gyro[0])
    if n < 100 or p.truth is None:
        print(f"FAIL: {n} IMU samples and "
              f"{'no' if p.truth is None else 'some'} ground truth in "
              f"{SECONDS:.0f} s -- is the sim running?")
        return 1

    speed = statistics.fmean(p.speeds) if p.speeds else 0.0
    print(f"{n} IMU samples over {SECONDS:.0f} s ({n / SECONDS:.0f} Hz), "
          f"{len(p.tof)} range, {len(p.flow)} usable flow")
    print(f"mean speed {speed:.2f} m/s")
    print()

    c = Checks()
    check_imu_noise(p, cfg, c)
    print()

    if speed >= MOVING_SPEED:
        check_rangefinder(p, cfg, c)
        check_optical_flow(p, cfg, c, speed)
    else:
        # Loud, not silent: the reader must know these did not run.
        print(f"  skip rangefinder and flow cross-checks  vehicle at "
              f"{speed:.2f} m/s, need {MOVING_SPEED} m/s for signal")
        print("       (launch with reference:=circle to exercise them)")

    print()
    print("SENSORS OK" if c.failures == 0
          else f"SENSORS FAILED: {c.failures} check(s)")
    return c.failures


if __name__ == "__main__":
    sys.exit(main())
