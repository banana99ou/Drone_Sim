#!/usr/bin/env python3
"""Measure the sensor noise in a running sim and compare it to the config.

Run INSIDE the container against a flying sim:

    python3 scripts/check_sensors.py            # 20 s of samples
    python3 scripts/check_sensors.py 40

Why this exists. config/drone.yaml says the gyro has a 1e-3 rad/s standard
deviation, and model.sdf tells Gazebo the same thing because both are generated
from one source. Neither file is evidence that the SIMULATOR is applying it. A
typo in the SDF path, an sdformat version that ignores the element, or a sensor
plugin that silently drops noise would leave both files looking perfect while
the data stayed clean -- and "we added sensor noise" would be a claim nobody had
checked.

So this measures the standard deviation of the live signal and fails if it does
not match. Getting a usable estimate needs the true value removed first, which
is why the drone is asked to HOVER: on a stationary vehicle the gyro should read
zero and the accelerometer should read one g, so any spread around those is
noise. On a moving vehicle the real motion would swamp it.

Exit code is the number of failed checks, so this works as a gate.
"""
import math
import statistics
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
CONFIG = "/ws/config/drone.yaml"

# A standard deviation estimated from a few thousand samples is itself noisy,
# and the vehicle is not perfectly still, so the band is wide on purpose: this
# is checking that noise of the right ORDER is present and configured, not
# calibrating an instrument. It would still catch zero noise, noise an order of
# magnitude out, or noise applied to the wrong quantity.
LOW, HIGH = 0.4, 2.5


class Probe(Node):
    def __init__(self):
        super().__init__("sensor_probe")
        self.gyro = [[], [], []]
        self.accel = [[], [], []]
        self.create_subscription(Imu, "/drone/imu", self.cb,
                                 qos_profile_sensor_data)

    def cb(self, m):
        g, a = m.angular_velocity, m.linear_acceleration
        for i, v in enumerate((g.x, g.y, g.z)):
            self.gyro[i].append(v)
        for i, v in enumerate((a.x, a.y, a.z)):
            self.accel[i].append(v)


def main():
    with open(CONFIG) as fh:
        cfg = yaml.safe_load(fh)["drone"]["sensors"]["imu"]

    rclpy.init()
    p = Probe()
    end = time.time() + SECONDS
    while time.time() < end:
        rclpy.spin_once(p, timeout_sec=0.2)

    n = len(p.gyro[0])
    if n < 100:
        print(f"FAIL: only {n} IMU samples in {SECONDS:.0f} s -- is the sim running?")
        return 1

    rate = n / SECONDS
    print(f"{n} IMU samples over {SECONDS:.0f} s ({rate:.0f} Hz, "
          f"config says {cfg['rate_hz']:.0f})")
    print()

    failures = 0

    def check(name, measured, expected):
        nonlocal failures
        ratio = measured / expected if expected else float("inf")
        ok = LOW <= ratio <= HIGH
        flag = "ok  " if ok else "FAIL"
        print(f"  {flag} {name:<28} measured {measured:.5f}   "
              f"configured {expected:.5f}   ratio {ratio:.2f}")
        if not ok:
            failures += 1

    # Per-axis standard deviation. On a hovering vehicle the true signal is
    # nearly constant, so the spread is the noise. Any real residual motion
    # only ever makes this LARGER, so a pass cannot be faked by a still drone.
    for i, axis in enumerate("xyz"):
        check(f"gyro {axis} stddev (rad/s)",
              statistics.stdev(p.gyro[i]), cfg["gyro_noise_rad_s"])
    for i, axis in enumerate("xyz"):
        check(f"accel {axis} stddev (m/s^2)",
              statistics.stdev(p.accel[i]), cfg["accel_noise_m_s2"])

    # A sanity anchor independent of the noise: a hovering accelerometer must
    # read one g upward. If it does not, the vehicle is not hovering and the
    # numbers above are measuring motion, not noise.
    mean_z = statistics.fmean(p.accel[2])
    print()
    print(f"  accel z mean {mean_z:.3f} m/s^2 (a hovering vehicle reads +9.81)")
    if not 8.5 < mean_z < 11.0:
        print("  FAIL: not hovering, so the spread above is motion, not noise")
        failures += 1

    # ...and the noise must not be so large that the controller is flying on
    # garbage. The rotational loop reads this gyro directly.
    worst_gyro = max(statistics.stdev(p.gyro[i]) for i in range(3))
    if worst_gyro > 0.05:
        print(f"  FAIL: gyro noise {worst_gyro:.4f} rad/s is large enough to "
              f"degrade the rotational loop")
        failures += 1

    print()
    print("SENSORS OK: configured noise is present in the live data"
          if failures == 0 else f"SENSORS FAILED: {failures} check(s)")
    return failures


if __name__ == "__main__":
    sys.exit(main())
