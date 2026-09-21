#!/usr/bin/env python3
"""Compare the state estimate against ground truth on a running sim.

Run INSIDE the container (make estimator) against a sim that is up:

    python3 scripts/check_estimator.py            # 20 s of samples
    python3 scripts/check_estimator.py 40

It pairs every /drone/state_est message with the /drone/truth message of the
same stamp and measures the attitude and velocity error, then reads
/drone/estimator_debug to see which sensors were actually fused. Bounds are
DERIVED from config/drone.yaml (the sensor noise) and estimator.yaml (the
gains), with the derivation printed next to each number, and they widen in
a turn by exactly the banked-turn error the attitude filter is documented to
have -- measured, not assumed, from the truth's own tilt and yaw rate.

WHAT WOULD MAKE EACH CHECK FAIL, because a check that cannot fail is not
evidence:

  tilt error RMS within bound   an accelerometer correction with the wrong
                                sign (diverges), gravity or the rotation
                                applied the wrong way, a filter that ignores
                                the gyro (in a turn the error is the whole
                                bank angle), or kp_accel raised until a turn
                                corrupts the attitude
  yaw error RMS / mean          a mag heading read without tilt compensation
                                (tens of degrees when banked), the wrong
                                field, a hard iron nobody subtracted, a yaw
                                that integrates the gyro and ignores the mag
  velocity error RMS per axis   a flipped flow sign, the gyro term dropped
                                from the flow, a body-frame innovation added
                                to the world state, the slant range used as
                                altitude (a phantom climb rate in every turn),
                                the gravity sign wrong in the integration
  position equals truth         a position source other than truth switched
                                on without anyone deciding to
  publish rate ~ IMU rate       an estimator that publishes on the wrong
                                trigger, or stalls waiting for a sensor
  errors are NOT zero           an estimator that copies truth would pass
                                every bound above. The floors here sit above
                                what stamp skew alone could produce.
  yaw responds to each mag sample   the filter equation itself: the CHANGE in
                                yaw error across one mag interval must be
                                correlated with that sample's heading
                                innovation, reconstructed here from /drone/mag
                                and the published attitude, independently of
                                the estimator's own bookkeeping. A truth copy
                                has zero error and zero change (nothing to
                                correlate); a gyro-only yaw changes by the
                                bias, uncorrelated with the mag. Doubling the
                                mag noise or removing the mag changes the yaw
                                error -- that is the point.
  velocity responds to each flow sample  same signature with
                                /drone/optical_flow: the change in velocity
                                error over one flow interval against the flow
                                innovation. Holds in a turn too, because the
                                banked-turn leak is exactly what the flow is
                                then correcting.

Exit code is the number of failed checks, so this works as a gate.
"""
import bisect
import math
import statistics
import sys
import time

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField

from dsim_msgs.msg import EstimatorDebug, OpticalFlow

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
DRONE_CONFIG = "/ws/config/drone.yaml"
ESTIMATOR_CONFIG = "/ws/src/dsim_estimation/config/estimator.yaml"

# Pairing tolerance: truth and IMU are both 250 Hz on the same clock, so a
# state_est stamp should have a truth stamp within one physics step.
PAIR_TOL_S = 0.0025
# Margin on the derived bounds, for what the derivation leaves out: the
# controller reacting to the estimate's own noise (which moves the vehicle,
# which the sensors then see), and the fact that 20 s of a slow filter is a
# few independent samples. Stated so it can be argued with; it is not large
# enough to admit any of the failures listed above (each is 10x or more).
MARGIN = 3.0
# Below this speed the vehicle is hovering; only changes which bounds apply.
HOVER_SPEED = 0.3
# The innovation-response correlations are near 1 for a filter that applies
# its corrections (the correction is 5-10x the noise accumulated over one
# sensor interval); 0.5 leaves room for a hard turn and still refuses a
# copied truth (NaN) or an ignored sensor (~0).
MIN_CORRELATION = 0.5
# Absolute caps on the derived bounds. The derivations scale with the gains,
# so a wrong GAIN would inflate its own bound and pass; these do not move.
# They are the errors at which the estimate alone costs the controller more
# position offset than its nominal tracking error (3-4 cm on the demo laps):
#   tilt      g * dtheta / kp_pos = 9.81 * 0.035 / 6  ~ 6 cm  ->  2 deg
#   velocity  kv * dv / kp_pos    = 4 * 0.08 / 6      ~ 5 cm  ->  0.08 m/s
#   yaw       no position cost at all in a hover; 3 deg is where the heading
#             error becomes visible in the viewer's body axis overlay
TILT_CAP = math.radians(2.0)
YAW_CAP = math.radians(3.0)
VEL_CAP = 0.08


# ---- quaternion helpers (w, x, y, z), body->world ---------------------------

def q_tuple(q):
    return (q.w, q.x, q.y, q.z)


def rotate(q, v):
    """q * v * q^-1: a body vector expressed in the world."""
    w, x, y, z = q
    vx, vy, vz = v
    tx, ty, tz = 2.0 * (y * vz - z * vy), 2.0 * (z * vx - x * vz), 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def rotate_inv(q, v):
    return rotate((q[0], -q[1], -q[2], -q[3]), v)


def yaw_of(q):
    """Heading of the body x axis: the controller's definition of yaw."""
    bx = rotate(q, (1.0, 0.0, 0.0))
    return math.atan2(bx[1], bx[0])


def body_up(q):
    """Where this attitude puts 'up' in the body frame: the accelerometer's
    observable, a function of roll and pitch alone."""
    return rotate_inv(q, (0.0, 0.0, 1.0))


def tilt_between(qa, qb):
    ua, ub = body_up(qa), body_up(qb)
    return math.acos(max(-1.0, min(1.0, sum(a * b for a, b in zip(ua, ub)))))


def tilt_of(q):
    """Angle between body z and world z."""
    return math.acos(max(-1.0, min(1.0, rotate(q, (0.0, 0.0, 1.0))[2])))


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def heading_from_mag(q, b, field_world):
    """Tilt-compensated heading error: how far the attitude q must turn about
    world z for the body field b to line up with the world field. Exactly
    what the estimator does, but with an attitude of the caller's choosing."""
    m = rotate(q, b)
    hm = math.hypot(m[0], m[1])
    hr = math.hypot(field_world[0], field_world[1])
    if hm < 1e-15 or hr < 1e-15:
        return float("nan")
    cross = m[0] * field_world[1] - m[1] * field_world[0]
    dot = m[0] * field_world[0] + m[1] * field_world[1]
    return math.atan2(cross, dot)


def rms(xs):
    return math.sqrt(statistics.fmean(x * x for x in xs)) if xs else float("nan")


def correlation(xs, ys):
    """Pearson correlation; NaN when either series is constant (a copied
    truth has zero error, and zero error correlates with nothing)."""
    if len(xs) < 10 or len(xs) != len(ys):
        return float("nan")
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    if sxx < 1e-30 or syy < 1e-30:
        return float("nan")
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / math.sqrt(sxx * syy)


def stamp_of(h):
    return h.stamp.sec + 1e-9 * h.stamp.nanosec


# ---- the probe ---------------------------------------------------------------

class Probe(Node):
    def __init__(self, field_world):
        super().__init__("estimator_probe")
        self.field_world = field_world
        self.truth_stamps = []       # sorted, for pairing
        self.truth = {}              # stamp -> (q, p, v_world)
        self.est = []                # (stamp, q, p, v_world, wall_recv, truth_latest_stamp)
        self.debug = []              # EstimatorDebug messages
        self.imu_count = 0
        self.imu_first = None
        self.imu_last = None
        self.mag_count = 0
        self.mag_events = []         # (stamp, heading innovation as the estimator sees it)
        self.flow_events = []        # (stamp, (dvx, dvy)) world-frame flow innovation
        self.latest_est_q = None
        self.latest_est_v = None     # world frame
        self.wall_start = time.time()
        qos = qos_profile_sensor_data
        self.create_subscription(Odometry, "/drone/truth", self.on_truth, qos)
        self.create_subscription(Odometry, "/drone/state_est", self.on_est, qos)
        self.create_subscription(EstimatorDebug, "/drone/estimator_debug", self.on_debug, qos)
        self.create_subscription(Imu, "/drone/imu", self.on_imu, qos)
        self.create_subscription(MagneticField, "/drone/mag", self.on_mag, qos)
        self.create_subscription(OpticalFlow, "/drone/optical_flow", self.on_flow, qos)

    def nearest_truth(self, t):
        i = bisect.bisect_left(self.truth_stamps, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(self.truth_stamps):
                s = self.truth_stamps[j]
                if best is None or abs(s - t) < abs(best - t):
                    best = s
        if best is None or abs(best - t) > PAIR_TOL_S:
            return None, None
        return best, self.truth[best]

    def on_truth(self, m):
        t = stamp_of(m.header)
        q = q_tuple(m.pose.pose.orientation)
        p = (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)
        vb = (m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z)
        if t not in self.truth:
            bisect.insort(self.truth_stamps, t)
        self.truth[t] = (q, p, rotate(q, vb))

    def on_est(self, m):
        t = stamp_of(m.header)
        q = q_tuple(m.pose.pose.orientation)
        p = (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)
        vb = (m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z)
        latest = self.truth_stamps[-1] if self.truth_stamps else float("nan")
        v_world = rotate(q, vb)
        self.est.append((t, q, p, v_world, time.time(), latest))
        self.latest_est_q = q
        self.latest_est_v = v_world

    def on_debug(self, m):
        self.debug.append(m)

    def on_imu(self, m):
        self.imu_count += 1
        t = stamp_of(m.header)
        if self.imu_first is None:
            self.imu_first = t
        self.imu_last = t

    def on_mag(self, m):
        # The heading innovation exactly as the estimator computes it -- the
        # sample levelled with the PUBLISHED attitude, against the world
        # field -- from the raw topic, so it does not rely on anything the
        # estimator says about itself. The yaw error must change in
        # proportion to this over the next mag interval.
        self.mag_count += 1
        if self.latest_est_q is None:
            return
        b = (m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z)
        err = heading_from_mag(self.latest_est_q, b, self.field_world)
        if math.isfinite(err):
            self.mag_events.append((stamp_of(m.header), err))

    def on_flow(self, m):
        # The flow innovation as the estimator computes it: the reading's
        # body x/y velocity against the published estimate's, rotated into
        # the world with the published attitude.
        if (m.quality == 0 or m.integration_time_s <= 0
                or not math.isfinite(m.ground_distance_m) or self.latest_est_q is None):
            return
        h, dt = m.ground_distance_m, m.integration_time_s
        vx = (m.integrated_y - m.integrated_ygyro) / dt * h
        vy = -(m.integrated_x - m.integrated_xgyro) / dt * h
        q = self.latest_est_q
        vb_est = rotate_inv(q, self.latest_est_v)
        innovation_world = rotate(q, (vx - vb_est[0], vy - vb_est[1], 0.0))
        self.flow_events.append((stamp_of(m.header), (innovation_world[0], innovation_world[1])))


class Checks:
    def __init__(self):
        self.failures = 0

    def ok(self, name, detail):
        print(f"  ok   {name:<40} {detail}")

    def bad(self, name, detail, would_catch):
        print(f"  FAIL {name:<40} {detail}")
        print(f"       would catch: {would_catch}")
        self.failures += 1

    def within(self, name, measured, limit, detail, would_catch):
        if math.isfinite(measured) and measured <= limit:
            self.ok(name, detail)
        else:
            self.bad(name, detail, would_catch)

    def above(self, name, measured, floor, detail, would_catch):
        if math.isfinite(measured) and measured >= floor:
            self.ok(name, detail)
        else:
            self.bad(name, detail, would_catch)


def series_at(stamps, values, t):
    """Value of a sorted (stamps, values) series at or just before t."""
    i = bisect.bisect_right(stamps, t) - 1
    return values[i] if i >= 0 else None


def response(events, stamps, errors, pick):
    """For consecutive sensor events (t_k, innovation_k): pairs of
    (innovation_k, error(t_k+1) - error(t_k)). A filter that applies the
    correction makes the second proportional to the first."""
    xs, ys = [], []
    for (t0, inn), (t1, _) in zip(events, events[1:]):
        # Strictly BEFORE each sensor stamp: the correction lands when the
        # sample arrives, and the estimate published with the same stamp may
        # already contain it (callback order is not defined), which would put
        # the response in the wrong interval. Measured: -0.05 correlation with
        # "at or before", 0.9+ with "strictly before".
        e0 = series_at(stamps, errors, t0 - 1e-4)
        e1 = series_at(stamps, errors, t1 - 1e-4)
        if e0 is None or e1 is None or t1 - t0 > 0.1:
            continue
        for x, y in pick(inn, e0, e1):
            xs.append(x)
            ys.append(y)
    return xs, ys


def main():
    with open(DRONE_CONFIG) as fh:
        sensors = yaml.safe_load(fh)["drone"]["sensors"]
    with open(ESTIMATOR_CONFIG) as fh:
        est_cfg = yaml.safe_load(fh)["estimator"]
    g = 9.80665
    imu, tof, flow = sensors["imu"], sensors["tof"], sensors["optical_flow"]
    mag = sensors.get("magnetometer")
    att, vel = est_cfg["attitude"], est_cfg["velocity"]
    field = [float(v) for v in mag["field_world_t"]] if mag else [1.0, 0.0, 0.0]

    rclpy.init()
    p = Probe(field)
    end = time.time() + SECONDS
    while time.time() < end:
        rclpy.spin_once(p, timeout_sec=0.2)
    wall = time.time() - p.wall_start

    c = Checks()
    if p.imu_count < 100 or not p.truth or not p.est:
        print(f"FAIL: {p.imu_count} IMU, {len(p.truth)} truth, {len(p.est)} state_est "
              f"messages in {SECONDS:.0f} s -- is the sim running with sensors:=true?")
        return 1

    # ---- pair every estimate with the truth of the same stamp -------------
    pairs, skews, unpaired = [], [], 0
    for t, q, pos, v, _, _ in p.est:
        s, truth = p.nearest_truth(t)
        if truth is None:
            unpaired += 1
            continue
        skews.append(abs(s - t))
        pairs.append((t, q, pos, v, truth))
    if len(pairs) < 100:
        print(f"FAIL: only {len(pairs)} of {len(p.est)} estimates could be paired with a "
              f"truth stamp within {PAIR_TOL_S * 1e3:.1f} ms")
        return 1
    skew_max = max(skews)

    est_stamps = [t for t, *_ in pairs]
    tilt_err = [tilt_between(q, tr[0]) for _, q, _, _, tr in pairs]
    yaw_err = [wrap_pi(yaw_of(q) - yaw_of(tr[0])) for _, q, _, _, tr in pairs]
    v_err = [[v[i] - tr[2][i] for i in range(3)] for _, _, _, v, tr in pairs]
    pos_err = [math.dist(pos, tr[1]) for _, _, pos, _, tr in pairs]
    speeds = [math.dist(tr[2], (0, 0, 0)) for *_, tr in pairs]
    speed = statistics.fmean(speeds)
    heights = [tr[1][2] for *_, tr in pairs]
    height = statistics.fmean(heights)
    tilt_true = statistics.fmean(tilt_of(tr[0]) for *_, tr in pairs)
    # Yaw rate from the truth, for the banked-turn term: median absolute
    # yaw difference per second over the paired samples.
    yaws = [(t, yaw_of(tr[0])) for t, *_, tr in pairs]
    yaw_rates = [abs(wrap_pi(y1 - y0)) / (t1 - t0)
                 for (t0, y0), (t1, y1) in zip(yaws, yaws[1:]) if t1 > t0]
    yaw_rate = statistics.median(yaw_rates) if yaw_rates else 0.0
    body_rates = [math.dist(rotate_inv(tr[0], (0, 0, yaw_rate)), (0, 0, 0)) for *_, tr in pairs[:1]]
    body_rate = body_rates[0] if body_rates else 0.0
    hovering = speed < HOVER_SPEED

    span = pairs[-1][0] - pairs[0][0]
    print(f"{len(p.est)} state_est ({len(pairs)} paired, {unpaired} unpaired, "
          f"stamp skew <= {skew_max * 1e3:.2f} ms), {p.imu_count} IMU, "
          f"{len(p.debug)} debug, {p.mag_count} mag over {span:.1f} s sim / {wall:.1f} s wall")
    print(f"regime: {'HOVER' if hovering else 'MOVING'}  speed {speed:.2f} m/s, height {height:.2f} m, "
          f"tilt {math.degrees(tilt_true):.1f} deg, yaw rate {yaw_rate:.2f} rad/s")
    d = p.debug[-1]
    print(f"estimator: initialised={d.initialised} mag_configured={d.mag_configured} "
          f"mag_seen={d.mag_seen} flow_rejected={d.flow_rejected} tof_rejected={d.tof_rejected} "
          f"epochs={d.epochs} gyro_bias=[{d.gyro_bias_rad_s.x:+.1e} {d.gyro_bias_rad_s.y:+.1e} "
          f"{d.gyro_bias_rad_s.z:+.1e}] rad/s")
    frac = {k: statistics.fmean(getattr(m, k) for m in p.debug) for k in
            ("accel_correction_applied", "mag_correction_applied",
             "flow_correction_applied", "tof_correction_applied")}
    print("fused on: " + ", ".join(f"{k.split('_')[0]} {v * 100:.0f}%" for k, v in frac.items())
          + "  (of debug steps)")
    print()

    # ---- derived bounds ------------------------------------------------------
    kp, kp_mag = float(att["kp_accel"]), float(att["kp_mag"])
    dt_imu = 1.0 / float(imu["rate_hz"])
    # Tilt: accel bias is a DC error the filter converges to; gyro bias is a
    # DC error the proportional term holds at bias/kp; the noise terms are
    # filtered by the crossover; the turn term is the documented banked-turn
    # error with its transient allowance (x2), from the truth's tilt and rate.
    tilt_dc = imu["accel_bias_m_s2"] / g + imu["gyro_bias_rad_s"] / kp
    tilt_noise = (imu["accel_noise_m_s2"] / g * math.sqrt(kp * dt_imu / 2)
                  + imu["gyro_noise_rad_s"] * math.sqrt(dt_imu / (2 * kp)))
    tilt_turn = kp * math.sin(tilt_true) / math.hypot(kp, yaw_rate)
    tilt_bound = min(TILT_CAP, MARGIN * (tilt_dc + 3 * tilt_noise) + 2 * tilt_turn)
    c.within("tilt error RMS (roll/pitch)", rms(tilt_err), tilt_bound,
             f"{math.degrees(rms(tilt_err)):.3f} deg, bound {math.degrees(tilt_bound):.3f} "
             f"= min(cap {math.degrees(TILT_CAP):.0f}, {MARGIN:.0f}x({math.degrees(tilt_dc):.3f} dc + "
             f"3x{math.degrees(tilt_noise):.4f} noise) + 2x{math.degrees(tilt_turn):.3f} turn)",
             "an accel correction with the wrong sign or axis, gravity/rotation the wrong "
             "way round, a gyro that is not integrated, or kp_accel high enough to corrupt a turn")
    # Not a copy of truth: above what stamp skew alone could produce.
    tilt_floor = 3 * body_rate * skew_max + 1e-4
    c.above("tilt error is not zero", rms(tilt_err), tilt_floor,
            f"{math.degrees(rms(tilt_err)):.4f} deg >= floor {math.degrees(tilt_floor):.4f} "
            f"(3 x rate x skew + 1e-4 rad)",
            "an estimator that publishes the truth attitude")

    # Yaw: mag noise through the crossover, the gyro-z bias held by kp_mag
    # until ki takes it, hard iron as a heading offset, and in a turn the
    # tilt error leaking through the tilt compensation (vertical field over
    # horizontal), attenuated by the crossover against the lap rate.
    if mag and d.mag_configured and frac["mag_correction_applied"] > 0.5:
        b_h = math.hypot(field[0], field[1])
        sig_h = float(mag["noise_t"]) / b_h
        dt_mag = 1.0 / float(mag["rate_hz"])
        yaw_noise = sig_h * math.sqrt(kp_mag * dt_mag / 2)
        # The DC tilt error leaks into the heading through the tilt
        # compensation, scaled by how vertical the field is: measured live
        # at -0.1 deg on a hover before this term existed.
        yaw_dc = (imu["gyro_bias_rad_s"] / kp_mag + float(mag["hard_iron_t"]) / b_h
                  + tilt_dc * abs(field[2]) / b_h)
        yaw_turn = tilt_turn * abs(field[2]) / b_h * kp_mag / math.hypot(kp_mag, yaw_rate)
        yaw_bound = min(YAW_CAP, MARGIN * (yaw_dc + 3 * yaw_noise) + 2 * yaw_turn)
        c.within("yaw error RMS", rms(yaw_err), yaw_bound,
                 f"{math.degrees(rms(yaw_err)):.3f} deg, bound {math.degrees(yaw_bound):.3f} "
                 f"= min(cap {math.degrees(YAW_CAP):.0f}, {MARGIN:.0f}x({math.degrees(yaw_dc):.3f} dc incl. tilt leak + "
                 f"3x{math.degrees(yaw_noise):.4f} noise) + 2x{math.degrees(yaw_turn):.3f} turn leak)",
                 "a heading read without tilt compensation, the wrong world field, a hard iron, "
                 "or a yaw that ignores the mag")
        c.within("yaw error mean", abs(statistics.fmean(yaw_err)), yaw_bound,
                 f"{math.degrees(statistics.fmean(yaw_err)):+.3f} deg, bound {math.degrees(yaw_bound):.3f}",
                 "a constant heading offset: hard iron, or a field the sensor and the estimator "
                 "disagree about")
        # The filter equation: over one mag interval the yaw error must move
        # by ~kp_mag * T * innovation. Correlate the two.
        xs, ys = response(p.mag_events, est_stamps, yaw_err,
                          lambda inn, e0, e1: [(inn, wrap_pi(e1 - e0))])
        corr = correlation(xs, ys)
        c.above("yaw responds to each mag sample", corr, MIN_CORRELATION,
                f"corr(innovation, change in yaw error) {corr:.2f} over {len(xs)} intervals "
                f"(min {MIN_CORRELATION})",
                "a yaw copied from truth (zero error, nothing to correlate) or integrated from "
                "the gyro alone (changes by the bias, uncorrelated with the mag)")
    else:
        why = ("no magnetometer block in config/drone.yaml" if not mag else
               "estimator reports mag not configured" if not d.mag_configured else
               f"mag fused on only {frac['mag_correction_applied'] * 100:.0f}% of steps")
        print(f"  skip yaw bounds                         {why}; yaw is gyro-only")
        # Gyro-only yaw drifts. Say what it did, and assert the drift is the
        # gyro's and not something worse.
        drift = (yaw_err[-1] - yaw_err[0]) / span if span > 0 else float("nan")
        # The gyro bias is drawn at up to a few sigma of the configured mean;
        # 5x covers that. In a turn the accelerometer correction has a
        # world-z component of order kp * sin(tilt)^2, which leaks into yaw.
        drift_bound = 5 * imu["gyro_bias_rad_s"] + kp * math.sin(tilt_true) ** 2
        c.within("gyro-only yaw drift rate", abs(drift), drift_bound,
                 f"{math.degrees(drift) * 60:+.2f} deg/min (mean yaw error "
                 f"{math.degrees(statistics.fmean(yaw_err)):+.2f} deg), bound "
                 f"{math.degrees(drift_bound) * 60:.2f} deg/min = 5 x gyro bias + accel leak",
                 "a yaw integrating something other than the gyro bias -- a frame error in "
                 "the integration, or an accelerometer correction leaking into yaw")

    # Velocity: flow noise x height through the flow crossover, accel bias
    # over the same time constant, the attitude error leaking g x tilt over
    # it, and the gyro noise in the flow's rotation compensation.
    dt_flow = 1.0 / float(flow["rate_hz"])
    tau = float(vel["flow_tau_s"])
    alpha = dt_flow / (tau + dt_flow)
    v_flow_noise = float(flow["noise_rad_s"]) * height * math.sqrt(alpha / 2)
    v_h_dc = imu["accel_bias_m_s2"] * tau + imu["gyro_noise_rad_s"] * height
    v_h_leak = g * (tilt_dc + 2 * tilt_turn) * tau
    v_h_bound = min(VEL_CAP, MARGIN * (v_h_dc + 3 * v_flow_noise) + v_h_leak)
    for axis, name in ((0, "x"), (1, "y")):
        e = rms([ve[axis] for ve in v_err])
        c.within(f"velocity error RMS world {name}", e, v_h_bound,
                 f"{e:.4f} m/s, bound {v_h_bound:.4f} = min(cap {VEL_CAP}, {MARGIN:.0f}x({v_h_dc:.4f} dc + "
                 f"3x{v_flow_noise:.4f} flow) + {v_h_leak:.4f} tilt leak)",
                 "a flipped flow sign, the gyro term dropped from the flow, a body-frame "
                 "innovation added to the world state, or the wrong gravity sign")
    dt_tof = 1.0 / float(tof["rate_hz"])
    omega, zeta = float(vel["tof_omega_rad_s"]), float(vel["tof_zeta"])
    k1, k2 = 2 * zeta * omega, omega * omega
    range_mean = height / max(math.cos(tilt_true), 0.1)
    sig_z = tof["noise_m"] + tof["noise_frac"] * range_mean + tof["resolution_m"]
    v_z_noise = sig_z * math.sqrt(dt_tof) * k2 / math.sqrt(2 * k1)
    v_z_dc = imu["accel_bias_m_s2"] * k1 / k2
    # Slant-vs-altitude: a tilt error e at tilt t is range*sin(t)*e of
    # altitude error, oscillating at the lap rate, reaching the rate through k2.
    v_z_leak = range_mean * math.sin(tilt_true) * (tilt_dc + 2 * tilt_turn) * k2 / max(k1, yaw_rate)
    v_z_bound = min(VEL_CAP, MARGIN * (v_z_dc + 3 * v_z_noise) + v_z_leak)
    e = rms([ve[2] for ve in v_err])
    c.within("velocity error RMS world z", e, v_z_bound,
             f"{e:.4f} m/s, bound {v_z_bound:.4f} = min(cap {VEL_CAP}, {MARGIN:.0f}x({v_z_dc:.4f} dc + "
             f"3x{v_z_noise:.4f} tof) + {v_z_leak:.4f} tilt leak)",
             "the slant range used as altitude (a phantom climb in every turn), the range "
             "residual not reaching the climb rate, or the wrong gravity sign")
    v_floor = 3 * 2.0 * skew_max + 1e-4      # 2 m/s^2 of accel over the skew
    v_rms = rms([math.dist(ve, (0, 0, 0)) for ve in v_err])
    c.above("velocity error is not zero", v_rms, v_floor,
            f"{v_rms:.4f} m/s >= floor {v_floor:.4f}",
            "an estimator that publishes the truth velocity")
    xs, ys = response(p.flow_events, est_stamps, v_err,
                      lambda inn, e0, e1: [(inn[0], e1[0] - e0[0]), (inn[1], e1[1] - e0[1])])
    corr = correlation(xs, ys)
    c.above("velocity responds to each flow sample", corr, MIN_CORRELATION,
            f"corr(innovation, change in velocity error) {corr:.2f} over {len(xs)} "
            f"axis-intervals (min {MIN_CORRELATION})",
            "a velocity copied from truth, or one that never fuses the flow")

    # Position is truth by decision, and must be exactly that -- up to one
    # IMU period: the estimator copies the LATEST truth when the IMU sample
    # arrives, and the truth of the same stamp may land a callback later, so
    # the copy can be one sample (4 ms) old. Measured: 83 um at hover.
    # 1.5x: max(speeds) is sampled at the paired stamps and can sit a little
    # under the speed at the worst sample (measured 7.25 vs 7.24 mm on the
    # demo lap); the failure this catches is centimetres, not tenths of a mm.
    p_floor = 1.5 * max(speeds) * (skew_max + 1.0 / float(imu["rate_hz"])) + 1e-6
    c.within("position equals truth", max(pos_err), p_floor,
             f"max |p_est - p_truth| {max(pos_err):.2e} m (allowed {p_floor:.1e} = "
             f"|v| x one IMU period)",
             "a position source other than truth switched on without anyone deciding to")

    # Rate and latency.
    est_rate = len(p.est) / span if span > 0 else 0.0
    imu_span = (p.imu_last - p.imu_first) if p.imu_first is not None else 0.0
    imu_rate = p.imu_count / imu_span if imu_span > 0 else 0.0
    c.above("state_est publishes at the IMU rate", est_rate, 0.9 * imu_rate,
            f"{est_rate:.1f} Hz vs IMU {imu_rate:.1f} Hz (sim time), {len(p.est) / wall:.1f} Hz wall",
            "an estimator publishing on a slower trigger, or dropping steps waiting for a sensor")
    lags = [(latest - t) * 1e3 for t, *_, latest in p.est if math.isfinite(latest)]
    print(f"  info state_est stamp lag                  median {statistics.median(lags):+.2f} ms "
          f"behind the latest truth stamp at receipt (max {max(lags):+.2f})")
    print(f"  info per-axis velocity RMS                x {rms([v[0] for v in v_err]):.4f}  "
          f"y {rms([v[1] for v in v_err]):.4f}  z {rms([v[2] for v in v_err]):.4f} m/s; "
          f"|v| RMS {v_rms:.4f}")
    print(f"  info attitude RMS                         tilt {math.degrees(rms(tilt_err)):.3f}  "
          f"yaw {math.degrees(rms(yaw_err)):.3f} deg (yaw mean "
          f"{math.degrees(statistics.fmean(yaw_err)):+.3f})")

    print()
    print("ESTIMATOR OK" if c.failures == 0 else f"ESTIMATOR FAILED: {c.failures} check(s)")
    return c.failures


if __name__ == "__main__":
    sys.exit(main())
