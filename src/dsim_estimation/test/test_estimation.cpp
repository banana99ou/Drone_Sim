// Invariants for the attitude and velocity filters.
//
// Every test feeds a synthetic sequence whose answer is known exactly -- a
// vehicle held at a chosen attitude, a known velocity swept under a flow
// sensor, a known altitude behind a slanted rangefinder -- and requires the
// filter to recover it. Each comment names the wrong implementation it would
// fail on, and scripts/mutation_check.sh injects those implementations to
// prove the failure is real.
//
// No ROS: this compiles on the host with g++ and gtest, which is what lets
// the mutation harness run without the container.

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include "dsim_estimation/attitude_filter.hpp"
#include "dsim_estimation/velocity_filter.hpp"

using namespace dsim_estimation;

namespace
{
constexpr double kG = 9.80665;
constexpr double kImuDt = 1.0 / 250.0;
constexpr double kDeg = M_PI / 180.0;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
const Eigen::Vector3d kField(3.0e-05, 0.0, -4.0e-05);   // the shipped field: dip 53 deg

/// Gains an order of magnitude hotter than the shipped ones, so convergence
/// tests finish in simulated seconds rather than minutes. These test the
/// CODE; the shipped config is exercised by the banked-turn test below and
/// measured live by scripts/check_estimator.py.
AttitudeConfig fastAttitude()
{
  AttitudeConfig c;
  c.kp_accel = 1.0;
  c.ki_accel = 0.1;
  c.kp_mag = 1.0;
  c.ki_mag = 0.1;
  c.field_world = kField;
  return c;
}

/// What a static vehicle at attitude q feeds the filter.
Eigen::Vector3d accelAtRest(const Eigen::Quaterniond & q)
{
  return q.conjugate() * Eigen::Vector3d(0.0, 0.0, kG);
}
Eigen::Vector3d magAt(const Eigen::Quaterniond & q)
{
  return q.conjugate() * kField;
}

/// Angle between two attitudes, radians.
double angularDistance(const Eigen::Quaterniond & a, const Eigen::Quaterniond & b)
{
  return Eigen::AngleAxisd(a.normalized().conjugate() * b.normalized()).angle();
}

/// Tilt error only: the angle between where the two attitudes put "up" in
/// the BODY frame, R^T e3. That is the accelerometer's observable, it is a
/// function of ZYX roll and pitch alone, and it is independent of yaw -- so
/// a test about roll and pitch cannot pass or fail on yaw. (The angle between
/// the body z axes in the WORLD frame is not yaw-free: a tilted vehicle's z
/// axis swings round as it yaws.)
double tiltError(const Eigen::Quaterniond & a, const Eigen::Quaterniond & b)
{
  const Eigen::Vector3d ua = a.conjugate() * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d ub = b.conjugate() * Eigen::Vector3d::UnitZ();
  return std::acos(std::min(1.0, ua.dot(ub)));
}

double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

/// Hold the vehicle at attitude q_true for `seconds`, with the gyro reading
/// `gyro` (a bias, since the vehicle is not moving), the accelerometer
/// reading gravity, and, optionally, the mag at 50 Hz.
void holdStill(
  AttitudeFilter & f, const Eigen::Quaterniond & q_true, const Eigen::Vector3d & gyro,
  double seconds, bool with_mag)
{
  const int n = static_cast<int>(seconds / kImuDt);
  for (int i = 0; i < n; ++i) {
    f.correctAccel(accelAtRest(q_true));
    if (with_mag && i % 5 == 0) {f.correctMag(magAt(q_true));}
    f.predict(gyro, kImuDt);
  }
}

VelocityConfig testVelocity()
{
  VelocityConfig c;
  c.gravity = kG;
  return c;
}
}  // namespace

// ===========================================================================
// Attitude
// ===========================================================================

// FAILS IF: initialisation gets the tilt from the accelerometer wrong (R for
// R^T, a wrong reference axis) or reads the mag heading without tilt
// compensation. A vehicle at a known attitude, from one accel and one mag
// sample, must come back exactly.
TEST(AttitudeFilter, InitialisesFromOneAccelAndOneMagSample)
{
  AttitudeFilter f(fastAttitude());
  const auto q_true = quaternionFromRollPitchYaw(25 * kDeg, 15 * kDeg, 50 * kDeg);
  const Eigen::Vector3d mag = magAt(q_true);
  ASSERT_TRUE(f.initialise(accelAtRest(q_true), &mag));
  EXPECT_LT(angularDistance(f.quaternion(), q_true), 1e-9);

  // Without a mag, roll and pitch are still right and yaw is DEFINED to be
  // zero: a fixed starting point is what makes gyro-only drift measurable.
  AttitudeFilter g(fastAttitude());
  ASSERT_TRUE(g.initialise(accelAtRest(q_true), nullptr));
  EXPECT_LT(tiltError(g.quaternion(), q_true), 1e-9);
  EXPECT_NEAR(g.rollPitchYaw().z(), 0.0, 1e-9);
}

// FAILS IF: the accelerometer correction has the wrong sign (the estimate
// runs away from gravity instead of towards it), the wrong axis, or is
// never applied. Started from LEVEL while the vehicle sits at 20/-10 deg,
// gravity alone must bring roll and pitch to the truth.
TEST(AttitudeFilter, GravityOnlyAccelConvergesToTheTrueTilt)
{
  AttitudeFilter f(fastAttitude());
  const auto q_true = quaternionFromRollPitchYaw(20 * kDeg, -10 * kDeg, 0.0);
  ASSERT_TRUE(f.initialise(Eigen::Vector3d(0, 0, kG), nullptr));   // wrong: level
  ASSERT_GT(tiltError(f.quaternion(), q_true), 20 * kDeg);          // and it knows it

  holdStill(f, q_true, Eigen::Vector3d::Zero(), 60.0, false);
  EXPECT_LT(tiltError(f.quaternion(), q_true), 0.05 * kDeg);
  const Eigen::Vector3d rpy = f.rollPitchYaw();
  EXPECT_NEAR(rpy.x(), 20 * kDeg, 0.05 * kDeg);
  EXPECT_NEAR(rpy.y(), -10 * kDeg, 0.05 * kDeg);
}

// FAILS IF: the bias integrator has the wrong sign (the loop diverges), is
// never fed, or the bias is not subtracted from the gyro in predict(). A
// gyro that reads a constant rate on a vehicle that is not moving must end
// up with that rate in the bias state and the attitude unmoved.
TEST(AttitudeFilter, ConstantGyroBiasIsEstimatedAway)
{
  AttitudeFilter f(fastAttitude());
  const auto q_true = quaternionFromRollPitchYaw(5 * kDeg, -3 * kDeg, 30 * kDeg);
  const Eigen::Vector3d bias(0.01, -0.02, 0.005);   // 50x the shipped 2e-4, for speed
  const Eigen::Vector3d mag = magAt(q_true);
  ASSERT_TRUE(f.initialise(accelAtRest(q_true), &mag));

  // The mag is needed for the z component: at hover the accelerometer's
  // error has no body-z part, so a yaw-rate bias is invisible to it.
  holdStill(f, q_true, bias, 120.0, true);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(f.gyroBias()(i), bias(i), 0.05 * bias.cwiseAbs().maxCoeff()) << "axis " << i;
  }
  EXPECT_LT(angularDistance(f.quaternion(), q_true), 0.05 * kDeg);
}

// FAILS IF: the mag heading is taken from the raw body x/y components. The
// shipped field dips 53 deg, so at 25 deg of bank the raw heading is off by
// tens of degrees; only a heading computed after rotating the sample level
// with the estimated roll/pitch converges to the true yaw.
TEST(AttitudeFilter, YawConvergesFromTheMagAtANonZeroTilt)
{
  AttitudeFilter f(fastAttitude());
  const auto q_true = quaternionFromRollPitchYaw(25 * kDeg, 15 * kDeg, 50 * kDeg);
  ASSERT_TRUE(f.initialise(accelAtRest(q_true), nullptr));   // right tilt, yaw 0
  ASSERT_NEAR(f.rollPitchYaw().z(), 0.0, 1e-9);

  holdStill(f, q_true, Eigen::Vector3d::Zero(), 60.0, true);
  EXPECT_NEAR(wrapAngle(f.rollPitchYaw().z() - 50 * kDeg), 0.0, 0.05 * kDeg);
  // ...and the mag must not have disturbed the tilt: its correction is about
  // the world vertical only.
  EXPECT_LT(tiltError(f.quaternion(), q_true), 0.05 * kDeg);
}

// FAILS IF: the mag correction is never applied (yaw then drifts WITH the
// mag too), or the gyro is not integrated (yaw then does not drift without
// it). Two runs, one bias, opposite outcomes.
TEST(AttitudeFilter, YawDriftsWithoutTheMagAndNotWithIt)
{
  const auto q_true = quaternionFromRollPitchYaw(0.0, 0.0, 0.0);
  const Eigen::Vector3d z_bias(0.0, 0.0, 0.01);    // 0.01 rad/s: 0.2 rad in 20 s
  const Eigen::Vector3d mag = magAt(q_true);

  AttitudeFilter without(fastAttitude());
  ASSERT_TRUE(without.initialise(accelAtRest(q_true), &mag));
  holdStill(without, q_true, z_bias, 20.0, false);
  EXPECT_GT(std::abs(without.rollPitchYaw().z()), 0.15);

  AttitudeFilter with(fastAttitude());
  ASSERT_TRUE(with.initialise(accelAtRest(q_true), &mag));
  holdStill(with, q_true, z_bias, 20.0, true);
  EXPECT_LT(std::abs(with.rollPitchYaw().z()), 0.01);
}

// FAILS IF: the gyro is integrated in the world frame instead of the body
// frame (q * dq vs dq * q). Indistinguishable at level, so this starts the
// vehicle rolled 90 deg, where a body-z rate is a world -y rate.
TEST(AttitudeFilter, PredictIntegratesTheGyroInTheBodyFrame)
{
  AttitudeFilter f(fastAttitude());
  const auto q0 = quaternionFromRollPitchYaw(90 * kDeg, 0.0, 0.0);
  ASSERT_TRUE(f.initialise(accelAtRest(q0), nullptr));
  f.setQuaternionForTest(q0);

  const Eigen::Vector3d gyro(0.0, 0.0, 1.0);
  for (int i = 0; i < 125; ++i) {f.predict(gyro, kImuDt);}   // 0.5 rad

  const Eigen::Quaterniond dq(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond body_frame = q0 * dq;
  const Eigen::Quaterniond world_frame = dq * q0;
  EXPECT_LT(angularDistance(f.quaternion(), body_frame), 1e-6);
  EXPECT_GT(angularDistance(f.quaternion(), world_frame), 0.1);
}

// FAILS IF: a sample that is plainly not gravity is used as one. Free fall
// (0 g) and a hard bounce (2 g) must be refused, at initialisation and as a
// correction, while the coordinated-turn magnitude at the tilt clamp
// (g / cos 40 deg) must be ADMITTED -- the gate is for impossible readings,
// not for manoeuvres.
TEST(AttitudeFilter, AccelGateRefusesWhatIsNotGravity)
{
  AttitudeFilter f(AttitudeConfig{});
  EXPECT_FALSE(f.initialise(Eigen::Vector3d::Zero(), nullptr));
  EXPECT_FALSE(f.initialise(Eigen::Vector3d(0, 0, 2 * kG), nullptr));
  EXPECT_FALSE(f.initialise(Eigen::Vector3d(0, 0, kNaN), nullptr));
  EXPECT_FALSE(f.initialised());
  ASSERT_TRUE(f.initialise(Eigen::Vector3d(0, 0, kG), nullptr));

  EXPECT_FALSE(f.correctAccel(Eigen::Vector3d::Zero()));
  EXPECT_FALSE(f.correctAccel(Eigen::Vector3d(0, 0, 2 * kG)));
  EXPECT_TRUE(f.correctAccel(Eigen::Vector3d(0, 0, kG / std::cos(40 * kDeg))));
  f.predict(Eigen::Vector3d::Zero(), kImuDt);
  EXPECT_TRUE(f.lastStep().accel_applied);
}

// The documented weakness, measured with the SHIPPED gains so the number in
// the class comment is a test result and not a belief. In a coordinated
// turn the specific force lies along body z, so the accelerometer says "you
// are level" while the vehicle is banked; the pull towards level is bounded
// by the low gain and by the fact that, in the world frame, the wrong
// direction rotates with the lap.
//
// FAILS IF: kp_accel is raised to where a 3.5 s lap corrupts the attitude by
// more than 2 deg (at kp = 1 this reads ~9 deg), or if the turn's specific
// force is gated out (the error would then be ~0, and the gate would leave
// roll/pitch unobserved in every turn -- the lower bound is here so that
// "fix" cannot land silently).
TEST(AttitudeFilter, BankedTurnPullsTheEstimateTowardsLevelByABoundedAmount)
{
  AttitudeConfig cfg;              // shipped defaults: kp 0.1, ki 0.01
  cfg.field_world = kField;
  AttitudeFilter f(cfg);

  const double bank = 18 * kDeg;   // make viz's 1 m / 3.5 s lap
  const double period = 3.5;
  const double yaw_rate = 2 * M_PI / period;
  const Eigen::Vector3d accel_turn(0.0, 0.0, kG / std::cos(bank));   // along body z

  auto attitude = [&](double t) {
      // Yaw advances with the lap; the bank is a constant roll towards the
      // centre. Which sign the roll has does not matter to the filter.
      return quaternionFromRollPitchYaw(bank, 0.0, yaw_rate * t);
    };
  ASSERT_TRUE(f.initialise(accelAtRest(attitude(0.0)), nullptr));
  f.setQuaternionForTest(attitude(0.0));                     // start exactly right

  // Fly laps; return the worst tilt error over the last `measure` of them.
  // The steady state is a world-frame error vector of constant magnitude
  // kp sin(bank) / sqrt(kp^2 + Omega^2) rotating with the lap, 90 deg
  // behind the bank. Starting from zero error adds a fixed transient of the
  // same size decaying over 1/kp = 10 s, so the first laps swing between 0
  // and twice the steady value; the measurement waits that out.
  auto fly = [&](int laps, int measure) {
      double worst = 0.0;
      const int n = static_cast<int>(laps * period / kImuDt);
      const int from = static_cast<int>((laps - measure) * period / kImuDt);
      for (int i = 0; i < n; ++i) {
        const double t = i * kImuDt;
        const Eigen::Quaterniond q = attitude(t);
        // The true body rate of a constant-bank turn: the yaw rate seen in
        // the body frame. Exact, so every degree of error below is the
        // accelerometer's doing.
        const Eigen::Vector3d gyro = q.conjugate() * Eigen::Vector3d(0.0, 0.0, yaw_rate);
        f.correctAccel(accel_turn);
        f.predict(gyro, kImuDt);
        if (i >= from) {worst = std::max(worst, tiltError(f.quaternion(), attitude(t + kImuDt)));}
      }
      return worst;
    };
  const double transient = fly(3, 3);
  const double steady = fly(10, 2);      // 13 laps in all; last 2 measured

  // Analytic: 0.1 * sin(18 deg) / sqrt(0.1^2 + 1.795^2) = 0.0172 rad = 0.985 deg.
  EXPECT_NEAR(steady, 0.985 * kDeg, 0.1 * kDeg) << "the class comment quotes this number";
  EXPECT_LT(transient, 2.2 * kDeg) << "at most twice the steady state, by the linear model";
  EXPECT_GT(steady, 0.3 * kDeg) << "the weakness has gone -- update the class comment";
  // Bounded means bounded: the last two laps are no worse than the first
  // three. A growing error means something is integrating the turn -- see
  // the next test.
  EXPECT_LT(steady, transient);
}

// Why ki_accel ships as zero. With any integral gain on the accelerometer
// error, a sustained turn -- whose error is constant in the body frame --
// is indistinguishable from a gyro bias, and the integrator winds up
// without bound: at ki = 0.01 about half a degree of tilt error PER LAP.
// This test measures that so the shipped zero is a result, not a belief.
//
// FAILS IF: the integrator stops winding up (someone gated it, or found a
// way to tell a turn from a bias). That would be good news; update the
// AttitudeConfig comment and consider shipping ki_accel > 0.
TEST(AttitudeFilter, AnAccelIntegratorWindsUpInASustainedTurn)
{
  AttitudeConfig cfg;
  cfg.ki_accel = 0.01;
  AttitudeFilter f(cfg);
  const double bank = 18 * kDeg, period = 3.5, yaw_rate = 2 * M_PI / period;
  const Eigen::Vector3d accel_turn(0.0, 0.0, kG / std::cos(bank));
  auto attitude = [&](double t) {return quaternionFromRollPitchYaw(bank, 0.0, yaw_rate * t);};
  ASSERT_TRUE(f.initialise(accelAtRest(attitude(0.0)), nullptr));
  f.setQuaternionForTest(attitude(0.0));

  auto fly_laps = [&](int laps) {
      const int n = static_cast<int>(laps * period / kImuDt);
      for (int i = 0; i < n; ++i) {
        const Eigen::Quaterniond q = attitude(i * kImuDt);
        f.correctAccel(accel_turn);
        f.predict(q.conjugate() * Eigen::Vector3d(0.0, 0.0, yaw_rate), kImuDt);
      }
      return tiltError(f.quaternion(), attitude(n * kImuDt));
    };
  const double after_3 = fly_laps(3);
  const double after_6 = fly_laps(3);
  EXPECT_GT(after_6, after_3 + 1.0 * kDeg) << "the integrator no longer winds up";
  EXPECT_GT(f.gyroBias().norm(), 0.05) << "no fake bias accumulated";
}

// FAILS IF: the corrections are applied when there is no interval to apply
// them over, or the filter integrates across a stall. A 1 s gap must not
// turn one gyro sample into a second of rotation.
TEST(AttitudeFilter, AStallIsNotAnInterval)
{
  AttitudeFilter f(fastAttitude());
  ASSERT_TRUE(f.initialise(Eigen::Vector3d(0, 0, kG), nullptr));
  f.predict(Eigen::Vector3d(0, 0, 1.0), 1.0);        // > max_dt_s
  f.predict(Eigen::Vector3d(0, 0, 1.0), 0.0);
  f.predict(Eigen::Vector3d(0, 0, 1.0), -kImuDt);
  EXPECT_LT(angularDistance(f.quaternion(), Eigen::Quaterniond::Identity()), 1e-12);
}

// FAILS IF: reset() leaves anything behind. After a simulator reset the
// vehicle is teleported home; a bias or attitude from the old run is a
// fabricated starting point for the new one.
TEST(AttitudeFilter, ResetForgetsEverything)
{
  AttitudeFilter f(fastAttitude());
  const auto q_true = quaternionFromRollPitchYaw(20 * kDeg, 0.0, 40 * kDeg);
  const Eigen::Vector3d mag = magAt(q_true);
  ASSERT_TRUE(f.initialise(accelAtRest(q_true), &mag));
  holdStill(f, q_true, Eigen::Vector3d(0.01, 0, 0), 5.0, true);
  ASSERT_GT(f.gyroBias().norm(), 0.0);

  f.reset();
  EXPECT_FALSE(f.initialised());
  EXPECT_EQ(f.gyroBias().norm(), 0.0);
  EXPECT_LT(angularDistance(f.quaternion(), Eigen::Quaterniond::Identity()), 1e-12);
  // Uninitialised means inert: no correction, no integration.
  EXPECT_FALSE(f.correctAccel(accelAtRest(q_true)));
  EXPECT_FALSE(f.correctMag(mag));
  f.predict(Eigen::Vector3d(0, 0, 1.0), kImuDt);
  EXPECT_LT(angularDistance(f.quaternion(), Eigen::Quaterniond::Identity()), 1e-12);
}

// FAILS IF: the Euler helpers disagree with each other or with the
// controller's definition of yaw (heading of the body x axis). Round trip
// through a quaternion at an attitude where every angle is non-zero.
TEST(AttitudeFilter, EulerHelpersRoundTrip)
{
  const Eigen::Vector3d rpy(31 * kDeg, -22 * kDeg, 117 * kDeg);
  const auto q = quaternionFromRollPitchYaw(rpy.x(), rpy.y(), rpy.z());
  const Eigen::Vector3d back = rollPitchYawOf(q);
  for (int i = 0; i < 3; ++i) {EXPECT_NEAR(back(i), rpy(i), 1e-12) << "angle " << i;}

  const Eigen::Vector3d x_body = q * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(std::atan2(x_body.y(), x_body.x()), rpy.z(), 1e-12);
}

// ===========================================================================
// Velocity
// ===========================================================================

// FAILS IF: gravity is added with the wrong sign (a hovering vehicle then
// accelerates upwards at 2 g in the estimate) or the accelerometer is
// rotated with R^T instead of R (a tilted vehicle at rest then has a
// spurious horizontal acceleration). At rest, at any attitude, the world
// acceleration must be zero.
TEST(VelocityFilter, AVehicleAtRestHasZeroWorldAcceleration)
{
  VelocityFilter f(testVelocity());
  for (const auto & rpy : {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(20 * kDeg, -15 * kDeg, 70 * kDeg),
      Eigen::Vector3d(-35 * kDeg, 10 * kDeg, -120 * kDeg)})
  {
    const auto q = quaternionFromRollPitchYaw(rpy.x(), rpy.y(), rpy.z());
    const Eigen::Vector3d a = f.worldAcceleration(q.toRotationMatrix(), accelAtRest(q));
    EXPECT_LT(a.norm(), 1e-12) << "at rpy " << rpy.transpose();
  }
}

// FAILS IF: the gravity sign or the rotation direction is wrong, or dt is
// applied incorrectly. A constant 1 m/s^2 along world x for one second,
// measured by a vehicle pitched 30 deg (so the specific force is spread
// across body x and z), must integrate to exactly 1 m/s along world x.
TEST(VelocityFilter, ConstantWorldAccelerationIntegratesToTheRightVelocity)
{
  VelocityFilter f(testVelocity());
  const auto q = quaternionFromRollPitchYaw(0.0, 30 * kDeg, 0.0);
  const Eigen::Matrix3d R = q.toRotationMatrix();
  const Eigen::Vector3d a_world(1.0, 0.0, 0.0);
  // Specific force = acceleration minus gravity, in the body frame.
  const Eigen::Vector3d accel_body = R.transpose() * (a_world - Eigen::Vector3d(0, 0, -kG));

  for (int i = 0; i < 250; ++i) {f.predict(R, accel_body, kImuDt);}
  EXPECT_NEAR(f.velocityWorld().x(), 1.0, 1e-9);
  EXPECT_NEAR(f.velocityWorld().y(), 0.0, 1e-9);
  EXPECT_NEAR(f.velocityWorld().z(), 0.0, 1e-9);
}

// FAILS IF: the flow-to-velocity formula has the sign of OpticalFlow.msg
// wrong on either axis, drops the gyro subtraction (a rotating vehicle then
// sees rate x height of phantom velocity), or applies the body-frame
// innovation without rotating it into the world (right only at zero yaw,
// which is why the vehicle here is yawed 60 deg and banked). A known
// velocity swept under the sensor at a known height must come back in
// WORLD coordinates.
TEST(VelocityFilter, FlowAtAKnownTiltHeightAndYawReturnsTheWorldVelocity)
{
  VelocityFilter f(testVelocity());
  const auto q = quaternionFromRollPitchYaw(10 * kDeg, -5 * kDeg, 60 * kDeg);
  const Eigen::Matrix3d R = q.toRotationMatrix();

  // The flow sees only body x/y, so the known velocity is chosen to have no
  // body-z component; anything else would be asking the sensor for a number
  // it cannot measure.
  const Eigen::Vector3d v_body(0.8, -0.3, 0.0);
  const Eigen::Vector3d v_world = R * v_body;
  const Eigen::Vector3d omega(0.3, -0.2, 0.1);      // rotating too: the gyro terms matter
  const double altitude = 1.5;
  const double range = altitude / R(2, 2);          // the slant range the module scales by
  const double T = 0.02;

  // The reading such a motion produces, by the sensor's own model:
  // flow = translation over the range plus the rotation, and the gyro
  // integrals reported alongside.
  FlowReading flow;
  flow.integration_time_s = T;
  flow.integrated_x = (-v_body.y() / range + omega.x()) * T;
  flow.integrated_y = (v_body.x() / range + omega.y()) * T;
  flow.integrated_xgyro = omega.x() * T;
  flow.integrated_ygyro = omega.y() * T;
  flow.ground_distance_m = range;
  flow.quality = 200;

  // The body-frame reconstruction alone, before any fusion.
  const Eigen::Vector3d recovered = VelocityFilter::flowBodyVelocity(flow);
  EXPECT_NEAR(recovered.x(), v_body.x(), 1e-12);
  EXPECT_NEAR(recovered.y(), v_body.y(), 1e-12);

  // Now fused: constant velocity, so the accelerometer reads pure gravity,
  // and 10 s of flow at 50 Hz against a 0.2 s time constant is converged.
  for (int i = 0; i < 500; ++i) {
    ASSERT_TRUE(f.correctFlow(R, flow));
    for (int k = 0; k < 5; ++k) {f.predict(R, accelAtRest(q), kImuDt);}
  }
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(f.velocityWorld()(i), v_world(i), 1e-6) << "world axis " << i;
    EXPECT_NEAR(f.lastFlowVelocityWorld()(i), v_world(i), 1e-9) << "world axis " << i;
  }
}

// FAILS IF: the flow innovation touches the body-z velocity. The sensor
// cannot see motion along body z, so a climb the accelerometer integrated
// must survive a flow correction untouched.
TEST(VelocityFilter, FlowLeavesTheBodyZVelocityAlone)
{
  VelocityFilter f(testVelocity());
  const auto q = quaternionFromRollPitchYaw(15 * kDeg, 0.0, 30 * kDeg);
  const Eigen::Matrix3d R = q.toRotationMatrix();
  const Eigen::Vector3d v_world = R * Eigen::Vector3d(0.0, 0.0, 0.4);   // pure body-z climb
  f.setVelocityForTest(v_world);

  FlowReading flow;                    // reports zero body x/y velocity, as it should
  flow.integration_time_s = 0.02;
  flow.ground_distance_m = 1.5;
  flow.quality = 255;
  for (int i = 0; i < 200; ++i) {ASSERT_TRUE(f.correctFlow(R, flow));}
  EXPECT_LT((f.velocityWorld() - v_world).norm(), 1e-9);
}

// FAILS IF: the slant range is used as the altitude. The beam runs along
// body -z, so at 30 deg of bank it is 1/cos(30) = 1.155 times longer than
// the altitude; 1.732 m of range over a 1.5 m floor.
TEST(VelocityFilter, ASlantRangeAtAKnownTiltGivesTheTrueAltitude)
{
  const double altitude = 1.5;
  const auto q = quaternionFromRollPitchYaw(30 * kDeg, 0.0, 0.0);
  const Eigen::Matrix3d R = q.toRotationMatrix();
  const double slant = 1.7320508;                       // hand-computed, pins the number
  EXPECT_NEAR(VelocityFilter::tofAltitude(R, slant), altitude, 1e-6);
  EXPECT_NEAR(VelocityFilter::tofAltitude(Eigen::Matrix3d::Identity(), altitude), altitude, 1e-12);

  VelocityFilter f(testVelocity());
  EXPECT_FALSE(f.altitudeValid());
  ASSERT_TRUE(f.correctTof(R, slant, 0.0));            // first reading seeds
  EXPECT_TRUE(f.altitudeValid());
  EXPECT_NEAR(f.altitude(), altitude, 1e-6);
  EXPECT_NEAR(f.lastTofAltitude(), altitude, 1e-6);
  // ...and seeding carries no rate information.
  EXPECT_EQ(f.velocityWorld().z(), 0.0);
}

// FAILS IF: the range residual is not fed into the climb rate, is fed with
// the wrong sign (diverges), or the slant is used raw (the rate then comes
// out as 0.3 / cos 30 = 0.346 m/s and the altitude 15% high). A floor
// receding at 0.3 m/s under a banked vehicle, seen only through the
// rangefinder -- the accelerometer reads pure gravity -- must appear in the
// estimate as 0.3 m/s of climb.
TEST(VelocityFilter, TheRangefinderPinsTheClimbRate)
{
  VelocityFilter f(testVelocity());
  const auto q = quaternionFromRollPitchYaw(30 * kDeg, 0.0, 45 * kDeg);
  const Eigen::Matrix3d R = q.toRotationMatrix();
  const double vz = 0.3;
  const double dt = 1.0 / 30.0;

  auto altitude = [&](double t) {return 1.5 + vz * t;};
  ASSERT_TRUE(f.correctTof(R, altitude(0.0) / R(2, 2), 0.0));      // seed
  for (int i = 1; i <= 600; ++i) {                                  // 20 s
    const double t = i * dt;
    f.predict(R, accelAtRest(q), dt);
    ASSERT_TRUE(f.correctTof(R, altitude(t) / R(2, 2), dt));
  }
  EXPECT_NEAR(f.velocityWorld().z(), vz, 0.01);
  EXPECT_NEAR(f.altitude(), altitude(600 * dt), 0.02);
  EXPECT_NEAR(f.velocityWorld().x(), 0.0, 1e-9);     // the range is vertical information only
  EXPECT_NEAR(f.velocityWorld().y(), 0.0, 1e-9);
}

// FAILS IF: an unusable reading is fused anyway. quality 0 means "no
// measurement"; a NaN range is the sensor node's "the rangefinder had no
// return"; +/-inf is REP 117's out-of-range. Each must leave the state
// untouched and be counted, not clamped into a number.
TEST(VelocityFilter, UnusableReadingsAreRejectedAndCounted)
{
  VelocityFilter f(testVelocity());
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d v0(0.5, -0.2, 0.1);
  f.setVelocityForTest(v0);

  FlowReading flow;
  flow.integration_time_s = 0.02;
  flow.integrated_y = 0.5;           // would be 37 m/s of body x if believed
  flow.ground_distance_m = 1.5;
  flow.quality = 0;
  EXPECT_FALSE(f.correctFlow(R, flow));
  flow.quality = 100;
  flow.ground_distance_m = kNaN;
  EXPECT_FALSE(f.correctFlow(R, flow));
  flow.ground_distance_m = 1.5;
  flow.integration_time_s = 0.0;
  EXPECT_FALSE(f.correctFlow(R, flow));
  EXPECT_EQ(f.flowRejected(), 3u);
  EXPECT_TRUE(f.velocityWorld().isApprox(v0));
  EXPECT_FALSE(f.lastStep().flow_applied);

  EXPECT_FALSE(f.correctTof(R, kInf, 0.0));
  EXPECT_FALSE(f.correctTof(R, -kInf, 0.0));
  EXPECT_FALSE(f.correctTof(R, kNaN, 0.0));
  EXPECT_FALSE(f.correctTof(R, 0.0, 0.0));
  EXPECT_EQ(f.tofRejected(), 4u);
  EXPECT_FALSE(f.altitudeValid());
  EXPECT_TRUE(f.velocityWorld().isApprox(v0));

  // A usable reading after the refusals is still taken.
  flow.integration_time_s = 0.02;
  EXPECT_TRUE(f.correctFlow(R, flow));
  EXPECT_TRUE(f.correctTof(R, 1.5, 0.0));
}

// FAILS IF: reset() leaves the old run's velocity or altitude in place.
TEST(VelocityFilter, ResetForgetsEverything)
{
  VelocityFilter f(testVelocity());
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  f.setVelocityForTest(Eigen::Vector3d(1, 2, 3));
  ASSERT_TRUE(f.correctTof(R, 1.5, 0.0));
  FlowReading flow;
  flow.quality = 0;
  f.correctFlow(R, flow);
  ASSERT_EQ(f.flowRejected(), 1u);

  f.reset();
  EXPECT_EQ(f.velocityWorld().norm(), 0.0);
  EXPECT_FALSE(f.altitudeValid());
  EXPECT_EQ(f.flowRejected(), 0u);
  EXPECT_EQ(f.tofRejected(), 0u);
  EXPECT_EQ(f.lastFlowVelocityWorld().norm(), 0.0);
}

// FAILS IF: a stall is integrated as if it were a sample interval. One
// accelerometer sample must not be applied across a whole second.
TEST(VelocityFilter, AStallIsNotAnInterval)
{
  VelocityFilter f(testVelocity());
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  f.predict(R, Eigen::Vector3d(1.0, 0.0, kG), 1.0);     // > max_dt_s
  f.predict(R, Eigen::Vector3d(1.0, 0.0, kG), 0.0);
  f.predict(R, Eigen::Vector3d(1.0, 0.0, kG), -kImuDt);
  EXPECT_EQ(f.velocityWorld().norm(), 0.0);
}
