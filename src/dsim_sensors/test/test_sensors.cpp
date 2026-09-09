// Invariants for the simplified ToF and optical-flow models.
//
// Written as statements that cannot be false if the model is right, with the
// noise draws passed in so every number below is exact rather than
// statistical. Each test says what would make it fail.

#include <gtest/gtest.h>
#include <cmath>

#include "dsim_sensors/optical_flow.hpp"
#include "dsim_sensors/rates.hpp"
#include "dsim_sensors/tof.hpp"

using namespace dsim_sensors;

namespace
{
Eigen::Matrix3d rollMatrix(double phi)
{
  return Eigen::Matrix3d(Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()));
}

TofConfig cleanTof()
{
  TofConfig c;
  c.noise_m = 0.0;
  c.noise_frac = 0.0;
  c.resolution_m = 0.0;
  return c;
}
}  // namespace

// FAILS IF: the rangefinder reports altitude instead of slant range. A tilted
// vehicle's downward beam travels 1/cos(tilt) further, and a sensor that
// magically returned height would hide the biggest error a flow/range pair
// makes in a banked turn.
TEST(Tof, TiltedBeamTravelsFurtherThanTheAltitude)
{
  const double h = 1.5;
  EXPECT_NEAR(tofTrueRange(h, Eigen::Matrix3d::Identity()), h, 1e-12);

  // Hand-computed slant ranges, so this pins the implementation rather than a
  // property of std::cos: at 1.5 m, 30 degrees of bank is 1.7321 m of beam.
  EXPECT_NEAR(tofTrueRange(h, rollMatrix(M_PI / 6)), 1.7320508, 1e-6);
  EXPECT_NEAR(tofTrueRange(h, rollMatrix(M_PI / 4)), 2.1213203, 1e-6);
  double previous = h;
  for (const double phi : {0.1, 0.3, 0.5, 0.7}) {
    const double got = tofTrueRange(h, rollMatrix(phi));
    EXPECT_NEAR(got, h / std::cos(phi), 1e-9) << "tilt " << phi << " rad";
    // Monotonic in tilt: a claim about the function, not about cosine.
    EXPECT_GT(got, previous) << "range must grow with tilt";
    previous = got;
  }
}

// FAILS IF: a beam pointing at the sky returns a number. Past 90 degrees of
// tilt the ray never meets the ground, and any finite answer there is
// fabricated.
TEST(Tof, BeamPointingUpwardsHasNoReturn)
{
  EXPECT_FALSE(std::isfinite(tofTrueRange(1.5, rollMatrix(M_PI / 2 + 0.01))));
  EXPECT_FALSE(std::isfinite(tofTrueRange(1.5, rollMatrix(M_PI))));
  // Exactly level-on-its-side is degenerate and must also refuse.
  EXPECT_FALSE(std::isfinite(tofTrueRange(1.5, rollMatrix(M_PI / 2))));
}

// FAILS IF: saturation is reported as a reading. Clamping to max_range would
// make "4.0 m" and "40 m" the same message, which is how a height estimator
// ends up confidently wrong.
TEST(Tof, OutOfRangeIsInfiniteNotClamped)
{
  auto c = cleanTof();
  c.min_range_m = 0.03;
  c.max_range_m = 4.0;

  EXPECT_EQ(tofMeasure(10.0, c, 0.0), kTooFar);
  EXPECT_EQ(tofMeasure(4.001, c, 0.0), kTooFar);
  EXPECT_EQ(tofMeasure(0.01, c, 0.0), kTooClose);
  // ...and a reading inside the band is passed through untouched.
  EXPECT_NEAR(tofMeasure(1.5, c, 0.0), 1.5, 1e-12);
}

// FAILS IF: the error stops growing with distance. A ToF is less accurate far
// away; a fixed-only error model would make the sensor implausibly good at the
// far end of its range, which is exactly where estimators struggle.
TEST(Tof, ErrorGrowsWithRange)
{
  TofConfig c;
  c.noise_m = 0.01;
  c.noise_frac = 0.01;
  c.resolution_m = 0.0;

  const double near_err = std::abs(tofMeasure(0.5, c, 1.0) - 0.5);
  const double far_err = std::abs(tofMeasure(3.5, c, 1.0) - 3.5);
  EXPECT_GT(far_err, near_err);
  // Exact, from the documented model: sigma = noise_m + noise_frac * range.
  EXPECT_NEAR(near_err, 0.01 + 0.01 * 0.5, 1e-12);
  EXPECT_NEAR(far_err, 0.01 + 0.01 * 3.5, 1e-12);
}

// FAILS IF: the reading is not quantised. The real part reports whole
// millimetres, and a controller differentiating a quantised signal sees steps
// -- pretending otherwise makes derived velocity look better than it is.
TEST(Tof, ReadingIsQuantisedToTheSensorResolution)
{
  TofConfig c;
  c.noise_m = 0.0;
  c.noise_frac = 0.0;
  c.resolution_m = 0.001;
  EXPECT_NEAR(tofMeasure(1.23456, c, 0.0), 1.235, 1e-9);
  EXPECT_NEAR(tofMeasure(1.23444, c, 0.0), 1.234, 1e-9);
}


// FAILS IF: the vehicle is in the floor and the sensor says "nothing within
// range". +inf and -inf mean opposite things, and confusing them tells a
// height estimator it has clear air beneath it while the airframe is on the
// ground.
TEST(Tof, AtOrBelowGroundLevelIsTooCloseNotTooFar)
{
  EXPECT_EQ(tofTrueRange(0.0, Eigen::Matrix3d::Identity()), kTooClose);
  EXPECT_EQ(tofTrueRange(-0.05, Eigen::Matrix3d::Identity()), kTooClose);
  EXPECT_EQ(tofTrueRange(-1.0, Eigen::Matrix3d::Identity()), kTooClose);
  // ...and it is still distinguishable from a beam pointing at the sky.
  EXPECT_EQ(tofTrueRange(1.5, rollMatrix(M_PI)), kTooFar);
}

// ---------------------------------------------------------------------------
// Optical flow.
//
// The central property is the one the model previously got backwards: the
// image is swept by the TRUE motion over the TRUE height, while the consumer
// only has the MEASURED range and the MEASURED gyro. Feeding the model the
// measured values made the documented reconstruction cancel them exactly, so
// a rangefinder 1 m out still recovered the velocity perfectly. The tests
// below feed deliberately wrong measurements and require the reconstruction to
// be wrong by the matching factor -- none of them could pass on that version.
// ---------------------------------------------------------------------------

namespace
{
FlowConfig cleanFlow()
{
  FlowConfig c;
  c.noise_rad_s = 0.0;
  return c;
}

/// The reconstruction OpticalFlow.msg tells a consumer to perform.
struct Recovered
{
  double vx, vy;
};

Recovered recover(const FlowSample & s, double dt)
{
  return {(s.integrated_y - s.integrated_ygyro) / dt * s.ground_distance_m,
          -(s.integrated_x - s.integrated_xgyro) / dt * s.ground_distance_m};
}

FlowInputs level(double v_x, double v_y, double range, double dt)
{
  FlowInputs in;
  in.v_body = Eigen::Vector3d(v_x, v_y, 0.0);
  in.true_range_m = range;
  in.measured_range_m = range;
  in.cos_tilt = 1.0;
  in.dt = dt;
  return in;
}
}  // namespace

// FAILS IF: flow is not inversely proportional to height. The same motion at
// twice the height sweeps half the angle; getting this wrong gives a velocity
// estimate that scales with altitude, the classic flow bug.
TEST(OpticalFlow, FlowScalesInverselyWithHeight)
{
  const auto c = cleanFlow();
  const auto low = flowSample(level(1.0, 0.0, 1.0, 0.02), c, 0.0, 0.0);
  const auto high = flowSample(level(1.0, 0.0, 2.0, 0.02), c, 0.0, 0.0);
  EXPECT_NEAR(low.integrated_y, 1.0 / 1.0 * 0.02, 1e-12);
  EXPECT_NEAR(high.integrated_y, 1.0 / 2.0 * 0.02, 1e-12);
  EXPECT_NEAR(low.integrated_y, 2.0 * high.integrated_y, 1e-12);
}

// FAILS IF: the sign convention drifts from the one OpticalFlow.msg documents.
// A sign error here makes a vehicle accelerate away from the position it is
// trying to hold.
TEST(OpticalFlow, SignConventionMatchesTheMessageDefinition)
{
  const auto c = cleanFlow();
  const auto fwd = flowSample(level(1.0, 0.0, 1.0, 0.1), c, 0, 0);
  EXPECT_GT(fwd.integrated_y, 0.0) << "motion along +x is positive flow about +y";
  EXPECT_NEAR(fwd.integrated_x, 0.0, 1e-12);

  const auto left = flowSample(level(0.0, 1.0, 1.0, 0.1), c, 0, 0);
  EXPECT_LT(left.integrated_x, 0.0) << "motion along +y is negative flow about +x";
  EXPECT_NEAR(left.integrated_y, 0.0, 1e-12);
}

// FAILS IF: a perfect sensor pair does not reconstruct the truth. This is the
// baseline the error tests below are measured against -- with no noise, no
// range error and no gyro error, the recovery must be exact.
TEST(OpticalFlow, PerfectSensorsReconstructVelocityExactly)
{
  const auto c = cleanFlow();
  const double dt = 0.02;
  auto in = level(0.7, -0.4, 1.5, dt);
  in.omega_true = Eigen::Vector3d(0.9, -1.3, 0.5);
  in.omega_measured = in.omega_true;              // a perfect gyro

  const auto r = recover(flowSample(in, c, 0.0, 0.0), dt);
  EXPECT_NEAR(r.vx, 0.7, 1e-9);
  EXPECT_NEAR(r.vy, -0.4, 1e-9);
}

// FAILS IF: a wrong range does not scale the reconstructed velocity.
//
// This is THE test the previous model could not pass: it generated the flow
// from the measured range, so the range divided out and a rangefinder reading
// 2.5 m instead of 1.5 m still recovered 1.000000000000 m/s. The error a
// flow/range pair actually makes is proportional, and now it appears.
TEST(OpticalFlow, WrongRangeScalesTheReconstructedVelocity)
{
  const auto c = cleanFlow();
  const double dt = 0.02;
  const double v = 1.0;
  const double truth = 1.5;

  for (const double measured : {0.75, 1.2, 2.5, 3.0}) {
    FlowInputs in = level(v, 0.0, truth, dt);
    in.measured_range_m = measured;
    const auto r = recover(flowSample(in, c, 0.0, 0.0), dt);
    // v_reconstructed = v_true * (r_measured / r_true), exactly.
    EXPECT_NEAR(r.vx, v * measured / truth, 1e-9)
      << "measured range " << measured;
    EXPECT_GT(std::abs(r.vx - v), 0.1) << "a 2x range error must be visible";
  }
}

// FAILS IF: a gyro error does not produce phantom velocity, ON EITHER AXIS. A
// stationary vehicle whose gyro disagrees with its real rotation must appear to
// be moving; that is the failure mode of the pair, and why both the raw flow
// and the gyro integral are published separately.
//
// Both axes are exercised because they were not: a mutation that swapped
// omega_true for omega_measured on the x term alone SURVIVED a version of this
// test that only inspected vx. One axis is not coverage.
TEST(OpticalFlow, GyroErrorProducesPhantomVelocityOnBothAxes)
{
  const auto c = cleanFlow();
  const double dt = 0.02;
  const double range = 1.5;

  // Gyro error about body y appears as phantom velocity along body x.
  FlowInputs y_err = level(0.0, 0.0, range, dt);      // genuinely stationary
  y_err.omega_measured = Eigen::Vector3d(0.0, 0.5, 0.0);
  const auto ry = recover(flowSample(y_err, c, 0.0, 0.0), dt);
  EXPECT_NEAR(ry.vx, -0.5 * range, 1e-9);
  EXPECT_NEAR(ry.vy, 0.0, 1e-9);

  // ...and about body x as phantom velocity along body y, with the sign the
  // message's convention implies.
  FlowInputs x_err = level(0.0, 0.0, range, dt);
  x_err.omega_measured = Eigen::Vector3d(0.4, 0.0, 0.0);
  const auto rx = recover(flowSample(x_err, c, 0.0, 0.0), dt);
  EXPECT_NEAR(rx.vy, 0.4 * range, 1e-9);
  EXPECT_NEAR(rx.vx, 0.0, 1e-9);
}

// FAILS IF: Eigen stops constraining AngleAxis to [0, pi]. bodyRateFromQuaternions
// relies on exactly that to absorb a quaternion sheet flip -- it has no sign
// check of its own, because one made no difference. This pins the assumption so
// that if it ever changes, the reason the rate function is safe changes visibly
// rather than silently.
TEST(BodyRate, AngleAxisCanonicalisesTheQuaternionSign)
{
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()));
  Eigen::Quaterniond negated = q;
  negated.coeffs() = -negated.coeffs();

  const Eigen::AngleAxisd plain(q);
  const Eigen::AngleAxisd flipped(negated);
  EXPECT_LE(plain.angle(), M_PI);
  EXPECT_LE(flipped.angle(), M_PI);
  EXPECT_GE(flipped.angle(), 0.0);
  // Same rotation, so the same axis-angle -- that is the whole guarantee.
  EXPECT_NEAR(plain.angle(), flipped.angle(), 1e-12);
  EXPECT_LT((plain.axis() - flipped.axis()).norm(), 1e-12);
}

// FAILS IF: rotation is left out of the raw flow. If the raw reading were
// already compensated, a gyro/flow mismatch would be invisible -- and that
// mismatch is what the two published numbers exist to expose.
TEST(OpticalFlow, RawFlowIncludesRotation)
{
  const auto c = cleanFlow();
  FlowInputs in = level(0.0, 0.0, 1.5, 0.1);
  in.omega_true = Eigen::Vector3d(1.0, 0.0, 0.0);
  in.omega_measured = in.omega_true;
  const auto s = flowSample(in, c, 0, 0);
  EXPECT_NEAR(s.integrated_x, 0.1, 1e-12);
  EXPECT_NEAR(s.integrated_xgyro, 0.1, 1e-12);
}

// FAILS IF: an unusable reading is reported as merely poor. quality 0 must
// mean "no measurement": a consumer that fuses it with a low weight will fly
// into things.
TEST(OpticalFlow, QualityIsZeroOutsideTheUsableEnvelope)
{
  const FlowConfig c;
  EXPECT_EQ(flowQuality(0.05, 1.0, c), 0) << "below the minimum focus height";
  EXPECT_EQ(flowQuality(5.0, 1.0, c), 0) << "above the tracking limit";
  EXPECT_EQ(flowQuality(kTooFar, 1.0, c), 0) << "no ground within range";
  EXPECT_EQ(flowQuality(1.5, std::cos(0.9), c), 0) << "tilted past the FOV";
  EXPECT_GT(flowQuality(1.5, 1.0, c), 200) << "level and mid-band should be good";
}

// FAILS IF: quality switches off at a cliff. Real modules degrade, and a hard
// edge lets a controller oscillate across the boundary between trusting and
// ignoring the sensor.
TEST(OpticalFlow, QualityFadesTowardsTheEdgesOfTheBand)
{
  const FlowConfig c;
  const auto mid = flowQuality(1.5, 1.0, c);
  EXPECT_LT(flowQuality(0.15, 1.0, c), mid);
  EXPECT_LT(flowQuality(2.9, 1.0, c), mid);
  EXPECT_GT(flowQuality(0.15, 1.0, c), 0) << "just inside is degraded, not dead";
  EXPECT_GT(flowQuality(2.9, 1.0, c), 0);
}

// FAILS IF: a reading a consumer cannot use is published with a usable
// quality. Each of these turns the documented formula into a NaN or a
// division by zero, so the contract "quality 0 means do not use" has to hold
// for all of them -- a zero dt happens on the FIRST publish of every run, and
// a negative one after any world reset.
TEST(OpticalFlow, UnusableReadingsAreAlwaysQualityZero)
{
  const auto c = cleanFlow();

  FlowInputs zero_dt = level(1.0, 0.0, 1.5, 0.0);
  EXPECT_EQ(flowSample(zero_dt, c, 0, 0).quality, 0) << "dt == 0 gives 0/0";

  FlowInputs back_dt = level(1.0, 0.0, 1.5, -0.01);
  EXPECT_EQ(flowSample(back_dt, c, 0, 0).quality, 0) << "time went backwards";

  FlowInputs no_range = level(1.0, 0.0, 1.5, 0.02);
  no_range.measured_range_m = kTooFar;
  const auto s = flowSample(no_range, c, 0, 0);
  EXPECT_EQ(s.quality, 0) << "cannot scale flow by an infinite range";
  EXPECT_EQ(s.integrated_x, 0.0);
  EXPECT_EQ(s.integrated_y, 0.0);

  // ...and the gyro integrals survive: a different sensor, still working.
  FlowInputs with_gyro = no_range;
  with_gyro.omega_measured = Eigen::Vector3d(0.5, 0.25, 0.0);
  const auto g = flowSample(with_gyro, c, 0, 0);
  EXPECT_NEAR(g.integrated_xgyro, 0.5 * 0.02, 1e-12);
  EXPECT_NEAR(g.integrated_ygyro, 0.25 * 0.02, 1e-12);
}

// ---------------------------------------------------------------------------
// Body rate from attitude, across the quaternion double cover.
// ---------------------------------------------------------------------------

// FAILS IF: a sheet flip is read as a real rotation. This is the exact bug
// that made Gazebo report 626 rad/s on a vehicle turning at 1.8: q and -q are
// the same attitude, so the rate between them must be the rate between the
// attitudes, not between the number pairs.
TEST(BodyRate, SignFlippedQuaternionsGiveTheSameRateAsUnflipped)
{
  const double dt = 0.004;
  const Eigen::Quaterniond a(Eigen::AngleAxisd(0.30, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond b(Eigen::AngleAxisd(0.30 + 1.8 * dt,
                                               Eigen::Vector3d::UnitZ()));

  const Eigen::Vector3d plain = bodyRateFromQuaternions(a, b, dt);
  Eigen::Quaterniond flipped = b;
  flipped.coeffs() = -flipped.coeffs();          // same rotation, other sheet
  const Eigen::Vector3d across = bodyRateFromQuaternions(a, flipped, dt);

  EXPECT_NEAR(plain.z(), 1.8, 1e-6) << "should recover the true yaw rate";
  EXPECT_LT((plain - across).norm(), 1e-9)
    << "a sheet flip changed the reported rate";
  // The failure being guarded against was a rate of hundreds of rad/s.
  EXPECT_LT(across.norm(), 5.0);
}

// FAILS IF: the sign or axis of the recovered rate is wrong. A damping term
// fed a sign-flipped rate drives divergence rather than damping it.
TEST(BodyRate, RecoversRateOnEachAxisWithTheRightSign)
{
  const double dt = 0.01;
  for (int axis = 0; axis < 3; ++axis) {
    for (const double rate : {-2.0, 1.5}) {
      Eigen::Vector3d unit = Eigen::Vector3d::Zero();
      unit(axis) = 1.0;
      const Eigen::Quaterniond a = Eigen::Quaterniond::Identity();
      const Eigen::Quaterniond b(Eigen::AngleAxisd(rate * dt, unit));
      const Eigen::Vector3d got = bodyRateFromQuaternions(a, b, dt);
      EXPECT_NEAR(got(axis), rate, 1e-6) << "axis " << axis << " rate " << rate;
      EXPECT_LT((got - unit * rate).norm(), 1e-6);
    }
  }
}

// FAILS IF: a zero or negative interval produces a division by zero. It
// happens on the first sample of every run.
TEST(BodyRate, NonPositiveIntervalYieldsZeroNotInfinity)
{
  const Eigen::Quaterniond a = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond b(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()));
  EXPECT_EQ(bodyRateFromQuaternions(a, b, 0.0), Eigen::Vector3d::Zero());
  EXPECT_EQ(bodyRateFromQuaternions(a, b, -0.01), Eigen::Vector3d::Zero());
}

// FAILS IF: ordinary coordinated flight is reported as poor quality. A 42 deg
// field of view is barely troubled by 15-20 degrees of bank, and a model that
// halved quality there would have consumers rejecting readings exactly when
// they are most useful. A linear tilt penalty did precisely that -- it gave
// 107/255 at the 17 degrees this simulator's own demo lap flies.
TEST(OpticalFlow, ModerateBankStaysUsable)
{
  const FlowConfig c;
  const auto level = flowQuality(1.5, 1.0, c);
  for (const double deg : {5.0, 10.0, 17.0, 20.0}) {
    const double q = flowQuality(1.5, std::cos(deg * M_PI / 180.0), c);
    EXPECT_GT(q, 150) << deg << " deg of bank should stay usable";
    EXPECT_LE(q, level) << "tilt must never improve quality";
  }
  // ...and it must still collapse as the ground leaves the frame, or the
  // envelope limit would be decoration.
  EXPECT_LT(flowQuality(1.5, std::cos(0.50), c), 60);
  EXPECT_EQ(flowQuality(1.5, std::cos(0.60), c), 0);
}
