// These tests are written as invariants: statements that cannot be false if
// the implementation is correct. Each comment says what would make it fail.

#include <gtest/gtest.h>
#include <cmath>
#include "dsim_control/mixer.hpp"
#include "dsim_control/se3_controller.hpp"
#include "dsim_control/angles.hpp"

using namespace dsim_control;

namespace
{
// Mirrors PARAMS in scripts/gen_assets.py. These tests check the CODE, not the
// shipped config -- whether the config itself describes a flyable vehicle is
// asserted by check_flyable() in that generator, which runs on every
// regeneration and refuses to emit an airframe that cannot hover.
constexpr double kArm = 0.20;
constexpr double kMoment = 0.016;
constexpr double kMotor = 9.0e-06;
constexpr double kMaxRot = 1000.0;
constexpr double kMass = 1.5;
constexpr double kG = 9.80665;

Mixer makeMixer() {return Mixer(kArm, kMoment, kMotor, kMaxRot);}

State hoverState()
{
  State s;
  s.position = Eigen::Vector3d(0, 0, 1.5);
  return s;
}
}  // namespace

// FAILS IF: the allocation matrix and its inverse disagree — i.e. the geometry
// or spin signs were edited in one place and not the other.
TEST(Mixer, InverseActuallyInverts)
{
  const auto m = makeMixer();
  const std::vector<Eigen::Vector4d> probes = {
    {14.7, 0, 0, 0}, {14.7, 0.5, 0, 0}, {14.7, 0, 0.5, 0},
    {14.7, 0, 0, 0.2}, {20.0, -0.3, 0.4, -0.1}, {5.0, 1.0, -1.0, 0.05},
  };
  for (const auto & w : probes) {
    EXPECT_LT(m.roundTripError(w), 1e-9) << "wrench " << w.transpose();
  }
}

// FAILS IF: a rotor column is duplicated or mis-signed. Hover is the one
// wrench whose answer is known without any derivation: all four equal.
TEST(Mixer, HoverIsSymmetric)
{
  const auto m = makeMixer();
  const Eigen::Vector4d f = m.rotorThrusts({kMass * kG, 0, 0, 0});
  const double expected = kMass * kG / 4.0;
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(f(i), expected, 1e-9) << "rotor " << i;
  }
}

// FAILS IF: the roll axis is swapped with pitch, or the y-offsets have the
// wrong sign. Body frame is FLU, so +tau_x must lift the +y (left) side:
// rotors 0 and 3 sit at y = +d.
TEST(Mixer, PositiveRollLoadsTheLeftRotors)
{
  const auto m = makeMixer();
  const Eigen::Vector4d f = m.rotorThrusts({kMass * kG, 0.4, 0, 0});
  EXPECT_GT(f(0), f(2));   // front-left  > front-right
  EXPECT_GT(f(3), f(1));   // back-left   > back-right
}

// FAILS IF: pitch sign is inverted. +tau_y lifts the -x (rear) side, so the
// rear rotors (1, 3) must work harder.
TEST(Mixer, PositivePitchLoadsTheRearRotors)
{
  const auto m = makeMixer();
  const Eigen::Vector4d f = m.rotorThrusts({kMass * kG, 0, 0.4, 0});
  EXPECT_GT(f(1), f(0));
  EXPECT_GT(f(3), f(2));
}

// FAILS IF: the reaction-torque sign disagrees with Gazebo's motor plugin.
// A rotor's reaction torque opposes its own spin, so +tau_z (CCW about +z)
// comes from spinning the CW rotors (2, 3) harder.
TEST(Mixer, PositiveYawLoadsTheClockwiseRotors)
{
  const auto m = makeMixer();
  const Eigen::Vector4d f = m.rotorThrusts({kMass * kG, 0, 0, 0.15});
  EXPECT_GT(f(2), f(0));
  EXPECT_GT(f(3), f(1));
}

// FAILS IF: thrust/speed conversion is not a true inverse pair.
TEST(Mixer, ThrustSpeedRoundTrip)
{
  const auto m = makeMixer();
  for (double thrust : {0.5, 1.0, 3.678, 7.0}) {
    EXPECT_NEAR(m.speedToThrust(m.thrustToSpeed(thrust)), thrust, 1e-9);
  }
}

// FAILS IF: the vehicle cannot hover with margin. This is a modelling check,
// not a code check: it catches a parameter edit that makes the drone unflyable.
TEST(Mixer, ThrustBudgetAllowsHoverWithMargin)
{
  const auto m = makeMixer();
  const double twr = 4.0 * m.maxThrustPerRotor() / (kMass * kG);
  EXPECT_GT(twr, 1.5) << "thrust-to-weight too low to manoeuvre";
}

// FAILS IF: gravity compensation is missing or scaled wrong. At rest on the
// reference, the only correct answer is exactly hover thrust and zero torque.
TEST(SE3Controller, PerfectHoverCommandsExactlyWeight)
{
  SE3Controller c{Gains{}};
  const State s = hoverState();
  Reference r;
  r.position = s.position;

  const Eigen::Vector4d w = c.compute(s, r);
  EXPECT_NEAR(w(0), kMass * kG, 1e-6);
  EXPECT_NEAR(w(1), 0.0, 1e-9);
  EXPECT_NEAR(w(2), 0.0, 1e-9);
  EXPECT_NEAR(w(3), 0.0, 1e-9);
}

// FAILS IF: the position error sign is flipped — the classic bug that makes a
// drone accelerate away from its setpoint. Below the reference must mean
// more thrust than weight.
TEST(SE3Controller, BelowSetpointCommandsMoreThanWeight)
{
  SE3Controller c{Gains{}};
  State s = hoverState();
  s.position.z() -= 0.5;
  Reference r;
  r.position = Eigen::Vector3d(0, 0, 1.5);

  EXPECT_GT(c.compute(s, r)(0), kMass * kG);
}

// FAILS IF: the horizontal error sign is flipped. A setpoint ahead in +x must
// produce a desired force with a +x component (nose-down pitch).
TEST(SE3Controller, SetpointAheadTiltsForward)
{
  SE3Controller c{Gains{}};
  const State s = hoverState();
  Reference r;
  r.position = s.position + Eigen::Vector3d(1.0, 0.0, 0.0);

  ControlDebug dbg;
  c.compute(s, r, &dbg);
  EXPECT_GT(dbg.desired_force.x(), 0.0);
  EXPECT_NEAR(dbg.desired_force.y(), 0.0, 1e-9);
}

// FAILS IF: the tilt clamp does not engage. A huge error must not produce an
// arbitrarily large bank angle.
TEST(SE3Controller, TiltIsClamped)
{
  Gains g;
  g.max_tilt_rad = 0.5;
  SE3Controller c{g};
  const State s = hoverState();
  Reference r;
  r.position = s.position + Eigen::Vector3d(100.0, 0.0, 0.0);

  ControlDebug dbg;
  c.compute(s, r, &dbg);
  EXPECT_TRUE(dbg.tilt_clamped);
  EXPECT_LE(dbg.commanded_tilt_rad, 0.5 + 1e-9);
}

// FAILS IF: acceleration feedforward is dropped. Two states identical except
// for reference acceleration must produce different thrust.
TEST(SE3Controller, AccelerationFeedforwardIsUsed)
{
  SE3Controller c{Gains{}};
  const State s = hoverState();
  Reference r0;
  r0.position = s.position;
  Reference r1 = r0;
  r1.acceleration = Eigen::Vector3d(0, 0, 2.0);

  EXPECT_NEAR(c.compute(s, r1)(0) - c.compute(s, r0)(0), kMass * 2.0, 1e-6);
}

// FAILS IF: yaw interpolation goes the long way around the circle. Crossing
// the +/-pi seam must take the 20-degree path, not the 340-degree one.
TEST(TrajectoryBuffer, YawInterpolationTakesTheShortArc)
{
  const double a = 3.0;            // ~172 deg
  const double b = -3.0;           // ~-172 deg
  const double mid = interpolateAngle(a, b, 0.5);
  // Shortest arc passes through +/-pi, not through zero.
  EXPECT_GT(std::abs(mid), 3.0);
}

// FAILS IF: the arm length is wrong, or the allocation does not actually scale
// torque with geometry. The previous tests could not catch this: they compare
// rotors against each other, and the round-trip check uses the same matrix on
// both sides, so a wrong arm length is self-consistent and invisible.
//
// This asserts against physics derived independently of the implementation:
//   tau_x = d * ((f0 + f3) - (f1 + f2))   with d = L / sqrt(2)
// so the left/right thrust differential MUST equal tau_x / d exactly.
TEST(Mixer, RollTorqueMatchesArmLengthAnalytically)
{
  const auto m = makeMixer();
  const double d = kArm / std::sqrt(2.0);
  const double tau_x = 0.4;

  const Eigen::Vector4d f = m.rotorThrusts({kMass * kG, tau_x, 0, 0});
  const double differential = (f(0) + f(3)) - (f(1) + f(2));
  EXPECT_NEAR(differential, tau_x / d, 1e-9);
}

// Same idea for yaw, against the moment constant instead of the arm length:
//   tau_z = c * ((f2 + f3) - (f0 + f1))
TEST(Mixer, YawTorqueMatchesMomentConstantAnalytically)
{
  const auto m = makeMixer();
  const double tau_z = 0.15;

  const Eigen::Vector4d f = m.rotorThrusts({kMass * kG, 0, 0, tau_z});
  const double differential = (f(2) + f(3)) - (f(0) + f(1));
  EXPECT_NEAR(differential, tau_z / kMoment, 1e-9);
}

// FAILS IF: geometry is hard-coded rather than read from the constructor.
// Physical impossibility check: a longer arm gives more leverage, so the SAME
// torque cannot require MORE thrust differential. This cannot be false for any
// correct implementation, whatever the actual numbers are.
TEST(Mixer, LongerArmNeedsLessThrustDifferential)
{
  const Mixer shortArm(0.15, kMoment, kMotor, kMaxRot);
  const Mixer longArm(0.30, kMoment, kMotor, kMaxRot);
  const Eigen::Vector4d w{kMass * kG, 0.4, 0, 0};

  auto differential = [&w](const Mixer & m) {
      const Eigen::Vector4d f = m.rotorThrusts(w);
      return (f(0) + f(3)) - (f(1) + f(2));
    };
  EXPECT_LT(differential(longArm), differential(shortArm));
}

// FAILS IF: total thrust leaks into the torque channels. Commanding pure torque
// on top of hover must not change the total thrust produced.
TEST(Mixer, TorqueDemandDoesNotChangeTotalThrust)
{
  const auto m = makeMixer();
  const double total = kMass * kG;
  for (const auto & w : std::vector<Eigen::Vector4d>{
      {total, 0.3, 0, 0}, {total, 0, 0.3, 0}, {total, 0, 0, 0.1}})
  {
    const Eigen::Vector4d f = m.rotorThrusts(w);
    EXPECT_NEAR(f.sum(), total, 1e-9);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
