#pragma once
#include <cmath>
#include <limits>

#include <Eigen/Dense>

namespace dsim_sensors
{

/// A downward single-beam time-of-flight rangefinder, VL53L1X class.
///
/// Simplified deliberately: one ray straight down the body -z axis, one flat
/// ground plane at z = 0, no beam cone and no returns off obstacles. The
/// simplification is stated rather than hidden because it decides where the
/// model is usable: the pillars in the obstacle course are 3 m tall and the
/// vehicle flies at 1.5 m, so it never passes over one, and the ground is the
/// only thing the beam can hit. Fly higher than the obstacles and this model
/// would report the floor through a pillar.
struct TofConfig
{
  // These member defaults are for TESTS only. sensors_node declares its
  // parameters without defaults, so the running sensor always takes these
  // numbers from config/drone.yaml -- which scripts/gen_assets.py generates.
  // A default in the node would have been a second copy of a generated
  // value, and the copy that drifts is always the one nobody tests.

  double min_range_m {0.03};
  double max_range_m {4.0};
  /// Fixed part of the error, in metres.
  double noise_m {0.01};
  /// Proportional part: a ToF's accuracy degrades with distance, so the error
  /// is (noise_m + noise_frac * range). Modelling only the fixed part would
  /// make the sensor implausibly good at the far end of its range, which is
  /// exactly where a height estimator gets into trouble.
  double noise_frac {0.01};
  /// The sensor reports whole millimetres. Quantisation matters: a controller
  /// differentiating this signal sees steps, not a smooth curve.
  double resolution_m {0.001};
};

/// Out-of-range conventions, following REP 117: +inf is "too far to measure",
/// -inf is "too close". Returning the clamped limit instead would make
/// saturation indistinguishable from a real reading at exactly that distance,
/// which is how a vehicle ends up trusting a 4.0 m reading at 40 m.
constexpr double kTooFar = std::numeric_limits<double>::infinity();
constexpr double kTooClose = -std::numeric_limits<double>::infinity();

/// Geometric range from the vehicle to the ground along the body -z axis.
///
/// Not the altitude: a tilted vehicle's downward beam travels further, by
/// exactly 1/cos(tilt). Reporting altitude here would be a sensor that
/// magically knows its own attitude, and would hide the single most important
/// error a flow/range pair makes in a banked turn.
///
/// Returns kTooFar if the beam cannot reach the ground at all -- level or
/// past vertical, where it points at the sky.
inline double tofTrueRange(double altitude_m, const Eigen::Matrix3d & R)
{
  // R(2,2) is the world-z component of the body z axis, i.e. cos(tilt). The
  // beam points along body -z, so it descends only while this is positive.
  // At or below ground level there is nothing to measure and the airframe is
  // in the floor. That is "too close", NOT "too far": a height estimator
  // seeing +inf would conclude nothing is within 4 m while the ground is
  // against the vehicle -- exactly the confusion the +/-inf convention exists
  // to prevent. Reachable after a crash or a spawn at z <= 0.
  if (altitude_m <= 0.0) {return kTooClose;}
  const double cos_tilt = R(2, 2);
  if (cos_tilt <= 1e-6) {return kTooFar;}
  return altitude_m / cos_tilt;
}

/// Apply the sensor's error model to a true range.
///
/// `gauss_unit` is one draw from N(0, 1), passed in rather than generated here
/// so this stays a pure function: the tests pin the noise model with an exact
/// draw, which is impossible if the randomness is inside.
inline double tofMeasure(double true_range, const TofConfig & c, double gauss_unit)
{
  if (!std::isfinite(true_range)) {return kTooFar;}

  const double sigma = c.noise_m + c.noise_frac * true_range;
  double measured = true_range + sigma * gauss_unit;

  // Quantise before the range test, so a reading that noise pushed just past
  // the limit is rejected on the number the sensor would actually report.
  if (c.resolution_m > 0.0) {
    measured = std::round(measured / c.resolution_m) * c.resolution_m;
  }

  if (measured > c.max_range_m) {return kTooFar;}
  if (measured < c.min_range_m) {return kTooClose;}
  return measured;
}

}  // namespace dsim_sensors
