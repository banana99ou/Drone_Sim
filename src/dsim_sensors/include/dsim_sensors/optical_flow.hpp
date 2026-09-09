#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

#include <Eigen/Dense>

namespace dsim_sensors
{

/// A downward optical-flow sensor, PMW3901 class.
///
/// Such a sensor measures how far the IMAGE moved, as an angle, and cannot
/// distinguish translation from rotation. It knows nothing about height, so its
/// output only becomes a velocity once something else supplies a range -- which
/// is why these modules are always paired with a rangefinder.
///
/// THE POINT OF THE MODEL, and the bug it used to have.
///
/// The image is swept by the vehicle's TRUE motion over its TRUE height. What
/// a consumer gets to divide by is the rangefinder's MEASURED range, and what
/// it gets to subtract is the gyro's MEASURED rate. Those are different
/// numbers, and their difference is the entire error budget of a flow/range
/// pair:
///
///     v_reconstructed = v_true * (r_measured / r_true)
///                       + (omega_true - omega_measured) * r_measured
///                       + flow_noise * r_measured
///
/// An earlier version generated the flow from the MEASURED range and the
/// MEASURED gyro and then reported those same values. The consumer's
/// documented reconstruction cancelled them algebraically: a rangefinder
/// reading 2.5 m instead of 1.5 m still recovered the velocity to twelve
/// decimal places, and a gyro 5 rad/s out did too. The model simulated
/// neither of the two errors it existed to simulate, and its own comments
/// claimed the opposite.
///
/// Hence the true/measured split in FlowInputs, and hence the tests that feed
/// a deliberately wrong range and require the reconstruction to be wrong by
/// the matching factor. Those tests could not exist before.
struct FlowConfig
{
  // These member defaults are for TESTS only. sensors_node declares its
  // parameters without defaults, so the running sensor always takes these
  // numbers from config/drone.yaml -- which scripts/gen_assets.py generates.
  // A default in the node would have been a second copy of a generated
  // value, and the copy that drifts is always the one nobody tests.

  double noise_rad_s {0.02};
  /// Below this the ground is inside the module's minimum focus distance;
  /// above it there is not enough texture contrast to track.
  double min_height_m {0.10};
  double max_height_m {3.00};
  /// Past this tilt the ground leaves the field of view entirely.
  double max_tilt_rad {0.52};
};

/// Everything one flow reading is generated from.
///
/// The true/measured split is the substance of the model, not bookkeeping. A
/// caller that passes the same value for both has switched off the error it
/// was trying to simulate.
struct FlowInputs
{
  Eigen::Vector3d v_body {Eigen::Vector3d::Zero()};          ///< true, body frame
  Eigen::Vector3d omega_true {Eigen::Vector3d::Zero()};      ///< true body rate
  Eigen::Vector3d omega_measured {Eigen::Vector3d::Zero()};  ///< what the gyro says
  double true_range_m {0.0};       ///< true slant range: this sweeps the image
  double measured_range_m {0.0};   ///< what the rangefinder reports
  double cos_tilt {1.0};
  double dt {0.0};
};

struct FlowSample
{
  double integrated_x {0.0};        ///< rad, flow about body +x
  double integrated_y {0.0};        ///< rad, flow about body +y
  double integrated_xgyro {0.0};    ///< rad, MEASURED gyro over the same dt
  double integrated_ygyro {0.0};
  double ground_distance_m {0.0};   ///< the MEASURED range
  std::uint8_t quality {0};
};

/// Quality figure for a reading, 0 meaning unusable.
///
/// 0 is not "poor", it is "no measurement": a consumer that treats it as a
/// weak reading and fuses it anyway will fly into things. It fades towards the
/// edges of the height band rather than switching off at a cliff, because real
/// modules degrade and a hard edge would let a controller oscillate across the
/// boundary between trusting and ignoring the sensor.
///
/// Judged on the TRUE geometry, because that is what decides whether the module
/// can see texture. The flow chip has no idea what the rangefinder read.
inline std::uint8_t flowQuality(
  double true_range_m, double cos_tilt, const FlowConfig & c)
{
  if (!std::isfinite(true_range_m)) {return 0;}
  if (true_range_m < c.min_height_m || true_range_m > c.max_height_m) {return 0;}
  if (cos_tilt < std::cos(c.max_tilt_rad)) {return 0;}

  // Distance from the nearer edge of the usable band, as a fraction of a 20%
  // margin: full marks in the middle, fading out at the ends.
  const double span = c.max_height_m - c.min_height_m;
  const double margin = 0.2 * span;
  const double edge = std::min(true_range_m - c.min_height_m,
                               c.max_height_m - true_range_m);
  const double band = margin > 0.0 ? std::min(1.0, edge / margin) : 1.0;

  // Tilt costs quality, but not linearly. These modules have a field of view
  // around 42 degrees, so moderate bank barely moves the ground within it and
  // the loss is dominated by foreshortening; quality collapses only as the
  // ground approaches the edge of the frame. A linear penalty implied 50%
  // quality at 15 degrees of bank, which is ordinary coordinated flight -- the
  // sensor would have looked unusable exactly when it is most needed, and
  // scripts/check_sensors.py caught it doing so at 17 degrees.
  const double tilt = std::acos(std::min(1.0, cos_tilt));
  const double frac = tilt / c.max_tilt_rad;
  const double tilt_term = 1.0 - frac * frac * frac;

  const double q = 255.0 * band * std::max(0.0, tilt_term);
  return static_cast<std::uint8_t>(std::lround(std::clamp(q, 0.0, 255.0)));
}

/// One flow reading over an interval of dt.
///
/// nx, ny are draws from N(0, 1), passed in so this stays a pure function: the
/// tests pin the noise model with exact draws, which is impossible if the
/// randomness lives inside.
///
/// Sign convention: flow about +x is driven by motion along +y, negatively.
/// Stated because every vendor differs, and a sign error in a flow sensor
/// produces a vehicle that accelerates away from where it is trying to hold.
inline FlowSample flowSample(
  const FlowInputs & in, const FlowConfig & c, double nx, double ny)
{
  FlowSample s;
  s.ground_distance_m = in.measured_range_m;
  s.quality = flowQuality(in.true_range_m, in.cos_tilt, c);
  s.integrated_xgyro = in.omega_measured.x() * in.dt;
  s.integrated_ygyro = in.omega_measured.y() * in.dt;

  // A reading a consumer cannot scale is unusable however good the image was.
  // Dividing by a non-finite range gives a non-finite velocity, and the
  // message contract promises quality 0 means exactly "do not use this" -- so
  // the sensor must not publish a confident quality alongside an unusable
  // range, or a zero dt that turns the documented formula into 0/0.
  if (!std::isfinite(in.measured_range_m) || in.measured_range_m <= 0.0 ||
      in.dt <= 0.0 || in.true_range_m <= 0.0)
  {
    s.quality = 0;
  }
  if (s.quality == 0) {
    // The gyro integrals stay: they come from a different sensor, which is
    // still working. The flow numbers do not, because there is none.
    s.integrated_x = 0.0;
    s.integrated_y = 0.0;
    return s;
  }

  // Angular rate of the image: translation over the TRUE height, plus the TRUE
  // rotation the sensor cannot separate from it.
  const double flow_rate_x = -in.v_body.y() / in.true_range_m + in.omega_true.x();
  const double flow_rate_y = in.v_body.x() / in.true_range_m + in.omega_true.y();

  s.integrated_x = (flow_rate_x + c.noise_rad_s * nx) * in.dt;
  s.integrated_y = (flow_rate_y + c.noise_rad_s * ny) * in.dt;
  return s;
}

}  // namespace dsim_sensors
