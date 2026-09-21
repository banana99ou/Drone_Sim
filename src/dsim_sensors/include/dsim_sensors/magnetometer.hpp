#pragma once
#include <cmath>

#include <Eigen/Dense>

namespace dsim_sensors
{

/// A three-axis magnetometer, the compass in a consumer IMU (AK8963 /
/// LIS3MDL class).
///
/// NOT a Gazebo sensor, although Gazebo ships one. It is synthesised from the
/// truth attitude, like the rangefinder and the flow, for two reasons. The
/// model is then a pure function of a rotation matrix: it compiles on the
/// host, exact tests can pin it, and the mutation harness can break it on
/// purpose -- none of which is possible for a plugin running inside the
/// simulator. And the error that actually decides whether a compass is
/// usable, a hard-iron offset from the vehicle's own magnetised parts, is
/// modelled explicitly here where it can be configured and reasoned about;
/// Gazebo's sensor offers Gaussian noise and nothing else.
///
/// Simplified deliberately, and the simplifications are stated so you know
/// where the model stops being usable. The Earth's field is one fixed vector
/// for the whole world, which is true over any distance a quadrotor flies.
/// There is no soft-iron distortion (an ellipsoid in place of the sphere,
/// needing a matrix calibration rather than a subtraction) and no
/// motor-current field (which would vary with thrust). So this will exercise
/// a heading estimator's noise handling and its bias handling, but not a
/// calibration routine meant to survive a real airframe.
struct MagConfig
{
  // These member defaults are for TESTS only. sensors_node declares its
  // parameters without defaults, so the running sensor always takes these
  // numbers from config/drone.yaml -- which scripts/gen_assets.py generates.
  // A default in the node would have been a second copy of a generated
  // value, and the copy that drifts is always the one nobody tests.

  /// White noise per axis, in tesla. 0.5 uT is what a consumer part achieves
  /// at 50 Hz, and it is worth knowing what it costs: a heading error of
  /// noise_t / |B_horizontal|, about one degree in a 30 uT horizontal field.
  double noise_t {5.0e-07};
  /// Magnitude of a fixed BODY-frame offset, in tesla: the field of the
  /// vehicle's own magnetised parts. It turns with the vehicle, so no single
  /// reading can tell it from the Earth's field -- which is what makes it the
  /// interesting error, and why it is a separate number from the noise.
  /// Direction is drawn once per run; see magHardIron(). Zero by default, so
  /// the compass starts ideal and the calibration problem is opt-in.
  double hard_iron_t {0.0};
  /// The Earth's field in the world frame. World +x is magnetic north -- the
  /// declination is folded in, there is no separate true north here -- and
  /// z is up, so the vertical component is NEGATIVE in the northern
  /// hemisphere. The default dips 53 degrees, roughly Korea.
  Eigen::Vector3d field_world_t {3.0e-05, 0.0, -4.0e-05};
};

/// The Earth's field as the body sees it: the world field rotated INTO the
/// body frame.
///
/// R_wb is the truth attitude, world <- body, the same matrix the rangefinder
/// and the flow take, so its transpose is what turns a world vector into a
/// body one. That transpose is the entire sensor: a magnetometer is only a
/// compass because the field it sees moves as the vehicle turns. Getting the
/// direction wrong, R for R^T, is a compass that turns the wrong way -- and a
/// heading estimator cannot notice on its own, because every reading it gets
/// is still a perfectly plausible field.
inline Eigen::Vector3d magTrueField(
  const Eigen::Matrix3d & R_wb, const Eigen::Vector3d & field_world)
{
  return R_wb.transpose() * field_world;
}

/// A hard-iron offset of the configured magnitude in a random direction.
///
/// gauss_unit is three draws from N(0, 1): a normalised Gaussian vector is
/// uniform on the sphere, which a normalised uniform cube is not. Drawn once
/// at startup and then held, because a hard-iron offset is a property of the
/// airframe -- it is the same in every reading, and that constancy is exactly
/// what a calibration exploits. A fresh direction every sample would just be
/// more noise, and the wrong kind.
///
/// Exactly zero when the magnitude is, not merely small: the default
/// configuration promises an ideal compass, and an estimator is entitled to
/// test against that promise.
inline Eigen::Vector3d magHardIron(double hard_iron_t, const Eigen::Vector3d & gauss_unit)
{
  if (hard_iron_t <= 0.0) {return Eigen::Vector3d::Zero();}
  const double n = gauss_unit.norm();
  // A zero draw has no direction. Three N(0, 1) draws are never all zero in
  // practice, but a caller passing zeros must get a bias of the configured
  // magnitude back, not a NaN and not a silent zero.
  if (n < 1e-12) {return Eigen::Vector3d(hard_iron_t, 0.0, 0.0);}
  return gauss_unit * (hard_iron_t / n);
}

/// Apply the sensor's error model to the true body-frame field.
///
/// gauss_unit is three draws from N(0, 1), passed in rather than generated
/// here so this stays a pure function: the tests pin the noise model with
/// exact draws, which is impossible if the randomness lives inside. Noise is
/// independent per axis at one sigma for all three -- the model of a
/// three-axis part with one ADC per axis. The hard iron is added in the BODY
/// frame, after the rotation, because that is the frame it lives in.
inline Eigen::Vector3d magMeasure(
  const Eigen::Vector3d & true_body, const MagConfig & c,
  const Eigen::Vector3d & hard_iron_body, const Eigen::Vector3d & gauss_unit)
{
  return true_body + hard_iron_body + c.noise_t * gauss_unit;
}

}  // namespace dsim_sensors
