#include "dsim_estimation/attitude_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dsim_estimation
{

namespace
{
/// Below this a vector has no usable direction. Well under any real sample
/// (an accelerometer reads ~10, a magnetometer ~1e-5 in Tesla) and well
/// above denormals.
constexpr double kTinyNorm = 1e-12;

/// Unit vector along a horizontal direction, or zero if it is degenerate.
Eigen::Vector2d horizontalUnit(const Eigen::Vector3d & v, bool * ok)
{
  const Eigen::Vector2d h(v.x(), v.y());
  const double n = h.norm();
  *ok = n > kTinyNorm;
  return *ok ? Eigen::Vector2d(h / n) : Eigen::Vector2d::Zero();
}
}  // namespace

AttitudeFilter::AttitudeFilter(const AttitudeConfig & cfg)
: cfg_(cfg)
{
  if (!(cfg.kp_accel >= 0.0) || !(cfg.ki_accel >= 0.0) ||
    !(cfg.kp_mag >= 0.0) || !(cfg.ki_mag >= 0.0))
  {
    throw std::invalid_argument("AttitudeFilter: gains must be >= 0");
  }
  if (!(cfg.gravity > 0.0) || !(cfg.max_dt_s > 0.0) || !(cfg.accel_gate_m_s2 > 0.0) ||
    !(cfg.mag_hold_s > 0.0))
  {
    throw std::invalid_argument(
      "AttitudeFilter: gravity, max_dt, mag_hold and the accel gate must be > 0");
  }
  // A field with no horizontal component has no heading in it. Refusing here
  // is better than a filter that silently applies a zero correction forever
  // while reporting that the mag is being fused.
  bool ok = false;
  horizontalUnit(cfg.field_world, &ok);
  if (!ok || !cfg.field_world.allFinite()) {
    throw std::invalid_argument(
      "AttitudeFilter: field_world must have a finite, non-zero horizontal component");
  }
}

bool AttitudeFilter::initialise(
  const Eigen::Vector3d & accel_body, const Eigen::Vector3d * mag_body)
{
  if (!accel_body.allFinite()) {return false;}
  const double a = accel_body.norm();
  if (std::abs(a - cfg_.gravity) > cfg_.accel_gate_m_s2) {return false;}

  // The specific force at rest points along world +z. The smallest rotation
  // taking the measured direction onto +z is a pure tilt (its axis is
  // horizontal), which fixes roll and pitch and leaves yaw to the mag.
  const Eigen::Vector3d up_body = accel_body / a;
  Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(up_body, Eigen::Vector3d::UnitZ());
  q.normalize();
  // A pure tilt still carries a small ZYX yaw when roll and pitch are both
  // non-zero. Zero it explicitly, so that "no mag" means yaw exactly 0 rather
  // than whatever the tilt happened to leave -- a defined starting point is
  // what makes gyro-only drift measurable. Rotating about WORLD z leaves the
  // body-frame gravity direction, the thing the accelerometer fixed, alone.
  const double residual_yaw = rollPitchYawOf(q).z();
  q = Eigen::Quaterniond(Eigen::AngleAxisd(-residual_yaw, Eigen::Vector3d::UnitZ())) * q;

  q_ = q.normalized();
  if (mag_body != nullptr) {
    // Heading from the mag, tilt-compensated with the attitude just found.
    // The error is "how far the estimate must turn about world z", so it is
    // applied about world z: a pre-multiplication.
    const double err = magHeadingError(*mag_body);
    if (std::isfinite(err)) {
      q_ = (Eigen::Quaterniond(Eigen::AngleAxisd(err, Eigen::Vector3d::UnitZ())) * q_).normalized();
    }
  }
  bias_.setZero();
  accel_error_.setZero();
  accel_pending_ = false;
  mag_error_z_ = 0.0;
  have_mag_ = false;
  mag_age_s_ = 0.0;
  last_ = AttitudeStepInfo{};
  initialised_ = true;
  return true;
}

void AttitudeFilter::predict(const Eigen::Vector3d & gyro_body, double dt)
{
  // Take the queued accelerometer error whatever happens below: it belongs
  // to the sample interval that is ending now, and must not leak into the
  // next one.
  const Eigen::Vector3d e_acc = accel_error_;
  const bool accel_now = accel_pending_;
  accel_error_.setZero();
  accel_pending_ = false;
  last_ = AttitudeStepInfo{};

  if (!initialised_ || !gyro_body.allFinite()) {return;}
  if (!(dt > 0.0) || dt > cfg_.max_dt_s) {
    // A stall, a reset, or the first sample: no interval to integrate over,
    // and applying a correction as a rate needs a dt to multiply by.
    return;
  }

  // The held mag error, rotated into the current body frame. It expires:
  // a compass that stopped publishing must stop steering yaw, rather than
  // pushing forever towards its last opinion.
  mag_age_s_ += dt;
  const bool mag_now = have_mag_ && mag_age_s_ <= cfg_.mag_hold_s;
  const Eigen::Vector3d e_mag = mag_now ?
    Eigen::Vector3d(q_.conjugate() * Eigen::Vector3d(0.0, 0.0, mag_error_z_)) :
    Eigen::Vector3d::Zero();

  // Bias integrator: the error is the direction the attitude has to turn to
  // agree with the references, and a persistent error of that sign means the
  // gyro has been reading too LOW along it -- so the bias moves the other
  // way. A flipped sign here diverges, which is what the test pins. Two
  // gains, because the accelerometer's integrator winds up in a turn and the
  // magnetometer's does not (see AttitudeConfig).
  bias_ -= (cfg_.ki_accel * e_acc + cfg_.ki_mag * e_mag) * dt;

  // Corrected rate: measured, minus the bias, plus the gain-weighted pull
  // towards the vector references. This is the entire complementary filter:
  // the gyro provides the high-frequency attitude, the references the
  // low-frequency anchor, and the kp values set the crossovers.
  const Eigen::Vector3d omega = gyro_body - bias_ + cfg_.kp_accel * e_acc + cfg_.kp_mag * e_mag;

  // Integrate in the BODY frame: q <- q * exp(omega dt). The right-hand
  // multiplication is what makes a body-x rate roll the vehicle about ITS
  // x axis rather than the world's.
  const double angle = omega.norm() * dt;
  if (angle > kTinyNorm) {
    const Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, omega / omega.norm()));
    q_ = (q_ * dq).normalized();
  }
  last_.accel_applied = accel_now;
  last_.mag_applied = mag_now;
}

bool AttitudeFilter::correctAccel(const Eigen::Vector3d & accel_body)
{
  if (!initialised_ || !accel_body.allFinite()) {return false;}
  const double a = accel_body.norm();
  if (a < kTinyNorm || std::abs(a - cfg_.gravity) > cfg_.accel_gate_m_s2) {return false;}

  // Measured "up" in the body frame, against the estimate's prediction of
  // it. Their cross product is the small rotation, in the body frame, that
  // would align the prediction with the measurement: e = v_meas x v_pred.
  //
  // Only the DIRECTION is used. Normalising throws away the one thing the
  // magnitude could have told us -- that the vehicle is accelerating -- but
  // keeping it would scale the correction by an unrelated quantity.
  const Eigen::Vector3d v_meas = accel_body / a;
  const Eigen::Vector3d v_pred = q_.conjugate() * Eigen::Vector3d::UnitZ();
  accel_error_ = v_meas.cross(v_pred);
  accel_pending_ = true;
  return true;
}

double AttitudeFilter::magHeadingError(const Eigen::Vector3d & mag_body) const
{
  if (!mag_body.allFinite()) {return std::numeric_limits<double>::quiet_NaN();}
  // Tilt compensation: rotate the sample into the world frame with the
  // current roll/pitch, THEN take its horizontal direction. Reading a heading
  // straight off the body x/y components is wrong by an amount that grows
  // with tilt and with the field's dip -- and the shipped field is mostly
  // vertical (dip 53 deg), so at 25 deg of bank the raw heading is off by
  // tens of degrees. The test at a non-zero tilt exists to catch a version
  // that drops this line.
  const Eigen::Vector3d m_world = q_ * mag_body;
  bool ok_m = false, ok_r = false;
  const Eigen::Vector2d h_meas = horizontalUnit(m_world, &ok_m);
  const Eigen::Vector2d h_ref = horizontalUnit(cfg_.field_world, &ok_r);
  if (!ok_m || !ok_r) {return std::numeric_limits<double>::quiet_NaN();}
  // Signed angle from the measured horizontal direction to the reference:
  // positive means the estimate must turn counter-clockwise (about +z).
  const double cross = h_meas.x() * h_ref.y() - h_meas.y() * h_ref.x();
  const double dot = h_meas.dot(h_ref);
  return std::atan2(cross, dot);
}

bool AttitudeFilter::correctMag(const Eigen::Vector3d & mag_body)
{
  if (!initialised_ || cfg_.kp_mag <= 0.0) {return false;}
  const double err = magHeadingError(mag_body);
  if (!std::isfinite(err)) {return false;}

  // The correction is a rotation about the WORLD vertical only. Restricting
  // it to the vertical is deliberate: the accelerometer owns roll and pitch,
  // and a mag with a wrong field model must be able to corrupt only yaw.
  //
  // sin(err) rather than err, so this is the same cross-product form as the
  // accelerometer term and stays bounded for a 180 deg error.
  mag_error_z_ = std::sin(err);
  have_mag_ = true;
  mag_age_s_ = 0.0;
  return true;
}

void AttitudeFilter::reset()
{
  q_ = Eigen::Quaterniond::Identity();
  bias_.setZero();
  accel_error_.setZero();
  accel_pending_ = false;
  mag_error_z_ = 0.0;
  have_mag_ = false;
  mag_age_s_ = 0.0;
  last_ = AttitudeStepInfo{};
  initialised_ = false;
}

Eigen::Vector3d AttitudeFilter::rollPitchYaw() const
{
  return rollPitchYawOf(q_);
}

void AttitudeFilter::setQuaternionForTest(const Eigen::Quaterniond & q)
{
  q_ = q.normalized();
}

Eigen::Vector3d rollPitchYawOf(const Eigen::Quaterniond & q)
{
  const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  // ZYX: yaw is the heading of the body x axis, the same definition the
  // controller uses for its yaw setpoint, so the two never disagree about
  // what "yaw" means.
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
  const double roll = std::atan2(R(2, 1), R(2, 2));
  return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Quaterniond quaternionFromRollPitchYaw(double roll, double pitch, double yaw)
{
  return Eigen::Quaterniond(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).normalized();
}

}  // namespace dsim_estimation
