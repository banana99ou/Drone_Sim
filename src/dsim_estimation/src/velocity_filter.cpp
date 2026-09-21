#include "dsim_estimation/velocity_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dsim_estimation
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

VelocityFilter::VelocityFilter(const VelocityConfig & cfg)
: cfg_(cfg)
{
  if (!(cfg.gravity > 0.0) || !(cfg.flow_tau_s > 0.0) || !(cfg.max_dt_s > 0.0)) {
    throw std::invalid_argument("VelocityFilter: gravity, flow_tau_s and max_dt_s must be > 0");
  }
  if (!(cfg.tof_omega_rad_s > 0.0) || !(cfg.tof_zeta > 0.0)) {
    throw std::invalid_argument("VelocityFilter: tof_omega_rad_s and tof_zeta must be > 0");
  }
}

Eigen::Vector3d VelocityFilter::worldAcceleration(
  const Eigen::Matrix3d & R_est, const Eigen::Vector3d & accel_body) const
{
  // Specific force to acceleration: rotate into the world with the
  // body->world estimate, then add gravity, which points DOWN in ENU. At
  // rest the accelerometer reads +g along body z, R maps that to +g along
  // world z, and the -g cancels it to zero. That cancellation is the whole
  // point, and it is exact only when R is right -- an attitude error of
  // e radians leaks g * e of phantom horizontal acceleration, which is why
  // the flow correction exists.
  return R_est * accel_body + Eigen::Vector3d(0.0, 0.0, -cfg_.gravity);
}

void VelocityFilter::predict(
  const Eigen::Matrix3d & R_est, const Eigen::Vector3d & accel_body, double dt)
{
  last_ = pending_;
  pending_ = VelocityStepInfo{};
  if (!accel_body.allFinite() || !(dt > 0.0) || dt > cfg_.max_dt_s) {return;}

  const Eigen::Vector3d a = worldAcceleration(R_est, accel_body);
  // Altitude first, from the velocity at the start of the step, so the two
  // integrations are consistent to second order.
  if (z_valid_) {
    z_ += v_.z() * dt + 0.5 * a.z() * dt * dt;
  }
  v_ += a * dt;
}

Eigen::Vector3d VelocityFilter::flowBodyVelocity(const FlowReading & f)
{
  // quality 0 is "no measurement", not "a poor one" -- OpticalFlow.msg is
  // explicit that fusing it anyway flies into things. A non-finite range is
  // the sensor node's NaN for "the rangefinder had no return", and a
  // non-positive integration time turns the formula into a division by
  // zero. All three are refusals, not clamps.
  if (f.quality == 0 || !std::isfinite(f.ground_distance_m) || f.ground_distance_m <= 0.0 ||
    !(f.integration_time_s > 0.0))
  {
    return Eigen::Vector3d(kNaN, kNaN, kNaN);
  }
  // Exactly the reconstruction the message documents. The gyro integrals
  // are subtracted because the image moves when the vehicle rotates as well
  // as when it translates, and the sensor cannot tell which; dropping them
  // makes every roll and pitch look like a sideways velocity of
  // rate * height -- 0.5 rad/s at 1.5 m is 0.75 m/s of phantom motion.
  const double h = f.ground_distance_m;
  const double vx = (f.integrated_y - f.integrated_ygyro) / f.integration_time_s * h;
  const double vy = -(f.integrated_x - f.integrated_xgyro) / f.integration_time_s * h;
  return Eigen::Vector3d(vx, vy, 0.0);
}

bool VelocityFilter::correctFlow(const Eigen::Matrix3d & R_est, const FlowReading & flow)
{
  const Eigen::Vector3d v_flow_body = flowBodyVelocity(flow);
  if (!v_flow_body.allFinite()) {
    ++flow_rejected_;
    return false;
  }

  // The flow measures the body x/y components of the velocity and nothing
  // about body z, so the innovation lives in the body x/y plane: compare
  // against the estimate expressed in the body frame, keep the estimate's
  // own body-z component, and rotate the difference back into the world.
  // Adding the body-frame innovation to the world-frame state without that
  // rotation is right only at zero yaw and zero tilt, which is exactly the
  // condition a hover test flies in -- so the test uses a yawed vehicle.
  const Eigen::Vector3d v_body_est = R_est.transpose() * v_;
  const Eigen::Vector3d innovation_body(
    v_flow_body.x() - v_body_est.x(), v_flow_body.y() - v_body_est.y(), 0.0);

  // First-order pull with the flow_tau_s time constant. alpha is derived
  // from the interval rather than fixed, so the filter keeps the same time
  // constant if the sensor rate changes.
  const double T = std::min(flow.integration_time_s, cfg_.max_dt_s);
  const double alpha = T / (cfg_.flow_tau_s + T);
  v_ += alpha * (R_est * innovation_body);

  v_flow_world_ = R_est * Eigen::Vector3d(v_flow_body.x(), v_flow_body.y(), v_body_est.z());
  pending_.flow_applied = true;
  return true;
}

double VelocityFilter::tofAltitude(const Eigen::Matrix3d & R_est, double range_m)
{
  if (!std::isfinite(range_m)) {return kNaN;}
  // R(2,2) is the world-z component of the body z axis: cos(tilt). The beam
  // runs along body -z, so the vertical drop it covers is the slant range
  // times this, whatever direction the tilt is in.
  return range_m * R_est(2, 2);
}

bool VelocityFilter::correctTof(const Eigen::Matrix3d & R_est, double range_m, double dt)
{
  const double cos_tilt = R_est(2, 2);
  if (!std::isfinite(range_m) || range_m <= 0.0 || cos_tilt <= 0.0) {
    ++tof_rejected_;
    return false;
  }
  const double z_tof = tofAltitude(R_est, range_m);
  z_tof_ = z_tof;

  if (!z_valid_) {
    // Seed. A single range says where the floor is, not how fast it is
    // approaching; the rate starts being informed from the second reading.
    z_ = z_tof;
    z_valid_ = true;
    pending_.tof_applied = true;
    return true;
  }
  if (!(dt > 0.0)) {
    ++tof_rejected_;
    return false;
  }

  // Second-order complementary update, the discrete form of
  //     z'  = v_z + 2 zeta omega (z_tof - z)
  //     v_z' = a_z +      omega^2 (z_tof - z)
  // where the a_z half lives in predict(). The residual moves BOTH states:
  // a persistent altitude error can only be explained by a velocity error,
  // and correcting only z would leave the accelerometer's integral to drift
  // off again between readings. Differentiating consecutive ranges instead
  // would put sqrt(2) x 2.5 cm / 33 ms = 1 m/s of noise on the rate per
  // sample; this form reaches the rate through omega^2 and never divides
  // by the interval.
  const double h = std::min(dt, cfg_.max_dt_s);
  const double residual = z_tof - z_;
  const double k_z = std::min(1.0, 2.0 * cfg_.tof_zeta * cfg_.tof_omega_rad_s * h);
  const double k_vz = cfg_.tof_omega_rad_s * cfg_.tof_omega_rad_s * h;
  z_ += k_z * residual;
  v_.z() += k_vz * residual;
  pending_.tof_applied = true;
  return true;
}

void VelocityFilter::setAltitude(double z_m)
{
  if (!std::isfinite(z_m)) {return;}
  z_ = z_m;
  z_valid_ = true;
}

void VelocityFilter::reset()
{
  v_.setZero();
  z_ = 0.0;
  z_valid_ = false;
  v_flow_world_.setZero();
  z_tof_ = 0.0;
  pending_ = VelocityStepInfo{};
  last_ = VelocityStepInfo{};
  flow_rejected_ = 0;
  tof_rejected_ = 0;
}

}  // namespace dsim_estimation
