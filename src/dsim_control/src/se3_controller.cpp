#include "dsim_control/se3_controller.hpp"
#include <cmath>

namespace dsim_control
{

Eigen::Vector3d vee(const Eigen::Matrix3d & m)
{
  return Eigen::Vector3d(m(2, 1), m(0, 2), m(1, 0));
}

Eigen::Matrix3d hat(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m <<     0.0, -v.z(),  v.y(),
        v.z(),     0.0, -v.x(),
       -v.y(),   v.x(),    0.0;
  return m;
}

SE3Controller::SE3Controller(const Gains & gains)
: gains_(gains) {}

Eigen::Vector4d SE3Controller::compute(
  const State & state, const Reference & ref, double dt, ControlDebug * debug)
{
  const Eigen::Matrix3d R = state.orientation.toRotationMatrix();
  const Eigen::Vector3d e3(0.0, 0.0, 1.0);

  // ---- translational loop: desired force in the world frame ----------------
  const Eigen::Vector3d e_p = state.position - ref.position;
  const Eigen::Vector3d e_v = state.velocity - ref.velocity;

  // ---- integral of position error -----------------------------------------
  //
  // Why this exists, since the stack deliberately had no integrator: with an
  // exact model and exact state there is no steady-state error for one to
  // remove, and that was true until an external force could be applied to the
  // airframe. A gust is a constant force nothing in the model accounts for,
  // and a PD loop answers it with a standing offset of exactly F/kp -- 0.83 m
  // for a 5 N push against kp = 6, held for as long as the push lasts. The
  // integral is the only term that can drive that to zero.
  //
  // Accumulated BEFORE the clamps below, and frozen when the previous step hit
  // the tilt clamp. That is conditional integration, and it is the standard
  // answer to the standard failure: a loop that keeps integrating while the
  // actuator is already saturated builds a demand nothing can satisfy, and
  // every newton of it has to be unwound before the vehicle can return.
  const bool hold = tilt_clamped_last_;
  if (dt > 0.0 && !hold) {
    integral_ += gains_.ki.cwiseProduct(e_p) * dt;
    // Clamped per axis, in newtons, so the limit is a force the vehicle can
    // actually produce rather than an abstract bound on an accumulator.
    integral_ = integral_.cwiseMax(-gains_.max_integral_n)
      .cwiseMin(gains_.max_integral_n);
  }

  Eigen::Vector3d f_des =
    -gains_.kp.cwiseProduct(e_p)
    - gains_.kv.cwiseProduct(e_v)
    - integral_
    + gains_.mass * gains_.gravity * e3
    + gains_.mass * ref.acceleration;

  // Never let the desired force point downward or sideways-only: without this,
  // a large position error can ask for an inverted vehicle.
  if (f_des.z() < gains_.min_thrust_n) {
    f_des.z() = gains_.min_thrust_n;
  }

  // Clamp commanded tilt. An unclamped geometric controller will happily ask
  // for 80 degrees of bank to fix a big position error, which is unflyable and
  // makes the planner look wrong when the controller is what gave up.
  const double horiz = f_des.head<2>().norm();
  double tilt = std::atan2(horiz, f_des.z());
  bool clamped = false;
  if (tilt > gains_.max_tilt_rad) {
    const double horiz_max = f_des.z() * std::tan(gains_.max_tilt_rad);
    f_des.head<2>() *= (horiz_max / horiz);
    tilt = gains_.max_tilt_rad;
    clamped = true;
  }

  // Thrust is the projection of the desired force on the current body z axis.
  const double thrust = std::max(f_des.dot(R * e3), gains_.min_thrust_n);

  // ---- desired attitude ----------------------------------------------------
  const Eigen::Vector3d b3_d = f_des.normalized();
  const Eigen::Vector3d b1_c(std::cos(ref.yaw), std::sin(ref.yaw), 0.0);

  Eigen::Vector3d b2_d = b3_d.cross(b1_c);
  if (b2_d.norm() < 1e-6) {
    // Desired thrust axis is parallel to the yaw reference (vehicle pointing
    // straight at its own heading vector). Fall back to the current body y.
    b2_d = R.col(1);
  }
  b2_d.normalize();
  const Eigen::Vector3d b1_d = b2_d.cross(b3_d);

  Eigen::Matrix3d R_d;
  R_d.col(0) = b1_d;
  R_d.col(1) = b2_d;
  R_d.col(2) = b3_d;

  // ---- rotational loop -----------------------------------------------------
  const Eigen::Vector3d e_R = 0.5 * vee(R_d.transpose() * R - R.transpose() * R_d);

  // Feedforward body rate from yaw rate only; a full derivation would need the
  // reference jerk. Documented limitation: yaw-rate feedforward is exact,
  // roll/pitch-rate feedforward is not, so very aggressive trajectories will
  // show a small attitude lag.
  const Eigen::Vector3d omega_d = R.transpose() * (ref.yaw_rate * e3);
  const Eigen::Vector3d e_omega = state.angular_rate - omega_d;

  const Eigen::Vector3d torque =
    -gains_.kR.cwiseProduct(e_R)
    - gains_.komega.cwiseProduct(e_omega)
    + state.angular_rate.cross(gains_.inertia * state.angular_rate);

  tilt_clamped_last_ = clamped;

  if (debug) {
    debug->integral_force = -integral_;
    debug->integral_held = hold;
    debug->position_error = e_p;
    debug->velocity_error = e_v;
    debug->attitude_error = e_R;
    debug->body_rate_error = e_omega;
    debug->desired_force  = f_des;
    debug->commanded_tilt_rad = tilt;
    debug->tilt_clamped = clamped;
  }

  return Eigen::Vector4d(thrust, torque.x(), torque.y(), torque.z());
}

}  // namespace dsim_control
