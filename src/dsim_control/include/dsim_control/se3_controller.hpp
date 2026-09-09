#pragma once
#include <Eigen/Dense>

namespace dsim_control
{

/// Full vehicle state, world frame ENU (+z up).
struct State
{
  Eigen::Vector3d position     {Eigen::Vector3d::Zero()};   ///< m, world
  Eigen::Vector3d velocity     {Eigen::Vector3d::Zero()};   ///< m/s, WORLD frame
  Eigen::Quaterniond orientation {Eigen::Quaterniond::Identity()};  ///< body->world
  Eigen::Vector3d angular_rate {Eigen::Vector3d::Zero()};   ///< rad/s, BODY frame
};

/// Flat-output reference. velocity/acceleration act as feedforward.
struct Reference
{
  Eigen::Vector3d position     {Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity     {Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration {Eigen::Vector3d::Zero()};
  double yaw      {0.0};
  double yaw_rate {0.0};
};

struct Gains
{
  Eigen::Vector3d kp     {6.0, 6.0, 8.0};      ///< position
  Eigen::Vector3d kv     {4.0, 4.0, 5.0};      ///< velocity
  Eigen::Vector3d kR     {3.0, 3.0, 0.5};      ///< attitude
  Eigen::Vector3d komega {0.5, 0.5, 0.1};      ///< body rate
  double mass    {1.5};
  double gravity {9.80665};
  Eigen::Matrix3d inertia {Eigen::Vector3d(0.012, 0.012, 0.023).asDiagonal()};
  double max_tilt_rad {0.7};                   ///< clamp on commanded tilt (~40 deg)
  double min_thrust_n {0.5};                   ///< never command a fully idle rotor set
};

/// Diagnostics from one control step — published for debugging, and used by the
/// eval node so tracking error is measured on exactly what the controller saw.
struct ControlDebug
{
  Eigen::Vector3d position_error {Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_error {Eigen::Vector3d::Zero()};
  Eigen::Vector3d attitude_error {Eigen::Vector3d::Zero()};
  Eigen::Vector3d desired_force  {Eigen::Vector3d::Zero()};
  double commanded_tilt_rad {0.0};
  bool   tilt_clamped {false};
};

/// Geometric controller on SE(3), after Lee/Leok/McClamroch (2010).
///
/// Chosen over a cascaded Euler-angle PID because it has no attitude
/// singularity and accepts acceleration feedforward directly — which is what
/// makes aggressive planner output trackable instead of merely stable.
class SE3Controller
{
public:
  explicit SE3Controller(const Gains & gains);

  /// Returns the body wrench [thrust_N, tau_x, tau_y, tau_z].
  Eigen::Vector4d compute(const State & state, const Reference & ref, ControlDebug * debug = nullptr) const;

  void setGains(const Gains & g) {gains_ = g;}
  const Gains & gains() const {return gains_;}

  /// Thrust needed to hold altitude at zero tilt. Useful as a sanity anchor:
  /// a correct controller commands almost exactly this while hovering.
  double hoverThrust() const {return gains_.mass * gains_.gravity;}

private:
  Gains gains_;
};

/// so(3) inverse-hat: extracts the vector from a skew-symmetric matrix.
Eigen::Vector3d vee(const Eigen::Matrix3d & m);
/// Vector to skew-symmetric matrix.
Eigen::Matrix3d hat(const Eigen::Vector3d & v);

}  // namespace dsim_control
