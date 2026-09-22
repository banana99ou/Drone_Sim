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
  /// Integral of position error, in N/(m*s). Zero makes this a PD controller
  /// again, which is what it was until external forces existed.
  ///
  /// The stability bound is not a matter of taste: the translational loop with
  /// an integrator is m*s^3 + kv*s^2 + kp*s + ki, and Routh-Hurwitz requires
  /// ki < kp*kv/m -- 16 on x/y and 26.7 on z with the gains above. The
  /// defaults sit at about a tenth of that, which is slow enough to leave the
  /// PD behaviour of the loop untouched and fast enough to remove a steady
  /// disturbance in a few seconds.
  Eigen::Vector3d ki     {1.5, 1.5, 2.0};
  /// Anti-windup: the largest force the integral term may contribute on any
  /// axis. A quadrotor at the 40 degree tilt clamp has about 12 N of lateral
  /// authority, so an integral allowed to grow past that is asking for an
  /// attitude the vehicle will not be given -- and every newton of it has to
  /// be unwound before the vehicle can come back.
  double max_integral_n {6.0};
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
  Eigen::Vector3d body_rate_error {Eigen::Vector3d::Zero()};
  Eigen::Vector3d desired_force  {Eigen::Vector3d::Zero()};
  /// The force the integral term is contributing, in newtons, world frame.
  /// Published because an integrator is otherwise invisible: it is the one
  /// part of the loop whose output does not follow from the state you can see,
  /// and a wound-up integrator looks exactly like a mis-trimmed vehicle.
  Eigen::Vector3d integral_force {Eigen::Vector3d::Zero()};
  /// True while the integrator is deliberately not accumulating.
  bool   integral_held {false};
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
  ///
  /// `dt` is the time since the previous call, in seconds, and is what the
  /// integral term accumulates over. It is NOT const any more for that reason:
  /// this controller now carries one piece of state, and pretending otherwise
  /// would mean hiding it in a mutable member where nothing could reset it.
  ///
  /// Pass dt = 0 to evaluate the loop without advancing the integrator, which
  /// is what a test of the proportional behaviour wants.
  Eigen::Vector4d compute(
    const State & state, const Reference & ref, double dt,
    ControlDebug * debug = nullptr);

  /// Forget the accumulated disturbance estimate.
  ///
  /// Called when the run restarts or the vehicle is disarmed. An integral is a
  /// claim about a force that is acting NOW; carrying one across a reset makes
  /// the new run start by correcting for a gust that belongs to the old one.
  void resetIntegral() {integral_.setZero();}
  const Eigen::Vector3d & integral() const {return integral_;}

  void setGains(const Gains & g) {gains_ = g;}
  const Gains & gains() const {return gains_;}

  /// Thrust needed to hold altitude at zero tilt. Useful as a sanity anchor:
  /// a correct controller commands almost exactly this while hovering.
  double hoverThrust() const {return gains_.mass * gains_.gravity;}

private:
  Gains gains_;
  /// Accumulated position error, scaled by ki -- i.e. a force, not an
  /// integral of metres, so the clamp below is in newtons and means something
  /// physical rather than "units of gain times seconds".
  Eigen::Vector3d integral_ {Eigen::Vector3d::Zero()};
  /// Whether the PREVIOUS step's demand hit the tilt clamp. Conditional
  /// integration needs to know the actuator was already saturated, and that is
  /// only known after the clamp has been applied -- so it is carried one step.
  bool tilt_clamped_last_ {false};
};

/// so(3) inverse-hat: extracts the vector from a skew-symmetric matrix.
Eigen::Vector3d vee(const Eigen::Matrix3d & m);
/// Vector to skew-symmetric matrix.
Eigen::Matrix3d hat(const Eigen::Vector3d & v);

}  // namespace dsim_control
