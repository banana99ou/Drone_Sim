#pragma once
#include <Eigen/Dense>

namespace dsim_control
{

/// Maps a desired body wrench [thrust, tau_x, tau_y, tau_z] onto four rotor
/// speeds, for an X-configuration quadrotor in an FLU body frame.
///
/// Rotor layout (must match dsim_description/models/drone/model.sdf):
///   index  position (x, y)   spin
///     0    (+d, +d)          ccw   front-left
///     1    (-d, -d)          ccw   back-right
///     2    (+d, -d)          cw    front-right
///     3    (-d, +d)          cw    back-left
///   with d = arm_length / sqrt(2)
///
/// Allocation, given per-rotor thrusts f_i >= 0:
///   T     =  sum f_i
///   tau_x =  sum  y_i * f_i
///   tau_y = -sum  x_i * f_i
///   tau_z = -sum  s_i * c_m * f_i     (s_i = +1 ccw, -1 cw; reaction torque
///                                      opposes the rotor's own spin)
///
/// The inverse is computed numerically at construction rather than hand-derived,
/// so changing the geometry in the config cannot silently desync the mixer.
class Mixer
{
public:
  Mixer(double arm_length_m,
        double moment_constant_m,
        double motor_constant,
        double max_rot_velocity);

  /// wrench -> per-rotor thrust in N. Negative demands are clamped to 0, which
  /// means the realised wrench may differ from the demand under saturation;
  /// use wrenchFromThrusts() to find out by how much.
  Eigen::Vector4d rotorThrusts(const Eigen::Vector4d & wrench) const;

  /// wrench -> per-rotor speed in rad/s, clamped to [0, max_rot_velocity].
  Eigen::Vector4d rotorSpeeds(const Eigen::Vector4d & wrench) const;

  /// Forward map, for verifying that the inverse actually inverts.
  Eigen::Vector4d wrenchFromThrusts(const Eigen::Vector4d & thrusts) const;

  /// Round-trip residual for a given wrench: ||M * M^-1 * w - w||_inf.
  /// A correct mixer returns ~0 for any unsaturated wrench. This is the check
  /// that fails loudly if the geometry, spin directions, or sign conventions
  /// in the SDF and here ever drift apart.
  double roundTripError(const Eigen::Vector4d & wrench) const;

  double conditionNumber() const;
  const Eigen::Matrix4d & allocation() const {return alloc_;}

  double thrustToSpeed(double thrust_n) const;
  double speedToThrust(double omega) const;
  double maxRotVelocity() const {return max_rot_velocity_;}
  double maxThrustPerRotor() const {return speedToThrust(max_rot_velocity_);}

private:
  Eigen::Matrix4d alloc_;      ///< wrench = alloc_ * thrusts
  Eigen::Matrix4d alloc_inv_;
  double motor_constant_;
  double max_rot_velocity_;
};

}  // namespace dsim_control
