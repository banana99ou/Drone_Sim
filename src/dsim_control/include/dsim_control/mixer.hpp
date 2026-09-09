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

  /// Rotor i's hub position in the body frame, in metres, z at the body plane.
  ///
  /// Comes from the same geometry table the allocation matrix is built from, so
  /// telemetry that reports "rotor i is at (x, y) and is producing f newtons"
  /// cannot put the thrust on the wrong arm. A viewer drawing arrows from these
  /// positions will show them visibly off the rotor discs if the mixer and the
  /// SDF ever disagree about the layout -- a loud failure instead of a silent
  /// one.
  Eigen::Vector3d rotorPosition(int i) const;
  static int rotorCount() {return 4;}

  double thrustToSpeed(double thrust_n) const;
  double speedToThrust(double omega) const;
  double maxRotVelocity() const {return max_rot_velocity_;}
  double maxThrustPerRotor() const {return speedToThrust(max_rot_velocity_);}

private:
  /// The one place the rotor layout is written down in this package.
  ///   sx, sy  hub position as a multiple of arm_length / sqrt(2)
  ///   spin    +1 counter-clockwise, -1 clockwise; the reaction torque on the
  ///           airframe opposes the rotor's own spin, hence the sign on tau_z
  /// Must match scripts/gen_assets.py rotor_layout(), which writes the SDF.
  struct RotorGeometry
  {
    double sx, sy, spin;
  };
  static constexpr RotorGeometry kLayout[4] = {
    {+1.0, +1.0, +1.0},        // 0  front-left,  ccw
    {-1.0, -1.0, +1.0},        // 1  back-right,  ccw
    {+1.0, -1.0, -1.0},        // 2  front-right, cw
    {-1.0, +1.0, -1.0},        // 3  back-left,   cw
  };

  Eigen::Matrix4d alloc_;      ///< wrench = alloc_ * thrusts
  Eigen::Matrix4d alloc_inv_;
  double arm_offset_ {0.0};    ///< arm_length / sqrt(2), the per-axis offset
  double motor_constant_;
  double max_rot_velocity_;
};

}  // namespace dsim_control
