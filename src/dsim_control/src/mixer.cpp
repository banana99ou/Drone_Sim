#include "dsim_control/mixer.hpp"
#include <cmath>
#include <stdexcept>

namespace dsim_control
{

Mixer::Mixer(
  double arm_length_m, double moment_constant_m,
  double motor_constant, double max_rot_velocity)
: motor_constant_(motor_constant), max_rot_velocity_(max_rot_velocity)
{
  if (motor_constant <= 0.0 || arm_length_m <= 0.0 || max_rot_velocity <= 0.0) {
    throw std::invalid_argument("Mixer: arm length, motor constant and max rotor speed must be > 0");
  }

  const double d = arm_length_m / std::sqrt(2.0);
  const double c = moment_constant_m;

  // Columns are rotors 0..3; rows are [T, tau_x, tau_y, tau_z].
  //                     r0    r1    r2    r3
  alloc_ <<  1.0,  1.0,  1.0,  1.0,      // T     = sum f
             d,   -d,   -d,    d,        // tau_x = sum  y_i f_i
            -d,    d,   -d,    d,        // tau_y = -sum x_i f_i
            -c,   -c,    c,    c;        // tau_z = -sum s_i c f_i

  const Eigen::FullPivLU<Eigen::Matrix4d> lu(alloc_);
  if (!lu.isInvertible()) {
    throw std::runtime_error("Mixer: allocation matrix is singular — check rotor geometry");
  }
  alloc_inv_ = lu.inverse();
}

Eigen::Vector4d Mixer::rotorThrusts(const Eigen::Vector4d & wrench) const
{
  Eigen::Vector4d f = alloc_inv_ * wrench;
  const double f_max = maxThrustPerRotor();
  for (int i = 0; i < 4; ++i) {
    f(i) = std::min(std::max(f(i), 0.0), f_max);
  }
  return f;
}

Eigen::Vector4d Mixer::rotorSpeeds(const Eigen::Vector4d & wrench) const
{
  const Eigen::Vector4d f = rotorThrusts(wrench);
  Eigen::Vector4d w;
  for (int i = 0; i < 4; ++i) {
    w(i) = thrustToSpeed(f(i));
  }
  return w;
}

Eigen::Vector4d Mixer::wrenchFromThrusts(const Eigen::Vector4d & thrusts) const
{
  return alloc_ * thrusts;
}

double Mixer::roundTripError(const Eigen::Vector4d & wrench) const
{
  const Eigen::Vector4d f = alloc_inv_ * wrench;       // unclamped on purpose
  return (alloc_ * f - wrench).cwiseAbs().maxCoeff();
}

double Mixer::conditionNumber() const
{
  Eigen::JacobiSVD<Eigen::Matrix4d> svd(alloc_);
  const auto & s = svd.singularValues();
  return s(0) / s(s.size() - 1);
}

double Mixer::thrustToSpeed(double thrust_n) const
{
  if (thrust_n <= 0.0) {return 0.0;}
  return std::min(std::sqrt(thrust_n / motor_constant_), max_rot_velocity_);
}

double Mixer::speedToThrust(double omega) const
{
  return motor_constant_ * omega * omega;
}

}  // namespace dsim_control
