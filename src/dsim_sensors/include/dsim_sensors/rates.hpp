#pragma once
#include <cmath>

#include <Eigen/Dense>

namespace dsim_sensors
{

/// True body angular rate from two attitudes, safe across the quaternion
/// double cover.
///
/// This exists because getting it wrong has already cost this project a real
/// bug. q and -q are the same rotation, so a differentiator that subtracts
/// components sees a jump of 2|q| when the representation flips, and reports an
/// enormous rate while the attitude has barely moved. Gazebo's
/// OdometryPublisher does exactly that, once per revolution: it reported
/// 626 rad/s on a vehicle turning at 1.8.
///
/// What protects this function is the choice of formulation, not a sign check.
///
/// Eigen's AngleAxis constrains its angle to [0, pi] -- it takes the magnitude
/// of the quaternion's scalar part and flips the axis to compensate -- so
/// converting the relative rotation through AngleAxis absorbs a sheet flip on
/// its own. Verified empirically before relying on it: with `now` negated, this
/// returns 1.800000 rad/s, the same as unflipped.
///
/// An explicit `if (a.dot(b) < 0) flip` guard was here first. It was dead code:
/// removing it changed nothing, which the mutation harness duly reported as a
/// surviving mutation. Keeping a guard that does nothing is worse than not
/// having one, because it advertises protection the code is not getting from
/// it.
///
/// The naive formulation `2 * (a.conjugate() * b).vec() / dt` is NOT safe here
/// -- it returns the rate with the sign flipped across a sheet change -- so
/// test_sensors.cpp pins both the [0, pi] assumption and the behaviour across
/// a flip, and mutation_check.sh injects the naive form to prove those tests
/// bite.
inline Eigen::Vector3d bodyRateFromQuaternions(
  const Eigen::Quaterniond & prev, const Eigen::Quaterniond & now, double dt)
{
  if (dt <= 0.0) {return Eigen::Vector3d::Zero();}

  // Relative rotation in the BODY frame, then its axis-angle over dt.
  const Eigen::Quaterniond dq =
    (prev.normalized().conjugate() * now.normalized()).normalized();
  const Eigen::AngleAxisd aa(dq);
  return aa.axis() * (aa.angle() / dt);
}

}  // namespace dsim_sensors
