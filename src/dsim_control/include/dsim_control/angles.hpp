#pragma once
#include <cmath>

namespace dsim_control
{

/// Interpolate between two angles along the SHORTEST arc.
///
/// Header-only and dependency-free on purpose: this lets the invariant tests
/// build without ROS, so the maths can be verified outside the container.
inline double interpolateAngle(double a, double b, double alpha)
{
  double diff = std::fmod(b - a + M_PI, 2.0 * M_PI);
  if (diff < 0.0) {diff += 2.0 * M_PI;}
  diff -= M_PI;
  return a + alpha * diff;
}

}  // namespace dsim_control
