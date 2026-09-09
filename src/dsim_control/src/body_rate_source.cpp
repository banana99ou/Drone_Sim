#include "dsim_control/body_rate_source.hpp"
#include <cmath>
#include <stdexcept>

namespace dsim_control
{

BodyRateSource::BodyRateSource(double max_rate_rad_s, double lowpass_tau_s)
: max_rate_(max_rate_rad_s), tau_(lowpass_tau_s)
{
  if (!(max_rate_rad_s > 0.0)) {
    throw std::invalid_argument("BodyRateSource: max rate must be > 0");
  }
}

Eigen::Vector3d BodyRateSource::update(const Eigen::Vector3d & measured, double dt)
{
  // Reject non-finite before comparing: NaN fails every comparison, so a
  // magnitude test alone would let it through and poison the loop silently.
  if (!measured.allFinite() || measured.norm() > max_rate_) {
    ++rejected_;
    return value_;
  }

  if (!valid_ || tau_ <= 0.0 || dt <= 0.0) {
    // First good sample: adopt it outright. Filtering towards it from zero
    // would fabricate a spin-up the vehicle never did.
    value_ = measured;
    valid_ = true;
    return value_;
  }

  // First-order low-pass. alpha is derived from dt rather than fixed, so the
  // filter keeps the same time constant if the control rate changes.
  const double alpha = dt / (tau_ + dt);
  value_ += alpha * (measured - value_);
  return value_;
}

}  // namespace dsim_control
