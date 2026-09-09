#include "dsim_control/trajectory_buffer.hpp"
#include <algorithm>
#include <cmath>

namespace dsim_control
{

namespace
{
double toSec(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + 1e-9 * static_cast<double>(d.nanosec);
}
Eigen::Vector3d toEigen(const geometry_msgs::msg::Point & p)
{
  return Eigen::Vector3d(p.x, p.y, p.z);
}
Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3 & v)
{
  return Eigen::Vector3d(v.x, v.y, v.z);
}
}  // namespace

void TrajectoryBuffer::set(const dsim_msgs::msg::Trajectory & traj)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (traj.points.empty()) {
    has_traj_ = false;
    return;
  }
  traj_ = traj;
  start_s_ = static_cast<double>(traj.header.stamp.sec) +
    1e-9 * static_cast<double>(traj.header.stamp.nanosec);
  has_traj_ = true;
}

void TrajectoryBuffer::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  has_traj_ = false;
}

bool TrajectoryBuffer::active() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_traj_;
}

double TrajectoryBuffer::duration() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_traj_) {return 0.0;}
  return toSec(traj_.points.back().time_from_start);
}

bool TrajectoryBuffer::finished(double now_s) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_traj_) {return false;}
  return (now_s - start_s_) > toSec(traj_.points.back().time_from_start);
}

std::optional<Reference> TrajectoryBuffer::sample(double now_s) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_traj_) {return std::nullopt;}

  const auto & pts = traj_.points;
  const double t = now_s - start_s_;

  auto fill = [](const dsim_msgs::msg::TrajectorySetpoint & p) {
      Reference r;
      r.position     = toEigen(p.position);
      r.velocity     = toEigen(p.velocity);
      r.acceleration = toEigen(p.acceleration);
      r.yaw          = p.yaw;
      r.yaw_rate     = p.yaw_rate;
      return r;
    };

  // Before the trajectory starts: hold the first point, stationary.
  if (t <= toSec(pts.front().time_from_start)) {
    Reference r = fill(pts.front());
    r.velocity.setZero();
    r.acceleration.setZero();
    r.yaw_rate = 0.0;
    return r;
  }

  // After it ends: hold the last point, stationary. The vehicle parks at the
  // goal instead of drifting, which keeps a finished run measurable.
  if (t >= toSec(pts.back().time_from_start)) {
    Reference r = fill(pts.back());
    r.velocity.setZero();
    r.acceleration.setZero();
    r.yaw_rate = 0.0;
    return r;
  }

  // Binary search for the bracketing pair.
  const auto it = std::lower_bound(
    pts.begin(), pts.end(), t,
    [](const dsim_msgs::msg::TrajectorySetpoint & p, double val) {
      return toSec(p.time_from_start) < val;
    });
  const size_t hi = static_cast<size_t>(std::distance(pts.begin(), it));
  const size_t lo = hi - 1;

  const double t_lo = toSec(pts[lo].time_from_start);
  const double t_hi = toSec(pts[hi].time_from_start);
  const double span = t_hi - t_lo;
  const double alpha = (span > 1e-9) ? (t - t_lo) / span : 0.0;

  const Reference a = fill(pts[lo]);
  const Reference b = fill(pts[hi]);

  Reference r;
  r.position     = a.position     + alpha * (b.position - a.position);
  r.velocity     = a.velocity     + alpha * (b.velocity - a.velocity);
  r.acceleration = a.acceleration + alpha * (b.acceleration - a.acceleration);
  r.yaw          = interpolateAngle(a.yaw, b.yaw, alpha);
  r.yaw_rate     = a.yaw_rate + alpha * (b.yaw_rate - a.yaw_rate);
  return r;
}

}  // namespace dsim_control
