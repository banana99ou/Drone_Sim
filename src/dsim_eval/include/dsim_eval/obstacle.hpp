#pragma once
// The referee's geometry, with no ROS in it so it can be unit tested and
// mutation-checked on the host.
//
// Two things live here. Obstacle is a shape that may be moving: its centre at
// scenario time t is pos0 + vel * t, and its distance function is SIGNED, so a
// vehicle inside it reads a negative number instead of the zero the old
// max(0, ...) clamp returned -- a clamp that made every collision look like a
// graze. ClearanceTracker turns a stream of (position, time) samples into the
// running minimum and a list of HITS: one entry per excursion into an
// obstacle, edge-triggered, carrying where the vehicle was when it went in and
// how deep it got. "It collided" is a boolean; "it went 14 cm into F5 at
// (5.21, 5.00, 0.50), t=5.2 s" is something you can act on.
//
// The scenario clock is the planner's t: it is zero when the plan starts
// executing, not when the simulator starts. Callers convert; this file only
// ever sees scenario time.
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace dsim_eval
{

struct Obstacle
{
  std::string name;
  std::string type;                  // "box" | "cylinder" | "sphere"
  double x{0}, y{0}, z{0};           // centre at scenario t = 0
  double sx{1}, sy{1}, sz{1};        // box: full extents; cylinder: radius,_,height; sphere: radius
  double vx{0}, vy{0}, vz{0};        // constant velocity, m/s; zero for the static courses

  bool moving() const {return vx != 0.0 || vy != 0.0 || vz != 0.0;}

  /// Centre at scenario time t. Unbounded in t on purpose: the Gazebo body
  /// keeps flying after the plan's horizon, and the referee scores the world
  /// that is actually there. (The planner clips obstacle motion to [0, T];
  /// inside the window the two agree exactly, and outside it only the
  /// referee has an opinion.)
  void centreAt(double t, double & cx, double & cy, double & cz) const
  {
    cx = x + vx * t; cy = y + vy * t; cz = z + vz * t;
  }

  /// SIGNED distance from a point to the surface at scenario time t.
  /// Positive outside, negative inside by the depth of penetration.
  double signedDistance(double px, double py, double pz, double t) const
  {
    double cx, cy, cz;
    centreAt(t, cx, cy, cz);
    const double dx = px - cx, dy = py - cy, dz = pz - cz;
    if (type == "sphere") {
      return std::sqrt(dx * dx + dy * dy + dz * dz) - sx;
    }
    if (type == "cylinder") {
      // Radial and axial excess; the usual box-style combination outside,
      // and the least distance to a face inside.
      const double er = std::sqrt(dx * dx + dy * dy) - sx;
      const double ez = std::abs(dz) - sz / 2.0;
      if (er > 0.0 || ez > 0.0) {
        const double a = std::max(er, 0.0), b = std::max(ez, 0.0);
        return std::sqrt(a * a + b * b);
      }
      return std::max(er, ez);
    }
    // box, axis aligned
    const double ex = std::abs(dx) - sx / 2.0;
    const double ey = std::abs(dy) - sy / 2.0;
    const double ez = std::abs(dz) - sz / 2.0;
    if (ex > 0.0 || ey > 0.0 || ez > 0.0) {
      const double a = std::max(ex, 0.0), b = std::max(ey, 0.0), c = std::max(ez, 0.0);
      return std::sqrt(a * a + b * b + c * c);
    }
    return std::max({ex, ey, ez});
  }
};

struct Hit
{
  std::string with;
  double x{0}, y{0}, z{0};           // vehicle position when it went in
  double t{0};                       // scenario time of entry
  double depth{0};                   // deepest penetration during this excursion
};

/// Running clearance and hit list over a stream of samples.
class ClearanceTracker
{
public:
  explicit ClearanceTracker(std::vector<Obstacle> obstacles, double vehicle_radius)
  : obstacles_(std::move(obstacles)), radius_(vehicle_radius),
    inside_(obstacles_.size(), false), entry_index_(obstacles_.size(), 0) {}

  struct Sample
  {
    double clearance{std::numeric_limits<double>::infinity()};
    std::string nearest;
  };

  /// One ground-truth sample at scenario time t.
  Sample update(double px, double py, double pz, double t)
  {
    Sample s;
    for (size_t i = 0; i < obstacles_.size(); ++i) {
      const double c = obstacles_[i].signedDistance(px, py, pz, t) - radius_;
      if (c < s.clearance) {
        s.clearance = c;
        s.nearest = obstacles_[i].name;
      }
      if (c < 0.0) {
        if (!inside_[i]) {
          // Entry: record where and when, once per excursion. Recording
          // every sample inside would report a 2 s pass through a sphere as
          // five hundred collisions.
          inside_[i] = true;
          hits_.push_back(Hit{obstacles_[i].name, px, py, pz, t, -c});
          entry_index_[i] = hits_.size() - 1;
        } else {
          Hit & h = hits_[entry_index_[i]];
          h.depth = std::max(h.depth, -c);
        }
      } else {
        inside_[i] = false;
      }
    }
    if (std::isfinite(s.clearance)) {
      min_clearance_ = std::min(min_clearance_, s.clearance);
    }
    return s;
  }

  const std::vector<Obstacle> & obstacles() const {return obstacles_;}
  const std::vector<Hit> & hits() const {return hits_;}
  double minClearance() const {return min_clearance_;}
  double vehicleRadius() const {return radius_;}

  void reset()
  {
    hits_.clear();
    std::fill(inside_.begin(), inside_.end(), false);
    min_clearance_ = std::numeric_limits<double>::infinity();
  }

private:
  std::vector<Obstacle> obstacles_;
  double radius_;
  std::vector<bool> inside_;
  std::vector<size_t> entry_index_;
  std::vector<Hit> hits_;
  double min_clearance_ {std::numeric_limits<double>::infinity()};
};

}  // namespace dsim_eval
