#pragma once
// The referee's geometry, with no ROS in it so it can be unit tested and
// mutation-checked on the host.
//
// An obstacle is exactly what the planner's solver believes one is: a ball (or
// a vertical column) of fixed radius whose centre follows a Bezier curve in
// space-time, with the ACTIVE WINDOW intrinsic to the first and last control
// point's time coordinate. Nothing here is specialised to constant velocity --
// that is simply the degree-1 case, and two of the shipped scenarios move
// their obstacles along cubics.
//
// Outside its window an obstacle DOES NOT EXIST. It is not far away and it is
// not frozen at an endpoint: the planner clips obstacle motion to the window
// and solves a problem in which the thing is absent, so a referee that kept
// scoring against it would be scoring a different problem. `wall` and `door3d`
// are built entirely on that -- the wall stands until t=5 and then is gone,
// and waiting for it is the behaviour being demonstrated.
//
// Two shapes, because the scenarios come in two dimensionalities:
//   sphere   a ball, for a scenario planned in 3 spatial dimensions
//   column   a disc at EVERY altitude, for one planned in 2. A 2D obstacle is
//            not a ball at the flight altitude: treating it as one would hand
//            a 3D vehicle an escape route over the top that the 2D problem
//            never had, and the scenario would stop being the scenario.
//
// ClearanceTracker turns a stream of (position, time) samples into the running
// minimum and a list of HITS: one entry per excursion into an obstacle,
// edge-triggered, carrying where the vehicle was when it went in and how deep
// it got. "It collided" is a boolean; "it went 14 cm into F5 at
// (5.21, 5.00, 0.50), t=5.2 s" is something you can act on.
//
// The scenario clock is the planner's t: zero when the plan starts executing,
// not when the simulator starts. Callers convert; this file only ever sees
// scenario time.
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace dsim_eval
{

/// One control point: x, y, z, t. Always four wide -- a scenario planned in
/// two spatial dimensions is imported at a fixed altitude, and its obstacles
/// are columns, so the z it carries is where it is DRAWN and never enters a
/// distance.
using ControlPoint = std::array<double, 4>;

struct Obstacle
{
  std::string name;
  std::string type {"sphere"};        ///< "sphere" | "column"
  double radius {1.0};
  std::vector<ControlPoint> control_points;

  double tStart() const {return control_points.empty() ? 0.0 : control_points.front()[3];}
  double tEnd() const {return control_points.empty() ? 0.0 : control_points.back()[3];}

  /// True while this obstacle exists. Outside the window it is absent, not
  /// parked: see the header note.
  bool activeAt(double t) const
  {
    return !control_points.empty() && t >= tStart() && t <= tEnd();
  }

  /// Centre at scenario time t, by de Casteljau on the spatial coordinates.
  ///
  /// Time is AFFINE in the curve parameter -- that is what lifting a motion
  /// which is polynomial in time gives -- so inverting it is a division and
  /// not a root solve. Clamped to the window so a caller that asks outside it
  /// gets the endpoint rather than an extrapolation off the end of the curve,
  /// where a Bezier means nothing.
  std::array<double, 3> centreAt(double t) const
  {
    std::array<double, 3> out {0.0, 0.0, 0.0};
    if (control_points.empty()) {return out;}
    const double span = tEnd() - tStart();
    const double s = (span > 1e-15)
      ? std::max(0.0, std::min(1.0, (t - tStart()) / span))
      : 0.0;
    std::vector<ControlPoint> pts = control_points;
    while (pts.size() > 1) {
      for (size_t i = 0; i + 1 < pts.size(); ++i) {
        for (int k = 0; k < 4; ++k) {
          pts[i][k] = (1.0 - s) * pts[i][k] + s * pts[i + 1][k];
        }
      }
      pts.pop_back();
    }
    return {pts[0][0], pts[0][1], pts[0][2]};
  }

  /// SIGNED distance from a point to the surface at scenario time t: positive
  /// outside, negative inside by the depth of penetration, and +infinity when
  /// the obstacle is not there at all.
  double signedDistance(double px, double py, double pz, double t) const
  {
    if (!activeAt(t)) {return std::numeric_limits<double>::infinity();}
    const auto c = centreAt(t);
    const double dx = px - c[0], dy = py - c[1], dz = pz - c[2];
    if (type == "column") {
      return std::sqrt(dx * dx + dy * dy) - radius;
    }
    return std::sqrt(dx * dx + dy * dy + dz * dz) - radius;
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
