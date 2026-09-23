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
// LINE OF SIGHT is the second thing an obstacle can take away, and some
// scenarios are about that rather than about collision. A scenario may name
// STATIONS -- fixed ground points the vehicle must stay visible from -- and
// the margin is
//
//     distance( obstacle centre at t, segment[station, vehicle] )  -  radius
//
// positive exactly when the sight line clears the body, because a sphere
// blocks a segment precisely when the segment passes within `radius` of its
// centre. The foot of the perpendicular is CLAMPED to the segment: a body
// behind the station, or beyond the vehicle, is not between them and blocks
// nothing. Inactive obstacles block nothing either.
//
// This is the same arithmetic as the planner's los_margin_at(), deliberately
// written out again rather than shared: the planner certifies line of sight
// through a convex occlusion relaxation, and the only way to find out whether
// that certificate means what it says is to measure the true geometry here and
// compare. scripts/check_planner.py holds the two against each other.
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
#include <stdexcept>
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
  /// Margin of the sight line from `station` to (px, py, pz) past this body.
  ///
  /// +infinity when the obstacle is not active: it does not exist, so it
  /// cannot be in the way. Negative means the sight line passes through the
  /// body -- the vehicle is not visible from that station.
  double losMargin(
    const std::array<double, 3> & station, double px, double py, double pz,
    double t) const
  {
    if (!activeAt(t)) {return std::numeric_limits<double>::infinity();}
    const auto c = centreAt(t);
    const double sx = px - station[0], sy = py - station[1], sz = pz - station[2];
    const double seg_sq = sx * sx + sy * sy + sz * sz;
    // Station and vehicle at the same point: there is no segment, so the only
    // sensible measurement is from that one point.
    double u = 0.0;
    if (seg_sq > 1e-24) {
      u = ((c[0] - station[0]) * sx + (c[1] - station[1]) * sy +
        (c[2] - station[2]) * sz) / seg_sq;
      u = std::min(1.0, std::max(0.0, u));
    }
    const double fx = station[0] + u * sx - c[0];
    const double fy = station[1] + u * sy - c[1];
    const double fz = station[2] + u * sz - c[2];
    return std::sqrt(fx * fx + fy * fy + fz * fz) - radius;
  }

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

/// One loss of line of sight, edge-triggered the way a Hit is.
///
/// Separate from Hit because it is a different failure: the vehicle can be
/// nowhere near an obstacle and still be behind it. A scenario whose point is
/// staying visible fails here without ever colliding.
struct Blackout
{
  std::string station;               // which station lost sight of it
  std::string blocker;               // the obstacle that got in the way
  double x{0}, y{0}, z{0};           // vehicle position when sight was lost
  double t{0};                       // scenario time sight was lost
  double duration{0};                // how long it stayed lost, on the scenario clock
  double depth{0};                   // deepest the sight line went into the body
};

/// Running clearance and hit list over a stream of samples.
class ClearanceTracker
{
public:
  explicit ClearanceTracker(
    std::vector<Obstacle> obstacles, double vehicle_radius,
    std::vector<std::array<double, 3>> stations = {})
  : obstacles_(std::move(obstacles)), radius_(vehicle_radius),
    stations_(std::move(stations)),
    inside_(obstacles_.size(), false), entry_index_(obstacles_.size(), 0),
    dark_(stations_.size(), false), dark_index_(stations_.size(), 0)
  {
    // A column is a 2D obstacle lifted to every altitude, and the planner has
    // no line-of-sight arithmetic for one -- its occlusion builder and its
    // los_margin_at() both treat an obstacle as a ball. Measuring a column
    // here would produce a number with nothing to check it against, which is
    // worse than no number: the whole point of computing this in the referee
    // is that the planner computes it too and the two must agree. Refuse, and
    // say what would have to be built.
    if (!stations_.empty()) {
      for (const auto & o : obstacles_) {
        if (o.type == "column") {
          throw std::runtime_error(
                  "obstacle '" + o.name + "' is a column and this scenario has "
                  "stations: line of sight past a column is not implemented, on "
                  "either side. Give the planner segment-to-cylinder occlusion "
                  "first, then mirror it here.");
        }
      }
    }
  }

  struct Sample
  {
    double clearance{std::numeric_limits<double>::infinity()};
    std::string nearest;
    /// Worst line-of-sight margin over all stations, +infinity when there are
    /// no stations or nothing is in the way. Negative means the vehicle is
    /// hidden from at least one station.
    double los_margin{std::numeric_limits<double>::infinity()};
    std::string los_blocker;         // the obstacle responsible for that margin
    std::string los_station;         // the station that has the worst view
  };

  /// One ground-truth sample at scenario time t.
  Sample update(double px, double py, double pz, double t)
  {
    Sample s;
    updateLineOfSight(s, px, py, pz, t);
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
  const std::vector<std::array<double, 3>> & stations() const {return stations_;}
  const std::vector<Hit> & hits() const {return hits_;}
  const std::vector<Blackout> & blackouts() const {return blackouts_;}
  double minClearance() const {return min_clearance_;}
  double minLosMargin() const {return min_los_;}
  double vehicleRadius() const {return radius_;}

  /// The name this class reports a station by. Stations are coordinates in the
  /// scenario with no names of their own, so the index IS the name and the two
  /// cannot drift apart.
  static std::string stationName(size_t i) {return "station" + std::to_string(i);}

  void reset()
  {
    hits_.clear();
    blackouts_.clear();
    std::fill(inside_.begin(), inside_.end(), false);
    std::fill(dark_.begin(), dark_.end(), false);
    min_clearance_ = std::numeric_limits<double>::infinity();
    min_los_ = std::numeric_limits<double>::infinity();
  }

private:
  /// Worst sight line over every station, and one Blackout per excursion.
  ///
  /// Edge-triggered per STATION, for the same reason hits are edge-triggered
  /// per obstacle: a two-second pass behind a body is one blackout, not five
  /// hundred. The blocker recorded is the one responsible when sight was LOST;
  /// a second body drifting across the same dark sight line does not start a
  /// new event, because sight was already gone.
  void updateLineOfSight(Sample & s, double px, double py, double pz, double t)
  {
    for (size_t k = 0; k < stations_.size(); ++k) {
      double worst = std::numeric_limits<double>::infinity();
      std::string blocker;
      for (const auto & o : obstacles_) {
        const double m = o.losMargin(stations_[k], px, py, pz, t);
        if (m < worst) {
          worst = m;
          blocker = o.name;
        }
      }
      if (worst < s.los_margin) {
        s.los_margin = worst;
        s.los_blocker = blocker;
        s.los_station = stationName(k);
      }
      if (worst < 0.0) {
        if (!dark_[k]) {
          dark_[k] = true;
          blackouts_.push_back(Blackout{stationName(k), blocker, px, py, pz, t, 0.0, -worst});
          dark_index_[k] = blackouts_.size() - 1;
        } else {
          Blackout & b = blackouts_[dark_index_[k]];
          b.depth = std::max(b.depth, -worst);
          b.duration = std::max(0.0, t - b.t);
        }
      } else {
        dark_[k] = false;
      }
    }
    if (std::isfinite(s.los_margin)) {
      min_los_ = std::min(min_los_, s.los_margin);
    }
  }

  std::vector<Obstacle> obstacles_;
  double radius_;
  std::vector<std::array<double, 3>> stations_;
  std::vector<bool> inside_;
  std::vector<size_t> entry_index_;
  std::vector<Hit> hits_;
  std::vector<bool> dark_;
  std::vector<size_t> dark_index_;
  std::vector<Blackout> blackouts_;
  double min_clearance_ {std::numeric_limits<double>::infinity()};
  double min_los_ {std::numeric_limits<double>::infinity()};
};

}  // namespace dsim_eval
