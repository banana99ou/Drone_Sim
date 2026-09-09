#pragma once
#include <mutex>
#include <optional>
#include <dsim_msgs/msg/trajectory.hpp>
#include "dsim_control/se3_controller.hpp"
#include "dsim_control/angles.hpp"

namespace dsim_control
{

/// Holds the active trajectory and samples it at the control rate.
///
/// Semantics deliberately match how a replanning planner behaves: every new
/// Trajectory REPLACES the previous one, and sampling is by wall-clock offset
/// from the trajectory's header stamp — so a planner that replans at 5 Hz just
/// keeps publishing, and a planner that emits one long trajectory also works.
class TrajectoryBuffer
{
public:
  void set(const dsim_msgs::msg::Trajectory & traj);
  void clear();

  /// Sample at absolute time `now` (seconds since epoch, same clock as stamps).
  /// Returns nullopt when no trajectory is loaded.
  std::optional<Reference> sample(double now_s) const;

  bool active() const;
  /// True once `now` is past the last point — the trajectory has been flown.
  bool finished(double now_s) const;
  double duration() const;

private:
  mutable std::mutex mutex_;
  dsim_msgs::msg::Trajectory traj_;
  bool has_traj_ {false};
  double start_s_ {0.0};
};

}  // namespace dsim_control
