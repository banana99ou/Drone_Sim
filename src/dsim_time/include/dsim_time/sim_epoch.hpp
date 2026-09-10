#pragma once
#include <cstddef>

namespace dsim_time
{

/// Detects that the simulation restarted, from the clock alone.
///
/// WHY THIS IS A SHARED RULE AND NOT A LOCAL `if`.
///
/// Resetting the world sends simulated time back to zero. Every node that
/// accumulates something -- an RMSE sum, a path length, an energy integral, a
/// trajectory buffer keyed on stamps, a time origin captured at startup -- is
/// then holding numbers that belong to a run that no longer exists. Before
/// there was a reset button this was invisible; the first time one was pressed
/// the referee reported an RMSE of 1118 m for a vehicle that was tracking a
/// circle to 5 cm, because `elapsed_s` followed the clock back to zero while
/// the sum of squares did not.
///
/// The signal has to be the CLOCK, not a message from whoever performed the
/// reset. A reset can come from this project's own control node, from the
/// Gazebo GUI, or from someone typing `gz service` in a terminal, and only one
/// of those three will ever send us a notification. Simulated time going
/// backwards is the one observable common to all of them.
///
/// It is deliberately not an equality test against zero: a world reset lands
/// near zero but not exactly on it, and a node may not sample the clock until
/// several ticks later.
///
/// Usage: feed it simulated time at the top of whatever callback owns the
/// accumulation, and treat `true` as "start a new run".
///
///     if (epoch_.restarted(now)) { reset(); }
class SimEpoch
{
public:
  /// `threshold_s` is how far backwards counts as a restart rather than
  /// jitter. It sits far above any ordering noise between two samples of the
  /// same clock and far below any interesting run length, so its exact value
  /// is not a tuning knob.
  explicit SimEpoch(double threshold_s = 0.05)
  : threshold_s_(threshold_s) {}

  /// Feed the current SIMULATED time in seconds.
  ///
  /// Returns true exactly once per restart: on the first sample that is more
  /// than `threshold_s` earlier than the previous one. Returns false on the
  /// very first sample -- there is nothing accumulated yet to invalidate, and
  /// reporting a restart at startup would make every node log a reset it did
  /// not perform.
  bool restarted(double now_s)
  {
    if (!have_) {
      have_ = true;
      last_s_ = now_s;
      return false;
    }
    const bool jumped_back = (last_s_ - now_s) > threshold_s_;
    last_s_ = now_s;
    if (jumped_back) {
      ++epochs_;
      return true;
    }
    return false;
  }

  /// Restarts seen so far. Published in telemetry so a viewer can tell "the
  /// numbers are small because the run is young" from "the numbers are small
  /// because they are wrong".
  std::size_t epochs() const {return epochs_;}

  /// Drop the remembered sample without counting a restart.
  ///
  /// For the window before a node's clock is meaningful: with `use_sim_time`
  /// and no `/clock` yet, `now()` reads zero, and the first real stamp would
  /// otherwise look like a large forward jump followed by nothing. Forgetting
  /// is the honest way to say "I have no previous sample", as opposed to
  /// pretending the last one was zero.
  ///
  /// Only the `have_` flag is cleared. Zeroing `last_s_` as well looked tidier
  /// and was worse: with no previous sample the value is never read, and while
  /// it was there the mutation harness could delete the line that actually
  /// does the work without a single test noticing -- because a forgotten
  /// `last_s_` of 0 behaves identically to no sample at all, for every
  /// simulated time that can occur. Dead state that hides a live bug.
  void forget()
  {
    have_ = false;
  }

  double threshold() const {return threshold_s_;}

private:
  double threshold_s_;
  double last_s_ {0.0};
  bool have_ {false};
  std::size_t epochs_ {0};
};

}  // namespace dsim_time
