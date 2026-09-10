#pragma once
#include <algorithm>
#include <cmath>

namespace dsim_simctl
{

/// Turns "run the world at S times real time" into "the world should have
/// reached THIS instant by now" -- which the control node turns into running
/// or pausing it.
///
/// WHY THE SIMULATOR IS NOT SIMPLY TOLD A REAL-TIME FACTOR.
///
/// Gazebo has a service for exactly this -- `/world/<name>/set_physics` with a
/// `real_time_factor` -- and it must never be called. In gz-sim 8.11.0,
/// `SimulationRunner::UpdatePhysicsParams()` assigns the world's gravity from
/// the request unconditionally, and gz.msgs.Physics has no gravity field: for
/// proto3 the absent field reads as (0, 0, 0). So every call, with every
/// payload, makes the world weightless.
///
/// Nothing reports it. The service answers `data: true`, `generate_world_sdf`
/// still prints `<gravity>0 0 -9.8</gravity>`, and the IMU carries on reading a
/// clean 1 g, so every health check in this repo stays green. The only symptom
/// is that the vehicle leaves. An overnight run was found at 41.5 km, still
/// climbing.
///
/// It was proved before it was explained, by sending the world the real-time
/// factor it already had -- a request that changes nothing -- and watching the
/// vehicle depart within one second. `real_time_factor` alone, `max_step_size`
/// alone and both together all do it. The value is innocent; the service is
/// not. (Upstream has since guarded the assignment with `has_gravity()`; this
/// code must keep working on the version that ships with Jazzy.)
///
/// `/world/<name>/control` is a different service and is safe: pause, resume,
/// `run_to_sim_time` and reset all go through it and leave gravity alone.
/// Reset in fact REPAIRS a world that has already been ruined.
///
/// WHY THE MECHANISM IS PAUSE AND RESUME.
///
/// The world is paced with the one verb that stores nothing: it runs at its
/// own rate for a fraction of each tick and is stopped for the rest, and that
/// fraction IS the speed. `multi_step` and `run_to_sim_time` were both tried
/// before this and both looked wrong in testing -- but that testing was
/// invalid, so no claim is made here about either of them. (Three copies of
/// the control node were running at once, left behind by a teardown script
/// whose list of processes to kill was hand-written and had not been updated;
/// all three were pacing the same world, and every speed came out at two or
/// three times what was asked for. See scripts/kill_sim.sh, which now derives
/// that list instead.)
///
/// What can be said for pause is that it is measured, with one control node
/// running: 0.05x -> 0.050x, 0.25x -> 0.255x, 0.50x -> 0.497x, 0.75x ->
/// 0.755x. It also puts nothing in the simulator that can accumulate, and it
/// sends a message only when the world's state has to change -- between two
/// and fourteen a second, rather than one per tick.
///
/// THE CEILING IS 1x, AND IT IS REAL. The world is throttled to real time by
/// <real_time_factor> in its SDF, and stepping does not bypass that throttle:
/// `multi_step: 10000` -- ten seconds of simulated time, requested in one
/// message -- took 10.38 seconds of wall clock, measured with nothing else
/// touching the world. The only way to raise the throttle is `set_physics`,
/// which is the call that deletes gravity. So faster-than-real-time playback
/// is not offered at all, rather than offered and quietly not delivered. The
/// slider that used to go to 4x was doing the latter.
///
/// Falling behind is normal -- the machine has other work -- and is bounded by
/// `max_slip_s`: the target is never allowed to get further ahead of the world
/// than that, so an interruption is forgiven rather than repaid in a sprint.
/// That makes the achieved speed lower than the requested one, which is why
/// the node measures and publishes what it actually got instead of echoing
/// back what it was asked for.
struct PacerConfig
{
  /// The world's physics step. Read from the SDF, not assumed. Targets are
  /// quantised to it so the world lands on the step grid rather than a
  /// fraction past it.
  double step_s {0.001};
  /// How far ahead of the world the target may get.
  ///
  /// Must comfortably exceed the interval between world-statistics messages
  /// (0.2 s), because the world's position is only known that often: a limit
  /// tighter than the staleness of the measurement would clamp the target to
  /// behind where the world already is, and the world would stall.
  double max_slip_s {0.5};
};

class Pacer
{
public:
  explicit Pacer(PacerConfig cfg = {})
  : cfg_(cfg) {}

  /// Simulated seconds per wall second. Values are range-checked by the
  /// caller; a non-positive one would mean "never advance", which is what
  /// pausing is for, so it is refused here rather than silently becoming a
  /// stall.
  void setSpeed(double speed)
  {
    if (speed > 0.0 && std::isfinite(speed)) {speed_ = speed;}
  }
  double speed() const {return speed_;}

  /// Adopt `sim_now` as the target, discarding any accumulated lead.
  ///
  /// For the moments when the previous target means nothing: resuming from a
  /// pause, a reset, or the first tick after taking control of the world.
  /// Letting the old target stand after a reset would tell the world to run
  /// forward through the whole of a run that no longer exists.
  void resync(double sim_now)
  {
    target_ = sim_now;
    have_target_ = true;
    backlog_ = 0.0;
  }

  /// The simulated instant the world should have reached by now, given the
  /// wall time since the last call and where the world actually is.
  ///
  /// The node runs the world while it is behind this and pauses it while it is
  /// not. Idempotent in the sense that matters: calling it again with no wall
  /// time elapsed returns the same instant, so an extra call cannot make the
  /// world run further.
  double target(double wall_dt, double sim_now)
  {
    if (!have_target_) {resync(sim_now);}
    if (wall_dt > 0.0 && std::isfinite(wall_dt)) {target_ += speed_ * wall_dt;}

    // Bounded in BOTH directions, and neither bound snaps the target onto the
    // world's own position.
    //
    // Behind: the world could not keep up. Forgive the debt rather than store
    // it up and repay it as a sprint.
    if (target_ - sim_now > cfg_.max_slip_s) {target_ = sim_now + cfg_.max_slip_s;}
    // Ahead: the world overran, because a pause takes effect a moment after it
    // is asked for. That overrun is time the world owes back, and it pays it
    // by staying stopped a little longer -- so the target must NOT be dragged
    // up to meet it. Doing exactly that was a real bug: the world outran the
    // target, the target followed it, and 0.25x came out as 0.63x.
    if (sim_now - target_ > cfg_.max_slip_s) {target_ = sim_now - cfg_.max_slip_s;}

    backlog_ = target_ - sim_now;
    if (!(cfg_.step_s > 0.0)) {return target_;}
    return std::round(target_ / cfg_.step_s) * cfg_.step_s;
  }

  /// Simulated seconds between the world and its target at the last call.
  /// Published, so "the slider says 2x and it is running at 1.3x" is visible
  /// rather than something the user has to time with a stopwatch.
  double backlog() const {return backlog_;}

  const PacerConfig & config() const {return cfg_;}

private:
  PacerConfig cfg_;
  double speed_ {1.0};
  double target_ {0.0};
  double backlog_ {0.0};
  bool have_target_ {false};
};

}  // namespace dsim_simctl
