// The only write path into the running simulator.
//
// Three verbs -- pause, speed, reset -- and nothing else. It cannot arm a
// vehicle, retune a controller, spawn a model or run a command, and that
// narrowness is deliberate: the HTTP server that forwards to it is reachable
// from the whole tailnet.
//
// WHY THIS NODE EXISTS AT ALL.
//
// Playback speed used to be set with `gz service -s /world/<name>/set_physics`
// from Python. Two things were wrong with that, one fatal:
//
//   * set_physics destroys the world. gz.msgs.Physics has no gravity field and
//     gz-sim assigns gravity from it anyway, so any call, with any payload,
//     leaves the world weightless -- silently, with every health check still
//     green. The vehicle simply leaves. See pacer.hpp for how that was proved
//     and why stepping replaces it.
//   * Shelling out to the `gz` CLI costs 305 ms of process startup per call,
//     regardless of what is being asked, and `subprocess.run(timeout=)` kills
//     the `gz` wrapper but not the `gz-transport-topic` helper it forked. One
//     of those orphans was found still alive 11.3 hours after the request that
//     spawned it timed out.
//
// Both problems come from the same place: the write path was a shell command
// in a Python process. This node holds ONE gz-transport connection for the
// life of the simulator, so a step request costs a message rather than a
// process, and the verbs it exposes are the only ones that exist.
//
// It runs on the WALL clock, and it must. Pacing means holding the world
// paused and handing it steps; a node whose own timers ran on simulated time
// would stop ticking the instant it paused the world and could never start it
// again. That is why use_sim_time is checked at startup and refused.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <dsim_msgs/msg/sim_state.hpp>
#include <dsim_msgs/srv/sim_control.hpp>
#include <dsim_simctl/pacer.hpp>

namespace dsim_simctl
{

using SimControlSrv = dsim_msgs::srv::SimControl;

class SimControlNode : public rclcpp::Node
{
public:
  SimControlNode()
  : Node("dsim_simctl")
  {
    world_ = require<std::string>("world");
    // Read from the SDF by the launch file rather than assumed. The pacer
    // counts in steps, so a step size that disagrees with the simulator's
    // would make every speed wrong by the ratio between them without any
    // single step ever being incorrect.
    cfg_.step_s = require<double>("step_size_s");
    tick_hz_ = declare_parameter("tick_hz", 20.0);
    min_speed_ = declare_parameter("min_speed", 0.05);
    // One, and not a decision: the world is throttled to real time by its SDF,
    // and stepping does not bypass the throttle (measured: ten seconds of
    // simulated time took 10.38 s of wall clock). Raising it needs the service
    // that deletes gravity. Offering 4x and delivering 1x would be worse than
    // not offering it -- and it is exactly what the old slider did.
    max_speed_ = declare_parameter("max_speed", 1.0);
    state_hz_ = declare_parameter("state_rate_hz", 5.0);
    cfg_.max_slip_s = declare_parameter("max_slip_s", 0.5);

    if (get_parameter("use_sim_time").as_bool()) {
      RCLCPP_FATAL(
        get_logger(),
        "use_sim_time must be false for this node: it pauses the world, and a "
        "node timed by the world it has paused never ticks again");
      throw std::runtime_error("dsim_simctl requires use_sim_time:=false");
    }

    pacer_ = std::make_unique<Pacer>(cfg_);

    control_topic_ = "/world/" + world_ + "/control";
    if (!gz_.Subscribe(
        "/world/" + world_ + "/stats", &SimControlNode::onStats, this))
    {
      RCLCPP_ERROR(
        get_logger(), "could not subscribe to /world/%s/stats — is the world name right?",
        world_.c_str());
      error_ = "no world statistics; check the world name";
    }

    state_pub_ = create_publisher<dsim_msgs::msg::SimState>(
      "/sim/state", rclcpp::QoS(1).transient_local());
    srv_ = create_service<SimControlSrv>(
      "/sim/control",
      [this](
        const SimControlSrv::Request::SharedPtr req,
        SimControlSrv::Response::SharedPtr res) {onCommand(*req, *res);});

    tick_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / tick_hz_), [this] {tick();});
    state_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / state_hz_), [this] {publishState();});

    RCLCPP_INFO(
      get_logger(),
      "sim control up on world '%s': step %.4f s, tick %.0f Hz, speed %.2f-%.2f, "
      "slip tolerance %.2f s",
      world_.c_str(), cfg_.step_s, tick_hz_, min_speed_, max_speed_, cfg_.max_slip_s);
  }

private:
  /// A parameter with no default.
  ///
  /// The world name and the step size both exist in exactly one place already
  /// -- the world SDF, read by the launch file. A default here would be a
  /// second copy, and the copy that drifts is always the one nobody tests: a
  /// wrong world name makes every command a silent no-op, and a wrong step
  /// size makes every speed wrong by a constant factor.
  template<typename T>
  T require(const std::string & name)
  {
    declare_parameter<T>(name);
    T value;
    if (!get_parameter(name, value)) {
      RCLCPP_FATAL(get_logger(), "required parameter '%s' was not set", name.c_str());
      throw std::runtime_error("missing required parameter: " + name);
    }
    return value;
  }

  // ---- what the simulator says --------------------------------------------

  void onStats(const gz::msgs::WorldStatistics & msg)
  {
    const double sim = msg.sim_time().sec() + 1e-9 * msg.sim_time().nsec();
    const auto now = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(mutex_);
    // Achieved speed, measured over a window rather than between consecutive
    // messages: at 5 Hz a single interval is short enough that scheduling
    // noise dominates, and a jittery number in the corner of the viewer reads
    // as a broken simulator.
    if (have_window_) {
      const double wall =
        std::chrono::duration<double>(now - window_wall_).count();
      if (wall >= 1.0) {
        achieved_ = (sim - window_sim_) / wall;
        window_sim_ = sim;
        window_wall_ = now;
      }
      // Simulated time going backwards is a reset -- ours or someone else's.
      // Either way the window is measuring across a discontinuity.
      if (sim < stats_sim_s_ - 0.05) {
        window_sim_ = sim;
        window_wall_ = now;
        achieved_ = 0.0;
        pacer_->resync(sim);
        est_sim_ = sim;
        last_target_ = sim;
      }
    } else {
      window_sim_ = sim;
      window_wall_ = now;
      have_window_ = true;
    }
    stats_sim_s_ = sim;
    est_sim_ = sim;
    stats_paused_ = msg.paused();
    have_stats_ = true;
  }

  // ---- driving the world ---------------------------------------------------

  /// Start or stop the world, as part of pacing it.
  ///
  /// Fire-and-forget: the reply carries no information the next tick will not
  /// discover anyway, and waiting for it would put a round trip inside the
  /// pacing loop. A lost message costs at most one tick, because the next tick
  /// re-derives the state the world should be in rather than assuming the last
  /// message arrived.
  void requestRunning(bool running)
  {
    gz::msgs::WorldControl req;
    req.set_pause(!running);
    const bool sent = gz_.Request(
      control_topic_, req, &SimControlNode::onPaceReply, this);
    if (!sent) {++step_errors_;}
  }

  /// A refused request is counted, not logged. At 20 Hz a broken world would
  /// produce twenty log lines a second; the count is published in SimState
  /// where a viewer can show it once.
  void onPaceReply(const gz::msgs::Boolean & rep, const bool result)
  {
    if (!result || !rep.data()) {++step_errors_;}
  }

  /// Blocking, because the caller is a user waiting for an answer and the
  /// reply is the answer. Rare by construction: pause, speed change, reset.
  bool requestBlocking(const gz::msgs::WorldControl & req, unsigned timeout_ms)
  {
    gz::msgs::Boolean rep;
    bool result = false;
    const bool ok = gz_.Request(control_topic_, req, timeout_ms, rep, result);
    const bool good = ok && result && rep.data();
    // Every non-step command this node sends to the simulator leaves a line.
    // When the write path last broke a run there was no record of what had
    // been sent, and the sequence had to be reconstructed from the spacing of
    // unrelated log messages.
    RCLCPP_INFO(
      get_logger(), "-> world control {%s} : %s",
      req.ShortDebugString().c_str(), good ? "ok" : "REFUSED");
    return good;
  }

  void tick()
  {
    const auto now = std::chrono::steady_clock::now();
    double wall_dt = 0.0;
    if (have_tick_) {
      wall_dt = std::chrono::duration<double>(now - last_tick_).count();
    }
    last_tick_ = now;
    have_tick_ = true;

    bool want_running = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_stats_ || paused_ || !pacing_) {return;}
      ++ticks_;

      // Where the world is, between statistics messages. Those arrive five
      // times a second, which is far too coarse to decide a 50 ms tick on, so
      // the estimate runs forward at the world's own rate while it is running
      // and is snapped back to the truth whenever a message lands.
      if (world_running_) {est_sim_ += wall_dt;}

      last_target_ = pacer_->target(wall_dt, est_sim_);
      backlog_ = pacer_->backlog();
      // Behind schedule: let it run. Caught up: stop it. The fraction of the
      // time it spends running IS the playback speed.
      want_running = last_target_ > est_sim_;
      if (want_running == world_running_) {return;}
      world_running_ = want_running;
      ++run_requests_;
    }
    requestRunning(want_running);
  }

  // ---- commands ------------------------------------------------------------

  void onCommand(const SimControlSrv::Request & req, SimControlSrv::Response & res)
  {
    switch (req.command) {
      case SimControlSrv::Request::SET_PAUSED:
        res.ok = setPaused(req.paused, res.message);
        break;
      case SimControlSrv::Request::TOGGLE_PAUSED:
        res.ok = setPaused(!paused(), res.message);
        break;
      case SimControlSrv::Request::SET_SPEED:
        res.ok = setSpeed(req.speed, res.message);
        break;
      case SimControlSrv::Request::RESET:
        res.ok = reset(res.message);
        break;
      default:
        res.ok = false;
        res.message = "unknown command";
        break;
    }
    res.state = buildState();
  }

  bool paused()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return paused_;
  }

  /// Put the world into the state this node intends, and record that it did.
  ///
  /// One function, because there are three ways to reach a new intent -- the
  /// pause button, the speed slider and a reset -- and having each work out
  /// its own request is how they drift apart. A reset did exactly that: it
  /// resumed the world (Gazebo's reset clears the pause) while this node went
  /// on reporting `paused: true`, so the viewer showed PAUSED over a vehicle
  /// that was flying.
  ///
  /// "Running" here means running NATIVELY, at real time. Any other speed
  /// leaves the world paused and lets the pacing tick start and stop it, which
  /// is why one button produces two different requests.
  bool applyRunMode()
  {
    bool run_natively;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      run_natively = !paused_ && !pacing_;
      world_running_ = run_natively;
    }
    gz::msgs::WorldControl req;
    req.set_pause(!run_natively);
    return requestBlocking(req, 2000);
  }

  /// Start the pacer from where the world actually is. Called on every
  /// transition into running, because a target left over from before a pause
  /// or a reset is a debt the world never owed.
  void resyncLocked(double sim_now)
  {
    pacer_->resync(sim_now);
    est_sim_ = sim_now;
    last_target_ = sim_now;
  }

  bool setPaused(bool paused, std::string & message)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      paused_ = paused;
      pacing_ = !paused && !atRealTime();
      if (!paused) {resyncLocked(stats_sim_s_);}
    }
    if (!applyRunMode()) {
      message = "the simulator refused the pause request";
      return false;
    }
    message = paused ? "paused" : (pacing_ ? "running, paced" : "running");
    return true;
  }

  bool setSpeed(double speed, std::string & message)
  {
    if (!std::isfinite(speed)) {
      message = "speed must be a real number";
      return false;
    }
    if (speed < min_speed_ || speed > max_speed_) {
      message = "speed must be between " + std::to_string(min_speed_) +
        " and " + std::to_string(max_speed_);
      return false;
    }

    bool want_pacing;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      speed_ = speed;
      pacer_->setSpeed(speed);
      want_pacing = !paused_ && !atRealTime();
      pacing_ = want_pacing;
      if (want_pacing) {resyncLocked(stats_sim_s_);}
    }
    if (!applyRunMode()) {
      message = "the simulator refused to change run mode";
      return false;
    }
    // A speed set while paused is remembered, not applied: the world stays
    // stopped and the new speed takes effect on resume. Saying "running at
    // real time" here, which is what this used to do, told a user their paused
    // world was running.
    message = paused_ ? "paused; the new speed applies on resume"
      : (want_pacing ? "paced" : "running at real time");
    return true;
  }

  /// Block until the world's clock restarts, or give up.
  ///
  /// The reply to a reset means "accepted", not "done": Gazebo applies it on a
  /// later update. Runs on the service thread while world statistics keep
  /// arriving on Gazebo's own, so there is nothing here to deadlock against.
  bool waitForClockReset(double before_s, double timeout_s)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stats_sim_s_ < before_s - 0.05) {return true;}
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  bool reset(std::string & message)
  {
    double before;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      before = stats_sim_s_;
    }

    gz::msgs::WorldControl req;
    req.mutable_reset()->set_all(true);
    if (!requestBlocking(req, 4000)) {
      message = "the simulator refused the reset";
      return false;
    }

    // Wait for it to actually land before touching the world again. Gazebo
    // holds ONE pending world-control state, so a second message sent before
    // the first has been applied replaces it and the reset is silently
    // dropped. Measured: a reset followed immediately by the resume below left
    // the clock at 66.7 s and the referee's metrics untouched, while the reply
    // said the reset had been accepted -- which it had.
    const bool landed = waitForClockReset(before, 2.0);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++resets_;
      // A target built from the old run would make the pacer run through the
      // whole of it. Anchored to where the world actually is, not to zero, so
      // that a reset which did not land does not leave the pacer lost as well.
      resyncLocked(stats_sim_s_);
      achieved_ = 0.0;
      have_window_ = false;
      step_errors_ = 0;
    }
    // Gazebo's reset also RESUMES the world, whatever it was doing before, so
    // the intent has to be re-asserted. Without this a reset pressed on a
    // paused simulator set it running while this node went on reporting it
    // paused -- and the viewer showed PAUSED over a flying vehicle.
    if (!applyRunMode()) {
      message = "world reset, but it would not go back to its previous state";
      return false;
    }
    if (!landed) {
      message = "the simulator accepted the reset but its clock did not restart";
      return false;
    }
    // Nothing is told to reset its metrics from here. Every node that
    // accumulates watches the clock instead (dsim_time/sim_epoch.hpp), which
    // also covers a reset from the Gazebo GUI or from a terminal -- neither of
    // which would ever send us a message.
    message = "world reset";
    return true;
  }

  /// Whether the requested speed is real time, to within a step's worth of
  /// slider resolution.
  bool atRealTime() const {return std::abs(speed_ - 1.0) < 1e-9;}

  // ---- telemetry -----------------------------------------------------------

  dsim_msgs::msg::SimState buildState()
  {
    dsim_msgs::msg::SimState s;
    s.header.stamp = now();
    std::lock_guard<std::mutex> lock(mutex_);
    s.enabled = have_stats_;
    // What the SIMULATOR says, not what we asked for: a request that never
    // arrived must show up as a disagreement, not be papered over.
    s.paused = paused_;
    s.pacing = pacing_;
    s.requested_speed = speed_;
    s.achieved_speed = paused_ ? 0.0 : achieved_;
    s.min_speed = min_speed_;
    s.max_speed = max_speed_;
    s.sim_time_s = stats_sim_s_;
    s.step_size_s = cfg_.step_s;
    s.backlog_s = pacing_ ? backlog_ : 0.0;
    s.target_sim_time_s = last_target_;
    s.resets = resets_;
    s.step_errors = step_errors_;
    s.ticks = ticks_;
    s.run_requests = run_requests_;
    s.error = have_stats_ ? error_ : "no world statistics; check the world name";
    return s;
  }

  void publishState() {state_pub_->publish(buildState());}

  // ---- state ---------------------------------------------------------------

  std::string world_, control_topic_, error_;
  PacerConfig cfg_;
  std::unique_ptr<Pacer> pacer_;
  double tick_hz_ {20.0}, state_hz_ {5.0}, min_speed_ {0.05}, max_speed_ {4.0};

  std::mutex mutex_;
  bool paused_ {false}, pacing_ {false};
  bool have_stats_ {false}, have_window_ {false}, stats_paused_ {false};
  double speed_ {1.0};
  double stats_sim_s_ {0.0}, est_sim_ {0.0}, last_target_ {0.0};
  double achieved_ {0.0}, backlog_ {0.0};
  /// What this node believes the world is doing right now, which is what lets
  /// it send a message only when that changes rather than twenty a second.
  bool world_running_ {true};
  double window_sim_ {0.0};
  std::chrono::steady_clock::time_point window_wall_, last_tick_;
  bool have_tick_ {false};
  std::atomic<unsigned int> step_errors_ {0};
  unsigned int resets_ {0};
  std::uint64_t ticks_ {0}, run_requests_ {0};

  gz::transport::Node gz_;
  rclcpp::Publisher<dsim_msgs::msg::SimState>::SharedPtr state_pub_;
  rclcpp::Service<SimControlSrv>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr tick_timer_, state_timer_;
};

}  // namespace dsim_simctl

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dsim_simctl::SimControlNode>());
  rclcpp::shutdown();
  return 0;
}
