// Scoring node for planner trials.
//
// Measures against GROUND TRUTH (/drone/truth), never against the
// degraded estimate the controller sees. Scoring off the noisy estimate would
// hide exactly the errors the estimate causes, and a plan that flies into a
// wall would still score well if the estimate said otherwise.
//
// Metrics:
//   tracking error   | truth position vs the setpoint the controller tracked
//   collision        | contact sensor on the guard envelope, AND the analytic
//                    | clearance going negative (edge-triggered, with position)
//   clearance        | analytic SIGNED distance to the scenario's obstacles,
//                    | whose centres follow a Bezier in space-time and which
//                    | exist only inside their own active window
//   path length      | integrated ground-truth travel
//   energy           | momentum-theory estimate anchored to the data sheet
//
// Clearance uses the world's declared obstacle list rather than the depth
// sensor, on purpose: this is the referee, and the referee must not share the
// planner's blind spots.
//
// Obstacles run on the SCENARIO clock: t = sim time - scenario.start_s, zero
// when the plan begins executing. They are NOT Gazebo bodies -- they have no
// collision geometry and two of the shipped scenarios move theirs along cubics
// while three switch them off partway through, none of which a constant-velocity
// rigid body can express. What this node publishes IS the obstacle field, and
// the viewer draws from it. scripts/check_planner.py holds those positions
// against the planner's own obstacle_positions_at(), which is an independent
// implementation of the same motion -- a stronger check than comparing the
// simulator against its own staging.

#include <cmath>
#include <fstream>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <actuator_msgs/msg/actuators.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <dsim_msgs/msg/clearance_report.hpp>
#include <dsim_msgs/msg/flight_status.hpp>
#include <dsim_msgs/msg/trajectory_setpoint.hpp>
#include <dsim_msgs/srv/reset_run.hpp>
#include <dsim_time/sim_epoch.hpp>
#include "dsim_eval/obstacle.hpp"

namespace dsim_eval
{

class EvalNode : public rclcpp::Node
{
public:
  EvalNode()
  : Node("dsim_eval")
  {
    vehicle_radius_ = declare_parameter("vehicle_radius_m", 0.25);
    hover_power_w_ = declare_parameter("hover_power_w", 438.0);
    mass_ = declare_parameter("vehicle.mass_kg", 1.5);
    motor_constant_ = declare_parameter("vehicle.motor_constant", 3.29e-06);
    csv_path_ = declare_parameter("csv_path", std::string(""));
    publish_hz_ = declare_parameter("publish_rate_hz", 20.0);
    // A vehicle sitting on the pad is touching the ground; that is not a crash.
    // It only becomes one once the vehicle has actually been airborne and then
    // touches down again -- i.e. it fell out of the sky.
    airborne_altitude_m_ = declare_parameter("airborne_altitude_m", 0.5);
    // Tracking error during takeoff is dominated by the initial position offset,
    // not by the plan. Including it makes short runs look far worse than long
    // ones for no real reason, so the RMSE window starts after this.
    warmup_s_ = declare_parameter("warmup_s", 6.0);
    // Scenario clock origin, in simulated seconds. Comes from the generated
    // config/obstacles_<world>.yaml for a space-time scenario; 0 for the
    // static courses, where nothing moves and the clock is unused.
    scenario_start_s_ = declare_parameter("scenario.start_s", 0.0);
    scenario_duration_s_ = declare_parameter("scenario.duration_s", 0.0);
    scenario_name_ = declare_parameter("scenario.name", std::string(""));

    loadObstacles();
    tracker_ = std::make_unique<ClearanceTracker>(obstacles_, vehicle_radius_, stations_);

    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/truth", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {onTruth(*m);});
    sp_sub_ = create_subscription<dsim_msgs::msg::TrajectorySetpoint>(
      "/drone/setpoint", rclcpp::QoS(10),
      [this](dsim_msgs::msg::TrajectorySetpoint::SharedPtr m) {
        have_sp_ = true; sp_ = *m;
      });
    contact_sub_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/drone/contacts", rclcpp::QoS(10),
      [this](ros_gz_interfaces::msg::Contacts::SharedPtr m) {onContacts(*m);});
    cmd_sub_ = create_subscription<actuator_msgs::msg::Actuators>(
      "/drone/command/motor_speed", rclcpp::QoS(1),
      [this](actuator_msgs::msg::Actuators::SharedPtr m) {onCommand(*m);});

    status_pub_ = create_publisher<dsim_msgs::msg::FlightStatus>(
      "/drone/eval/status", rclcpp::QoS(10));
    clearance_pub_ = create_publisher<dsim_msgs::msg::ClearanceReport>(
      "/drone/eval/clearance", rclcpp::QoS(10));

    reset_srv_ = create_service<dsim_msgs::srv::ResetRun>(
      "/drone/eval/reset",
      [this](
        const dsim_msgs::srv::ResetRun::Request::SharedPtr,
        dsim_msgs::srv::ResetRun::Response::SharedPtr res) {
        reset();
        res->success = true;
        res->message = "metrics reset (vehicle NOT moved — use scripts/reset_pose.sh for that)";
      });

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / publish_hz_),
      [this] {publishStatus();});

    if (!csv_path_.empty()) {
      csv_.open(csv_path_, std::ios::out | std::ios::trunc);
      if (csv_) {
        csv_ << "t,x,y,z,ref_x,ref_y,ref_z,err,rmse,clearance,collided,energy_wh\n";
        RCLCPP_INFO(get_logger(), "logging to %s", csv_path_.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "could not open %s for writing", csv_path_.c_str());
      }
    }
    RCLCPP_INFO(get_logger(),
      "eval up: scenario '%s', %zu obstacles, vehicle radius %.2f m, "
      "scenario t=0 at sim %.1f s, horizon %.1f s",
      scenario_name_.c_str(), obstacles_.size(), vehicle_radius_,
      scenario_start_s_, scenario_duration_s_);
  }

private:
  void loadObstacles()
  {
    // Obstacles are declared as a flat list because ROS 2 parameters have no
    // native array-of-struct type, and each one's motion is a Bezier in
    // space-time flattened to a single array of doubles:
    //   obstacles.names: ["F0", ...]
    //   obstacles.F0.type   "sphere" | "column"
    //   obstacles.F0.radius
    //   obstacles.F0.control_points: [x,y,z,t, x,y,z,t, ...]
    // Four numbers per control point, at least two points. This is the
    // planner's own canonical form, not a translation of it, so the referee
    // and the solver cannot disagree about where anything is or when.
    const auto names = declare_parameter<std::vector<std::string>>(
      "obstacles.names", std::vector<std::string>{});
    for (const auto & n : names) {
      Obstacle o;
      o.name = n;
      o.type = declare_parameter("obstacles." + n + ".type", std::string("sphere"));
      o.radius = declare_parameter("obstacles." + n + ".radius", 1.0);
      const auto flat = declare_parameter<std::vector<double>>(
        "obstacles." + n + ".control_points", std::vector<double>{});
      if (flat.size() < 8 || flat.size() % 4 != 0) {
        // Refuse rather than score against half an obstacle. A truncated
        // control polygon still produces a number, and that number is the
        // thing this node exists to be trusted about.
        throw std::runtime_error(
          "obstacle " + n + " has " + std::to_string(flat.size()) +
          " control-point numbers; need a multiple of 4, at least 8 (x,y,z,t per point)");
      }
      for (size_t i = 0; i < flat.size(); i += 4) {
        o.control_points.push_back({flat[i], flat[i + 1], flat[i + 2], flat[i + 3]});
      }
      if (o.tEnd() < o.tStart()) {
        throw std::runtime_error("obstacle " + n + " has control point times running backwards");
      }
      obstacles_.push_back(o);
    }

    // Stations: fixed ground points the vehicle must stay visible from, as a
    // flat [x,y,z, x,y,z, ...] for the same reason the control points are.
    // Most scenarios have none, and then line of sight is not constrained and
    // not reported -- an unconstrained margin is +inf, never 0.
    const auto flat_stations = declare_parameter<std::vector<double>>(
      "stations", std::vector<double>{});
    if (flat_stations.size() % 3 != 0) {
      throw std::runtime_error(
              "stations has " + std::to_string(flat_stations.size()) +
              " numbers; need a multiple of 3 (x,y,z per station)");
    }
    for (size_t i = 0; i < flat_stations.size(); i += 3) {
      stations_.push_back({flat_stations[i], flat_stations[i + 1], flat_stations[i + 2]});
    }
  }

  double scenarioTime(double sim_s) const {return sim_s - scenario_start_s_;}

  static bool isDrone(const std::string & name)
  {
    return name.rfind("drone::", 0) == 0;
  }

  static bool isGround(const std::string & name)
  {
    // Entity names look like "ground_plane::link::collision" and
    // "drone::base_link::envelope" -- captured from a live contact message,
    // not guessed.
    return name.find("ground_plane") != std::string::npos;
  }

  void onContacts(const ros_gz_interfaces::msg::Contacts & m)
  {
    for (const auto & c : m.contacts) {
      const bool ground =
        isGround(c.collision1.name) || isGround(c.collision2.name);

      if (ground && !was_airborne_) {
        // Resting on the pad before takeoff. Counting this as a crash made
        // every run in an empty world report a collision.
        ++ground_contacts_;
        continue;
      }

      // Report what we HIT, which is whichever party is not the vehicle.
      // Reporting collision1 blindly named the drone's own envelope, which
      // tells you nothing about what the plan ran into.
      const std::string what = ground
        ? std::string("ground")
        : (isDrone(c.collision1.name) ? c.collision2.name : c.collision1.name);
      if (!collided_) {
        RCLCPP_ERROR(
          get_logger(), "COLLISION with %s at t=%.2f s", what.c_str(), elapsed_);
      }
      collided_ = true;
      ++collision_count_;
    }
  }

  void onCommand(const actuator_msgs::msg::Actuators & m)
  {
    // Momentum-theory scaling anchored to the data sheet's hover draw:
    //   P / P_hover = (T / T_hover)^1.5
    // This is a MODEL, not a measurement. It is anchored to one data-sheet
    // number and has never been compared against a real battery, so it is
    // a figure for ranking plans against each other and nothing else.
    double thrust = 0.0;
    for (const auto & w : m.velocity) {thrust += motor_constant_ * w * w;}
    const double hover_thrust = mass_ * 9.80665;
    if (hover_thrust <= 0.0) {return;}
    const double ratio = std::max(0.0, thrust / hover_thrust);
    const double power_w = hover_power_w_ * std::pow(ratio, 1.5);

    const double now = get_clock()->now().seconds();
    if (last_power_s_ > 0.0) {
      energy_wh_ += power_w * (now - last_power_s_) / 3600.0;
    }
    last_power_s_ = now;
  }

  void onTruth(const nav_msgs::msg::Odometry & m)
  {
    const double now = get_clock()->now().seconds();
    // Checked here rather than on the publish timer because this is where
    // everything accumulates. One truth sample attributed to the wrong run is
    // one sample of a 3.8 km excursion inside the sum of squares for a flight
    // that never left 5 cm.
    if (epoch_.restarted(now)) {
      RCLCPP_INFO(
        get_logger(), "simulated time went backwards — scoring a new run (reset %zu)",
        epoch_.epochs());
      reset();
    }
    const double px = m.pose.pose.position.x;
    const double py = m.pose.pose.position.y;
    const double pz = m.pose.pose.position.z;

    if (t0_ <= 0.0) {t0_ = now;}
    elapsed_ = now - t0_;

    if (pz > airborne_altitude_m_) {was_airborne_ = true;}

    if (have_last_pos_) {
      path_length_ += std::sqrt(
        (px - lx_) * (px - lx_) + (py - ly_) * (py - ly_) + (pz - lz_) * (pz - lz_));
    }
    lx_ = px; ly_ = py; lz_ = pz;
    have_last_pos_ = true;

    // Signed clearance to the nearest declared obstacle, minus the guard
    // envelope, with the moving ones where they are NOW on the scenario clock.
    // Going negative is a collision, recorded once per excursion with the
    // position it happened at -- see dsim_eval/obstacle.hpp.
    const size_t hits_before = tracker_->hits().size();
    // The instant this whole report describes. Held so publishClearance uses
    // it for the obstacle positions AND the margins, rather than stamping the
    // positions with the publish time while the margins come from here.
    //
    // Those are different instants, and at a window boundary they disagree:
    // the referee reported scenario t = 0.00 with a sight-line margin measured
    // a few milliseconds earlier, when the obstacle whose window opens at 0 did
    // not exist yet -- so the report said "unconstrained at t=0" while the
    // planner, asked about t=0, said +49.76 m. One report, one instant.
    last_sample_t_ = scenarioTime(now);
    last_sample_p_ = {px, py, pz};
    have_sample_ = true;
    const auto sample = tracker_->update(px, py, pz, last_sample_t_);
    // Taken unconditionally, infinity included. Keeping the last finite value
    // when every obstacle has gone left `door3d` reporting 0.82 m of clearance
    // to a door that had opened five seconds earlier -- a stale number that
    // looks exactly like a live one.
    last_clearance_ = sample.clearance;
    nearest_ = sample.nearest;
    last_los_ = sample.los_margin;
    los_blocker_ = sample.los_blocker;
    los_station_ = sample.los_station;
    if (tracker_->hits().size() > hits_before) {
      const auto & h = tracker_->hits().back();
      RCLCPP_ERROR(
        get_logger(), "COLLISION (analytic) with %s at (%.2f, %.2f, %.2f), scenario t=%.2f s",
        h.with.c_str(), h.x, h.y, h.z, h.t);
      collided_ = true;
      ++collision_count_;
    }

    if (have_sp_) {
      const double ex = px - sp_.position.x;
      const double ey = py - sp_.position.y;
      const double ez = pz - sp_.position.z;
      err_ = std::sqrt(ex * ex + ey * ey + ez * ez);
      if (elapsed_ >= warmup_s_) {
        sq_sum_ += err_ * err_;
        ++n_samples_;
        max_err_ = std::max(max_err_, err_);
      }
    }

    if (csv_ && csv_.is_open()) {
      csv_ << elapsed_ << ',' << px << ',' << py << ',' << pz << ','
           << (have_sp_ ? sp_.position.x : 0.0) << ','
           << (have_sp_ ? sp_.position.y : 0.0) << ','
           << (have_sp_ ? sp_.position.z : 0.0) << ','
           << err_ << ',' << rmse() << ','
           << (std::isfinite(last_clearance_) ? last_clearance_
               : std::numeric_limits<double>::quiet_NaN()) << ','
           << (collided_ ? 1 : 0) << ',' << energy_wh_ << '\n';
    }
  }

  double rmse() const
  {
    return (n_samples_ > 0) ? std::sqrt(sq_sum_ / static_cast<double>(n_samples_)) : 0.0;
  }

  void publishStatus()
  {
    dsim_msgs::msg::FlightStatus s;
    s.header.stamp = get_clock()->now();
    s.armed = true;
    s.trajectory_active = have_sp_;
    s.collided = collided_;
    s.collision_count = static_cast<uint32_t>(collision_count_);
    s.tracking_error_m = err_;
    s.tracking_rmse_m = rmse();
    s.tracking_max_m = max_err_;
    // NaN, not -1: with no obstacles in the world there IS no clearance, and
    // -1.0 reads exactly like "penetrated by one metre".
    s.min_obstacle_clearance_m = std::isfinite(tracker_->minClearance())
      ? tracker_->minClearance()
      : std::numeric_limits<double>::quiet_NaN();
    s.path_length_m = path_length_;
    s.elapsed_s = elapsed_;
    s.energy_wh = energy_wh_;
    status_pub_->publish(s);
    publishClearance(s.header.stamp);
  }

  void publishClearance(const builtin_interfaces::msg::Time & stamp)
  {
    dsim_msgs::msg::ClearanceReport r;
    r.header.stamp = stamp;
    // The time of the last ground-truth sample, not of this publish: every
    // number below was measured at that instant. Before any state has arrived
    // there is nothing to describe, so the clock is the only answer available.
    const double now = get_clock()->now().seconds();
    r.scenario_time_s = have_sample_ ? last_sample_t_ : scenarioTime(now);
    r.sample_valid = have_sample_;
    r.sample_position.x = last_sample_p_[0];
    r.sample_position.y = last_sample_p_[1];
    r.sample_position.z = last_sample_p_[2];
    for (const auto & o : tracker_->obstacles()) {
      const auto centre = o.centreAt(r.scenario_time_s);
      geometry_msgs::msg::Point c;
      c.x = centre[0]; c.y = centre[1]; c.z = centre[2];
      r.names.push_back(o.name);
      r.positions.push_back(c);
      r.radii.push_back(o.radius);
      r.active.push_back(o.activeAt(r.scenario_time_s));
      r.types.push_back(o.type);
    }
    r.clearance_m = last_clearance_;
    r.nearest = nearest_;
    r.min_clearance_m = tracker_->minClearance();
    for (const auto & h : tracker_->hits()) {
      geometry_msgs::msg::Point p;
      p.x = h.x; p.y = h.y; p.z = h.z;
      r.hit_positions.push_back(p);
      r.hit_with.push_back(h.with);
      r.hit_times_s.push_back(h.t);
      r.hit_depths_m.push_back(h.depth);
    }
    for (const auto & st : tracker_->stations()) {
      geometry_msgs::msg::Point p;
      p.x = st[0]; p.y = st[1]; p.z = st[2];
      r.stations.push_back(p);
    }
    r.los_margin_m = last_los_;
    r.los_blocker = los_blocker_;
    r.los_station = los_station_;
    r.min_los_margin_m = tracker_->minLosMargin();
    for (const auto & b : tracker_->blackouts()) {
      geometry_msgs::msg::Point p;
      p.x = b.x; p.y = b.y; p.z = b.z;
      r.dark_positions.push_back(p);
      r.dark_station.push_back(b.station);
      r.dark_blocker.push_back(b.blocker);
      r.dark_times_s.push_back(b.t);
      r.dark_durations_s.push_back(b.duration);
      r.dark_depths_m.push_back(b.depth);
    }
    r.scenario = scenario_name_;
    r.scenario_duration_s = scenario_duration_s_;
    clearance_pub_->publish(r);
  }

  void reset()
  {
    t0_ = -1.0; elapsed_ = 0.0; err_ = 0.0; sq_sum_ = 0.0; n_samples_ = 0;
    max_err_ = 0.0; path_length_ = 0.0; energy_wh_ = 0.0; last_power_s_ = 0.0;
    tracker_->reset();
    nearest_.clear();
    last_clearance_ = std::numeric_limits<double>::infinity();
    last_los_ = std::numeric_limits<double>::infinity();
    have_sample_ = false;
    los_blocker_.clear();
    los_station_.clear();
    collided_ = false; collision_count_ = 0; ground_contacts_ = 0;
    have_last_pos_ = false; was_airborne_ = false;
    RCLCPP_INFO(get_logger(), "metrics reset");
  }

  // Simulated time going backwards means a new run. See dsim_time/sim_epoch.hpp.
  dsim_time::SimEpoch epoch_;

  std::vector<Obstacle> obstacles_;
  std::vector<std::array<double, 3>> stations_;
  std::unique_ptr<ClearanceTracker> tracker_;
  double scenario_start_s_ {0.0}, scenario_duration_s_ {0.0};
  std::string scenario_name_, nearest_;
  double vehicle_radius_, hover_power_w_, mass_, motor_constant_, publish_hz_;
  std::string csv_path_;
  std::ofstream csv_;

  dsim_msgs::msg::TrajectorySetpoint sp_;
  bool have_sp_ {false}, have_last_pos_ {false}, collided_ {false};
  bool was_airborne_ {false};
  double airborne_altitude_m_ {0.5}, warmup_s_ {6.0};
  size_t collision_count_ {0}, ground_contacts_ {0}, n_samples_ {0};
  double t0_ {-1.0}, elapsed_ {0.0}, err_ {0.0}, sq_sum_ {0.0}, max_err_ {0.0};
  double path_length_ {0.0}, energy_wh_ {0.0}, last_power_s_ {0.0};
  double lx_ {0}, ly_ {0}, lz_ {0};
  double last_clearance_ {std::numeric_limits<double>::infinity()};
  double last_los_ {std::numeric_limits<double>::infinity()};
  double last_sample_t_ {0.0};
  std::array<double, 3> last_sample_p_ {{0.0, 0.0, 0.0}};
  bool have_sample_ {false};
  std::string los_blocker_, los_station_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;
  rclcpp::Subscription<dsim_msgs::msg::TrajectorySetpoint>::SharedPtr sp_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_sub_;
  rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr cmd_sub_;
  rclcpp::Publisher<dsim_msgs::msg::FlightStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<dsim_msgs::msg::ClearanceReport>::SharedPtr clearance_pub_;
  rclcpp::Service<dsim_msgs::srv::ResetRun>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dsim_eval

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dsim_eval::EvalNode>());
  rclcpp::shutdown();
  return 0;
}
