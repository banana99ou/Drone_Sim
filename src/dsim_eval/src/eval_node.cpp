// Scoring node for planner trials.
//
// Measures against GROUND TRUTH (/drone/truth), never against the
// degraded estimate the controller sees. Scoring off the noisy estimate would
// hide exactly the errors the estimate causes, and a plan that flies into a
// wall would still score well if the estimate said otherwise.
//
// Metrics:
//   tracking error   | truth position vs the setpoint the controller tracked
//   collision        | contact sensor on the 50 cm guard envelope
//   clearance        | analytic distance to obstacles listed in the world config
//   path length      | integrated ground-truth travel
//   energy           | momentum-theory estimate anchored to the data sheet
//
// Clearance uses the world's declared obstacle list rather than the depth
// sensor, on purpose: this is the referee, and the referee must not share the
// planner's blind spots.

#include <cmath>
#include <fstream>
#include <memory>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <actuator_msgs/msg/actuators.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <dsim_msgs/msg/flight_status.hpp>
#include <dsim_msgs/msg/trajectory_setpoint.hpp>
#include <dsim_msgs/srv/reset_run.hpp>

namespace dsim_eval
{

struct Obstacle
{
  std::string type;                  // "box" | "cylinder" | "sphere"
  double x{0}, y{0}, z{0};
  double sx{1}, sy{1}, sz{1};        // box: full extents; cylinder: radius,_,height; sphere: radius

  /// Distance from a point to this obstacle's surface (0 inside).
  double distance(double px, double py, double pz) const
  {
    if (type == "sphere") {
      const double d = std::sqrt(
        (px - x) * (px - x) + (py - y) * (py - y) + (pz - z) * (pz - z));
      return std::max(0.0, d - sx);
    }
    if (type == "cylinder") {
      const double dr = std::max(0.0, std::sqrt((px - x) * (px - x) + (py - y) * (py - y)) - sx);
      const double dz = std::max(0.0, std::abs(pz - z) - sz / 2.0);
      return std::sqrt(dr * dr + dz * dz);
    }
    // box, axis aligned
    const double dx = std::max(0.0, std::abs(px - x) - sx / 2.0);
    const double dy = std::max(0.0, std::abs(py - y) - sy / 2.0);
    const double dz = std::max(0.0, std::abs(pz - z) - sz / 2.0);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }
};

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

    loadObstacles();

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
    RCLCPP_INFO(get_logger(), "eval up: %zu obstacles, vehicle radius %.2f m",
      obstacles_.size(), vehicle_radius_);
  }

private:
  void loadObstacles()
  {
    // Obstacles are declared as a flat list because ROS 2 parameters have no
    // native array-of-struct type:
    //   obstacles.names: ["pillar_a", ...]
    //   obstacles.pillar_a.type / .pose / .size
    const auto names = declare_parameter<std::vector<std::string>>(
      "obstacles.names", std::vector<std::string>{});
    for (const auto & n : names) {
      Obstacle o;
      o.type = declare_parameter("obstacles." + n + ".type", std::string("box"));
      const auto pose = declare_parameter<std::vector<double>>(
        "obstacles." + n + ".pose", {0.0, 0.0, 0.0});
      const auto size = declare_parameter<std::vector<double>>(
        "obstacles." + n + ".size", {1.0, 1.0, 1.0});
      o.x = pose.at(0); o.y = pose.at(1); o.z = pose.at(2);
      o.sx = size.at(0);
      o.sy = size.size() > 1 ? size.at(1) : size.at(0);
      o.sz = size.size() > 2 ? size.at(2) : size.at(0);
      obstacles_.push_back(o);
    }
  }

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
    // This is a MODEL, not a measurement — see docs/SYSID.md before quoting it.
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

    // clearance to the nearest declared obstacle, minus the guard envelope
    double clearance = std::numeric_limits<double>::infinity();
    for (const auto & o : obstacles_) {
      clearance = std::min(clearance, o.distance(px, py, pz) - vehicle_radius_);
    }
    if (std::isfinite(clearance)) {
      min_clearance_ = std::min(min_clearance_, clearance);
      last_clearance_ = clearance;
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
           << err_ << ',' << rmse() << ',' << last_clearance_ << ','
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
    s.min_obstacle_clearance_m = std::isfinite(min_clearance_)
      ? min_clearance_
      : std::numeric_limits<double>::quiet_NaN();
    s.path_length_m = path_length_;
    s.elapsed_s = elapsed_;
    s.energy_wh = energy_wh_;
    status_pub_->publish(s);
  }

  void reset()
  {
    t0_ = -1.0; elapsed_ = 0.0; err_ = 0.0; sq_sum_ = 0.0; n_samples_ = 0;
    max_err_ = 0.0; path_length_ = 0.0; energy_wh_ = 0.0; last_power_s_ = 0.0;
    min_clearance_ = std::numeric_limits<double>::infinity();
    collided_ = false; collision_count_ = 0; ground_contacts_ = 0;
    have_last_pos_ = false; was_airborne_ = false;
    RCLCPP_INFO(get_logger(), "metrics reset");
  }

  std::vector<Obstacle> obstacles_;
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
  double lx_ {0}, ly_ {0}, lz_ {0}, last_clearance_ {0.0};
  double min_clearance_ {std::numeric_limits<double>::infinity()};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;
  rclcpp::Subscription<dsim_msgs::msg::TrajectorySetpoint>::SharedPtr sp_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_sub_;
  rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr cmd_sub_;
  rclcpp::Publisher<dsim_msgs::msg::FlightStatus>::SharedPtr status_pub_;
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
