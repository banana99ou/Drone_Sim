// Controller node: turns a planner's trajectory into rotor speeds.
//
//   /drone/odom        (nav_msgs/Odometry)      state feedback
//   /drone/trajectory  (dsim_msgs/Trajectory) planner input  <-- YOUR PLANNER
//        |
//        v  SE(3) geometric control + mixer
//   /drone/command/motor_speed (actuator_msgs/Actuators) -> bridged to Gazebo
//   /drone/control_debug       (dsim_msgs/ControlDebug)   telemetry
//
// Also republishes the currently-tracked setpoint on /drone/setpoint so that
// tracking error is measured against what the controller actually chased, not
// against a re-interpolation done elsewhere.
//
// /drone/control_debug carries what every loop of the cascade demanded and what
// the rotors could deliver. It is what lets the viewer draw the control action
// instead of only the resulting motion -- those look identical from position
// alone, whether the stack is working hard or coasting.

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <actuator_msgs/msg/actuators.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <dsim_msgs/msg/control_debug.hpp>
#include <dsim_msgs/msg/trajectory.hpp>
#include <dsim_msgs/msg/trajectory_setpoint.hpp>

#include "dsim_control/mixer.hpp"
#include "dsim_control/se3_controller.hpp"
#include "dsim_control/trajectory_buffer.hpp"

using namespace std::chrono_literals;

namespace dsim_control
{

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : Node("dsim_controller")
  {
    // ---- parameters -------------------------------------------------------
    const double mass        = declare_parameter("vehicle.mass_kg", 1.5);
    const double gravity     = declare_parameter("vehicle.gravity_m_s2", 9.80665);
    const double arm_length  = declare_parameter("vehicle.arm_length_m", 0.18);
    const double motor_const = declare_parameter("vehicle.motor_constant", 3.29e-06);
    const double moment_const = declare_parameter("vehicle.moment_constant", 0.016);
    const double max_rot_vel = declare_parameter("vehicle.max_rot_velocity", 1500.0);
    const auto inertia = declare_parameter<std::vector<double>>(
      "vehicle.inertia", {0.012, 0.012, 0.023});

    Gains g;
    g.mass = mass;
    g.gravity = gravity;
    g.inertia = Eigen::Vector3d(inertia.at(0), inertia.at(1), inertia.at(2)).asDiagonal();
    auto vec3 = [this](const std::string & name, std::vector<double> def) {
        const auto v = declare_parameter<std::vector<double>>(name, def);
        return Eigen::Vector3d(v.at(0), v.at(1), v.at(2));
      };
    g.kp     = vec3("gains.kp",     {6.0, 6.0, 8.0});
    g.kv     = vec3("gains.kv",     {4.0, 4.0, 5.0});
    g.kR     = vec3("gains.kR",     {3.0, 3.0, 0.5});
    g.komega = vec3("gains.komega", {0.5, 0.5, 0.1});
    g.max_tilt_rad = declare_parameter("gains.max_tilt_rad", 0.7);
    g.min_thrust_n = declare_parameter("gains.min_thrust_n", 0.5);

    control_rate_hz_ = declare_parameter("control_rate_hz", 250.0);
    odom_timeout_s_  = declare_parameter("odom_timeout_s", 0.5);
    start_armed_     = declare_parameter("start_armed", true);
    armed_ = start_armed_;

    // nav_msgs/Odometry states twist in the child frame (REP-145). Gazebo's
    // OdometryPublisher follows that. Kept as a parameter because getting this
    // wrong is silent at hover and only shows up in banked flight — see
    // scripts/check_twist_frame.py, which measures it instead of assuming.
    twist_in_body_frame_ = declare_parameter("odom.twist_in_body_frame", true);

    controller_ = std::make_unique<SE3Controller>(g);
    mixer_ = std::make_unique<Mixer>(arm_length, moment_const, motor_const, max_rot_vel);

    runSelfTest(g);

    // ---- interfaces -------------------------------------------------------
    const auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/odom", qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {onOdom(*msg);});

    traj_sub_ = create_subscription<dsim_msgs::msg::Trajectory>(
      "/drone/trajectory", rclcpp::QoS(4),
      [this](dsim_msgs::msg::Trajectory::SharedPtr msg) {
        if (msg->points.empty()) {
          RCLCPP_WARN(get_logger(), "ignoring empty trajectory");
          return;
        }
        buffer_.set(*msg);
        RCLCPP_INFO(
          get_logger(), "accepted trajectory: %zu points, %.2f s",
          msg->points.size(), buffer_.duration());
      });

    cmd_pub_ = create_publisher<actuator_msgs::msg::Actuators>(
      "/drone/command/motor_speed", rclcpp::QoS(1));
    setpoint_pub_ = create_publisher<dsim_msgs::msg::TrajectorySetpoint>(
      "/drone/setpoint", rclcpp::QoS(10));
    // Best-effort, shallow queue: telemetry must never be able to back-pressure
    // the control loop, and a viewer that misses a frame at 250 Hz loses
    // nothing it could have drawn anyway.
    debug_pub_ = create_publisher<dsim_msgs::msg::ControlDebug>(
      "/drone/control_debug", rclcpp::SensorDataQoS());

    arm_srv_ = create_service<std_srvs::srv::SetBool>(
      "/drone/arm",
      [this](
        const std_srvs::srv::SetBool::Request::SharedPtr req,
        std_srvs::srv::SetBool::Response::SharedPtr res) {
        armed_ = req->data;
        hold_valid_ = false;
        res->success = true;
        res->message = armed_ ? "armed" : "disarmed";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });

    // Sim-clock timer, not a wall timer: if Gazebo runs slower than real time
    // a wall timer would tick faster than simulated time and silently change
    // the effective control rate.
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / control_rate_hz_),
      [this] {step();});

    RCLCPP_INFO(
      get_logger(),
      "controller up: %.0f Hz, hover thrust %.2f N, max thrust %.2f N (TWR %.2f)",
      control_rate_hz_, controller_->hoverThrust(), 4.0 * mixer_->maxThrustPerRotor(),
      4.0 * mixer_->maxThrustPerRotor() / controller_->hoverThrust());
  }

private:
  /// Startup checks that CAN fail. Each asserts something that must hold if
  /// the geometry and conventions are right, and is cheap to evaluate.
  void runSelfTest(const Gains & g)
  {
    // 1. The mixer inverse must actually invert, for a spread of wrenches.
    double worst = 0.0;
    const double hover = g.mass * g.gravity;
    const std::vector<Eigen::Vector4d> probes = {
      {hover, 0.0, 0.0, 0.0},
      {hover, 0.5, 0.0, 0.0},
      {hover, 0.0, 0.5, 0.0},
      {hover, 0.0, 0.0, 0.2},
      {1.4 * hover, -0.3, 0.4, -0.1},
    };
    for (const auto & w : probes) {
      worst = std::max(worst, mixer_->roundTripError(w));
    }
    if (worst > 1e-9) {
      RCLCPP_FATAL(get_logger(), "mixer round-trip error %.3e — allocation is wrong", worst);
      throw std::runtime_error("mixer self-test failed");
    }

    // 2. Hovering must be achievable well inside saturation. If hover already
    //    needs >90% of max rotor speed the vehicle cannot manoeuvre, and every
    //    tracking result afterwards would be actuator-limited rather than
    //    control-limited.
    const double hover_per_rotor = g.mass * g.gravity / 4.0;
    const double hover_omega = mixer_->thrustToSpeed(hover_per_rotor);
    const double util = hover_omega / mixer_->maxRotVelocity();
    if (util > 0.9) {
      RCLCPP_FATAL(
        get_logger(), "hover needs %.0f%% of max rotor speed — vehicle cannot fly", util * 100.0);
      throw std::runtime_error("thrust budget self-test failed");
    }

    // 3. A pure hover wrench must produce four equal rotor speeds. Catches
    //    sign or index errors in the allocation columns.
    const Eigen::Vector4d f = mixer_->rotorThrusts({g.mass * g.gravity, 0, 0, 0});
    if ((f.array() - f(0)).abs().maxCoeff() > 1e-9) {
      RCLCPP_FATAL(get_logger(), "hover produced asymmetric rotor thrusts — mixer columns are wrong");
      throw std::runtime_error("hover symmetry self-test failed");
    }

    RCLCPP_INFO(
      get_logger(),
      "self-test passed: mixer residual %.2e, condition %.2f, hover at %.0f%% rotor speed",
      worst, mixer_->conditionNumber(), util * 100.0);
  }

  void onOdom(const nav_msgs::msg::Odometry & msg)
  {
    State s;
    s.position = Eigen::Vector3d(
      msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    s.orientation = Eigen::Quaterniond(
      msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y, msg.pose.pose.orientation.z).normalized();

    const Eigen::Vector3d v_raw(
      msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
    s.velocity = twist_in_body_frame_ ? (s.orientation * v_raw) : v_raw;

    // Angular rate is body-frame in both conventions.
    s.angular_rate = Eigen::Vector3d(
      msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z);

    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = s;
    last_odom_s_ = nowSeconds();
    have_state_ = true;
  }

  double nowSeconds() const
  {
    return get_clock()->now().seconds();
  }

  void step()
  {
    State state;
    bool have_state;
    double last_odom;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state = state_;
      have_state = have_state_;
      last_odom = last_odom_s_;
    }

    const double now = nowSeconds();

    if (!armed_ || !have_state) {
      publishIdle();
      return;
    }
    if ((now - last_odom) > odom_timeout_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "no odometry for %.2f s — cutting motors", now - last_odom);
      publishIdle();
      return;
    }

    Reference ref;
    if (auto sampled = buffer_.sample(now)) {
      ref = *sampled;
      hold_valid_ = false;
    } else {
      // No trajectory yet: hold the position we were at when we first noticed.
      // Latching the hold point matters — holding "current position" every tick
      // would let the vehicle drift away with no error signal to correct it.
      if (!hold_valid_) {
        hold_position_ = state.position;
        hold_yaw_ = yawFromQuat(state.orientation);
        hold_valid_ = true;
      }
      ref.position = hold_position_;
      ref.yaw = hold_yaw_;
    }

    ControlDebug dbg;
    const Eigen::Vector4d wrench = controller_->compute(state, ref, &dbg);
    // Per-rotor thrusts first, then the speeds derived from them. Calling
    // rotorSpeeds() as well would compute the same clamp a second time and let
    // the commanded speeds and the reported thrusts drift apart under a future
    // edit; this way they are the same numbers by construction.
    const Eigen::Vector4d thrusts = mixer_->rotorThrusts(wrench);
    Eigen::Vector4d speeds;
    for (int i = 0; i < 4; ++i) {
      speeds(i) = mixer_->thrustToSpeed(thrusts(i));
    }

    if (dbg.tilt_clamped) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "tilt clamped at %.0f deg — trajectory is more aggressive than the vehicle allows",
        controller_->gains().max_tilt_rad * 180.0 / M_PI);
    }

    actuator_msgs::msg::Actuators cmd;
    cmd.header.stamp = get_clock()->now();
    cmd.velocity = {speeds(0), speeds(1), speeds(2), speeds(3)};
    cmd_pub_->publish(cmd);

    publishDebug(true, wrench, thrusts, dbg, state.velocity);

    dsim_msgs::msg::TrajectorySetpoint sp;
    sp.position.x = ref.position.x();
    sp.position.y = ref.position.y();
    sp.position.z = ref.position.z();
    sp.velocity.x = ref.velocity.x();
    sp.velocity.y = ref.velocity.y();
    sp.velocity.z = ref.velocity.z();
    sp.acceleration.x = ref.acceleration.x();
    sp.acceleration.y = ref.acceleration.y();
    sp.acceleration.z = ref.acceleration.z();
    sp.yaw = ref.yaw;
    sp.yaw_rate = ref.yaw_rate;
    setpoint_pub_->publish(sp);
  }

  void publishIdle()
  {
    actuator_msgs::msg::Actuators cmd;
    cmd.header.stamp = get_clock()->now();
    cmd.velocity = {0.0, 0.0, 0.0, 0.0};
    cmd_pub_->publish(cmd);
    // Publish the debug frame here too, with armed=false. A viewer that kept
    // showing the last commanded thrust after the motors were cut would be
    // drawing a vehicle that no longer exists.
    publishDebug(
      false, Eigen::Vector4d::Zero(), Eigen::Vector4d::Zero(), ControlDebug{},
      Eigen::Vector3d::Zero());
  }

  static geometry_msgs::msg::Vector3 toVec3(const Eigen::Vector3d & v)
  {
    geometry_msgs::msg::Vector3 m;
    m.x = v.x();
    m.y = v.y();
    m.z = v.z();
    return m;
  }

  /// Telemetry for one control step: the demand, what the rotors could actually
  /// deliver, and what each loop was reacting to.
  ///
  /// The realised wrench is recomputed here from the per-rotor thrusts rather
  /// than tracked alongside them, so the two cannot disagree. Consumers may
  /// assert sum(rotor_thrust_n) == realised_thrust_n; that identity is exactly
  /// what breaks if the allocation matrix ever stops inverting.
  void publishDebug(
    bool armed, const Eigen::Vector4d & wrench,
    const Eigen::Vector4d & thrusts, const ControlDebug & dbg,
    const Eigen::Vector3d & state_velocity)
  {
    const Eigen::Vector4d realised = mixer_->wrenchFromThrusts(thrusts);

    dsim_msgs::msg::ControlDebug m;
    m.header.stamp = get_clock()->now();
    m.header.frame_id = "drone/base_link";
    m.armed = armed;

    // Sent on the wire so no consumer needs its own copy of config/drone.yaml.
    // A viewer that hardcoded the mass would draw wrong forces the moment the
    // vehicle changed, and would do it silently.
    m.mass_kg = controller_->gains().mass;
    m.gravity_m_s2 = controller_->gains().gravity;
    m.max_rotor_thrust_n = mixer_->maxThrustPerRotor();

    m.thrust_n = wrench(0);
    m.torque_nm = toVec3(Eigen::Vector3d(wrench.tail<3>()));
    m.realised_thrust_n = realised(0);
    m.realised_torque_nm = toVec3(Eigen::Vector3d(realised.tail<3>()));
    // Saturation is defined by its observable consequence -- the rotors could
    // not deliver what was asked -- not by bookkeeping inside the clamp. That
    // keeps the flag true even for a saturation route added later.
    m.saturated = (realised - wrench).cwiseAbs().maxCoeff() > kWrenchTol;

    for (int i = 0; i < 4; ++i) {
      m.rotor_position[i] = toVec3(mixer_->rotorPosition(i));
      m.rotor_thrust_n[i] = thrusts(i);
      m.rotor_speed_rad_s[i] = mixer_->thrustToSpeed(thrusts(i));
    }

    m.velocity_world = toVec3(state_velocity);
    m.position_error = toVec3(dbg.position_error);
    m.velocity_error = toVec3(dbg.velocity_error);
    m.attitude_error = toVec3(dbg.attitude_error);
    m.body_rate_error = toVec3(dbg.body_rate_error);
    m.desired_force = toVec3(dbg.desired_force);
    m.commanded_tilt_rad = dbg.commanded_tilt_rad;
    m.tilt_clamped = dbg.tilt_clamped;

    debug_pub_->publish(m);
  }

  static double yawFromQuat(const Eigen::Quaterniond & q)
  {
    const Eigen::Vector3d x_body = q * Eigen::Vector3d::UnitX();
    return std::atan2(x_body.y(), x_body.x());
  }

  std::unique_ptr<SE3Controller> controller_;
  std::unique_ptr<Mixer> mixer_;
  TrajectoryBuffer buffer_;

  mutable std::mutex state_mutex_;
  State state_;
  bool have_state_ {false};
  double last_odom_s_ {0.0};

  bool armed_ {true};
  bool start_armed_ {true};
  bool twist_in_body_frame_ {true};
  double control_rate_hz_ {250.0};
  double odom_timeout_s_ {0.5};

  /// Below this, a demanded-vs-realised difference is numerical noise from the
  /// mixer inverse, not saturation. The self-test asserts the round-trip
  /// residual is under 1e-9, so this sits an order of magnitude above it.
  static constexpr double kWrenchTol = 1e-8;

  bool hold_valid_ {false};
  Eigen::Vector3d hold_position_ {Eigen::Vector3d::Zero()};
  double hold_yaw_ {0.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<dsim_msgs::msg::Trajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr cmd_pub_;
  rclcpp::Publisher<dsim_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<dsim_msgs::msg::ControlDebug>::SharedPtr debug_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dsim_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dsim_control::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
