// State estimator: the controller flies on THIS, not on ground truth.
//
//   /drone/imu           (sensor_msgs/Imu)            250 Hz  gyro + accel: drives every step
//   /drone/mag           (sensor_msgs/MagneticField)   50 Hz  yaw reference
//   /drone/tof           (sensor_msgs/Range)           30 Hz  altitude / climb rate
//   /drone/optical_flow  (dsim_msgs/OpticalFlow)       50 Hz  horizontal velocity
//   /drone/truth         (nav_msgs/Odometry)          250 Hz  POSITION ONLY -- see position_source
//        |
//        v  Mahony attitude filter + complementary velocity filter
//   /drone/state_est       (nav_msgs/Odometry)       on every IMU sample
//   /drone/estimator_debug (dsim_msgs/EstimatorDebug) on every IMU sample
//
// What is estimated from what:
//   attitude    gyro integration, roll/pitch pulled to the accelerometer's
//               gravity direction, yaw pulled to the magnetometer's
//               tilt-compensated heading           (attitude_filter.hpp)
//   body rate   the raw gyro sample, passed through; the controller already
//               takes its rates from the gyro and gates them itself
//   velocity    accelerometer rotated with the estimated attitude and
//               integrated, pinned horizontally by the flow and vertically
//               by the rangefinder                  (velocity_filter.hpp)
//   position    GROUND TRUTH, copied. There is no position sensor in this
//               simulator yet; see the position_source parameter for where
//               one plugs in.
//
// The output is shaped exactly like /drone/truth -- nav_msgs/Odometry with
// the twist in the body frame per REP-145 -- so the controller consumes it
// through a topic remap and nothing in dsim_control changes. That is the
// point: the controller cannot tell which it is flying on, and the launch
// argument state:=est|truth decides.
//
// Nothing is published until the attitude has been initialised, an IMU
// sample has been integrated and a truth position is in hand. A controller
// that received an Odometry with a fabricated attitude would act on it.

#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <dsim_time/sim_epoch.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <dsim_msgs/msg/estimator_debug.hpp>
#include <dsim_msgs/msg/optical_flow.hpp>

#include "dsim_estimation/attitude_filter.hpp"
#include "dsim_estimation/velocity_filter.hpp"

namespace dsim_estimation
{

class EstimatorNode : public rclcpp::Node
{
public:
  EstimatorNode()
  : Node("dsim_estimator")
  {
    // ---- the position seam ---------------------------------------------
    // Position is the one state this node does not estimate: the simulator
    // has no position sensor, and inventing one (GPS, a SLAM or VIO
    // emulator with its own drift model) is a separate piece of work. This
    // parameter is where that work plugs in. Its only accepted value today
    // is "truth"; anything else is refused at startup rather than silently
    // falling back, because a controller flying on a position source it did
    // not ask for is the kind of thing that only shows up on the wrong day.
    position_source_ = declare_parameter<std::string>("position_source", "truth");
    if (position_source_ != "truth") {
      RCLCPP_FATAL(
        get_logger(),
        "position_source '%s' is not implemented; the only source today is "
        "'truth'. This is the seam a SLAM/VIO emulator plugs into.",
        position_source_.c_str());
      throw std::runtime_error("dsim_estimation: unsupported position_source");
    }

    // ---- gains ------------------------------------------------------------
    // Declared WITHOUT defaults: these come from
    // src/dsim_estimation/config/estimator.yaml through the launch file, and
    // gravity from config/drone.yaml. A default here would be a second copy
    // that nobody tests -- the same rule the sensors node follows.
    AttitudeConfig att;
    att.gravity = require("gravity_m_s2");
    att.kp_accel = require("attitude.kp_accel");
    att.ki_accel = require("attitude.ki_accel");
    att.kp_mag = require("attitude.kp_mag");
    att.ki_mag = require("attitude.ki_mag");
    att.accel_gate_m_s2 = require("attitude.accel_gate_m_s2");
    att.max_dt_s = require("attitude.max_dt_s");
    att.mag_hold_s = require("attitude.mag_hold_s");

    VelocityConfig vel;
    vel.gravity = att.gravity;
    vel.flow_tau_s = require("velocity.flow_tau_s");
    vel.tof_omega_rad_s = require("velocity.tof_omega_rad_s");
    vel.tof_zeta = require("velocity.tof_zeta");
    vel.max_dt_s = require("velocity.max_dt_s");

    mag_timeout_s_ = require("mag_timeout_s");
    truth_timeout_s_ = require("truth_timeout_s");

    // The world field is the one thing the magnetometer path cannot do
    // without, and it belongs to the SENSOR's config (config/drone.yaml,
    // generated). If the launch found no magnetometer block it passes
    // nothing, and yaw is then gyro-only -- stated at startup, once, and
    // in every debug message, rather than fused against a guessed field.
    const auto field = declare_parameter<std::vector<double>>(
      "mag.field_world_t", std::vector<double>{});
    mag_configured_ = field.size() == 3;
    if (mag_configured_) {
      att.field_world = Eigen::Vector3d(field[0], field[1], field[2]);
    } else if (!field.empty()) {
      RCLCPP_FATAL(get_logger(), "mag.field_world_t must have exactly 3 elements, got %zu", field.size());
      throw std::runtime_error("dsim_estimation: malformed mag.field_world_t");
    }

    attitude_ = std::make_unique<AttitudeFilter>(att);
    velocity_ = std::make_unique<VelocityFilter>(vel);

    // ---- interfaces -------------------------------------------------------
    const auto qos = rclcpp::SensorDataQoS();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/drone/imu", qos, [this](sensor_msgs::msg::Imu::SharedPtr m) {onImu(*m);});
    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
      "/drone/mag", qos, [this](sensor_msgs::msg::MagneticField::SharedPtr m) {onMag(*m);});
    tof_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/drone/tof", qos, [this](sensor_msgs::msg::Range::SharedPtr m) {onTof(*m);});
    flow_sub_ = create_subscription<dsim_msgs::msg::OpticalFlow>(
      "/drone/optical_flow", qos, [this](dsim_msgs::msg::OpticalFlow::SharedPtr m) {onFlow(*m);});
    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/truth", qos, [this](nav_msgs::msg::Odometry::SharedPtr m) {onTruth(*m);});

    // Same QoS the controller subscribes to /drone/truth with, so the remap
    // is a drop-in. Best-effort is right for a 250 Hz state stream: a
    // sample that arrives late is worse than one that is dropped.
    state_pub_ = create_publisher<nav_msgs::msg::Odometry>("/drone/state_est", qos);
    debug_pub_ = create_publisher<dsim_msgs::msg::EstimatorDebug>("/drone/estimator_debug", qos);

    RCLCPP_INFO(
      get_logger(),
      "estimator up: attitude kp_accel %.2f ki_accel %.3f kp_mag %.2f ki_mag %.3f, "
      "velocity flow_tau %.2f s tof omega %.1f rad/s zeta %.2f, mag %s, position from %s",
      att.kp_accel, att.ki_accel, att.kp_mag, att.ki_mag, vel.flow_tau_s,
      vel.tof_omega_rad_s, vel.tof_zeta,
      mag_configured_ ? "configured" : "NOT configured (yaw will be gyro-only)",
      position_source_.c_str());
  }

private:
  /// A parameter with no default: absent means misconfigured, and the node
  /// refuses to start rather than substituting a number of its own.
  ///
  /// rclcpp throws at the declaration itself when a statically typed
  /// parameter has no value, so the check has to be a catch, not a
  /// get_parameter() afterwards -- that branch would never run.
  double require(const std::string & name)
  {
    try {
      return declare_parameter<double>(name);
    } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &) {
      RCLCPP_FATAL(
        get_logger(),
        "parameter '%s' was not supplied. It comes from estimator.yaml or "
        "config/drone.yaml via the launch file; run this node with "
        "'ros2 launch dsim_bringup sim.launch.py' rather than directly.",
        name.c_str());
      throw std::runtime_error("dsim_estimation: missing parameter " + name);
    }
  }

  static double stampOf(const std_msgs::msg::Header & h)
  {
    return h.stamp.sec + 1e-9 * h.stamp.nanosec;
  }

  static Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3 & v)
  {
    return Eigen::Vector3d(v.x, v.y, v.z);
  }

  static geometry_msgs::msg::Vector3 toMsg(const Eigen::Vector3d & v)
  {
    geometry_msgs::msg::Vector3 m;
    m.x = v.x();
    m.y = v.y();
    m.z = v.z();
    return m;
  }

  /// Simulated time went backwards: the vehicle has been teleported home and
  /// the clock restarted. Everything integrated belongs to a run that no
  /// longer exists. See dsim_time/sim_epoch.hpp for why this is the signal.
  void noteEpoch(double now)
  {
    if (!epoch_.restarted(now)) {return;}
    RCLCPP_INFO(get_logger(), "simulated time went backwards — estimator starting a new run");
    attitude_->reset();
    velocity_->reset();
    have_imu_ = false;
    have_truth_ = false;
    have_mag_ = false;
    mag_heading_ = std::numeric_limits<double>::quiet_NaN();
    last_tof_stamp_ = 0.0;
    have_tof_ = false;
    tof_rate_raw_ = 0.0;
    warned_no_mag_ = false;
  }

  /// The driver: one step per IMU sample.
  void onImu(const sensor_msgs::msg::Imu & m)
  {
    const double now = get_clock()->now().seconds();
    // The sensor's own stamp is the integration clock: it is exact at the
    // IMU rate where the callback time jitters with scheduling. Falls back
    // to the node clock for a message with no stamp.
    double stamp = stampOf(m.header);
    if (stamp <= 0.0) {stamp = now;}
    const Eigen::Vector3d gyro = toEigen(m.angular_velocity);
    const Eigen::Vector3d accel = toEigen(m.linear_acceleration);

    std::lock_guard<std::mutex> lock(mutex_);
    noteEpoch(now);

    if (!attitude_->initialised()) {
      if (!have_imu_) {first_imu_s_ = now;}
      have_imu_ = true;
      // Give the magnetometer a moment to arrive before initialising
      // without it: a yaw that starts from the mag is right from the first
      // sample, one that starts at zero is only right by coincidence.
      const bool waiting_for_mag = mag_configured_ && !have_mag_ &&
        (now - first_imu_s_) < mag_timeout_s_;
      if (waiting_for_mag) {
        publishDebug(m, gyro);
        return;
      }
      if (!have_mag_ && !warned_no_mag_) {
        warned_no_mag_ = true;
        if (mag_configured_) {
          RCLCPP_WARN(
            get_logger(),
            "no /drone/mag within %.1f s — initialising yaw at 0 and integrating the "
            "gyro; yaw will drift at the gyro bias until a mag sample arrives",
            mag_timeout_s_);
        } else {
          RCLCPP_WARN(
            get_logger(),
            "no magnetometer field configured (config/drone.yaml has no "
            "drone.sensors.magnetometer block) — yaw is gyro-only and will drift");
        }
      }
      if (!attitude_->initialise(accel, have_mag_ ? &mag_body_ : nullptr)) {
        // Not a gravity reading (a bounce, free fall, a crash): try the
        // next one rather than start from a fabricated attitude.
        publishDebug(m, gyro);
        return;
      }
      const Eigen::Vector3d rpy = attitude_->rollPitchYaw();
      RCLCPP_INFO(
        get_logger(), "attitude initialised: roll %.1f pitch %.1f yaw %.1f deg (%s)",
        rpy.x() * 180.0 / M_PI, rpy.y() * 180.0 / M_PI, rpy.z() * 180.0 / M_PI,
        have_mag_ ? "yaw from mag" : "yaw assumed 0");
      last_imu_stamp_ = stamp;
      publishDebug(m, gyro);
      return;
    }

    const double dt = stamp - last_imu_stamp_;
    last_imu_stamp_ = stamp;
    // Order matters and is deliberate: the accelerometer error is computed
    // against the attitude BEFORE this step's gyro is integrated, so the
    // correction and the prediction see the same state; the velocity then
    // integrates with the attitude AFTER, which is the one current at the
    // end of the interval. Both filters refuse a non-positive or stalled dt
    // on their own.
    attitude_->correctAccel(accel);
    attitude_->predict(gyro, dt);
    velocity_->predict(attitude_->rotation(), accel, dt);

    if (have_truth_ && (now - last_truth_s_) <= truth_timeout_s_) {
      publishState(m, gyro);
    } else if (have_truth_) {
      // A dead position source must not be hidden behind a live attitude.
      // Going quiet lets the controller's own odom timeout cut the motors,
      // which is what it does for a dead /drone/truth today.
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_, 2000,
        "no truth position for %.2f s (limit %.2f) — not publishing a state",
        now - last_truth_s_, truth_timeout_s_);
    }
    publishDebug(m, gyro);
  }

  void onMag(const sensor_msgs::msg::MagneticField & m)
  {
    const Eigen::Vector3d b = toEigen(m.magnetic_field);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!b.allFinite()) {return;}
    mag_body_ = b;
    have_mag_ = true;
    if (!mag_configured_) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "/drone/mag is publishing but no world field is configured — ignoring it; "
        "a heading needs a field to compare against");
      return;
    }
    if (attitude_->initialised()) {
      attitude_->correctMag(b);
      const double err = attitude_->magHeadingError(b);
      mag_heading_ = std::isfinite(err) ? attitude_->rollPitchYaw().z() + err : err;
    }
  }

  void onTof(const sensor_msgs::msg::Range & m)
  {
    // float -> double keeps +/-inf, which is the point of REP 117's markers.
    const double range = static_cast<double>(m.range);
    const double stamp = stampOf(m.header);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!attitude_->initialised()) {return;}
    const double dt = have_tof_ ? (stamp - last_tof_stamp_) : 0.0;
    const double prev_alt = velocity_->lastTofAltitude();
    if (velocity_->correctTof(attitude_->rotation(), range, dt)) {
      // The differenced rate the filter does NOT use, for the debug stream:
      // seeing its noise is the argument for the second-order form.
      tof_rate_raw_ = (have_tof_ && dt > 0.0) ?
        (velocity_->lastTofAltitude() - prev_alt) / dt : 0.0;
      last_tof_stamp_ = stamp;
      have_tof_ = true;
    }
  }

  void onFlow(const dsim_msgs::msg::OpticalFlow & m)
  {
    FlowReading f;
    f.integration_time_s = m.integration_time_s;
    f.integrated_x = m.integrated_x;
    f.integrated_y = m.integrated_y;
    f.integrated_xgyro = m.integrated_xgyro;
    f.integrated_ygyro = m.integrated_ygyro;
    f.ground_distance_m = m.ground_distance_m;
    f.quality = m.quality;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!attitude_->initialised()) {return;}
    velocity_->correctFlow(attitude_->rotation(), f);
  }

  /// Position only. The attitude and twist in this message are never read:
  /// that is the whole discipline of this node, and it is enforced by never
  /// copying them out of the message.
  void onTruth(const nav_msgs::msg::Odometry & m)
  {
    const Eigen::Vector3d p(m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z);
    std::lock_guard<std::mutex> lock(mutex_);
    truth_position_ = p;
    last_truth_s_ = get_clock()->now().seconds();
    have_truth_ = true;
  }

  /// Caller holds mutex_.
  void publishState(const sensor_msgs::msg::Imu & imu, const Eigen::Vector3d & gyro)
  {
    const Eigen::Quaterniond & q = attitude_->quaternion();
    // REP-145: an Odometry's twist is in the child frame. The controller's
    // odom.twist_in_body_frame is true and rotates it back with the
    // orientation in the same message, so this must be R^T v -- the same
    // convention Gazebo's OdometryPublisher uses for /drone/truth.
    const Eigen::Vector3d v_body = q.conjugate() * velocity_->velocityWorld();

    nav_msgs::msg::Odometry s;
    s.header.stamp = imu.header.stamp;
    s.header.frame_id = "world";
    s.child_frame_id = "base_link";
    s.pose.pose.position.x = truth_position_.x();
    s.pose.pose.position.y = truth_position_.y();
    s.pose.pose.position.z = truth_position_.z();
    s.pose.pose.orientation.w = q.w();
    s.pose.pose.orientation.x = q.x();
    s.pose.pose.orientation.y = q.y();
    s.pose.pose.orientation.z = q.z();
    s.twist.twist.linear = toMsg(v_body);
    // The raw gyro sample. The controller does not read this field -- it
    // takes rates from /drone/imu and gates them -- but a consumer that does
    // gets a measurement, not a differentiated pose.
    s.twist.twist.angular = toMsg(gyro);
    state_pub_->publish(s);
  }

  /// Caller holds mutex_.
  void publishDebug(const sensor_msgs::msg::Imu & imu, const Eigen::Vector3d & /*gyro*/)
  {
    dsim_msgs::msg::EstimatorDebug d;
    d.header.stamp = imu.header.stamp;
    d.header.frame_id = "world";
    d.initialised = attitude_->initialised();

    const Eigen::Quaterniond & q = attitude_->quaternion();
    d.attitude.w = q.w();
    d.attitude.x = q.x();
    d.attitude.y = q.y();
    d.attitude.z = q.z();
    d.gyro_bias_rad_s = toMsg(attitude_->gyroBias());
    d.yaw_est_rad = attitude_->rollPitchYaw().z();
    d.mag_heading_rad = mag_heading_;
    d.mag_seen = have_mag_;
    d.mag_configured = mag_configured_;

    d.velocity_world = toMsg(velocity_->velocityWorld());
    d.flow_velocity_world = toMsg(velocity_->lastFlowVelocityWorld());
    d.tof_altitude_m = velocity_->lastTofAltitude();
    d.altitude_est_m = velocity_->altitudeValid() ?
      velocity_->altitude() : std::numeric_limits<double>::quiet_NaN();
    d.altitude_rate_est_m_s = velocity_->velocityWorld().z();
    d.tof_altitude_rate_raw_m_s = tof_rate_raw_;

    d.accel_correction_applied = attitude_->lastStep().accel_applied;
    d.mag_correction_applied = attitude_->lastStep().mag_applied;
    d.flow_correction_applied = velocity_->lastStep().flow_applied;
    d.tof_correction_applied = velocity_->lastStep().tof_applied;
    d.flow_rejected = velocity_->flowRejected();
    d.tof_rejected = velocity_->tofRejected();
    d.epochs = static_cast<std::uint32_t>(epoch_.epochs());
    debug_pub_->publish(d);
  }

  std::unique_ptr<AttitudeFilter> attitude_;
  std::unique_ptr<VelocityFilter> velocity_;
  dsim_time::SimEpoch epoch_;
  // Throttling on the simulated clock goes mute after a reset until the
  // clock passes its old value; warnings about right now use a clock that
  // only moves forwards.
  rclcpp::Clock steady_ {RCL_STEADY_TIME};

  std::string position_source_;
  bool mag_configured_ {false};
  double mag_timeout_s_ {1.0};
  double truth_timeout_s_ {0.5};

  mutable std::mutex mutex_;
  bool have_imu_ {false};
  double first_imu_s_ {0.0};
  double last_imu_stamp_ {0.0};
  bool have_mag_ {false};
  Eigen::Vector3d mag_body_ {Eigen::Vector3d::Zero()};
  double mag_heading_ {std::numeric_limits<double>::quiet_NaN()};
  bool warned_no_mag_ {false};
  bool have_tof_ {false};
  double last_tof_stamp_ {0.0};
  double tof_rate_raw_ {0.0};
  bool have_truth_ {false};
  double last_truth_s_ {0.0};
  Eigen::Vector3d truth_position_ {Eigen::Vector3d::Zero()};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr tof_sub_;
  rclcpp::Subscription<dsim_msgs::msg::OpticalFlow>::SharedPtr flow_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
  rclcpp::Publisher<dsim_msgs::msg::EstimatorDebug>::SharedPtr debug_pub_;
};

}  // namespace dsim_estimation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dsim_estimation::EstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
