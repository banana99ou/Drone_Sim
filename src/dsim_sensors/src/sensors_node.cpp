// Downward rangefinder, optical-flow and magnetometer sensors, derived from
// ground truth.
//
//   /drone/truth   (nav_msgs/Odometry)   pose + body velocity
//   /drone/imu     (sensor_msgs/Imu)     gyro, for the flow's rotation terms
//        |
//        v
//   /drone/tof            (sensor_msgs/Range)          ~30 Hz
//   /drone/optical_flow   (dsim_msgs/OpticalFlow)      ~50 Hz
//   /drone/mag            (sensor_msgs/MagneticField)  ~50 Hz
//
// These pretend to BE hardware, which is why they live apart from dsim_eval:
// the referee measures the vehicle and must never lie, while these are
// entitled to be wrong within their stated error model. Mixing the two in one
// node would make it far too easy to score a run against a noisy reading.
//
// The flow sensor is generated from TRUE motion over TRUE height, and reports
// the MEASURED range and the MEASURED gyro. That split is the model -- see the
// long note in optical_flow.hpp. Generating the flow from the measured values
// (as this node originally did) makes the consumer's reconstruction cancel
// them exactly, so neither the range error nor the gyro error exists.
//
// The true body rate is differentiated from consecutive truth attitudes rather
// than taken from /drone/truth's twist.angular, because that field is produced
// by Gazebo's wheeled-robot OdometryPublisher and jumps to 626 rad/s once per
// revolution when the quaternion changes sheet. See rates.hpp.
//
// The magnetometer is the same truth attitude seen through one transpose: the
// world field rotated into the body, plus a hard-iron offset drawn once per
// run, plus noise. It is not a Gazebo sensor, for the reasons in
// magnetometer.hpp -- and because it shares R_ with the rangefinder and the
// flow, the three sensors can never disagree about the attitude they were
// sampled at.

#include <algorithm>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <dsim_time/sim_epoch.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <dsim_msgs/msg/optical_flow.hpp>

#include "dsim_sensors/magnetometer.hpp"
#include "dsim_sensors/optical_flow.hpp"
#include "dsim_sensors/rates.hpp"
#include "dsim_sensors/tof.hpp"

namespace dsim_sensors
{

class SensorsNode : public rclcpp::Node
{
public:
  SensorsNode()
  : Node("dsim_sensors")
  {
    // Declared WITHOUT defaults on purpose. Every one of these numbers is
    // owned by PARAMS in scripts/gen_assets.py, which writes config/drone.yaml,
    // which the launch file passes here. A default would be a second copy of a
    // generated value -- and the copy that drifts is always the one nobody
    // tests. Without one, running this node outside the launch fails loudly
    // instead of flying on stale numbers.
    tof_.min_range_m = require("tof.min_range_m");
    tof_.max_range_m = require("tof.max_range_m");
    tof_.noise_m = require("tof.noise_m");
    tof_.noise_frac = require("tof.noise_frac");
    tof_.resolution_m = require("tof.resolution_m");
    const double tof_rate = require("tof.rate_hz");
    fov_rad_ = require("tof.fov_rad");

    flow_.noise_rad_s = require("optical_flow.noise_rad_s");
    flow_.min_height_m = require("optical_flow.min_height_m");
    flow_.max_height_m = require("optical_flow.max_height_m");
    flow_.max_tilt_rad = require("optical_flow.max_tilt_rad");
    const double flow_rate = require("optical_flow.rate_hz");

    mag_.noise_t = require("magnetometer.noise_t");
    mag_.hard_iron_t = require("magnetometer.hard_iron_t");
    mag_.field_world_t = requireVector3("magnetometer.field_world_t");
    const double mag_rate = require("magnetometer.rate_hz");
    truth_timeout_s_ = 3.0 / std::min({tof_rate, flow_rate, mag_rate});

    // Fixed by default so noise is not a fresh surprise every run. Worth
    // being precise about what this does and does not promise: ONE generator
    // feeds all three timers, at 30, 50 and 50 Hz, drawing one, two and three
    // variates respectively, and the flow's dt comes from clock deltas. So
    // the interleaving -- and therefore which draw lands in which reading --
    // depends on callback order and /clock delivery. A dropped tick shifts
    // every sequence permanently.
    //
    // What you get is a fixed noise DISTRIBUTION and a repeatable starting
    // point, not bit-identical sensor streams between runs. Seeding per sensor
    // and drawing on a sample counter rather than on call order would give the
    // stronger property; it is not needed for comparing planners on aggregate
    // metrics, which is what this simulator is for.
    const int seed = declare_parameter("noise_seed", 1);
    rng_.seed(seed != 0 ? static_cast<std::uint32_t>(seed) : std::random_device{}());

    // The hard-iron direction is the first thing drawn after seeding, and it
    // is drawn whether or not a hard iron is configured, so that switching
    // one on does not shift every other sensor's noise sequence by three
    // draws. With hard_iron_t = 0 the result is exactly zero regardless.
    hard_iron_ = magHardIron(
      mag_.hard_iron_t, Eigen::Vector3d(gauss_(rng_), gauss_(rng_), gauss_(rng_)));

    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/truth", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {onTruth(*m);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/drone/imu", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        gyro_ = Eigen::Vector3d(
          m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);
      });

    range_pub_ = create_publisher<sensor_msgs::msg::Range>(
      "/drone/tof", rclcpp::SensorDataQoS());
    flow_pub_ = create_publisher<dsim_msgs::msg::OpticalFlow>(
      "/drone/optical_flow", rclcpp::SensorDataQoS());
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(
      "/drone/mag", rclcpp::SensorDataQoS());

    // Sim-clock timers: a wall timer would change the sensor rate whenever
    // the simulation ran slower than real time, and the playback-speed slider
    // in the viewer does exactly that on purpose.
    tof_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / tof_rate),
      [this] {publishTof();});
    flow_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / flow_rate),
      [this] {publishFlow();});
    mag_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / mag_rate),
      [this] {publishMag();});

    RCLCPP_INFO(
      get_logger(),
      "sensors up: tof %.0f Hz (%.2f-%.2f m, sigma %.3f + %.1f%%), "
      "flow %.0f Hz (%.2f-%.2f m band), "
      "mag %.0f Hz (sigma %.2f uT, hard iron %.2f uT, field [%.1f %.1f %.1f] uT), "
      "seed %d, truth timeout %.2f s",
      tof_rate, tof_.min_range_m, tof_.max_range_m, tof_.noise_m,
      100.0 * tof_.noise_frac, flow_rate, flow_.min_height_m,
      flow_.max_height_m, mag_rate, 1e6 * mag_.noise_t, 1e6 * mag_.hard_iron_t,
      1e6 * mag_.field_world_t.x(), 1e6 * mag_.field_world_t.y(),
      1e6 * mag_.field_world_t.z(), seed, truth_timeout_s_);
  }

private:
  /// A parameter with no default: absent means misconfigured, and the node
  /// refuses to start rather than substituting a number of its own.
  double require(const std::string & name)
  {
    declare_parameter<double>(name);
    double value = 0.0;
    if (!get_parameter(name, value)) {
      RCLCPP_FATAL(
        get_logger(),
        "parameter '%s' was not supplied. These come from config/drone.yaml "
        "via the launch file; run this node with "
        "'ros2 launch dsim_bringup sim.launch.py' rather than directly.",
        name.c_str());
      throw std::runtime_error("dsim_sensors: missing parameter " + name);
    }
    return value;
  }

  /// The three-component form of require(): a double array of exactly three,
  /// with the same refusal to start on anything else. The world field is one
  /// vector in config/drone.yaml and stays one vector here, so nobody has to
  /// remember which of three scalars is the vertical.
  Eigen::Vector3d requireVector3(const std::string & name)
  {
    declare_parameter<std::vector<double>>(name);
    std::vector<double> v;
    if (!get_parameter(name, v) || v.size() != 3) {
      RCLCPP_FATAL(
        get_logger(),
        "parameter '%s' must be a list of exactly three doubles (got %zu). "
        "It comes from config/drone.yaml via the launch file; run this node "
        "with 'ros2 launch dsim_bringup sim.launch.py' rather than directly.",
        name.c_str(), v.size());
      throw std::runtime_error("dsim_sensors: bad parameter " + name);
    }
    return Eigen::Vector3d(v[0], v[1], v[2]);
  }

  void onTruth(const nav_msgs::msg::Odometry & m)
  {
    const Eigen::Quaterniond q = Eigen::Quaterniond(
      m.pose.pose.orientation.w, m.pose.pose.orientation.x,
      m.pose.pose.orientation.y, m.pose.pose.orientation.z).normalized();
    const double stamp = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec;

    std::lock_guard<std::mutex> lock(mutex_);
    // True body rate, differentiated across the double cover. This is the
    // rotation that actually sweeps the image; the gyro's noisy version is
    // what gets REPORTED, and the difference is the phantom velocity a
    // consumer sees during rotation.
    if (have_truth_ && stamp > truth_stamp_) {
      omega_true_ = bodyRateFromQuaternions(q_, q, stamp - truth_stamp_);
    }
    q_ = q;
    R_ = q.toRotationMatrix();
    altitude_ = m.pose.pose.position.z;
    // Odometry states twist in the child frame (REP-145), which is what an
    // onboard sensor sees, so this needs no conversion -- unlike the
    // controller, which wants it in the world frame.
    v_body_ = Eigen::Vector3d(
      m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z);
    truth_stamp_ = stamp;
    have_truth_ = true;
  }

  /// True once ground truth has arrived AND is recent enough to sense from.
  ///
  /// The case this guards is a truth source that has died while the world
  /// keeps running -- a crashed bridge, a stalled publisher. Both timers would
  /// otherwise go on publishing confident readings from a frozen state
  /// forever. Real hardware in that position stops reporting; a simulated
  /// sensor that keeps insisting is worse than one that goes quiet, because
  /// nothing downstream can tell.
  ///
  /// A PAUSED world is not that case and never reaches here: these timers run
  /// on simulated time, so pausing stops the sensors and the clock together.
  /// The comment here used to claim otherwise, describing a scenario the code
  /// cannot be in.
  bool truthUsable(double now) const
  {
    return have_truth_ && (now - truth_stamp_) <= truth_timeout_s_;
  }

  /// Drop the state that only means something within one run.
  ///
  /// Called by both timers because either may be the first to notice; the
  /// SimEpoch reports a restart exactly once, and the work is the same
  /// whichever caller sees it.
  void noteEpoch(double now)
  {
    if (!epoch_.restarted(now)) {return;}
    RCLCPP_INFO(get_logger(), "simulated time went backwards — sensors starting a new run");
    // Negative integration time. flowSample() would reject it, but only after
    // reporting a reading with quality 0, and the gyro integrals in that
    // message would be a fabricated zero rather than a measurement.
    last_flow_s_ = 0.0;
    // Truth from the old run is not fresh truth, however recent its stamp
    // looks against a clock that has just jumped backwards.
    have_truth_ = false;
  }

  void publishTof()
  {
    const double now = get_clock()->now().seconds();
    double measured = kTooFar;
    {
      // One lock for the whole step, covering the RNG too. The default
      // executor is single-threaded so nothing races today, but partial
      // locking would advertise a safety property the code does not have --
      // and a switch to a MultiThreadedExecutor is a one-line change someone
      // might make to fix timer jitter, which would then be a data race on
      // std::mt19937 and would destroy the reproducibility the seed exists for.
      std::lock_guard<std::mutex> lock(mutex_);
      noteEpoch(now);
      if (!truthUsable(now)) {
        warnStale(now);
        return;
      }
      true_range_ = tofTrueRange(altitude_, R_);
      measured = tofMeasure(true_range_, tof_, gauss_(rng_));
      last_range_ = measured;
    }

    sensor_msgs::msg::Range msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "drone/tof";
    msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    msg.field_of_view = static_cast<float>(fov_rad_);
    msg.min_range = static_cast<float>(tof_.min_range_m);
    msg.max_range = static_cast<float>(tof_.max_range_m);
    msg.range = static_cast<float>(measured);
    range_pub_->publish(msg);
  }

  void publishFlow()
  {
    const double now = get_clock()->now().seconds();
    FlowInputs in;
    FlowSample s;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      noteEpoch(now);
      if (!truthUsable(now)) {
        warnStale(now);
        return;
      }
      in.v_body = v_body_;
      in.omega_true = omega_true_;
      in.omega_measured = gyro_;
      in.true_range_m = true_range_;
      in.measured_range_m = last_range_;
      in.cos_tilt = R_(2, 2);
      in.dt = last_flow_s_ > 0.0 ? (now - last_flow_s_) : 0.0;
      last_flow_s_ = now;
      s = flowSample(in, flow_, gauss_(rng_), gauss_(rng_));
    }

    dsim_msgs::msg::OpticalFlow msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "drone/flow";
    msg.integration_time_s = in.dt;
    msg.integrated_x = s.integrated_x;
    msg.integrated_y = s.integrated_y;
    msg.integrated_xgyro = s.integrated_xgyro;
    msg.integrated_ygyro = s.integrated_ygyro;
    // NaN rather than a number when the rangefinder had no return: a finite
    // stand-in would be silently scaled into a velocity by any consumer.
    msg.ground_distance_m = std::isfinite(s.ground_distance_m)
      ? s.ground_distance_m
      : std::numeric_limits<double>::quiet_NaN();
    msg.quality = s.quality;
    flow_pub_->publish(msg);
  }

  void publishMag()
  {
    const double now = get_clock()->now().seconds();
    Eigen::Vector3d b;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      noteEpoch(now);
      if (!truthUsable(now)) {
        warnStale(now);
        return;
      }
      b = magMeasure(
        magTrueField(R_, mag_.field_world_t), mag_, hard_iron_,
        Eigen::Vector3d(gauss_(rng_), gauss_(rng_), gauss_(rng_)));
    }

    sensor_msgs::msg::MagneticField msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "base_link";
    msg.magnetic_field.x = b.x();
    msg.magnetic_field.y = b.y();
    msg.magnetic_field.z = b.z();
    // The white noise, and only that. A hard-iron offset is precisely the
    // error a covariance cannot describe -- it is not random -- so it is not
    // in here, and an estimator that trusts this matrix will be confidently
    // wrong by exactly the configured bias. That is the point of modelling it.
    const double var = mag_.noise_t * mag_.noise_t;
    msg.magnetic_field_covariance = {var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var};
    mag_pub_->publish(msg);
  }

  /// Throttled, so a paused world says so once rather than at the sensor rate.
  void warnStale(double now) const
  {
    if (!have_truth_) {return;}
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_, 2000,
      "no ground truth for %.2f s (limit %.2f) -- sensors are silent rather "
      "than reporting a frozen state",
      now - truth_stamp_, truth_timeout_s_);
  }

  dsim_time::SimEpoch epoch_;
  // Throttling on the simulated clock stays mute until simulated time passes
  // the value it had before a reset. A warning about right now needs a clock
  // that only moves forwards.
  mutable rclcpp::Clock steady_ {RCL_STEADY_TIME};

  TofConfig tof_;
  FlowConfig flow_;
  MagConfig mag_;
  Eigen::Vector3d hard_iron_ {Eigen::Vector3d::Zero()};   ///< body frame, per run
  double fov_rad_ {0.0};
  double truth_timeout_s_ {0.1};

  std::mt19937 rng_;
  std::normal_distribution<double> gauss_ {0.0, 1.0};

  mutable std::mutex mutex_;
  Eigen::Quaterniond q_ {Eigen::Quaterniond::Identity()};
  Eigen::Matrix3d R_ {Eigen::Matrix3d::Identity()};
  Eigen::Vector3d v_body_ {Eigen::Vector3d::Zero()};
  Eigen::Vector3d omega_true_ {Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_ {Eigen::Vector3d::Zero()};
  double altitude_ {0.0};
  double true_range_ {kTooFar};
  double last_range_ {kTooFar};
  double truth_stamp_ {0.0};
  bool have_truth_ {false};
  double last_flow_s_ {0.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::Publisher<dsim_msgs::msg::OpticalFlow>::SharedPtr flow_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::TimerBase::SharedPtr tof_timer_;
  rclcpp::TimerBase::SharedPtr flow_timer_;
  rclcpp::TimerBase::SharedPtr mag_timer_;
};

}  // namespace dsim_sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dsim_sensors::SensorsNode>());
  rclcpp::shutdown();
  return 0;
}
