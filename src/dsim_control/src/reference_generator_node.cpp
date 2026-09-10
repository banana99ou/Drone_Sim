// Built-in reference trajectories, so the simulator can be exercised and
// validated with NO planner attached.
//
// This exists to separate two failure modes that otherwise look identical:
// "my planner produces bad paths" and "the sim/controller cannot fly".
// If the circle below tracks to a few centimetres, the plant and controller
// are healthy and any later tracking blow-up is attributable to the plan.
//
// It publishes on the same topic a planner would (/drone/trajectory) using a
// rolling horizon, so it exercises exactly the same code path as replanning.

#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <dsim_msgs/msg/trajectory.hpp>
#include <dsim_time/sim_epoch.hpp>

using namespace std::chrono_literals;

namespace dsim_control
{

class ReferenceGeneratorNode : public rclcpp::Node
{
public:
  ReferenceGeneratorNode()
  : Node("drone_reference_generator")
  {
    mode_        = declare_parameter("mode", std::string("circle"));
    altitude_    = declare_parameter("altitude_m", 1.5);
    radius_      = declare_parameter("radius_m", 2.0);
    period_      = declare_parameter("period_s", 12.0);
    center_      = declare_parameter<std::vector<double>>("center", {0.0, 0.0});
    yaw_follows_ = declare_parameter("yaw_follows_path", true);
    horizon_s_   = declare_parameter("horizon_s", 2.0);
    sample_dt_   = declare_parameter("sample_dt_s", 0.05);
    publish_hz_  = declare_parameter("publish_rate_hz", 5.0);
    takeoff_s_   = declare_parameter("takeoff_s", 4.0);

    pub_ = create_publisher<dsim_msgs::msg::Trajectory>(
      "/drone/trajectory", rclcpp::QoS(4));

    t0_ = get_clock()->now();
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / publish_hz_),
      [this] {publish();});

    RCLCPP_INFO(
      get_logger(), "reference generator: mode=%s alt=%.2f m radius=%.2f m period=%.1f s",
      mode_.c_str(), altitude_, radius_, period_);
  }

private:
  struct Sample
  {
    double x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate;
  };

  Sample evaluate(double t) const
  {
    Sample s{};

    // The quintic below is only a valid interpolant on [0, T]; outside it the
    // basis functions diverge fast. t used to be non-negative by construction,
    // which stopped being true when the world became resettable -- one sample
    // taken between the clock jumping back and t0_ being re-anchored is enough
    // to emit a setpoint hundreds of metres away, and the controller would
    // chase it.
    t = std::max(0.0, t);

    // Phase 1: takeoff, blended into the cruise path with a quintic Hermite.
    //
    // Three separate discontinuities had to die here, and each one showed up as
    // a tracking spike rather than as an error message:
    //   position  -- takeoff ended at the origin while the lemniscate starts at
    //                (radius, 0): a 2 m step, worth 200 cm of tracking error.
    //   velocity  -- takeoff ended at rest while the circle starts at
    //                1.05 m/s: worth 32 cm.
    //   yaw       -- holding the path tangent from t=0 demands an instant
    //                90-degree slew.
    //
    // The blend matches position, velocity AND acceleration at the seam, so the
    // reference is C2 there and a controller can track it without a transient.
    // Boundary conditions: at rest with zero acceleration on the pad, exactly
    // cruise(0) in all three at handover.
    if (t < takeoff_s_) {
      const double T = takeoff_s_;
      const double u = t / T;
      const Sample c0 = cruise(0.0);

      // Quintic Hermite basis with p'(0) = p''(0) = 0.
      const double u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
      const double h0 = 1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5;   // start point
      const double h3 = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;         // end point
      const double h4 = -4.0 * u3 + 7.0 * u4 - 3.0 * u5;          // end velocity
      const double h5 = 0.5 * u3 - u4 + 0.5 * u5;                 // end accel

      const double d0 = -30.0 * u2 + 60.0 * u3 - 30.0 * u4;
      const double d3 = 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
      const double d4 = -12.0 * u2 + 28.0 * u3 - 15.0 * u4;
      const double d5 = 1.5 * u2 - 4.0 * u3 + 2.5 * u4;

      const double e0 = -60.0 * u + 180.0 * u2 - 120.0 * u3;
      const double e3 = 60.0 * u - 180.0 * u2 + 120.0 * u3;
      const double e4 = -24.0 * u + 84.0 * u2 - 60.0 * u3;
      const double e5 = 3.0 * u - 12.0 * u2 + 10.0 * u3;

      auto blend = [&](double p0, double p1, double v1, double a1,
                       double & pos, double & vel, double & acc) {
          pos = p0 * h0 + p1 * h3 + v1 * T * h4 + a1 * T * T * h5;
          vel = (p0 * d0 + p1 * d3 + v1 * T * d4 + a1 * T * T * d5) / T;
          acc = (p0 * e0 + p1 * e3 + v1 * T * e4 + a1 * T * T * e5) / (T * T);
        };

      blend(center_.at(0), c0.x, c0.vx, c0.ax, s.x, s.vx, s.ax);
      blend(center_.at(1), c0.y, c0.vy, c0.ay, s.y, s.vy, s.ay);
      blend(0.0,           c0.z, c0.vz, c0.az, s.z, s.vz, s.az);
      blend(0.0,           c0.yaw, c0.yaw_rate, 0.0, s.yaw, s.yaw_rate, dummy_);
      return s;
    }

    return cruise(t - takeoff_s_);
  }

  /// The cruise trajectory, with t measured from the END of takeoff.
  Sample cruise(double tt) const
  {
    Sample s{};
    const double w = 2.0 * M_PI / period_;

    if (mode_ == "hover") {
      s.x = center_.at(0);
      s.y = center_.at(1);
      s.z = altitude_;
    } else if (mode_ == "circle") {
      s.x = center_.at(0) + radius_ * std::cos(w * tt);
      s.y = center_.at(1) + radius_ * std::sin(w * tt);
      s.z = altitude_;
      s.vx = -radius_ * w * std::sin(w * tt);
      s.vy =  radius_ * w * std::cos(w * tt);
      s.ax = -radius_ * w * w * std::cos(w * tt);
      s.ay = -radius_ * w * w * std::sin(w * tt);
      if (yaw_follows_) {
        s.yaw = std::atan2(s.vy, s.vx);
        s.yaw_rate = w;
      }
    } else if (mode_ == "lemniscate") {
      // Figure-eight: velocity and curvature both reverse sign, which is a
      // much harsher test of a controller than a circle's constant curvature.
      const double c = std::cos(w * tt), sn = std::sin(w * tt);
      const double den = 1.0 + sn * sn;
      s.x = center_.at(0) + radius_ * c / den;
      s.y = center_.at(1) + radius_ * c * sn / den;
      s.z = altitude_;
      // Numerical derivatives: exact analytic ones add nothing here because the
      // controller only needs feedforward accurate to a few percent.
      const double h = 1e-3;
      const double c1 = std::cos(w * (tt + h)), s1 = std::sin(w * (tt + h));
      const double d1 = 1.0 + s1 * s1;
      const double c2 = std::cos(w * (tt - h)), s2 = std::sin(w * (tt - h));
      const double d2 = 1.0 + s2 * s2;
      const double xp = center_.at(0) + radius_ * c1 / d1, xm = center_.at(0) + radius_ * c2 / d2;
      const double yp = center_.at(1) + radius_ * c1 * s1 / d1, ym = center_.at(1) + radius_ * c2 * s2 / d2;
      s.vx = (xp - xm) / (2.0 * h);
      s.vy = (yp - ym) / (2.0 * h);
      s.ax = (xp - 2.0 * s.x + xm) / (h * h);
      s.ay = (yp - 2.0 * s.y + ym) / (h * h);
      if (yaw_follows_) {s.yaw = std::atan2(s.vy, s.vx);}
    } else if (mode_ == "step") {
      // Alternating position step every half period. Deliberately untrackable
      // in the ideal sense; useful for measuring settling time and overshoot.
      const bool second = std::fmod(tt, period_) > (period_ / 2.0);
      s.x = center_.at(0) + (second ? radius_ : 0.0);
      s.y = center_.at(1);
      s.z = altitude_;
    } else {
      s.x = center_.at(0);
      s.y = center_.at(1);
      s.z = altitude_;
    }
    return s;
  }

  void publish()
  {
    const auto now = get_clock()->now();
    // The world was reset: fly the profile again from the beginning rather
    // than from wherever the old run had got to. t0_ happened to be ~0 at
    // launch, so this used to come out right by accident -- but only for a
    // generator started before the clock did, and only for a reset that lands
    // exactly on zero.
    if (epoch_.restarted(now.seconds())) {
      t0_ = now;
      RCLCPP_INFO(get_logger(), "simulated time went backwards — restarting the reference");
    }
    const double t_now = (now - t0_).seconds();

    dsim_msgs::msg::Trajectory msg;
    msg.header.stamp = now;
    msg.header.frame_id = "world";

    const size_t n = static_cast<size_t>(horizon_s_ / sample_dt_) + 1;
    msg.points.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      const double dt = static_cast<double>(i) * sample_dt_;
      const Sample s = evaluate(t_now + dt);

      dsim_msgs::msg::TrajectorySetpoint p;
      p.time_from_start.sec = static_cast<int32_t>(dt);
      p.time_from_start.nanosec = static_cast<uint32_t>((dt - std::floor(dt)) * 1e9);
      p.position.x = s.x; p.position.y = s.y; p.position.z = s.z;
      p.velocity.x = s.vx; p.velocity.y = s.vy; p.velocity.z = s.vz;
      p.acceleration.x = s.ax; p.acceleration.y = s.ay; p.acceleration.z = s.az;
      p.yaw = s.yaw;
      p.yaw_rate = s.yaw_rate;
      msg.points.push_back(p);
    }
    pub_->publish(msg);
  }

  dsim_time::SimEpoch epoch_;
  mutable double dummy_ {0.0};
  std::string mode_;
  double altitude_, radius_, period_, horizon_s_, sample_dt_, publish_hz_, takeoff_s_;
  std::vector<double> center_;
  bool yaw_follows_;
  rclcpp::Time t0_;
  rclcpp::Publisher<dsim_msgs::msg::Trajectory>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dsim_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dsim_control::ReferenceGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
