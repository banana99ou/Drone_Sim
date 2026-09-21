// Decimates the fast telemetry topics for viewers.
//
//   /drone/truth          250 Hz  ->  /drone/viz/truth          30 Hz
//   /drone/setpoint       250 Hz  ->  /drone/viz/setpoint       30 Hz
//   /drone/control_debug  250 Hz  ->  /drone/viz/control_debug  30 Hz
//   /drone/imu            250 Hz  ->  /drone/viz/imu            30 Hz
//   <state_topic>         250 Hz  ->  /drone/viz/state          30 Hz
//
// The last one is whichever Odometry the controller is flying on -- /drone/truth
// or /drone/state_est, chosen by the state:= launch argument -- so a check can
// compare what the controller resolved against what it was actually given.
// scripts/check_telemetry.py asserts |velocity_world| == |twist| on that pair;
// against /drone/truth it would fail by exactly the estimation error, which is
// a measurement, not an identity.
//
// Why this node exists, with numbers.
//
// The browser viewer is a Python process, and it was subscribing to all four of
// those at full rate: about 1000 messages a second. Measured cost of a Python
// subscription with the cheapest possible callback -- a single increment -- is
// ~12% of a CPU core per 250 Hz topic, and the cost is in rclpy's ingestion
// path BEFORE the callback runs, so decimating inside the callback saves
// nothing. The viewer was burning 60% of a core with no browser connected at
// all, more than Gazebo's own 52%, to prepare a stream that goes out at 30 Hz.
// Every one of those messages was overwritten before anyone could see it.
//
// The same subscriptions in C++ are nearly free, so the decimation belongs
// here. A viewer must never be the reason the physics slows down.
//
// The full-rate topics stay exactly as they are, and this node is a pure
// pass-through of the LATEST message -- it never interpolates or averages.
// That matters: the full-rate /drone/control_debug is what caught a 10 ms
// once-per-lap actuator saturation event. Decimating the original would have
// hidden it, which is why the fix was to add a slow copy rather than to slow
// the original down.

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <dsim_msgs/msg/control_debug.hpp>
#include <dsim_msgs/msg/trajectory_setpoint.hpp>

namespace dsim_eval
{

/// Holds the latest message of one type and republishes it on a timer.
template<typename MsgT>
class LatestRelay
{
public:
  LatestRelay(rclcpp::Node * node, const std::string & in, const std::string & out)
  {
    pub_ = node->create_publisher<MsgT>(out, rclcpp::SensorDataQoS());
    sub_ = node->create_subscription<MsgT>(
      in, rclcpp::SensorDataQoS(),
      [this](typename MsgT::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = msg;
      });
  }

  /// Publishes the newest message, if one has arrived since the last tick.
  ///
  /// Republishing an unchanged message would be worse than silence: the viewer
  /// uses arrival to decide the stream is alive, so a stalled source must look
  /// stalled rather than steady.
  void tick()
  {
    typename MsgT::SharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg = latest_;
      latest_.reset();
    }
    if (msg) {pub_->publish(*msg);}
  }

private:
  std::mutex mutex_;
  typename MsgT::SharedPtr latest_;
  typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
  typename rclcpp::Subscription<MsgT>::SharedPtr sub_;
};

class VizRelayNode : public rclcpp::Node
{
public:
  VizRelayNode()
  : Node("dsim_viz_relay")
  {
    const double rate = declare_parameter("rate_hz", 30.0);
    const std::string state_topic = declare_parameter("state_topic", std::string("/drone/truth"));

    truth_ = std::make_unique<LatestRelay<nav_msgs::msg::Odometry>>(
      this, "/drone/truth", "/drone/viz/truth");
    setpoint_ = std::make_unique<LatestRelay<dsim_msgs::msg::TrajectorySetpoint>>(
      this, "/drone/setpoint", "/drone/viz/setpoint");
    debug_ = std::make_unique<LatestRelay<dsim_msgs::msg::ControlDebug>>(
      this, "/drone/control_debug", "/drone/viz/control_debug");
    imu_ = std::make_unique<LatestRelay<sensor_msgs::msg::Imu>>(
      this, "/drone/imu", "/drone/viz/imu");
    state_ = std::make_unique<LatestRelay<nav_msgs::msg::Odometry>>(
      this, state_topic, "/drone/viz/state");

    // Sim-clock timer, like every other periodic task here: if Gazebo runs
    // slower than real time, a wall timer would tick faster than simulated
    // time and quietly change the decimation ratio.
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate),
      [this] {
        truth_->tick();
        setpoint_->tick();
        debug_->tick();
        imu_->tick();
        state_->tick();
      });

    RCLCPP_INFO(
      get_logger(), "viz relay up: 5 topics decimated to %.0f Hz, controller state from %s",
      rate, state_topic.c_str());
  }

private:
  std::unique_ptr<LatestRelay<nav_msgs::msg::Odometry>> truth_;
  std::unique_ptr<LatestRelay<dsim_msgs::msg::TrajectorySetpoint>> setpoint_;
  std::unique_ptr<LatestRelay<dsim_msgs::msg::ControlDebug>> debug_;
  std::unique_ptr<LatestRelay<sensor_msgs::msg::Imu>> imu_;
  std::unique_ptr<LatestRelay<nav_msgs::msg::Odometry>> state_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dsim_eval

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dsim_eval::VizRelayNode>());
  rclcpp::shutdown();
  return 0;
}
