#pragma once
#include <Eigen/Dense>
#include <cstddef>

namespace dsim_control
{

/// Body angular rate for the rotational loop, from the gyro, with a sanity
/// gate and an optional low-pass.
///
/// Why this exists as its own object rather than three lines in the node:
///
/// The controller used to take its body rate from the simulator's
/// OdometryPublisher, which is a WHEELED-ROBOT plugin: it reconstructs
/// velocity by finite-differencing successive poses. A quaternion and its
/// negation are the same rotation, so once per revolution the reported
/// orientation flipped sign -- pose and yaw stayed perfectly smooth, and the
/// differentiated angular velocity jumped from 1.8 rad/s to 626 rad/s. The
/// controller believed it, demanded 111 N.m of torque against an airframe that
/// can produce 2.55, and slammed two rotors to full and two to zero for ~10 ms
/// once per lap. Nothing in the flight looked wrong; the referee's numbers
/// barely moved.
///
/// Two lessons, both encoded here:
///   1. Rates come from a GYRO, which measures them, not from a differentiated
///      attitude, which has a representation that can jump.
///   2. Even so, a control loop must not act on a physically impossible input.
///      The gate below is what turns "the state source lied" from a 44x torque
///      demand into a logged, counted, harmless rejection.
class BodyRateSource
{
public:
  /// max_rate_rad_s   reject any sample whose magnitude exceeds this
  /// lowpass_tau_s    first-order time constant; <= 0 disables filtering
  BodyRateSource(double max_rate_rad_s, double lowpass_tau_s);

  /// Feed one gyro sample. Returns the rate the controller should use.
  ///
  /// A rejected sample holds the previous accepted value rather than zeroing:
  /// zero would tell the rotational loop the vehicle had stopped rotating,
  /// which is its own lie and would provoke a correction just as wrong.
  Eigen::Vector3d update(const Eigen::Vector3d & measured, double dt);

  /// True once at least one plausible sample has arrived. Before that the
  /// controller has no rate feedback and must not command.
  bool valid() const {return valid_;}

  /// How many samples have been thrown away. Non-zero means the state source
  /// is producing impossible values -- worth surfacing, never worth ignoring.
  std::size_t rejected() const {return rejected_;}

  double maxRate() const {return max_rate_;}

  /// The last value handed out, without feeding a new sample.
  const Eigen::Vector3d & value() const {return value_;}

private:
  double max_rate_;
  double tau_;
  Eigen::Vector3d value_ {Eigen::Vector3d::Zero()};
  bool valid_ {false};
  std::size_t rejected_ {0};
};

}  // namespace dsim_control
