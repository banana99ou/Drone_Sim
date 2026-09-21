#pragma once
#include <cstdint>

#include <Eigen/Dense>

namespace dsim_estimation
{

/// Tuning for the velocity filter. Continuous-time quantities, converted to
/// per-update gains from the actual interval, so a sensor that drops to half
/// rate gets half the correction per second and not half the time constant.
struct VelocityConfig
{
  // These member defaults are for TESTS. estimator_node reads the real values
  // from src/dsim_estimation/config/estimator.yaml through the launch file.

  double gravity {9.80665};
  /// Time constant of the pull of the horizontal velocity towards the
  /// optical-flow value, seconds. Sets where the crossover between "trust
  /// the integrated accelerometer" (above) and "trust the flow" (below)
  /// sits. 0.2 s: a flow sample is 0.02 rad/s x 1.5 m = 3 cm/s of noise at
  /// 50 Hz, and this averages ~10 of them, while an accelerometer bias of
  /// 0.02 m/s^2 integrated for 0.2 s is 4 mm/s of steady error.
  double flow_tau_s {0.2};
  /// Natural frequency and damping of the altitude / climb-rate pair driven
  /// by the rangefinder, rad/s and dimensionless. A 2 rad/s, 0.7 damped
  /// second-order response: the range's 2.5 cm of noise reaches the climb
  /// rate as ~8 mm/s instead of the ~1 m/s that differencing consecutive
  /// 30 Hz readings would give.
  double tof_omega_rad_s {2.0};
  double tof_zeta {0.7};
  /// Longest interval one accelerometer sample is integrated across, and the
  /// longest gap a correction is scaled by. Past it the stream stalled, and
  /// the honest thing is to skip rather than to apply one sample's worth of
  /// acceleration to a whole second.
  double max_dt_s {0.1};
};

/// One optical-flow reading, exactly as dsim_msgs/OpticalFlow carries it.
struct FlowReading
{
  double integration_time_s {0.0};
  double integrated_x {0.0};       ///< rad, flow about body +x
  double integrated_y {0.0};       ///< rad, flow about body +y
  double integrated_xgyro {0.0};   ///< rad, gyro integral, same interval
  double integrated_ygyro {0.0};
  double ground_distance_m {0.0};  ///< the range to scale by; NaN = none
  std::uint8_t quality {0};        ///< 0 = no measurement
};

/// Which corrections were applied since the last predict(), for telemetry.
struct VelocityStepInfo
{
  bool flow_applied {false};
  bool tof_applied {false};
};

/// World-frame velocity from an accelerometer, pinned by an optical-flow
/// sensor horizontally and a downward rangefinder vertically.
///
/// A complementary structure rather than a Kalman filter for the same
/// reason as the attitude: every term is one physical sentence. The
/// accelerometer, rotated with the attitude estimate and with gravity
/// removed, is integrated at 250 Hz and carries the high-frequency motion
/// with no lag -- which is what keeps the controller's velocity loop stable
/// on this estimate. The two slow sensors only pull the integral back where
/// it belongs.
///
/// Sign convention, tested rather than assumed: the IMU reports SPECIFIC
/// FORCE, gravity included, so a level vehicle at rest reads (0, 0, +g) and
///
///     a_world = R_est * accel_body + (0, 0, -g)
///
/// is zero at rest. Write "- g" as "+ g" and a hovering vehicle climbs at
/// 2 g in the estimate; use R^T for R and the horizontal acceleration of a
/// yawed vehicle lands on the wrong world axes. Both are mutations the
/// harness injects.
///
/// Where it stops being valid. The flow is only a velocity over a FLAT
/// FLOOR: it measures the angular motion of whatever texture is below, and
/// scales it by the rangefinder's slant range, so over a pillar top or a
/// slope the height is wrong and so is the velocity. The rangefinder gives
/// altitude only through cos(tilt) of the ESTIMATED attitude, so an attitude
/// error of e radians at tilt t is an altitude error of ~ range * sin(t) * e.
/// And an accelerometer bias is not estimated here: it becomes a steady
/// velocity offset of bias * flow_tau_s horizontally, and of
/// bias * 2 * zeta / omega vertically. The shipped 0.02 m/s^2 is 4 mm/s and
/// 14 mm/s respectively, both below what the sensors themselves contribute.
class VelocityFilter
{
public:
  explicit VelocityFilter(const VelocityConfig & cfg);

  /// Integrate one accelerometer sample (body frame, specific force) across
  /// dt, using the given body->world rotation. Also advances the internal
  /// altitude state by the vertical velocity.
  void predict(const Eigen::Matrix3d & R_est, const Eigen::Vector3d & accel_body, double dt);

  /// World-frame acceleration for one sample: the quantity predict()
  /// integrates. Exposed so the sign convention can be tested on its own.
  Eigen::Vector3d worldAcceleration(
    const Eigen::Matrix3d & R_est, const Eigen::Vector3d & accel_body) const;

  /// Horizontal correction from one optical-flow reading. The reading is
  /// turned into a body-frame velocity with the formula in OpticalFlow.msg,
  /// and the innovation is applied in the body x/y plane only -- the flow
  /// knows nothing about motion along body z -- then rotated into the world
  /// with R_est.
  ///
  /// The reading's own integration_time_s is the interval the pull is
  /// scaled by against flow_tau_s: it is the period the sensor actually
  /// covered, so a dropped sample gets proportionally more weight rather
  /// than being treated as one more 20 ms reading. Rejected, returning
  /// false and changing nothing, when quality is 0, the range is not
  /// finite or not positive, or the integration time is not positive.
  bool correctFlow(const Eigen::Matrix3d & R_est, const FlowReading & flow);

  /// Vertical correction from one rangefinder reading. The sensor reports
  /// SLANT range along body -z; the altitude is range * cos(tilt), with
  /// cos(tilt) = R_est(2, 2). Feeding the raw range in as altitude is wrong
  /// by range * (1 - cos tilt) -- 5 cm at 15 deg of bank -- and would show
  /// up as a phantom climb in every turn.
  ///
  /// `dt` is the interval since the last accepted range (the caller's
  /// clock; the message carries no interval of its own), which scales the
  /// second-order gains. The first accepted range only seeds the altitude
  /// state: one reading carries no rate information.
  ///
  /// Rejected, returning false, for a non-finite range (REP 117's +/-inf
  /// out-of-range markers), a non-positive range, a tilt at which the beam
  /// points at or above the horizon, or a non-positive dt after seeding.
  bool correctTof(const Eigen::Matrix3d & R_est, double range_m, double dt);

  /// Body-frame velocity a flow reading encodes, before any fusion. NaN
  /// components if the reading is unusable. Exposed for telemetry and tests.
  static Eigen::Vector3d flowBodyVelocity(const FlowReading & flow);

  /// Altitude a slant range encodes at the given attitude.
  static double tofAltitude(const Eigen::Matrix3d & R_est, double range_m);

  /// Seed the altitude state (e.g. from the first accepted range) without
  /// touching the velocity. Until this or the first correctTof() the
  /// altitude state is meaningless and altitudeValid() is false.
  void setAltitude(double z_m);

  /// Forget everything: velocity to zero, altitude unknown.
  void reset();

  const Eigen::Vector3d & velocityWorld() const {return v_;}
  double altitude() const {return z_;}
  bool altitudeValid() const {return z_valid_;}
  /// Last world-frame velocity derived from an accepted flow reading, for
  /// telemetry. Zero until one has been accepted.
  const Eigen::Vector3d & lastFlowVelocityWorld() const {return v_flow_world_;}
  /// Last altitude derived from an accepted range reading, for telemetry.
  double lastTofAltitude() const {return z_tof_;}
  /// What has been applied since the last predict(); cleared by predict().
  const VelocityStepInfo & lastStep() const {return last_;}
  const VelocityStepInfo & pending() const {return pending_;}
  std::uint32_t flowRejected() const {return flow_rejected_;}
  std::uint32_t tofRejected() const {return tof_rejected_;}
  const VelocityConfig & config() const {return cfg_;}

  /// Test hook: overwrite the velocity, so a test can start from a known
  /// wrong value and watch a correction pull it back.
  void setVelocityForTest(const Eigen::Vector3d & v) {v_ = v;}

private:
  VelocityConfig cfg_;
  Eigen::Vector3d v_ {Eigen::Vector3d::Zero()};
  double z_ {0.0};
  bool z_valid_ {false};
  Eigen::Vector3d v_flow_world_ {Eigen::Vector3d::Zero()};
  double z_tof_ {0.0};
  VelocityStepInfo pending_;
  VelocityStepInfo last_;
  std::uint32_t flow_rejected_ {0};
  std::uint32_t tof_rejected_ {0};
};

}  // namespace dsim_estimation
