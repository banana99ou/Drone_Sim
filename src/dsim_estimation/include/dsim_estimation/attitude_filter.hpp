#pragma once
#include <Eigen/Dense>

namespace dsim_estimation
{

/// Tuning for the attitude filter. Continuous-time gains, so the behaviour
/// does not change with the IMU rate: every update scales them by dt.
struct AttitudeConfig
{
  // These member defaults are for TESTS. estimator_node reads the real values
  // from src/dsim_estimation/config/estimator.yaml through the launch file.

  /// Proportional pull of roll/pitch towards the accelerometer's "up", 1/s.
  /// Deliberately LOW -- see the class comment on banked turns. 0.1 means a
  /// 10 s time constant, so a 20 deg bank held for one 3.5 s lap drags the
  /// estimate about 1 deg towards level, while a 2e-4 rad/s gyro bias left
  /// uncorrected by ki would cost only 0.1 deg of steady tilt.
  double kp_accel {0.1};
  /// Integral gain on the accelerometer error, feeding the roll/pitch
  /// components of the gyro-bias state, 1/s^2. SHIPPED AS ZERO, and the
  /// reason is measured, not argued: in a coordinated turn the accel error
  /// is constant in the BODY frame -- "roll towards level", lap after lap --
  /// which is exactly what a gyro bias looks like, so the integrator winds
  /// up without bound. At 0.01 it accumulated 0.01 rad/s of fake bias and
  /// half a degree of tilt error PER LAP of the 3.5 s demo circle (the
  /// wind-up test below pins this). The bias it would have estimated is
  /// 2e-4 rad/s, which through kp_accel costs 0.11 deg of steady tilt --
  /// less than the accelerometer's own 0.12 deg bias floor. Not worth a
  /// filter that gets worse the longer it flies.
  double ki_accel {0.0};
  /// Proportional pull of yaw towards the magnetometer heading, 1/s. Higher
  /// than kp_accel because the mag has no equivalent of the banked-turn
  /// failure: it measures a fixed field, not a force the vehicle produces.
  double kp_mag {0.2};
  /// Integral gain on the mag error, feeding the yaw-rate component of the
  /// bias, 1/s^2. Safe where ki_accel is not: the heading error a turn
  /// induces (through the tilt error's leak into the tilt compensation)
  /// rotates with the lap in the world frame and averages to zero.
  double ki_mag {0.02};
  /// How long one mag sample's correction keeps being applied, seconds. The
  /// mag arrives at 50 Hz and the gyro at 250 Hz; applying each mag error
  /// for one 4 ms step only would give it a fifth of its nominal gain, so
  /// the error is held until the next sample. Past this age the mag is
  /// considered gone and yaw becomes gyro-only.
  double mag_hold_s {0.1};
  /// The accelerometer is only a gravity reference while it is measuring
  /// mostly gravity. When | |a| - g | exceeds this the sample is skipped:
  /// the vehicle is in free fall, on the floor after a crash, or bouncing.
  /// 3 m/s^2 admits a coordinated turn at the 40 deg tilt clamp (g/cos 40 deg
  /// is 12.8 m/s^2, 3.0 above g) -- turns are handled by the low gain, not
  /// by the gate, because gating them out would leave roll and pitch
  /// unobserved during exactly the manoeuvres the velocity estimate needs
  /// them for.
  double accel_gate_m_s2 {3.0};
  /// Longest step the gyro is integrated across. A gap past this is a stall,
  /// not a sample interval; integrating one gyro reading across it would
  /// fabricate a rotation.
  double max_dt_s {0.05};
  /// Direction of the Earth's field in the WORLD frame, Tesla. Only its
  /// horizontal direction is used for heading; its magnitude and dip are
  /// irrelevant once the measurement is tilt-compensated and normalised.
  Eigen::Vector3d field_world {3.0e-05, 0.0, -4.0e-05};
  double gravity {9.80665};
};

/// Which corrections one step applied. Published so a live check can tell
/// "the mag is being fused" from "the mag is being received".
struct AttitudeStepInfo
{
  bool accel_applied {false};
  bool mag_applied {false};
};

/// Mahony-style explicit complementary filter on SO(3): a quaternion plus a
/// gyro-bias state, driven by the gyro and pulled towards two vector
/// references -- the accelerometer for roll and pitch, the magnetometer for
/// yaw. Chosen over an EKF because every term has a physical reading and a
/// single gain, which is what makes a wrong sign visible in a test.
///
/// Frames: body FLU, world ENU (+z up), q is body->world, as everywhere else
/// in this simulator. accel is SPECIFIC FORCE in the body frame, gravity
/// included, exactly as sensor_msgs/Imu carries it: a level vehicle at rest
/// reads (0, 0, +g), and that "up" vector is the reference.
///
/// THE KNOWN WEAKNESS, stated so nobody tunes it away by accident. The
/// accelerometer cannot separate gravity from the vehicle's own acceleration.
/// In a coordinated banked turn the specific force lies along body z -- the
/// rotors produce it -- so the accelerometer reports "up is along my z axis"
/// while the vehicle is banked 20 deg. The correction then pulls the estimate
/// TOWARDS LEVEL, at kp_accel per second, for as long as the turn lasts.
/// That is why kp_accel is low: with a 10 s time constant a 3.5 s lap costs
/// about a degree, and the gyro carries the attitude through the manoeuvre.
/// A high gain would make the filter confidently wrong in every turn. The
/// error is bounded, not eliminated, and test_estimation.cpp measures it.
///
/// The magnetometer path has a different weakness: a hard-iron offset in the
/// body frame is a heading error the filter cannot distinguish from the
/// field, so it is estimated by nobody and shows up as a yaw bias. The mag
/// error is applied only about the WORLD vertical, so a wrong field model can
/// corrupt yaw but never roll or pitch -- the accelerometer owns those.
class AttitudeFilter
{
public:
  explicit AttitudeFilter(const AttitudeConfig & cfg);

  /// Set the attitude from one accelerometer sample and, if `mag` is
  /// non-null, one magnetometer sample. Roll and pitch come from the
  /// direction of the specific force; yaw from the tilt-compensated heading.
  /// Without a mag, yaw starts at zero and is thereafter gyro-only.
  ///
  /// Returns false, and stays uninitialised, if the accel is unusable (not
  /// finite, or far from 1 g): the filter must not start from a fabricated
  /// attitude.
  bool initialise(const Eigen::Vector3d & accel_body, const Eigen::Vector3d * mag_body);

  /// Integrate one gyro sample over dt, with the bias removed. The
  /// accelerometer correction queued since the last predict and the held
  /// magnetometer correction are applied here, as angular-rate terms, which
  /// is what makes this a complementary filter rather than a switch between
  /// sources.
  void predict(const Eigen::Vector3d & gyro_body, double dt);

  /// Queue a roll/pitch correction from one accelerometer sample; it is
  /// consumed by the next predict(). Returns whether it was accepted (see
  /// AttitudeConfig::accel_gate_m_s2).
  bool correctAccel(const Eigen::Vector3d & accel_body);

  /// Set the held yaw correction from one magnetometer sample, body frame,
  /// any units; it is applied by every predict() until the next sample or
  /// until mag_hold_s expires. Returns false for a non-finite or zero-length
  /// sample, or one whose tilt-compensated horizontal component is
  /// degenerate.
  bool correctMag(const Eigen::Vector3d & mag_body);

  /// Tilt-compensated heading of a body-frame field sample against the
  /// configured world field, radians, using the CURRENT attitude estimate
  /// for the tilt. Exposed for the debug message and the tests.
  double magHeadingError(const Eigen::Vector3d & mag_body) const;

  /// Forget everything: initialised() goes false, bias to zero. For a
  /// simulator reset, where the vehicle is teleported home.
  void reset();

  bool initialised() const {return initialised_;}
  const Eigen::Quaterniond & quaternion() const {return q_;}
  Eigen::Matrix3d rotation() const {return q_.toRotationMatrix();}
  const Eigen::Vector3d & gyroBias() const {return bias_;}
  /// What the last predict() applied. Cleared at the start of each predict.
  const AttitudeStepInfo & lastStep() const {return last_;}
  const AttitudeConfig & config() const {return cfg_;}

  /// Roll, pitch, yaw (ZYX, radians) of the estimate. For tests and logs;
  /// nothing in the filter itself runs on Euler angles.
  Eigen::Vector3d rollPitchYaw() const;

  /// Test hook: overwrite the attitude without touching the bias or the
  /// initialised flag, so a test can start the filter from a known WRONG
  /// attitude and watch it converge.
  void setQuaternionForTest(const Eigen::Quaterniond & q);

private:
  AttitudeConfig cfg_;
  Eigen::Quaterniond q_ {Eigen::Quaterniond::Identity()};
  Eigen::Vector3d bias_ {Eigen::Vector3d::Zero()};
  /// Accelerometer error since the last predict, body frame, unweighted:
  /// v_meas x v_pred. Consumed and cleared by predict().
  Eigen::Vector3d accel_error_ {Eigen::Vector3d::Zero()};
  bool accel_pending_ {false};
  /// Held magnetometer error: sin of the heading error, a rotation about the
  /// WORLD vertical. Kept as a world-frame scalar and rotated into the body
  /// at each predict, so holding it across a rotating body stays exact.
  double mag_error_z_ {0.0};
  bool have_mag_ {false};
  double mag_age_s_ {0.0};
  AttitudeStepInfo last_;
  bool initialised_ {false};
};

/// Roll/pitch/yaw (ZYX) from a body->world quaternion. Shared with the
/// velocity tests and the node's logging.
Eigen::Vector3d rollPitchYawOf(const Eigen::Quaterniond & q);

/// Body->world quaternion from ZYX roll, pitch, yaw.
Eigen::Quaterniond quaternionFromRollPitchYaw(double roll, double pitch, double yaw);

}  // namespace dsim_estimation
