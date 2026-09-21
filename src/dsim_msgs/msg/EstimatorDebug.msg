# One step of the state estimator, exactly as it happened.
#
# Published on /drone/estimator_debug at the IMU rate, best-effort: telemetry,
# never allowed to back-pressure the filter. It exists so a live check can
# tell "the estimator is fusing the sensors" from "the estimator is
# publishing something": every correction that was applied on this step is
# flagged, and every sensor-derived intermediate is reported alongside the
# fused state, so the two can be compared against ground truth separately.
#
# Frames: attitude is body->world (FLU body, ENU world); velocities are in
# the WORLD frame here (the fused /drone/state_est carries twist in the body
# frame per REP-145; this message is for looking at, not for control).

std_msgs/Header header

bool initialised                    # attitude has been set from accel (+mag)

# --- attitude ----------------------------------------------------------------
geometry_msgs/Quaternion attitude   # the estimate, body->world
geometry_msgs/Vector3 gyro_bias_rad_s   # estimated gyro bias, BODY frame
float64 yaw_est_rad                 # ZYX yaw of the estimate (heading of body x)
float64 mag_heading_rad             # yaw the LAST mag sample alone implies, tilt-
                                    # compensated with the current estimate; NaN
                                    # until a mag sample has been seen
bool mag_seen                       # any /drone/mag sample has ever arrived
bool mag_configured                 # a world field was supplied (mag usable)

# --- velocity ----------------------------------------------------------------
geometry_msgs/Vector3 velocity_world        # the fused estimate, m/s
geometry_msgs/Vector3 flow_velocity_world   # the last ACCEPTED flow reading as a
                                            # world velocity (body-z from the
                                            # estimate); zero until one arrives
float64 tof_altitude_m              # last accepted range * cos(tilt_est)
float64 altitude_est_m              # the filter's own altitude state (from the
                                    # IMU integral pinned by the range) -- NOT
                                    # what /drone/state_est reports as z, which
                                    # is truth for now
float64 altitude_rate_est_m_s       # = velocity_world.z; repeated so the pair
                                    # tof_altitude / altitude_rate reads together
float64 tof_altitude_rate_raw_m_s   # consecutive accepted altitudes differenced;
                                    # what the filter deliberately does NOT use,
                                    # published so its noise can be seen

# --- what happened on this step ---------------------------------------------
bool accel_correction_applied       # roll/pitch pulled towards the accelerometer
bool mag_correction_applied         # yaw pulled towards the (held) mag heading
bool flow_correction_applied        # a flow reading was fused since the last step
bool tof_correction_applied         # a range reading was fused since the last step

# --- counters, since the last reset ------------------------------------------
uint32 flow_rejected                # quality 0, NaN range, or bad interval
uint32 tof_rejected                 # +/-inf, non-positive, or beam above horizon
uint32 epochs                       # simulator restarts observed (clock went back)
