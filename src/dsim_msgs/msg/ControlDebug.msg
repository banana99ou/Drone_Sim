# One control step, exactly as the controller saw it.
#
# Published every step on /drone/control_debug, best-effort: this is telemetry,
# and a slow subscriber must never be able to back-pressure the control loop.
#
# It exists so the control action can be SEEN. Without it a viewer can only
# draw where the vehicle is, which looks identical whether the stack is working
# hard or coasting. With it you can draw what each loop demanded and what the
# rotors could actually deliver.
#
# Frames:
#   torque, body_rate_error, attitude_error   BODY  (FLU: +x fwd, +y left, +z up)
#   position_error, velocity_error,
#   desired_force                             WORLD (ENU, +z up)
# A consumer that wants a body-frame quantity in world coordinates should
# rotate it with the pose it is drawing, not with a pose from another message.
#
# Deliberate redundancy, kept as a cross-check rather than removed:
#   sum(rotor_thrust_n)                  == realised_thrust_n
#   allocation * rotor_thrust_n          == realised_torque_nm
# Both sides are computed and sent separately so a mixer that stops inverting
# shows up as a visible disagreement instead of hiding inside one number.
# scripts/flight_check.sh asserts the first identity on a live run.

std_msgs/Header header

bool armed                       # false => the loop is not commanding; all demands are zero

# --- vehicle constants, so a consumer needs no config file of its own -------
float64 mass_kg
float64 gravity_m_s2
float64 max_rotor_thrust_n       # scale reference: a rotor cannot exceed this

# --- wrench the controller demanded ----------------------------------------
float64 thrust_n                 # along body +z
geometry_msgs/Vector3 torque_nm

# --- wrench the rotors can actually deliver, after per-rotor clamping -------
float64 realised_thrust_n
geometry_msgs/Vector3 realised_torque_nm
bool saturated                   # true when realised != demanded, for any component

# --- per rotor, in mixer index order: 0 FL, 1 BR, 2 FR, 3 BL ---------------
# rotor_position is the hub in the BODY frame, from the same geometry table the
# allocation matrix is built from. It travels with the thrusts so that anything
# reporting "this rotor is producing this much" cannot attribute a thrust to the
# wrong arm -- the position and the force come from one object.
geometry_msgs/Vector3[4] rotor_position
float64[4] rotor_thrust_n
float64[4] rotor_speed_rad_s

# --- the state the loop reacted to -----------------------------------------
# Velocity in the WORLD frame, already resolved. nav_msgs/Odometry states twist
# in the child frame (REP-145) and Gazebo follows that, so /drone/odom carries
# BODY-frame velocity; the controller is the one node that knows which
# convention was configured. Publishing the resolved vector here means no
# consumer has to guess, and guessing wrong is invisible at hover and only
# wrong in banked flight -- the worst kind of silent error.
# Cross-check available to any consumer: |velocity_world| == |odom twist|,
# because a rotation cannot change a vector's length.
geometry_msgs/Vector3 velocity_world

# --- what each loop of the cascade is reacting to (state - reference) -------
geometry_msgs/Vector3 position_error
geometry_msgs/Vector3 velocity_error
geometry_msgs/Vector3 attitude_error     # so(3) error vector
geometry_msgs/Vector3 body_rate_error

# --- the outer loop's output; normalise it for the commanded thrust axis ----
geometry_msgs/Vector3 desired_force
float64 commanded_tilt_rad
bool tilt_clamped                # the demand was more aggressive than allowed
