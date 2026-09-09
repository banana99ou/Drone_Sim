# One sample of a flat-output trajectory for a multirotor.
#
# A quadrotor is differentially flat in (x, y, z, yaw): given position and its
# derivatives plus yaw, the required attitude and thrust are determined
# algebraically. Send the derivatives you have — the controller uses velocity
# and acceleration as feedforward and does NOT numerically differentiate
# position, which is what lets it track fast trajectories without lag.

builtin_interfaces/Duration time_from_start   # offset from Trajectory.header.stamp

geometry_msgs/Point   position        # m,      world ENU frame
geometry_msgs/Vector3 velocity        # m/s     optional, zeros if unknown
geometry_msgs/Vector3 acceleration    # m/s^2   optional, zeros if unknown
geometry_msgs/Vector3 jerk            # m/s^3   optional, currently unused

float64 yaw                           # rad, world ENU, +CCW about z
float64 yaw_rate                      # rad/s
