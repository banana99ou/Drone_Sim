# A time-parameterised trajectory produced by a planner.
#
# Publish to /drone/trajectory. Each new message REPLACES the active
# trajectory, so a replanning planner just publishes again at its replan rate.
# header.stamp is the time at which points[0] should be executed; stamp it in
# the future to schedule, or with 'now' to start immediately.
#
# Points must be sorted by increasing time_from_start. The controller does
# linear interpolation in position and holds the final point when the
# trajectory runs out.

std_msgs/Header header
TrajectorySetpoint[] points
