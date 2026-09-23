# Where the obstacles are, how close the vehicle is, and every hit so far.
#
# Published by the referee (dsim_eval) on /drone/eval/clearance at the status
# rate. It exists because a collision that is only a boolean tells you the run
# failed and nothing about why: this carries WHERE the vehicle was, WHAT it
# hit and WHEN, so the viewer can put a marker on the spot.
#
# Clearance here is ANALYTIC: the referee follows every obstacle's centre along
# its Bezier in space-time (t on the scenario clock, which starts at
# scenario.start_s of simulated time) and measures the vehicle's ground-truth
# position against it, minus the vehicle's own radius. It is signed: negative
# means the envelope has entered the obstacle by that much. This is the same
# quantity the planner certifies (compute_min_clearance on its side), so on a
# perfectly tracked plan the two must agree -- a disagreement means one of them
# has the obstacles somewhere else. scripts/check_planner.py holds the
# positions below against the planner's own obstacle_positions_at(), which is
# an independent implementation of the same motion.
#
# The obstacles are NOT in the physics. They never were -- they have no
# collision geometry, because the referee scores clearance analytically and a
# contact solver would only add a second, differently shaped opinion. Since
# two scenarios move theirs along CUBICS and three switch them off partway
# through, they are not Gazebo bodies at all any more: this report is what
# exists, and the viewer draws from it.

std_msgs/Header header

float64 scenario_time_s          # sim time minus scenario.start_s; negative before the plan starts

# The vehicle position every number below was measured at, and the instant is
# scenario_time_s above. The report describes ONE instant: the obstacle
# positions, the clearance and the sight-line margins are all evaluated here,
# not at the moment of publishing.
#
# It is in the message because a cross-check that fetches the pose from
# somewhere else is comparing two different samples. At loiter's 3.4 m/s, the
# few milliseconds between the referee's last odometry and the viewer's showed
# up as 2.5 cm of "disagreement" between two implementations that agree
# exactly.
geometry_msgs/Point sample_position
bool sample_valid                # false before any state has arrived

# --- the obstacles NOW, in world ENU, one entry each ------------------------
# `active` is false outside an obstacle's window, and then it is ABSENT, not
# far away: `positions` still carries the clamped endpoint so a viewer has
# something to fade out, but nothing inactive enters a clearance. `types` is
# "sphere" or "column"; a column is a disc at every altitude, which is what an
# obstacle from a scenario planned in two spatial dimensions actually is.
string[] names
geometry_msgs/Point[] positions
float64[] radii
bool[] active
string[] types

# --- the closest one, right now ----------------------------------------------
float64 clearance_m              # signed; minus vehicle radius; +inf with no obstacles
string nearest
float64 min_clearance_m          # worst so far this run

# --- every analytic hit this run, edge-triggered on clearance going negative -
geometry_msgs/Point[] hit_positions   # vehicle position at the moment of entry
string[] hit_with
float64[] hit_times_s                 # on the scenario clock
float64[] hit_depths_m                # deepest penetration during that entry

# --- line of sight, for scenarios that have stations -------------------------
# A station is a fixed ground point the vehicle is meant to stay visible from.
# The margin is the distance from an obstacle's centre to the SEGMENT between
# station and vehicle, less its radius, minimised over the obstacles active
# then and over every station: positive means every station can see the
# vehicle, negative means at least one cannot, by that much.
#
# This is the same arithmetic as the planner's los_margin_at(), written out
# independently on this side; scripts/check_planner.py holds the two against
# each other, which is the only way to find out whether the solver's convex
# occlusion certificate means what it claims.
#
# Empty stations means the scenario does not constrain visibility, and then
# los_margin_m is +inf -- not zero, which would read as "just barely visible".
geometry_msgs/Point[] stations
float64 los_margin_m             # signed; +inf with no stations or nothing in the way
string los_blocker               # the obstacle responsible, "" when nothing is
string los_station               # the station with the worst view, "" with none
float64 min_los_margin_m         # worst so far this run

# --- every loss of sight this run, edge-triggered on the margin going negative
geometry_msgs/Point[] dark_positions   # vehicle position at the moment sight was lost
string[] dark_station
string[] dark_blocker
float64[] dark_times_s                 # on the scenario clock
float64[] dark_durations_s             # how long sight stayed lost
float64[] dark_depths_m                # deepest the sight line went into the body

# --- the scenario this is scoring against ------------------------------------
string scenario                  # empty for a world with no scenario
float64 scenario_duration_s      # the plan horizon T; obstacle windows live inside it
