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

# --- the scenario this is scoring against ------------------------------------
string scenario                  # empty for a world with no scenario
float64 scenario_duration_s      # the plan horizon T; obstacle windows live inside it
