# Where the obstacles are, how close the vehicle is, and every hit so far.
#
# Published by the referee (dsim_eval) on /drone/eval/clearance at the status
# rate. It exists because a collision that is only a boolean tells you the run
# failed and nothing about why: this carries WHERE the vehicle was, WHAT it
# hit and WHEN, so the viewer can put a marker on the spot.
#
# Clearance here is ANALYTIC: the referee knows every obstacle's centre as
# pos0 + vel * t (t on the scenario clock, which starts at scenario.start_s of
# simulated time) and measures the vehicle's ground-truth position against
# it, minus the vehicle's own radius. It is signed: negative means the
# envelope has entered the obstacle by that much. This is the same quantity
# the planner certifies (compute_min_clearance on its side), so on a perfectly
# tracked plan the two must agree -- a disagreement means one of them has the
# obstacles somewhere else.
#
# For moving obstacles the referee also listens to the poses Gazebo reports
# and records the worst disagreement with its own pos0 + vel * t. That is the
# check that the world being FLOWN is the world being SCORED. NaN until a
# pose for a listed obstacle has arrived, and a gate must treat NaN as a
# failure, not as "no problem".

std_msgs/Header header

float64 scenario_time_s          # sim time minus scenario.start_s; negative before the plan starts

# --- the obstacles NOW, in world ENU, one entry each ------------------------
string[] names
geometry_msgs/Point[] positions
float64[] radii

# --- the closest one, right now ----------------------------------------------
float64 clearance_m              # signed; minus vehicle radius; +inf with no obstacles
string nearest
float64 min_clearance_m          # worst so far this run

# --- every analytic hit this run, edge-triggered on clearance going negative -
geometry_msgs/Point[] hit_positions   # vehicle position at the moment of entry
string[] hit_with
float64[] hit_times_s                 # on the scenario clock
float64[] hit_depths_m                # deepest penetration during that entry

# --- does Gazebo agree where the obstacles are? --------------------------------
float64 world_mismatch_m         # max over obstacles of |reported - analytic|; NaN if unheard
uint32 world_poses_seen
