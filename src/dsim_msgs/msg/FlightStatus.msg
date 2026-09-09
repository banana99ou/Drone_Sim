# Per-run scoring published by the eval node on /drone/eval/status.
std_msgs/Header header

bool    armed
bool    trajectory_active
bool    collided                 # true once the guard envelope has touched anything
uint32  collision_count

float64 tracking_error_m         # instantaneous |p - p_ref|
float64 tracking_rmse_m          # running RMSE since the run started
float64 tracking_max_m           # worst instantaneous error so far
float64 min_obstacle_clearance_m # closest approach to any obstacle so far
float64 path_length_m
float64 elapsed_s
float64 energy_wh                # integrated from the rotor model
