# Planner interface

Everything your planner needs to know. The simulator is a black box: **you
publish trajectories, it flies them and scores the result.**

## The contract

| Direction | Topic | Type | Rate |
|---|---|---|---|
| sim → you | `/drone/truth` | `nav_msgs/msg/Odometry` | 250 Hz |
| sim → you | `/drone/eval/status` | `dsim_msgs/msg/FlightStatus` | 20 Hz |
| **you → sim** | `/drone/trajectory` | `dsim_msgs/msg/Trajectory` | your replan rate |
| sim → you | `/drone/imu` | `sensor_msgs/msg/Imu` | 250 Hz |
| sim → you | `/drone/control_debug` | `dsim_msgs/msg/ControlDebug` | 250 Hz |
| sim → you | `/drone/setpoint` | `dsim_msgs/msg/TrajectorySetpoint` | 250 Hz |

**Renamed:** this topic was `/drone/odom` until it was pointed out that nothing
here is odometry — it is ground truth straight out of the simulator, with no
dead reckoning and no drift. Calling it odom invited exactly the wrong
assumption. If your planner subscribes to `/drone/odom`, change it to
`/drone/truth`.

**Do not use its `twist.angular`.** It is differentiated from the pose by
Gazebo's `OdometryPublisher`, which is a wheeled-robot plugin, and that
differentiation is blind to the quaternion double cover: once per revolution
the reported body rate jumps to 626 rad/s while the pose stays perfectly
smooth. Take angular rates from `/drone/imu`, which is a gyro and measures
them. The controller does.

`/drone/truth` is exactly that -- ground truth, with no noise and no drift. The
SENSORS are a different matter: `/drone/imu`, `/drone/tof` and
`/drone/optical_flow` all carry configured noise (see `config/drone.yaml` under
`drone.sensors`, and `scripts/check_sensors.py`, which measures the live signal
rather than trusting the config). So a planner can be written against perfect
state, against realistic sensors, or against both to compare -- but it should
say which.

Your planner
sees the true state.

### Reading back what the controller did

`/drone/control_debug` reports one control step exactly as the controller saw
it: the wrench each loop demanded, the wrench the rotors could actually
deliver, the four per-rotor thrusts and speeds, and every loop's error term.
Best-effort QoS — it is telemetry, and a slow subscriber must never be able to
back-pressure the control loop.

It is worth subscribing to from a planner for two reasons:

* **`saturated` and `tilt_clamped` tell you your trajectory was infeasible.**
  Without them, an over-aggressive plan looks like a tracking failure, and you
  would tune the wrong thing. If `tilt_clamped` is true, the controller gave up
  before the vehicle did.
* **It is self-describing.** `mass_kg`, `gravity_m_s2`, `max_rotor_thrust_n`
  and the four `rotor_position` entries travel with the data, so nothing
  downstream needs its own copy of `config/drone.yaml` to interpret it.

Two identities hold on every message, and are worth asserting if you consume
it: `sum(rotor_thrust_n) == realised_thrust_n`, and
`|velocity_world| == |odom twist|` (a rotation cannot change a length). Both
sides are computed separately on purpose, so a disagreement is visible rather
than silent. `scripts/check_telemetry.py` asserts them against a live run.

## Publishing a trajectory

```cpp
#include <dsim_msgs/msg/trajectory.hpp>

dsim_msgs::msg::Trajectory traj;
traj.header.stamp = node->get_clock()->now();   // when points[0] executes
traj.header.frame_id = "world";

for (const auto & knot : my_plan) {
  dsim_msgs::msg::TrajectorySetpoint p;
  p.time_from_start = rclcpp::Duration::from_seconds(knot.t);
  p.position.x = knot.x;  p.position.y = knot.y;  p.position.z = knot.z;
  p.velocity.x = knot.vx; p.velocity.y = knot.vy; p.velocity.z = knot.vz;
  p.acceleration.x = knot.ax; /* ... */
  p.yaw = knot.yaw;
  traj.points.push_back(p);
}
pub->publish(traj);
```

### Rules

1. **Each message replaces the previous trajectory.** Replanning is just
   publishing again — no append, no splice, no cancel.
2. **Points must be sorted** by `time_from_start`.
3. **`header.stamp` anchors the trajectory in time.** Stamp `now` to start
   immediately. A stale stamp makes the vehicle jump into the middle of your plan.
4. **Send derivatives if you have them.** `velocity` and `acceleration` are used
   as feedforward. Zeros still work — the position loop chases — but tracking
   error grows roughly with the square of speed, so fast trajectories will look
   far worse than your planner deserves.
5. **Before the first point and after the last, the vehicle holds position.**
   It does not drift and does not land.
6. With no trajectory ever received, it hovers where it first got state.

## Use sim time

Gazebo publishes `/clock` and every node in the sim runs on it. Your planner
must too, or its stamps are in a different time base than the controller's:

```python
Node(package='my_planner', executable='planner', parameters=[{'use_sim_time': True}])
```

This is the most common reason a planner "publishes fine" but never flies.

## Building against it

Your planner needs only `dsim_msgs`, which depends on nothing beyond
`std_msgs`, `geometry_msgs` and `builtin_interfaces`.

```bash
PLANNER_REPO=/mnt/windows/Users/JHY/code/my_planner \
  docker compose -f docker/compose.yaml up -d
docker compose -f docker/compose.yaml exec sim bash
colcon build --symlink-install
```

Your repo lands at `/ws/planner_src`.

## Running a trial

```bash
ros2 launch dsim_bringup sim.launch.py world:=pillars reference:=none csv:=/ws/logs/run1.csv
# then, in the container:
ros2 run my_planner planner_node --ros-args -p use_sim_time:=true
```

Between trials:

```bash
scripts/reset_pose.sh -4 0 1.0                                   # move the drone
ros2 service call /drone/eval/reset dsim_msgs/srv/ResetRun "{}"  # zero the metrics
```

## Scoring

`/drone/eval/status`:

| Field | Meaning |
|---|---|
| `collided`, `collision_count` | contact on the 0.30 m envelope |
| `min_obstacle_clearance_m` | closest approach minus vehicle radius. Negative = penetrated |
| `tracking_rmse_m`, `tracking_max_m` | how well the controller followed *your* trajectory |
| `path_length_m` | distance actually flown |
| `energy_wh` | relative cost figure, not calibrated watt-hours |
| `elapsed_s` | time since the run started |

**Read `tracking_rmse_m` as a feasibility signal.** A large value usually means
you asked for a trajectory the vehicle cannot fly, not that the controller is
bad. Check the log for `tilt clamped` warnings — that is the controller telling
you the plan exceeded 40° of bank.

The referee knows the obstacle list analytically, from the same source that
generated the world. It is the referee, so it does not share the planner's
blind spots.

## Vehicle limits to respect

Generic quadrotor, from `PARAMS` in `scripts/gen_assets.py`:

| Quantity | Value |
|---|---|
| Mass | 1.5 kg |
| Thrust-to-weight | 2.45 → up to ~14 m/s² vertical accel |
| Max tilt (controller clamp) | 40° → ~8.3 m/s² lateral accel |
| Collision envelope radius | 0.30 m — inflate obstacles by at least this |
| Control rate | 250 Hz, physics step 1 ms |

Edit `PARAMS` and rerun the generator to change any of these; it refuses
parameter sets that cannot hover.

## Not built

- **No obstacle sensing.** The referee knows the obstacles; your planner gets no
  sensor stream. Fine if your planner assumes a known map. If it needs to
  *discover* obstacles, a depth camera has to be added to the model.
- **No `/drone/goal` publisher.** Unclear whether goals should come from config,
  a topic, or your planner choosing them. Trivial once decided.
- **No hardware path.** Low priority by design — see *Scope* in the README.
