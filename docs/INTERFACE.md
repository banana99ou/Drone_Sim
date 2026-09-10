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
| sim → you | `/drone/tof` | `sensor_msgs/msg/Range` | 30 Hz |
| sim → you | `/drone/optical_flow` | `dsim_msgs/msg/OpticalFlow` | 50 Hz |
| sim → you | `/sim/state` | `dsim_msgs/msg/SimState` | 5 Hz |
| **you → sim** | `/sim/control` (service) | `dsim_msgs/srv/SimControl` | on demand |

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

### The onboard sensors

Both are simplified, and the simplifications are stated so you know where the
model stops being usable.

**`/drone/tof`** — a downward single-beam rangefinder, VL53L1X class. It
reports the **slant range**, not the altitude: a tilted vehicle's downward beam
travels `1/cos(tilt)` further. Error is `0.01 m + 1% of range`, quantised to
whole millimetres. Out of range follows REP 117 — `+inf` for too far, `-inf`
for too close, never a clamped limit, so saturation cannot be mistaken for a
real reading. It sees only the flat ground plane, so it would report the floor
straight through a pillar; the vehicle flies at 1.5 m and the pillars are 3 m,
so that case does not arise in the shipped course.

**`/drone/optical_flow`** — a PMW3901-class module. It reports how far the
image moved as an **angle**, not a velocity, because that is what the hardware
measures: turning it into a velocity needs a range, which is why
`ground_distance_m` travels with it and why that field carries the
rangefinder's noisy measurement rather than the true altitude. Raw flow and the
gyro integral over the same interval are both published so you can subtract
them, exactly as the real part expects; a pre-compensated value would hide the
timing mismatch that pairing is famous for. `quality == 0` means **no
measurement**, not a weak one.

```
v_body_x ≈ (integrated_y - integrated_ygyro) / integration_time_s * ground_distance_m
v_body_y ≈ -(integrated_x - integrated_xgyro) / integration_time_s * ground_distance_m
```

On the 1.7 m/s demo lap that reconstruction matches ground truth to a median
**0.027 m/s in body x and 0.021 m/s in body y**. `scripts/check_sensors.py`
asserts it against a limit derived from the error model —
`3 × 0.674 × noise_rad_s × height`, about 0.064 m/s — not against a fraction of
the speed, because the reconstruction error does not depend on speed.

That check also covers the range: a wrong `ground_distance_m` scales the
reconstructed velocity proportionally, so reporting twice the true height fails
it.

`noise_seed:=1` by default gives a repeatable starting point and a fixed noise
distribution. It does **not** promise bit-identical streams between runs: one
generator feeds both sensor timers, so which draw lands in which reading
depends on callback interleaving. Use `noise_seed:=0` for a fresh sequence.

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

Between trials, one call does all of it — vehicle home, clock to zero, metrics
cleared:

```bash
ros2 service call /sim/control dsim_msgs/srv/SimControl "{command: 3}"   # RESET
```

Nothing has to be told about that reset. Every node that accumulates watches
the clock instead, because simulated time going backwards is the one signal
common to a reset from here, from the Gazebo GUI, or from a terminal — see
`src/dsim_time/include/dsim_time/sim_epoch.hpp`. So this works for your nodes
too: **if your planner integrates anything across time, watch for the clock
going backwards and start again.** Before this existed, the referee reported an
RMSE of 1118 m for a vehicle tracking a circle to 5 cm, because `elapsed_s`
followed the clock back to zero and the sum of squares did not.

The narrower tools are still there when you want only one of the effects:

```bash
scripts/reset_pose.sh -4 0 1.0                                   # move the drone
ros2 service call /drone/eval/reset dsim_msgs/srv/ResetRun "{}"  # zero the metrics
```

`/sim/control` also carries pause (`command: 1`, with `paused`) and playback
speed (`command: 2`, with `speed`, from 0.05 to 1.0). There is no
faster-than-real-time: the world is throttled to real time by its SDF, and the
only Gazebo service that lifts that throttle also deletes the world's gravity.
See `src/dsim_simctl/include/dsim_simctl/pacer.hpp`.

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
