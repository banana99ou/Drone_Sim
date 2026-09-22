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
| sim → you | `/drone/mag` | `sensor_msgs/msg/MagneticField` | 50 Hz |
| sim → you | `/drone/state_est` | `nav_msgs/msg/Odometry` | 250 Hz |
| sim → you | `/drone/estimator_debug` | `dsim_msgs/msg/EstimatorDebug` | 250 Hz |
| sim → you | `/sim/state` | `dsim_msgs/msg/SimState` | 5 Hz |
| **you → sim** | `/sim/control` (service) | `dsim_msgs/srv/SimControl` | on demand |

`/sim/control` also carries **gusts**: an external force on the airframe, in
newtons, world frame, applied by the simulator and never announced to anything
downstream. A planner subscribing to `/drone/truth` sees only the state that
results, which is the point -- it is the one disturbance in this simulator that
is not in any model your planner could hold. `/sim/state` reports the force
currently applied, so a run can be labelled with the disturbance it met.

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

* **`integral_force_n` is the force the loop is applying that your trajectory
  does not explain.** The position loop has an integral term, so a disturbance
  the plan knows nothing about -- a gust, in this simulator -- shows up here as
  a steady force rather than as a permanent tracking error. A planner watching
  it can tell "my trajectory is wrong" from "something is pushing the vehicle".
  `integral_held` means anti-windup has frozen it because the demand is already
  clamped, which is a stronger statement than `tilt_clamped` alone: the loop
  has given up asking for more.
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

All three are simplified, and the simplifications are stated so you know where
the model stops being usable.

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

**`/drone/mag`** — a three-axis magnetometer, the compass in a consumer IMU.
`sensor_msgs/MagneticField` in **tesla**, in the body frame
(`frame_id: base_link`), at 50 Hz. It reports the Earth's field as the body
sees it: `B_body = R^T · field_world + hard_iron + noise`, where `R` is the
true attitude. The world field is one fixed vector, **+x is magnetic north**
with the declination already folded in (there is no separate true north), z
is up, so the vertical component is negative — the default
`[30, 0, -40] µT` dips 53°, roughly Korea. That vertical component is most of
the field, and under bank it leaks into the horizontal axes: at 20° of roll
an uncompensated compass reads 24° while the vehicle points north. Level the
reading with roll and pitch before taking a heading from it. Noise is 0.5 µT
per axis, about **one degree of heading** in a 30 µT horizontal field, and
`magnetic_field_covariance` carries exactly that and nothing else: a
hard-iron offset — a fixed body-frame bias of magnitude `hard_iron_t` in a
seeded random direction, the field of the airframe's own magnetised parts —
is not random, so no covariance can describe it, and an estimator that
trusts the matrix will be confidently wrong by it. It ships at **zero**, an
ideal compass; set `hard_iron_t` in `scripts/gen_assets.py` to make
calibration the estimator's problem. It is **not a Gazebo sensor**: like the
rangefinder and the flow it is synthesised from the truth attitude in
`dsim_sensors`, so the model is a pure function the unit tests pin and the
mutation harness breaks on purpose, and so the hard iron exists at all —
Gazebo's magnetometer offers Gaussian noise and nothing else. No soft iron,
no motor-current field. `scripts/check_sensors.py` measures the live noise
against `noise_t` in every regime and, on a lap, levels each reading with the
true roll and pitch and requires the heading to match the true yaw to a
limit derived from `noise_t / |B_horizontal|` — measured median **0.66°** on
the 3.5 s demo lap against a 2.3° limit, where a sensor reporting the world
field unrotated errs by 94°. Its noise estimate uses second differences, not
first: at 50 Hz with the yaw following the path, the body field moves ~0.5 µT
per sample on that lap, the size of the noise, so a first-difference estimate
passes a noiseless compass. Dropping the noise from the model fails the check
at ratio 0.02.

`noise_seed:=1` by default gives a repeatable starting point and a fixed noise
distribution. It does **not** promise bit-identical streams between runs: one
generator feeds both sensor timers, so which draw lands in which reading
depends on callback interleaving. Use `noise_seed:=0` for a fresh sequence.

## State estimate

`/drone/state_est` is what the controller flies on by default. It is shaped
exactly like `/drone/truth` — `nav_msgs/Odometry`, `frame_id: world`,
`child_frame_id: base_link`, twist in the **body** frame per REP-145 — so the
controller consumes it through a topic remap and nothing in `dsim_control`
knows which of the two it is reading. That is the point: a run on the
estimate and a run on truth differ in one launch argument and nothing else.

| Field | Comes from | Notes |
|---|---|---|
| `pose.orientation` | gyro integrated at 250 Hz; roll/pitch pulled to the accelerometer's gravity direction, yaw to the magnetometer's tilt-compensated heading | Mahony-style complementary filter, `src/dsim_estimation/include/dsim_estimation/attitude_filter.hpp` |
| `twist.linear` (body) | accelerometer rotated with the estimated attitude, gravity removed, integrated; pinned horizontally by the optical flow (`flow_tau_s`) and vertically by the rangefinder through a second-order altitude / climb-rate pair | `velocity_filter.hpp` |
| `twist.angular` | the raw gyro sample | the controller does not read it here — it takes rates from `/drone/imu` and gates them itself |
| `pose.position` | **ground truth, copied** | no position sensor exists yet; see below |

**Position is still truth**, by decision. The simulator has no position
sensor, and inventing one — a GPS, a SLAM or VIO emulator with its own drift
model — is separate work with its own error budget. The estimator's
`position_source` parameter is the seam it plugs into; its only accepted
value today is `truth`, and anything else refuses to start rather than
falling back. Until then the position loop closes on perfect position, and
the estimate is exercised where it matters most to a quadrotor: the attitude
and velocity loops, which are the fast ones.

**`state:=est|truth`** on `sim.launch.py` chooses, default `est`. Only the
controller's subscription is remapped; the referee, the sensors and the
viewer always see truth, and the estimator itself reads truth for position.
`state:=est` with `sensors:=false` is refused at launch — a controller whose
state source never publishes sits on the ground with its motors cut, which
looks like a quiet, healthy sim until someone asks why it never took off.

**`/drone/estimator_debug`** reports every step: the attitude and gyro-bias
estimates, the fused velocity, the last flow-derived world velocity, the
last rangefinder-derived altitude and the climb rate, the mag heading, and
a flag per sensor saying whether its correction was applied on that step.
`make estimator` (`scripts/check_estimator.py`) reads it and `/drone/truth`
over 20 s and asserts the errors against bounds derived from the configured
sensor noise and the gains — and also asserts the errors are **not zero**
and are **correlated with the sensor noise** reconstructed independently
from `/drone/mag` and `/drone/optical_flow`, because an estimator that
copied truth would pass every bound.

**Known limitations**, each measured rather than assumed:

* **Banked turns.** The accelerometer cannot separate gravity from the
  vehicle's own acceleration; in a coordinated turn it reports "up" along
  the thrust axis, and the roll/pitch correction pulls the estimate towards
  level for as long as the turn lasts. The gain is low (`kp_accel` 0.1 /s)
  so this costs about **1° of tilt on the 3.5 s demo lap** (steady state
  `kp·sin(bank)/√(kp²+Ω²)`, up to twice that during the first ten seconds
  of a turn), and it is why there is **no integral gain on the
  accelerometer**: the turn's error is constant in the body frame,
  indistinguishable from a gyro bias, and any integrator winds up on it at
  about half a degree per lap without bound. The 2e-4 rad/s gyro bias the
  integrator would have removed costs 0.11° through `kp_accel`, below the
  accelerometer's own bias floor.
* **Yaw bias from hard iron.** A body-frame magnetic offset turns with the
  vehicle, so no single reading can tell it from the Earth's field; the
  estimator does not calibrate it, and it shows up as a constant heading
  error of `hard_iron_t / |B_horizontal|`. It ships at zero. Without a mag
  at all (no `/drone/mag` within `mag_timeout_s`, or no field configured),
  yaw starts at zero and integrates the gyro: about 0.7°/min of drift from
  the bias, plus a slow leak from the accelerometer correction while banked.
  The node warns once at startup and the debug message says so on every step.
* **Flow over a flat floor only.** The optical flow is a velocity only
  because the rangefinder supplies a height, and both assume the ground
  plane at z = 0: over a pillar top or a slope the height is wrong and so is
  the velocity, and neither sensor can tell. The vehicle flies at 1.5 m and
  the pillars are 3 m, so it never passes over one in the shipped course.
* **No accelerometer bias state.** A 0.02 m/s² bias becomes a steady
  velocity offset of `bias × flow_tau_s` horizontally (4 mm/s) and
  `bias × 2ζ/ω` vertically (14 mm/s) — below what the sensors themselves
  contribute, so not worth a third filter state yet.

Measured live on this machine with `make estimator`, 20 s windows, shipped
noise and gains (the script prints each bound's derivation next to the
number):

| | hover, 1.5 m | 1 m circle, 3.5 s lap (18° bank, 1.8 m/s) |
|---|---|---|
| tilt (roll/pitch) error RMS | **0.033°** | **1.00°** — the banked-turn figure above, to the decimal |
| yaw error RMS / mean | 0.070° / +0.05° | 0.106° / +0.02° |
| velocity error RMS x / y / z | 6 / 8 / 9 mm/s | 29 / 26 / 11 mm/s |
| `/drone/state_est` rate | 250.1 Hz, stamps equal to the IMU's | 250.1 Hz |
| referee tracking RMSE, flying on the estimate | — | 1.3 cm (r = 2 m, 6 s lap: 2.6 cm) |

The response correlations — how much each mag and flow sample moves the
estimate — read 0.99 in both regimes. Setting `flow_tau_s` to 1000 (flow
ignored) fails the velocity checks at 0.87 m/s; `kp_accel` at 5 puts the
vehicle on the floor; `kp_mag` at 0 (mag not fused) reports the gyro-only
branch with the yaw error at 0.31° and drifting.

One consequence for existing tooling: `scripts/check_telemetry.py` asserts
`|velocity_world| == |odom twist|` between the controller's telemetry and
`/drone/truth`. With `state:=est` the controller reports the *estimated*
velocity, so that identity now holds only to the estimation error (measured
2.4–5.7 cm/s in the turn against a 4 mm/s tolerance), and `make fly` fails
on it while the flight itself passes.

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
