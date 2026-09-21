# Drone_Sim

A quadrotor simulator for **testing path planners**. You publish a trajectory,
it flies it and scores the result. The drone is infrastructure you should never
have to touch.

ROS 2 Jazzy + Gazebo Harmonic, in Docker. Your host stays on Ubuntu 22.04 /
Humble — nothing here touches it.

**This models no specific hardware.** The vehicle is a generic 1.5 kg quadrotor
with round, self-consistent numbers chosen to fly well. That is the right
trade for comparing planners against each other; see *Scope* below.

## Run it

One-time (already done on this machine):

```bash
bash scripts/host_setup.sh      # docker group + nvidia-container-toolkit
```

Then:

```bash
make build      # build the image (once, ~10 min)
make up         # start the container
make sim        # build the workspace and fly
```

You should see the drone take off and fly a 2 m circle at 1.5 m altitude.

## Plug in your planner

Full contract in [docs/INTERFACE.md](docs/INTERFACE.md). The short version:

| Direction | Topic | Type |
|---|---|---|
| sim → you | `/drone/truth` | `nav_msgs/Odometry` |
| **you → sim** | `/drone/trajectory` | `dsim_msgs/Trajectory` |
| sim → you | `/drone/eval/status` | `dsim_msgs/FlightStatus` |

Mount your planner repo and build it alongside:

```bash
PLANNER_REPO=/path/to/my_planner docker compose -f docker/compose.yaml up -d
```

## Layout

```
scripts/gen_assets.py       ONE source for the vehicle + worlds. Edit PARAMS here.
config/drone.yaml           generated — what the controller assumes
config/obstacles_*.yaml     generated — what the referee scores against
worlds/                     generated — Gazebo scenes
src/dsim_msgs/              the planner interface
src/dsim_description/       generated — the Gazebo model
src/dsim_control/           SE(3) geometric controller, mixer, test trajectories
src/dsim_eval/              the referee: collision, clearance, tracking, energy
src/dsim_sensors/           simulated rangefinder, optical flow and compass, with noise
src/dsim_estimation/        attitude + velocity from those sensors; what the controller flies on
src/dsim_bringup/           launch, bridge, RViz
scripts/flight_check.sh     headless "does it actually fly" gate
scripts/plot_run.py         CSV -> SVG, no dependencies
scripts/kill_sim.sh         clear leftover sim processes
scripts/run_sim.sh          make viz -- start sim + viewer, verify it is up
scripts/check_telemetry.py  cross-check a running sim's telemetry against physics
scripts/check_estimator.py  the estimate against ground truth, live
src/dsim_viz/               one-port viewer server + the overlay geometry it sends
web/                        the browser viewer (dependency-free, draw-only)
docs/VIEWER.md              how to watch from any tailnet device
docs/reference/             a collaborator's hardware brief. Not used by this sim.
```

The vehicle is described in two places that must agree — the SDF that Gazebo
simulates and the YAML the controller assumes. Both are **generated** from
`PARAMS` in `scripts/gen_assets.py`. If they ever disagreed the controller
would be flying a vehicle that does not exist, and nothing would visibly
break, so `--check` exists to catch drift.

## Common commands

```bash
# built-in trajectories, no planner needed
ros2 launch dsim_bringup sim.launch.py reference:=circle
ros2 launch dsim_bringup sim.launch.py reference:=lemniscate world:=pillars

# your planner drives, metrics to CSV
ros2 launch dsim_bringup sim.launch.py world:=pillars reference:=none csv:=/ws/logs/run1.csv

# headless
ros2 launch dsim_bringup sim.launch.py gui:=false
```

Change the vehicle:

```bash
$EDITOR scripts/gen_assets.py    # edit PARAMS
python3 scripts/gen_assets.py    # model.sdf + drone.yaml regenerate together
```

It refuses to emit a vehicle that cannot hover or has no control margin.

## Seeing it fly

`make viz` streams the drone's state to a browser over your tailnet — that is
the way to watch it, and it needs no display on this machine at all.

**There is no X display on this machine** — the only X sockets are
GDM's greeter sockets, no Xorg/Xwayland is running, and every login is a remote
SSH session. The Gazebo GUI has nothing to render to, so `gui:=true` fails with
"unable to open display". Three ways round it:

| | |
|---|---|
| **Browser viewer over Tailscale** (recommended) | `make viz`, then open `http://your-sim-host.your-tailnet.ts.net:8080` from the MacBook, an iPad, anything on the tailnet. Nothing to install. See [docs/VIEWER.md](docs/VIEWER.md) |
| **Plot a run** | `make fly` then `make plot F=logs/circle.csv` → an SVG you can open from Windows, since this repo lives on the Windows drive |
| **Local desktop** | log into the machine's desktop, then `make sim` shows the live Gazebo GUI |
| **SSH X forwarding** | reconnect with `ssh -X`, then `make sim` |

For the GUI to render at full speed it also needs GPU passthrough, which is
opt-in — see `docker/compose.gpu.yaml`. Without it Gazebo falls back to
software rendering (llvmpipe): visible, but slow. Headless runs never need it.

## Seeing that it is being controlled

Position alone cannot tell you whether the controller is working: a drone
coasting through a gentle arc and one fighting for it trace the same line. The
viewer draws the control action itself — one thrust arrow per rotor with a tick
at hover thrust, coloured blue below and orange above, plus thrust, weight,
**measured** aerodynamic force, velocity, torque and the commanded attitude axis
next to the actual one. In a turn the thrust visibly splits across the diagonal;
that split *is* the control action. See [docs/VIEWER.md](docs/VIEWER.md).

One thing worth knowing before deciding the stack looks idle: a circle needs
`bank = atan(4·pi²·r / (T²·g))`, so a 12 s lap on a 1 m circle is **1.6 degrees**
of bank. Nothing looked like it was happening because nothing was being asked
for. `make viz PERIOD=3.5` asks for 18 degrees, and prints the figure at launch.

All the arithmetic behind those arrows runs on the ROS side
(`src/dsim_viz/dsim_viz/overlay.py`) and reaches the page as line segments in
world metres. The browser only draws. Physics in a browser is physics you
cannot test, and when the viewer disagreed with the simulator you would have two
suspects instead of one.

## Verification

```bash
make test                         # C++ invariants + overlay geometry + viewer maths
make verify                       # mutation check + generated-asset drift (host)
make fly                          # headless: proves it flies AND reports honestly
make telemetry                    # cross-check a sim that is already running
make estimator                    # the estimate vs truth on a sim that is running
```

Measured on this machine, empty world, 42 s runs:

| trajectory | tracking RMSE | worst error | collisions |
|---|---|---|---|
| circle, 2 m radius, 12 s period | **3.2 cm** | 6.5 cm | none |
| lemniscate (figure-eight) | **4.2 cm** | 6.6 cm | none |

Those two were flown on ground truth. The controller now flies on the
sensor-based estimate by default (`state:=est`); `make fly` flies the same
2 m circle at a 6 s lap (13° of bank) either way, and the two are directly
comparable:

| state source, same lap | tracking RMSE | worst error | mean radial / tangential |
|---|---|---|---|
| `state:=est` (default) | **1.4 cm** | 2.9 cm | −0.1 cm / +1.4 cm |
| `state:=truth` | 7.2 cm | 7.4 cm | −5.8 cm / −4.2 cm |

The perfect-state run is the *worse* one, by a constant offset — 6 cm inside
the circle and 4 cm behind, every lap, reproducible to a millimetre across
three runs. Nothing on its control path changed when the estimator was added
(`git diff` on `dsim_control`, the gains and the vehicle is empty), so that
offset was always there at this bank; the earlier table is a 3° lap, where it
is small. It is **not explained yet**. The one hypothesis on the table is the
velocity in `/drone/truth`, which comes from the same pose-differentiating
`OdometryPublisher` that produced the body-rate bug below; a lag there enters
the velocity loop as an inward force proportional to centripetal acceleration,
which is the right direction and roughly the right size. The estimate takes
velocity from the accelerometer, corrected by flow, and shows no such offset.

Independent cross-checks that the plant, mixer and config agree:
mean rotor speed in hover measured **638.4 rad/s** against the generator's
predicted **639**; flown speed 1.037 m/s against the circle's implied
1.047 m/s; flown radius 1.987 m against the commanded 2.0 m.

Cross-checks on the live telemetry, from `scripts/check_telemetry.py` — each
compares numbers produced by *different* paths, so agreement is evidence:

| check | why it is not self-report |
|---|---|
| `sum(rotor_thrust_n) == realised_thrust_n` | one side is the mixer's inverse, the other its forward map |
| `\|velocity_world\| == \|odom twist\|` | same vector, two frames; a rotation cannot change a length |
| `thrust == weight / cos(tilt)` in a steady turn | pure physics — thrust comes from rotor speeds, tilt from the commanded force direction |
| aero residual is small but non-zero | exactly zero would mean the IMU path is dead, not that drag is |
| rotor thrusts are split while turning | four equal rotors in a turn is not quiet, it is impossible |

On a 13° banked lap those agree to 0.15%, and dropping the `cos(tilt)` term
makes the third one fail — so it is not vacuous.

The collision detector is checked in both directions — silent in the empty
world, and it fires in the pillar field at the geometrically predicted moment
(0.489 m from `pillar_c`'s centre, needing 0.60 m).

`make verify` runs two things:

- **`scripts/mutation_check.sh`** — deliberately breaks the mixer and controller
  (flipped yaw torque, swapped roll/pitch, dropped feedforward, inverted
  position error, faked arm length, a rotor swapped in the layout table), the
  overlay geometry (unrotated body vectors, a flipped aero residual, an arrow
  decoupled from its rotor, a doubly-rotated velocity), the sensor models, the
  playback pacer and the restart detector, then fails if the tests do not
  notice. A green suite is only evidence if it would have gone red on a wrong
  implementation.

  Currently **99 C++ + 54 Python + 10 viewer tests pass, 61/61 injected bugs
  caught, 0 skipped.**

  Three rules keep the harness honest, all added after it lied. A mutation whose
  pattern no longer matches the source counts as a **failure**, not a pass:
  that bug went untested, and reporting it green would make this script the
  very thing it exists to catch. A baseline that does not compile **aborts
  the run**, because every mutation is then "caught" by the same build error —
  which is exactly what happened the moment a new source file was added to the
  package and not to the harness's compile line. And every `.cpp` in a package
  must be either compiled by the harness or **declared untestable on the host**,
  so that failure cannot recur quietly.
- **`gen_assets.py --check`** — fails if any generated file drifted from source.

## Scope

**In:** geometric path planning, obstacle clearance, trajectory feasibility,
replanning behaviour, collision counting, comparing planners fairly.

**Sensors:** an IMU with noise derived from a consumer MEMS data sheet (gyro
1.0e-3 rad/s, accel 4.4e-2 m/s² at 250 Hz) plus startup bias and slow thermal
drift; a downward rangefinder (`/drone/tof`, slant range, 1 cm + 1% error,
millimetre quantised, REP 117 out-of-range); an optical-flow sensor
(`/drone/optical_flow`, integrated angle plus gyro terms, PMW3901 style); and
a magnetometer (`/drone/mag`, body-frame field with 0.5 µT noise and an
optional hard-iron offset — the only yaw reference there is, since there is
no GPS and no lidar). Ground truth on `/drone/truth` stays clean, so a planner
can be developed against either.

`make sensors` checks them. The IMU noise checks run in **every** regime: they
estimate σ from consecutive-sample differences (`std(diff)/√2` for white noise,
while real motion is smooth at 250 Hz), so they are not confined to hover — an
earlier version only ran them when nearly still, which silently removed the
gate from `make fly`. Two files agreeing that noise exists is not evidence the
simulator applies it; zeroing the stddev fields makes these fail at ratio 0.01.

In flight it also checks the sensors against ground truth: the rangefinder's
excess over altitude against the geometric `alt·(sec θ − 1)` (71.8 mm measured
vs 71.3 mm predicted at 17.3° of bank — a sensor returning plain altitude fails
this), and the flow sensor's reconstructed velocity to a median **0.027 m/s at
1.70 m/s** against a limit derived from the noise model.

**State estimation, partly in.** By default the controller flies on
`/drone/state_est` from `dsim_estimation`: attitude from a Mahony-style
complementary filter (gyro integrated, roll/pitch pulled toward the
accelerometer's gravity, yaw toward the compass), velocity from the
accelerometer integrated in the world frame and pulled toward optical flow
horizontally and the rangefinder vertically, body rate straight from the gyro.
**Position is still ground truth**, behind a `position_source` parameter that
accepts nothing else yet — that is the seam where a SLAM/VIO emulator plugs in.
Measured live on the 18° lap: 1.0° of tilt error (the analytic value for the
accelerometer correction in a coordinated turn is 0.985°), 0.1° of yaw, 3 cm/s
of velocity. Details and known weaknesses in [docs/INTERFACE.md](docs/INTERFACE.md)
under *State estimate*; `state:=truth` flies on perfect state for comparison.

**Out:** position estimation, motor identification, matching a real airframe's
numbers. The controller's model of the vehicle is exactly right and its
position is exact, which is why tracking is as good as it is: with an exact
model and acceleration feedforward, the feedback terms have little left to do.
The stack is a two-level cascade (position/velocity → attitude/body-rate →
mixer) and it is **PD, not PID** — there are no integrators, because with no
model error there is no steady-state error for one to remove.

### A bug worth knowing about, because it was invisible

The controller used to take its body rate from `/drone/truth`'s `twist.angular`,
which Gazebo's `OdometryPublisher` produces by differentiating the pose. That
plugin is built for wheeled robots. A quaternion and its negation are the same
rotation, so once per revolution the representation flipped sign — pose and yaw
stayed perfectly smooth — and the differentiated angular velocity jumped from
1.8 to **626 rad/s**. The controller believed it, demanded **111 N·m** against
an airframe that can produce 2.55, and slammed two rotors to full and two to
zero for ~10 ms, once per lap.

Nothing looked wrong. The flight was smooth, the referee's RMSE barely moved,
and every test passed. It surfaced only because the force overlays put a number
on the commanded torque and the number was exactly the arithmetic saturation
limit.

Body rates now come from the gyro, and `BodyRateSource` rejects any sample a
1.5 kg quadrotor could not physically produce. Both are tested, and the
saturation is gone: 0 events in 7500 control steps where there used to be 22.

Getting a planner onto real hardware later is a **nice-to-have, low priority**,
and the seam is deliberately shallow: your planner emits world-frame
trajectories with derivatives, which is what any flight stack wants. Matching a
specific vehicle's contract is a conversation with whoever owns that flight
stack — this sim adapts to them, not the reverse. See
[docs/reference/ROS2_SIM_BRIEF.md](docs/reference/ROS2_SIM_BRIEF.md) for one
such contract; nothing in this simulator implements it.
