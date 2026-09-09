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
| sim → you | `/drone/odom` | `nav_msgs/Odometry` |
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
src/dsim_bringup/           launch, bridge, RViz
scripts/flight_check.sh     headless "does it actually fly" gate
scripts/plot_run.py         CSV -> SVG, no dependencies
scripts/kill_sim.sh         clear leftover sim processes
scripts/run_sim.sh          make viz -- start sim + viewer, verify it is up
web/index.html              the browser viewer (dependency-free)
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

## Verification

```bash
make verify                       # host-side, no container needed
colcon test --packages-select dsim_control    # in the container
```

Measured on this machine, empty world, 42 s runs:

| trajectory | tracking RMSE | worst error | collisions |
|---|---|---|---|
| circle, 2 m radius, 12 s period | **3.2 cm** | 6.5 cm | none |
| lemniscate (figure-eight) | **4.2 cm** | 6.6 cm | none |

Independent cross-checks that the plant, mixer and config agree:
mean rotor speed in hover measured **638.4 rad/s** against the generator's
predicted **639**; flown speed 1.037 m/s against the circle's implied
1.047 m/s; flown radius 1.987 m against the commanded 2.0 m.

The collision detector is checked in both directions — silent in the empty
world, and it fires in the pillar field at the geometrically predicted moment
(0.489 m from `pillar_c`'s centre, needing 0.60 m).

`make verify` runs two things:

- **`scripts/mutation_check.sh`** — deliberately breaks the mixer and controller
  (flipped yaw torque, swapped roll/pitch, dropped feedforward, inverted
  position error, faked arm length) and fails if the tests do not notice.
  A green suite is only evidence if it would have gone red on a wrong
  implementation. Currently 17 tests pass, 7/7 injected bugs caught.
- **`gen_assets.py --check`** — fails if any generated file drifted from source.

## Scope

**In:** geometric path planning, obstacle clearance, trajectory feasibility,
replanning behaviour, collision counting, comparing planners fairly.

**Out:** sensor noise, state estimation, motor identification, matching a real
airframe's numbers. There is no noise model — the controller and the referee
both see ground truth.

Getting a planner onto real hardware later is a **nice-to-have, low priority**,
and the seam is deliberately shallow: your planner emits world-frame
trajectories with derivatives, which is what any flight stack wants. Matching a
specific vehicle's contract is a conversation with whoever owns that flight
stack — this sim adapts to them, not the reverse. See
[docs/reference/ROS2_SIM_BRIEF.md](docs/reference/ROS2_SIM_BRIEF.md) for one
such contract; nothing in this simulator implements it.
