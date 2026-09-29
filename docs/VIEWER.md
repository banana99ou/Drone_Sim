# Remote viewer

Watch the drone fly from any device on your tailnet — MacBook, iPad, phone —
with nothing to install. You open a URL.

## Start it

```bash
make viz          # starts the sim + viewer, waits until it is really up, prints the URL
make status       # is it up and publishing?
make stop         # stop everything
```

`make viz` passes options through:

```bash
make viz WORLD=empty REFERENCE=lemniscate RADIUS=2.0
make viz WORLD=fence3d PLAN=plans/fence3d_seed.json   # a space-time scenario
make viz PERIOD=3.5                      # a lap that actually demands bank
```

### PERIOD is the knob that decides whether there is anything to watch

A circle needs `bank = atan(4·pi²·r / (T²·g))`. The old 12 s lap on a 1 m
circle is **1.6 degrees** of bank — the control stack is idling, the four rotor
thrusts are nearly identical, and the overlay correctly shows almost nothing
happening. That is not a viewer bug; it is an accurate picture of a vehicle
being asked for nothing.

| radius | period | speed | bank | what you see |
|---|---|---|---|---|
| 1.0 m | 12 s | 0.52 m/s | 1.6° | flat, arrows nearly equal |
| 1.0 m | 6 s | 1.05 m/s | 6.4° | a visible lean |
| 1.0 m | 3.5 s | 1.80 m/s | 18° | thrust clearly split across the diagonal |
| 2.0 m | 4.5 s | 2.79 m/s | 22° | working hard, still inside the 40° clamp |

`make viz` prints the implied bank angle at launch, so "nothing is happening on
screen" can be checked against what was actually asked for before anyone goes
looking for a bug in the controller.

## Open it

From any tailnet device:

```
http://your-sim-host.your-tailnet.ts.net:8080
```

or by IP, `http://100.64.0.1:8080`. Both are placeholders: `scripts/run_sim.sh`
prints the real URL when it starts, and `tailscale ip -4` gives you the address
on its own. With MagicDNS enabled the hostname works from any device on the
tailnet without remembering the address at all.

The page derives its WebSocket endpoint from whatever URL you loaded, so the
hostname, the IP and a proxied name all work with no config.

### Optional: HTTPS with no port number

`tailscale serve` gives a real certificate and drops the port, at the cost of
needing root once. This stays **private to the tailnet** — it is not `funnel`,
so nothing is published to the internet:

```bash
sudo tailscale serve --bg --https=443 http://127.0.0.1:8080
```

One command, because the page and its data stream share a single port and a
single origin. Then open `https://your-sim-host.your-tailnet.ts.net`.

To undo: `sudo tailscale serve reset`. To stop needing `sudo` for this:
`sudo tailscale set --operator=$USER` once.

## Controls

drag to orbit · wheel or pinch to zoom · shift-drag to pan · **recentre view** ·
**clear trails**.

### Pausing, slowing and resetting the simulation

**pause / resume** freezes the physics. **reset run** puts the vehicle back on
the pad, sends the clock back to zero and starts the referee's metrics again.
The **speed** slider runs the world in slow motion, from 0.05× to 1×.

**There is no fast-forward, and that is a measured limit rather than an
omission.** The world is throttled to real time by `<real_time_factor>` in its
SDF; stepping does not bypass the throttle (ten seconds of simulated time
requested in one message took 10.38 seconds of wall clock), and the only
service that lifts it is the one that deletes gravity — see below. The slider
used to go to 4×, which it could not deliver.

The speed applies on release, not on every drag pixel. Its range is not written
in the page: the control node publishes the range it will accept, and the
slider takes its bounds from that, so the page cannot offer a speed the
simulator will refuse.

The HUD's `simulator` row shows two numbers, "asked" and "real". They differ
whenever the machine cannot keep up, and that is the point: a viewer that
showed only the requested speed would be lying at exactly the moment it
mattered.

### How the speed is actually produced

Not by telling Gazebo a real-time factor. `dsim_simctl` runs the world for a
fraction of each 50 ms tick and pauses it for the rest; the fraction is the
speed. Measured: 0.05× → 0.050×, 0.25× → 0.255×, 0.50× → 0.497×, 0.75× →
0.755×.

The reason it is done this way is `docs/`-worthy in itself. `set_physics`, the
obvious call, makes the world **weightless**: `gz.msgs.Physics` has no gravity
field, gz-sim 8.11.0 assigns gravity from the message regardless, and proto3
reads the absent field as (0, 0, 0). It answers `data: true`, the world SDF
still reports `<gravity>0 0 -9.8</gravity>`, and the IMU keeps reading a clean
1 g — so nothing in this repo noticed while a drone climbed to 41.5 km. Sending
the world the real-time factor it already had, a request that changes nothing,
reproduces it in one second.

`scripts/check_simcontrol.py` is the regression test. It drives pause, speed
and reset against a live sim and asserts, after each one, that the vehicle is
still producing about 14.7 N of thrust — which a weightless one cannot be.

### Pushing the vehicle: gusts

The **gust** controls apply an external force to the airframe, in the world
frame, in newtons. Six buttons (±x, ±y, ±z), a magnitude slider, and a **hold
until cleared** box — unticked, a press lasts 1.5 simulated seconds.

The controller is never told. Nothing publishes "a force was applied"; the loop
sees only the state that results, exactly as it would outdoors. That is the
whole point of pushing the vehicle rather than moving its setpoint.

Watch the **aero** arrow: it is measured from the accelerometer, so the gust
shows up there as an orange arrow of the applied size, and the rotors visibly
split to fight it.

What a steady gust does to *this* controller: the vehicle is thrown about 68 cm
off the plan, and the position loop's integral term then walks it back to
**0.4 cm while the force is still being applied**, holding 4.9 N of integral
against the 5 N gust. Watch the **integral** row in the HUD grow as it does.

That term exists because of this feature. Without it the loop answers a
constant force with a standing offset of exactly `F/kp` and holds it forever —
measured at 82 cm with `ki` set to zero, against the 83 cm the arithmetic
predicts. It cannot help with an error that *turns* with the vehicle, though:
the integral is accumulated in the world frame, so anything rotating averages
to nothing over a lap.

Duration is in SIMULATED seconds, so a gust is the same push at any playback
speed, and it does not expire while the world is paused. The force is capped
(20 N against a 14.7 N weight) and the cap is published in `/sim/state`, so the
slider, the HTTP layer and the node cannot disagree about it. There is no
torque: a gust that spun the airframe would be indistinguishable from a broken
mixer in every plot here.

`scripts/check_simcontrol.py` covers it, and the check that matters reads the
**accelerometer**, not the reply: a gust published to a link that does not
exist still answers HTTP 200 and still shows up in `/sim/state`. Aimed at
`drone::not_a_link` the measured residual stays at 0.37 N against 5.00 N
applied, and only that check notices.

### Frame rate

The HUD shows the page's own render rate. It exists because "it feels slower"
is not something you can act on, and because the viewer's cost is the one thing
a remote user can see directly. The obstacle-course selector follows whatever world the sim
is actually running (written to `web/current.json` at launch) so it cannot draw
a course that is not there; you can still override it manually.

The blue ring on the ground is the 0.30 m collision envelope — the radius your
planner has to keep clear.

## Seeing the control action

Position alone cannot tell you whether a controller is working: a drone
coasting through a gentle arc and a drone fighting for the same arc trace an
identical line. The overlays draw what the stack is *doing*.

**One arrow per rotor**, along body +z, length proportional to that rotor's
thrust, with a white tick at exactly hover thrust (weight / 4). Arrows are
coloured by their deviation from that tick — **blue below hover, orange above**.
A quadrotor holds a bank by splitting thrust across a diagonal, so in a turn two
rotors go orange and two go blue, and the split grows with the manoeuvre. That
split *is* the control action.

**Vectors at the centre of mass:**

| arrow | where it comes from |
|---|---|
| thrust | the realised total from the rotor speeds, along body +z |
| weight | mass × g, straight down |
| aero | **measured**: mass × IMU proper acceleration, minus the rotor thrust |
| velocity | the world-frame velocity the controller resolved |
| torque | the realised body torque |

**And what each loop of the cascade asked for** (dashed, one toggle, because
seeing one without the others tells you a loop is unhappy but not which):

| arrow | what it is |
|---|---|
| cmd position | where the position loop wants the vehicle to be, drawn from where it is — the tracking error, magnified ×10 with the true distance in the label |
| cmd velocity | the velocity being asked for, on the same scale as the actual velocity arrow, so the gap between them is the velocity error |
| cmd tilt | the demanded thrust axis, against the actual body z |

Each one is recovered from the ERROR the controller published (state minus
reference), not re-sampled from the trajectory here. Re-sampling would let the
picture disagree with the numbers the loop actually ran on, and that
disagreement would read as a tracking failure.

Numeric labels carry two decimals. Torque is labelled in **mN·m**: a steady
turn realises about 0.017 N·m, and two decimals of that is one significant
figure that does not move until the demand changes by 40%.

The aero arrow is a measurement, not a model. An accelerometer reads total
non-gravitational force over mass; subtract the thrust the rotors are producing
and what is left is rotor drag, rolling moments and anything the plant does that
the controller does not know about. Re-deriving it from a drag coefficient would
have meant drawing a guess in the same style as a measurement.

All forces share **one** scale (0.05 m per newton), so lengths are directly
comparable — a thrust arrow twice as long as the weight arrow means twice the
weight. The **arrow scale** slider multiplies every length together and never
changes a ratio; auto-normalising to the largest value on screen would draw
hover and a hard bank identically, which is exactly what these overlays exist
to prevent.

Checkboxes toggle each group, and the choices persist per browser.

### The rule: all logic is on the ROS side

The page only draws. Overlay geometry is computed in
`src/dsim_viz/dsim_viz/overlay.py` and arrives as world-space line segments
already in metres, tagged with a `kind` and a `group`; the page projects them,
picks colours from `web/js/palette.js`, filters by checkbox and strokes them.
It never multiplies a mass by an acceleration, never rotates a measured vector
into the world frame, and holds no opinion about what a force is.

That boundary is not tidiness. Physics in a browser is physics you cannot
test: it needs a headless-browser harness to inspect, it duplicates constants
that live in `config/drone.yaml`, and when the viewer disagrees with the
simulator you have two suspects instead of one. On the ROS side it is a pure
function of three messages, with `src/dsim_viz/test/test_overlay.py` asserting
its output against hand-computed values, and eight injected bugs in
`make verify` proving those assertions bite.

Everything the overlay needs travels on `/drone/control_debug` — including the
vehicle's mass, gravity and rotor hub positions — so nothing downstream needs a
copy of the vehicle config, and a rotor's thrust cannot be drawn on the wrong
arm.

## How the data gets there

`dsim_viz` serves both the page and the live state on **one port, one origin**:

| endpoint | what |
|---|---|
| `/` | the viewer (`web/index.html`) |
| `/snapshot` | one JSON object with the latest state — curl-friendly |
| `/state` | `text/event-stream`, ~30 Hz of the same object |

`/snapshot` carries the pose, the referee's status, the raw control step, the
IMU, and the `overlay` block the page draws.

You can check the transport without a browser at all, which is the main reason
it is shaped this way:

```bash
curl -s http://100.64.0.1:8080/snapshot | python3 -m json.tool
curl -N  http://100.64.0.1:8080/state | head -3
```

And you can check the *numbers*, not just the transport:

```bash
make telemetry      # or: python3 scripts/check_telemetry.py http://host:8080
```

That compares quantities produced by different paths, so agreement is evidence:
the per-rotor thrusts must sum to the realised thrust; the world-frame velocity
must have the same magnitude as Gazebo's body-frame twist; and in a steady turn
the thrust must equal `weight / cos(tilt)` — a relation neither the thrust nor
the tilt was computed from. On the current run those agree to 0.15%, and
dropping the `cos(tilt)` term makes the check fail, so it is not vacuous.

The page prefers the event stream and falls back to polling `/snapshot` at
10 Hz. The fallback is driven by a **watchdog**, not only by error events: an
`EventSource` that hangs in CONNECTING never fires `error`, so a
failure-counting fallback waits forever — which is exactly the symptom of a
page that renders the scene with a dead HUD. The server sends a frame every
1/30 s even when nothing changed, so silence for 2.5 s is unambiguous evidence
the stream is not working, whatever its `readyState` claims.

You can also force the fallback to isolate a problem in one reload:

```
http://100.64.0.1:8080/?poll=1
```

If the page works that way and not otherwise, the event stream is the fault and
nothing else is.

### Why not video, and why not a WebSocket

Streaming video of the Gazebo GUI would need a virtual display, a software
OpenGL rasteriser and an H.264 encoder on the sim host, for a blurry laggy
picture that also costs CPU the physics wants. A pose is seven floats; state is
about 5 KB/s and renders crisply at whatever resolution the device has.

The first version used `rosbridge` on a second port. It was replaced because
two ports means two origins (and mixed-content trouble the moment the page is
served over HTTPS), and because a WebSocket upgrade from Chrome to rosbridge
hung at CONNECTING on this host while raw clients sending byte-identical
handshake headers got `101` immediately. Rather than keep chasing that, the
dependency was deleted: SSE is plain HTTP and `curl` can prove it works.

The viewer is dependency-free — no three.js, no CDN, no build step, nothing
installed. It works in Safari and on an iPad. It is split into ES modules only
so the parts that are pure can be tested (`make test-viewer` runs them in
node), and `web/package.json` exists for that reason alone: it is not an npm
project and has no dependencies.

| file | what | tested |
|---|---|---|
| `web/js/vec3.js` | vector / quaternion maths for drawing | node |
| `web/js/shapes.js` | scene geometry as face lists | node |
| `web/js/palette.js` | overlay colours and labels | node |
| `web/js/render.js` | the only file that touches the canvas | — |
| `web/js/telemetry.js` | the event stream and its fallback | — |
| `web/js/app.js` | state, HUD, input, the frame loop | — |

## Cost: the viewer must never slow the physics

It did, and by a lot. With **no browser connected at all**, `viz_server` was
using 62% of a CPU core — more than Gazebo's own 55%. Two causes, both measured
rather than guessed:

| cause | cost | fix |
|---|---|---|
| `use_sim_time: True` on the viewer | ~50% of a core | It is now `False`. That parameter makes rclpy subscribe to `/clock`, which the bridge publishes at the 1 ms physics step — **1000 messages a second** into a Python process, for a clock the viewer never reads. |
| four 250 Hz subscriptions | ~12% per topic | `dsim_eval`'s `viz_relay_node` decimates them to `/drone/viz/*` at 30 Hz in C++. The cost is inside rclpy's ingestion path, *before* any callback runs, so decimating in the viewer saves nothing. |

Result: **62% → 5.3%**, and Gazebo itself dropped from 55% to 50% with the
contention gone. Real-time factor stays at 0.9999.

The full-rate topics are untouched. That matters: `/drone/control_debug` at
250 Hz is what caught a 10 ms once-per-lap saturation event, and decimating the
original would have hidden it. The relay adds a slow copy; it never slows the
source.

## Security

`bind` defaults to `0.0.0.0`, so the port is reachable from your LAN, not only
the tailnet.

Every `GET` is read-only. The **one** write path is `POST /control`, and it can
express exactly four things: `{"paused": bool}`, `{"toggle_pause": true}`,
`{"speed": x}` and `{"reset": true}`. One verb per request. It cannot arm a
vehicle, retune a controller, move the drone, or run a command.

There is no shell anywhere in that path any more. It used to build `gz` command
lines and run them with `subprocess`, and the narrowness claimed for it was not
real: the CLI it reached for could delete the world's gravity, and did. The
endpoint now forwards to one ROS service with four commands defined in
`dsim_msgs/srv/SimControl.srv`, and `dsim_simctl` holds the only connection to
Gazebo. Parsing and validation live in `dsim_viz/simcontrol.py` and are
unit-tested; the accepted speed range comes from the control node rather than
being a third copy of a constant.

Every request to `/control` is logged with its outcome and the client address.
The `GET`s are not — a 30 Hz stream would drown the launch output — but the
write path leaves a record, because the last time it broke a run there was
nothing to read afterwards.

To serve a read-only viewer — the endpoints still exist and answer 403, so the
page can say they are off rather than appear broken:

```bash
ros2 launch dsim_bringup viz.launch.py control:=false
```

To keep it off the LAN entirely, bind it to the Tailscale interface:

```bash
ros2 launch dsim_bringup viz.launch.py bind:=100.64.0.1
BIND=100.64.0.1 bash scripts/run_sim.sh      # the same thing, both launches
```

## Line of sight

For a scenario with stations (only `loiter` today) the viewer draws the station
as a small pad-and-mast and a line from it to the vehicle: green while the
station can see the drone, red while an obstacle is between them. The HUD gains
two rows — the current margin with the obstacle responsible, and the last time
sight was lost.

It is worth its own picture because it is the one failure a drawing of
clearances cannot show: the vehicle can be far from every obstacle and still be
behind one. See [PLANNER.md](PLANNER.md#line-of-sight) for the arithmetic and
what checks it.

## Two launches, and why

The viewer is `viz.launch.py`. The simulator is `sim.launch.py`. They are
started separately and only the second one is ever restarted.

That split exists for the scenario dropdown. Picking a scenario restarts the
simulator, and a server that was part of the launch it restarts would kill
itself mid-reply — leaving the page on a closed socket with no way to say
whether the new run came up, failed, or was never started. So the viewer
outlives every run it shows: the tab stays live across a switch, the HUD keeps
its numbers, and a launch that dies puts the tail of its own log on the page.

The handoff between them is `web/current.json`, written by `sim.launch.py`
**after** it has accepted its arguments. So the viewer reports what is running,
not what was asked for: a launch that refuses its arguments leaves the previous
run named on the page, which is true, where "now flying wall" would be a lie.

`scripts/kill_sim.sh` kills the viewer too — it derives its pattern list from
the install tree and `viz_server` is in it. The one path that must spare the
viewer is the teardown the viewer itself runs, and that is already covered:
the script excludes its own ancestors, and there the viewer is one. Nothing is
special-cased.

## The scenario dropdown

It restarts the simulator with the run you pick. It is not a drawing
preference — the world, the obstacles and the plan all come from whichever run
is actually up.

The list is derived from disk, not written down: every scenario in
`scenarios/` that some plan in `plans/` **declares** is offered, plus the
built-in `hover` and `circle`, which need no planner at all. The server will
only launch a name from that list, and the page builds the dropdown from the
same object — so it cannot offer a run that would then be refused, and cannot
be asked for one it never offered.

Two things the catalogue works out rather than being told:

* **Which plan.** A solved plan beats the straight seed. The seed exists to be
  flown *into* an obstacle — it is the fixture that proves the referee reports
  a hit — so defaulting to it would report the fixture rather than the planner.
  Seeds are still flyable from the command line.
* **Which state source.** A plan whose control points go above the optical
  flow's `max_height_m` is launched `state:=truth`, and the dropdown says why.
  `loiter` flies at 62.5 m: measured tracking error is 48 m on the estimate
  against 2.2 cm on truth. That is the sensors being honest about their
  envelope, not the controller failing.

While a run is coming up the line under the dropdown counts the seconds. If the
launch exits, or produces no telemetry inside 90 s, it says so and shows the
last lines of `logs/sim.log` in the page.

## When it says "disconnected"

The HUD shows the endpoint it tried, next to the status. Almost always the sim
is not running rather than the network being wrong:

```bash
make status     # "NOT publishing" means the nodes are gone even if the page still serves
make viz        # restart both launches
tail -f logs/sim.log
tail -f logs/viz.log
```

Since the viewer outlives the simulator, "the page serves but there is no
drone" is now a normal state with its own label: the line under the scenario
dropdown reads `no simulator running`, and picking a scenario starts one.

The page reconnects by itself once the sim is back, so leave the tab open.

The status line next to the dot is deliberately diagnostic:

| it says | meaning |
|---|---|
| `live · /state` | the event stream is flowing |
| `live · /snapshot (polling)` | the stream failed, the fallback is working |
| `disconnected — /state (no frame for 2.5 s)` | the stream connected and went silent |
| `disconnected — /snapshot (polling)` | even polling is failing: the sim is down |
| `hud error: …` | the page hit a JavaScript exception, and names it |
