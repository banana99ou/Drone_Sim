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
make viz WORLD=pillars RADIUS=1.0        # 1.0 m clears the whole pillar course
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

or by IP, `http://100.64.0.1:8080`. MagicDNS is enabled on this tailnet, so
the hostname works from the Macs (`your-macbook`, `your-laptop`) without
remembering the address.

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

drag to orbit · wheel or pinch to zoom · shift-drag to pan · **reset view** ·
**clear trails**. The obstacle-course selector follows whatever world the sim
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
| attitude cmd | the demanded thrust axis (dashed) next to the actual body z |
| torque | the realised body torque |

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

## Security

`bind` defaults to `0.0.0.0`, so the port is reachable from your LAN, not only
the tailnet. What limits the damage is that the server is **read-only by
construction**: `dsim_viz` exposes only `GET /`, `GET /snapshot` and
`GET /state`. There is no endpoint that writes anything to ROS, so reaching
this port cannot arm a vehicle or retune a controller.

To keep it off the LAN entirely, bind it to the Tailscale interface:

```bash
ros2 launch dsim_bringup sim.launch.py viz:=true bind:=100.64.0.1
```

## When it says "disconnected"

The HUD shows the endpoint it tried, next to the status. Almost always the sim
is not running rather than the network being wrong:

```bash
make status     # "NOT publishing" means the nodes are gone even if the page still serves
make viz        # restart
tail -f logs/sim.log
```

The page reconnects by itself once the sim is back, so leave the tab open.

The status line next to the dot is deliberately diagnostic:

| it says | meaning |
|---|---|
| `live · /state` | the event stream is flowing |
| `live · /snapshot (polling)` | the stream failed, the fallback is working |
| `disconnected — /state (no frame for 2.5 s)` | the stream connected and went silent |
| `disconnected — /snapshot (polling)` | even polling is failing: the sim is down |
| `hud error: …` | the page hit a JavaScript exception, and names it |
