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
```

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

## How the data gets there

`dsim_viz` serves both the page and the live state on **one port, one origin**:

| endpoint | what |
|---|---|
| `/` | the viewer (`web/index.html`) |
| `/snapshot` | one JSON object with the latest state — curl-friendly |
| `/state` | `text/event-stream`, ~30 Hz of the same object |

You can check the transport without a browser at all, which is the main reason
it is shaped this way:

```bash
curl -s http://100.64.0.1:8080/snapshot | python3 -m json.tool
curl -N  http://100.64.0.1:8080/state | head -3
```

The page prefers the event stream and falls back to polling `/snapshot` at
10 Hz if the stream fails four times, so a browser or proxy that dislikes
event-streams still shows the drone.

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

The viewer itself is a single dependency-free HTML file — no three.js, no CDN,
no build step. It works in Safari and on an iPad.

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

The status line next to the dot is deliberately diagnostic: `live` means the
stream is flowing, `disconnected — /state` means the stream is not connecting,
`disconnected — /snapshot (polling)` means it fell back, and `hud error: …`
means the page hit a JavaScript exception and names it.
