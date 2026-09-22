"""One-port viewer server: static files + live state over Server-Sent Events.

Replaces rosbridge + a separate static file server. The reason is not taste:

  * Two ports means two origins. A page on :8080 opening a WebSocket to :9090
    is cross-origin, and once the page is served over HTTPS (tailscale serve)
    the browser refuses a plain ws:// socket as mixed content -- so it needed a
    second TLS port and more moving parts.
  * A WebSocket upgrade from Chrome to rosbridge hung at CONNECTING on this
    host while raw clients sending byte-identical handshake headers got 101
    immediately. Undiagnosed, and not worth diagnosing when the dependency can
    be deleted instead.
  * SSE is plain HTTP: it can be verified with `curl`, with no browser in the
    loop at all. A transport you can debug with curl is worth a lot more than
    one that needs a headless-browser harness to inspect.

What is streamed is not only the pose. The control step (/drone/control_debug)
and the IMU come along too, because position alone cannot tell you whether the
stack is working: a drone coasting through a gentle arc and a drone fighting for
it trace the same line. The per-rotor thrusts, the demanded wrench and the
measured proper acceleration are what let the viewer draw the difference.

Those raw messages are turned into ready-to-draw geometry HERE, by overlay.py,
not in the page. All logic lives on the ROS side; the browser only draws. See
the module docstring in overlay.py for why that boundary is where it is.

COST DISCIPLINE. The control step and the IMU arrive at 250 Hz; the stream goes
out at 30. Converting every message into JSON-shaped dicts as it arrived cost
500 dict-builds a second and made this process the heaviest thing on the
machine -- 62% of a core, more than Gazebo's own 55% -- for work that was
overwritten before anyone saw it. So the callbacks now do the cheapest possible
thing: keep a reference to the latest message. All conversion happens in
snapshot(), memoised on a counter, so it runs once per frame no matter how many
browsers are watching. A viewer must never be the reason the physics slows
down.

Endpoints, all on one port and one origin:

    /              web/index.html and friends (static)
    /snapshot      one JSON object with the latest state  (curl-friendly)
    /state         text/event-stream, ~30 Hz of the same object
    /control       POST: pause, playback speed, reset, gust  (see simcontrol.py)
    /solve         POST: re-solve the running scenario         (see solver.py)

Almost read-only. Every GET is. There are two write paths, kept apart on
purpose: /control reaches the simulator, /solve reaches the planner and
nothing else. Widening /control to run the solver would have falsified the
paragraph below while leaving it on the page. It forwards
to one ROS service with four commands -- pause, toggle pause, speed, reset --
and can express nothing else. It cannot arm a vehicle, retune a controller,
move the drone or run a command. That narrowness is the point: this port is
reachable from the whole tailnet. With `control:=false` the endpoint still
exists and answers 403 -- it is disabled, not removed.

The narrowness claim used to be made about a version of this endpoint that
could, through `gz service -s set_physics`, delete the world's gravity and
throw the vehicle four kilometres. So: every request to /control is logged,
with its outcome. The GETs are not -- a 30 Hz stream would drown the launch
output -- but the write path leaves a record, because the last time it broke a
run there was nothing to read afterwards and the sequence of events had to be
reconstructed from the spacing of unrelated INFO lines.
"""
import json
import math
import threading
import time
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu

from dsim_msgs.msg import ClearanceReport, ControlDebug, FlightStatus, TrajectorySetpoint
from nav_msgs.msg import Path

from . import overlay
from .simcontrol import UNKNOWN, ControlError, SimControlClient
from .solver import SolveError, Solver


def _v3(v):
    """geometry_msgs/Vector3 (or anything with x/y/z) -> [x, y, z]."""
    return [v.x, v.y, v.z]


class State:
    """Latest messages, shared between the ROS thread and the HTTP threads.

    Holds ROS message OBJECTS, not converted dicts. Conversion is expensive
    relative to how often anyone looks: see the cost note in the module
    docstring.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.seq = 0                # bumped by every message of any kind
        self.odom = None
        # The Odometry the CONTROLLER flew on: /drone/truth or /drone/state_est,
        # whichever state:= selected. Kept apart from odom, which is always
        # truth, so the page draws the vehicle where it is while the telemetry
        # check compares the controller against what it was given.
        self.consumed = None
        self.setpoint = None
        self.status = None
        self.control = None
        self.imu = None
        # The referee's obstacle report: where the (possibly moving) obstacles
        # are, and every hit so far with its position. Slow at the source.
        self.clearance = None
        # The planner's whole path, latched: drawn before it is flown.
        self.plan_path = None
        # The simulator's own state is READ THROUGH, not copied in. It arrives
        # on its own topic at its own rate, and a second copy here would be one
        # more thing that can go stale: the first version of this rewrite kept
        # a copy, forgot to refresh it, and the viewer reported "waiting for
        # the simulation control node" while the node published happily.
        self.sim_source = lambda: dict(UNKNOWN)
        # Read through like sim_source, and for the same reason: the plan on
        # disk changes without any message arriving here.
        self.solver_source = lambda: None
        self._cache = None
        self._cache_seq = -1

    def _bump(self):
        self.seq += 1

    def snapshot(self):
        """The latest state as JSON-ready data, built at most once per change.

        Several browsers polling at 30 Hz each would otherwise repeat identical
        work; the memo makes the cost independent of how many are watching.
        """
        with self.lock:
            if self._cache_seq == self.seq:
                # The memo holds vehicle state; the simulator's pause/speed
                # arrives on its own topic, so it is read fresh here.
                return dict(self._cache, sim=self.sim_source(),
                            solver=self.solver_source())
            seq, odom, sp = self.seq, self.odom, self.setpoint
            status, control, imu = self.status, self.control, self.imu
            consumed = self.consumed
            clearance, plan_path = self.clearance, self.plan_path

        # Built outside the lock: message objects are not mutated after
        # publication, so reading them here cannot tear, and holding the lock
        # through the conversion would stall the 250 Hz subscriptions.
        snap = {
            "seq": seq,
            "t": _stamp_seconds(odom) if odom else 0.0,
            "pose": _pose_of(odom),
            "vel": _twist_linear_of(odom),
            "state_vel": _twist_linear_of(consumed),
            "setpoint": _setpoint_of(sp),
            "status": _status_of(status),
            "control": _control_of(control),
            "imu": _imu_of(imu),
            "clearance": _clearance_of(clearance),
            "plan_path": _path_of(plan_path),
        }
        snap["overlay"] = overlay.build(snap["pose"], snap["control"], snap["imu"])

        with self.lock:
            # A message may have landed while we were converting; keeping the
            # newer seq would make this snapshot look fresher than it is, so
            # the memo is only valid for the seq it was built from.
            if self._cache_seq < seq:
                self._cache, self._cache_seq = snap, seq
            snap = dict(snap, sim=self.sim_source(), solver=self.solver_source())
        return snap


def _stamp_seconds(m):
    return m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec


def _pose_of(m):
    if m is None:
        return None
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return {"p": [p.x, p.y, p.z], "q": [q.w, q.x, q.y, q.z]}


def _twist_linear_of(m):
    if m is None:
        return None
    return _v3(m.twist.twist.linear)


def _setpoint_of(m):
    if m is None:
        return None
    return [m.position.x, m.position.y, m.position.z]


def _status_of(m):
    if m is None:
        return None
    clearance = m.min_obstacle_clearance_m
    return {
        "collided": bool(m.collided),
        "collision_count": int(m.collision_count),
        "tracking_error_m": m.tracking_error_m,
        "tracking_rmse_m": m.tracking_rmse_m,
        # Centimetres computed here, not in the page. The browser only draws:
        # converting a measured physical quantity is exactly the boundary this
        # project keeps, and tilt_deg is already converted for the same reason.
        "tracking_error_cm": 100.0 * m.tracking_error_m,
        "tracking_rmse_cm": 100.0 * m.tracking_rmse_m,
        "tracking_max_m": m.tracking_max_m,
        # JSON has no NaN; send null so the page shows "n/a" rather than a
        # fake number.
        "min_obstacle_clearance_m": None if math.isnan(clearance) else clearance,
        "path_length_m": m.path_length_m,
        "elapsed_s": m.elapsed_s,
        "energy_wh": m.energy_wh,
    }


def _finite_or_none(v):
    return None if (v is None or math.isnan(v) or math.isinf(v)) else v


def _clearance_of(m):
    """The referee's report, field for field. Infinities and NaN become null:
    JSON has neither, and the page must show 'n/a', not a made-up number."""
    if m is None:
        return None
    return {
        "scenario_time_s": m.scenario_time_s,
        # The obstacle FIELD, live. This is the only account of where anything
        # is: they are not Gazebo bodies, because two scenarios move theirs
        # along cubics and three switch them off partway through.
        "obstacles": [
            {"name": n, "pos": [p.x, p.y, p.z], "r": r, "active": bool(a), "type": ty}
            for n, p, r, a, ty in zip(m.names, m.positions, m.radii, m.active, m.types)
        ],
        "scenario": m.scenario,
        "duration_s": m.scenario_duration_s,
        "clearance_m": _finite_or_none(m.clearance_m),
        "nearest": m.nearest,
        "min_clearance_m": _finite_or_none(m.min_clearance_m),
        "hits": [
            {"pos": [p.x, p.y, p.z], "with": w, "t": t, "depth_m": d}
            for p, w, t, d in zip(m.hit_positions, m.hit_with, m.hit_times_s, m.hit_depths_m)
        ],
    }


def _path_of(m):
    if m is None:
        return None
    return [[ps.pose.position.x, ps.pose.position.y, ps.pose.position.z] for ps in m.poses]


def _control_of(m):
    """Passed through field for field, with no arithmetic.

    Any force the viewer draws should be traceable to one number the
    controller published; a "helpful" conversion here would be a second,
    silent opinion about the vehicle's physics.
    """
    if m is None:
        return None
    return {
        "armed": bool(m.armed),
        "mass_kg": m.mass_kg,
        "gravity_m_s2": m.gravity_m_s2,
        "max_rotor_thrust_n": m.max_rotor_thrust_n,
        "thrust_n": m.thrust_n,
        "torque_nm": _v3(m.torque_nm),
        "realised_thrust_n": m.realised_thrust_n,
        "realised_torque_nm": _v3(m.realised_torque_nm),
        "saturated": bool(m.saturated),
        "rotor_position": [_v3(v) for v in m.rotor_position],
        "rotor_thrust_n": list(m.rotor_thrust_n),
        "rotor_speed_rad_s": list(m.rotor_speed_rad_s),
        "velocity_world": _v3(m.velocity_world),
        "position_error": _v3(m.position_error),
        "velocity_error": _v3(m.velocity_error),
        "attitude_error": _v3(m.attitude_error),
        "body_rate_error": _v3(m.body_rate_error),
        "desired_force": _v3(m.desired_force),
        "commanded_tilt_rad": m.commanded_tilt_rad,
        "tilt_clamped": bool(m.tilt_clamped),
        "integral_force_n": _v3(m.integral_force_n),
        "integral_held": bool(m.integral_held),
    }


def _imu_of(m):
    """Proper acceleration in the BODY frame: what an accelerometer feels.

    It includes the reaction to gravity (a resting vehicle reads +9.81 on z)
    and excludes gravity itself, so mass times this is the total
    non-gravitational force on the airframe -- thrust plus everything
    aerodynamic. That is how the viewer gets a MEASURED drag arrow instead of
    one recomputed from a drag coefficient it would have had to duplicate.
    """
    if m is None:
        return None
    return {
        "accel_body": _v3(m.linear_acceleration),
        "rate_body": _v3(m.angular_velocity),
    }


class VizNode(Node):
    def __init__(self, state):
        super().__init__("dsim_viz")
        self.state = state
        # The /drone/viz/* topics are the SAME messages, decimated to ~30 Hz by
        # dsim_eval's viz_relay_node. Subscribing to the full-rate originals
        # from Python cost about 12% of a CPU core per 250 Hz topic -- four of
        # them had this process using more CPU than Gazebo, with no browser
        # even connected. The cost is inside rclpy's ingestion path, before any
        # callback runs, so it cannot be avoided by decimating here.
        self.create_subscription(Odometry, "/drone/viz/truth", self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "/drone/viz/state", self.on_consumed,
                                 qos_profile_sensor_data)
        self.create_subscription(TrajectorySetpoint, "/drone/viz/setpoint",
                                 self.on_setpoint, qos_profile_sensor_data)
        self.create_subscription(ControlDebug, "/drone/viz/control_debug",
                                 self.on_control, qos_profile_sensor_data)
        self.create_subscription(Imu, "/drone/viz/imu",
                                 self.on_imu, qos_profile_sensor_data)
        # Already slow at the source, so it comes straight from the referee.
        self.create_subscription(FlightStatus, "/drone/eval/status",
                                 self.on_status, 10)
        self.create_subscription(ClearanceReport, "/drone/eval/clearance",
                                 self.on_clearance, 10)
        # Latched by the publisher; the subscription must be transient-local
        # too or a page opened after the bridge started never sees the path.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, "/drone/plan_path", self.on_plan_path, latched)

    # Every callback does the same cheap thing: keep the newest message and
    # bump a counter. No parsing, no allocation, no arithmetic. At 250 Hz on
    # two topics that difference is most of a CPU core.
    def on_odom(self, m):
        with self.state.lock:
            self.state.odom = m
            self.state._bump()

    def on_consumed(self, m):
        with self.state.lock:
            self.state.consumed = m
            self.state._bump()

    def on_setpoint(self, m):
        with self.state.lock:
            self.state.setpoint = m
            self.state._bump()

    def on_status(self, m):
        with self.state.lock:
            self.state.status = m
            self.state._bump()

    def on_control(self, m):
        with self.state.lock:
            self.state.control = m
            self.state._bump()

    def on_clearance(self, m):
        with self.state.lock:
            self.state.clearance = m
            self.state._bump()

    def on_plan_path(self, m):
        with self.state.lock:
            self.state.plan_path = m
            self.state._bump()

    def on_imu(self, m):
        with self.state.lock:
            self.state.imu = m
            self.state._bump()


class Handler(SimpleHTTPRequestHandler):
    state = None          # set via partial()
    sim_control = None    # set via partial()
    solver = None         # set via partial()
    logger = None         # set via partial(): the node's logger
    rate_hz = 30.0
    # Refuse a body larger than this outright. The only legitimate request is a
    # few dozen bytes of JSON, so anything bigger is a mistake or an attempt to
    # make the server allocate.
    max_body = 4096

    def log_message(self, *args):
        pass              # a 30 Hz stream would drown the launch output

    def _json(self, obj, code=200):
        # Drain the request body if it has not already been consumed. Replying
        # without reading it leaves those bytes in the socket, where the next
        # keep-alive request parses them as a request line -- so one rejected
        # POST corrupts every later request on the connection.
        #
        # The `_body_read` flag matters: draining a body that do_POST already
        # read would block waiting for bytes that will never come, which turned
        # every SUCCESSFUL control request into an empty reply.
        if self.command == "POST" and not getattr(self, "_body_read", False):
            try:
                pending = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                pending = 0
            if 0 < pending <= self.max_body:
                try:
                    self.rfile.read(pending)
                    self._body_read = True
                except OSError:
                    pass
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _audit(self, outcome, detail=""):
        """One line per write attempt, whatever the outcome.

        The route is part of the detail the callers pass, so this says which
        write path was used -- it hard-coded "/control" while logging /solve
        requests, which is exactly the kind of log that makes an incident
        harder to reconstruct rather than easier.
        """
        if self.logger is not None:
            self.logger.info(
                f"write from {self.client_address[0]}: {outcome}"
                + (f" — {detail}" if detail else ""))

    def do_POST(self):
        """The write paths: /control drives the simulator, /solve the planner.

        Everything a /control request can express is parsed in simcontrol.py
        and applied by dsim_simctl, and the reply reports what was OBSERVED
        afterwards rather than what was asked for. /solve is in solver.py and
        can express three range-checked numbers.
        """
        route = self.path.split("?")[0]
        if route not in ("/control", "/solve"):
            self._json({"error": "not found"}, 404)
            return
        if route == "/control" and (self.sim_control is None or not self.sim_control.enabled):
            self._audit("refused", "control disabled")
            self._json({"error": "simulation control is disabled"}, 403)
            return
        if route == "/solve" and (self.solver is None or not self.solver.enabled):
            self._audit("refused", "solving disabled")
            self._json({"error": "this run has no plan to re-solve"}, 403)
            return

        # Require a JSON content type. This is not pedantry: without it the
        # request qualifies as a CORS "simple request", so any web page the
        # user happens to visit could pause their simulator with a form POST.
        # Demanding application/json forces a preflight, which this server
        # never answers, so a cross-origin page cannot reach the endpoint.
        ctype = (self.headers.get("Content-Type") or "").split(";")[0].strip()
        if ctype != "application/json":
            self._json({"error": "Content-Type must be application/json"}, 415)
            return

        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            self._json({"error": "bad Content-Length"}, 400)
            return
        # Both ends of the range. A NEGATIVE length passed the old
        # `length > max_body` test and then reached read(-1), which reads to
        # EOF -- an unbounded read that parks a server thread until the client
        # closes, from an endpoint reachable across the whole tailnet.
        if not 0 <= length <= self.max_body:
            self._json({"error": "bad or oversized body"}, 413)
            return
        try:
            raw = self.rfile.read(length)
            self._body_read = True
            body = json.loads(raw.decode() or "{}")
            if not isinstance(body, dict):
                raise ValueError("expected an object")
        except (ValueError, UnicodeDecodeError) as exc:
            self._json({"error": f"bad JSON: {exc}"}, 400)
            return

        try:
            result = (self.sim_control.command(body) if route == "/control"
                      else self.solver.solve(body))
        except (ControlError, SolveError) as exc:
            # A refused request is a normal outcome, not a server fault: 400 so
            # the page can show the reason instead of a generic failure.
            self._audit("refused", f"{route} {body} — {exc}")
            self._json({"error": str(exc)}, 400)
            return

        self._audit("applied", f"{route} {body} — {result.get('message', '')}")
        self._json(result)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/snapshot":
            self._json(self.state.snapshot())
            return
        if path == "/state":
            self._stream()
            return
        super().do_GET()

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        # Without this, a proxy in the middle may buffer the stream forever.
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        period = 1.0 / self.rate_hz
        try:
            while True:
                snap = self.state.snapshot()
                # Always send on a timer even when nothing changed, so the
                # client can tell "sim is idle" from "connection is dead".
                payload = json.dumps(snap)
                self.wfile.write(f"data: {payload}\n\n".encode())
                self.wfile.flush()
                time.sleep(period)
        except (BrokenPipeError, ConnectionResetError):
            pass          # the tab was closed; entirely normal


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--directory", default="/ws/web")
    parser.add_argument("--rate", type=float, default=30.0)
    # No --world-name: this process no longer talks to Gazebo at all. The world
    # name belongs to dsim_simctl, which is the only thing that does.
    parser.add_argument("--allow-control", action="store_true",
                        help="expose POST /control (pause, speed, reset) and, "
                             "when this run is flying a plan, POST /solve")
    parser.add_argument("--scenario", default="",
                        help="the space-time scenario this run is flying, if any")
    parser.add_argument("--plan-file", default="",
                        help="the plan file the bridge reads; /solve overwrites it")
    parser.add_argument("--ws", default="/ws", help="workspace root, for the solver")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    state = State()
    node = VizNode(state)

    # The simulator's state arrives as a ROS message now. The polling thread
    # that used to keep it fresh spawned a Ruby interpreter and a gz-transport
    # discovery round every two seconds -- 1,800 processes an hour -- to read
    # numbers that are published to any subscriber, inside a process whose own
    # docstring is about not being the reason the physics slows down.
    sim_control = SimControlClient(node, enabled=bool(args.allow_control))
    state.sim_source = lambda: sim_control.state

    solver = Solver(args.ws, args.scenario, args.plan_file,
                    enabled=bool(args.allow_control))
    state.solver_source = lambda: solver.state

    Handler.state = state
    Handler.sim_control = sim_control
    Handler.solver = solver
    Handler.logger = node.get_logger()
    Handler.rate_hz = args.rate
    handler = partial(Handler, directory=args.directory)
    server = ThreadingHTTPServer((args.bind, args.port), handler)
    server.daemon_threads = True

    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(
        f"viewer on http://{args.bind}:{args.port}  "
        f"(static {args.directory}, SSE /state at {args.rate:.0f} Hz, "
        f"control {'ON' if args.allow_control else 'OFF'}, "
        f"solve {'ON for ' + args.scenario if solver.enabled else 'OFF'})")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
