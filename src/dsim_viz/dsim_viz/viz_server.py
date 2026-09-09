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
    /control       POST: pause/resume and playback speed  (see simcontrol.py)

Almost read-only. Every GET is; /control is the single write path, and it can
express exactly two things -- paused, and a range-checked real-time factor.
It cannot arm a vehicle, retune a controller, move the drone or run a command.
That narrowness is the point: this port is reachable from the whole tailnet,
and `control:=false` in the launch removes the endpoint entirely.
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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from dsim_msgs.msg import ControlDebug, FlightStatus, TrajectorySetpoint

from . import overlay
from .simcontrol import ControlError, SimControl


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
        self.setpoint = None
        self.status = None
        self.control = None
        self.imu = None
        # Simulator pause/speed, sampled by a background thread. Kept outside
        # the snapshot memo because it changes on its own schedule and must not
        # invalidate a cached frame of vehicle state.
        self.sim = {"paused": False, "measured_rtf": None,
                    "target_rtf": 1.0, "enabled": False, "error": None}
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
                # The memo holds vehicle state; the simulator's pause/speed is
                # refreshed independently, so it is stamped in fresh.
                return dict(self._cache, sim=dict(self.sim))
            seq, odom, sp = self.seq, self.odom, self.setpoint
            status, control, imu = self.status, self.control, self.imu

        # Built outside the lock: message objects are not mutated after
        # publication, so reading them here cannot tear, and holding the lock
        # through the conversion would stall the 250 Hz subscriptions.
        snap = {
            "seq": seq,
            "t": _stamp_seconds(odom) if odom else 0.0,
            "pose": _pose_of(odom),
            "vel": _twist_linear_of(odom),
            "setpoint": _setpoint_of(sp),
            "status": _status_of(status),
            "control": _control_of(control),
            "imu": _imu_of(imu),
        }
        snap["overlay"] = overlay.build(snap["pose"], snap["control"], snap["imu"])

        with self.lock:
            # A message may have landed while we were converting; keeping the
            # newer seq would make this snapshot look fresher than it is, so
            # the memo is only valid for the seq it was built from.
            if self._cache_seq < seq:
                self._cache, self._cache_seq = snap, seq
            snap = dict(snap, sim=dict(self.sim))
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
        "tracking_max_m": m.tracking_max_m,
        # JSON has no NaN; send null so the page shows "n/a" rather than a
        # fake number.
        "min_obstacle_clearance_m": None if math.isnan(clearance) else clearance,
        "path_length_m": m.path_length_m,
        "elapsed_s": m.elapsed_s,
        "energy_wh": m.energy_wh,
    }


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
        self.create_subscription(TrajectorySetpoint, "/drone/viz/setpoint",
                                 self.on_setpoint, qos_profile_sensor_data)
        self.create_subscription(ControlDebug, "/drone/viz/control_debug",
                                 self.on_control, qos_profile_sensor_data)
        self.create_subscription(Imu, "/drone/viz/imu",
                                 self.on_imu, qos_profile_sensor_data)
        # Already slow at the source, so it comes straight from the referee.
        self.create_subscription(FlightStatus, "/drone/eval/status",
                                 self.on_status, 10)

    # Every callback does the same cheap thing: keep the newest message and
    # bump a counter. No parsing, no allocation, no arithmetic. At 250 Hz on
    # two topics that difference is most of a CPU core.
    def on_odom(self, m):
        with self.state.lock:
            self.state.odom = m
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

    def on_imu(self, m):
        with self.state.lock:
            self.state.imu = m
            self.state._bump()


class Handler(SimpleHTTPRequestHandler):
    state = None          # set via partial()
    sim_control = None    # set via partial()
    rate_hz = 30.0
    # Refuse a body larger than this outright. The only legitimate request is a
    # few dozen bytes of JSON, so anything bigger is a mistake or an attempt to
    # make the server allocate.
    max_body = 4096

    def log_message(self, *args):
        pass              # a 30 Hz stream would drown the launch output

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        """The one write path: pause/resume and playback speed.

        Everything the request can express is validated in simcontrol.py before
        it reaches the simulator, and the reply reports what was OBSERVED
        afterwards rather than what was asked for.
        """
        if self.path.split("?")[0] != "/control":
            self._json({"error": "not found"}, 404)
            return
        if self.sim_control is None or not self.sim_control.enabled:
            self._json({"error": "simulation control is disabled"}, 403)
            return
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            self._json({"error": "bad Content-Length"}, 400)
            return
        if length > self.max_body:
            self._json({"error": "request too large"}, 413)
            return
        try:
            body = json.loads(self.rfile.read(length).decode() or "{}")
            if not isinstance(body, dict):
                raise ValueError("expected an object")
        except (ValueError, UnicodeDecodeError) as exc:
            self._json({"error": f"bad JSON: {exc}"}, 400)
            return

        try:
            result = None
            # Speed first, then pause: setting a speed on a paused world should
            # leave it paused, not silently resume it.
            if "rtf" in body:
                result = self.sim_control.set_rtf(body["rtf"])
            if "paused" in body:
                result = self.sim_control.set_paused(bool(body["paused"]))
            if result is None:
                self._json({"error": "nothing to do: send paused or rtf"}, 400)
                return
        except ControlError as exc:
            # A refused request is a normal outcome, not a server fault: 400 so
            # the page can show the reason instead of a generic failure.
            self._json({"error": str(exc)}, 400)
            return

        with self.state.lock:
            self.state.sim.update(result)
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
        last_seq = -1
        try:
            while True:
                snap = self.state.snapshot()
                # Always send on a timer even when nothing changed, so the
                # client can tell "sim is idle" from "connection is dead".
                payload = json.dumps(snap)
                self.wfile.write(f"data: {payload}\n\n".encode())
                self.wfile.flush()
                last_seq = snap["seq"]
                time.sleep(period)
        except (BrokenPipeError, ConnectionResetError):
            pass          # the tab was closed; entirely normal


def _watch_sim(state, sim_control, period=2.0):
    """Keep the reported pause/speed honest without costing anything.

    Sampling gz means spawning a process, so it happens here on a slow timer
    instead of inside snapshot() -- which the event stream calls 30 times a
    second per browser, and which must never block on a subprocess.
    """
    while True:
        observed = sim_control.observe()
        with state.lock:
            state.sim.update(observed)
        time.sleep(period)


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--directory", default="/ws/web")
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--world-name", default="",
                        help="Gazebo world name, for the pause/speed services. "
                             "This is the name INSIDE the SDF, not the file "
                             "stem; the launch file reads it and passes it.")
    parser.add_argument("--allow-control", action="store_true",
                        help="expose POST /control (pause and playback speed)")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    state = State()
    node = VizNode(state)

    control_enabled = bool(args.allow_control and args.world_name)
    sim_control = SimControl(args.world_name, enabled=control_enabled)
    with state.lock:
        state.sim.update(sim_control.state)
    if control_enabled:
        threading.Thread(target=_watch_sim, args=(state, sim_control),
                         daemon=True).start()

    Handler.state = state
    Handler.sim_control = sim_control
    Handler.rate_hz = args.rate
    handler = partial(Handler, directory=args.directory)
    server = ThreadingHTTPServer((args.bind, args.port), handler)
    server.daemon_threads = True

    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(
        f"viewer on http://{args.bind}:{args.port}  "
        f"(static {args.directory}, SSE /state at {args.rate:.0f} Hz, "
        f"control {'ON for world ' + args.world_name if control_enabled else 'OFF'})")

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
