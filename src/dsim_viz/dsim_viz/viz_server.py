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

Endpoints, all on one port and one origin:

    /              web/index.html and friends (static)
    /snapshot      one JSON object with the latest state  (curl-friendly)
    /state         text/event-stream, ~30 Hz of the same object

Read-only by construction: there is no endpoint that writes anything to ROS, so
exposing this port cannot arm a vehicle or retune a controller.
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


def _v3(v):
    """geometry_msgs/Vector3 (or anything with x/y/z) -> [x, y, z]."""
    return [v.x, v.y, v.z]


class State:
    """Latest values, shared between the ROS thread and HTTP threads."""

    def __init__(self):
        self.lock = threading.Lock()
        self.seq = 0
        self.pose = None
        self.vel = None
        self.setpoint = None
        self.status = None
        self.control = None
        self.imu = None
        self.stamp = 0.0

    def snapshot(self):
        # Copy under the lock, then compute outside it: overlay.build() is pure
        # arithmetic and holding the lock through it would stall the 250 Hz
        # subscriptions for every connected browser.
        with self.lock:
            snap = {
                "seq": self.seq,
                "t": self.stamp,
                "pose": self.pose,
                "vel": self.vel,
                "setpoint": self.setpoint,
                "status": self.status,
                "control": self.control,
                "imu": self.imu,
            }
        snap["overlay"] = overlay.build(snap["pose"], snap["control"], snap["imu"])
        return snap


class VizNode(Node):
    def __init__(self, state):
        super().__init__("dsim_viz")
        self.state = state
        self.create_subscription(Odometry, "/drone/odom", self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(TrajectorySetpoint, "/drone/setpoint",
                                 self.on_setpoint, 10)
        self.create_subscription(FlightStatus, "/drone/eval/status",
                                 self.on_status, 10)
        # Both of these publish at 250 Hz while the stream goes out at 30, so
        # most frames are overwritten before anyone sees them. That is the
        # intent: keep the LATEST, never a backlog. Best-effort QoS
        # (qos_profile_sensor_data) is what makes a slow viewer harmless to the
        # control loop.
        self.create_subscription(ControlDebug, "/drone/control_debug",
                                 self.on_control, qos_profile_sensor_data)
        self.create_subscription(Imu, "/drone/imu",
                                 self.on_imu, qos_profile_sensor_data)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        v = m.twist.twist.linear
        with self.state.lock:
            self.state.pose = {"p": [p.x, p.y, p.z], "q": [q.w, q.x, q.y, q.z]}
            self.state.vel = [v.x, v.y, v.z]
            self.state.stamp = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
            self.state.seq += 1

    def on_setpoint(self, m):
        with self.state.lock:
            self.state.setpoint = [m.position.x, m.position.y, m.position.z]

    def on_status(self, m):
        clearance = m.min_obstacle_clearance_m
        with self.state.lock:
            self.state.status = {
                "collided": bool(m.collided),
                "collision_count": int(m.collision_count),
                "tracking_error_m": m.tracking_error_m,
                "tracking_rmse_m": m.tracking_rmse_m,
                "tracking_max_m": m.tracking_max_m,
                # JSON has no NaN; send null so the page shows "n/a" rather
                # than a fake number.
                "min_obstacle_clearance_m": (
                    None if math.isnan(clearance) else clearance),
                "path_length_m": m.path_length_m,
                "elapsed_s": m.elapsed_s,
                "energy_wh": m.energy_wh,
            }

    def on_control(self, m):
        # Passed through field for field, with no arithmetic. Any force the
        # viewer draws should be traceable to one number the controller
        # published; a "helpful" conversion here would be a second, silent
        # opinion about the vehicle's physics.
        with self.state.lock:
            self.state.control = {
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

    def on_imu(self, m):
        # Proper acceleration in the BODY frame: what an accelerometer feels,
        # so it includes the reaction to gravity (a resting vehicle reads
        # +9.81 on z) and excludes gravity itself. Multiplied by mass it is the
        # total non-gravitational force on the airframe -- thrust plus
        # everything aerodynamic -- which is how the viewer gets a MEASURED
        # drag arrow instead of one recomputed from a drag coefficient it would
        # have had to duplicate.
        with self.state.lock:
            self.state.imu = {
                "accel_body": _v3(m.linear_acceleration),
                "rate_body": _v3(m.angular_velocity),
            }


class Handler(SimpleHTTPRequestHandler):
    state = None          # set via partial()
    rate_hz = 30.0

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


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--directory", default="/ws/web")
    parser.add_argument("--rate", type=float, default=30.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    state = State()
    node = VizNode(state)

    Handler.state = state
    Handler.rate_hz = args.rate
    handler = partial(Handler, directory=args.directory)
    server = ThreadingHTTPServer((args.bind, args.port), handler)
    server.daemon_threads = True

    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(
        f"viewer on http://{args.bind}:{args.port}  "
        f"(static {args.directory}, SSE /state at {args.rate:.0f} Hz)")

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
