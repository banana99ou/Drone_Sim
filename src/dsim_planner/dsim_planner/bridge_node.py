"""Publish a space-time Bezier plan as /drone/trajectory.

This is the whole seam between the planner and the simulator: a plan file
(control points in x, y, z, t) comes in, a dsim_msgs/Trajectory sampled uniform
in time goes out. The controller already accepts time-parameterised setpoints
with velocity and acceleration feedforward, so nothing on its side changes.

    ros2 run dsim_planner bridge_node --ros-args -p plan_file:=/ws/plans/fence3d_seed.json \\
        -p start_s:=10.0

The trajectory's header.stamp is `start_s` of SIMULATED time, absolute: that
is when scenario t=0 happens, the same instant the world's moving obstacles
pass through their pos0 (worlds/<scenario>.sdf is generated with that offset)
and the same origin the referee scores against. Before it, the controller
holds the plan's first point, which is how the vehicle gets from the pad to
the start; after the last point it parks at the goal.

The message is REPUBLISHED every second, unchanged. That is not a replan: it
is how the plan survives a reset. A reset sends simulated time back to zero,
the controller drops the plan it was flying (it is stamped minutes into the
future of the new run), and the next republish, with the same absolute stamp,
puts it back so the new run flies the same plan at the same time.

The plan FILE is re-read on the same tick when its mtime changes, so solving a
new plan over the running one -- scripts/solve_plan.py, or the viewer's solve
panel -- takes effect without relaunching the simulator. A file that does not
parse, or whose control points are not a trajectory, is REFUSED and the old
plan keeps flying: a half-written file must not be able to disarm a vehicle in
the air. (solve_plan.py writes aside and renames, so that should never be the
reason, but "should" is not a mechanism.)

Nothing here reads /clock: with use_sim_time an rclpy node ingests the 1 kHz
clock topic at a measured ~50% of a core, for a node that needs the sim time
only to warn that it started late. The referee's status carries a sim-time
stamp at 20 Hz; that is enough.
"""
import json
import math
import os
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from dsim_msgs.msg import FlightStatus, Trajectory, TrajectorySetpoint

from . import spacetime


def load_plan(path):
    with open(path) as fh:
        plan = json.load(fh)
    if "control_points" not in plan:
        raise spacetime.PlanError(f"{path}: no 'control_points'")
    return plan


def build_trajectory(control_points, start_s, sample_dt, yaw):
    """A dsim_msgs/Trajectory from control points; pure, so it is testable."""
    P = spacetime.validate(control_points)
    samples = spacetime.sample_uniform_in_time(P, sample_dt)
    msg = Trajectory()
    sec = int(math.floor(start_s))
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = int(round((start_s - sec) * 1e9))
    msg.header.frame_id = "world"
    t_origin = samples[0][0]
    for t, p, v, a in samples:
        sp = TrajectorySetpoint()
        rel = t - t_origin
        sp.time_from_start.sec = int(math.floor(rel))
        sp.time_from_start.nanosec = int(round((rel - sp.time_from_start.sec) * 1e9))
        sp.position.x, sp.position.y, sp.position.z = p
        sp.velocity.x, sp.velocity.y, sp.velocity.z = v
        sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = a
        sp.yaw = yaw
        sp.yaw_rate = 0.0
        msg.points.append(sp)
    return msg, samples


class BridgeNode(Node):
    def __init__(self):
        super().__init__("dsim_planner_bridge")
        plan_file = self.declare_parameter("plan_file", "").value
        self.start_s = float(self.declare_parameter("start_s", 10.0).value)
        sample_dt = float(self.declare_parameter("sample_dt_s", 0.02).value)
        yaw = float(self.declare_parameter("yaw_rad", 0.0).value)
        period = float(self.declare_parameter("republish_period_s", 1.0).value)
        if not plan_file:
            raise RuntimeError("plan_file is required; there is no default plan to fly")

        self.plan_file = plan_file
        self.sample_dt = sample_dt
        self.yaw = yaw
        self.mtime = None

        self.pub = self.create_publisher(Trajectory, "/drone/trajectory", 4)
        # The whole path, for anything that wants to draw it before it is
        # flown. Latched so a viewer that connects later still gets it.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "/drone/plan_path", latched)
        self.msg = None
        self.load()                     # raises if the plan given at launch is bad

        self.warned_late = False
        self.create_subscription(FlightStatus, "/drone/eval/status", self.on_status, 10)
        self.timer = self.create_timer(period, self.republish)
        self.republish()

    def load(self):
        """(Re)read the plan file. Returns True if a new plan was installed.

        Raises on the FIRST load -- a bridge given a plan it cannot fly should
        not start -- and swallows afterwards, because by then something is
        already flying and dropping it would be worse than ignoring a bad edit.
        """
        try:
            mtime = os.path.getmtime(self.plan_file)
        except OSError as exc:
            if self.msg is None:
                raise
            self._warn_once(f"cannot stat {self.plan_file}: {exc}")
            return False
        if mtime == self.mtime:
            return False
        try:
            plan = load_plan(self.plan_file)
            msg, samples = build_trajectory(
                plan["control_points"], self.start_s, self.sample_dt, self.yaw)
        except (spacetime.PlanError, OSError, KeyError, TypeError,
                json.JSONDecodeError) as exc:
            if self.msg is None:
                raise
            self._warn_once(f"{self.plan_file} did not load ({exc}); "
                            f"keeping the plan already flying")
            return False
        self.mtime = mtime
        self.msg = msg
        self.bad_reason = None
        s = spacetime.summarize(samples)
        d = plan.get("demands") or {}
        self.get_logger().info(
            f"plan {self.plan_file} ({plan.get('solver', '?')}"
            f"{', ' + plan['config'] if plan.get('config') else ''}): "
            f"{s['points']} samples over {s['duration_s']:.2f} s, "
            f"max speed {s['max_speed_mps']:.2f} m/s, "
            f"max accel {s['max_accel_mps2']:.2f} m/s^2 (needs {s['max_tilt_deg']:.1f} deg "
            f"of tilt); scenario t=0 at sim {self.start_s:.1f} s")
        if d and not d.get("flyable", True):
            self.get_logger().warn(
                f"this plan is NOT flyable by this vehicle: {s['max_tilt_deg']:.1f} deg "
                f"of tilt demanded against a {d.get('tilt_limit_deg', 0):.0f} deg clamp. "
                f"The loop will saturate and the vehicle will fall behind.")
        self.path_pub.publish(self._path(samples))
        return True

    def _warn_once(self, text):
        if getattr(self, "bad_reason", None) != text:
            self.bad_reason = text
            self.get_logger().warn(text)

    def _path(self, samples):
        path = Path()
        path.header = self.msg.header
        for t, p, _, _ in samples[::5]:      # 0.1 s spacing is plenty for a line
            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = p
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    def republish(self):
        self.load()
        self.pub.publish(self.msg)

    def on_status(self, m):
        sim_s = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        if sim_s > self.start_s and not self.warned_late and m.elapsed_s < 1.0:
            # A fresh run (elapsed just reset) whose clock is already past the
            # start: the plan begins mid-way and the obstacles are ahead of it.
            self.get_logger().warn(
                f"sim time {sim_s:.1f} s is already past start_s={self.start_s:.1f}: "
                "the plan is being joined late and the moving obstacles are not "
                "where the planner thinks they are")
            self.warned_late = True


def main(argv=None):
    rclpy.init(args=argv)
    try:
        node = BridgeNode()
    except (spacetime.PlanError, RuntimeError, OSError, json.JSONDecodeError) as exc:
        print(f"[dsim_planner_bridge] refusing to start: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
