"""Turning an HTTP request into a simulator command, and back into a reply.

This is the whole of the viewer's write path, and it is deliberately thin: it
parses, it forwards, it reports what came back. Nothing here decides anything
about the simulator. Pausing, stepping and resetting all happen in
dsim_simctl's C++ node, which owns the one persistent connection to Gazebo.

WHAT THIS MODULE USED TO BE, AND WHY IT IS NOT ANY MORE.

It used to build `gz` command lines and run them with subprocess:

    gz service -s /world/<w>/set_physics --req "real_time_factor: 0.5, ..."

That was wrong in two independent ways.

  * `set_physics` makes the world weightless. gz.msgs.Physics has no gravity
    field, gz-sim assigns gravity from the message regardless, and proto3 reads
    the absent field as (0, 0, 0). The service answers `data: true` and nothing
    logs anything. A user moved the speed slider and their vehicle left; it was
    at 41.5 km eleven hours later. See dsim_simctl/pacer.hpp.
  * Every command cost a process. `gz` is a Ruby wrapper: 305 ms of startup per
    call whatever is being asked. `observe()` ran one every two seconds
    forever, and each timeout orphaned a `gz-transport-topic` helper that
    outlived the request by hours.

The lesson worth keeping is not "validate harder". It is that a write path
made of shell commands cannot be reasoned about: the surface it can reach is
whatever the CLI happens to expose, which here included a way to delete
gravity. The surface is now three ROS service commands that exist in an
interface definition, and there is no shell.

WHERE THE LIMITS LIVE. Not here. The accepted speed range is published by the
control node in SimState and validated against what it reported, so the slider
in the page, the check in this file and the check in the node cannot disagree
about what 4x means. A value this module accepts is still re-checked at the
far end; this validation exists to give a person a usable error, not to be the
last line of defence.
"""
import threading
from collections import namedtuple

# rclpy and dsim_msgs are imported inside SimControlClient, not here. The
# parsing and validation below are the part most likely to be wrong and the
# part cheapest to test, and requiring a built ROS workspace to test them means
# they get tested on the machine where a failure is least likely to be noticed.
# Nothing above SimControlClient touches ROS.

#: How long to wait for the control node to answer before giving up. Generous
#: relative to the work (a reset is one Gazebo service call) and short enough
#: that a wedged simulator does not hold an HTTP thread open indefinitely.
CALL_TIMEOUT_S = 6.0

#: What the viewer shows before the control node has said anything. Every
#: numeric field is None rather than a plausible default: "unknown" and "1.00x"
#: must not look the same on screen.
UNKNOWN = {
    "enabled": False,
    "paused": False,
    "pacing": False,
    "requested_speed": None,
    "achieved_speed": None,
    "min_speed": None,
    "max_speed": None,
    "sim_time_s": None,
    "step_size_s": None,
    "backlog_s": None,
    "target_sim_time_s": None,
    "resets": 0,
    "step_errors": 0,
    "ticks": 0,
    "run_requests": 0,
    "error": "waiting for the simulation control node",
}


class ControlError(Exception):
    """A request that must be refused, with a reason fit to show a user."""


def state_to_dict(msg):
    """SimState message -> the JSON the page receives.

    Field for field, with no arithmetic and no defaults filled in. Everything
    in SimState is measured or counted by the control node; recomputing any of
    it here would be a second opinion about the simulator held by something
    that is not connected to it.
    """
    return {
        "enabled": bool(msg.enabled),
        "paused": bool(msg.paused),
        "pacing": bool(msg.pacing),
        "requested_speed": msg.requested_speed,
        "achieved_speed": msg.achieved_speed,
        "min_speed": msg.min_speed,
        "max_speed": msg.max_speed,
        "sim_time_s": msg.sim_time_s,
        "step_size_s": msg.step_size_s,
        "backlog_s": msg.backlog_s,
        "target_sim_time_s": msg.target_sim_time_s,
        "resets": int(msg.resets),
        "step_errors": int(msg.step_errors),
        # Cumulative counters, so two samples give a rate. The pacer is
        # otherwise invisible: when it ran at twice the requested speed the
        # only way to see it was to time the world with a stopwatch.
        "ticks": int(msg.ticks),
        "run_requests": int(msg.run_requests),
        "error": msg.error,
    }


#: One parsed request. `verb` is one of VERBS; the other fields carry whatever
#: that verb needs and are ignored otherwise.
Command = namedtuple("Command", "verb speed paused")

VERBS = ("speed", "paused", "toggle_pause", "reset")


def parse_command(body, limits):
    """HTTP body -> a Command, or ControlError.

    `limits` is the last observed state; the speed range comes from it so this
    check and the control node's cannot drift apart. A body may name exactly
    one verb: "set the speed and also unpause" would need an order between the
    two, and an order nobody stated is an order nobody tested.
    """
    if not isinstance(body, dict):
        raise ControlError("expected a JSON object")

    if "rtf" in body:
        # The old spelling, and the old mechanism. Refused loudly rather than
        # quietly translated, because anyone still sending it is working from
        # instructions that also named a Gazebo service that must not be
        # called.
        raise ControlError(
            "'rtf' is gone: playback speed is no longer a Gazebo real-time "
            'factor. Send {"speed": <x>} instead.')

    verbs = [k for k in VERBS if k in body]
    if not verbs:
        raise ControlError(
            "nothing to do: send one of " + ", ".join(VERBS))
    if len(verbs) > 1:
        raise ControlError(f"send one command at a time, got {', '.join(verbs)}")

    verb = verbs[0]
    if verb == "speed":
        return Command("speed", _speed(body["speed"], limits), False)
    if verb == "paused":
        return Command("paused", 0.0, bool(body["paused"]))
    # toggle_pause and reset carry nothing: the node holds the current state.
    return Command(verb, 0.0, False)


def _speed(value, limits):
    try:
        speed = float(value)
    except (TypeError, ValueError):
        raise ControlError(f"speed must be a number, got {value!r}")
    if speed != speed:                                   # NaN
        raise ControlError("speed must be a real number")
    lo, hi = limits.get("min_speed"), limits.get("max_speed")
    if lo is None or hi is None:
        raise ControlError(
            "the simulation control node has not reported its speed range yet")
    if not lo <= speed <= hi:
        raise ControlError(f"speed must be between {lo:g} and {hi:g}, got {speed:g}")
    return speed


class SimControlClient:
    """The viewer's end of /sim/control and /sim/state.

    `state` reports what the control node last OBSERVED of the simulator, not
    what anyone last requested. Those differ whenever a command fails or the
    machine cannot keep up, and reporting the request as though it were the
    truth is how a viewer ends up lying about the thing it exists to show.
    """

    def __init__(self, node, enabled=True, timeout_s=CALL_TIMEOUT_S):
        self._lock = threading.Lock()
        self._timeout_s = timeout_s
        self.enabled = bool(enabled)
        self._state = dict(UNKNOWN)
        self._srv = None
        self._client = None
        if not self.enabled:
            self._state["error"] = "simulation control is disabled on this server"
            return

        from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
        from dsim_msgs.msg import SimState
        from dsim_msgs.srv import SimControl

        self._srv = SimControl
        # Transient-local so a viewer started after the control node still
        # learns the speed range and the pause state, instead of showing
        # "unknown" until something happens to change.
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(SimState, "/sim/state", self._on_state, qos)
        self._client = node.create_client(SimControl, "/sim/control")

    def _to_request(self, cmd):
        """Command -> the ROS request. The only place the wire format lives."""
        Request = self._srv.Request
        req = Request()
        req.command = {
            "paused": Request.SET_PAUSED,
            "speed": Request.SET_SPEED,
            "reset": Request.RESET,
            "toggle_pause": Request.TOGGLE_PAUSED,
        }[cmd.verb]
        req.speed = float(cmd.speed)
        req.paused = bool(cmd.paused)
        return req

    def _on_state(self, msg):
        with self._lock:
            self._state = state_to_dict(msg)

    @property
    def state(self):
        with self._lock:
            return dict(self._state)

    def command(self, body):
        """Apply one command and return the state observed afterwards."""
        if not self.enabled or self._client is None:
            raise ControlError("simulation control is disabled on this server")
        req = self._to_request(parse_command(body, self.state))
        if not self._client.service_is_ready():
            raise ControlError("the simulation control node is not running")

        # call_async plus an event rather than the blocking call(): this runs
        # on an HTTP thread while the executor spins elsewhere, and the
        # blocking form is documented as unsafe in that arrangement.
        future = self._client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(self._timeout_s):
            future.cancel()
            raise ControlError("the simulator did not answer in time")

        res = future.result()
        if res is None:
            raise ControlError("the simulation control node dropped the request")
        state = state_to_dict(res.state)
        with self._lock:
            self._state = state
        if not res.ok:
            raise ControlError(res.message or "the simulator refused the request")
        return dict(state, message=res.message)
