"""Pause, resume and playback speed for the running simulation.

This is the one place the viewer stack can WRITE to the simulator, and it is
deliberately narrow: two operations, both validated here, neither able to
express anything else. The server used to be read-only by construction, which
was a real safety property -- reaching the port could not arm a vehicle or
retune a controller. That property is now weaker, so the surface is kept as
small as it can be:

  * exactly two verbs: set paused, set real-time factor
  * the real-time factor is range-checked before it reaches a subprocess
  * the command is built as an argument LIST, never a shell string, so nothing
    from the network can be interpreted as shell syntax
  * the world name comes from the launch file, not from the request

All of the logic lives here rather than in the HTTP handler so it can be
tested without a server or a simulator: test_simcontrol.py checks the
validation and the exact commands, which is where a mistake would be both easy
to make and invisible.

Gazebo exposes these as services on the world:
    /world/<name>/control       gz.msgs.WorldControl   pause/resume
    /world/<name>/set_physics   gz.msgs.Physics        real-time factor
"""
import os
import re
import shutil
import subprocess
import threading

# A quadrotor at 0.05x is watchable frame by frame; past 4x the 1 ms physics
# step stops keeping up on this machine and the "speed" becomes a lie, because
# the simulator silently fails to hit the target instead of running faster.
MIN_RTF = 0.05
MAX_RTF = 4.0

# gz-tools is vendored under the ROS prefix on Jazzy, NOT in /usr/bin, and
# `docker exec` does not run the entrypoint that would put it on PATH.
VENDORED_GZ = "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz"

TIMEOUT_S = 4.0


class ControlError(Exception):
    """A request that must be refused, with a reason fit to show a user."""


def find_gz():
    """Absolute path to the gz CLI, or None if it is not installed."""
    if os.path.isfile(VENDORED_GZ) and os.access(VENDORED_GZ, os.X_OK):
        return VENDORED_GZ
    return shutil.which("gz")


def validate_rtf(value):
    """Coerce a request's real-time factor to a float in range, or refuse.

    Anything from the network is untrusted: a string, a null, a NaN or an
    absurd number must be refused here rather than handed to the simulator,
    where a bad physics setting is much harder to notice than a rejected
    request.
    """
    try:
        rtf = float(value)
    except (TypeError, ValueError):
        raise ControlError(f"speed must be a number, got {value!r}")
    if rtf != rtf:                                   # NaN
        raise ControlError("speed must be a real number")
    if not (MIN_RTF <= rtf <= MAX_RTF):
        raise ControlError(
            f"speed must be between {MIN_RTF} and {MAX_RTF}, got {rtf}")
    return rtf


def pause_command(gz, world, paused):
    """The gz command that pauses or resumes the world."""
    return [
        gz, "service", "-s", f"/world/{world}/control",
        "--reqtype", "gz.msgs.WorldControl",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000",
        "--req", f"pause: {'true' if paused else 'false'}",
    ]


def rtf_command(gz, world, rtf):
    """The gz command that sets the world's real-time factor.

    max_step_size is sent alongside because gz.msgs.Physics replaces the whole
    physics profile: omitting it resets the step to a default, which would
    silently change the integration accuracy along with the playback speed.
    """
    return [
        gz, "service", "-s", f"/world/{world}/set_physics",
        "--reqtype", "gz.msgs.Physics",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000",
        "--req", f"real_time_factor: {rtf}, max_step_size: 0.001",
    ]


def stats_command(gz, world):
    """The gz command that samples one world-statistics message."""
    return [gz, "topic", "-e", "-t", f"/world/{world}/stats", "-n", "1"]


def parse_stats(text):
    """Read paused state and real-time factor out of gz's text output.

    Protobuf text format OMITS false booleans, so a missing `paused` field
    means running. Treating absence as unknown would make the viewer show
    "paused?" for every normal frame.
    """
    paused = bool(re.search(r"^\s*paused:\s*true\s*$", text, re.MULTILINE))
    m = re.search(r"^\s*real_time_factor:\s*([0-9.eE+-]+)", text, re.MULTILINE)
    rtf = float(m.group(1)) if m else None
    return {"paused": paused, "measured_rtf": rtf}


class SimControl:
    """Applies pause/speed to a named world and remembers what it observed.

    `state` reports what was last OBSERVED from the simulator, not what was
    last requested. Those differ whenever a command fails or someone pauses
    from the Gazebo GUI, and reporting the request as though it were the truth
    is how a viewer ends up lying about the thing it exists to show.
    """

    def __init__(self, world, enabled=True, runner=subprocess.run):
        self.world = world
        self.enabled = enabled
        self._runner = runner
        self._lock = threading.Lock()
        self._state = {"paused": False, "measured_rtf": None,
                       "target_rtf": 1.0, "enabled": enabled,
                       "error": None if enabled else "control disabled"}
        self._gz = find_gz()
        if enabled and self._gz is None:
            self._state["enabled"] = False
            self._state["error"] = "gz CLI not found; controls unavailable"

    @property
    def state(self):
        with self._lock:
            return dict(self._state)

    def _require_enabled(self):
        if not self.enabled or self._gz is None:
            raise ControlError("simulation control is disabled on this server")

    def _run(self, cmd):
        try:
            return self._runner(cmd, capture_output=True, text=True,
                                timeout=TIMEOUT_S)
        except subprocess.TimeoutExpired:
            raise ControlError("the simulator did not answer in time")
        except OSError as exc:
            raise ControlError(f"could not run gz: {exc}")

    def set_paused(self, paused):
        self._require_enabled()
        self._run(pause_command(self._gz, self.world, bool(paused)))
        return self.observe()

    def set_rtf(self, value):
        self._require_enabled()
        rtf = validate_rtf(value)
        self._run(rtf_command(self._gz, self.world, rtf))
        with self._lock:
            self._state["target_rtf"] = rtf
        return self.observe()

    def observe(self):
        """Sample the simulator and update the cached state."""
        if not self.enabled or self._gz is None:
            return self.state
        try:
            done = self._run(stats_command(self._gz, self.world))
            observed = parse_stats(done.stdout or "")
            err = None
        except ControlError as exc:
            observed = {}
            err = str(exc)
        with self._lock:
            self._state.update(observed)
            self._state["error"] = err
            return dict(self._state)
