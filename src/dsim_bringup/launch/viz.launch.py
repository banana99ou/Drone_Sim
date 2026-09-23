"""Bring up the browser viewer, and nothing else.

    ros2 launch dsim_bringup viz.launch.py
    ros2 launch dsim_bringup viz.launch.py web_port:=8080 bind:=100.64.0.1

SEPARATE FROM sim.launch.py ON PURPOSE, and the purpose is the scenario
dropdown. The viewer's POST /launch tears the simulator down and brings a
different scenario up; a server that was part of the launch it restarts would
kill itself mid-reply, leaving the page on a closed socket with no way to say
whether the new run came up, failed, or was never started. So this process
outlives the simulator, and only the simulator is cycled.

What that costs, and where it is paid:

  * The scenario is no longer a property of this launch. It is read from
    web/current.json, which sim.launch.py writes when it comes up -- so the
    viewer reports what is RUNNING, not what someone asked for.
  * The viewer holds the web port for as long as this is up. Starting a second
    one on the same port fails loudly on bind, which is the right outcome:
    two servers answering one port would serve two different ideas of what is
    flying.
  * scripts/kill_sim.sh kills this too, by design -- it derives its pattern
    list from the install tree and viz_server is in it. The one path that must
    NOT kill it is the teardown the viewer itself runs, and that is already
    handled: kill_sim.sh excludes its own ancestors, and there the viewer is
    one. Nothing here is special-cased.

The relay (dsim_eval/viz_relay_node) stays in sim.launch.py, because its state
topic follows state:=est|truth, which is a property of a RUN and not of the
viewer.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WS = os.environ.get('DSIM_WS', '/ws')
WEB_DIR = os.path.join(WS, 'web')


def launch_setup(context, *args, **kwargs):
    control = LaunchConfiguration('control').perform(context)
    if control not in ('true', 'false'):
        raise RuntimeError(f"control:={control} is not true | false")
    return [Node(
        package='dsim_viz', executable='viz_server', name='dsim_viz',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=(['--port', LaunchConfiguration('web_port').perform(context),
                    '--bind', LaunchConfiguration('bind').perform(context),
                    '--directory', WEB_DIR, '--ws', WS]
                   + (['--allow-control'] if control == 'true' else [])),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('bind', default_value='0.0.0.0',
                              description='interface the viewer binds to. Set '
                                          'to your Tailscale IP to keep it off '
                                          'the local LAN.'),
        DeclareLaunchArgument('control', default_value='true',
                              description='expose the write paths: POST '
                              '/control (pause, playback speed, reset, gust), '
                              'POST /solve (re-solve the running scenario) and '
                              'POST /launch (switch scenario). false serves a '
                              'read-only viewer -- the endpoints still exist '
                              'and answer 403, so the page can say they are '
                              'off rather than appear broken.'),
        # use_sim_time is deliberately FALSE, and it is the single biggest cost
        # in this launch if you get it wrong. True makes rclpy subscribe to
        # /clock, which the bridge publishes at the 1 ms physics step -- 1000
        # messages a second into a Python process, ~50% of a core, for a clock
        # this node never reads. It is also now wrong for a second reason: this
        # process outlives the simulator, so there are stretches with no /clock
        # publisher at all, and a node timed by a clock nobody publishes never
        # ticks again.
        OpaqueFunction(function=launch_setup),
    ])
