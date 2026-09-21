"""Bring up the quadrotor simulator.

    ros2 launch dsim_bringup sim.launch.py
    ros2 launch dsim_bringup sim.launch.py world:=pillars reference:=none
    ros2 launch dsim_bringup sim.launch.py state:=truth gui:=false

Vehicle parameters are read from ONE file (config/drone.yaml) and handed to
every node that needs them, so the controller, the referee and the Gazebo model
cannot disagree about what the drone weighs.
"""
import json
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# The workspace layout inside the container (see docker/compose.yaml).
WS = os.environ.get('DSIM_WS', '/ws')
CONFIG_DIR = os.path.join(WS, 'config')
WORLD_DIR = os.path.join(WS, 'worlds')
WEB_DIR = os.path.join(WS, 'web')


def _load_yaml(path):
    with open(path, 'r') as fh:
        return yaml.safe_load(fh)


def _vehicle_params():
    """Flatten config/drone.yaml into the parameter names the nodes expect."""
    cfg = _load_yaml(os.path.join(CONFIG_DIR, 'drone.yaml'))['drone']
    inertia = cfg['inertia']
    return {
        'vehicle.mass_kg': float(cfg['mass_kg']),
        'vehicle.gravity_m_s2': float(cfg['gravity_m_s2']),
        'vehicle.arm_length_m': float(cfg['arm_length_m']),
        'vehicle.motor_constant': float(cfg['rotor']['motor_constant']),
        'vehicle.moment_constant': float(cfg['rotor']['moment_constant']),
        'vehicle.max_rot_velocity': float(cfg['rotor']['max_rot_velocity']),
        'vehicle.inertia': [float(inertia['ixx']), float(inertia['iyy']), float(inertia['izz'])],
    }, cfg


def _sensor_params(raw_cfg):
    """Flatten config/drone.yaml's drone.sensors block for the sensors node.

    Same single-source rule as the vehicle parameters: the IMU noise in
    model.sdf, these figures, and scripts/check_sensors.py all come from PARAMS
    in scripts/gen_assets.py, so a sensor cannot be configured one way and
    checked against another.
    """
    s = raw_cfg['sensors']
    out = {}
    for group in ('tof', 'optical_flow', 'magnetometer'):
        for key, value in s[group].items():
            # A list stays a list (the magnetometer's world field is one
            # vector), everything else is one double.
            out[f'{group}.{key}'] = ([float(v) for v in value] if isinstance(value, list)
                                     else float(value))
    return out


def _mag_field(raw_cfg):
    """The magnetometer's world field from config/drone.yaml, or None.

    Tolerates both shapes the generator might emit: one list under
    `field_world_t`, or three scalars `field_world_t_x/_y/_z`. None means no
    magnetometer block at all, in which case the estimator runs yaw
    gyro-only and says so. A block that is present but carries the field in
    neither form is a broken config and is refused here, not degraded.
    """
    mag = (raw_cfg.get('sensors') or {}).get('magnetometer')
    if not mag:
        return None
    field = mag.get('field_world_t')
    if isinstance(field, list) and len(field) == 3:
        return [float(v) for v in field]
    scalars = [mag.get(f'field_world_t_{axis}') for axis in 'xyz']
    if all(v is not None for v in scalars):
        return [float(v) for v in scalars]
    raise RuntimeError(
        'config/drone.yaml has a drone.sensors.magnetometer block but no '
        'field_world_t (as a 3-list or as field_world_t_x/_y/_z); the '
        'estimator cannot take a heading from a field it does not know.')


def _estimator_params(raw_cfg, vehicle):
    """Flatten src/dsim_estimation/config/estimator.yaml for the estimator.

    Gravity comes from the VEHICLE config, not from a copy here, and the
    magnetometer's world field from the SENSOR config: the estimator must
    compare the compass against the same field the sensor is generated from,
    or its yaw is wrong by exactly the difference and nothing would say so.
    """
    est = _load_yaml(os.path.join(get_package_share_directory('dsim_estimation'),
                                  'config', 'estimator.yaml'))['estimator']
    out = {
        'use_sim_time': True,
        'gravity_m_s2': vehicle['vehicle.gravity_m_s2'],
        'position_source': str(est['position_source']),
        'mag_timeout_s': float(est['mag_timeout_s']),
        'truth_timeout_s': float(est['truth_timeout_s']),
    }
    for group in ('attitude', 'velocity'):
        for key, value in est[group].items():
            out[f'{group}.{key}'] = float(value)
    field = _mag_field(raw_cfg)
    if field is not None:
        out['mag.field_world_t'] = field
    return out


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    reference = LaunchConfiguration('reference').perform(context)
    use_rviz = LaunchConfiguration('rviz').perform(context)
    csv_path = LaunchConfiguration('csv').perform(context)
    viz = LaunchConfiguration('viz').perform(context)
    bind = LaunchConfiguration('bind').perform(context)
    web_port = LaunchConfiguration('web_port').perform(context)
    control = LaunchConfiguration('control').perform(context)
    radius = LaunchConfiguration('radius').perform(context)
    altitude = LaunchConfiguration('altitude').perform(context)
    period = LaunchConfiguration('period').perform(context)

    world_file = os.path.join(WORLD_DIR, f'{world}.sdf')
    if not os.path.exists(world_file):
        raise RuntimeError(
            f'world file not found: {world_file}. '
            f'Available: {sorted(f[:-4] for f in os.listdir(WORLD_DIR) if f.endswith(".sdf"))}')

    # The Gazebo world name is set inside the SDF and is NOT the file name;
    # the contact-sensor topic is namespaced by it, so read it rather than
    # assume it. Guessing here produces a bridge that silently carries nothing.
    import xml.etree.ElementTree as ET
    world_root = ET.parse(world_file).getroot().find('world')
    world_name = world_root.get('name')
    # The physics step, read from the same file the simulator reads. The
    # playback-speed control counts in steps, so a hard-coded 0.001 here would
    # make every speed wrong by the ratio between the two numbers, silently,
    # without any single step being incorrect.
    step_node = world_root.find('physics/max_step_size')
    if step_node is None or not step_node.text:
        raise RuntimeError(
            f'{world_file} declares no <physics><max_step_size>; the playback '
            f'speed control has nothing to count in.')
    step_size_s = float(step_node.text)
    sensors = LaunchConfiguration('sensors').perform(context)
    noise_seed = LaunchConfiguration('noise_seed').perform(context)
    state = LaunchConfiguration('state').perform(context)
    if state not in ('est', 'truth'):
        raise RuntimeError(f"state:={state} is not one of est | truth")
    if state == 'est' and sensors != 'true':
        # Refuse rather than fall back. With no sensors there is no estimate,
        # and a controller whose state source never publishes sits on the
        # ground holding position with its motors cut -- which looks like a
        # quiet, healthy sim right up until someone asks why it never took
        # off. Say so at launch instead.
        raise RuntimeError(
            'state:=est needs sensors:=true -- the estimator has nothing to '
            'estimate from. Either enable the sensors or fly on ground truth '
            'explicitly with state:=truth.')

    vehicle, raw_cfg = _vehicle_params()
    gains = _load_yaml(
        os.path.join(get_package_share_directory('dsim_bringup'),
                     'config', 'gains.yaml'))['controller']

    # ---- Gazebo -----------------------------------------------------------
    # gui:=false uses -s (server only). NOT --headless-rendering: that spins up
    # the render engine for offscreen cameras, and this model has no cameras, so
    # it would demand a GPU/GL context for nothing.
    gz_args = f'-r -v 3 {world_file}' + ('' if gui == 'true' else ' -s')
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    # ---- bridge -----------------------------------------------------------
    contact_topic = (f'/world/{world_name}/model/drone/link/base_link'
                     f'/sensor/crash_contact/contact')
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='drone_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/drone/odometry_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/drone/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
            f'{contact_topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/drone/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        remappings=[
            ('/model/drone/odometry_truth', '/drone/truth'),
            (contact_topic, '/drone/contacts'),
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ---- controller -------------------------------------------------------
    controller_params = {'use_sim_time': True}
    controller_params.update(vehicle)
    controller_params.update({
        'gains.kp': [float(v) for v in gains['gains']['kp']],
        'gains.kv': [float(v) for v in gains['gains']['kv']],
        'gains.kR': [float(v) for v in gains['gains']['kR']],
        'gains.komega': [float(v) for v in gains['gains']['komega']],
        'gains.max_tilt_rad': float(gains['gains']['max_tilt_rad']),
        'gains.min_thrust_n': float(gains['gains']['min_thrust_n']),
        'control_rate_hz': float(gains['control_rate_hz']),
        'odom_timeout_s': float(gains['odom_timeout_s']),
        'imu_timeout_s': float(gains['imu_timeout_s']),
        'start_armed': bool(gains['start_armed']),
        'odom.twist_in_body_frame': bool(gains['odom']['twist_in_body_frame']),
        'state.max_body_rate_rad_s': float(gains['state']['max_body_rate_rad_s']),
        'state.gyro_lowpass_tau_s': float(gains['state']['gyro_lowpass_tau_s']),
    })
    # state:=est points the controller's state subscription at the estimator.
    # A remap on THIS node only: the referee, the sensors and the viewer keep
    # reading /drone/truth, because they measure the vehicle rather than fly
    # it, and the estimator itself reads truth for position. Nothing in
    # dsim_control knows which it is getting -- that is what makes the two
    # runs comparable.
    controller = Node(
        package='dsim_control', executable='controller_node', name='dsim_controller',
        output='screen', parameters=[controller_params],
        remappings=[('/drone/truth', '/drone/state_est')] if state == 'est' else [],
    )

    # ---- referee ----------------------------------------------------------
    eval_params = [{
        'use_sim_time': True,
        'vehicle_radius_m': float(raw_cfg['radius_m']),
        # Nominal hover draw, used only to turn thrust into a relative energy
        # figure so plans can be compared on cost. It is not calibrated to any
        # real battery, so treat energy_wh as a comparison number, not watt-hours.
        'hover_power_w': 200.0,
        'vehicle.mass_kg': vehicle['vehicle.mass_kg'],
        'vehicle.motor_constant': vehicle['vehicle.motor_constant'],
        'csv_path': csv_path,
    }]
    obstacles_file = os.path.join(CONFIG_DIR, f'obstacles_{world}.yaml')
    if os.path.exists(obstacles_file):
        eval_params.append(obstacles_file)
    referee = Node(
        package='dsim_eval', executable='eval_node', name='dsim_eval',
        output='screen', parameters=eval_params,
    )

    nodes = [gz, bridge, controller, referee]

    # ---- simulated onboard sensors ----------------------------------------
    # Derived from ground truth with a configured error model. Separate from
    # the referee on purpose: these are allowed to be wrong, and the referee
    # never is.
    if sensors == 'true':
        sensor_params = {'use_sim_time': True, 'noise_seed': int(noise_seed)}
        sensor_params.update(_sensor_params(raw_cfg))
        nodes.append(Node(
            package='dsim_sensors', executable='sensors_node', name='dsim_sensors',
            output='screen', parameters=[sensor_params],
        ))

        # ---- state estimator ----------------------------------------------
        # Runs whenever the sensors do, whichever state the controller flies
        # on, so /drone/state_est can be compared against /drone/truth in a
        # state:=truth run too (scripts/check_estimator.py). Position is
        # still ground truth inside it; see estimator_node.cpp.
        nodes.append(Node(
            package='dsim_estimation', executable='estimator_node', name='dsim_estimator',
            output='screen', parameters=[_estimator_params(raw_cfg, vehicle)],
        ))

    # ---- optional built-in reference --------------------------------------
    if reference != 'none':
        nodes.append(Node(
            package='dsim_control', executable='reference_generator_node',
            name='drone_reference_generator', output='screen',
            parameters=[{'use_sim_time': True, 'mode': reference,
                         'radius_m': float(radius), 'altitude_m': float(altitude),
                         'period_s': float(period)}],
        ))

    # ---- the one write path into the simulator ----------------------------
    # Separate from the viewer on purpose. It owns a persistent gz-transport
    # connection, and it is the only thing in the system that sends Gazebo a
    # command. use_sim_time is FALSE and must be: it pauses the world to pace
    # it, and a node timed by the world it has paused never ticks again.
    if control == 'true':
        nodes.append(Node(
            package='dsim_simctl', executable='sim_control_node', name='dsim_simctl',
            output='screen',
            # gust_link is the link a gust pushes on, spelled as Gazebo names
            # it. It has no default in the node: a name that does not resolve
            # is not an error anywhere in the stack, it is a force that
            # silently never lands.
            parameters=[{'use_sim_time': False,
                         'world': world_name,
                         'step_size_s': step_size_s,
                         'gust_link': 'drone::base_link'}],
        ))

    # Tell the viewer which world is actually running. Without this its world
    # selector is cosmetic, and it could happily draw the pillar course while
    # the sim flies an empty one -- showing obstacles that are not there.
    if viz == 'true':
        try:
            with open(os.path.join(WEB_DIR, 'current.json'), 'w') as fh:
                json.dump({'world': world, 'reference': reference}, fh)
        except OSError as exc:
            print(f'[sim.launch] could not write current.json: {exc}')

    # ---- remote viewer ----------------------------------------------------
    # Streams state over a WebSocket for web/index.html to render, rather than
    # streaming video of a GUI. See docs/VIEWER.md.
    if viz == 'true':
        # One process, one port, one origin: static files + an SSE state stream.
        # This replaced rosbridge plus a separate http.server -- see the module
        # docstring in dsim_viz/viz_server.py for why.
        # Decimates the 250 Hz telemetry to ~30 Hz for the Python viewer.
        # See the header of dsim_eval/src/viz_relay_node.cpp for the measured
        # reason this is a separate C++ node and not a filter in the viewer.
        nodes.append(Node(
            package='dsim_eval', executable='viz_relay_node', name='dsim_viz_relay',
            output='screen',
            # state_topic is the Odometry the controller was remapped to, so
            # /drone/viz/state is what it consumed and check_telemetry.py can
            # hold |velocity_world| == |twist| against the right source.
            parameters=[{'use_sim_time': True, 'rate_hz': 30.0,
                         'state_topic': '/drone/state_est' if state == 'est'
                         else '/drone/truth'}],
        ))
        # use_sim_time is deliberately FALSE here, and it is the single biggest
        # cost in this launch if you get it wrong. Setting it True makes rclpy
        # subscribe to /clock, which the bridge publishes at the 1 ms physics
        # step -- 1000 messages a second into a Python process, costing ~50% of
        # a CPU core, for a clock this node never reads. Every timestamp the
        # viewer shows comes out of a message header, and the only sleep is the
        # stream's own pacing, which should be wall time anyway: a browser
        # refreshing at sim time would stutter whenever the physics did.
        nodes.append(Node(
            package='dsim_viz', executable='viz_server', name='dsim_viz',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=(['--port', web_port, '--bind', bind,
                        '--directory', WEB_DIR]
                       + (['--allow-control'] if control == 'true' else [])),
        ))

    if use_rviz == 'true':
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('dsim_bringup'), 'rviz', 'drone.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition('true'),
        ))

    return nodes


def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('dsim_description'), 'models')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty',
                              description='world file stem in worlds/ (empty, pillars)'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='run the Gazebo GUI'),
        DeclareLaunchArgument('reference', default_value='circle',
                              description='built-in trajectory: none | hover | circle | '
                                          'lemniscate | step. Use none when your own '
                                          'planner is publishing /drone/trajectory.'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('viz', default_value='true',
                              description='serve the browser viewer (static + SSE state)'),
        DeclareLaunchArgument('bind', default_value='0.0.0.0',
                              description='interface the viewer binds to. '
                                          'Set to your Tailscale IP to keep it off '
                                          'the local LAN.'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('control', default_value='true',
                              description='expose the viewer\'s pause and '
                              'playback-speed buttons (POST /control). This is '
                              'the only write path into the simulator from the '
                              'web port; set false to keep the server '
                              'read-only. It can pause/resume and set a '
                              'range-checked real-time factor, nothing else.'),
        DeclareLaunchArgument('radius', default_value='2.0',
                              description='built-in trajectory radius (m). '
                              'A 2 m circle collides with pillar_c in the '
                              'pillars world; 1.0 clears the whole course.'),
        DeclareLaunchArgument('altitude', default_value='1.5'),
        DeclareLaunchArgument('sensors', default_value='true',
                              description='publish the simulated downward '
                              'rangefinder (/drone/tof) and optical-flow '
                              'sensor (/drone/optical_flow). Both are derived '
                              'from ground truth with the noise model in '
                              'config/drone.yaml.'),
        DeclareLaunchArgument('state', default_value='est',
                              description='what the controller flies on: est '
                              '(the sensor-based estimate on /drone/state_est: '
                              'attitude from gyro + accel + mag, velocity from '
                              'accel + optical flow + rangefinder, position '
                              'still ground truth) or truth (/drone/truth, '
                              'perfect state). est is the default and needs '
                              'sensors:=true; the launch refuses est without '
                              'them. Only the controller is remapped -- the '
                              'referee, sensors and viewer always see truth.'),
        DeclareLaunchArgument('noise_seed', default_value='1',
                              description='RNG seed for the sensor noise. '
                              'Fixed by default, which gives a repeatable '
                              'starting point and a fixed noise distribution '
                              '-- not bit-identical streams, since one '
                              'generator feeds two timers and the interleaving '
                              'depends on callback order. Set 0 for a fresh '
                              'sequence every run.'),
        DeclareLaunchArgument('period', default_value='12.0',
                              description='seconds per lap of the built-in '
                              'trajectory. This is the knob that decides how '
                              'hard the vehicle has to work: a circle needs '
                              'bank = atan(4*pi^2*r / (T^2*g)), so r=1 m at '
                              'T=12 s is 6 deg of bank -- almost nothing, and '
                              'it LOOKS like nothing on screen. r=2 m at '
                              'T=4.5 s is 39 deg, right at the tilt clamp, '
                              'with the rotor thrusts visibly split.'),
        DeclareLaunchArgument('csv', default_value='',
                              description='path to write per-step metrics, e.g. /ws/logs/run.csv'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               f'{model_path}:{WORLD_DIR}:' +
                               os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        OpaqueFunction(function=launch_setup),
    ])
