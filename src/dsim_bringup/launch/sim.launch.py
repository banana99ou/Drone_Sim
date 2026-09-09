"""Bring up the quadrotor simulator.

    ros2 launch dsim_bringup sim.launch.py
    ros2 launch dsim_bringup sim.launch.py world:=pillars reference:=none
    ros2 launch dsim_bringup sim.launch.py state_mode:=perfect gui:=false

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


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    reference = LaunchConfiguration('reference').perform(context)
    use_rviz = LaunchConfiguration('rviz').perform(context)
    csv_path = LaunchConfiguration('csv').perform(context)
    viz = LaunchConfiguration('viz').perform(context)
    bind = LaunchConfiguration('bind').perform(context)
    web_port = LaunchConfiguration('web_port').perform(context)
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
    world_name = ET.parse(world_file).getroot().find('world').get('name')

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
            ('/model/drone/odometry_truth', '/drone/odom'),
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
        'start_armed': bool(gains['start_armed']),
        'odom.twist_in_body_frame': bool(gains['odom']['twist_in_body_frame']),
    })
    controller = Node(
        package='dsim_control', executable='controller_node', name='dsim_controller',
        output='screen', parameters=[controller_params],
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

    # ---- optional built-in reference --------------------------------------
    if reference != 'none':
        nodes.append(Node(
            package='dsim_control', executable='reference_generator_node',
            name='drone_reference_generator', output='screen',
            parameters=[{'use_sim_time': True, 'mode': reference,
                         'radius_m': float(radius), 'altitude_m': float(altitude),
                         'period_s': float(period)}],
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
        nodes.append(Node(
            package='dsim_viz', executable='viz_server', name='dsim_viz',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['--port', web_port, '--bind', bind,
                       '--directory', WEB_DIR],
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
        DeclareLaunchArgument('radius', default_value='2.0',
                              description='built-in trajectory radius (m). '
                              'A 2 m circle collides with pillar_c in the '
                              'pillars world; 1.0 clears the whole course.'),
        DeclareLaunchArgument('altitude', default_value='1.5'),
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
