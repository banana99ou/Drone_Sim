#!/usr/bin/env python3
"""Generate every file that must agree about the vehicle, from one source.

Three files describe the same quadrotor and MUST NOT disagree:

  src/dsim_description/models/drone/model.sdf   what Gazebo physically simulates
  config/drone.yaml                             what the controller assumes
  web/scene.json                                what the viewer draws

If the SDF says 1.5 kg and the YAML says 1.8, the controller is flying a
vehicle that does not exist and every tracking number is meaningless -- and
nothing would visibly break.

A second family comes from scenarios/*.json, which scripts/import_scenarios.py
generates from the planner's own SCENARIO_MAP. From each one this script emits
the Gazebo world (ground, light, and the vehicle at the plan's first point),
the referee's obstacle list, the viewer's scene entry and a straight-line seed
plan.

The obstacles are NOT in the world file. They have no collision geometry --
the referee scores clearance analytically -- and two scenarios move theirs
along cubics while three switch them off partway through, none of which a
constant-velocity rigid body can express. The referee publishes the obstacle
field on /drone/eval/clearance and the viewer draws that, so there is one
account of where anything is rather than two that can drift.

    python3 scripts/gen_assets.py            # regenerate
    python3 scripts/gen_assets.py --check    # fail if anything drifted
"""
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent

# ---------------------------------------------------------------------------
# Generic quadrotor. Round numbers chosen for good flying behaviour, NOT to
# match any real airframe -- this simulator deliberately models no specific
# hardware. Pick different numbers freely; everything downstream follows.
# ---------------------------------------------------------------------------
PARAMS = {
    "mass_kg":          1.5,
    "gravity_m_s2":     9.80665,
    "arm_length_m":     0.20,     # rotor hub to centre, X configuration
    "rotor_z_m":        0.02,     # rotor hub height above the body origin
    "body_box_m":       (0.16, 0.16, 0.08),
    "radius_m":         0.30,     # collision + planner clearance envelope
    "ixx":              0.015,
    "iyy":              0.015,
    "izz":              0.025,
    "rotor_mass_kg":    0.005,
    "motor_constant":   9.0e-06,  # thrust = k * omega^2  [N s^2/rad^2]
    "moment_constant":  0.016,    # yaw torque / thrust   [m]
    "max_rot_velocity": 1000.0,   # rad/s
    "time_constant_up": 0.02,
    "time_constant_down": 0.03,
    "drag_coefficient": 8.06428e-05,
    "rolling_moment_coefficient": 1.0e-06,
    "physics_step_s":   0.001,
    "control_rate_hz":  250.0,

    # --- IMU noise ---------------------------------------------------------
    # Modelled on a decent consumer MEMS IMU (MPU-6000 class) at this sensor
    # rate, from the data sheet's noise DENSITY integrated over the bandwidth:
    #   gyro   0.005 deg/s/sqrt(Hz) over 125 Hz  ->  sigma ~= 1.0e-3 rad/s
    #   accel  400 ug/sqrt(Hz)      over 125 Hz  ->  sigma ~= 4.4e-2 m/s^2
    # Derived rather than picked, so the numbers mean something and can be
    # argued with. Set them all to 0.0 and regenerate for clean sensors.
    #
    # The gyro figure matters more than it looks: the controller's rotational
    # loop reads this sensor directly, so gyro noise becomes torque noise at
    # k_omega times its magnitude. 1e-3 rad/s against body rates of ~1.8 rad/s
    # is 0.06% -- verified in scripts/check_sensors.py rather than assumed.
    "imu_gyro_noise_rad_s":    0.001,
    "imu_gyro_bias_rad_s":     0.0002,
    "imu_gyro_bias_stddev":    5.0e-05,
    "imu_accel_noise_m_s2":    0.044,
    "imu_accel_bias_m_s2":     0.02,
    "imu_accel_bias_stddev":   0.005,
    # Bias wanders with temperature; 300 s is plausible for a small airframe
    # warming up. Gazebo random-walks the bias with this correlation time.
    "imu_bias_correlation_s":  300.0,
    "imu_gyro_dynamic_bias":   1.0e-05,
    "imu_accel_dynamic_bias":  1.0e-03,

    # --- downward rangefinder (ToF) ----------------------------------------
    # A VL53L1X-class single-beam time-of-flight sensor: a few metres of
    # useful range, millimetre reporting resolution, and accuracy that
    # degrades with distance.
    "tof_min_range_m":         0.03,
    "tof_max_range_m":         4.0,
    "tof_fov_rad":             0.47,      # ~27 deg cone
    "tof_noise_m":             0.01,      # fixed part of the error
    "tof_noise_frac":          0.01,      # plus 1% of the measured range
    "tof_resolution_m":        0.001,     # reports whole millimetres
    "tof_rate_hz":             30.0,

    # --- optical flow ------------------------------------------------------
    # A PMW3901-class sensor: integrated angular flow plus a quality figure,
    # only usable within a height band and while roughly level.
    "flow_rate_hz":            50.0,
    "flow_noise_rad_s":        0.02,
    "flow_min_height_m":       0.10,
    "flow_max_height_m":       3.00,
    "flow_max_tilt_rad":       0.52,      # 30 deg; past this the ground
                                          # leaves the field of view

    # --- magnetometer ------------------------------------------------------
    # A consumer three-axis part (AK8963 / LIS3MDL class): 0.5 uT of white
    # noise per axis at 50 Hz, which is about one degree of heading in a
    # 30 uT horizontal field. NOT a Gazebo sensor: like the rangefinder and
    # the flow it is synthesised from the truth attitude in dsim_sensors, so
    # the model is a pure function that compiles on the host, and so the
    # hard-iron offset -- the error that actually decides whether a compass
    # is usable -- is modelled explicitly rather than not at all. Nothing
    # here reaches model.sdf. The Earth's field is one vector in the WORLD
    # frame: +x is magnetic north with the declination folded in, z is up,
    # so the vertical component is negative in the northern hemisphere; the
    # default dips 53 deg, roughly Korea. hard_iron_t is the magnitude of a
    # fixed body-frame offset in a seeded random direction; 0 is an ideal
    # compass, and 2.0e-06 (2 uT, ~4 deg of heading) is a realistic airframe.
    "mag_rate_hz":             50.0,
    "mag_noise_t":             5.0e-07,
    "mag_hard_iron_t":         0.0,
    "mag_field_world_t":       (3.0e-05, 0.0, -4.0e-05),
}

def derived():
    """Numbers implied by PARAMS. Printed so the flight envelope is never a
    mystery, and asserted below so an unflyable parameter set fails loudly."""
    p = PARAMS
    hover_total = p["mass_kg"] * p["gravity_m_s2"]
    hover_rotor = hover_total / 4.0
    max_rotor = p["motor_constant"] * p["max_rot_velocity"] ** 2
    hover_omega = math.sqrt(hover_rotor / p["motor_constant"])
    return {
        "hover_thrust_total_n": hover_total,
        "hover_thrust_rotor_n": hover_rotor,
        "max_thrust_rotor_n": max_rotor,
        "thrust_to_weight": 4.0 * max_rotor / hover_total,
        "hover_omega_rad_s": hover_omega,
        "rotor_utilisation": hover_omega / p["max_rot_velocity"],
    }


def check_flyable():
    d = derived()
    if d["thrust_to_weight"] < 1.5:
        raise SystemExit(
            f"unflyable parameters: thrust-to-weight {d['thrust_to_weight']:.2f} < 1.5")
    if d["rotor_utilisation"] > 0.85:
        raise SystemExit(
            f"no control margin: hover needs {d['rotor_utilisation']*100:.0f}% rotor speed")


# ---------------------------------------------------------------------------
# drone model SDF
# ---------------------------------------------------------------------------
def rotor_layout():
    """Rotor hubs and spin directions, in MIXER INDEX ORDER.

    One definition, consumed by everything that has an opinion about which
    rotor is which:

      * model.sdf       where Gazebo puts each link and its thrust plugin
      * web/scene.json  where the viewer draws each rotor's thrust arrow
      * dsim_control/mixer.hpp documents this same table, and
        test_control.cpp asserts the allocation matrix that follows from it

    Retyping it somewhere would let a viewer draw rotor 1's thrust on rotor 3,
    which makes a correct controller look broken -- the most expensive kind of
    bug, because you go looking in the wrong place.
    """
    d = round(PARAMS["arm_length_m"] / math.sqrt(2.0), 6)
    return [
        ("rotor_0",  d,  d, "ccw", 0),   # front-left
        ("rotor_1", -d, -d, "ccw", 1),   # back-right
        ("rotor_2",  d, -d, "cw",  2),   # front-right
        ("rotor_3", -d,  d, "cw",  3),   # back-left
    ]


def imu_noise():
    """Per-axis Gaussian noise for the IMU, from PARAMS.

    Written out per axis because that is the shape sdformat wants -- there is
    no way to say "the same on all three". Generated rather than hand-written
    so the six identical blocks cannot drift apart, which is exactly the sort
    of difference nobody notices: a vehicle with noise on two axes and not the
    third flies subtly differently and looks like a tuning problem.
    """
    p = PARAMS

    def axes(indent, stddev, bias_mean, bias_stddev, dyn_bias):
        pad = " " * indent
        out = []
        for axis in ("x", "y", "z"):
            out.append(
                f"{pad}<{axis}>\n"
                f"{pad}  <noise type=\"gaussian\">\n"
                f"{pad}    <mean>0.0</mean>\n"
                f"{pad}    <stddev>{stddev}</stddev>\n"
                f"{pad}    <bias_mean>{bias_mean}</bias_mean>\n"
                f"{pad}    <bias_stddev>{bias_stddev}</bias_stddev>\n"
                f"{pad}    <dynamic_bias_stddev>{dyn_bias}</dynamic_bias_stddev>\n"
                f"{pad}    <dynamic_bias_correlation_time>"
                f"{p['imu_bias_correlation_s']}"
                f"</dynamic_bias_correlation_time>\n"
                f"{pad}  </noise>\n"
                f"{pad}</{axis}>\n")
        return "".join(out)

    return (
        "        <imu>\n"
        "          <angular_velocity>\n"
        + axes(12, p["imu_gyro_noise_rad_s"], p["imu_gyro_bias_rad_s"],
               p["imu_gyro_bias_stddev"], p["imu_gyro_dynamic_bias"])
        + "          </angular_velocity>\n"
        "          <linear_acceleration>\n"
        + axes(12, p["imu_accel_noise_m_s2"], p["imu_accel_bias_m_s2"],
               p["imu_accel_bias_stddev"], p["imu_accel_dynamic_bias"])
        + "          </linear_acceleration>\n"
        "        </imu>\n")


def model_sdf():
    p = PARAMS
    d = round(p["arm_length_m"] / math.sqrt(2.0), 6)
    bx, by, bz = p["body_box_m"]
    body_mass = round(p["mass_kg"] - 4 * p["rotor_mass_kg"], 6)

    rotors = rotor_layout()

    links, plugins = [], []
    for name, x, y, dirn, num in rotors:
        colour = "0.9 0.3 0.1" if x > 0 else "0.2 0.2 0.25"
        links.append(f"""
    <link name="{name}">
      <pose>{x} {y} {p["rotor_z_m"]} 0 0 0</pose>
      <inertial>
        <mass>{p["rotor_mass_kg"]}</mass>
        <inertia>
          <ixx>1.0e-06</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1.7e-05</iyy><iyz>0</iyz><izz>1.7e-05</izz>
        </inertia>
      </inertial>
      <visual name="{name}_visual">
        <geometry><cylinder><radius>0.09</radius><length>0.006</length></cylinder></geometry>
        <material>
          <ambient>{colour} 1</ambient><diffuse>{colour} 1</diffuse>
        </material>
      </visual>
    </link>
    <joint name="{name}_joint" type="revolute">
      <parent>base_link</parent>
      <child>{name}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0.0</damping></dynamics>
      </axis>
    </joint>""")
        plugins.append(f"""
    <plugin filename="gz-sim-multicopter-motor-model-system"
            name="gz::sim::systems::MulticopterMotorModel">
      <robotNamespace>drone</robotNamespace>
      <jointName>{name}_joint</jointName>
      <linkName>{name}</linkName>
      <turningDirection>{dirn}</turningDirection>
      <timeConstantUp>{p["time_constant_up"]}</timeConstantUp>
      <timeConstantDown>{p["time_constant_down"]}</timeConstantDown>
      <maxRotVelocity>{p["max_rot_velocity"]}</maxRotVelocity>
      <motorConstant>{p["motor_constant"]}</motorConstant>
      <momentConstant>{p["moment_constant"]}</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>{num}</motorNumber>
      <rotorDragCoefficient>{p["drag_coefficient"]}</rotorDragCoefficient>
      <rollingMomentCoefficient>{p["rolling_moment_coefficient"]}</rollingMomentCoefficient>
      <motorSpeedPubTopic>motor_speed/{num}</motorSpeedPubTopic>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    </plugin>""")

    return f"""<?xml version="1.0"?>
<!-- GENERATED by scripts/gen_assets.py. Do not edit by hand.
     Generic quadrotor, X configuration, arm {p["arm_length_m"]} m (offset {d} m per axis).

         rotor_0 (ccw, FL)     rotor_2 (cw, FR)
                       \\  +x  /
                        [body]
                       /      \\
         rotor_3 (cw, BL)      rotor_1 (ccw, BR)
-->
<sdf version="1.9">
  <model name="drone">
    <pose>0 0 0.1 0 0 0</pose>

    <link name="base_link">
      <inertial>
        <mass>{body_mass}</mass>
        <inertia>
          <ixx>{p["ixx"]}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{p["iyy"]}</iyy><iyz>0</iyz><izz>{p["izz"]}</izz>
        </inertia>
      </inertial>

      <visual name="body_visual">
        <geometry><box><size>{bx} {by} {bz}</size></box></geometry>
        <material>
          <ambient>0.1 0.1 0.12 1</ambient><diffuse>0.18 0.18 0.22 1</diffuse>
        </material>
      </visual>

      <!-- Collision is the {p["radius_m"]} m envelope, not the small body box:
           this is the volume a path planner has to keep clear. -->
      <collision name="envelope">
        <geometry><cylinder><radius>{p["radius_m"]}</radius><length>{bz}</length></cylinder></geometry>
        <surface>
          <friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction>
        </surface>
      </collision>

      <sensor name="crash_contact" type="contact">
        <always_on>1</always_on>
        <update_rate>200</update_rate>
        <contact><collision>envelope</collision></contact>
      </sensor>

      <sensor name="imu_sensor" type="imu">
        <always_on>1</always_on>
        <update_rate>{p["control_rate_hz"]:.0f}</update_rate>
        <topic>drone/imu</topic>
{imu_noise()}      </sensor>
    </link>
{"".join(links)}
{"".join(plugins)}

    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>drone</robot_base_frame>
      <dimensions>3</dimensions>
      <odom_publish_frequency>250</odom_publish_frequency>
      <odom_topic>/model/drone/odometry_truth</odom_topic>
    </plugin>
  </model>
</sdf>
"""


def drone_yaml():
    p, d = PARAMS, derived()
    return f"""# GENERATED by scripts/gen_assets.py. Do not edit by hand -- edit PARAMS
# in that script, rerun it, and the Gazebo model is regenerated to match.
#
# Generic quadrotor. These numbers model NO specific hardware; they were picked
# to fly well. The simulator is self-consistent, which is what matters for
# comparing planners against each other.
#
# Flight envelope implied by these values:
#   hover thrust        {d["hover_thrust_total_n"]:.2f} N total, {d["hover_thrust_rotor_n"]:.2f} N per rotor
#   max thrust          {4*d["max_thrust_rotor_n"]:.2f} N total
#   thrust-to-weight    {d["thrust_to_weight"]:.2f}
#   hover rotor speed   {d["hover_omega_rad_s"]:.0f} rad/s ({d["rotor_utilisation"]*100:.0f}% of max)
#   max lateral accel   ~{p["gravity_m_s2"]*math.tan(0.7):.1f} m/s^2 at the 40 deg tilt clamp

drone:
  mass_kg: {p["mass_kg"]}
  gravity_m_s2: {p["gravity_m_s2"]}
  arm_length_m: {p["arm_length_m"]}
  radius_m: {p["radius_m"]}
  inertia:
    ixx: {p["ixx"]}
    iyy: {p["iyy"]}
    izz: {p["izz"]}
  rotor:
    count: 4
    # written with an explicit decimal point: YAML 1.1 parses "9e-06" as a
    # STRING, not a float, which only worked because the loader cast it.
    motor_constant: {p["motor_constant"]:.4e}
    moment_constant: {p["moment_constant"]}
    max_rot_velocity: {p["max_rot_velocity"]}

  # Sensors. The IMU noise here MUST match what model.sdf gives the Gazebo
  # sensor -- both come from the same PARAMS, and scripts/check_sensors.py
  # measures the live data against these numbers rather than trusting them.
  sensors:
    imu:
      rate_hz: {p["control_rate_hz"]}
      gyro_noise_rad_s: {p["imu_gyro_noise_rad_s"]:.4e}
      gyro_bias_rad_s: {p["imu_gyro_bias_rad_s"]:.4e}
      accel_noise_m_s2: {p["imu_accel_noise_m_s2"]:.4e}
      accel_bias_m_s2: {p["imu_accel_bias_m_s2"]:.4e}
    tof:
      min_range_m: {p["tof_min_range_m"]}
      max_range_m: {p["tof_max_range_m"]}
      fov_rad: {p["tof_fov_rad"]}
      noise_m: {p["tof_noise_m"]:.4e}
      noise_frac: {p["tof_noise_frac"]:.4e}
      resolution_m: {p["tof_resolution_m"]:.4e}
      rate_hz: {p["tof_rate_hz"]}
    optical_flow:
      rate_hz: {p["flow_rate_hz"]}
      noise_rad_s: {p["flow_noise_rad_s"]:.4e}
      min_height_m: {p["flow_min_height_m"]}
      max_height_m: {p["flow_max_height_m"]}
      max_tilt_rad: {p["flow_max_tilt_rad"]}
    # Not a Gazebo sensor: synthesised from the truth attitude by
    # dsim_sensors, so nothing here has a twin in model.sdf. The field is in
    # the WORLD frame, +x magnetic north, z up (vertical component negative
    # in the northern hemisphere); readings are body frame, in tesla.
    magnetometer:
      rate_hz: {p["mag_rate_hz"]}
      noise_t: {p["mag_noise_t"]:.4e}
      hard_iron_t: {p["mag_hard_iron_t"]:.4e}
      field_world_t: [{", ".join(f"{v:.4e}" for v in p["mag_field_world_t"])}]
"""


# ---------------------------------------------------------------------------
# worlds
# ---------------------------------------------------------------------------
COMMON_PLUGINS = """
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <!-- Lets an external force be applied to the airframe at runtime, which is
         how the viewer's gust button reaches the physics. Without this system
         the wrench topics exist for nobody and every push is a silent no-op.
         Only dsim_simctl publishes to them: see its header for why exactly one
         node owns the write path into Gazebo. -->
    <plugin filename="gz-sim-apply-link-wrench-system"
            name="gz::sim::systems::ApplyLinkWrench"/>"""

SCENE = """
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.82 0.9 1</background>
      <grid>true</grid>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.3 -0.9</direction>
    </light>"""

GROUND = """
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.3 0.32 0.35 1</ambient><diffuse>0.4 0.42 0.45 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def world(name, header="", spawn=(0.0, 0.0, 0.1)):
    sx, sy, sz = spawn
    physics = f"""
    <physics name="step" type="ignored">
      <max_step_size>{PARAMS["physics_step_s"]}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>"""
    return f"""<?xml version="1.0"?>
<!-- {header} -->
<sdf version="1.9">
  <world name="{name}">{physics}{COMMON_PLUGINS}{SCENE}{GROUND}
    <include>
      <uri>model://drone</uri>
      <name>drone</name>
      <pose>{sx} {sy} {sz} 0 0 0</pose>
    </include>
  </world>
</sdf>
"""


def scene_json():
    """Scene description for the browser viewer (web/index.html).

    Generated for the same reason as everything else here: the viewer draws the
    obstacles and the vehicle envelope, and if it drew a DIFFERENT course from
    the one Gazebo simulates, you would be watching a lie. Same source list,
    so it cannot drift.
    """
    import json
    p = PARAMS
    bx, by, bz = p["body_box_m"]
    scene = {
        "_generated_by": "scripts/gen_assets.py -- do not edit",
        "drone": {
            "radius": p["radius_m"],
            "arm": p["arm_length_m"],
            "body": [bx, by, bz],
            "rotor_radius": 0.09,
            # Hub positions in mixer index order (0 FL, 1 BR, 2 FR, 3 BL) and
            # the matching spin directions. The viewer draws one thrust arrow
            # per entry, indexed by the same number the mixer commands, so an
            # arrow cannot end up on the wrong arm. See rotor_layout().
            "rotors": [[x, y, p["rotor_z_m"]] for _, x, y, _, _ in rotor_layout()],
            "rotor_spin": [spin for _, _, _, spin, _ in rotor_layout()],
        },
        # The obstacle FIELD is not here: it arrives live on
        # /drone/eval/clearance, because obstacles move along curves and switch
        # off, and a second copy in a static file would be a second opinion.
        # What is here is what the referee does not send -- colours, the plan's
        # endpoints, and enough to aim the camera.
        # The bare world has no plan and no obstacles, so its extent is a
        # choice rather than a measurement: enough ground for the built-in
        # circle (2 m radius) and a step's worth of room around it.
        "worlds": {"empty": {"title": "Bare world", "spatial_dim": 3,
                             "colors": {}, "start": None, "end": None,
                             "bounds": [[-8.0, -8.0], [8.0, 8.0]]}},
    }
    for path in SCENARIOS:
        sc = load_scenario(path)
        scene["worlds"][sc["name"]] = scenario_scene_entry(sc)
    return json.dumps(scene, indent=2) + "\n"


# ---------------------------------------------------------------------------
# Space-time scenarios: obstacles that MOVE.
#
# The scenario file is the planner's dict (see scenarios/fence3d.json), read
# here rather than transcribed, so a number in it appears in exactly one
# place. Each obstacle is a sphere whose centre is pos0 + vel * t, with t the
# SCENARIO clock -- which starts at sim["start_s"] seconds of simulated time,
# because the vehicle has to fly from the pad to the start point first.
# ---------------------------------------------------------------------------
SCENARIOS = sorted((ROOT / "scenarios").glob("*.json"))


def load_scenario(path):
    import json
    sc = json.loads(pathlib.Path(path).read_text())
    for key in ("name", "spatial_dim", "start", "end", "T", "obstacles", "sim"):
        if key not in sc:
            raise SystemExit(f"{path}: scenario has no '{key}' "
                             f"(regenerate with scripts/import_scenarios.py)")
    if len(sc["start"]) != 4 or len(sc["end"]) != 4:
        raise SystemExit(f"{path}: start/end must be (x, y, z, t)")
    for o in sc["obstacles"]:
        if o["type"] not in ("sphere", "column"):
            raise SystemExit(f"{path}: obstacle {o['name']} has type {o['type']!r}; "
                             f"the referee knows sphere and column")
        cps = o["control_points"]
        if len(cps) < 2 or any(len(row) != 4 for row in cps):
            raise SystemExit(f"{path}: obstacle {o['name']} needs >= 2 control points "
                             f"of (x, y, z, t)")
        if cps[-1][3] < cps[0][3]:
            raise SystemExit(f"{path}: obstacle {o['name']} has times running backwards")
    return sc


def scenario_obstacle_yaml(sc):
    """The referee's obstacle list: the planner's canonical form, flattened.

    ROS 2 parameters have no array-of-struct type, so each obstacle's motion
    is one flat array of doubles, four per control point. The referee refuses
    a list whose length is not a multiple of four rather than score against
    half an obstacle.
    """
    names = ", ".join(f'"{o["name"]}"' for o in sc["obstacles"])
    lines = [
        f"# GENERATED by scripts/gen_assets.py from scenarios/{sc['name']}.json,",
        f"# which scripts/import_scenarios.py generates from the planner.",
        "# Do not hand-edit: the referee scores clearance against these numbers.",
        "/**:", "  ros__parameters:",
        f'    scenario.name: "{sc["name"]}"',
        f"    scenario.start_s: {float(sc['sim']['start_s'])}",
        f"    scenario.duration_s: {float(sc['T'])}",
        "    obstacles:", f"      names: [{names}]",
    ]
    for o in sc["obstacles"]:
        flat = ", ".join(str(round(float(c), 9)) for row in o["control_points"] for c in row)
        lines += [f"      {o['name']}:", f'        type: "{o["type"]}"',
                  f"        radius: {float(o['radius'])}",
                  f"        control_points: [{flat}]"]
    # Stations, flattened the same way: fixed ground points the vehicle must
    # stay VISIBLE from. Only a scenario that constrains line of sight has any,
    # and the key is emitted only then -- an empty list and an absent one mean
    # the same thing to the referee, but writing `stations: []` into every
    # scenario would suggest the constraint exists and is satisfied.
    stations = sc.get("stations") or []
    if stations:
        flat = ", ".join(str(round(float(c), 9)) for st in stations
                         for c in (list(st) + [0.0, 0.0, 0.0])[:3])
        lines += [f"    stations: [{flat}]"]
    return "\n".join(lines) + "\n"


def scenario_seed_plan(sc):
    """The planner's own initial guess: a straight line in space-time, N=8.

    This is what optimize_spacetime() starts from (build_initial_guess with
    init_curve straight), reproduced here because the Rust solver is not
    always built. It is a valid space-time Bezier and on most scenes it flies
    straight through something on purpose: the collision path gets exercised
    before any optimised plan exists, and a bridge that reports this plan as
    clear is broken.
    """
    import json
    n_cp = 9
    P = [[round((1 - s) * a + s * b, 9) for a, b in zip(sc["start"], sc["end"])]
         for s in (i / (n_cp - 1) for i in range(n_cp))]
    plan = {
        "_generated_by": "scripts/gen_assets.py -- the straight-line seed, not a solution",
        "scenario": sc["name"],
        "solver": "seed",
        "expect": seed_expectation(sc, P),
        "control_points": P,
    }
    text = json.dumps({k: v for k, v in plan.items() if k != "control_points"}, indent=1)
    rows = ",\n".join("  " + json.dumps(row) for row in P)
    return text[:-2] + ',\n "control_points": [\n' + rows + "\n ]\n}\n"


def obstacle_centre_at(o, t):
    """de Casteljau on the spatial coordinates, time affine in the parameter.

    The same construction as the referee's C++ and the planner's Python. Three
    implementations is two too many, but they are in three languages and the
    check_planner gate holds two of them against each other on a live run."""
    cps = [list(row) for row in o["control_points"]]
    t0, t1 = cps[0][3], cps[-1][3]
    span = t1 - t0
    s = min(1.0, max(0.0, (t - t0) / span)) if span > 1e-15 else 0.0
    while len(cps) > 1:
        cps = [[(1 - s) * a + s * b for a, b in zip(cps[i], cps[i + 1])]
               for i in range(len(cps) - 1)]
    return cps[0][:3]


def seed_expectation(sc, P):
    """First collision of the straight seed, predicted at 1 ms resolution.

    The seed is linear in every coordinate including time, so the vehicle is
    at start + s*(end - start) with s = t/T. Clearance counts the vehicle's
    own radius, as the referee does, and an obstacle outside its window is
    absent rather than far away.
    """
    x0, x1 = P[0], P[-1]
    T = x1[3] - x0[3]
    r_v = PARAMS["radius_m"]
    first = None
    n = max(1, int(T * 1000))
    for k in range(n + 1):
        t = k * T / n
        s = t / T if T > 0 else 0.0
        px, py, pz = (x0[i] + s * (x1[i] - x0[i]) for i in range(3))
        for o in sc["obstacles"]:
            cps = o["control_points"]
            if not cps[0][3] <= t <= cps[-1][3]:
                continue
            cx, cy, cz = obstacle_centre_at(o, t)
            d2 = (px - cx) ** 2 + (py - cy) ** 2
            if o["type"] != "column":
                d2 += (pz - cz) ** 2
            c = math.sqrt(d2) - o["radius"] - r_v
            if c < 0.0:
                if first is None:
                    first = {"with": o["name"], "t": round(t, 3), "depth_m": 0.0}
                if o["name"] == first["with"]:
                    first["depth_m"] = round(max(first["depth_m"], -c), 4)
    return {"first_hit": first} if first else {}


def scenario_bounds(sc):
    """The ground extent the scenario actually occupies, as [[x0,y0],[x1,y1]].

    Every obstacle control point as well as the plan's endpoints, because the
    viewer draws its ground grid over this and a grid sized from the endpoints
    alone would stop short of the obstacles beside them. The origin is always
    included: the world frame's axes are worth being able to see, and every
    scenario out of the planner starts near (0, 0) anyway.

    Not padded here. Padding is a drawing decision and belongs in the page.
    """
    xs, ys = [0.0], [0.0]
    for o in sc["obstacles"]:
        for cp in o["control_points"]:
            xs.append(float(cp[0]))
            ys.append(float(cp[1]))
    for key in ("start", "end"):
        point = sc.get(key)
        if point:
            xs.append(float(point[0]))
            ys.append(float(point[1]))
    return [[min(xs), min(ys)], [max(xs), max(ys)]]


def scenario_scene_entry(sc):
    """What the viewer needs that the referee's report does not carry: the
    colours, the plan's endpoints, and where the camera should look."""
    return {
        "title": sc.get("title", sc["name"]),
        "spatial_dim": sc["spatial_dim"],
        "colors": {o["name"]: o.get("color", "#e67e22") for o in sc["obstacles"]},
        "start": [float(v) for v in sc["start"]],
        "end": [float(v) for v in sc["end"]],
        "bounds": scenario_bounds(sc),
        "duration_s": float(sc["T"]),
    }


def scenario_targets():
    out = {}
    for path in SCENARIOS:
        sc = load_scenario(path)
        name = sc["name"]
        if path.stem != name:
            raise SystemExit(f"{path}: file is named {path.stem} but the scenario is {name}")
        spawn = tuple(float(v) for v in sc["sim"]["spawn"])
        out[f"worlds/{name}.sdf"] = lambda sc=sc, spawn=spawn: world(
            f"drone_{sc['name']}",
            # No "--" anywhere in here: it goes inside an XML comment, and a
            # double hyphen there is not well-formed. Caught by validate_xml.
            header=f"Space-time scenario '{sc['name']}': {sc['title']}. The obstacles "
                   f"are NOT in this file. They have no collision geometry and follow "
                   f"curves a rigid body cannot, so the referee publishes them and the "
                   f"viewer draws that. Generated by scripts/gen_assets.py.",
            spawn=spawn)
        out[f"config/obstacles_{name}.yaml"] = lambda sc=sc: scenario_obstacle_yaml(sc)
        out[f"plans/{name}_seed.json"] = lambda sc=sc: scenario_seed_plan(sc)
    return out


TARGETS = {
    "web/scene.json": scene_json,
    "src/dsim_description/models/drone/model.sdf": model_sdf,
    "config/drone.yaml": drone_yaml,
    "worlds/empty.sdf": lambda: world(
        "drone_empty",
        header="Bare world. No obstacles, so any tracking error here is the "
               "vehicle or the controller, never the plan."),
}
TARGETS.update(scenario_targets())


def validate_xml(rel, text):
    """Gazebo's failure mode for malformed SDF is an empty scene plus a line in
    a log nobody reads, which looks exactly like a launch problem. Fail here."""
    if not rel.endswith(".sdf"):
        return
    import xml.etree.ElementTree as ET
    try:
        ET.fromstring(text)
    except ET.ParseError as exc:
        raise SystemExit(f"generated {rel} is not valid XML: {exc}")


def main():
    check = "--check" in sys.argv
    check_flyable()
    stale = []
    for rel, make in TARGETS.items():
        path = ROOT / rel
        new = make()
        validate_xml(rel, new)
        old = path.read_text() if path.exists() else None
        if check:
            if old != new:
                stale.append(rel)
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(new)
            print(f"{'unchanged' if old == new else 'wrote    '}  {rel}")

    if check:
        if stale:
            print("STALE (run scripts/gen_assets.py):")
            for s in stale:
                print("  " + s)
            return 1
        print("all generated files match the source")
        return 0

    d = derived()
    print()
    print(f"  thrust-to-weight  {d['thrust_to_weight']:.2f}")
    print(f"  hover rotor speed {d['hover_omega_rad_s']:.0f} rad/s "
          f"({d['rotor_utilisation']*100:.0f}% of max)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
