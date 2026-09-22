#!/usr/bin/env python3
"""Generate every file that must agree about the vehicle, from one source.

Four files describe the same quadrotor and MUST NOT disagree:

  src/dsim_description/models/drone/model.sdf   what Gazebo physically simulates
  config/drone.yaml                             what the controller assumes
  worlds/*.sdf                                  the scenes
  config/obstacles_pillars.yaml                 what the referee scores against

and one more source feeds a second family: scenarios/*.json, a space-time
planning scenario in the planner's own format (start, end, moving spherical
obstacles). From each one this script emits the Gazebo world with the spheres
in motion, the referee's obstacle list WITH velocities, the viewer's scene
entry and a straight-line seed plan -- so the planner, the physics, the
referee and the picture cannot disagree about where the fence is at t=4.

If the SDF says 1.5 kg and the YAML says 1.8, the controller is flying a
vehicle that does not exist and every tracking number is meaningless -- and
nothing would visibly break. Same for the obstacle course: the referee would
score clearance against a world that is not the one being flown.

So all four are emitted here from PARAMS and OBSTACLES.

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

# name, type, x, y, z, sx, sy, sz     (cylinder: sx=radius, sz=length)
OBSTACLES = [
    ("pillar_a",   "cylinder", -3.0,  1.2, 1.5, 0.30, 0.30, 3.0),
    ("pillar_b",   "cylinder", -1.0, -1.5, 1.5, 0.30, 0.30, 3.0),
    ("pillar_c",   "cylinder",  1.2,  1.8, 1.5, 0.30, 0.30, 3.0),
    ("pillar_d",   "cylinder",  3.2, -1.0, 1.5, 0.30, 0.30, 3.0),
    ("wall_gap_l", "box",       0.0,  3.2, 1.2, 0.30, 2.4,  2.4),
    ("wall_gap_r", "box",       0.0, -3.2, 1.2, 0.30, 2.4,  2.4),
    ("low_beam",   "box",       5.0,  0.0, 2.2, 0.40, 6.0,  0.4),
]


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


def world(name, extra_models="", header="", spawn=(0.0, 0.0, 0.1)):
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
{extra_models}
    <include>
      <uri>model://drone</uri>
      <name>drone</name>
      <pose>{sx} {sy} {sz} 0 0 0</pose>
    </include>
  </world>
</sdf>
"""


def obstacle_models():
    out = []
    for name, typ, x, y, z, sx, sy, sz in OBSTACLES:
        geom = (f"<cylinder><radius>{sx}</radius><length>{sz}</length></cylinder>"
                if typ == "cylinder" else f"<box><size>{sx} {sy} {sz}</size></box>")
        out.append(f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry>{geom}</geometry></collision>
        <visual name="visual">
          <geometry>{geom}</geometry>
          <material>
            <ambient>0.55 0.27 0.20 1</ambient><diffuse>0.70 0.35 0.25 1</diffuse>
          </material>
        </visual>
      </link>
    </model>""")
    return "".join(out)


def obstacle_yaml():
    lines = [
        "# GENERATED by scripts/gen_assets.py together with worlds/pillars.sdf.",
        "# Do not hand-edit: the referee scores clearance against these numbers,",
        "# so drift from the world file would make the scores quietly wrong.",
        "/**:", "  ros__parameters:", "    obstacles:",
        "      names: [" + ", ".join(f'"{o[0]}"' for o in OBSTACLES) + "]",
    ]
    for name, typ, x, y, z, sx, sy, sz in OBSTACLES:
        lines += [f"      {name}:", f'        type: "{typ}"',
                  f"        pose: [{x}, {y}, {z}]", f"        size: [{sx}, {sy}, {sz}]"]
    return "\n".join(lines) + "\n"


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
        "worlds": {
            "empty": {"obstacles": []},
            "pillars": {"obstacles": [
                {"name": n, "type": ty, "pose": [x, y, z], "size": [sx, sy, sz]}
                for n, ty, x, y, z, sx, sy, sz in OBSTACLES
            ]},
        },
    }
    # Moving obstacles carry their velocity so the page can draw them where
    # they ARE; the live referee report (/drone/eval/clearance) is the
    # authority, and `pose` is only where they sit at scenario t=0.
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
    for key in ("name", "start", "end", "T", "obstacles", "sim"):
        if key not in sc:
            raise SystemExit(f"{path}: scenario has no '{key}'")
    if len(sc["start"]) != 4 or len(sc["end"]) != 4:
        raise SystemExit(f"{path}: start/end must be (x, y, z, t); this sim flies in "
                         f"three spatial dimensions and the planner's dim is spatial+1")
    for o in sc["obstacles"]:
        if "t_start" in o or "t_end" in o:
            # A window cannot be expressed with a constant-velocity Gazebo body,
            # and a sphere that is drawn but not scored (or the reverse) is the
            # exact disagreement this generator exists to prevent.
            raise SystemExit(f"{path}: obstacle {o.get('name')} has a time window; "
                             "the sim cannot stage an obstacle appearing or vanishing")
        if len(o["pos0"]) != 3 or len(o["vel"]) != 3:
            raise SystemExit(f"{path}: obstacle {o.get('name')} is not 3-D")
    return sc


def scenario_world_models(sc):
    """Visual-only spheres flying at constant velocity.

    No <collision>: the referee scores clearance analytically (dsim_eval), so a
    contact here would add nothing but a second, differently-shaped opinion
    and thirteen more bodies for the contact solver to worry about.

    Motion is left to the physics rather than scripted, through Gazebo's
    VelocityControl system with an initial velocity -- gravity off, so the
    commanded velocity is the whole motion. The spheres start moving at sim
    t=0, so they are placed at pos0 - vel*start_s and pass through pos0 exactly
    when the scenario clock reads zero.

    Each one also reports its own pose (OdometryPublisher, 20 Hz, on
    /model/<name>/odometry) and the referee holds that against pos0 + vel*t.
    That check is what makes this construction trustworthy, not this comment.
    Per model rather than the world's dynamic_pose/info because the ros_gz
    bridge drops entity names when it converts Pose_V to TFMessage --
    measured: every child_frame_id arrived empty -- and a pose with no name
    cannot be checked against anything. OdometryPublisher rather than
    PosePublisher because the latter goes silent after a world reset (it
    throttles against a last-publish time that the reset leaves in the
    future) and the check would be dead for the rest of the session without
    anything saying so; the odometry one is what the vehicle's own truth uses
    and survives resets, measured.
    """
    t0 = float(sc["sim"]["start_s"])
    out = []
    for o in sc["obstacles"]:
        x0, y0, z0 = (float(o["pos0"][i]) - float(o["vel"][i]) * t0 for i in range(3))
        vx, vy, vz = (float(v) for v in o["vel"])
        r = float(o["r"])
        rgb = _hex_rgb(o.get("color", "#e67e22"))
        out.append(f"""
    <model name="{o['name']}">
      <pose>{x0:.6f} {y0:.6f} {z0:.6f} 0 0 0</pose>
      <link name="link">
        <gravity>false</gravity>
        <inertial><mass>1.0</mass>
          <inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <visual name="visual">
          <geometry><sphere><radius>{r}</radius></sphere></geometry>
          <material>
            <ambient>{rgb} 0.55</ambient><diffuse>{rgb} 0.55</diffuse>
          </material>
        </visual>
      </link>
      <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
        <initial_linear>{vx} {vy} {vz}</initial_linear>
      </plugin>
      <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
        <odom_frame>world</odom_frame>
        <robot_base_frame>{o['name']}</robot_base_frame>
        <dimensions>3</dimensions>
        <odom_publish_frequency>20</odom_publish_frequency>
        <odom_topic>/model/{o['name']}/odometry</odom_topic>
      </plugin>
    </model>""")
    return "".join(out)


def _hex_rgb(h):
    h = h.lstrip("#")
    return " ".join(f"{int(h[i:i + 2], 16) / 255:.3f}" for i in (0, 2, 4))


def scenario_obstacle_yaml(sc):
    """The referee's list, same shape as obstacle_yaml() plus `vel`, and the
    scenario clock's origin so the referee evaluates pos0 + vel*t at the same
    t the planner meant."""
    names = ", ".join(f'"{o["name"]}"' for o in sc["obstacles"])
    lines = [
        f"# GENERATED by scripts/gen_assets.py from scenarios/{sc['name']}.json,",
        f"# together with worlds/{sc['name']}.sdf. Do not hand-edit.",
        "/**:", "  ros__parameters:",
        f"    scenario.start_s: {float(sc['sim']['start_s'])}",
        f"    scenario.duration_s: {float(sc['T'])}",
        "    obstacles:", f"      names: [{names}]",
    ]
    for o in sc["obstacles"]:
        lines += [f"      {o['name']}:", '        type: "sphere"',
                  f"        pose: [{', '.join(str(round(float(v), 9)) for v in o['pos0'])}]",
                  f"        size: [{float(o['r'])}]",
                  f"        vel: [{', '.join(str(round(float(v), 9)) for v in o['vel'])}]"]
    return "\n".join(lines) + "\n"


def scenario_seed_plan(sc):
    """The planner's own initial guess: a straight line in space-time, N=8.

    This is what optimize_spacetime() starts from (build_initial_guess with
    init_curve straight), reproduced here because the Rust solver does not
    build on this host. It is a valid space-time Bezier and it flies THROUGH
    the fence on purpose: the collision path gets exercised before any
    optimised plan exists, and a bridge that reports this plan as clear is
    broken.
    """
    import json
    n_cp = 9
    P = [[round((1 - s) * a + s * b, 9) for a, b in zip(sc["start"], sc["end"])]
         for s in (i / (n_cp - 1) for i in range(n_cp))]
    plan = {
        "_generated_by": "scripts/gen_assets.py -- the straight-line seed, not a solution",
        "scenario": sc["name"],
        "solver": "seed",
        # What flying this line through the scenario must produce, computed
        # here in Python from the same start/end/obstacles. The referee
        # computes it again in C++ on the live run (dsim_eval/obstacle.hpp),
        # and scripts/check_planner.py holds the two against each other.
        "expect": seed_expectation(sc, P),
        "control_points": P,
    }
    # One control point per line: a 9x4 table is readable that way and not
    # as 36 lines of one number each.
    text = json.dumps({k: v for k, v in plan.items() if k != "control_points"}, indent=1)
    rows = ",\n".join("  " + json.dumps(row) for row in P)
    return text[:-2] + ',\n "control_points": [\n' + rows + "\n ]\n}\n"


def seed_expectation(sc, P):
    """First collision of the straight seed, predicted at 1 ms resolution.

    The seed is linear in every coordinate including time, so the vehicle is
    at start + s*(end - start) with s = t/T. Clearance counts the vehicle's
    own radius, as the referee does.
    """
    x0, x1 = P[0], P[-1]
    T = x1[3] - x0[3]
    r_v = PARAMS["radius_m"]
    first = None
    n = int(T * 1000)
    for k in range(n + 1):
        t = k * T / n
        s = t / T
        px, py, pz = (x0[i] + s * (x1[i] - x0[i]) for i in range(3))
        for o in sc["obstacles"]:
            cx, cy, cz = (o["pos0"][i] + o["vel"][i] * t for i in range(3))
            c = math.sqrt((px - cx) ** 2 + (py - cy) ** 2 + (pz - cz) ** 2) - o["r"] - r_v
            if c < 0.0:
                if first is None:
                    first = {"with": o["name"], "t": round(t, 3), "depth_m": 0.0}
                if o["name"] == first["with"]:
                    first["depth_m"] = round(max(first["depth_m"], -c), 4)
    return {"first_hit": first} if first else {}


def scenario_scene_entry(sc):
    return {"obstacles": [
        {"name": o["name"], "type": "sphere", "pose": [float(v) for v in o["pos0"]],
         "size": [float(o["r"])], "vel": [float(v) for v in o["vel"]],
         "color": o.get("color", "#e67e22")}
        for o in sc["obstacles"]],
        "start": [float(v) for v in sc["start"]],
        "end": [float(v) for v in sc["end"]]}


def scenario_spawn(sc):
    """The pad: on the ground directly under the plan's start point.

    The transit from the pad to `start` is not part of the plan and nothing
    certifies it, so it is made trivial -- a vertical climb. The first
    version spawned at the origin, and the diagonal transit to fence3d's
    start ran along the fence row while it was still sweeping in from its
    t=-start_s position: five hits before the plan had begun.
    """
    return (float(sc["start"][0]), float(sc["start"][1]), 0.1)


def check_pad_is_clear(sc):
    """Refuse a scenario whose obstacles sweep through the pad or the climb
    before the plan starts. Between sim t=0 and start_s the vehicle is
    somewhere on the vertical line from the pad to `start`; every obstacle
    must clear that whole segment by the vehicle radius the whole time."""
    x, y, _ = scenario_spawn(sc)
    z_top = float(sc["start"][2])
    t0 = float(sc["sim"]["start_s"])
    r_v = PARAMS["radius_m"]
    for k in range(int(t0 * 100) + 1):
        t = -t0 + k / 100.0
        for o in sc["obstacles"]:
            cx, cy, cz = (o["pos0"][i] + o["vel"][i] * t for i in range(3))
            dz = max(0.0, cz - z_top, 0.0 - cz)   # distance to the segment in z
            d = math.sqrt((x - cx) ** 2 + (y - cy) ** 2 + dz ** 2) - o["r"] - r_v
            if d < 0.0:
                raise SystemExit(
                    f"scenarios/{sc['name']}.json: obstacle {o['name']} is {-d:.2f} m into "
                    f"the pad-to-start climb at scenario t={t:.2f} s (sim t={t + t0:.2f}); "
                    f"the transit is not part of the plan, so change sim.start_s or the "
                    f"obstacle rather than fly into it before the run begins")


def scenario_targets():
    out = {}
    for path in SCENARIOS:
        sc = load_scenario(path)
        name = sc["name"]
        if path.stem != name:
            raise SystemExit(f"{path}: file is named {path.stem} but the scenario is {name}")
        check_pad_is_clear(sc)
        out[f"worlds/{name}.sdf"] = lambda sc=sc: world(
            f"drone_{sc['name']}", scenario_world_models(sc),
            header=f"Space-time scenario '{sc['name']}': moving spherical obstacles, "
                   f"visual only. Generated from scenarios/{sc['name']}.json "
                   f"by scripts/gen_assets.py; the referee scores the same list.",
            spawn=scenario_spawn(sc))
        out[f"config/obstacles_{name}.yaml"] = lambda sc=sc: scenario_obstacle_yaml(sc)
        out[f"plans/{name}_seed.json"] = lambda sc=sc: scenario_seed_plan(sc)
    return out


TARGETS = {
    "web/scene.json": scene_json,
    "src/dsim_description/models/drone/model.sdf": model_sdf,
    "config/drone.yaml": drone_yaml,
    "config/obstacles_pillars.yaml": obstacle_yaml,
    "worlds/empty.sdf": lambda: world(
        "drone_empty",
        header="Bare world. No obstacles, so any tracking error here is the "
               "vehicle or the controller, never the plan."),
    "worlds/pillars.sdf": lambda: world(
        "drone_pillars", obstacle_models(),
        header="Obstacle course. Generated with config/obstacles_pillars.yaml "
               "from one source list. See scripts/gen_assets.py."),
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
