"""Generate the Gazebo Harmonic model `ecps295_quad` and a flat test world (TechRoute §6A G1).

Same structure as ardupilot_gazebo's iris_with_ardupilot (blade LiftDrag + ArduPilotPlugin rotor velocity loop),
but sized from the FHB twin (§5.4): 295 g, 0.16 m wheelbase, 4" props. The blade area is solved so that the course
motor curve (MOT_THST_EXPO 0.52, MOT_SPIN_MIN/MAX 0.15/0.95) hovers at MOT_THST_HOVER = HOVER_THR.

    python make_quad.py [--blade-area A] [--hover-thr 0.32] [--max-omega 838]
"""

from __future__ import annotations

import argparse
import math
import pathlib

HERE = pathlib.Path(__file__).parent

ap = argparse.ArgumentParser()
ap.add_argument("--mass", type=float, default=0.295)            # kg, §11.2 nominal
ap.add_argument("--wheelbase", type=float, default=0.16)        # m, motor-to-motor diagonal
ap.add_argument("--inertia", type=float, nargs=3, default=[0.00020, 0.00020, 0.00035])
ap.add_argument("--prop-radius", type=float, default=0.0508)    # 4 in
ap.add_argument("--max-omega", type=float, default=838.0)       # rad/s at full PWM (Iris value; numerically proven)
ap.add_argument("--hover-thr", type=float, default=0.32)        # target learned MOT_THST_HOVER
ap.add_argument("--expo", type=float, default=0.52)             # MOT_THST_EXPO (course)
ap.add_argument("--spin", type=float, nargs=2, default=[0.15, 0.95])  # MOT_SPIN_MIN/MAX (course)
ap.add_argument("--motor-tau", type=float, default=0.016)       # s, rotor speed loop time constant
# The solved area (0.00205) hovered at learned MOT_THST_HOVER 0.371 (P-only rotor loop error + blade inflow);
# 0.00240 gives 0.323 with course_base.parm + gz_ecps295.parm (G1, 2026-10-03). Pass 0 to use the solved value.
ap.add_argument("--blade-area", type=float, default=0.00240, help="blade area in m^2; 0 = solve from hover point")
a = ap.parse_args()

RHO = 1.2041
A0, CLA, CDA = 0.3, 4.25, 0.10      # blade incidence / lift / drag slopes (Iris values)
CL = CLA * A0
R_CP = 0.7 * a.prop_radius           # blade centre of pressure
ARM = a.wheelbase / 2 / math.sqrt(2)  # motor x/y offset
ROTOR_M = 0.003                       # prop + bell
ROTOR_IZZ = 1.6e-6                    # 4" prop + bell, kg m^2
IMU_M = 0.001
BASE_M = a.mass - 4 * ROTOR_M - IMU_M

# hover PWM fraction from the ArduPilot motor curve: thrust t -> actuator x with (1-e) x + e x^2 = t
e, t = a.expo, a.hover_thr
x = (-(1 - e) + math.sqrt((1 - e) ** 2 + 4 * e * t)) / (2 * e)
pwm_hover = a.spin[0] + (a.spin[1] - a.spin[0]) * x
omega_hover = pwm_hover * a.max_omega
blade_lift = a.mass * 9.80665 / 8   # 4 rotors x 2 blades
area = a.blade_area or blade_lift / (0.5 * RHO * (omega_hover * R_CP) ** 2 * CL)
drag_torque = 2 * 0.5 * RHO * (omega_hover * R_CP) ** 2 * area * CDA * A0 * R_CP
p_gain = ROTOR_IZZ / a.motor_tau
print(f"hover PWM fraction {pwm_hover:.3f}, omega {omega_hover:.0f} rad/s, blade area {area:.5f} m^2, "
      f"rotor drag torque {drag_torque:.2e} N m -> P-only speed error {drag_torque / p_gain:.1f} rad/s; "
      f"thrust/weight at full PWM {(1 / pwm_hover) ** 2:.2f}")

# rotor i: (x, y, spin sign) -- ArduPilot quad-X order, same as Iris: 0 FR ccw, 1 BL ccw, 2 FL cw, 3 BR cw
ROTORS = [(ARM, -ARM, 1), (-ARM, ARM, 1), (ARM, ARM, -1), (-ARM, -ARM, -1)]
Z_ROTOR = 0.02


def inertia(ixx, iyy, izz):
    return (f"<inertia><ixx>{ixx:.3e}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{iyy:.3e}</iyy><iyz>0</iyz>"
            f"<izz>{izz:.3e}</izz></inertia>")


def rotor_link(i, x, y, s):
    color = "0.1 0.4 1 1" if s > 0 else "0.1 0.1 0.1 1"
    return f"""
    <link name="rotor_{i}">
      <pose>{x:.4f} {y:.4f} {Z_ROTOR} 0 0 0</pose>
      <inertial><mass>{ROTOR_M}</mass>{inertia(ROTOR_IZZ / 2, ROTOR_IZZ / 2, ROTOR_IZZ)}</inertial>
      <collision name="collision"><geometry><cylinder><length>0.003</length><radius>{a.prop_radius}</radius></cylinder></geometry></collision>
      <visual name="disc"><geometry><cylinder><length>0.002</length><radius>{a.prop_radius}</radius></cylinder></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse><transparency>0.5</transparency></material></visual>
      <visual name="blade"><geometry><box><size>{2 * a.prop_radius} 0.008 0.003</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
    </link>
    <joint name="rotor_{i}_joint" type="revolute">
      <child>rotor_{i}</child><parent>base_link</parent>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit><dynamics><damping>1e-7</damping></dynamics></axis>
    </joint>"""


def lift_drag(i, s, side):
    # blade at +cp moves along +y for a ccw rotor; mirrored for cw (same convention as Iris)
    fwd = s * side
    return f"""
    <plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
      <a0>{A0}</a0><alpha_stall>1.4</alpha_stall><cla>{CLA}</cla><cda>{CDA}</cda><cma>0.0</cma>
      <cla_stall>-0.025</cla_stall><cda_stall>0.0</cda_stall><cma_stall>0.0</cma_stall>
      <area>{area:.6f}</area><air_density>{RHO}</air_density>
      <cp>{side * R_CP:.4f} 0 0</cp><forward>0 {fwd} 0</forward><upward>0 0 1</upward>
      <link_name>rotor_{i}</link_name>
    </plugin>"""


def control(i, s):
    return f"""
      <control channel="{i}">
        <jointName>rotor_{i}_joint</jointName><useForce>1</useForce>
        <multiplier>{s * a.max_omega:.1f}</multiplier><offset>0</offset>
        <servo_min>1000</servo_min><servo_max>2000</servo_max>
        <type>VELOCITY</type>
        <p_gain>{p_gain:.3e}</p_gain><i_gain>0</i_gain><d_gain>0</d_gain><i_max>0</i_max><i_min>0</i_min>
        <cmd_max>0.05</cmd_max><cmd_min>-0.05</cmd_min>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>"""


model = f"""<?xml version="1.0"?>
<!-- generated by ecps295/gazebo/make_quad.py; edit the generator, not this file -->
<sdf version="1.9">
  <model name="ecps295_quad">
    <pose>0 0 0.03 0 0 0</pose>
    <link name="base_link">
      <inertial><mass>{BASE_M:.4f}</mass>{inertia(*a.inertia)}</inertial>
      <collision name="body"><geometry><box><size>0.09 0.05 0.04</size></box></geometry></collision>
      <collision name="legs"><pose>0 0 -0.02 0 0 0</pose><geometry><box><size>0.10 0.10 0.005</size></box></geometry></collision>
      <visual name="body"><geometry><box><size>0.09 0.05 0.035</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>
      <visual name="battery"><pose>0 0 -0.03 0 0 0</pose><geometry><box><size>0.075 0.035 0.02</size></box></geometry>
        <material><ambient>0.8 0.6 0.1 1</ambient><diffuse>0.8 0.6 0.1 1</diffuse></material></visual>
      <visual name="arm_a"><pose>0 0 0.01 0 0 {math.pi / 4:.5f}</pose><geometry><box><size>{a.wheelbase} 0.012 0.004</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
      <visual name="arm_b"><pose>0 0 0.01 0 0 {-math.pi / 4:.5f}</pose><geometry><box><size>{a.wheelbase} 0.012 0.004</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
      <visual name="nose"><pose>0.05 0 0 0 0 0</pose><geometry><box><size>0.01 0.02 0.02</size></box></geometry>
        <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual>
    </link>
    <link name="imu_link">
      <inertial><mass>{IMU_M}</mass>{inertia(1e-8, 1e-8, 1e-8)}</inertial>
      <sensor name="imu_sensor" type="imu">
        <gz_frame_id>imu_link</gz_frame_id>
        <pose degrees="true">0 0 0 180 0 0</pose>
        <always_on>1</always_on><update_rate>1000.0</update_rate>
      </sensor>
    </link>
    <joint name="imu_joint" type="fixed"><child>imu_link</child><parent>base_link</parent></joint>
{"".join(rotor_link(i, x, y, s) for i, (x, y, s) in enumerate(ROTORS))}
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
{"".join(lift_drag(i, s, side) for i, (_, _, s) in enumerate(ROTORS) for side in (1, -1))}
{"".join(f'''
    <plugin filename="gz-sim-apply-joint-force-system" name="gz::sim::systems::ApplyJointForce">
      <joint_name>rotor_{i}_joint</joint_name>
    </plugin>''' for i in range(4))}
    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9002</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>
      <no_time_sync>1</no_time_sync>
      <have_32_channels>0</have_32_channels>
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>imu_link::imu_sensor</imuName>
{"".join(control(i, s) for i, (_, _, s) in enumerate(ROTORS))}
    </plugin>
  </model>
</sdf>
"""

config = """<?xml version="1.0"?>
<model>
  <name>ecps295_quad</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>ECPS 295 course drone twin: MOD-L 4in X, 295 g, 2S (generated by make_quad.py)</description>
</model>
"""

world = """<?xml version="1.0"?>
<!-- flat test world for ecps295_quad (G1); UCI coordinates to match SITL --home -->
<sdf version="1.9">
  <world name="ecps295_flat">
    <physics name="1ms" type="ignore"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <scene><ambient>1 1 1</ambient><background>0.8 0.8 0.8</background><sky/></scene>
    <spherical_coordinates>
      <latitude_deg>33.6430</latitude_deg><longitude_deg>-117.8420</longitude_deg><elevation>20</elevation>
      <heading_deg>0</heading_deg><surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse><specular>0.8 0.8 0.8 1</specular><direction>-0.5 0.1 -0.9</direction></light>
    <model name="ground_plane"><static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>
      </link>
    </model>
    <include><uri>model://ecps295_quad</uri><pose degrees="true">0 0 0.03 0 0 90</pose></include>
  </world>
</sdf>
"""

mdir = HERE / "models" / "ecps295_quad"
mdir.mkdir(parents=True, exist_ok=True)
(mdir / "model.sdf").write_text(model)
(mdir / "model.config").write_text(config)
(HERE / "worlds").mkdir(exist_ok=True)
(HERE / "worlds" / "ecps295_flat.sdf").write_text(world)
print("wrote", mdir, "and worlds/ecps295_flat.sdf")
