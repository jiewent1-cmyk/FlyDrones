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
# G2: ELP OV7725 front camera (§1 FHB, §4.2 C1/C2)
ap.add_argument("--camera", choices=["wide", "pinhole", "none"], default="wide",
                help="wide = wideanglecamera with equidistant lens (C1); pinhole = plain camera for comparison")
ap.add_argument("--cam-hfov", type=float, default=120.0)        # deg
ap.add_argument("--cam-tilt", type=float, default=12.0)         # deg, nose-down mount (C2)
ap.add_argument("--cam-res", type=int, nargs=2, default=[640, 480])
ap.add_argument("--cam-rate", type=float, default=60.0)         # Hz (MJPEG 60 fps)
ap.add_argument("--cam-noise", type=float, default=0.007)       # gaussian stddev on [0,1] pixel values
# mount position (§4.2 C4 estimate): ~30 mm ahead of the front motor line, ~15 mm below the prop plane
ap.add_argument("--cam-ahead", type=float, default=0.030)       # m ahead of the front motors
ap.add_argument("--cam-below", type=float, default=0.015)       # m below the prop plane
a = ap.parse_args()

RHO = 1.2041
A0, CLA, CDA = 0.3, 4.25, 0.10      # blade incidence / lift / drag slopes (Iris values)
CL = CLA * A0
R_CP = 0.7 * a.prop_radius           # blade centre of pressure
ARM = a.wheelbase / 2 / math.sqrt(2)  # motor x/y offset
ROTOR_M = 0.003                       # prop + bell
ROTOR_IZZ = 1.6e-6                    # 4" prop + bell, kg m^2
Z_ROTOR = 0.02                        # prop plane above the body origin
CAM_M = 0.010 if a.camera != "none" else 0.0   # ELP board after trimming the cable, 8-12 g (§11.2)
# IMU and camera sensors sit on base_link itself. A 10 g camera link on a `fixed` joint made the vehicle hover at
# 15% less thrust (MOT_THST_HOVER 0.283 vs 0.323, same total mass) -- a joint-constraint artefact, so no extra links.
BASE_M = a.mass - 4 * ROTOR_M
CAM_X, CAM_Z = ARM + a.cam_ahead, Z_ROTOR - a.cam_below
CG_X = CAM_M * CAM_X / BASE_M                  # camera shifts the body CG forward a little
CAM_TOPIC = "ecps295/camera"

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
      <visual name="disc"><transparency>0.75</transparency><geometry><cylinder><length>0.002</length><radius>{a.prop_radius}</radius></cylinder></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      <visual name="blade"><geometry><box><size>{2 * a.prop_radius} 0.008 0.003</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
    </link>
    <joint name="rotor_{i}_joint" type="revolute">
      <child>rotor_{i}</child><parent>base_link</parent>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit><dynamics><damping>1e-7</damping></dynamics></axis>
    </joint>"""


def camera_parts():
    if a.camera == "none":
        return ""
    w, h = a.cam_res
    if a.camera == "wide":
        sensor_type = "wideanglecamera"
        # equidistant r = f*theta (fisheye, §4.2 C1); scale_to_hfov keeps the requested hfov across the image width
        lens = ("<lens><type>equidistant</type><scale_to_hfov>true</scale_to_hfov>"
                f"<cutoff_angle>{math.radians(90):.4f}</cutoff_angle><env_texture_size>1024</env_texture_size></lens>")
    else:
        sensor_type, lens = "camera", ""
    return f"""
      <visual name="cam"><pose>{CAM_X:.4f} 0 {CAM_Z:.4f} 0 0 0</pose><geometry><box><size>0.012 0.03 0.03</size></box></geometry>
        <material><ambient>0 0.6 0 1</ambient><diffuse>0 0.6 0 1</diffuse></material></visual>
      <sensor name="front_camera" type="{sensor_type}">
        <pose>{CAM_X + 0.006:.4f} 0 {CAM_Z:.4f} 0 {math.radians(a.cam_tilt):.5f} 0</pose>
        <always_on>1</always_on><update_rate>{a.cam_rate}</update_rate><topic>{CAM_TOPIC}</topic>
        <camera>
          <horizontal_fov>{math.radians(a.cam_hfov):.5f}</horizontal_fov>
          <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
          <clip><near>0.02</near><far>200</far></clip>
          <noise><type>gaussian</type><mean>0</mean><stddev>{a.cam_noise}</stddev></noise>
          {lens}
        </camera>
      </sensor>"""


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
      <inertial><pose>{CG_X:.4f} 0 0 0 0 0</pose><mass>{BASE_M:.4f}</mass>{inertia(*a.inertia)}</inertial>
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
      <sensor name="imu_sensor" type="imu">
        <gz_frame_id>base_link</gz_frame_id>
        <pose degrees="true">0 0 0 180 0 0</pose>
        <always_on>1</always_on><update_rate>1000.0</update_rate>
      </sensor>
{camera_parts()}
    </link>
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
      <imuName>base_link::imu_sensor</imuName>
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

WORLD_TMPL = """<?xml version="1.0"?>
<!-- generated by ecps295/gazebo/make_quad.py: NAME; UCI coordinates to match SITL --home -->
<sdf version="1.9">
  <world name="NAME">
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
EXTRA
    <include><uri>model://ecps295_quad</uri><pose degrees="true">0 0 0.03 0 0 90</pose></include>
  </world>
</sdf>
"""

mdir = HERE / "models" / "ecps295_quad"
mdir.mkdir(parents=True, exist_ok=True)
(mdir / "model.sdf").write_text(model)
(mdir / "model.config").write_text(config)
(HERE / "worlds").mkdir(exist_ok=True)


def tape_bands(size, w=0.05, shade=0.18):
    """Dark tape: two horizontal rings and two vertical bands each way, i.e. a grid on every face."""
    sx, sy, sz = size
    e = 0.004
    c = f"{shade} {shade} {shade}"
    bands = []
    for k, f in enumerate((-0.25, 0.25)):
        bands.append((f"ring{k}", (0, 0, f * sz), (sx + e, sy + e, w)))
        bands.append((f"bx{k}", (f * sx, 0, 0), (w, sy + e, sz + e)))
        bands.append((f"by{k}", (0, f * sy, 0), (sx + e, w, sz + e)))
    return "".join(f"""
        <visual name="tape_{n}"><pose>{p[0]:.3f} {p[1]:.3f} {p[2]:.3f} 0 0 0</pose><geometry><box><size>{q[0]:.3f} {q[1]:.3f} {q[2]:.3f}</size></box></geometry>
          <material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material></visual>""" for n, p, q in bands)


def box(name, xyz, size, rgb, static=True, tape=False):
    c = " ".join(f"{v:.2f}" for v in rgb)
    return (f"""
    <model name="{name}"><static>{str(static).lower()}</static><pose>{xyz[0]} {xyz[1]} {xyz[2]} 0 0 0</pose>
      <link name="link"><collision name="c"><geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>
          <material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material></visual>{tape_bands(size) if tape else ""}</link></model>""")


def checker_floor(n=10, tile=0.61, lo=0.30, hi=0.55):
    """24-inch two-tone EVA mat (§4.3), n x n tiles centred on the origin, as one static model."""
    vis = []
    for i in range(n):
        for j in range(n):
            g = hi if (i + j) % 2 else lo
            x, y = (i - n / 2 + 0.5) * tile, (j - n / 2 + 0.5) * tile
            vis.append(f"""<visual name="t{i}_{j}"><pose>{x:.3f} {y:.3f} 0.001 0 0 0</pose>
          <geometry><box><size>{tile} {tile} 0.002</size></box></geometry>
          <material><ambient>{g} {g} {g} 1</ambient><diffuse>{g} {g} {g} 1</diffuse></material></visual>""")
    return f"""
    <model name="eva_mat"><static>true</static><link name="link">{"".join(vis)}</link></model>"""


# camera test: drone faces world +y; cardboard boxes ahead, a pole to show fisheye bending, coloured markers at the edges
CARDBOARD = (0.65, 0.48, 0.30)


def camtest_extra(tape: bool) -> str:
    return (checker_floor()
            + box("box_a", (0.0, 1.5, 0.25), (0.5, 0.5, 0.5), CARDBOARD, tape=tape)
            + box("box_b", (1.0, 2.2, 0.30), (0.6, 0.45, 0.6), CARDBOARD, tape=tape)
            + box("box_c", (-1.2, 2.6, 0.25), (0.45, 0.6, 0.5), CARDBOARD, tape=tape)
            + box("pole", (0.6, 1.0, 1.0), (0.04, 0.04, 2.0), (0.9, 0.1, 0.1))
            + box("marker_left", (-2.4, 1.2, 0.5), (0.2, 0.2, 1.0), (0.1, 0.3, 0.9))
            + box("marker_right", (2.4, 1.2, 0.5), (0.2, 0.2, 1.0), (0.1, 0.8, 0.2)))
(HERE / "worlds" / "ecps295_flat.sdf").write_text(WORLD_TMPL.replace("NAME", "ecps295_flat").replace("EXTRA", ""))
for wname, tape in (("ecps295_camtest", False), ("ecps295_camtest_tape", True)):
    (HERE / "worlds" / f"{wname}.sdf").write_text(WORLD_TMPL.replace("NAME", wname).replace("EXTRA", camtest_extra(tape)))
print("wrote", mdir, "and worlds/ecps295_flat.sdf, ecps295_camtest.sdf, ecps295_camtest_tape.sdf")
