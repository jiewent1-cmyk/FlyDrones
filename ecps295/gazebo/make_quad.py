"""Generate the Gazebo Harmonic model `ecps295_quad` and a flat test world (TechRoute §6A G1).

Same structure as ardupilot_gazebo's iris_with_ardupilot (blade LiftDrag + ArduPilotPlugin rotor velocity loop),
but sized from the FHB twin (§5.4): 295 g, 0.16 m wheelbase, 4" props. The blade area is solved so that the course
motor curve (MOT_THST_EXPO 0.52, MOT_SPIN_MIN/MAX 0.15/0.95) hovers at MOT_THST_HOVER = HOVER_THR.

    python make_quad.py [--blade-area A] [--hover-thr 0.32] [--max-omega 838]
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib

HERE = pathlib.Path(__file__).parent

ap = argparse.ArgumentParser()
ap.add_argument("--mass", type=float, default=0.295)  # kg, §11.2 nominal
ap.add_argument("--wheelbase", type=float, default=0.16)  # m, motor-to-motor diagonal
ap.add_argument("--inertia", type=float, nargs=3, default=[0.00020, 0.00020, 0.00035])
ap.add_argument("--prop-radius", type=float, default=0.0508)  # 4 in
ap.add_argument("--max-omega", type=float, default=838.0)  # rad/s at full PWM (Iris value; numerically proven)
ap.add_argument("--hover-thr", type=float, default=0.32)  # target learned MOT_THST_HOVER
ap.add_argument("--expo", type=float, default=0.52)  # MOT_THST_EXPO (course)
ap.add_argument("--spin", type=float, nargs=2, default=[0.15, 0.95])  # MOT_SPIN_MIN/MAX (course)
ap.add_argument("--motor-tau", type=float, default=0.016)  # s, rotor speed loop time constant
# The solved area (0.00205) hovered at learned MOT_THST_HOVER 0.371 (P-only rotor loop error + blade inflow);
# 0.00240 gives 0.323 with course_base.parm + gz_ecps295.parm (G1, 2026-10-03). Pass 0 to use the solved value.
ap.add_argument("--blade-area", type=float, default=0.00240, help="blade area in m^2; 0 = solve from hover point")
# G2: ELP OV7725 front camera (§1 FHB, §4.2 C1/C2)
ap.add_argument(
    "--camera",
    choices=["wide", "pinhole", "none"],
    default="wide",
    help="wide = wideanglecamera with equidistant lens (C1); pinhole = plain camera for comparison",
)
ap.add_argument("--cam-hfov", type=float, default=120.0)  # deg
ap.add_argument("--cam-tilt", type=float, default=12.0)  # deg, nose-down mount (C2)
ap.add_argument("--cam-res", type=int, nargs=2, default=[640, 480])
ap.add_argument("--cam-rate", type=float, default=60.0)  # Hz (MJPEG 60 fps)
ap.add_argument("--cam-env-tex", type=int, default=1024, help="wide-angle cube map face size (512 did not raise RTF)")
ap.add_argument("--cam-noise", type=float, default=0.007)  # gaussian stddev on [0,1] pixel values
# monitor views (not seen by the brain): chase camera rigid behind the drone + a fixed overview camera per world
# Rendering is lockstepped with physics: every extra camera costs a render pass. With the 60 Hz wide-angle drone
# camera, two 960x540@30 views dropped RTF to 0.88 and two 640x360@15 to 0.92 (rates not dividing 60 interleave).
ap.add_argument(
    "--view-cams",
    choices=["chase", "overview", "both", "none"],
    default="chase",
    help="views in the *_monitor model/worlds for monitor.py (the brain never sees them); "
    "the plain ecps295_quad and worlds never have them, so experiment runs keep RTF 1.0",
)
ap.add_argument("--view-res", type=int, nargs=2, default=[640, 360])
ap.add_argument("--view-hz", type=float, default=30.0, help="keep it a divisor of --cam-rate")
# mount position (§4.2 C4 estimate): ~30 mm ahead of the front motor line, ~15 mm below the prop plane
ap.add_argument("--cam-ahead", type=float, default=0.030)  # m ahead of the front motors
ap.add_argument("--cam-below", type=float, default=0.015)  # m below the prop plane
# Matek 3901-L0X: VL53L0X ToF facing down, sent to SITL as rng_1 (JSON). The PMW3901 flow itself is simulated by
# SITL (SIM_FLOW_*); sitl/ardupilot_sitl.patch makes it use this range so flow scales with box tops underneath.
ap.add_argument("--tof", choices=["on", "off"], default="on")
ap.add_argument("--tof-max", type=float, default=2.0)  # m, Matek spec (RNGFND1_MAX in fhb_delta.parm stays 1.2)
ap.add_argument("--tof-fov", type=float, default=27.0)  # deg, full cone (Matek 3901-L0X spec)
ap.add_argument("--tof-rate", type=float, default=10.0)  # Hz; real VL53L0X ~30 Hz, but the GPU ray pass at 30 Hz cost
# RTF 1.000 -> 0.906 over whole runs (2026-10-03), 10 Hz -> 0.973; the EKF fuses height at <= 10-20 Hz anyway
a = ap.parse_args()
a.views_active = "none"  # set per generated variant

RHO = 1.2041
A0, CLA, CDA = 0.3, 4.25, 0.10  # blade incidence / lift / drag slopes (Iris values)
CL = CLA * A0
R_CP = 0.7 * a.prop_radius  # blade centre of pressure
ARM = a.wheelbase / 2 / math.sqrt(2)  # motor x/y offset
ROTOR_M = 0.003  # prop + bell
ROTOR_IZZ = 1.6e-6  # 4" prop + bell, kg m^2
Z_ROTOR = 0.02  # prop plane above the body origin
CAM_M = 0.010 if a.camera != "none" else 0.0  # ELP board after trimming the cable, 8-12 g (§11.2)
# IMU and camera sensors sit on base_link itself. A 10 g camera link on a `fixed` joint made the vehicle hover at
# 15% less thrust (MOT_THST_HOVER 0.283 vs 0.323, same total mass) -- a joint-constraint artefact, so no extra links.
BASE_M = a.mass - 4 * ROTOR_M
CAM_X, CAM_Z = ARM + a.cam_ahead, Z_ROTOR - a.cam_below
CG_X = CAM_M * CAM_X / BASE_M  # camera shifts the body CG forward a little
CAM_TOPIC = "ecps295/camera"

# hover PWM fraction from the ArduPilot motor curve: thrust t -> actuator x with (1-e) x + e x^2 = t
e, t = a.expo, a.hover_thr
x = (-(1 - e) + math.sqrt((1 - e) ** 2 + 4 * e * t)) / (2 * e)
pwm_hover = a.spin[0] + (a.spin[1] - a.spin[0]) * x
omega_hover = pwm_hover * a.max_omega
blade_lift = a.mass * 9.80665 / 8  # 4 rotors x 2 blades
area = a.blade_area or blade_lift / (0.5 * RHO * (omega_hover * R_CP) ** 2 * CL)
drag_torque = 2 * 0.5 * RHO * (omega_hover * R_CP) ** 2 * area * CDA * A0 * R_CP
p_gain = ROTOR_IZZ / a.motor_tau
print(
    f"hover PWM fraction {pwm_hover:.3f}, omega {omega_hover:.0f} rad/s, blade area {area:.5f} m^2, "
    f"rotor drag torque {drag_torque:.2e} N m -> P-only speed error {drag_torque / p_gain:.1f} rad/s; "
    f"thrust/weight at full PWM {(1 / pwm_hover) ** 2:.2f}"
)

# rotor i: (x, y, spin sign) -- ArduPilot quad-X order, same as Iris: 0 FR ccw, 1 BL ccw, 2 FL cw, 3 BR cw
ROTORS = [(ARM, -ARM, 1), (-ARM, ARM, 1), (ARM, ARM, -1), (-ARM, -ARM, -1)]


def inertia(ixx, iyy, izz):
    return f"<inertia><ixx>{ixx:.3e}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{iyy:.3e}</iyy><iyz>0</iyz><izz>{izz:.3e}</izz></inertia>"


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


def view_camera(name: str, pose: str, hfov_deg: float) -> str:
    """Plain RGB camera for the monitor window (no noise); not part of the drone twin."""
    return f"""
      <sensor name="{name}" type="camera">
        <pose>{pose}</pose>
        <always_on>1</always_on><update_rate>{a.view_hz}</update_rate><topic>ecps295/{name}</topic>
        <camera>
          <horizontal_fov>{math.radians(hfov_deg):.5f}</horizontal_fov>
          <image><width>{a.view_res[0]}</width><height>{a.view_res[1]}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>200</far></clip>
        </camera>
      </sensor>"""


def chase_camera() -> str:
    # 0.9 m behind, 0.45 m above, looking 24 deg down at the drone; rigid, so it yaws (and tilts) with the drone
    return (
        ""
        if a.views_active not in ("chase", "both")
        else view_camera("chase_camera", f"-0.9 0 0.45 0 {math.radians(24):.4f} 0", 75)
    )


def overview_camera(xyz, look_at) -> str:
    if a.views_active not in ("overview", "both"):
        return ""
    dx, dy, dz = (look_at[i] - xyz[i] for i in range(3))
    yaw, pitch = math.atan2(dy, dx), math.atan2(-dz, math.hypot(dx, dy))
    return f"""
    <model name="overview_cam"><static>true</static><pose>{xyz[0]} {xyz[1]} {xyz[2]} 0 {pitch:.4f} {yaw:.4f}</pose>
      <link name="link">{view_camera("overview_camera", "0 0 0 0 0 0", 70)}
      </link>
    </model>"""


def camera_parts():
    if a.camera == "none":
        return ""
    w, h = a.cam_res
    if a.camera == "wide":
        sensor_type = "wideanglecamera"
        # equidistant r = f*theta (fisheye, §4.2 C1); scale_to_hfov keeps the requested hfov across the image width
        lens = (
            "<lens><type>equidistant</type><scale_to_hfov>true</scale_to_hfov>"
            f"<cutoff_angle>{math.radians(90):.4f}</cutoff_angle><env_texture_size>{a.cam_env_tex}</env_texture_size></lens>"
        )
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


TOF_TOPIC = "ecps295/tof"
# behind the battery (its visual would block the beam), 15 mm below the body origin: ~8 mm above the floor when
# landed, which keeps the reading inside the ray's min range (a reading below min is inf and would look like "no
# ground" to SITL)
TOF_X, TOF_Z = -0.045, -0.015


def tof_parts():
    if a.tof == "off":
        return ""
    half = math.radians(a.tof_fov / 2)
    scan = lambda tag: (  # noqa: E731
        f"<{tag}><samples>5</samples><resolution>1</resolution>"
        f"<min_angle>{-half:.4f}</min_angle><max_angle>{half:.4f}</max_angle></{tag}>"
    )
    return f"""
      <visual name="tof_board"><pose>{TOF_X} 0 {TOF_Z + 0.004} 0 0 0</pose><geometry><box><size>0.02 0.02 0.006</size></box></geometry>
        <material><ambient>0 0 0.6 1</ambient><diffuse>0 0 0.6 1</diffuse></material></visual>
      <sensor name="tof" type="gpu_lidar">
        <pose>{TOF_X} 0 {TOF_Z} 0 {math.pi / 2:.5f} 0</pose>
        <always_on>1</always_on><update_rate>{a.tof_rate}</update_rate><topic>{TOF_TOPIC}</topic>
        <lidar>
          <scan>{scan("horizontal")}{scan("vertical")}</scan>
          <range><min>0.005</min><max>{a.tof_max}</max><resolution>0.001</resolution></range>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>
        </lidar>
      </sensor>"""


def tof_plugin_entry():
    # ArduPilotPlugin takes the minimum over the beams (no return -> 2 x max) and sends it as rng_<index>
    if a.tof == "off":
        return ""
    return f"""
      <sensor><type>range</type><index>1</index><topic>/{TOF_TOPIC}</topic></sensor>"""


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


def build_model(name: str) -> str:
    return f"""<?xml version="1.0"?>
<!-- generated by ecps295/gazebo/make_quad.py; edit the generator, not this file -->
<sdf version="1.9">
  <model name="{name}">
    <pose>0 0 0.03 0 0 0</pose>
    <link name="base_link">
      <inertial><pose>{CG_X:.4f} 0 0 0 0 0</pose><mass>{BASE_M:.4f}</mass>{inertia(*a.inertia)}</inertial>
      <collision name="body"><geometry><box><size>0.09 0.05 0.04</size></box></geometry></collision>
      <collision name="legs"><pose>0 0 -0.02 0 0 0</pose><geometry><box><size>0.10 0.10 0.005</size></box></geometry></collision>
      <visual name="body"><geometry><box><size>0.09 0.05 0.035</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>
      <visual name="battery"><pose>0 0 -0.03 0 0 0</pose><geometry><box><size>0.075 0.035 0.02</size></box></geometry>
        <material><ambient>0.8 0.6 0.1 1</ambient><diffuse>0.8 0.6 0.1 1</diffuse></material></visual>
      <visual name="arm_a"><pose>0 0 0.01 0 0 {math.pi / 4:.5f}</pose><geometry><box><size>{
        a.wheelbase
    } 0.012 0.004</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
      <visual name="arm_b"><pose>0 0 0.01 0 0 {-math.pi / 4:.5f}</pose><geometry><box><size>{
        a.wheelbase
    } 0.012 0.004</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
      <visual name="nose"><pose>0.05 0 0 0 0 0</pose><geometry><box><size>0.01 0.02 0.02</size></box></geometry>
        <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual>
      <sensor name="imu_sensor" type="imu">
        <gz_frame_id>base_link</gz_frame_id>
        <pose degrees="true">0 0 0 180 0 0</pose>
        <always_on>1</always_on><update_rate>1000.0</update_rate>
      </sensor>
{camera_parts()}
{tof_parts()}
{chase_camera()}
    </link>
{"".join(rotor_link(i, x, y, s) for i, (x, y, s) in enumerate(ROTORS))}
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
{"".join(lift_drag(i, s, side) for i, (_, _, s) in enumerate(ROTORS) for side in (1, -1))}
{
        "".join(
            f'''
    <plugin filename="gz-sim-apply-joint-force-system" name="gz::sim::systems::ApplyJointForce">
      <joint_name>rotor_{i}_joint</joint_name>
    </plugin>'''
            for i in range(4)
        )
    }
    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9002</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>
      <no_time_sync>1</no_time_sync>
      <have_32_channels>0</have_32_channels>
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>base_link::imu_sensor</imuName>{tof_plugin_entry()}
{"".join(control(i, s) for i, (_, _, s) in enumerate(ROTORS))}
    </plugin>
  </model>
</sdf>
"""


def build_config(name: str) -> str:
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
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
    <include><uri>model://MODEL</uri><pose degrees="true">0 0 0.03 0 0 90</pose></include>
  </world>
</sdf>
"""

VARIANTS = (("ecps295_quad", "", "none"), ("ecps295_quad_monitor", "_monitor", a.view_cams))
for mname, _, views in VARIANTS:
    a.views_active = views
    mdir = HERE / "models" / mname
    mdir.mkdir(parents=True, exist_ok=True)
    (mdir / "model.sdf").write_text(build_model(mname))
    (mdir / "model.config").write_text(build_config(mname))
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
    return "".join(
        f"""
        <visual name="tape_{n}"><pose>{p[0]:.3f} {p[1]:.3f} {p[2]:.3f} 0 0 0</pose><geometry><box><size>{q[0]:.3f} {q[1]:.3f} {q[2]:.3f}</size></box></geometry>
          <material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material></visual>"""
        for n, p, q in bands
    )


def box(name, xyz, size, rgb, static=True, tape=False):
    c = " ".join(f"{v:.2f}" for v in rgb)
    return f"""
    <model name="{name}"><static>{str(static).lower()}</static><pose>{xyz[0]} {xyz[1]} {xyz[2]} 0 0 0</pose>
      <link name="link"><collision name="c"><geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>
          <material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material></visual>{tape_bands(size) if tape else ""}</link></model>"""


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


# (name, centre ENU [m], size [m], colour, cardboard?) -- also exported as <world>.layout.json for the dashboard
CAMTEST_OBJECTS = [
    ("box_a", (0.0, 1.5, 0.25), (0.5, 0.5, 0.5), CARDBOARD, True),
    ("box_b", (1.0, 2.2, 0.30), (0.6, 0.45, 0.6), CARDBOARD, True),
    ("box_c", (-1.2, 2.6, 0.25), (0.45, 0.6, 0.5), CARDBOARD, True),
    ("pole", (0.6, 1.0, 1.0), (0.04, 0.04, 2.0), (0.9, 0.1, 0.1), False),
    ("marker_left", (-2.4, 1.2, 0.5), (0.2, 0.2, 1.0), (0.1, 0.3, 0.9), False),
    ("marker_right", (2.4, 1.2, 0.5), (0.2, 0.2, 1.0), (0.1, 0.8, 0.2), False),
]
FLOOR_M = 10 * 0.61


def camtest_extra(tape: bool) -> str:
    return (
        checker_floor()
        + "".join(box(n, c, sz, rgb, tape=tape and cb) for n, c, sz, rgb, cb in CAMTEST_OBJECTS)
        + overview_camera((-2.2, -1.6, 1.9), (0.0, 1.0, 0.4))
    )


def write_layout(wname: str, objects, floor_m: float = FLOOR_M) -> None:
    """Obstacles for flydrones' dashboard ROOM panel; the drone spawns at the origin, so ENU here = SITL local ENU."""
    lay = {
        "world": wname,
        "floor_m": floor_m,
        "boxes": [{"name": n, "center_enu": c, "size_enu": sz} for n, c, sz, _, _ in objects],
    }
    (HERE / "worlds" / f"{wname}.layout.json").write_text(json.dumps(lay, indent=1))


def write_world(wname: str, extra_fn, objects=(), floor_m: float = FLOOR_M) -> None:
    """Plain world (twin only) and *_monitor variant (monitor model + overview camera if selected)."""
    for mname, suffix, views in VARIANTS:
        a.views_active = views
        name = wname + suffix
        (HERE / "worlds" / f"{name}.sdf").write_text(
            WORLD_TMPL.replace("NAME", name).replace("MODEL", mname).replace("EXTRA", extra_fn())
        )
        if objects:
            write_layout(name, objects, floor_m)


write_world("ecps295_flat", lambda: overview_camera((-2.0, -2.0, 1.6), (0.0, 0.0, 0.5)))
write_world("ecps295_camtest", lambda: camtest_extra(False), CAMTEST_OBJECTS)
write_world("ecps295_camtest_tape", lambda: camtest_extra(True), CAMTEST_OBJECTS)

# flight-test walls: 1.5 m tall (above the 1.0 m ceiling, so climbing over is not an option), face 1.7 m ahead
SIDE_MARKERS = [o for o in CAMTEST_OBJECTS if o[0].startswith("marker")]


def wall_objects(east: float, tape: bool):
    return [("wall", (east, 1.85, 0.75), (1.2, 0.3, 1.5), CARDBOARD, tape), *SIDE_MARKERS]


for wname, east, tape in (
    ("ecps295_wall_tape", 0.0, True),
    ("ecps295_wall_plain", 0.0, False),
    ("ecps295_wall_offset", 0.45, True),
    ("ecps295_wall_offset_plain", 0.45, False),
):
    objs = wall_objects(east, tape)
    write_world(
        wname,
        lambda objs=objs: (
            checker_floor()
            + "".join(box(n, c, sz, rgb, tape=cb) for n, c, sz, rgb, cb in objs)
            + overview_camera((-2.2, -1.6, 1.9), (0.0, 1.2, 0.6))
        ),
        objs,
    )


# ----------------------------------------------------------------------------------------------- G3: course cages
# TechRoute §4.3 / §6A G3. The cage is centred on the take-off point (FlyDrones' geofence is a radius around it).
ALU = (0.72, 0.72, 0.74)
NET = (0.16, 0.16, 0.16)


def static_visuals(name: str, visuals: list[str], collision: str = "") -> str:
    return f"""
    <model name="{name}"><static>true</static><link name="link">{collision}{"".join(visuals)}
      </link></model>"""


def vbox(vname, xyz, size, rgb, yaw=0.0) -> str:
    c = " ".join(f"{v:.2f}" for v in rgb)
    return (
        f"""
        <visual name="{vname}"><pose>{xyz[0]:.3f} {xyz[1]:.3f} {xyz[2]:.3f} 0 0 {yaw:.4f}</pose>"""
        f"""<geometry><box><size>{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}</size></box></geometry>
          <material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material></visual>"""
    )


def net_wall(name, center, along, length, height, spacing=0.10, strand=0.004) -> str:
    """Knotted netting as real strands (4 mm on a 10 cm grid): nearly see-through, as §4.3 describes."""
    cx, cy = center
    vis = []
    for k in range(int(length / spacing) + 1):
        o = -length / 2 + k * spacing
        xyz = (cx + o, cy, height / 2) if along == "x" else (cx, cy + o, height / 2)
        vis.append(vbox(f"v{k}", xyz, (strand, strand, height), NET))
    for k in range(1, int(height / spacing) + 1):
        z = k * spacing
        size = (length, strand, strand) if along == "x" else (strand, length, strand)
        vis.append(vbox(f"h{k}", (cx, cy, z), size, NET))
    csize = (length, 0.02, height) if along == "x" else (0.02, length, height)
    col = (
        f"""<collision name="c"><pose>{cx:.3f} {cy:.3f} {height / 2:.3f} 0 0 0</pose>"""
        f"""<geometry><box><size>{csize[0]:.3f} {csize[1]:.3f} {csize[2]:.3f}</size></box></geometry></collision>"""
    )
    return static_visuals(name, vis, col)


def cage_frame(side: float, height: float) -> str:
    """5 cm aluminium posts every <= 3.05 m and top rails."""
    h = side / 2
    n = max(1, round(side / 3.05))
    ticks = [-h + k * side / n for k in range(n + 1)]
    vis = []
    for i, x in enumerate(ticks):
        for j, y in enumerate(ticks):
            if x in (-h, h) or y in (-h, h):
                vis.append(vbox(f"post{i}_{j}", (x, y, height / 2), (0.05, 0.05, height), ALU))
    for k, v in enumerate((-h, h)):
        vis.append(vbox(f"railx{k}", (0, v, height), (side, 0.05, 0.05), ALU))
        vis.append(vbox(f"raily{k}", (v, 0, height), (0.05, side, 0.05), ALU))
    return static_visuals("cage_frame", vis)


def floor_tape(side: float, n: int = 10, seed: int = 7) -> str:
    """Random strips of light tape on the EVA mat (§4.3)."""
    import random

    rnd = random.Random(seed)
    vis = []
    for k in range(n):
        x, y = rnd.uniform(-side / 2 + 0.3, side / 2 - 0.3), rnd.uniform(-side / 2 + 0.3, side / 2 - 0.3)
        vis.append(
            vbox(f"tape{k}", (x, y, 0.0025), (rnd.uniform(0.5, 1.4), 0.05, 0.001), (0.82, 0.80, 0.74), rnd.uniform(0, math.pi))
        )
    return static_visuals("floor_tape", vis)


def scenery(seed: int = 3) -> str:
    """Far buildings and trees seen through the net (30-60 m)."""
    import random

    rnd = random.Random(seed)
    vis = []
    for k in range(10):
        a, r = rnd.uniform(0, 2 * math.pi), rnd.uniform(30, 60)
        w, d, hgt = rnd.uniform(8, 22), rnd.uniform(8, 20), rnd.uniform(6, 22)
        g = rnd.uniform(0.45, 0.8)
        vis.append(vbox(f"bldg{k}", (r * math.cos(a), r * math.sin(a), hgt / 2), (w, d, hgt), (g, g * 0.97, g * 0.92), a))
    for k in range(14):
        a, r = rnd.uniform(0, 2 * math.pi), rnd.uniform(18, 40)
        x, y, hgt = r * math.cos(a), r * math.sin(a), rnd.uniform(4, 9)
        vis.append(
            f"""
        <visual name="trunk{k}"><pose>{x:.2f} {y:.2f} {hgt * 0.3:.2f} 0 0 0</pose><geometry><cylinder><radius>0.25</radius>"""
            f"""<length>{hgt * 0.6:.2f}</length></cylinder></geometry>
          <material><ambient>0.35 0.25 0.15 1</ambient><diffuse>0.35 0.25 0.15 1</diffuse></material></visual>
        <visual name="crown{k}"><pose>{x:.2f} {y:.2f} {hgt * 0.75:.2f} 0 0 0</pose><geometry><sphere><radius>{hgt * 0.3:.2f}"""
            f"""</radius></sphere></geometry>
          <material><ambient>0.18 0.4 0.15 1</ambient><diffuse>0.18 0.4 0.15 1</diffuse></material></visual>"""
        )
    return static_visuals("scenery", vis)


def lab_lights(side: float, height: float) -> str:
    return "".join(
        f"""
    <light type="point" name="lamp{k}"><cast_shadows>false</cast_shadows><pose>{x:.2f} {y:.2f} {height + 1.0:.2f} 0 0 0</pose>
      <diffuse>0.55 0.55 0.52 1</diffuse><specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>15</range><constant>0.6</constant><linear>0.05</linear><quadratic>0.01</quadratic></attenuation>
    </light>"""
        for k, (x, y) in enumerate(((-side / 4, -side / 4), (side / 4, side / 4), (-side / 4, side / 4), (side / 4, -side / 4)))
    )


# obstacles (name, centre ENU, size, colour, taped?) -- the drone starts at the cage centre facing north (+y)
CAGE20_OBJECTS = [
    ("stack_low", (0.0, 1.9, 0.275), (0.55, 0.55, 0.55), CARDBOARD, True),  # two 22" boxes stacked: 1.1 m,
    ("stack_high", (0.0, 1.9, 0.825), (0.55, 0.55, 0.55), CARDBOARD, True),  # above the 1.0 m ceiling
    ("box_w", (-1.6, 0.6, 0.23), (0.46, 0.46, 0.46), CARDBOARD, False),  # 18" plain
    ("box_e", (1.7, -0.8, 0.3), (0.6, 0.6, 0.6), CARDBOARD, True),  # 24" taped
    ("box_sw", (-1.0, -1.9, 0.25), (0.5, 0.5, 0.5), CARDBOARD, False),
]
CAGE10_OBJECTS = [("box_n", (0.25, 0.95, 0.3), (0.6, 0.6, 0.6), CARDBOARD, True)]


def cage_walls(side: float, height: float):
    """Net walls as layout obstacles (thin boxes) so the ROOM panel and the clearance metric see them."""
    h = side / 2
    return [
        ("net_n", (0, h, height / 2), (side, 0.02, height), NET, False),
        ("net_s", (0, -h, height / 2), (side, 0.02, height), NET, False),
        ("net_e", (h, 0, height / 2), (0.02, side, height), NET, False),
        ("net_w", (-h, 0, height / 2), (0.02, side, height), NET, False),
    ]


def cage_extra(side: float, height: float, objects) -> str:
    h = side / 2
    return (
        checker_floor(n=round(side / 0.61))
        + floor_tape(side)
        + cage_frame(side, height)
        + net_wall("net_n", (0, h), "x", side, height)
        + net_wall("net_s", (0, -h), "x", side, height)
        + net_wall("net_e", (h, 0), "y", side, height)
        + net_wall("net_w", (-h, 0), "y", side, height)
        + "".join(box(n, c, sz, rgb, tape=cb) for n, c, sz, rgb, cb in objects)
        + scenery()
        + lab_lights(side, height)
        + overview_camera((-h + 0.25, -h + 0.25, height - 0.3), (0.3, 0.6, 0.4))
    )


for wname, side, objs in (("ecps295_cage20", 6.1, CAGE20_OBJECTS), ("ecps295_cage10", 3.05, CAGE10_OBJECTS)):
    write_world(
        wname, lambda side=side, objs=objs: cage_extra(side, 3.05, objs), objs + cage_walls(side, 3.05), floor_m=side + 0.4
    )
print(
    "wrote models", [v[0] for v in VARIANTS], "and worlds ecps295_{flat,camtest,camtest_tape,wall_*,cage10,cage20}[_monitor].sdf"
)
