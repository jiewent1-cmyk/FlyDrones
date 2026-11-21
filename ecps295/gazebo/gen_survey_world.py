"""Generate the VANTAGE survey site worlds (2026-11-03): AprilTags and solar panels under the ecps295 drone.

The site is VANTAGE's survey/sites/synth.json (survey/synth.py build_site) at half scale: at the twin's ~1 m ceiling a
70 deg 4:3 nadir camera only takes in ~1.15 x 0.80 m of tag centres (survey/vision/CAPTURE_BENCH.md), and the full
1.8 x 1.3 m site needs 1.5-2 m. Half scale: boundary 0.9 x 0.65 m, boundary tags 7.5 cm, panels 0.20 x 0.15 m with
4 cm tags.

Geometry, textures and soiling are synth.py's (copied here so this tree does not import VANTAGE). Every textured
surface is one quad mesh (OBJ + MTL) with explicit UVs, written in world coordinates, so a texture's top-left corner
lands exactly where synth.draw() would put it: no guessing at Gazebo's plane / box UV conventions.

Frames. Gazebo world = ENU (x east, y north, z up), as make_quad.write_layout says; the drone spawns at the origin
facing +y (north). The survey frame (tag 1 at the origin, 1->2 is +y) is the world frame translated, so the site centre
lies 1.0 m north of take-off:
    world_e = sx - 0.45, world_n = sy + 0.675, up = sz        (NED north = sy + 0.675, east = sx - 0.45)
The survey frame's +x is east and +y north; mapping survey +x to north would need a mirror image.

    python gen_survey_world.py        (needs numpy + opencv-contrib with cv2.aruco; writes worlds/ and models/)

Outputs: worlds/ecps295_survey.sdf (clean), ecps295_survey_dirty.sdf (panel tag 12 soiled), ecps295_survey_preview.sdf
(the clean site plus a static copy of the survey camera at 0.95 m over the site centre, topic
/ecps295/survey_cam_preview), ecps295_survey.layout.json (no boxes: the panels are visual only), ecps295_survey.truth.json.
"""

from __future__ import annotations

import json
import math
import pathlib

import cv2
import numpy as np

HERE = pathlib.Path(__file__).parent
MODEL = "vantage_survey_site"
MDIR = HERE / "models" / MODEL
SCALE = 0.5
OFFSET = (-0.45, 0.675)  # survey (x, y) + OFFSET = world (east, north)
BOUNDARY_TAG = 0.15 * SCALE  # black square side, as synth.render(boundary_tag=...)
PANEL_TAG = 0.08 * SCALE
PANEL_L, PANEL_W, FRAME = 0.40 * SCALE, 0.30 * SCALE, 0.015 * SCALE
DIRTY_TAG = 12
CAMERA = {"width": 2592, "height": 1944, "hfov_deg": 70.0, "rate_hz": 2.0, "altitude_m": 0.95}
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)


# ---------------------------------------------------------------- synth.py, unchanged
def tag_texture(tag_id, px=400):
    inner = cv2.aruco.generateImageMarker(DICT, tag_id, px * 8 // 10)
    img = np.full((px, px), 255, np.uint8)
    o = px // 10
    img[o:o + inner.shape[0], o:o + inner.shape[1]] = inner
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def panel_texture(px_w=800, px_h=600):
    img = np.zeros((px_h, px_w, 3), np.uint8)
    img[:] = (70, 40, 25)
    rng = np.random.default_rng(1)
    img = np.clip(img + rng.normal(0, 8, img.shape), 0, 255).astype(np.uint8)
    for y in np.linspace(0, px_h, 12)[1:-1]:
        cv2.line(img, (0, int(y)), (px_w, int(y)), (200, 200, 200), 3)
    return img


def soil(img, seed):
    out = img.astype(np.float32)
    rng = np.random.default_rng(seed)
    h, w = img.shape[:2]
    for _ in range(14):
        layer = np.zeros((h, w), np.float32)
        centre = (int(rng.integers(0, w)), int(rng.integers(0, h)))
        axes = (int(rng.integers(w // 16, w // 6)), int(rng.integers(h // 16, h // 6)))
        cv2.ellipse(layer, centre, axes, float(rng.integers(0, 180)), 0, 360, 1.0, -1)
        layer = cv2.GaussianBlur(layer, (0, 0), w / 80)[..., None] * rng.uniform(0.5, 0.8)
        out = out * (1 - layer) + np.array((95, 135, 165), np.float32) * layer
    for _ in range(10):
        centre = (int(rng.integers(0, w)), int(rng.integers(0, h)))
        r = int(rng.integers(w // 80, w // 30))
        cv2.circle(out, centre, r, (225, 230, 230), -1)
    return np.clip(out, 0, 255).astype(np.uint8)


def floor_texture(px=2000):
    rng = np.random.default_rng(2)
    img = np.full((px, px, 3), 215, np.float32)
    img += rng.normal(0, 6, img.shape)
    for _ in range(4000):
        x, y = rng.integers(0, px, 2)
        cv2.ellipse(img, (int(x), int(y)), (int(rng.integers(3, 12)), 2), float(rng.integers(0, 180)), 0, 360,
                    (110, 110, 115), -1)
    return np.clip(img, 0, 255).astype(np.uint8)


def rect3d(centre, length, width, yaw, z=0.0, tilt=0.0):
    ux = np.array([np.cos(yaw), np.sin(yaw), 0])
    uy = np.array([-np.sin(yaw), np.cos(yaw), 0])
    uy_t = uy * np.cos(tilt) + np.array([0, 0, np.sin(tilt)])
    c = np.array([centre[0], centre[1], z])
    hl, hw = length / 2, width / 2
    return np.array([c - ux * hl + uy_t * hw, c + ux * hl + uy_t * hw, c + ux * hl - uy_t * hw, c - ux * hl - uy_t * hw])


def build_site():
    boundary = {1: (0.0, 0.0), 2: (0.0, 1.3), 3: (1.8, 1.3), 4: (1.8, 0.0)}
    panels = [
        dict(tag=11, centre=(0.45, 0.40), yaw=np.deg2rad(0), tilt=np.deg2rad(20), low=0.05),
        dict(tag=12, centre=(1.00, 0.35), yaw=np.deg2rad(8), tilt=np.deg2rad(20), low=0.05),
        dict(tag=13, centre=(1.40, 0.80), yaw=np.deg2rad(-90), tilt=np.deg2rad(15), low=0.05),
        dict(tag=None, centre=(0.60, 0.95), yaw=np.deg2rad(5), tilt=np.deg2rad(0), low=0.02),
    ]
    return boundary, panels


def panel_geometry(p, length, width):
    zc = p["low"] + width / 2 * np.sin(p["tilt"])
    return rect3d(p["centre"], length, width, p["yaw"], zc, p["tilt"]), zc


# ---------------------------------------------------------------- the scaled site
def scaled_site():
    boundary, panels = build_site()
    boundary = {k: (x * SCALE, y * SCALE) for k, (x, y) in boundary.items()}
    for p in panels:
        p["centre"] = (p["centre"][0] * SCALE, p["centre"][1] * SCALE)
        p["low"] *= SCALE
    return boundary, panels


def to_world(q):
    q = np.asarray(q, float).copy()
    q[..., 0] += OFFSET[0]
    q[..., 1] += OFFSET[1]
    return q


def write_quad(name, quad, tex):
    """One textured quad: corners TL, TR, BR, BL in survey coordinates, the texture's corners in that order."""
    (MDIR / "meshes").mkdir(parents=True, exist_ok=True)
    (MDIR / "materials" / "textures").mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(MDIR / "materials" / "textures" / f"{name}.png"), tex)
    w = to_world(quad)
    n = np.cross(w[1] - w[0], w[0] - w[3])  # TL->TR x BL->TL points up for a quad seen from above
    n = n / np.linalg.norm(n)
    v = "".join(f"v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n" for p in w)
    obj = (f"# generated by gen_survey_world.py\nmtllib {name}.mtl\n{v}vt 0 1\nvt 1 1\nvt 1 0\nvt 0 0\n"
           f"vn {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\nusemtl {name}\n"
           "f 4/4/1 3/3/1 2/2/1\nf 4/4/1 2/2/1 1/1/1\n")  # counter-clockwise seen from the normal's side
    (MDIR / "meshes" / f"{name}.obj").write_text(obj)
    (MDIR / "meshes" / f"{name}.mtl").write_text(
        f"newmtl {name}\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nNs 1\nillum 1\nmap_Kd ../materials/textures/{name}.png\n")
    return (f'        <visual name="{name}"><cast_shadows>false</cast_shadows><geometry><mesh>'
            f"<uri>model://{MODEL}/meshes/{name}.obj</uri></mesh></geometry></visual>\n")


def lift(quad, normal_offset):
    """The quad moved along its own normal: cells and tags sit just above the surface they lie on (no z-fighting)."""
    n = np.cross(quad[1] - quad[0], quad[0] - quad[3])
    return quad + n / np.linalg.norm(n) * normal_offset


def build(dirty: bool):
    boundary, panels = scaled_site()
    vis, truth = [], {"boundary": {}, "panels": []}
    size = 4.0  # floor 4 x 4 m around the site centre
    cx, cy = 0.9 * SCALE, 0.65 * SCALE
    vis.append(write_quad("floor", rect3d((cx, cy), size, size, 0.0, z=0.0005), floor_texture()))
    for tid, (x, y) in boundary.items():
        side = BOUNDARY_TAG / 0.8
        vis.append(write_quad(f"btag{tid}", rect3d((x, y), side, side, 0.0, z=0.001), tag_texture(tid)))
        truth["boundary"][str(tid)] = {"survey": [x, y, 0.001], "ned": [y + OFFSET[1], x + OFFSET[0], -0.001]}
    ptex = panel_texture()
    sfx = "_dirty" if dirty else ""
    for p in sorted(panels, key=lambda p: p["low"]):
        pname = f"panel{p['tag'] or 0}"
        outline, zc = panel_geometry(p, PANEL_L, PANEL_W)
        vis.append(write_quad(f"{pname}_frame", outline, np.full((300, 400, 3), 235, np.uint8)))
        # in the frame's own plane: synth.py's panel_geometry() for the inset centres it a few mm lower on a tilted
        # panel, which painter's order hides and a depth buffer does not
        cells = lift(rect3d(p["centre"], PANEL_L - 2 * FRAME, PANEL_W - 2 * FRAME, p["yaw"], zc, p["tilt"]), 0.0005)
        soiled = dirty and p["tag"] == DIRTY_TAG
        vis.append(write_quad(f"{pname}_cells{'_soiled' if soiled else ''}", cells,
                              soil(ptex, p["tag"] or 0) if soiled else ptex))
        tag_xyz = None
        if p["tag"] is not None:
            s = PANEL_TAG / 0.8
            ux = (outline[1] - outline[0]) / np.linalg.norm(outline[1] - outline[0])
            uy = (outline[0] - outline[3]) / np.linalg.norm(outline[0] - outline[3])
            c = outline.mean(0)
            q = np.array([c - ux * s / 2 + uy * s / 2, c + ux * s / 2 + uy * s / 2,
                          c + ux * s / 2 - uy * s / 2, c - ux * s / 2 - uy * s / 2])
            vis.append(write_quad(f"ptag{p['tag']}", lift(q, 0.0015), tag_texture(p["tag"])))
            tag_xyz = c.tolist()
        truth["panels"].append({
            "tag": p["tag"], "centre": list(p["centre"]), "yaw_deg": float(np.rad2deg(p["yaw"])),
            "tilt_deg": float(np.rad2deg(p["tilt"])), "low": p["low"], "tag_xyz": tag_xyz,
            "tag_ned": None if tag_xyz is None else [tag_xyz[1] + OFFSET[1], tag_xyz[0] + OFFSET[0], -tag_xyz[2]],
            "footprint": outline[:, :2].tolist(), "outline3d": outline.tolist(), "dirty": soiled})
    model = (f'    <model name="{MODEL}{sfx}"><static>true</static>\n      <link name="link">\n'
             + "".join(vis) + "      </link>\n    </model>\n")
    return model, truth


def preview_camera():
    """The survey camera alone, where the drone's would be at 0.95 m over the site centre, facing north."""
    c = CAMERA
    return f"""
    <model name="survey_cam_preview"><static>true</static><pose>0 1.0 {c['altitude_m']} 0 0 {math.pi / 2:.5f}</pose>
      <link name="link">
        <sensor name="survey_cam_preview" type="camera">
          <pose>0 0 0 0 {math.pi / 2:.5f} 0</pose>
          <always_on>1</always_on><update_rate>{c['rate_hz']}</update_rate><topic>ecps295/survey_cam_preview</topic>
          <camera>
            <horizontal_fov>{math.radians(c['hfov_deg']):.5f}</horizontal_fov>
            <image><width>{c['width']}</width><height>{c['height']}</height><format>R8G8B8</format></image>
            <clip><near>0.02</near><far>50</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
"""


def main():
    tmpl = (HERE / "worlds" / "ecps295_flat.sdf").read_text()
    head, tail = tmpl.split("    <include><uri>model://ecps295_quad</uri>")
    tail = "    <include><uri>model://ecps295_quad_survey</uri>" + tail
    MDIR.mkdir(parents=True, exist_ok=True)
    (MDIR / "model.config").write_text(
        f'<?xml version="1.0"?>\n<model><name>{MODEL}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>'
        "<description>textures and meshes of the VANTAGE survey site (gen_survey_world.py)</description></model>\n")
    truths = {}
    for name, dirty, extra in (("ecps295_survey", False, ""), ("ecps295_survey_dirty", True, ""),
                               ("ecps295_survey_preview", False, preview_camera())):
        model, truth = build(dirty)
        h = head.replace("ecps295_flat", name).replace("make_quad.py", "gen_survey_world.py (from make_quad.py's flat world)")
        (HERE / "worlds" / f"{name}.sdf").write_text(h + model + extra + tail)
        truths[name] = truth
    lay = {"world": "ecps295_survey", "floor_m": 10 * 0.61, "boxes": []}
    (HERE / "worlds" / "ecps295_survey.layout.json").write_text(json.dumps(lay, indent=1))
    fx = CAMERA["width"] / 2 / math.tan(math.radians(CAMERA["hfov_deg"]) / 2)
    out = {
        "comment": "VANTAGE synth site at half scale (gen_survey_world.py). survey frame: tag 1 at the origin, 1->2 is +y.",
        "scale": SCALE,
        "transform": {"world_enu": "east = sx + %.3f, north = sy + %.3f, up = sz" % OFFSET,
                      "ned": "north = sy + %.3f, east = sx + %.3f, down = -sz" % (OFFSET[1], OFFSET[0]),
                      "offset_east_north": list(OFFSET)},
        "camera": dict(CAMERA, fx_px=fx, focal_35mm=fx * math.hypot(36, 24) / math.hypot(CAMERA["width"], CAMERA["height"]),
                       mount="drone belly, nadir; image top = drone forward, image right = drone right"),
        "site_centre_ned": [0.65 * SCALE + OFFSET[1], 0.9 * SCALE + OFFSET[0]],
        "boundary": truths["ecps295_survey"]["boundary"],
        "panels": truths["ecps295_survey"]["panels"],
        "dirty_world": {"world": "ecps295_survey_dirty", "dirty_tags": [DIRTY_TAG]},
    }
    (HERE / "worlds" / "ecps295_survey.truth.json").write_text(json.dumps(out, indent=1))
    print(f"wrote {MDIR.name} and worlds ecps295_survey{{,_dirty,_preview}}.sdf, layout and truth")


if __name__ == "__main__":
    main()
