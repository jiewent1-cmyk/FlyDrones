"""Target-tracking Gazebo worlds (2026-10-18): copy ecps295_<w>.sdf -> ecps295_<w>_mk.sdf with one ArUco marker panel
(4x4_50 id 7, 15 cm black square + 25 % white quiet zone) standing in the cage, its printed face toward the centre.
Position: on a ring (radius R) at the takeoff height, at a bearing where the panel and the line of sight from the
centre clear every box; the drone starts at the centre facing north, so bearings close to north are avoided (the
tracker has to search). Writes <w>_mk.layout.json (panel added as a thin box) and <w>_mk.marker.json (n, e, up, yaw).

    python gen_marker_worlds.py <worlds dir> <png out> cage20 lowbox_s0 ...
"""
import json
import math
import random
import sys
from pathlib import Path

import cv2
import numpy as np

_here = Path(__file__).resolve().parent  # ROG ~/sim/rl_trk (next to ecps295/) or the repo's ecps295/gazebo
sys.path[:0] = [str(_here / "ecps295"), str(_here.parent)]
from ecps_target import marker_image  # noqa: E402

W_DIR, PNG = Path(sys.argv[1]), Path(sys.argv[2])
SIZE, BORDER, ID, UP, R0 = 0.30, 0.25, 7, 0.6, 2.3  # 30 cm: the Gazebo camera is a 120 deg fisheye (~2/3 of a 70 deg pinhole's px per rad); 15 cm was ~15 px at 3 m, never detected
SIDE = SIZE * (1 + 2 * BORDER)
cv2.imwrite(str(PNG), marker_image("4x4_50", ID, px=400, border_frac=BORDER))


def seg_box_clear(p, q, b, z, pad):
    """2D segment p->q (ENU) vs box footprint grown by pad, only boxes reaching above z."""
    (cx, cy, cz), (sx, sy, sz) = b["center_enu"], b["size_enu"]
    if cz + sz / 2 < z or b["name"].startswith("net"):
        return True
    for k in range(41):
        x, y = p[0] + (q[0] - p[0]) * k / 40, p[1] + (q[1] - p[1]) * k / 40
        if abs(x - cx) < sx / 2 + pad and abs(y - cy) < sy / 2 + pad:
            return False
    return True


for w in sys.argv[3:]:
    lay = json.load(open(W_DIR / f"ecps295_{w}.layout.json"))
    # 2026-10-19: inside the nets (cage10's are at 1.525 m: the fixed 2.3 m ring put its marker outside the cage)
    half = min(max(abs(bx["center_enu"][0]), abs(bx["center_enu"][1])) for bx in lay["boxes"] if bx["name"].startswith("net"))
    R = min(R0, half - 0.3)
    ok = []
    for deg in range(0, 360, 10):  # bearing from north (clockwise), NED
        if deg <= 40 or deg >= 320:
            continue
        b = math.radians(deg)
        n, e = R * math.cos(b), R * math.sin(b)
        mk = (e, n)  # ENU
        if all(seg_box_clear((0, 0), mk, bx, UP - 0.15, 0.25) for bx in lay["boxes"]) and all(
            seg_box_clear(mk, mk, bx, 0.0, 0.35) for bx in lay["boxes"]
        ):
            ok.append(deg)
    if not ok:
        print(w, "no free bearing, skipped")
        continue
    deg = random.Random(w).choice(ok)
    b = math.radians(deg)
    n, e = R * math.cos(b), R * math.sin(b)
    face_yaw_ned = (deg + 180) % 360  # the face looks back to the centre
    yaw_enu = math.radians(90 - face_yaw_ned)  # model +x = face normal
    model = f"""
    <model name="aruco_marker"><static>true</static><pose>{e:.3f} {n:.3f} {UP:.3f} 0 0 {yaw_enu:.5f}</pose>
      <link name="link">
        <visual name="face"><pose>0.0015 0 0 0 0 0</pose><geometry><box><size>0.001 {SIDE:.4f} {SIDE:.4f}</size></box></geometry>
          <material><diffuse>1 1 1 1</diffuse><ambient>1 1 1 1</ambient><pbr><metal><albedo_map>{PNG}</albedo_map>
            <metalness>0</metalness><roughness>1</roughness></metal></pbr></material></visual>
        <visual name="back"><pose>-0.004 0 0 0 0 0</pose><geometry><box><size>0.008 {SIDE + 0.02:.4f} {SIDE + 0.02:.4f}</size></box></geometry>
          <material><diffuse>0.25 0.25 0.3 1</diffuse><ambient>0.25 0.25 0.3 1</ambient></material></visual>
        <collision name="c"><geometry><box><size>0.01 {SIDE:.4f} {SIDE:.4f}</size></box></geometry></collision>
      </link></model>
"""
    sdf = (W_DIR / f"ecps295_{w}.sdf").read_text()
    i = sdf.rindex("</world>")
    (W_DIR / f"ecps295_{w}_mk.sdf").write_text(sdf[:i] + model + sdf[i:])
    nm = math.radians(face_yaw_ned)
    lay["world"] = f"ecps295_{w}_mk"
    lay["boxes"].append(
        {"name": "marker", "center_enu": [round(e, 3), round(n, 3), UP],
         "size_enu": [round(abs(SIDE * math.cos(nm)) + 0.01, 3), round(abs(SIDE * math.sin(nm)) + 0.01, 3), round(SIDE, 3)]}
    )
    json.dump(lay, open(W_DIR / f"ecps295_{w}_mk.layout.json", "w"))
    spec = {"n": round(n, 3), "e": round(e, 3), "up": UP, "yaw": face_yaw_ned, "id": ID, "size_m": SIZE, "bearing": deg}
    json.dump(spec, open(W_DIR / f"ecps295_{w}_mk.marker.json", "w"))
    print(w, spec, "free bearings", len(ok))
