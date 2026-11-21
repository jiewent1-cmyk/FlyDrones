"""VL53L7CX 前后安装座 —— v5 喇叭防撞版（2026-11-18）。

在 v4 平板版（gen_tof_plates.py）的基础上，绕载板四周加一圈向外张开的喇叭形护圈：
喇叭口比 IC 表面更靠前（HORN_LIP_Y），正面磕碰先碰到喇叭口；喇叭内壁从载板外缘起算向外张开
（起始约 22°，口部约 52°），始终在视场之外。载板仍然粘在立板的定位浅槽里，孔距和传感器位置不变。

前座的喇叭在底板下表面（z=0）处截平，后座的喇叭在顶板上表面（z=0）处截平，不和机架干涉。
打印姿态：喇叭口朝下贴床，不需要支撑（立板是喇叭喉部的一段约 13 mm 桥接）。
运行：python gen_tof_horn.py
"""
import math
import os
import numpy as np
import manifold3d as m3

import gen_tof_plates as g
from gen_tof_plates import box, cyl_y, cyl_z, PX, PZ, HX, HZ_TOP, PLATE_T, RIM, IC_Y, IC_Z, IC_HALF

HERE = os.path.dirname(os.path.abspath(__file__))

HORN_LIP_Y = 6.5      # 喇叭口平面（局部 y，载板背面为 0）；IC 表面在 3.35，喇叭口比它靠前 3.15 mm
HORN_WALL = 2.0       # 喇叭壁厚（垂直于轴线方向量）
HORN_A, HORN_B = 0.40, 0.45   # 内壁外扩量 o(s) = L·(A·s + B·s²)，s∈[0,1]；口部斜率 A+2B
HORN_N = 14           # 分段数
R_IN = 1.0            # 内壁圆角（喉部）


def rrect(hx, hz, r, y0, y1):
    """xz 平面内的圆角矩形薄片，半宽 hx、半高 hz、圆角 r，y 从 y0 到 y1。"""
    r = min(r, hx - 0.01, hz - 0.01)
    return m3.Manifold.batch_hull([cyl_y(sx * (hx - r), sz * (hz - r), y0, y1, r)
                                   for sx in (-1, 1) for sz in (-1, 1)])


def horn_offset(s):
    return (HORN_LIP_Y - RIM) * (HORN_A * s + HORN_B * s * s)


def build_horn():
    """局部坐标（和立板相同）。返回 (实体, 内腔)；内腔要在并入立板之后再减。"""
    ys = [RIM + (HORN_LIP_Y - RIM) * i / HORN_N for i in range(HORN_N + 1)]
    os_ = [horn_offset(i / HORN_N) for i in range(HORN_N + 1)]
    e = 0.01
    outer_s = [rrect(PX + o + HORN_WALL, PZ + o + HORN_WALL, R_IN + o + HORN_WALL, y, y + e) for y, o in zip(ys, os_)]
    inner_s = [rrect(PX + o, PZ + o, R_IN + o, y - e, y + 2 * e) for y, o in zip(ys, os_)]
    # 外壳从立板背面开始，和立板连成一体
    outer = rrect(PX + HORN_WALL, PZ + HORN_WALL, R_IN + HORN_WALL, -PLATE_T, RIM + e)
    inner = m3.Manifold()
    for i in range(HORN_N):
        outer += m3.Manifold.batch_hull([outer_s[i], outer_s[i + 1]])
        inner += m3.Manifold.batch_hull([inner_s[i], inner_s[i + 1]])
    outer -= box(-99, 99, HORN_LIP_Y, 99, -99, 99)     # 喇叭口截平
    return outer, inner


# ---------------- 前座 ----------------
def build_front():
    tr = [0, g.FRONT_HOOD_Y + PLATE_T, -HZ_TOP]
    outer, inner = build_horn()
    m = g.build_front() + outer.translate(tr)
    m -= inner.translate(tr)
    m -= g.plate_cuts().translate(tr)
    m -= box(-99, 99, -99, 99, 0, 99)                  # 底板下表面以上不留东西
    return m


# ---------------- 后座 ----------------
def build_rear():
    y_plate_back = g.GPS_FL[1] - g.REAR_GAP

    def place(x):
        return x.mirror([0, 0, 1]).rotate([0, 0, 180]).translate([0, y_plate_back - PLATE_T, HZ_TOP])

    outer, inner = build_horn()
    m = g.build_rear() + place(outer)
    m -= place(inner)
    m -= place(g.plate_cuts())
    m -= box(-99, 99, -99, 99, -99, 0)                 # 顶板上表面以下不留东西
    for sx in (-1, 1):                                 # 垫片上的机架螺丝孔保持畅通
        m -= cyl_z(sx * g.REAR_HOLE_X / 2, 0, -1, g.REAR_FL_T + 1, g.M2_CLEAR / 2)
    return m


def clearance_deg(tm, to_local):
    """喇叭/立板上所有比 IC 表面靠前的顶点，相对 IC 封装边缘的最小离轴角（度）。视场半角是 30°（对角约 39°）。"""
    v = to_local(np.asarray(tm.vertices))
    v = v[v[:, 1] > IC_Y + 0.05]
    dx = np.clip(np.abs(v[:, 0]) - IC_HALF[0], 0, None)
    dz = np.clip(np.abs(v[:, 2] - IC_Z) - IC_HALF[1], 0, None)
    ang = np.degrees(np.arctan2(np.hypot(dx, dz), v[:, 1] - IC_Y))
    ang_axis = np.degrees(np.arctan2(np.maximum(dx, dz), v[:, 1] - IC_Y))
    return ang.min(), ang_axis.min()


def export(man, name, rot_x):
    tm = g.to_trimesh(man)
    tm.export(os.path.join(HERE, f"{name}_装机姿态.stl"))
    pr = g.to_trimesh(man.rotate([rot_x, 0, 0]))
    pr.apply_translation([0, 0, -pr.bounds[0][2]])
    pr.export(os.path.join(HERE, f"{name}_打印姿态.stl"))
    vol = tm.volume / 1000
    print(f"{name}: watertight={tm.is_watertight} bodies={len(tm.split())} bbox={np.round(tm.extents, 1)} mm "
          f"vol={vol:.2f} cm3 PLA≈{vol * 1.24:.1f} g / PETG≈{vol * 1.27:.1f} g / TPU≈{vol * 1.21:.1f} g")
    return tm


if __name__ == "__main__":
    f = export(build_front(), "tof_front_喇叭", rot_x=-90)   # 喇叭口朝下贴床
    r = export(build_rear(), "tof_rear_喇叭", rot_x=90)
    fy, fz = g.FRONT_HOOD_Y + PLATE_T, -HZ_TOP
    a = clearance_deg(f, lambda v: v - [0, fy, fz])
    ry = g.GPS_FL[1] - g.REAR_GAP - PLATE_T
    b = clearance_deg(r, lambda v: np.c_[-v[:, 0], ry - v[:, 1], HZ_TOP - v[:, 2]])
    print(f"遮挡检查（IC 封装边缘到最近结构的离轴角，越大越好；视场半角 30°、对角 39°）："
          f"前座 {a[0]:.1f}°（按单轴 {a[1]:.1f}°），后座 {b[0]:.1f}°（按单轴 {b[1]:.1f}°）")
    lip = HORN_LIP_Y
    o = horn_offset(1.0)
    print(f"喇叭口：比 IC 表面靠前 {lip - IC_Y:.2f} mm；内口 {2 * (PX + o):.1f} × {2 * (PZ + o):.1f} mm，"
          f"口部张角 {math.degrees(math.atan(HORN_A + 2 * HORN_B)):.0f}°；"
          f"前座喇叭口在两孔前方 {fy + lip:.1f} mm，后座在两孔后方 {-(ry - lip):.1f} mm")
