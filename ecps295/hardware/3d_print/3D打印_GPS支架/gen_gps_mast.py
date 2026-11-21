"""HGLRC M100 Pro GPS 支架（一体件：底座 + 桁架杆 + 托盘），固定在机架顶板尾端的两个孔上。

对称布局（从后往前看是一个"工"字）：
  底座法兰横跨两孔，两颗 M2 螺丝左右横向分布（孔距 16 mm），杆子立在两孔正中间；
  杆根左右两片三角筋 + 后面一片三角筋；
  托盘在杆顶，向后伸出，GPS 中心在机身中线上、略靠后；托盘上对着螺丝开了让刀孔。

坐标（装机姿态）：z=0 = 顶板上表面；+y = 机头；x = 左右（两孔在 x=±HOLE_PITCH/2，y=HOLE_Y）。
打印姿态：+y 面（杆子前面、法兰前边、托盘前边、左右三角筋）贴床，桁架孔竖直，层纹沿杆长方向，免支撑。

运行：python gen_gps_mast.py
"""
import os
import numpy as np
import manifold3d as m3
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
SEG = 48

# ---- 机架尾端孔位 ----
HOLE_PITCH = 16.0    # 【实测】两孔横向间距（10-15 更正，原记约 17）
M2_CLEAR = 2.3

# ---- 高度 ----
GPS_BOTTOM_Z = 90.0  # GPS 底面离顶板高度（10-29 用户要求加高 30 mm，原 60）；电池顶面约 46.5 mm（Pi 罩 28 + 电池 18.5）

# ---- 杆子（桁架在 xz 平面，桁架孔沿 y 方向穿透）----
W = 10.0             # x 向（左右）宽度
D = 7.0              # y 向（前后）厚度：前后受力靠边梁的这个厚度
RAIL, DIAG, BAY = 2.2, 1.6, 8.0
Y_FRONT = D / 2      # 杆子前面 = 打印贴床面

# ---- 底座法兰 ----
FL_T = 2.4
FL_X = HOLE_PITCH + 7.0          # 法兰左右总宽
FL_Y0 = -12.0                    # 法兰后缘（前缘 = Y_FRONT）
HOLE_Y = -4.0                    # 孔位在杆子中心后方 4 mm，法兰前后都有支撑长度
GUSSET_T, GUSSET_H = 1.6, 14.0

# ---- 托盘（向后伸出）----
GPS = 21.2
PLAT_T = 1.6
PLAT_X = GPS + 6.0
PLAT_Y0 = Y_FRONT - (GPS + 6.0)  # 托盘后缘
Z_PLAT = GPS_BOTTOM_Z - PLAT_T
TIE_W, TIE_L = 1.8, 4.5
ACCESS_D = 4.5       # 托盘上的让刀孔（对准下面的螺丝）


def box(x0, x1, y0, y1, z0, z1):
    return m3.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def bar(p0, p1, w):
    """xz 平面内的斜撑，y 向厚度 D。"""
    a = box(p0[0] - w / 2, p0[0] + w / 2, -D / 2, D / 2, p0[1] - w / 2, p0[1] + w / 2)
    b = box(p1[0] - w / 2, p1[0] + w / 2, -D / 2, D / 2, p1[1] - w / 2, p1[1] + w / 2)
    return m3.Manifold.batch_hull([a, b])


def _ccw(pts):
    (x0, y0), (x1, y1), (x2, y2) = pts
    return pts if (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0) > 0 else [pts[0], pts[2], pts[1]]


def tri_xz(pts, y0, t):
    """xz 平面三角板，y 从 y0 到 y0+t。"""
    cs = m3.CrossSection([_ccw(pts)])
    return m3.Manifold.extrude(cs, t).rotate([90, 0, 0]).translate([0, y0 + t, 0])


def tri_yz(pts, x0, t):
    """yz 平面三角板，x 从 x0 到 x0+t。"""
    cs = m3.CrossSection([_ccw(pts)])
    return m3.Manifold.extrude(cs, t).rotate([90, 0, 90]).translate([x0, 0, 0])


def build():
    hw = W / 2
    # 底座法兰
    m = box(-FL_X / 2, FL_X / 2, FL_Y0, Y_FRONT, 0, FL_T)
    # 杆根实心段
    m += box(-hw, hw, -D / 2, D / 2, 0, FL_T + 3.0)
    # 边梁
    m += box(-hw, -hw + RAIL, -D / 2, D / 2, FL_T, Z_PLAT + 0.01)
    m += box(hw - RAIL, hw, -D / 2, D / 2, FL_T, Z_PLAT + 0.01)
    # 斜撑
    z0, z1 = FL_T + 3.0, Z_PLAT - 2.0
    n = max(1, round((z1 - z0) / BAY))
    step = (z1 - z0) / n
    xi = hw - RAIL / 2
    for i in range(n):
        xa, xb = (-xi, xi) if i % 2 == 0 else (xi, -xi)
        m += bar((xa, z0 + i * step), (xb, z0 + (i + 1) * step), DIAG)
    m += box(-hw, hw, -D / 2, D / 2, Z_PLAT - 2.0, Z_PLAT + 0.01)

    # 杆根左右三角筋（xz 平面，贴前面 = 打印床面）。孔位在 y=HOLE_Y，螺丝头不会碰到前面的筋，
    # 所以筋可以一直伸到法兰两端
    for sx in (-1, 1):
        xe = sx * (FL_X / 2 - 0.5)
        m += tri_xz([(sx * (hw - 0.01), FL_T - 0.01), (xe, FL_T - 0.01), (sx * (hw - 0.01), FL_T + GUSSET_H)],
                    Y_FRONT - GUSSET_T, GUSSET_T)
    # 杆根后三角筋（yz 平面，中线上）
    m += tri_yz([(-D / 2 + 0.01, FL_T - 0.01), (FL_Y0, FL_T - 0.01), (-D / 2 + 0.01, FL_T + GUSSET_H)],
                -GUSSET_T / 2, GUSSET_T)

    # 托盘 + 托盘下的两片三角筋（yz 平面，压在边梁上）
    m += box(-PLAT_X / 2, PLAT_X / 2, PLAT_Y0, Y_FRONT, Z_PLAT, Z_PLAT + PLAT_T)
    for x0 in (-hw, hw - GUSSET_T):
        m += tri_yz([(-D / 2 + 0.01, Z_PLAT - GUSSET_H), (PLAT_Y0 + 3.0, Z_PLAT + 0.01),
                     (-D / 2 + 0.01, Z_PLAT + 0.01)], x0, GUSSET_T)

    # 机架螺丝孔（左右横向分布）+ 托盘让刀孔
    for sx in (-1, 1):
        x = sx * HOLE_PITCH / 2
        m -= m3.Manifold.cylinder(FL_T + 2, M2_CLEAR / 2, M2_CLEAR / 2, SEG).translate([x, HOLE_Y, -1])
        m -= m3.Manifold.cylinder(PLAT_T + 2, ACCESS_D / 2, ACCESS_D / 2, SEG) \
            .translate([x, HOLE_Y, Z_PLAT - 1])
    # 扎带槽：托盘左右边各一个，扎带沿 x 方向压过 GPS 顶面
    cy = (PLAT_Y0 + Y_FRONT) / 2
    for sx in (-1, 1):
        xx = sx * (GPS / 2 + 0.6)
        m -= box(min(xx, xx + sx * TIE_W), max(xx, xx + sx * TIE_W), cy - TIE_L / 2, cy + TIE_L / 2,
                 Z_PLAT - 1, Z_PLAT + PLAT_T + 1)
    return m


def to_trimesh(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts))


if __name__ == "__main__":
    m = build()
    tm = to_trimesh(m)
    tm.export(os.path.join(HERE, "gps_mast_装机姿态.stl"))   # 只用来查看/装配，不要直接打印
    pm = m.rotate([-90, 0, 0])                                # +y 面朝下贴床
    pr = to_trimesh(pm)
    zmin = pr.bounds[0][2]
    pr.apply_translation([0, 0, -zmin])
    pr.export(os.path.join(HERE, "gps_mast_打印姿态.stl"))
    vol = tm.volume / 1000
    print(f"watertight={tm.is_watertight} bbox={np.round(tm.extents, 1)} mm vol={vol:.2f} cm3 "
          f"PETG≈{vol * 1.27:.1f} g（100% 填充）")
    print(f"孔位 x=±{HOLE_PITCH / 2}, y={HOLE_Y}；GPS 中心 x=0, y={(PLAT_Y0 + Y_FRONT) / 2:.1f}；"
          f"GPS 底面 z={GPS_BOTTOM_Z}")
    print("bed contact area (mm2):", round(pm.translate([0, 0, -zmin]).slice(0.05).area(), 1))
