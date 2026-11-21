"""Matek 3901-L0X 下视光流/测距保护底座（MOD-L HD Cargo 底板，左右 36 × 前后 40 mm 四孔）。

坐标 = 打印姿态：z=0 是贴机架底板的平面（打印时朝下贴热床），z 向上 = 装机后朝地面。
-y 方向 = 机头（与传感器上的小箭头一致）；+y 端是焊线端，走线缺口在这一端。

运行：python gen_mount.py   → 输出 3901L0X_mount.stl 和预览图。
标【实测】的参数请用游标卡尺量过传感器后再改。
"""
import math
import numpy as np
import manifold3d as m3
import trimesh

SEG = 64

# ---- 传感器（Matek 3901-L0X，F051 36x12 / L051 36x13）----
# u = 横向（看元件面、焊线端朝上时距左边），v = 距焊线端。位置来自实物照片 IMG_7328 的透视校正，
# 以实测孔距 30 mm 标定，误差约 ±0.5 mm。
PCB_L, PCB_W = 36.0, 13.0   # 实物为 13 mm 宽版本
PCB_T = 1.2          # 【实测】板厚，取偏大值，多余用泡棉胶补
COMP_H = 3.6         # 【实测】元件面最高元件（镜头模组）高度
HOLE_V = (3.0, 33.0) # 两个安装孔距焊线端（实测孔距 30 mm）
HOLE_U = 2.7         # 安装孔中心距左边（照片估算）
TOF_UV = (8.2, 11.6) # VL53L0X（照片估算）
LENS_UV = (9.2, 29.2)# PMW3901 镜头中心（照片估算，靠远端安装孔）

# ---- 底座 ----
FIT = 0.3            # 槽位单边间隙
POCKET_W = PCB_W + 2 * FIT
POCKET_L = PCB_L + 2 * FIT
BACK_GAP = 1.5       # 板背面到机架的间隙：容纳背面焊点，贴 1–1.5 mm 泡棉双面胶
WALL = 1.0
FLOOR_T = 1.2
SKIRT_H = 2.5        # 护裙比底面再伸出的高度（护镜头）
HALF_ANGLE = 32.0    # 窗口半锥角；光流 FOV 42°（半角 21°）、测距 27°，留余量
PIN_D, PIN_TIP_D, PIN_H, POST_D = 1.4, 1.0, 1.0, 3.4   # 定位销：第一版 Ø1.8 插不进，改 Ø1.4、尖端收到 Ø1.0 导向

ARM_T = 2.0          # 连接臂厚度
ARM_W = 5.0          # 连接臂宽度
PAD_R = 3.5          # 螺丝孔处圆垫半径
SCREW_X, SCREW_Y = 36.0, 40.0   # 实测（10-15 复测）：底板孔位 左右 36、前后 40
SCREW_D = 2.3        # M2 过孔
SLOT_HALF = 2.0      # 螺丝孔做成沿臂方向（对角线）的长条孔，中心两侧各可滑 2 mm，吸收孔位测量误差
WIRE_NOTCH_W = 10.0

Z_PCB_TOP = BACK_GAP
Z_PCB_BOT = BACK_GAP + PCB_T
Z_FLOOR = Z_PCB_BOT + COMP_H
Z_FLOOR_BOT = Z_FLOOR + FLOOR_T
Z_SKIRT = Z_FLOOR_BOT + SKIRT_H

HX = POCKET_W / 2 + WALL
HY = POCKET_L / 2 + WALL


def uv_to_xy(u, v):
    """传感器板坐标 → 底座坐标（板居中，焊线端在 +y）。"""
    return u - PCB_W / 2, PCB_L / 2 - v


def box(x0, x1, y0, y1, z0, z1):
    return m3.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl(x, y, z0, z1, r0, r1=None):
    r1 = r0 if r1 is None else r1
    return m3.Manifold.cylinder(z1 - z0, r0, r1, SEG).translate([x, y, z0])


def window(extra, z0, z1):
    """镜头 + 测距窗口：两个锥台的凸包，随高度按 HALF_ANGLE 张开。"""
    t = math.tan(math.radians(HALF_ANGLE))
    parts = []
    for (u, v), r in ((TOF_UV, 2.8), (LENS_UV, 3.5)):
        x, y = uv_to_xy(u, v)
        dz0, dz1 = z0 - Z_FLOOR, z1 - Z_FLOOR
        parts.append(cyl(x, y, z0, z1, r + extra + dz0 * t, r + extra + dz1 * t))
    return m3.Manifold.batch_hull(parts)


def slot_ends(sx, sy):
    """长条孔两端圆心：以名义孔位为中心，沿底座中心指向孔位的方向各伸 SLOT_HALF。"""
    hx, hy = sx * SCREW_X / 2, sy * SCREW_Y / 2
    d = math.hypot(hx, hy)
    ux, uy = hx / d * SLOT_HALF, hy / d * SLOT_HALF
    return (hx - ux, hy - uy), (hx + ux, hy + uy)


def build():
    body = box(-HX, HX, -HY, HY, 0, Z_FLOOR_BOT)
    # X 形连接臂：每个螺丝孔一条臂连到壳体
    for sx in (-1, 1):
        for sy in (-1, 1):
            (ax0, ay0), (ax1, ay1) = slot_ends(sx, sy)
            body += m3.Manifold.batch_hull([cyl(ax0, ay0, 0, ARM_T, PAD_R), cyl(ax1, ay1, 0, ARM_T, PAD_R),
                                            cyl(sx * (HX - ARM_W / 2), sy * 10.0, 0, ARM_T, ARM_W / 2)])
    body += window(1.2, Z_FLOOR_BOT - 0.01, Z_SKIRT)          # 护裙外形

    # 机头箭头（凸起 0.4 mm，指向 -y）
    ax, ay = -HX + 2.6, HY - 5.5
    tri = m3.CrossSection([[(ax, ay), (ax + 1.8, ay + 3.0), (ax - 1.8, ay + 3.0)]])
    body += m3.Manifold.extrude(tri, 0.4).translate([0, 0, Z_FLOOR_BOT])

    # 传感器槽（从贴板面开口，一直到窗口底板）
    body -= box(-POCKET_W / 2, POCKET_W / 2, -POCKET_L / 2, POCKET_L / 2, -1, Z_FLOOR)
    # 焊线端走线缺口
    body -= box(-WIRE_NOTCH_W / 2, WIRE_NOTCH_W / 2, POCKET_L / 2 - 0.5, HY + 1, -1, Z_PCB_BOT + 0.8)
    # 窗口
    body -= window(0.0, Z_FLOOR - 0.5, Z_SKIRT + 0.5)
    # M2 螺丝过孔
    for sx in (-1, 1):
        for sy in (-1, 1):
            (ax0, ay0), (ax1, ay1) = slot_ends(sx, sy)
            body -= m3.Manifold.batch_hull([cyl(ax0, ay0, -1, ARM_T + 1, SCREW_D / 2),
                                            cyl(ax1, ay1, -1, ARM_T + 1, SCREW_D / 2)])

    # 安装孔焊环处的支撑柱（只有焊环区域没有元件）+ 定位销。
    # 第一版销子 Ø1.8 插不进孔（10-15 试装），改细并做锥形导向尖，单边留约 0.3 mm 吸收孔位估算误差。
    for v in HOLE_V:
        x, y = uv_to_xy(HOLE_U, v)
        body += cyl(x, y, Z_PCB_BOT, Z_FLOOR + 0.01, POST_D / 2)
        body += cyl(x, y, Z_PCB_BOT - PIN_H, Z_PCB_BOT + 0.01, PIN_TIP_D / 2, PIN_D / 2)
    return body


def to_trimesh(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts))


if __name__ == "__main__":
    import os, sys
    out = os.path.dirname(os.path.abspath(__file__))
    tm = to_trimesh(build())
    tm.export(os.path.join(out, "3901L0X_mount.stl"))
    vol = tm.volume / 1000
    print(f"watertight={tm.is_watertight} bbox={np.round(tm.extents, 2)} mm "
          f"volume={vol:.2f} cm3  PLA≈{vol*1.24:.1f} g  PETG≈{vol*1.27:.1f} g  TPU≈{vol*1.21:.1f} g")
    print(f"z: 板背={Z_PCB_TOP} 元件面={Z_PCB_BOT} 窗口底板={Z_FLOOR}-{Z_FLOOR_BOT} 护裙底={Z_SKIRT}")
    if "--preview" in sys.argv:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        fig = plt.figure(figsize=(12, 6))
        for i, (el, az, title) in enumerate(((35, -60, "贴板面（朝上）"), (-35, -60, "朝地面"))):
            ax = fig.add_subplot(1, 2, i + 1, projection="3d")
            pc = Poly3DCollection(tm.vertices[tm.faces], linewidths=0.05, edgecolor="k")
            n = tm.face_normals
            shade = 0.45 + 0.5 * np.clip(n @ np.array([0.3, -0.4, 0.85]), 0, 1)
            pc.set_facecolor(np.c_[shade * 0.3, shade * 0.55, shade * 0.9])
            ax.add_collection3d(pc)
            lim = 21
            ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim + 5, lim + 5)
            ax.view_init(el, az); ax.set_box_aspect((1, 1, 1))
            ax.set_xlabel("x"); ax.set_ylabel("y (-y = nose)")
        plt.tight_layout()
        plt.savefig(os.path.join(out, "preview.png"), dpi=110)
