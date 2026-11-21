"""MicoAir MTF-01 下视光流/测距底座（MOD-L 底板，左右 36 × 前后 40 mm 四孔，和 3901-L0X 底座同一组孔）。

结构：一块贴底板的平板 + 4 根支撑柱。MTF-01 的接口面朝机架、镜头朝地面，4 颗 M2 自攻螺丝从下面穿过
载板孔（24.3 × 12，Ø2.5）拧进支撑柱。三面护墙（机尾侧和左右两侧）比镜筒端面长出 GUARD_PROUD，
落地或磕碰先碰到护墙；机头一侧（测距发射芯片和接口那一边）不做墙，测距和光流的视场都不被遮挡。

坐标 = 打印姿态：z=0 是贴机架底板的平面（打印时贴热床），z 向上 = 装机后朝地面；-y = 机头。
载板长边沿 x（横向），两个镜筒在机尾一侧，测距发射芯片在机头一侧（ArduPilot/PX4 默认朝向）。

运行：python gen_mtf01_mount.py [--preview]
"""
import math
import os
import sys
import numpy as np
import manifold3d as m3
import trimesh

SEG = 48

# ---- MTF-01（官方尺寸图：29.3 × 17 × 14.5 mm，孔距 24.3 × 12，Ø2.5）----
PCB_L, PCB_W = 29.3, 17.0
PCB_T = 1.4            # 板厚（尺寸图侧视量得约 1.4）【实测】
HOLE_X, HOLE_Y = 24.3, 12.0
CONN_H = 2.9           # 背面 SH1.0-4P 卧式座高度（尺寸图量得约 2.8）
TUBE_H = 10.0          # 镜筒高出载板正面（尺寸图量得约 9.95）【实测】
TUBE_D, TUBE_DX, TUBE_FROM_EDGE = 8.0, 5.05, 5.2   # 镜筒直径、两筒离中心、筒心到机尾侧长边（尺寸图/照片量得）

# ---- 底座 ----
PLATE_T = 2.0
STAND_H = 4.0          # 支撑柱高度 = 载板背面到平板的间隙（接口座 2.9 + 插头和线的余量）
STAND_D = 5.0
PILOT_D = 1.7          # M2 自攻底孔
FIT = 0.5              # 载板边到护墙的间隙
GUARD_T = 2.0          # 护墙厚度
GUARD_PROUD = 1.5      # 护墙比镜筒端面长出的量
WIRE_NOTCH_W = 16.0    # 机尾侧护墙上的备用走线缺口（接口如果在这一边）
PAD_R, ARM_W = 3.5, 5.0
SCREW_X, SCREW_Y = 36.0, 40.0   # 底板孔位（用户 10-15 复测）
SCREW_D = 2.3
SLOT_HALF = 2.0        # 长条孔，沿对角线方向各 2 mm

Z_PCB_BACK = PLATE_T + STAND_H
Z_PCB_FRONT = Z_PCB_BACK + PCB_T
Z_TUBE = Z_PCB_FRONT + TUBE_H
Z_GUARD = Z_TUBE + GUARD_PROUD
IX, IY = PCB_L / 2 + FIT, PCB_W / 2 + FIT      # 护墙内侧
OX, OY = IX + GUARD_T, IY + GUARD_T            # 护墙外侧 = 平板主体外形


def box(x0, x1, y0, y1, z0, z1):
    return m3.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl(x, y, z0, z1, r0, r1=None):
    r1 = r0 if r1 is None else r1
    return m3.Manifold.cylinder(z1 - z0, r0, r1, SEG).translate([x, y, z0])


def slot_ends(sx, sy):
    hx, hy = sx * SCREW_X / 2, sy * SCREW_Y / 2
    d = math.hypot(hx, hy)
    ux, uy = hx / d * SLOT_HALF, hy / d * SLOT_HALF
    return (hx - ux, hy - uy), (hx + ux, hy + uy)


def build():
    body = box(-OX, OX, -OY, OY, 0, PLATE_T)
    # 四条臂连到底板螺丝孔
    for sx in (-1, 1):
        for sy in (-1, 1):
            (ax0, ay0), (ax1, ay1) = slot_ends(sx, sy)
            body += m3.Manifold.batch_hull([cyl(ax0, ay0, 0, PLATE_T, PAD_R), cyl(ax1, ay1, 0, PLATE_T, PAD_R),
                                            cyl(sx * (OX - ARM_W / 2), sy * (OY - ARM_W / 2), 0, PLATE_T, ARM_W / 2)])
    # 机头标记：平板机头一侧的三角凸耳
    tri = m3.CrossSection([[(-3.0, -OY + 0.01), (0.0, -OY - 3.5), (3.0, -OY + 0.01)]])
    body += m3.Manifold.extrude(tri, PLATE_T)
    # 三面护墙（机尾侧 + 左右），机头侧敞开；顶边外侧倒角
    ch = 1.0
    for x0, x1, y0, y1 in ((-OX, OX, IY, OY), (-OX, -IX, -OY, OY), (IX, OX, -OY, OY)):
        body += box(x0, x1, y0, y1, 0, Z_GUARD - ch)
    top = box(-OX + ch, OX - ch, -OY + ch, OY - ch, Z_GUARD - ch - 0.01, Z_GUARD)
    low = box(-OX, OX, -OY, OY, Z_GUARD - ch - 0.02, Z_GUARD - ch)
    cap = m3.Manifold.batch_hull([low, top])
    cap -= box(-IX, IX, -OY - 1, IY, 0, Z_GUARD + 1)
    body += cap
    # 机尾侧护墙上的备用走线缺口（平板到载板背面之间）
    body -= box(-WIRE_NOTCH_W / 2, WIRE_NOTCH_W / 2, IY - 0.5, OY + 1, PLATE_T, Z_PCB_BACK + 1.5)
    # 支撑柱 + 自攻底孔（打穿平板）
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = sx * HOLE_X / 2, sy * HOLE_Y / 2
            body += cyl(x, y, PLATE_T - 0.01, Z_PCB_BACK, STAND_D / 2 + 0.8, STAND_D / 2)
    for sx in (-1, 1):
        for sy in (-1, 1):
            body -= cyl(sx * HOLE_X / 2, sy * HOLE_Y / 2, -1, Z_PCB_BACK + 1, PILOT_D / 2)
            (ax0, ay0), (ax1, ay1) = slot_ends(sx, sy)
            body -= m3.Manifold.batch_hull([cyl(ax0, ay0, -1, PLATE_T + 1, SCREW_D / 2),
                                            cyl(ax1, ay1, -1, PLATE_T + 1, SCREW_D / 2)])
    return body


def sensor_dummy():
    """载板 + 镜筒的示意体，只用于预览和干涉检查。"""
    s = box(-PCB_L / 2, PCB_L / 2, -PCB_W / 2, PCB_W / 2, Z_PCB_BACK, Z_PCB_FRONT)
    for sx in (-1, 1):
        s += cyl(sx * TUBE_DX, PCB_W / 2 - TUBE_FROM_EDGE, Z_PCB_FRONT, Z_TUBE, TUBE_D / 2)
    return s


def to_trimesh(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts))


if __name__ == "__main__":
    out = os.path.dirname(os.path.abspath(__file__))
    man = build()
    tm = to_trimesh(man)
    tm.export(os.path.join(out, "mtf01_mount_打印姿态.stl"))
    inst = to_trimesh(man.rotate([180, 0, 0]))     # 装机姿态：z 向上，贴板面在上，+y = 机头
    inst.export(os.path.join(out, "mtf01_mount_装机姿态.stl"))
    vol = tm.volume / 1000
    clash = (man ^ sensor_dummy()).volume()
    print(f"watertight={tm.is_watertight} bodies={len(tm.split())} bbox={np.round(tm.extents, 1)} mm "
          f"vol={vol:.2f} cm3 PLA≈{vol * 1.24:.1f} g / PETG≈{vol * 1.27:.1f} g")
    print(f"z（底板下方）：载板背面 {Z_PCB_BACK}，载板正面 {Z_PCB_FRONT}，镜筒端面 {Z_TUBE}，护墙底 {Z_GUARD}；"
          f"和传感器示意体的干涉体积 {clash:.3f} mm3")
    # 遮挡：护墙端面内沿到镜筒端面边缘的离轴角（光流视场半角 21°）
    side = math.degrees(math.atan((IX - TUBE_DX - TUBE_D / 2) / GUARD_PROUD))
    rear = math.degrees(math.atan((IY - (PCB_W / 2 - TUBE_FROM_EDGE) - TUBE_D / 2) / GUARD_PROUD))
    print(f"护墙相对镜筒端面边缘的离轴角：左右 {side:.0f}°，机尾侧 {rear:.0f}°（光流半角 21°，测距半角 3°）")
    if "--preview" in sys.argv:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        plt.rcParams["font.sans-serif"] = ["PingFang SC", "Arial Unicode MS"]
        sd = to_trimesh(sensor_dummy())
        fig = plt.figure(figsize=(14, 6))
        for i, (el, az, title, with_s) in enumerate(((35, -60, "打印姿态（朝地面的一面在上，-y = 机头）", False),
                                                     (35, -60, "加传感器示意（橙色）", True),
                                                     (35, 120, "从机尾看", True))):
            ax = fig.add_subplot(1, 3, i + 1, projection="3d")
            for mesh, col in ((tm, (0.3, 0.55, 0.9)),) + (((sd, (0.95, 0.6, 0.2)),) if with_s else ()):
                sh = 0.45 + 0.5 * np.clip(mesh.face_normals @ np.array([0.3, -0.4, 0.85]), 0, 1)
                pc = Poly3DCollection(mesh.vertices[mesh.faces], linewidths=0)
                pc.set_facecolor(np.outer(sh, col))
                ax.add_collection3d(pc)
            lim = 24
            ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim + 9, lim + 9)
            ax.view_init(el, az); ax.set_box_aspect((1, 1, 1)); ax.set_title(title)
            ax.set_xlabel("x"); ax.set_ylabel("y")
        plt.tight_layout()
        plt.savefig(os.path.join(out, "preview.png"), dpi=90)
