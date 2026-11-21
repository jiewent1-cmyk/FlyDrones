"""Pololu #3418 VL53L7CX 载板的前后安装座 —— v4 平板版（2026-11-18）。

和 v3 护罩版（gen_tof_mounts.py）的区别：传感器一侧没有任何结构，只有一块立板，
载板背面用胶粘在立板上，视场完全不受遮挡。机架孔距、长条孔、垫片尺寸、载板位置都和 v3 相同。

立板正面有一个 0.4 mm 深的定位浅槽（载板外形 + 0.25 间隙），载板背面贴进去；槽沿比载板正面低 1.2 mm，
不进入视场。排针一侧开了通槽，排针焊点或导线从立板背面引出。两个 M2 自攻孔保留，可选（胶粘时不用）。

坐标：前座 —— 原点在两孔中点，z=0 是底板下表面（z 向上），+y = 机头；
      后座 —— 原点在两孔中点，z=0 是顶板上表面，+y = 机头。
运行：python gen_tof_plates.py
"""
import math
import os
import numpy as np
import manifold3d as m3
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
SEG = 40

# ---- 载板（和 gen_tof_mounts.py 一致）----
PCB_L, PCB_H, PCB_T = 17.8, 12.7, 1.6
HOLE_FROM_EDGE = 2.5
HOLE_DX = 12.7 / 2
IC_FROM_PIN_EDGE = 7.6
IC_HALF = (1.5, 3.2)
IC_H = 1.75
FIT, WALL = 0.25, 1.6
M2_CLEAR, M2_PILOT = 2.3, 1.7

PLATE_T = 2.0        # 立板厚度（载板背面以后）
RIM = 0.4            # 定位浅槽深度
PIN_SLOT = 3.0       # 排针通槽宽度（从排针侧长边往里）
BOTTOM_RIM = 1.0     # 孔侧长边外的槽沿宽度

# ---- 前座（和 v3 一致）----
FRONT_HOLE_X = 27.0
FRONT_PAD_T = 2.4
FRONT_SLOT_HALF = 1.5
FRONT_HOOD_Y = 14.0  # 立板背面在两孔前方的距离

# ---- 后座（和 v3 一致）----
REAR_HOLE_X = 16.0
REAR_FL_T = 2.4
GPS_FL = (REAR_HOLE_X + 7.0, -8.0, 7.5)
REAR_GAP = 1.0       # 立板背面离 GPS 法兰后缘的间隙
REAR_EXTRA_W = 2.0   # 后座立板每侧加宽（后面是 GPS 法兰，不能加筋，靠加宽保证排针槽两侧的强度）

PX, PZ = PCB_L / 2 + FIT, PCB_H / 2 + FIT
HX, HZ_TOP = PX + WALL, PZ + WALL
HOLE_Z = -PCB_H / 2 + HOLE_FROM_EDGE
IC_Z = PCB_H / 2 - IC_FROM_PIN_EDGE
IC_Y = PCB_T + IC_H


def box(x0, x1, y0, y1, z0, z1):
    return m3.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl_z(x, y, z0, z1, r):
    return m3.Manifold.cylinder(z1 - z0, r, r, SEG).translate([x, y, z0])


def cyl_y(x, z, y0, y1, r):
    return m3.Manifold.cylinder(y1 - y0, r, r, SEG).rotate([-90, 0, 0]).translate([x, y0, z])


# ---------------- 立板（局部坐标，和 v3 护罩相同）----------------
# 原点：载板背面中心；+y = 传感器朝向；载板长边沿 x；排针边在 +z，孔在 -z 边。
def plate_solid(extra_w=0.0):
    w = HX + extra_w
    return box(-w, w, -PLATE_T, RIM, -PZ - BOTTOM_RIM, HZ_TOP)


def plate_cuts():
    c = box(-PX, PX, 0, RIM + 30, -PZ, PZ)                                   # 定位浅槽（往前全部挖空）
    c += box(-PX + 0.6, PX - 0.6, -PLATE_T - 1, 0.01, PCB_H / 2 - PIN_SLOT, PZ)   # 排针通槽
    for sx in (-1, 1):
        c += cyl_y(sx * HOLE_DX, HOLE_Z, -PLATE_T - 1, 1, M2_PILOT / 2)      # 可选 M2 自攻孔
    return c


# ---------------- 前座 ----------------
def build_front():
    z_top, z_pad = 0.0, -FRONT_PAD_T
    tr = [0, FRONT_HOOD_Y + PLATE_T, z_top - HZ_TOP]
    y_back, y_rim = FRONT_HOOD_Y, FRONT_HOOD_Y + PLATE_T + RIM
    z_bot = tr[2] - PZ - BOTTOM_RIM
    m = plate_solid().translate(tr)
    for sx in (-1, 1):
        hx = sx * FRONT_HOLE_X / 2
        ax, ay = sx * (HX - 2.5), y_back
        m += m3.Manifold.batch_hull([cyl_z(hx - FRONT_SLOT_HALF, 0, z_pad, z_top, 3.5),
                                     cyl_z(hx + FRONT_SLOT_HALF, 0, z_pad, z_top, 3.5)])
        m += m3.Manifold.batch_hull([cyl_z(hx, 0, z_pad, z_top, 2.5), cyl_z(ax, ay, z_pad, z_top, 2.5)])
        # 三角加强筋：从臂上离螺丝 4 mm 处，连到立板背面侧边的整个高度（在排针槽外侧，不挡出线）
        d = math.hypot(ax - hx, ay)
        sx0, sy0 = hx + (ax - hx) * 4.0 / d, ay * 4.0 / d
        gx = sx * (HX - 0.8)
        m += m3.Manifold.batch_hull([cyl_z(sx0, sy0, z_pad - 0.5, z_top, 0.8),
                                     box(gx - 0.8, gx + 0.8, y_back - 0.5, y_back + 0.01, z_bot, z_top)])
        m -= m3.Manifold.batch_hull([cyl_z(hx - FRONT_SLOT_HALF, 0, z_pad - 1, z_top + 1, M2_CLEAR / 2),
                                     cyl_z(hx + FRONT_SLOT_HALF, 0, z_pad - 1, z_top + 1, M2_CLEAR / 2)])
    m -= box(-50, 50, y_rim, y_rim + 30, -50, 50)          # 槽沿平面以前不留任何东西
    m -= plate_cuts().translate(tr)
    return m


# ---------------- 后座 ----------------
def build_rear():
    w, y_back, y_front = GPS_FL
    m = box(-w / 2, w / 2, y_back, y_front, 0, REAR_FL_T)              # GPS 法兰垫片
    y_plate_back = y_back - REAR_GAP                                   # 立板背面（朝机头那面）

    def place(x):   # 上下翻转（排针朝下），再转 180° 朝后
        return x.mirror([0, 0, 1]).rotate([0, 0, 180]).translate([0, y_plate_back - PLATE_T, HZ_TOP])

    hw = HX + REAR_EXTRA_W
    m += place(plate_solid(REAR_EXTRA_W))
    m += box(-hw, hw, y_plate_back - 0.5, y_back + 0.01, 0, REAR_FL_T)  # 连接舌
    m -= place(plate_cuts())
    for sx in (-1, 1):
        m -= cyl_z(sx * REAR_HOLE_X / 2, 0, -1, REAR_FL_T + 1, M2_CLEAR / 2)
    return m


def to_trimesh(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts))


def export(man, name, flip):
    tm = to_trimesh(man)
    tm.export(os.path.join(HERE, f"{name}_装机姿态.stl"))
    pm = man.rotate([180, 0, 0]) if flip else man
    pr = to_trimesh(pm)
    pr.apply_translation([0, 0, -pr.bounds[0][2]])
    pr.export(os.path.join(HERE, f"{name}_打印姿态.stl"))
    vol = tm.volume / 1000
    print(f"{name}: watertight={tm.is_watertight} bbox={np.round(tm.extents, 1)} mm vol={vol:.2f} cm3 "
          f"PLA≈{vol * 1.24:.1f} g / PETG≈{vol * 1.27:.1f} g（100% 填充）")
    return tm


if __name__ == "__main__":
    export(build_front(), "tof_front_平板", flip=True)   # 圆垫和臂的上表面贴床
    export(build_rear(), "tof_rear_平板", flip=False)    # 法兰贴床
    print(f"前座：IC 中心在两孔前方 {FRONT_HOOD_Y + PLATE_T + IC_Y:.1f} mm、底板下方 {HZ_TOP - IC_Z:.1f} mm；"
          f"最前面是 IC 表面，槽沿在两孔前方 {FRONT_HOOD_Y + PLATE_T + RIM:.1f} mm")
    print(f"后座：IC 中心在两孔后方 {-(GPS_FL[1] - REAR_GAP - PLATE_T - IC_Y):.1f} mm、顶板上方 {HZ_TOP - IC_Z:.1f} mm")
