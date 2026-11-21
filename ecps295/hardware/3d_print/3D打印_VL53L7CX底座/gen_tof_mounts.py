"""Pololu #3418 VL53L7CX 载板的前后两个安装座（带防撞护罩）。

前座 tof_front：挂在机头下方单独的两个孔（横向 27 mm）下面，传感器朝正前；不和光流底座共用螺丝。
后座 tof_rear：垫在顶板尾端两个孔（横向 16 mm）上、GPS 支架底座下面，兼做 GPS 法兰的垫片；
    传感器朝正后，护罩在 GPS 法兰后缘的后面。

护罩：载板从开口一侧插进槽里，2 颗 M2×8 从护罩背面穿过载板孔，拧进前面板的自攻孔；
    前面板盖住载板边缘，只在传感器 IC 前面开一个方锥窗口（视场 60°×60°）。
    v3（2026-11-05）：v2 窗口按估算的开口区域（1.6×5.0）+ 35° 半角开，实测太小，影响识别。
    现在从整个 IC 封装（6.4×3.0）外扩 WIN_MARGIN 起算，按 45° 半角张开（视场半角 30° + 15°，
    发射端照明锥比接收视场宽，IC 位置是照片估算的，也一并留余量）；前面板 2.5 → 2.2 mm，
    IC 离面板外表面更近；前面板往槽开口一侧加长 WIN_EXT，窗口变大后下沿仍然封闭。
    IC 表面缩在前面板后约 2.7 mm，正面磕碰先碰到护罩。前缘倒角，便于擦撞时滑开。

坐标：前座 —— 原点在两孔中点，z=0 是底板下表面（z 向上），+y = 机头；
      后座 —— 原点在两孔中点，z=0 是顶板上表面，+y = 机头。
运行：python gen_tof_mounts.py
"""
import math
import os
import numpy as np
import manifold3d as m3
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
SEG = 40

# ---- Pololu #3418 载板（官网尺寸图：0.5″ × 0.7″，两孔 Ø0.086″、间距 0.5″）----
PCB_L, PCB_H, PCB_T = 17.8, 12.7, 1.6     # 长 × 宽 × 板厚（板厚按 1/16″ 估）【实测】
HOLE_FROM_EDGE = 2.5                      # 孔中心到孔侧长边的距离（正视照片量得约 2.45）
HOLE_DX = 12.7 / 2                        # 两孔沿长边各距中心 6.35
IC_FROM_PIN_EDGE = 7.6                    # 传感器 IC 中心到排针侧长边（正视照片量得）
IC_HALF = (1.5, 3.2)                      # VL53L7CX 封装 6.4 × 3.0 mm 的一半；长边垂直于排针边（x 1.5、z 3.2）
APER_HALF = (0.8, 2.5)                    # IC 顶面发射/接收开口所在区域的一半（照片估算），窗口从这里起算
FRONT_CLEAR = 2.0                         # 载板正面元件的净空（IC 高 1.75）
FIT, WALL, BACK_T, FACE_T = 0.25, 1.6, 2.0, 2.2   # FACE_T v2 是 2.5
HALF_ANGLE = 45.0                         # 窗口半角（视场半角 30° + 15° 余量；v2 是 35°）
WIN_MARGIN = 0.4                          # 窗口起算区域 = IC 封装每边外扩这么多（v2 从开口区域 APER_HALF 起算）
WIN_EXT = 2.2                             # 前面板往槽开口一侧（-z）加长，包住变大的窗口下沿
M2_CLEAR, M2_PILOT = 2.3, 1.7

# ---- 前座 ----
FRONT_HOLE_X = 27.0
LIDAR_STACK_T = 0.0     # 圆垫上方叠装的垫片厚度（前座有自己的孔，不叠装，为 0）
FRONT_PAD_T = 2.4
FRONT_SLOT_HALF = 1.5   # 前座螺丝孔做成横向长条孔，中心两侧各 1.5 mm，吸收孔距测量误差
FRONT_HOOD_Y = 14.0     # 护罩背面在两孔前方的距离（越大越能避开前桨，但更容易先撞）

# ---- 后座 ----
REAR_HOLE_X = 16.0
REAR_FL_T = 2.4
GPS_FL = (REAR_HOLE_X + 7.0, -8.0, 7.5)   # GPS 底座法兰：宽、后缘 y、前缘 y（相对两孔），后座要垫满这块
REAR_HOOD_GAP = 1.0          # 护罩背面离 GPS 法兰后缘的间隙


def box(x0, x1, y0, y1, z0, z1):
    return m3.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl_z(x, y, z0, z1, r):
    return m3.Manifold.cylinder(z1 - z0, r, r, SEG).translate([x, y, z0])


def cyl_y(x, z, y0, y1, r):
    return m3.Manifold.cylinder(y1 - y0, r, r, SEG).rotate([-90, 0, 0]).translate([x, y0, z])


# ---------------- 护罩（局部坐标）----------------
# 原点：载板背面中心；+y = 传感器朝向；载板长边沿 x；排针边在 +z，孔在 -z 边；槽从 -z 侧插入。
PX, PZ = PCB_L / 2 + FIT, PCB_H / 2 + FIT
Y_PCB = PCB_T + 0.2
Y_FACE_IN = Y_PCB + FRONT_CLEAR
Y_FACE_OUT = Y_FACE_IN + FACE_T
HX, HZ_TOP = PX + WALL, PZ + WALL
HOLE_Z = -PCB_H / 2 + HOLE_FROM_EDGE
IC_Z = PCB_H / 2 - IC_FROM_PIN_EDGE
IC_Y = PCB_T + 1.75


def window(z0_extra=0.0):
    t = math.tan(math.radians(HALF_ANGLE))
    y0, y1 = IC_Y, Y_FACE_OUT + 1.0
    hx0, hz0 = IC_HALF[0] + WIN_MARGIN, IC_HALF[1] + WIN_MARGIN
    a = box(-hx0, hx0, y0, y0 + 0.01, IC_Z - hz0, IC_Z + hz0)
    gx, gz = hx0 + (y1 - y0) * t, hz0 + (y1 - y0) * t
    b = box(-gx, gx, y1 - 0.01, y1, IC_Z - gz, IC_Z + gz)
    return m3.Manifold.batch_hull([a, b])


def build_hood():
    ch = 1.5   # 前缘倒角
    rear = box(-HX, HX, -BACK_T, Y_FACE_OUT - ch, -PZ, HZ_TOP)
    front = box(-HX + ch, HX - ch, Y_FACE_OUT - 0.01, Y_FACE_OUT, -PZ, HZ_TOP - ch)
    h = m3.Manifold.batch_hull([rear, front])
    # 前面板往 -z（槽开口一侧）加长 WIN_EXT：只加在面板厚度范围内，不挡载板和元件从 -z 插入
    h += m3.Manifold.batch_hull([box(-HX, HX, Y_FACE_IN, Y_FACE_OUT - ch, -PZ - WIN_EXT, -PZ + 0.01),
                                 box(-HX + ch, HX - ch, Y_FACE_OUT - 0.01, Y_FACE_OUT, -PZ - WIN_EXT, -PZ + 0.01)])
    # 载板槽（-z 侧开口）+ 正面元件腔
    h -= box(-PX, PX, 0, Y_PCB, -PZ - 1, PZ)
    h -= box(-PX + 0.6, PX - 0.6, Y_PCB - 0.01, Y_FACE_IN, -PZ - 1, PZ)
    # 不做孔位凸台：载板沿 z 滑进槽时，孔所在那一列的电容会被凸台挡住。
    # 载板靠槽两端的窄边和两颗螺丝定位，螺丝只穿过元件腔，不夹紧载板。
    # 视场窗口
    h -= window()
    # 背板：排针出线槽、载板螺丝过孔；前面板自攻孔（v3 改成通孔，M2×8 尖端齐平）
    h -= box(-PX + 0.6, PX - 0.6, -BACK_T - 1, 0.01, PCB_H / 2 - 3.0, PZ)
    for sx in (-1, 1):
        h -= cyl_y(sx * HOLE_DX, HOLE_Z, -BACK_T - 1, 0.01, M2_CLEAR / 2)
        h -= cyl_y(sx * HOLE_DX, HOLE_Z, 0, Y_FACE_OUT + 1, M2_PILOT / 2)   # 通孔：M2×8 头贴背板时尖端正好和前面板外表面齐平
    return h


# ---------------- 前座 ----------------
def build_front():
    z_top = -LIDAR_STACK_T
    z_pad = z_top - FRONT_PAD_T
    hood = build_hood().translate([0, FRONT_HOOD_Y + BACK_T, z_top - HZ_TOP])
    m = hood
    for sx in (-1, 1):
        hx = sx * FRONT_HOLE_X / 2
        ax, ay = sx * (HX - 2.5), FRONT_HOOD_Y + 2.5
        m += m3.Manifold.batch_hull([cyl_z(hx - FRONT_SLOT_HALF, 0, z_pad, z_top, 3.5),
                                     cyl_z(hx + FRONT_SLOT_HALF, 0, z_pad, z_top, 3.5)])
        m += m3.Manifold.batch_hull([cyl_z(hx, 0, z_pad, z_top, 2.5), cyl_z(ax, ay, z_pad, z_top, 2.5)])
        # 臂下加强筋（离开螺丝头 4 mm 起）
        d = math.hypot(ax - hx, ay)
        sx0, sy0 = hx + (ax - hx) * 4.0 / d, ay * 4.0 / d
        m += m3.Manifold.batch_hull([cyl_z(sx0, sy0, z_pad - 4.0, z_top, 0.8),
                                     cyl_z(ax, ay, z_pad - 4.0, z_top, 0.8)])
        m -= m3.Manifold.batch_hull([cyl_z(hx - FRONT_SLOT_HALF, 0, z_pad - 1, z_top + 1, M2_CLEAR / 2),
                                     cyl_z(hx + FRONT_SLOT_HALF, 0, z_pad - 1, z_top + 1, M2_CLEAR / 2)])
    return m


# ---------------- 后座 ----------------
def build_rear():
    w, y_back, y_front = GPS_FL
    m = box(-w / 2, w / 2, y_back, y_front, 0, REAR_FL_T)       # GPS 法兰垫片
    y_hood_back = y_back - REAR_HOOD_GAP                       # 护罩背面（朝机头那面）
    # 护罩：上下翻转（排针朝下、槽从上面插），再转 180° 朝后
    hood = build_hood().mirror([0, 0, 1]).rotate([0, 0, 180])
    hood = hood.translate([0, y_hood_back - BACK_T, HZ_TOP])
    m += hood
    m += box(-HX, HX, y_hood_back - 0.5, y_back + 0.01, 0, REAR_FL_T)   # 连接舌
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
          f"TPU≈{vol * 1.21:.1f} g / PETG≈{vol * 1.27:.1f} g（100% 填充）")


if __name__ == "__main__":
    export(build_front(), "tof_front", flip=True)   # 打印时圆垫和臂的上表面贴床
    export(build_rear(), "tof_rear", flip=False)    # 法兰贴床
    print(f"前座：IC 中心在两孔前方 {FRONT_HOOD_Y + BACK_T + IC_Y:.1f} mm、底板下方 "
          f"{LIDAR_STACK_T + HZ_TOP - IC_Z:.1f} mm；护罩前缘在两孔前方 {FRONT_HOOD_Y + BACK_T + Y_FACE_OUT:.1f} mm")
    print(f"后座：IC 中心在两孔后方 {-(GPS_FL[1] - REAR_HOOD_GAP - BACK_T - IC_Y):.1f} mm、顶板上方 "
          f"{HZ_TOP - IC_Z:.1f} mm")
