"""Raspberry Pi AI Camera（IMX500，课程 BOM 默认摄像头）机头座，插入式（和 ELP 座同一种固定方法）。

底座和 ELP 机头座（../3D打印_摄像头机头座）完全一样：装在机架顶板机头上部的两个孔上，
横向 27 mm，2 颗 M2 从上往下拧，横向长条孔左右各滑 1.5 mm，法兰厚 2.4 mm。
法兰上立一个夹板框，光轴朝正前、下倾 TILT。摄像头板从上往下插进框里的槽：正面朝前，
传感器模组和镜头顺着前面板的 U 口滑进方窗；背面只留左右两条压边，中间敞开给背面插座和元件。
排线插座在板子下沿背面，排线从槽底中间的开口往下出来，再往后弯。插好后可以从背面拧 4 颗
M2 自攻螺丝压住焊盘（可选，板子主要靠槽夹住）。

摄像头尺寸来自官方 product brief（2024-09）第 4 页机械图；PCB 外形和孔位与 Camera Module 3 相同，
所以 Camera Module 3 / v2 也能装（镜头更短，视场不同）。

坐标：原点在两孔中点，z=0 是顶板上表面（z 向上），+y = 机头，x 向右。
运行：python gen_piaicam_mount.py
"""
import math
import os
import numpy as np
import manifold3d as m3
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
SEG = 40

# ---- Pi AI Camera（官方机械图）----
BW, BH, BT = 25.0, 23.862, 1.0          # 板宽、板高、板厚
HOLE_DX, HOLE_DZ = 21.0, 12.5           # 孔距：左右 21（离边 2.0），上下 12.5
HOLE_TOP = 2.0                          # 上排孔离上沿
LENS_FROM_TOP = 14.4                    # 镜头中心离上沿（下排孔 14.5）
MOD_W, MOD_TOP, MOD_BOT = 12.8, 4.36, 20.6   # 正面方形传感器模组：宽 12.8，离上沿 4.36..约 20.6
CONN_W, CONN_H, CONN_D = 19.61, 5.71, 2.75   # 背面排线插座：宽、离下沿高度、厚
LENS_LEN = 8.15                         # 镜头前端到板正面（总厚 11.9 − 插座 2.75 − 板 1.0）
HFOV, VFOV = 66.3, 52.3

TILT = 12.0                             # 光轴下倾角（和 ELP 座一致）
FIT = 0.3                               # 板边到槽侧壁
SLOT_FIT = 0.25                         # 板厚方向每边间隙（板厚 1.0 → 槽宽 1.5）
WALL, FACE_T, BACK_T = 1.8, 2.0, 1.6
RIM = 2.4                               # 背面压边宽度：板边到插座边只有 2.7 mm
WIN_M = 0.6                             # 方窗 / U 口比模组每边大
M2_CLEAR, M2_PILOT = 2.3, 1.7
HEAD_R = 2.1                            # 背面压边上给 M2 螺丝头让的孔（头直接压在焊盘上）
FPC_GAP = 5.0                           # 板下沿背面离法兰顶面的最小高度，给排线弯折
SCREW_CLEAR_Y = 3.0                     # 板子和插座在俯视里离两孔连线的最小距离（螺丝刀从上面下）
LEG_CLEAR_Y = 2.6                       # 框两侧下端在法兰处离两孔连线的最小距离（螺丝头 + 让位）

# ---- 机架（和 ELP 座相同）----
HOLE_X = 27.0
SLOT_HALF = 1.5
FL_T = 2.4
PAD_R = 3.6

# 局部坐标：原点 = 板正面上镜头中心，+y = 光轴，z = 板子朝上方向
Z_TOP = LENS_FROM_TOP
Z_BOT = LENS_FROM_TOP - BH
PX = BW / 2 + FIT                       # 槽半宽
HX = PX + WALL                          # 框外半宽
Y_SLOT0, Y_SLOT1 = -(BT + 2 * SLOT_FIT), 0.0   # 槽：板正面贴 y=0 一侧
Y_BACK = Y_SLOT0 - BACK_T
X_OPEN = PX - RIM                       # 背面敞开 / 槽底排线开口的半宽
HOLES = [(sx * HOLE_DX / 2, Z_TOP - HOLE_TOP - k * HOLE_DZ) for sx in (-1, 1) for k in (0, 1)]
assert X_OPEN >= CONN_W / 2 + 0.5, "背面压边会碰到排线插座"


def box(x0, x1, y0, y1, z0, z1):
    return m3.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl_z(x, y, z0, z1, r):
    return m3.Manifold.cylinder(z1 - z0, r, r, SEG).translate([x, y, z0])


def cyl_y(x, z, y0, y1, r):
    return m3.Manifold.cylinder(y1 - y0, r, r, SEG).rotate([-90, 0, 0]).translate([x, y0, z])


def rot(y, z):
    """局部 → 机体（未平移）：绕 x 转，光轴 +y 转向下 TILT。"""
    t = math.radians(TILT)
    return y * math.cos(t) + z * math.sin(t), -y * math.sin(t) + z * math.cos(t)


def offset():
    """平移量 (dy, dz)：板下沿背面离法兰 FPC_GAP；板、插座和框两侧下端俯视都不挡两孔的螺丝。"""
    t = math.radians(TILT)
    _, zb = rot(-BT, Z_BOT)
    dz = FL_T + FPC_GAP - zb
    yb, _ = rot(-BT - CONN_D, Z_BOT)            # 插座背面下棱
    zl = (FL_T - 0.6 - dz + Y_BACK * math.sin(t)) / math.cos(t)    # 框背面在法兰处的局部 z
    yl, _ = rot(Y_BACK, zl)
    dy = max(SCREW_CLEAR_Y - yb, LEG_CLEAR_Y - yl)
    return dy, dz


def build_upright():
    """局部坐标的夹板框，往下伸得很长，装配后再按法兰切平。"""
    zlo = Z_BOT - 40
    f = box(-HX, HX, Y_BACK, FACE_T, zlo, Z_TOP)
    f -= box(-PX, PX, Y_SLOT0, Y_SLOT1, Z_BOT - FIT, Z_TOP + 1)              # 插板槽，从上面开口
    f -= box(-X_OPEN, X_OPEN, Y_BACK - 1, Y_SLOT0 + 0.01, zlo - 1, Z_TOP + 1)  # 背面敞开（插座、元件）
    f -= box(-X_OPEN, X_OPEN, Y_BACK - 1, Y_SLOT1 + 0.01, zlo - 1, Z_BOT + 0.5)  # 槽底中间开口，排线往下出
    # 前面板：方窗盖住传感器模组，上方开 U 口让模组和镜头顺着滑下来
    hw = MOD_W / 2 + WIN_M
    f -= box(-hw, hw, -0.01, FACE_T + 1, Z_TOP - MOD_BOT - WIN_M, Z_TOP + 1)
    # 四角 M2：前面板自攻孔，背面压边给螺丝头让位
    for x, z in HOLES:
        f -= cyl_y(x, z, -0.01, FACE_T - 0.6, M2_PILOT / 2)
        f -= cyl_y(x, z, Y_BACK - 1, Y_SLOT0 + 0.01, HEAD_R)
    return f


def build():
    dy, dz = offset()
    up = build_upright().rotate([-TILT, 0, 0]).translate([0, dy, dz])
    up = up ^ box(-50, 50, -50, 80, FL_T - 0.6, 80)     # 下端切平，嵌进法兰 0.6 mm
    # 面板前面最低处的 y（法兰顶面高度）
    yf = dy + FACE_T * math.cos(math.radians(TILT)) + (FL_T - dz + FACE_T * math.sin(math.radians(TILT))) \
        / math.cos(math.radians(TILT)) * math.sin(math.radians(TILT))
    fl_back, fl_front = -4.0, yf + 4.0
    m = box(-HX, HX, fl_back, fl_front, 0, FL_T)
    for sx in (-1, 1):
        hx = sx * HOLE_X / 2
        m += m3.Manifold.batch_hull([cyl_z(hx - SLOT_HALF, 0, 0, FL_T, PAD_R),
                                     cyl_z(hx + SLOT_HALF, 0, 0, FL_T, PAD_R)])
    m += up
    # 面板前根部三角筋（整宽），在视场下沿以下
    t = math.radians(TILT)
    h = 4.0
    m += m3.Manifold.batch_hull([box(-HX, HX, yf - 1.0, fl_front, FL_T - 0.1, FL_T),
                                 box(-HX, HX, yf - 1.0, yf + h * math.tan(t) - 0.2, FL_T + h - 0.3, FL_T + h)])
    # 螺丝孔（长条）+ 上方让位（螺丝头和螺丝刀）
    for sx in (-1, 1):
        hx = sx * HOLE_X / 2
        m -= m3.Manifold.batch_hull([cyl_z(hx - SLOT_HALF, 0, -1, FL_T + 1, M2_CLEAR / 2),
                                     cyl_z(hx + SLOT_HALF, 0, -1, FL_T + 1, M2_CLEAR / 2)])
        m -= m3.Manifold.batch_hull([cyl_z(hx - SLOT_HALF, 0, FL_T, 60, 2.4),
                                     cyl_z(hx + SLOT_HALF, 0, FL_T, 60, 2.4)])
    return m


def lens_pose():
    """镜头前端中心在机体坐标中的位置 (y, z)。"""
    dy, dz = offset()
    y, z = rot(LENS_LEN, 0.0)
    return dy + y, dz + z


def board_corners():
    """板子（含背面插座）的几个关键点，机体坐标 (y, z)，用于检查。"""
    dy, dz = offset()
    pts = {"板上沿正面": (0, Z_TOP), "板下沿背面": (-BT, Z_BOT), "插座背面下棱": (-BT - CONN_D, Z_BOT),
           "框背面上沿": (Y_BACK, Z_TOP)}
    return {k: (dy + rot(*v)[0], dz + rot(*v)[1]) for k, v in pts.items()}


def to_trimesh(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts))


if __name__ == "__main__":
    tm = to_trimesh(build())
    tm.export(os.path.join(HERE, "piaicam_mount_装机姿态.stl"))
    pr = tm.copy()
    pr.apply_translation([0, 0, -pr.bounds[0][2]])
    pr.export(os.path.join(HERE, "piaicam_mount_打印姿态.stl"))
    vol = tm.volume / 1000
    ly, lz = lens_pose()
    print(f"watertight={tm.is_watertight} bbox={np.round(tm.extents, 1)} mm  bounds y {tm.bounds[0][1]:.1f}..{tm.bounds[1][1]:.1f}"
          f" z {tm.bounds[0][2]:.1f}..{tm.bounds[1][2]:.1f}  vol={vol:.2f} cm3  PLA≈{vol * 1.24:.1f} g / PETG≈{vol * 1.27:.1f} g（100%）")
    print(f"镜头前端：两孔前方 {ly:.1f} mm、顶板上方 {lz:.1f} mm")
    for k, (y, z) in board_corners().items():
        print(f"{k}: y={y:.1f} z={z:.1f}")
