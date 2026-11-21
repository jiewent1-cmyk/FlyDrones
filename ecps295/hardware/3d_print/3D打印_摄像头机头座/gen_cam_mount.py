"""ELP OV7725 120°（ELP-USB30W02M-MHV120，32×32 板，M7 镜头）机头摄像头座。

装在机架顶板机头上部的两个孔上（横向 27 mm，和 Pi 罩的前排脚孔不是同一对孔）。
法兰平贴顶板，2 颗 M2 从上往下拧；法兰前方立一个夹板框，摄像头板从上面插进框里的槽，
镜头座从前面板的方窗伸出去；光轴朝正前、下倾 12°（与孪生 C1 / Gazebo G2 的安装一致）。

坐标：原点在两孔中点，z=0 是顶板上表面（z 向上），+y = 机头，x 向右。
运行：python gen_cam_mount.py
"""
import math
import os
import numpy as np
import manifold3d as m3
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
SEG = 40

# ---- ELP OV7725 板（商品页：32×32 mm；孔位按同系列 32×32 板常见的 28 mm 方孔距）【实测】----
PCB_W, PCB_T = 32.0, 1.6
PCB_HOLE = 28.0                 # 四角孔方孔距；只是辅助固定，板子主要靠槽夹住
LENS_BASE = 14.0                # M7 镜座底面边长（估计），前面板的窗口按它加余量
LENS_Z_OFF = 0.0                # 镜头中心相对板中心的上下偏移（+ = 往排针/上边）
LENS_LEN = 12.0                 # 镜头最前端到板正面的距离（估计）

TILT = 12.0                     # 光轴下倾角
FIT, WALL, FACE_T, BACK_T = 0.3, 1.8, 2.0, 1.6
RIM = 2.6                       # 背面压边宽度（中间敞开，给 4P 插座和背面元件）
WIN = LENS_BASE + 3.0           # 前面板方窗边长
M2_CLEAR, M2_PILOT = 2.3, 1.7

# ---- 机架 ----
HOLE_X = 27.0
SLOT_HALF = 1.5                 # 横向长条孔，两侧各 1.5 mm
FL_T = 2.4
FRAME_BACK_Y = 4.6              # 夹板框最下沿后缘离两孔连线的距离（给螺丝头和螺丝刀让位）
RIB_T = 1.8

PX = PCB_W / 2 + FIT            # 槽半宽
HX = PX + WALL                  # 框外半宽
Y_SLOT0, Y_SLOT1 = -(PCB_T + 2 * FIT), 0.0     # 局部：y=0 是板正面
Y_BACK = Y_SLOT0 - BACK_T
Z_BOT = -PX - WALL


def box(x0, x1, y0, y1, z0, z1):
    return m3.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl_z(x, y, z0, z1, r):
    return m3.Manifold.cylinder(z1 - z0, r, r, SEG).translate([x, y, z0])


def cyl_y(x, z, y0, y1, r):
    return m3.Manifold.cylinder(y1 - y0, r, r, SEG).rotate([-90, 0, 0]).translate([x, y0, z])


# ---------------- 夹板框（局部坐标：原点 = 板正面中心，+y = 光轴，板在 x-z 平面，槽从 +z 开口）----------------
def build_frame():
    f = box(-HX, HX, Y_BACK, FACE_T, Z_BOT, PX)
    f -= box(-PX, PX, Y_SLOT0, Y_SLOT1, -PX, PX + 1)                       # 插板槽
    f -= box(-WIN / 2, WIN / 2, -0.01, FACE_T + 1, LENS_Z_OFF - WIN / 2, LENS_Z_OFF + WIN / 2)   # 镜头窗
    f -= box(-PX + RIM, PX - RIM, Y_BACK - 1, Y_SLOT0 + 0.01, -PX + RIM, PX + 1)                # 背面敞开
    # 前面板上沿开一道 U 口，镜头座可以从上面顺着滑进窗口（板从上往下插时镜头不卡）
    f -= box(-WIN / 2, WIN / 2, -0.01, FACE_T + 1, LENS_Z_OFF, PX + 1)
    # 四角 M2：背面压边过孔、前面板自攻盲孔
    h = PCB_HOLE / 2
    for sx in (-1, 1):
        for sz in (-1, 1):
            f -= cyl_y(sx * h, sz * h, Y_BACK - 1, Y_SLOT0 + 0.01, M2_CLEAR / 2)
            f -= cyl_y(sx * h, sz * h, 0, FACE_T - 0.6, M2_PILOT / 2)
    return f


def place(m):
    """局部 → 机体：先下倾 TILT（光轴 +y 转向 -z），再平移到法兰上。"""
    m = m.rotate([-TILT, 0, 0])
    return m


def frame_offset():
    # 下倾后，框底面后棱是最低点之一；让框底面最低点落在法兰顶面，框最下沿后缘在 FRAME_BACK_Y
    t = math.radians(TILT)
    # 局部底面两条棱 (y=Y_BACK, z=Z_BOT) 与 (y=FACE_T, z=Z_BOT) 旋转后的 (y, z)
    def rot(y, z):
        return y * math.cos(t) + z * math.sin(t), -y * math.sin(t) + z * math.cos(t)
    yb, zb = rot(Y_BACK, Z_BOT)
    yf, zf = rot(FACE_T, Z_BOT)
    dz = FL_T - min(zb, zf) - 0.6        # 嵌进法兰 0.6 mm，保证连成一体
    dy = FRAME_BACK_Y - yb
    return dy, dz


def build():
    dy, dz = frame_offset()
    frame = place(build_frame()).translate([0, dy, dz])
    # 法兰：两颗螺丝的圆垫 + 一块连到框底的底板
    fl_back = -4.0
    fl_front = FRAME_BACK_Y + 10.0
    m = box(-HX - RIB_T, HX + RIB_T, fl_back, fl_front, 0, FL_T)
    for sx in (-1, 1):
        hx = sx * HOLE_X / 2
        m += m3.Manifold.batch_hull([cyl_z(hx - SLOT_HALF, 0, 0, FL_T, 3.6),
                                     cyl_z(hx + SLOT_HALF, 0, 0, FL_T, 3.6)])
    m += frame
    # 框底面下倾后后缘略抬起，补一块垫层把框底和法兰连实（高度低于槽底）
    m += box(-HX, HX, FRAME_BACK_Y - 0.3, FRAME_BACK_Y + 6.5, FL_T - 0.1, FL_T + 0.9)
    # 两片外侧三角筋：贴在框左右外侧，从法兰后沿斜到框背面约 2/3 高处
    t = math.radians(TILT)
    z_top_rib = dz + (Z_BOT + (2 * PX + WALL) * 0.7) * math.cos(t) - Y_BACK * math.sin(t)
    y_top_rib = dy + Y_BACK * math.cos(t) + (Z_BOT + (2 * PX + WALL) * 0.7) * math.sin(t)
    for sx in (-1, 1):
        x0, x1 = (HX - 0.01, HX + RIB_T) if sx > 0 else (-HX - RIB_T, -HX + 0.01)
        a = box(x0, x1, fl_back, fl_back + 0.5, 0, FL_T)
        b = box(x0, x1, y_top_rib + 0.3, y_top_rib + 2.5, z_top_rib - 0.5, z_top_rib)
        c = box(x0, x1, FRAME_BACK_Y + 0.5, fl_front, 0, FL_T)
        m += m3.Manifold.batch_hull([a, b, c])
    # 螺丝孔（长条），再把圆垫上方清空，给螺丝头和螺丝刀
    for sx in (-1, 1):
        hx = sx * HOLE_X / 2
        m -= m3.Manifold.batch_hull([cyl_z(hx - SLOT_HALF, 0, -1, FL_T + 1, M2_CLEAR / 2),
                                     cyl_z(hx + SLOT_HALF, 0, -1, FL_T + 1, M2_CLEAR / 2)])
    return m


def lens_pose():
    """镜头前端中心在机体坐标中的位置。"""
    dy, dz = frame_offset()
    t = math.radians(TILT)
    y_l, z_l = LENS_LEN, LENS_Z_OFF
    return (dy + y_l * math.cos(t) + z_l * math.sin(t), dz - y_l * math.sin(t) + z_l * math.cos(t))


def to_trimesh(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts))


if __name__ == "__main__":
    m = build()
    tm = to_trimesh(m)
    tm.export(os.path.join(HERE, "cam_mount_装机姿态.stl"))
    pr = tm.copy()
    pr.apply_translation([0, 0, -pr.bounds[0][2]])
    pr.export(os.path.join(HERE, "cam_mount_打印姿态.stl"))     # 法兰贴床，和装机姿态相同
    vol = tm.volume / 1000
    ly, lz = lens_pose()
    print(f"watertight={tm.is_watertight} bbox={np.round(tm.extents, 1)} mm  bounds y {tm.bounds[0][1]:.1f}..{tm.bounds[1][1]:.1f}"
          f" z {tm.bounds[0][2]:.1f}..{tm.bounds[1][2]:.1f}  vol={vol:.2f} cm3  PETG≈{vol * 1.27:.1f} g（100%）")
    print(f"镜头前端：两孔前方 {ly:.1f} mm、顶板上方 {lz:.1f} mm")
