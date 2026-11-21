"""Orange Pi Zero 3W 罩子（罩在机架顶板上，顶面兼作电池平台）。

两个零件：
  pibox_shell.stl  罩体：上下都敞开的四面墙，底部 4 只内脚用 M2 螺丝固定到机架顶板
  pibox_lid.stl    盖板：上表面防滑横纹放电池；Pi 倒挂在盖板下面（背面朝上，排针和散热器朝下）

装配：先把罩体拧到机架上（这时顶部敞开，螺丝刀能伸进去）→ 把 Pi 拧到盖板底面的 4 根柱上
→ 盖板扣到罩体上，四角 M2 螺丝锁紧。

坐标：z=0 是机架顶板上表面；+y = 机头；x = 左右。俯视时 Pi 的接口边（USB-C/HDMI）朝 +x，
40 针排针朝 -x，排针 1–10 号针（5V/GND/UART）在 -y 端（机尾）。

运行：python gen_pibox.py   标【实测】的参数请装前用卡尺核对。
"""
import math
import os
import numpy as np
import manifold3d as m3
import trimesh

SEG = 48

# ---- Orange Pi Zero 3W（排针朝下安装）----
PI_L, PI_W, PI_T = 65.0, 32.0, 1.2
PI_HOLE_X, PI_HOLE_Y = 23.0, 58.0   # 【实测】按 Pi Zero 标准孔位 58×23
PI_FACE_Z = 21.0     # Pi 元件面（朝下那面）离顶板的高度：排针 2.5 + 杜邦胶壳 14 = 16.5，
                     # 杜邦线底端离板面 4.5 mm，高过机架脚的螺丝头；散热器 11.8 mm 也装得下
LID_STANDOFF = 3.0   # Pi 背面到盖板底面的距离
# 接口中心距 Pi 机头端（+y 端）的距离，由课程文档照片估算【实测】
USBC_PWR_Y, USBC_OTG_Y, HDMI_Y = 10.4, 23.3, 52.5

# ---- 机架顶板孔位（实测：前排横向 27、后排横向 33、前后 55）----
FRAME_FRONT_X, FRAME_REAR_X, FRAME_Y = 27.0, 33.0, 55.0
FRAME_CY = 0.0       # 孔位中心相对罩子中心的前后偏移
M2_CLEAR, M2_PILOT = 2.3, 1.7
FOOT_T, FOOT_R = 2.4, 3.2

# ---- 电池绑带 ----
STRAP_W, STRAP_H = 22.0, 2.6   # 罩体两侧墙底部的绑带缺口（适配 ≤20 mm 宽绑带）

# ---- 罩体 ----
WALL = 1.2
IN_X = 44.0          # Pi 两侧各留 6 mm：放盖板锁紧柱
IN_Y = PI_L + 4.0
OX, OY = IN_X / 2 + WALL, IN_Y / 2 + WALL
Z_PCB_BACK = PI_FACE_Z + PI_T
Z_TOP = Z_PCB_BACK + LID_STANDOFF   # 罩体上沿 = 盖板底面
POST_XY = (IN_X / 2 - 2.5, IN_Y / 2 - 2.5)

# ---- 盖板 ----
LID_T = 2.0
LIP_H, LIP_W, LIP_FIT = 1.6, 1.2, 0.25
RIB_W, RIB_H, RIB_PITCH = 1.2, 0.8, 3.5


def box(x0, x1, y0, y1, z0, z1):
    return m3.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cbox(cx, cy, w, l, z0, z1):
    return box(cx - w / 2, cx + w / 2, cy - l / 2, cy + l / 2, z0, z1)


def cyl(x, y, z0, z1, r):
    return m3.Manifold.cylinder(z1 - z0, r, r, SEG).translate([x, y, z0])


def rrect(w, l, r, z0, z1):
    return m3.Manifold.batch_hull([cyl(sx * (w / 2 - r), sy * (l / 2 - r), z0, z1, r)
                                   for sx in (-1, 1) for sy in (-1, 1)])


def corners(px, py):
    return [(sx * px, sy * py) for sx in (-1, 1) for sy in (-1, 1)]


def pi_y(d):
    """距 Pi 机头端 d mm → y 坐标。"""
    return PI_L / 2 - d


def frame_holes():
    yf, yr = FRAME_CY + FRAME_Y / 2, FRAME_CY - FRAME_Y / 2
    return [(-FRAME_FRONT_X / 2, yf), (FRAME_FRONT_X / 2, yf),
            (-FRAME_REAR_X / 2, yr), (FRAME_REAR_X / 2, yr)]


def build_shell():
    s = rrect(2 * OX, 2 * OY, 2.0, 0, Z_TOP)
    s -= rrect(IN_X, IN_Y, 0.6, -1, Z_TOP + 1)

    # 机架固定脚：圆垫 + 连到最近侧墙的筋，贴在顶板上
    for fx, fy in frame_holes():
        wx = math.copysign(IN_X / 2 + 0.1, fx)
        s += cyl(fx, fy, 0, FOOT_T, FOOT_R)
        s += box(min(fx, wx), max(fx, wx), fy - FOOT_R, fy + FOOT_R, 0, FOOT_T)
    # 盖板锁紧柱（角落，贴墙，全高）
    for x, y in corners(*POST_XY):
        s += cyl(x, y, 0, Z_TOP, 2.5)
        s += cbox(x + math.copysign(1.5, x), y, 3.0, 5.0, 0, Z_TOP)

    # 接口开孔（+x 墙）。排针朝下安装，接口挂在 Pi 板下面
    zc = PI_FACE_Z - 1.7
    usb_c, usb_w = (USBC_PWR_Y + USBC_OTG_Y) / 2, USBC_OTG_Y - USBC_PWR_Y + 12.0
    for d, w, h in ((usb_c, usb_w, 7.5), (HDMI_Y, 15.0, 8.5)):
        s -= cbox(OX - WALL / 2, pi_y(d), WALL + 2, w, zc - h / 2, min(zc + h / 2, Z_TOP - 1.0))
    # 杜邦线出口（-x 墙底边、靠机尾的排针 1–10 号针），线顺着顶板引向飞控
    s -= box(-OX - 1, -IN_X / 2 + 0.5, -IN_Y / 2 + 6.0, -STRAP_W / 2 + 0.01, -1, 7.0)   # 与绑带缺口连通
    # 绑带缺口（两侧墙底边，绑带贴着顶板从 Pi 下方横穿）
    s -= cbox(0, 0, 2 * OX + 2, STRAP_W, -1, STRAP_H)
    # 散热窗：前后端墙 + -x 墙前半段，给散热风扇进出风
    for sy in (-1, 1):
        for x in (-8.0, 0.0, 8.0):
            s -= cbox(x, sy * (OY - WALL / 2), 3.0, WALL + 2, 6.0, Z_TOP - 3.0)
    for y in (6.0, 12.0, 18.0, 24.0):
        s -= cbox(-OX + WALL / 2, y, WALL + 2, 3.0, 6.0, Z_TOP - 3.0)

    # 螺丝孔
    for fx, fy in frame_holes():
        s -= cyl(fx, fy, -1, FOOT_T + 1, M2_CLEAR / 2)
    for x, y in corners(*POST_XY):
        s -= cyl(x, y, Z_TOP - 8, Z_TOP + 1, M2_PILOT / 2)
    return s


def build_lid():
    """装机姿态：z=0 是盖板最低点（Pi 柱底端）。导出 STL 时翻面，防滑纹贴床打印，
    定位唇和 Pi 柱朝上，免支撑。"""
    lid = rrect(2 * OX, 2 * OY, 2.0, 0, LID_T)
    # 定位唇（插进罩体内壁）
    lip_o = rrect(IN_X - 2 * LIP_FIT, IN_Y - 2 * LIP_FIT, 0.6, -LIP_H, 0.01)
    lip_i = rrect(IN_X - 2 * (LIP_FIT + LIP_W), IN_Y - 2 * (LIP_FIT + LIP_W), 0.6, -LIP_H - 1, 0.5)
    lid += lip_o - lip_i
    for x, y in corners(*POST_XY):
        lid -= cyl(x, y, -LIP_H - 1, 0, 3.0)
        lid -= cbox(x + math.copysign(1.5, x), y, 3.6, 5.6, -LIP_H - 1, 0)
    # 底面减重凹槽，顶面保留 0.8 mm 实面托住电池（避开 Pi 柱）
    for sx in (-1, 1):
        for sy in (-1, 1):
            lid -= rrect(14.0, 20.0, 3.0, -LIP_H - 1, LID_T - 0.8).translate([sx * 10.0, sy * 13.5, 0])
    # Pi 安装柱（从盖板底面往下 LID_STANDOFF），Pi 用 M2 自攻螺丝从下往上拧
    for x, y in corners(PI_HOLE_X / 2, PI_HOLE_Y / 2):
        lid += cyl(x, y, -LID_STANDOFF, 0.01, 2.6)
    # 防滑横纹（垂直于机头方向），避开角上的沉头孔
    span_x = 2 * OX - 6.0
    y_lim = POST_XY[1] - 4.0
    n = int(2 * y_lim // RIB_PITCH)
    for i in range(n + 1):
        y = -n * RIB_PITCH / 2 + i * RIB_PITCH
        lid += cbox(0, y, span_x, RIB_W, LID_T - 0.01, LID_T + RIB_H)
    # 孔：Pi 柱自攻孔（盲孔，不穿透顶面）、锁紧螺丝过孔 + 沉头
    for x, y in corners(PI_HOLE_X / 2, PI_HOLE_Y / 2):
        lid -= cyl(x, y, -LID_STANDOFF - 1, LID_T - 0.8, M2_PILOT / 2)
    for x, y in corners(*POST_XY):
        lid -= cyl(x, y, -LIP_H - 1, LID_T + 2, M2_CLEAR / 2)
        lid -= cyl(x, y, LID_T - 1.4, LID_T + 2, 2.2)
    return lid.translate([0, 0, max(LIP_H, LID_STANDOFF)])


def to_trimesh(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts))


if __name__ == "__main__":
    out = os.path.dirname(os.path.abspath(__file__))
    for name, man in (("pibox_shell", build_shell()), ("pibox_lid", build_lid().rotate([180, 0, 0]))):
        tm = to_trimesh(man)
        tm.apply_translation([0, 0, -tm.bounds[0][2]])
        tm.export(os.path.join(out, name + ".stl"))
        vol = tm.volume / 1000
        print(f"{name}: watertight={tm.is_watertight} bbox={np.round(tm.extents, 1)} mm "
              f"vol={vol:.2f} cm3  PETG≈{vol * 1.27:.1f} g（100% 填充上限）")
    print(f"罩体外形 {2*OX:.1f} x {2*OY:.1f} x {Z_TOP:.1f} mm；加盖板后总高 {Z_TOP + LID_T + RIB_H:.1f} mm")
