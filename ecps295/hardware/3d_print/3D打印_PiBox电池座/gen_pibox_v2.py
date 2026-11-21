"""Orange Pi Zero 3W 罩子 v2：抽屉式侧滑盖板（罩在机架顶板上，盖板顶面兼作电池平台）。

和 v1 的区别：
  - 内腔长度 +2 mm（69 → 71），整体高 4 mm（Pi 抬高 3 mm，盖板加厚 1 mm 做滑轨）
  - 盖板不再四角拧螺丝，改成从 -x 侧（排针那侧）横向推进去：盖板两条长边的燕尾舌
    滑进前后端墙顶部的槽里；盖板自带 -x 端板，Pi 倒挂在盖板下面一起推进去
  - 4 只脚孔位改成矩形 33 × 55，4 只脚做成一样（圆垫 + 连到前后端墙的筋）
  - +x 墙 USB-C 开孔往下一直开到 z=6，插头和线往机身方向弯时有空间

两个零件：
  pibox_v2_shell.stl  罩体：+x 墙和前后端墙，-x 侧和顶部敞开；4 只脚 M2 拧到机架顶板
  pibox_v2_lid.stl    抽屉盖：顶板（防滑横纹）+ -x 端板 + Pi 安装柱

装配：罩体拧到机架上 → Pi 拧到盖板底面 4 根柱上、插好杜邦线 → 盖板从 -x 侧推进去
→ 端板上 2 颗 M2 自攻螺丝横向锁到罩体的角柱里（可选，电池绑带也会压住端板）。

坐标：z=0 是机架顶板上表面；+y = 机头；x = 左右。俯视时 Pi 的接口边（USB-C/HDMI）朝 +x，
40 针排针朝 -x，排针 1–10 号针（5V/GND/UART）在 -y 端（机尾）。

运行：python gen_pibox_v2.py   标【实测】的参数请装前用卡尺核对。
"""
import os
import numpy as np
import manifold3d as m3
import trimesh

SEG = 48

# ---- Orange Pi Zero 3W（排针朝下安装）----
PI_L, PI_W, PI_T = 65.0, 32.0, 1.2
PI_HOLE_X, PI_HOLE_Y = 23.0, 58.0   # 【实测】按 Pi Zero 标准孔位 58×23
PI_FACE_Z = 24.0     # Pi 元件面（朝下那面）离顶板的高度，v1 是 21；杜邦线底端离顶板 7.5 mm
LID_STANDOFF = 3.0   # Pi 背面到盖板底面的距离
# 接口中心距 Pi 机头端（+y 端）的距离，由课程文档照片估算【实测】
USBC_PWR_Y, USBC_OTG_Y, HDMI_Y = 10.4, 23.3, 52.5
USBC_Z0 = 6.0        # USB-C 开孔下沿：往机身方向留空间给插头和线

# ---- 机架顶板孔位（矩形：横向 33、前后 55）【实测】----
FRAME_X, FRAME_Y = 33.0, 55.0
FRAME_CY = 0.0       # 孔位中心相对罩子中心的前后偏移
M2_CLEAR, M2_PILOT = 2.3, 1.7
FOOT_T, FOOT_R = 2.4, 3.2

# ---- 电池绑带 ----
STRAP_W, STRAP_H = 22.0, 2.6   # 罩体 +x 墙和端板底边的绑带缺口（适配 ≤20 mm 宽绑带）

# ---- 罩体 ----
WALL = 1.2
IN_X = 44.0
IN_Y = PI_L + 6.0    # v1 是 +4，加长 2 mm
OX, OY = IN_X / 2 + WALL, IN_Y / 2 + WALL
Z_LID0 = PI_FACE_Z + PI_T + LID_STANDOFF   # 盖板底面

# ---- 滑轨：前后端墙顶部内侧加厚成导轨，开 45° 顶的槽；盖板长边是对应的舌 ----
LID_T = 3.0          # 盖板厚度，顶面和罩体上沿齐平
Z_RIM = Z_LID0 + LID_T
RAIL_W = 1.4         # 导轨加厚量（端墙内侧）
TONGUE_D, TONGUE_T = 1.0, 1.0   # 舌伸出量、舌直边厚度（上面再接 45° 斜面）
FIT, FIT_Z = 0.25, 0.2          # 侧向、竖向间隙
RAIL_IN = IN_Y / 2 - RAIL_W     # 导轨内表面 |y|
LID_HY = RAIL_IN - FIT          # 盖板本体半宽（y）
Z_RAIL0 = Z_LID0 - FIT_Z - 1.0  # 导轨下沿（槽底下留 1 mm），下面再接 45° 斜面

# ---- 锁紧：-x 端两根角柱，端板上 M2 自攻螺丝横向拧进去 ----
BOSS_IN = 33.0       # 角柱内表面 |y|，Pi 板边在 32.5
BOSS_L = 5.0
BOSS_YC = (BOSS_IN + OY) / 2
SCREW_Z = 20.0

RIB_W, RIB_H, RIB_PITCH = 1.2, 0.8, 3.5


def box(x0, x1, y0, y1, z0, z1):
    return m3.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cbox(cx, cy, w, l, z0, z1):
    return box(cx - w / 2, cx + w / 2, cy - l / 2, cy + l / 2, z0, z1)


def ybox(x0, x1, sy, a, b, z0, z1):
    """|y| 在 [a, b] 的一侧（sy=±1）。"""
    y0, y1 = sorted((sy * a, sy * b))
    return box(x0, x1, y0, y1, z0, z1)


def cyl(x, y, z0, z1, r):
    return m3.Manifold.cylinder(z1 - z0, r, r, SEG).translate([x, y, z0])


def cylx(x0, x1, y, z, r):
    return m3.Manifold.cylinder(x1 - x0, r, r, SEG).rotate([0, 90, 0]).translate([x0, y, z])


def rrect(w, l, r, z0, z1):
    return m3.Manifold.batch_hull([cyl(sx * (w / 2 - r), sy * (l / 2 - r), z0, z1, r)
                                   for sx in (-1, 1) for sy in (-1, 1)])


def hull(*parts):
    return m3.Manifold.batch_hull(list(parts))


def rail_profile(x0, x1, sy, y_in, d, z0, z1):
    """|y| 从 y_in 到 y_in+d、z0..z1 的直边，上面接 45° 斜面（外侧高 z1、内侧高 z1+d）。
    舌和槽都用它，两者都能免支撑打印。"""
    e = 0.01
    return (ybox(x0, x1, sy, y_in, y_in + d, z0, z1)
            + hull(ybox(x0, x1, sy, y_in, y_in + d, z1 - e, z1),
                   ybox(x0, x1, sy, y_in, y_in + e, z1 + d - e, z1 + d)))


def pi_y(d):
    """距 Pi 机头端 d mm → y 坐标。"""
    return PI_L / 2 - d


def frame_holes():
    return [(sx * FRAME_X / 2, FRAME_CY + sy * FRAME_Y / 2) for sx in (-1, 1) for sy in (-1, 1)]


def build_shell():
    x_open = -IN_X / 2   # -x 侧敞开，端墙到这里为止，抽屉端板贴上来
    s = rrect(2 * OX, 2 * OY, 2.0, 0, Z_RIM) ^ box(x_open, OX + 1, -OY - 1, OY + 1, -1, Z_RIM + 1)
    s -= box(x_open - 1, IN_X / 2, -IN_Y / 2, IN_Y / 2, -1, Z_RIM + 1)

    for sy in (-1, 1):
        # 导轨（端墙顶部内侧加厚），下沿 45° 斜面免支撑
        s += ybox(x_open, IN_X / 2, sy, RAIL_IN, IN_Y / 2 + 0.1, Z_RAIL0, Z_RIM)
        s += hull(ybox(x_open, IN_X / 2, sy, RAIL_IN, IN_Y / 2 + 0.1, Z_RAIL0, Z_RAIL0 + 0.01),
                  ybox(x_open, IN_X / 2, sy, IN_Y / 2, IN_Y / 2 + 0.1, Z_RAIL0 - RAIL_W, Z_RAIL0 - RAIL_W + 0.01))
        # -x 端角柱（端板螺丝拧进这里），在 Pi 推进去的路径外面
        s += ybox(x_open, x_open + BOSS_L, sy, BOSS_IN, IN_Y / 2 + 0.1, 0, Z_RAIL0)
        # 槽（比舌大 FIT/FIT_Z，往内多切 1 mm 保证切透导轨内表面）
        s -= rail_profile(x_open - 1, IN_X / 2, sy, RAIL_IN - 1.0, TONGUE_D + 1.0,
                          Z_LID0 - FIT_Z, Z_LID0 + TONGUE_T + FIT_Z)
        s -= cylx(x_open - 1, x_open + BOSS_L - 0.5, sy * BOSS_YC, SCREW_Z, M2_PILOT / 2)

    # 机架固定脚：4 只一样，圆垫 + 连到前后端墙的筋
    for fx, fy in frame_holes():
        s += cyl(fx, fy, 0, FOOT_T, FOOT_R)
        s += box(fx - FOOT_R, fx + FOOT_R, min(fy, np.sign(fy) * (IN_Y / 2 + 0.1)),
                 max(fy, np.sign(fy) * (IN_Y / 2 + 0.1)), 0, FOOT_T)

    # 接口开孔（+x 墙）。排针朝下安装，接口挂在 Pi 板下面
    zc = PI_FACE_Z - 1.7
    usb_c, usb_w = (USBC_PWR_Y + USBC_OTG_Y) / 2, USBC_OTG_Y - USBC_PWR_Y + 14.0
    s -= cbox(OX - WALL / 2, pi_y(usb_c), WALL + 2, usb_w, USBC_Z0, Z_LID0 - 0.6)
    s -= cbox(OX - WALL / 2, pi_y(HDMI_Y), WALL + 2, 15.0, zc - 4.25, zc + 4.25)
    # 绑带缺口（+x 墙底边，绑带贴着顶板从 Pi 下方横穿）
    s -= cbox(0, 0, 2 * OX + 2, STRAP_W, -1, STRAP_H)
    # 散热窗：前后端墙
    for sy in (-1, 1):
        for x in (-8.0, 0.0, 8.0):
            s -= cbox(x, sy * (OY - WALL / 2), 3.0, WALL + 2, 6.0, Z_RAIL0 - RAIL_W - 1.0)

    for fx, fy in frame_holes():
        s -= cyl(fx, fy, -1, FOOT_T + 1, M2_CLEAR / 2)
    return s


def build_lid():
    """装机姿态（z=0 是机架顶板）。导出 STL 时翻面，防滑纹贴床打印，端板和 Pi 柱朝上，免支撑。"""
    x_in = -IN_X / 2
    x_end = IN_X / 2 - 0.4   # 推到底时端板贴住罩体，+x 端离 +x 墙留 0.4
    lid = box(x_in - 0.01, x_end, -LID_HY, LID_HY, Z_LID0, Z_RIM)
    for sy in (-1, 1):
        lid += rail_profile(x_in - 0.01, x_end, sy, LID_HY - 0.01, TONGUE_D + 0.01, Z_LID0, Z_LID0 + TONGUE_T)
    # -x 端板：外形和罩体一致（圆角），从顶板一直下到离机架 0.3 mm
    lid += rrect(2 * OX, 2 * OY, 2.0, 0.3, Z_RIM) ^ box(-OX - 1, x_in, -OY - 1, OY + 1, 0, Z_RIM + 1)

    # 底面减重凹槽，顶面保留 0.8 mm 实面托住电池（避开 Pi 柱）
    for sx in (-1, 1):
        for sy in (-1, 1):
            lid -= rrect(14.0, 20.0, 3.0, Z_LID0 - 1, Z_RIM - 0.8).translate([sx * 10.0, sy * 13.5, 0])
    # Pi 安装柱，Pi 用 M2 自攻螺丝从下往上拧
    for sx in (-1, 1):
        for sy in (-1, 1):
            lid += cyl(sx * PI_HOLE_X / 2, sy * PI_HOLE_Y / 2, Z_LID0 - LID_STANDOFF, Z_LID0 + 0.01, 2.6)
    # 防滑横纹（垂直于机头方向），从端板顶一直铺到 +x 端
    y_lim = LID_HY - 2.0
    n = int(2 * y_lim // RIB_PITCH)
    for i in range(n + 1):
        y = -n * RIB_PITCH / 2 + i * RIB_PITCH
        lid += box(-OX + 2.0, x_end - 2.0, y - RIB_W / 2, y + RIB_W / 2, Z_RIM - 0.01, Z_RIM + RIB_H)

    # 端板开孔：绑带缺口 + 杜邦线出口（靠机尾，和绑带缺口连通）、散热窗、锁紧螺丝过孔
    xe = (-OX + x_in) / 2
    s_w = OX - IN_X / 2 + 2
    lid -= cbox(xe, 0, s_w, STRAP_W, -1, STRAP_H)
    lid -= box(-OX - 1, x_in + 0.5, -IN_Y / 2 + 6.0, -STRAP_W / 2 + 0.01, -1, 9.0)
    for y in (6.0, 12.0, 18.0, 24.0):
        lid -= cbox(xe, y, s_w, 3.0, 10.0, Z_LID0 - 3.0)
    for sy in (-1, 1):
        lid -= cylx(-OX - 1, x_in + 1, sy * BOSS_YC, SCREW_Z, M2_CLEAR / 2)
    # Pi 柱自攻孔（盲孔，不穿透顶面）
    for sx in (-1, 1):
        for sy in (-1, 1):
            lid -= cyl(sx * PI_HOLE_X / 2, sy * PI_HOLE_Y / 2, Z_LID0 - LID_STANDOFF - 1, Z_RIM - 0.8, M2_PILOT / 2)
    return lid


def to_trimesh(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts))


if __name__ == "__main__":
    out = os.path.dirname(os.path.abspath(__file__))
    shell, lid = build_shell(), build_lid()
    print(f"装配干涉体积：{(shell ^ lid).volume():.3f} mm³（应为 0）")
    for name, man in (("pibox_v2_shell", shell), ("pibox_v2_lid", lid.rotate([180, 0, 0]))):
        tm = to_trimesh(man)
        tm.apply_translation([0, 0, -tm.bounds[0][2]])
        tm.export(os.path.join(out, name + ".stl"))
        vol = tm.volume / 1000
        print(f"{name}: watertight={tm.is_watertight} bbox={np.round(tm.extents, 1)} mm "
              f"vol={vol:.2f} cm3  PETG≈{vol * 1.27:.1f} g（100% 填充上限）")
    print(f"外形 {2*OX:.1f} x {2*OY:.1f} mm；罩体高 {Z_RIM:.1f} mm，加防滑纹总高 {Z_RIM + RIB_H:.1f} mm")
