"""MOD-L 4" 臂电机下方起落脚 v2（×4，TPU 95A）：上粗下尖、外倾的锥形腿。

装在每个电机正下方，和电机共用 2 颗 M2 螺丝（臂上只有 2 个孔，孔距 9 mm）：
螺丝从下往上依次穿过 脚的顶座 → 碳臂 → 拧进电机底座。
顶座是 Ø19 圆盘，螺丝头沉在两个从下面开口的沉孔里；
腿从顶座下面偏到两孔连线的一侧长出来，根部粗（椭圆截面），往下收成圆头尖脚，并向外倾斜。
腿偏在一侧，两个沉孔正下方是空的，螺丝刀竖直伸上去就能拧，拆装不用拆脚。

坐标：原点在两孔中点、臂下表面（z=0），脚往 -z 伸；两孔沿 x 轴，腿往 +y 倾斜。
运行：python gen_landing_foot.py
"""
import math
import os
import numpy as np
import manifold3d as m3
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
SEG = 96

# ---- 机架 / 电机 ----
HOLE_PITCH = 9.0                # 两孔中心距（MOD-L 电机座 9 mm，XING2 1404 为 9×9 M2）
M2_CLEAR = 2.4                  # TPU 孔收缩，比 PETG 件的 2.3 大一点
CENTER_HOLE = 5.0               # 顶面中心让位盲孔：电机轴卡簧 / 臂中心孔
CENTER_DEPTH = 2.5
HEIGHT = 50.0                   # 臂下表面到地面

# ---- 顶座 ----
R_HUB = 9.5                     # 顶座半径（Ø19，不超过电机 Ø19.9 的投影）
HUB_T = 5.0                     # 顶座总厚：比 v1 的 2 mm 法兰厚，腿受侧撞时不会把顶座掀起来
FL_T = 2.0                      # 沉孔底（螺丝头座）以上的厚度：臂 3 + 2 = 5 mm，仍用 M2×7（吃牙 2 mm）
CB_R = 2.6                      # 沉孔半径（Ø5.2：M2 头 Ø3.8 + 螺丝刀余量）
HUB_RND = 1.2                   # 顶座上下外缘圆角

# ---- 腿 ----
TILT = 12.0                     # 腿轴线外倾角（相对竖直）
ROOT_Y = 5.9                    # 腿根中心离两孔连线的距离（在顶座底面）
ROOT_AX, ROOT_AY = 5.5, 4.0     # 腿根椭圆半轴（x 沿孔连线，y 沿倾斜方向）：11 × 8 mm
FLARE_AX, FLARE_AY = 6.0, 3.6   # 和顶座相接处外扩成的椭圆（代替圆角，侧撞时根部不应力集中；不超出顶座）
FLARE_H = 3.0                   # 外扩过渡高度
R_TIP = 2.0                     # 脚尖圆头半径（Ø4，"尖"但不扎手、不戳坏地面）
SCREW_GAP = 0.4                 # 腿任何截面到沉孔竖直通道的最小间隙


def arc(cx, cy, r, a0, a1, n=12):
    return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
             cy + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]


def ellipse(cy, ax, ay, z, n=64):
    return [(ax * math.cos(2 * math.pi * i / n), cy + ay * math.sin(2 * math.pi * i / n), z) for i in range(n)]


def tip_center():
    z = -HEIGHT + R_TIP
    return np.array([0.0, ROOT_Y + (-HUB_T - z) * math.tan(math.radians(TILT)), z])


def hub():
    """顶座回转截面（r, z）：上下外缘圆角，逆时针。"""
    pts = [(0.0, -HUB_T)]
    pts += arc(R_HUB - HUB_RND, -HUB_T + HUB_RND, HUB_RND, -90, 0)
    pts += arc(R_HUB - HUB_RND, -HUB_RND, HUB_RND, 0, 90)
    pts += [(0.0, 0.0)]
    body = m3.Manifold.revolve(m3.CrossSection([pts]), SEG)
    for sx in (-1, 1):
        x = sx * HOLE_PITCH / 2
        body -= m3.Manifold.cylinder(HUB_T + 2, M2_CLEAR / 2, M2_CLEAR / 2, 40).translate([x, 0, -HUB_T - 1])
        body -= m3.Manifold.cylinder(HUB_T - FL_T + 1, CB_R, CB_R, 48).translate([x, 0, -HUB_T - 1])
    body -= m3.Manifold.cylinder(CENTER_DEPTH + 1, CENTER_HOLE / 2, CENTER_HOLE / 2, 40).translate(
        [0, 0, -CENTER_DEPTH])
    return body


def leg():
    """腿 = 凸包（根部外扩椭圆 + 根部椭圆 + 脚尖球），天然上粗下尖。"""
    z0 = -HUB_T
    root = ellipse(ROOT_Y + FLARE_H * math.tan(math.radians(TILT)), ROOT_AX, ROOT_AY, z0 - FLARE_H)
    flare = ellipse(ROOT_Y, FLARE_AX, FLARE_AY, z0 + 0.5)
    tip = np.asarray(m3.Manifold.sphere(R_TIP, 48).to_mesh().vert_properties)[:, :3] + tip_center()
    upper = m3.Manifold.hull_points(flare + root)
    lower = m3.Manifold.hull_points(root + [tuple(p) for p in tip])
    return upper + lower


def build():
    return hub() + leg()


def screw_clearance(body):
    """腿在沉孔竖直通道（半径 CB_R，从顶座底面一直到地面）里的最小间隙。"""
    tm = to_trimesh(leg())
    worst = 1e9
    for z in np.linspace(-HUB_T - 0.01, -HEIGHT + 0.2, 60):
        s = tm.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
        if s is None:
            continue
        v = s.vertices[:, :2]
        d = np.hypot(np.abs(v[:, 0]) - HOLE_PITCH / 2, v[:, 1]).min() - CB_R
        worst = min(worst, d)
    return worst


def to_trimesh(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts))


if __name__ == "__main__":
    body = build()
    tm = to_trimesh(body)
    tm.export(os.path.join(HERE, "landing_foot_装机姿态.stl"))
    pr = tm.copy()
    pr.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [1, 0, 0]))   # 翻过来：顶座贴床
    pr.apply_translation([0, 0, -pr.bounds[0][2]])
    pr.export(os.path.join(HERE, "landing_foot_打印姿态.stl"))
    vol = tm.volume / 1000
    print(f"watertight={tm.is_watertight} bbox={np.round(tm.extents, 1)} mm  z {tm.bounds[0][2]:.1f}..{tm.bounds[1][2]:.1f}"
          f"  vol={vol:.2f} cm3  TPU≈{vol * 1.21:.1f} g/只（100%），4 只 {vol * 1.21 * 4:.1f} g")
    tc = tip_center()
    gap = screw_clearance(body)
    print(f"腿根 {2 * ROOT_AX:.1f}×{2 * ROOT_AY:.1f} → 脚尖 Ø{2 * R_TIP:.0f}，外倾 {TILT:.0f}°，"
          f"脚尖着地点偏出孔连线 {tc[1]:.1f} mm（顶座边缘 {R_HUB} mm）；螺丝通道最小间隙 {gap:.2f} mm")
    assert gap >= SCREW_GAP, "腿挡住了螺丝刀通道"
