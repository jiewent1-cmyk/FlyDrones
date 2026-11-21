"""画 sections.png：中线侧剖面（含板子轮廓和视场边缘）、x=13.7 侧剖面（定位墙 + 螺丝让位）、俯视。运行：python render_sections.py"""
import math, os
import numpy as np, trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import gen_piaicam_mount as g

plt.rcParams["font.family"] = ["PingFang SC", "Heiti SC", "Arial Unicode MS"]
tm = g.to_trimesh(g.build())
ly, lz = g.lens_pose()
dy, dz = g.offset()
fig, ax = plt.subplots(1, 3, figsize=(17, 6))


def draw_section(a, origin, normal, proj, col="#7aa6d8"):
    sec = tm.section(plane_origin=origin, plane_normal=normal)
    if sec is None: return
    for e in sec.entities:
        p = sec.vertices[e.points]
        a.fill(p[:, proj[0]], p[:, proj[1]], color=col, alpha=0.35, edgecolor="#1f4e8c", lw=1)


def body(y, z):
    ry, rz = g.rot(y, z); return dy + ry, dz + rz


def poly(a, pts, **kw):
    p = np.array([body(*q) for q in pts + [pts[0]]]); a.plot(p[:, 0], p[:, 1], **kw)


for a, x in ((ax[0], 0.0), (ax[1], 13.7)):
    draw_section(a, [x, 0, 0], [1, 0, 0], (1, 2))
    a.axhline(0, color="k", lw=2)
    # 板子、背面插座（只在 x=0 剖面画镜头）
    poly(a, [(-g.BT, g.Z_BOT), (0, g.Z_BOT), (0, g.Z_TOP), (-g.BT, g.Z_TOP)], color="g", lw=1.5)
    poly(a, [(-g.BT - g.CONN_D, g.Z_BOT), (-g.BT, g.Z_BOT), (-g.BT, g.Z_BOT + g.CONN_H), (-g.BT - g.CONN_D, g.Z_BOT + g.CONN_H)], color="g", lw=1)
    a.set_xlabel("y (mm)"); a.set_ylabel("z (mm)"); a.set_xlim(-8, 45); a.set_ylim(-4, 40)
poly(ax[0], [(0, -5.35), (g.LENS_LEN - 1.1, -5.35), (g.LENS_LEN - 1.1, 5.35), (0, 5.35)], color="g", lw=1)
poly(ax[0], [(g.LENS_LEN - 1.1, -2.9), (g.LENS_LEN, -2.9), (g.LENS_LEN, 2.9), (g.LENS_LEN - 1.1, 2.9)], color="g", lw=1)
t = math.radians(g.TILT)
for ang, c in ((g.VFOV / 2, "r"), (-g.VFOV / 2, "r"), (0, "g")):
    a = math.radians(ang) - t
    ax[0].plot([ly, ly + 40 * math.cos(a)], [lz, lz + 40 * math.sin(a)], color=c, ls="--", lw=1)
ax[0].plot([0], [0], "kx"); ax[0].text(0.5, -3, "孔位", fontsize=9)
ax[0].set_title(f"中线侧剖面：光轴下倾 {g.TILT:.0f}°，红虚线=垂直视场 ±{g.VFOV / 2:.1f}°，绿=板/插座/镜头")
ax[1].axvspan(-2.4, 2.4, color="orange", alpha=0.15); ax[1].text(-2.3, 36, "螺丝让位", fontsize=9)
ax[1].set_title("x=13.7 侧剖面（槽侧壁；橙=螺丝头/螺丝刀让位）")

for z, col in ((1.2, "#7aa6d8"), (12, "#e0a050")):
    draw_section(ax[2], [0, 0, z], [0, 0, 1], (0, 1), col)
for sx in (-1, 1): ax[2].plot([sx * g.HOLE_X / 2], [0], "k+", ms=12)
for ang in (-g.HFOV / 2, g.HFOV / 2):
    a = math.radians(90 + ang); ax[2].plot([0, 40 * math.cos(a)], [ly, ly + 40 * math.sin(a)], "r--", lw=1)
ax[2].set_title(f"俯视：蓝=法兰（z=1.2），橙=面板（z=12），红虚线=水平视场 ±{g.HFOV / 2:.1f}°")
ax[2].set_xlabel("x (mm)"); ax[2].set_ylabel("y (mm)"); ax[2].set_xlim(-30, 30); ax[2].set_ylim(-8, 45)
for a in ax: a.set_aspect("equal"); a.grid(alpha=0.3)
plt.tight_layout(); plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "sections.png"), dpi=110)

# 立体图（装机姿态），两个角度
import matplotlib.tri as mtri
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
fig = plt.figure(figsize=(13, 6))
for i, (el, az) in enumerate(((25, -60), (20, 130))):
    a = fig.add_subplot(1, 2, i + 1, projection="3d")
    pc = Poly3DCollection(tm.vertices[tm.faces], alpha=1.0)
    n = tm.face_normals; light = np.clip(n @ np.array([0.3, -0.5, 0.8]) / 1.0, 0, 1) * 0.6 + 0.35
    pc.set_facecolor(np.c_[light * 0.45, light * 0.65, light * 0.9, np.ones_like(light)]); pc.set_edgecolor("none")
    a.add_collection3d(pc)
    a.set_xlim(-20, 20); a.set_ylim(-10, 30); a.set_zlim(0, 40); a.set_box_aspect((1, 1, 1))
    a.view_init(el, az); a.set_xlabel("x"); a.set_ylabel("y（机头）"); a.set_zlabel("z")
    a.set_title("前方" if i == 0 else "后方（板子从上面插）")
plt.tight_layout(); plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "render_装机.png"), dpi=100)
