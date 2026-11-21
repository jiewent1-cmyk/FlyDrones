"""画 sections.png：中线侧剖面（含视场边缘）、前视、俯视。运行：python render_sections.py"""
import math, os
import numpy as np, trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import gen_cam_mount as g

plt.rcParams["font.family"] = ["PingFang SC", "Heiti SC", "Arial Unicode MS"]
tm = g.to_trimesh(g.build())
ly, lz = g.lens_pose()
fig, ax = plt.subplots(1, 3, figsize=(16, 6))

def draw_section(a, origin, normal, proj):
    sec = tm.section(plane_origin=origin, plane_normal=normal)
    if sec is None: return
    for e in sec.entities:
        p = sec.vertices[e.points]
        a.fill(p[:, proj[0]], p[:, proj[1]], color="#7aa6d8", alpha=0.35, edgecolor="#1f4e8c", lw=1)

# 侧剖面 x=0 和 x=15（框侧墙 + 筋）
draw_section(ax[0], [0, 0, 0], [1, 0, 0], (1, 2))
sec = tm.section(plane_origin=[g.HX + 0.9, 0, 0], plane_normal=[1, 0, 0])
for e in sec.entities:
    p = sec.vertices[e.points]; ax[0].plot(p[:, 1], p[:, 2], color="gray", lw=0.8, ls=":")
ax[0].axhline(0, color="k", lw=2); ax[0].text(-6, -2.5, "顶板上表面", fontsize=9)
ax[0].plot([0], [0], "kx"); ax[0].text(0.5, -4.5, "孔位", fontsize=9)
t = math.radians(g.TILT)
for ang, c in ((45, "r"), (-45, "r"), (0, "g")):
    a = math.radians(ang) - t
    L = 40
    ax[0].plot([ly, ly + L * math.cos(a)], [lz, lz + L * math.sin(a)], color=c, ls="--", lw=1)
ax[0].plot([ly], [lz], "ro"); ax[0].set_title(f"中线侧剖面（+y=机头）：光轴下倾 {g.TILT:.0f}°，红虚线=垂直视场 ±45°")
ax[0].set_xlabel("y (mm)"); ax[0].set_ylabel("z (mm)")
ax[0].set_xlim(-10, 60); ax[0].set_ylim(-20, 50)

# 前视（沿 -y 看）：投影轮廓
for z0 in np.linspace(0.2, tm.bounds[1][2] - 0.2, 1):
    pass
sec = tm.section(plane_origin=[0, g.FRAME_BACK_Y + 3, 0], plane_normal=[0, 1, 0])
for e in sec.entities:
    p = sec.vertices[e.points]; ax[1].fill(p[:, 0], p[:, 2], color="#7aa6d8", alpha=0.35, edgecolor="#1f4e8c")
ax[1].axhline(0, color="k", lw=2)
for sx in (-1, 1): ax[1].plot([sx * g.HOLE_X / 2] * 2, [-3, 0], "k-", lw=3)
ax[1].set_title(f"横剖面 y={g.FRAME_BACK_Y + 3:.1f}（槽 + 背面压边 + 侧筋）"); ax[1].set_xlabel("x (mm)")

# 俯视：z=1.2 法兰切片 + z=20 框切片
for z, col in ((1.2, "#7aa6d8"), (20, "#e0a050")):
    sec = tm.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    for e in sec.entities:
        p = sec.vertices[e.points]; ax[2].fill(p[:, 0], p[:, 1], color=col, alpha=0.4, edgecolor="k", lw=0.6)
for sx in (-1, 1): ax[2].plot([sx * g.HOLE_X / 2], [0], "k+", ms=12)
for ang in (-60, 60):
    a = math.radians(90 + ang); ax[2].plot([0, 40 * math.cos(a)], [ly, ly + 40 * math.sin(a)], "r--", lw=1)
ax[2].set_title("俯视：蓝=法兰（z=1.2），橙=框（z=20），红虚线=水平视场 ±60°"); ax[2].set_xlabel("x (mm)"); ax[2].set_ylabel("y (mm)")
for a in ax: a.set_aspect("equal"); a.grid(alpha=0.3)
ax[1].set_xlim(-30, 30); ax[2].set_xlim(-35, 35); ax[2].set_ylim(-10, 50)
plt.tight_layout(); plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "sections.png"), dpi=110)

# 立体图
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
fig = plt.figure(figsize=(12, 6))
for i, (el, az, ttl) in enumerate(((25, -60, "左前上方看"), (25, 130, "右后上方看"))):
    a = fig.add_subplot(1, 2, i + 1, projection="3d")
    tri = tm.vertices[tm.faces]
    n = tm.face_normals; light = np.clip(n @ np.array([0.3, -0.5, 0.8]) * 0.5 + 0.5, 0.15, 1)
    pc = Poly3DCollection(tri, facecolors=plt.cm.Blues(0.3 + 0.6 * light), edgecolor="none")
    a.add_collection3d(pc)
    a.plot([-20, 20], [-6, -6], [0, 0], "k-"); a.scatter([-13.5, 13.5], [0, 0], [0, 0], c="k", marker="x")
    a.quiver(0, ly, lz, 0, 15 * math.cos(t), -15 * math.sin(t), color="r")
    a.set_xlim(-25, 25); a.set_ylim(-10, 40); a.set_zlim(-5, 45); a.set_box_aspect((50, 50, 50))
    a.view_init(el, az); a.set_title(ttl + "（红箭头=光轴，+y=机头）"); a.set_xlabel("x"); a.set_ylabel("y"); a.set_zlabel("z")
plt.tight_layout(); plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "render_装机.png"), dpi=100)
