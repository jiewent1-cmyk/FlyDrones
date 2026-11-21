"""画 sections.png：侧视竖剖面（含碳臂、电机、螺丝示意）、仰视、立体图。运行：python render_sections.py"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import gen_landing_foot as g

plt.rcParams["font.family"] = ["PingFang SC", "Heiti SC", "Arial Unicode MS"]
tm = g.to_trimesh(g.build())
fig = plt.figure(figsize=(17, 7))
ax0 = fig.add_subplot(1, 4, 1)
ax3 = fig.add_subplot(1, 4, 2)
ax1 = fig.add_subplot(1, 4, 3)
ax2 = fig.add_subplot(1, 4, 4, projection="3d")
ARM_T = 3.0


def draw_section(ax, normal, horiz):
    sec = tm.section(plane_origin=[0, 0, 0], plane_normal=normal)
    for e in sec.entities:
        p = sec.vertices[e.points]
        ax.fill(p[:, horiz], p[:, 2], color="#7aa6d8", alpha=0.45, edgecolor="#1f4e8c", lw=1)
    ax.add_patch(plt.Rectangle((-16, 0), 32, ARM_T, color="#444", alpha=0.8))
    ax.add_patch(plt.Rectangle((-9.95, ARM_T), 19.9, 18.4, fill=False, ec="gray", ls="--"))
    ax.text(-5, ARM_T + 9, "XING2 1404", color="gray", fontsize=9)
    ax.axhline(-g.HEIGHT, color="#8b5a2b", lw=2)
    ax.set_aspect("equal"); ax.set_xlim(-20, 26); ax.set_ylim(-54, 24)
    ax.set_ylabel("z (mm)")


# 侧视：x=0 剖面，看腿的倾斜和锥度
draw_section(ax0, [1, 0, 0], 1)
ax0.annotate("", xy=(23, 0), xytext=(23, -g.HEIGHT), arrowprops=dict(arrowstyle="<->"))
ax0.text(23.6, -g.HEIGHT / 2, f"{g.HEIGHT:.0f} mm", rotation=90, va="center")
ax0.text(-14, -g.HEIGHT - 2.5, "地面", fontsize=9)
ax0.set_title(f"侧视剖面 x=0：外倾 {g.TILT:.0f}°（→ 朝机外）")
ax0.set_xlabel("y (mm)")

# 过两孔剖面 y=0：沉孔 + 螺丝
draw_section(ax3, [0, 1, 0], 0)
for sx in (-1, 1):
    x = sx * g.HOLE_PITCH / 2
    ax3.add_patch(plt.Rectangle((x - 1, -g.FL_T), 2, 7, color="#c33", alpha=0.8))           # M2×7 螺杆
    ax3.add_patch(plt.Rectangle((x - 1.9, -g.FL_T - 2), 3.8, 2, color="#c33", alpha=0.8))    # 头
    ax3.annotate("", xy=(x, -g.FL_T - 2.2), xytext=(x, -20), arrowprops=dict(arrowstyle="->", color="#c33"))
ax3.text(-10, -23, "螺丝刀从下面竖直伸进沉孔", color="#c33", fontsize=8)
ax3.set_title("过两孔剖面 y=0（红 = M2×7）")
ax3.set_xlabel("x (mm)")

# 仰视：顶座底面 + 腿各高度截面 + 脚尖
ax1.add_patch(plt.Circle((0, 0), 9.95, fill=False, ls="--", color="gray"))
ax1.text(-6, -11.5, "电机外径 Ø19.9", color="gray", fontsize=8)
for z, col, lab in ((-g.HUB_T + 0.5, "#7aa6d8", "顶座（沉孔开口）"), (-15, "#4a7", "z=-15"),
                    (-30, "#d80", "z=-30"), (-g.HEIGHT + 0.3, "#c33", "脚尖")):
    s = tm.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    for i, e in enumerate(s.entities):
        p = s.vertices[e.points]
        ax1.plot(p[:, 0], p[:, 1], color=col, lw=1.5, label=lab if i == 0 else None)
ax1.legend(loc="upper left", fontsize=7); ax1.set_aspect("equal")
ax1.set_title(f"截面：孔距 {g.HOLE_PITCH:.0f} mm，腿偏到 +y 侧")
ax1.set_xlim(-16, 16); ax1.set_ylim(-14, 20)

# 立体图
ax2.add_collection3d(Poly3DCollection(tm.vertices[tm.faces], facecolor="#7aa6d8", edgecolor="#1f4e8c",
                                      linewidths=0.03, alpha=0.95))
ax2.set_xlim(-14, 14); ax2.set_ylim(-10, 20); ax2.set_zlim(-50, 0)
ax2.set_box_aspect((28, 30, 50)); ax2.view_init(elev=-20, azim=-35)
ax2.set_title("立体（从斜下方看）")
plt.tight_layout()
plt.savefig(os.path.join(g.HERE, "sections.png"), dpi=130)
