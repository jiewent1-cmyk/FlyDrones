"""画 sections_v3.png：罩体（蓝）+ 抽屉盖（绿）装机姿态的平面切片和剖面。运行：python render_sections_v3.py"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import gen_pibox_v3 as g

plt.rcParams["font.family"] = ["PingFang SC", "Heiti SC", "Arial Unicode MS"]
shell, lid = g.to_trimesh(g.build_shell()), g.to_trimesh(g.build_lid())
F = g.PI_FACE_Z


def sec(a, tm, origin, normal, proj, col, ec):
    s = tm.section(plane_origin=origin, plane_normal=normal)
    if s is None:
        return
    for e in s.entities:
        p = s.vertices[e.points]
        a.fill(p[:, proj[0]], p[:, proj[1]], color=col, alpha=0.4, edgecolor=ec, lw=0.8)


def both(a, origin, normal, proj):
    sec(a, shell, origin, normal, proj, "#7aa6d8", "#1f4e8c")
    sec(a, lid, origin, normal, proj, "#8fcf8f", "#2c7a2c")


def rect(a, u0, u1, v0, v1, **kw):
    a.plot([u0, u1, u1, u0, u0], [v0, v0, v1, v1, v0], **kw)


fig, ax = plt.subplots(2, 3, figsize=(18, 12))
a = ax[0, 0]; both(a, [0, 0, 1.2], [0, 0, 1], (0, 1))
for fx, fy in g.frame_holes():
    a.plot(fx, fy, "kx")
rect(a, -16, 16, -32.5, 32.5, color="g", ls=":", lw=1)
a.set_title("z=1.2 俯视：4 只脚（前 27 / 后 33 × 55）+ 底框 + 角筋，-x 绿色是抽屉端板")
a = ax[0, 1]; both(a, [0, 0, g.SCREW_ZS[1]], [0, 0, 1], (0, 1))
rect(a, -16, 16, -32.5, 32.5, color="g", ls=":", lw=1)
a.set_title(f"z={g.SCREW_ZS[1]:.0f} 俯视：-x 角柱 + 锁紧螺丝孔，+x 墙开孔（点线=Pi）")
a = ax[0, 2]; both(a, [0, 0, g.Z_LID0 + 0.5], [0, 0, 1], (0, 1))
a.annotate("", xy=(15, -40), xytext=(-25, -40), arrowprops=dict(arrowstyle="->", lw=2))
a.text(-24, -44, "盖板从 -x 推入", fontsize=10)
a.set_title(f"z={g.Z_LID0 + 0.5:.1f} 俯视：盖板舌在端墙槽里")
for a in ax[0]:
    a.set_xlim(-30, 30); a.set_ylim(-46, 42); a.set_xlabel("x"); a.set_ylabel("y")

a = ax[1, 0]; both(a, [0, 0, 0], [1, 0, 0], (1, 2))
a.set_title("x=0 剖面（前后方向）：两侧燕尾舌/槽"); a.set_xlim(-40, 40); a.set_ylim(-2, 36)
a2 = a.inset_axes([0.62, 0.05, 0.36, 0.45])
both(a2, [0, 0, 0], [1, 0, 0], (1, 2)); a2.set_xlim(32, 38); a2.set_ylim(24, 33); a2.set_aspect("equal")
a2.set_xticks([]); a2.set_yticks([])
a = ax[1, 1]; both(a, [0, g.pi_y((g.USBC_PWR_Y + g.USBC_OTG_Y) / 2), 0], [0, 1, 0], (0, 2))
rect(a, -16, 16, F, F + g.PI_T, color="g", lw=1.5)
rect(a, -16, 16, F - 11.8, F, color="k", ls=":", lw=1)
rect(a, -14.5, -10, F - 16.5, F, color="orange", lw=1.5)
rect(a, 17.5, 33, F - 1.7 - 3.5, F - 1.7 + 3.5, color="m", lw=1.5)
a.text(18, F - 8, "USB-C 插头\n下面开到 z=6", color="m", fontsize=9)
a.set_title("USB-C 中心剖面（y 向）：橙=杜邦线，点线=散热器"); a.set_xlim(-28, 36); a.set_ylim(-2, 36)
a = ax[1, 2]; both(a, [-13.5, 0, 0], [1, 0, 0], (1, 2))
rect(a, -32.5, 32.5, F, F + g.PI_T, color="g", lw=1.5)
rect(a, -36.5, 36.5, g.Z_RIM + g.RIB_H, g.Z_RIM + g.RIB_H + 18, color="k", ls="--", lw=1)
a.set_title("x=-13.5 侧剖面（虚线=电池）"); a.set_xlim(-40, 40); a.set_ylim(-2, 52)
for a in ax[1]:
    a.axhline(0, color="k", lw=2)
for a in ax.flat:
    a.set_aspect("equal"); a.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "sections_v3.png"), dpi=100)
