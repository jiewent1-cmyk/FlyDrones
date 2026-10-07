"""Figures for ecps295/reports/ from the result files committed in this branch.

    cd ecps295 && python reports/make_figures.py          # writes reports/figures/*.png

Inputs (all in the repo): results/ablation_2026-10-06/{e1,e2,e3_*}.csv|json,
rl/results/gazebo/gazebo_runs.csv, rl/results/twin/*/{gens,val}.jsonl|best_val.json.
The S7e numbers come from the README table (the per-run CSVs of that test are on the ROG only).
"""
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
ABL = ROOT / "results" / "ablation_2026-10-06"
RL = ROOT / "rl" / "results"
OUT = HERE / "figures"

# Palette: validated categorical order (blue, orange, aqua, yellow, ...), recessive greys for chrome.
BLUE, ORANGE, AQUA, YELLOW, MAGENTA, GREEN, VIOLET, RED = (
    "#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948")
INK, INK2, MUTED, GRID, SURF = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e0", "#fcfcfb"
NS = "#b9b8b2"  # not significant

plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "font.family": "DejaVu Sans", "font.size": 9.5,
    "axes.edgecolor": GRID, "axes.linewidth": 0.8, "axes.labelcolor": INK2,
    "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlecolor": INK, "axes.titlelocation": "left",
    "axes.titlepad": 10, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.axisbelow": True,
    "xtick.color": INK2, "ytick.color": INK2, "xtick.major.size": 0, "ytick.major.size": 0,
    "legend.frameon": False, "legend.fontsize": 9, "legend.labelcolor": INK2,
})

RNG = np.random.default_rng(295)


def boot_ci(x, n=10000):
    x = np.asarray(x, float)
    m = x[RNG.integers(0, len(x), (n, len(x)))].mean(axis=1)
    return x.mean(), np.percentile(m, 2.5), np.percentile(m, 97.5)


def wilson(k, n, z=1.96):
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return p, c - h, c + h


def rows(path):
    return list(csv.DictReader(open(path)))


def by(rs, key):
    d = {}
    for r in rs:
        d.setdefault(r[key], []).append(r)
    return d


def col(rs, k):
    return np.array([float(r[k]) for r in rs if r[k] != ""])


def jsonl(path):
    return [json.loads(line) for line in open(path)]


def note(fig, text, y=-0.04):
    fig.text(0.01, y, text, color=MUTED, fontsize=7.5, ha="left", va="top")


def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / name, dpi=200, bbox_inches="tight", pad_inches=0.15)
    plt.close(fig)
    print("wrote", OUT / name)


LABEL = {
    "full": "Full system (v4)", "no_new_cells": "− all new cells (LCb, LCn, LCv)",
    "no_saccade": "− saccade escape", "no_lcb": "− LCb (blank cue)", "no_lcb_side": "− LCb side split",
    "no_lcv": "− LCv (ventral cue)", "no_retreat": "− retreat (MDN)", "no_routeB": "− route B (FC height)",
    "up": "Upstream FlyDrones", "up_B": "Upstream + route B", "no_lcn": "− LCn (centering)",
    "no_retreat_cap": "− retreat cap", "no_baro_rise": "− baro-rise check", "no_caution": "− caution period",
    "no_brake_mem": "− brake memory", "no_efference": "− efference copy", "no_yaw_decouple": "− yaw decoupling",
    "no_fence_turn": "− fence turn-back", "gps_ref": "GPS reference (v4)",
    "lad_v1": "v1.3", "lad_v2": "v2.1", "lad_v3": "v3",
}


# ---------------------------------------------------------------------------------------------------------
def fig_ladder():
    e1, e2 = by(rows(ABL / "e1.csv"), "condition"), by(rows(ABL / "e2.csv"), "condition")
    steps = ["up_B", "lad_v1", "lad_v2", "lad_v3", "full"]
    names = ["Upstream\n+ route B", "v1.3\ndecoder", "v2.1\n+ LCb", "v3\n+ LCn", "v4\n+ LCv/MDN"]
    x = np.arange(len(steps))
    fig, axs = plt.subplots(1, 3, figsize=(14.5, 3.8), gridspec_kw={"wspace": 0.3})

    ax = axs[0]
    p = [wilson(int(col(e2[s], "contact_any").sum()), len(e2[s])) for s in steps]
    ax.bar(x, [100 * v[0] for v in p], width=0.56, color=BLUE, edgecolor=SURF, linewidth=2)
    ax.errorbar(x, [100 * v[0] for v in p], yerr=[[100 * (v[0] - v[1]) for v in p], [100 * (v[2] - v[0]) for v in p]],
                fmt="none", ecolor=INK2, elinewidth=1, capsize=3)
    for i, v in enumerate(p):
        ax.text(i, 100 * v[2] + 3, f"{100 * v[0]:.0f}%", ha="center", color=INK, fontsize=9)
    ax.set_ylim(0, 115)
    ax.set_title("E2 wall approach: runs with contact")
    ax.set_ylabel("% of 32 approaches (95% Wilson CI)")

    ax = axs[1]
    for i, s in enumerate(steps):
        v = col(e2[s], "min_clearance_m")
        ax.scatter(i + RNG.uniform(-0.17, 0.17, len(v)), v, s=11, color=BLUE, alpha=0.45, linewidths=0)
        ax.plot([i - 0.25, i + 0.25], [np.median(v)] * 2, color=INK, lw=2, solid_capstyle="round")
    ax.axhline(0, color=RED, lw=1)
    ax.text(len(steps) - 0.5, 0.02, "contact", color=RED, fontsize=8, ha="right", va="bottom")
    ax.set_title("E2: minimum clearance to the wall")
    ax.set_ylabel("m (dots: runs, bar: median)")

    ax = axs[2]
    m = [boot_ci(col(e1[s], "contacts_per_100m")) for s in steps]
    ax.bar(x, [v[0] for v in m], width=0.56, color=ORANGE, edgecolor=SURF, linewidth=2)
    ax.errorbar(x, [v[0] for v in m], yerr=[[v[0] - v[1] for v in m], [v[2] - v[0] for v in m]],
                fmt="none", ecolor=INK2, elinewidth=1, capsize=3)
    ax.set_title("E1 low-box patrol: contacts per 100 m")
    ax.set_ylabel("mean, 95% bootstrap CI")
    for ax in axs:
        ax.set_xticks(x, names, fontsize=8.5)
        ax.grid(axis="x", visible=False)
    note(fig, "Gazebo Harmonic + ArduPilot SITL, no GPS (optical flow + ToF + route B). E2 n=32 per step; "
              "E1 n=20 (upstream, ladder) / 50 (v4). Source: results/ablation_2026-10-06/.")
    save(fig, "fig1_version_ladder.png")


def forest(ax, conds, js, metric, scale=1.0, xlabel=""):
    y = np.arange(len(conds))[::-1]
    for yi, c in zip(y, conds):
        s = js["conditions"][c][metric]
        d, lo, hi = (scale * v for v in (s["diff"], *s["diff_ci"]))
        sig = s["p_holm"] < 0.05
        colr = BLUE if sig else NS
        ax.plot([lo, hi], [yi, yi], color=colr, lw=2, solid_capstyle="round")
        ax.scatter([d], [yi], s=46, color=colr, zorder=3, edgecolors=SURF, linewidths=1.5)
    ax.axvline(0, color=INK2, lw=0.9)
    ax.set_yticks(y, [LABEL[c] + f"  (n={js['conditions'][c][metric]['n']})" for c in conds])
    ax.grid(axis="y", visible=False)
    ax.set_xlabel(xlabel)


def fig_e1_forest():
    js = json.load(open(ABL / "e1.json"))
    conds = [c for c in js["conditions"] if not c.startswith("lad_")]
    conds.sort(key=lambda c: -js["conditions"][c]["contacts_per_100m"]["diff"])
    fig, axs = plt.subplots(1, 2, figsize=(11.5, 6.2), sharey=True, gridspec_kw={"wspace": 0.06})
    forest(axs[0], conds, js, "contacts_per_100m", xlabel="Δ contacts per 100 m vs full system")
    forest(axs[1], conds, js, "success", scale=100, xlabel="Δ success rate vs full system (percentage points)")
    axs[0].set_title("E1 ablation: what each component is worth")
    axs[1].tick_params(axis="y", length=0)
    h = [plt.Line2D([], [], color=BLUE, marker="o", lw=2, label="Holm-corrected p < 0.05"),
         plt.Line2D([], [], color=NS, marker="o", lw=2, label="not significant")]
    axs[1].legend(handles=h, loc="lower left")
    note(fig, "Paired on the episode (same world, same brain seed). Point: mean difference; line: 95% bootstrap CI. "
              "Full system: 1.07 contacts/100 m, 82% success (n=50). 20 random low-box cages, 120 s patrols.")
    save(fig, "fig2_e1_ablation_forest.png")


def fig_e2():
    e2 = by(rows(ABL / "e2.csv"), "condition")
    js = json.load(open(ABL / "e2.json"))["conditions"]
    conds = [c for c in e2 if not c.startswith("lad_")]
    conds.sort(key=lambda c: np.median(col(e2[c], "min_clearance_m")))
    fig, ax = plt.subplots(figsize=(10, 5.4))
    for i, c in enumerate(conds):
        v = col(e2[c], "min_clearance_m")
        sig = c != "full" and js[c]["min_clearance_m"]["p_holm"] < 0.05
        colr = INK if c == "full" else (BLUE if sig else NS)
        ax.scatter(v, i + RNG.uniform(-0.18, 0.18, len(v)), s=12, color=colr, alpha=0.5, linewidths=0)
        ax.plot([np.median(v)] * 2, [i - 0.3, i + 0.3], color=colr, lw=2.4, solid_capstyle="round")
        rate = col(e2[c], "contact_any").mean()
        ax.text(1.42, i, f"{100 * rate:.0f}%", va="center", ha="right", color=INK if rate > 0 else MUTED, fontsize=9)
    ax.text(1.42, len(conds) - 0.2, "contact", ha="right", va="bottom", color=INK2, fontsize=8.5)
    ax.axvline(0, color=RED, lw=1)
    ax.set_yticks(range(len(conds)), [LABEL[c] for c in conds])
    ax.set_xlim(-0.25, 1.45)
    ax.grid(axis="y", visible=False)
    ax.set_xlabel("minimum clearance prop-tip to wall (m)   ·   left of the red line = contact")
    ax.set_title("E2 wall approach: clearance by condition (4 walls × 8 seeds)")
    h = [plt.Line2D([], [], color=INK, lw=2.4, label="full system"),
         plt.Line2D([], [], color=BLUE, lw=2.4, label="differs from full, Holm p < 0.05"),
         plt.Line2D([], [], color=NS, lw=2.4, label="not significant")]
    ax.legend(handles=h, loc="lower right", bbox_to_anchor=(0.9, 0))
    note(fig, "Walls 1.5 m tall (above the 1.0 m ceiling): taped, plain, offset, offset plain. Bar: median of 32 runs.")
    save(fig, "fig3_e2_wall_clearance.png")


def fig_e3():
    f = {k: by(rows(ABL / f"e3_{k}.csv"), "condition") for k in ("cam_freeze", "hdrop", "hang")}
    panels = [
        ("Camera freezes", "with contact", f["cam_freeze"]["cam_freeze"], f["cam_freeze"]["cam_freeze_no_wd"],
         "contact_any", "camera watchdog", "p = 0.25 (n.s.)"),
        ("Height module stops", "above 1.3 m fence", f["hdrop"]["hdrop"], f["hdrop"]["hdrop_no_wd"],
         "above_fence", "height-source watchdog", "p = 0.016"),
        ("Companion hangs", "landed safely", f["hang"]["hang"], f["hang"]["hang_no_fs"],
         "fc_land", "FC failsafe (FS_GCS)", "p = 0.002"),
    ]
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.6), sharey=True, gridspec_kw={"wspace": 0.22})
    for ax, (t, yl, w, wo, k, mech, p) in zip(axs, panels):
        a, b = int(col(w, k).sum()), int(col(wo, k).sum())
        ax.bar([0, 1], [a, b], width=0.55, color=[BLUE, ORANGE], edgecolor=SURF, linewidth=2)
        for i, v in enumerate((a, b)):
            ax.text(i, v + 0.3, f"{v}/10", ha="center", color=INK, fontsize=9.5)
        ax.set_xticks([0, 1], [f"with\n{mech}", "without"], fontsize=8.5)
        ax.set_title(t, pad=22)
        ax.text(0, 1.02, f"runs {yl}  ·  McNemar {p}", transform=ax.transAxes, fontsize=8.5, color=INK2, va="bottom")
        ax.grid(axis="x", visible=False)
        ax.set_ylim(0, 11.5)
    axs[0].set_ylabel("runs out of 10")
    note(fig, "Fault injected at t = 30 s in 10 low-box cages, v4 + route B, no GPS. "
              "The one camera-freeze contact with the watchdog happened at t = 20 s, before the fault.")
    save(fig, "fig4_e3_fault_injection.png")


def fig_s7e():
    # README §S7e table, 5 random low-box cages per group, NAV=flow.
    groups = ["v3", "v4"]
    ekf = {"Runs with contact": (4, 1), "FC landings": (4, 4), "FC height error, median (m)": (1.11, 0.71)}
    comp = {"Runs with contact": (1, 0), "FC landings": (0, 0), "FC height error, median (m)": (0.15, 0.10)}
    fig, axs = plt.subplots(1, 3, figsize=(11, 3.3), gridspec_kw={"wspace": 0.3})
    x = np.arange(2)
    for ax, k in zip(axs, ekf):
        ax.bar(x - 0.17, ekf[k], 0.32, color=ORANGE, edgecolor=SURF, linewidth=2, label="FC height: flow EKF (ToF)")
        ax.bar(x + 0.17, comp[k], 0.32, color=BLUE, edgecolor=SURF, linewidth=2, label="FC height: companion (route B)")
        for xi, v in zip(list(x - 0.17) + list(x + 0.17), list(ekf[k]) + list(comp[k])):
            ax.text(xi, v, f"{v:g}" if "m)" in k else f"{v}/5", ha="center", va="bottom", fontsize=8.5, color=INK)
        ax.set_xticks(x, [f"MiniFly {g}" for g in groups])
        ax.set_title(k)
        ax.grid(axis="x", visible=False)
        ax.set_ylim(0, max(ekf[k]) * 1.25)
    h, l = axs[0].get_legend_handles_labels()
    fig.legend(h[:2], l[:2], loc="upper center", ncol=2, bbox_to_anchor=(0.5, 1.08))
    note(fig, "S7e (2026-10-04): 120 s patrols, 5 random low-box cages per group, optical flow + ToF, no GPS. "
              "Route B = Orange Pi sends height above the floor as VISION_POSITION_ESTIMATE.")
    save(fig, "fig5_s7e_route_b.png")


def fig_cmaes():
    fig, axs = plt.subplots(1, 2, figsize=(11.5, 3.8), sharey=True, gridspec_kw={"wspace": 0.06})
    for ax, run in zip(axs, ("es2", "es3")):
        g = jsonl(RL / "twin" / run / "gens.jsonl")
        v = jsonl(RL / "twin" / run / "val.jsonl")
        gen = [r["gen"] for r in g]
        ax.plot(gen, [r["v3_2"]["F"] for r in g], color=MUTED, lw=1.6, label="v3_2 (hand-tuned) on the same worlds")
        ax.plot(gen, [r["F_median"] for r in g], color=AQUA, lw=2, label="population median")
        ax.plot(gen, [r["best"]["F"] for r in g], color=BLUE, lw=2, label="best candidate")
        ax.plot([r["gen"] for r in v], [r["mean"]["F"] for r in v], "o", color=ORANGE, ms=7, mec=SURF, mew=1.5,
                label="distribution mean on 24-episode hold-out")
        ax.axhline(v[0]["v3_2"]["F"], color=ORANGE, lw=1, alpha=0.6)
        ax.set_title(f"CMA-ES run {run}" + (" (17 params, wide bounds)" if run == "es3" else " (17 params)"))
        ax.set_xlabel("generation (12 candidates × 12 episodes × 90 s)")
    axs[0].set_ylabel("fitness F (higher is better)")
    axs[0].legend(loc="lower right", fontsize=8)
    axs[1].text(29, v[0]["v3_2"]["F"] - 0.04, "v3_2 hold-out", color=ORANGE, ha="right", va="top", fontsize=8)
    note(fig, "Fast twin (ray-cast fisheye camera + real Retina/MiniFly code). Common random numbers per generation; "
              "the hold-out set and cage20 never enter training.")
    save(fig, "fig6_cmaes_convergence.png")


def fig_cage20():
    g = by(rows(RL / "gazebo" / "gazebo_runs.csv"), "group")
    sets = [("val_v3_2", "v3_2", MUTED), ("val_es2", "es2", AQUA), ("val_es3", "es3", BLUE),
            ("fix_v3_2", "v3_2", MUTED), ("fix_es3", "es3", BLUE), ("fix_v4", "v4", ORANGE)]
    xs = [0, 1, 2, 3.8, 4.8, 5.8]
    fig, axs = plt.subplots(1, 3, figsize=(12.5, 3.7), gridspec_kw={"wspace": 0.3, "width_ratios": [1, 1, 1.1]})
    ax = axs[0]
    for x, (k, _, c) in zip(xs, sets):
        n, kk = len(g[k]), int((col(g[k], "contact_episodes") > 0).sum())
        p, lo, hi = wilson(kk, n)
        ax.bar(x, 100 * p, 0.7, color=c, edgecolor=SURF, linewidth=2)
        ax.errorbar(x, 100 * p, [[100 * (p - lo)], [100 * (hi - p)]], fmt="none", ecolor=INK2, elinewidth=1, capsize=3)
        ax.text(x, 100 * hi + 2, f"{kk}/{n}", ha="center", fontsize=8.5, color=INK)
    ax.set_ylim(0, 100)
    ax.set_title("Runs with any contact")
    ax.set_ylabel("% of 32 runs (Wilson 95% CI)")
    ax = axs[1]
    for x, (k, _, c) in zip(xs, sets):
        m, lo, hi = boot_ci(col(g[k], "coverage"))
        ax.bar(x, m, 0.7, color=c, edgecolor=SURF, linewidth=2)
        ax.errorbar(x, m, [[m - lo], [hi - m]], fmt="none", ecolor=INK2, elinewidth=1, capsize=3)
    ax.set_ylim(0.5, 0.7)
    ax.set_title("Coverage of 0.5 m cells")
    ax.set_ylabel("mean, 95% bootstrap CI (axis from 0.5)")
    for ax in axs[:2]:
        ax.set_xticks(xs, [s[1] for s in sets])
        ax.grid(axis="x", visible=False)
        ax.text(1, -0.15, "before slew fix", transform=ax.get_xaxis_transform(), ha="center", color=INK2, fontsize=8.5)
        ax.text(4.8, -0.15, "after slew fix (8b8eef9)", transform=ax.get_xaxis_transform(), ha="center",
                color=INK2, fontsize=8.5)
    ax = axs[2]
    for i, (k, lab, c) in enumerate([("val_v3_2", "v3_2", MUTED), ("val_es2", "es2", AQUA), ("val_es3", "es3", BLUE)]):
        v = col(g[k], "saccade_turn_deg_median")
        ax.scatter(v, i + RNG.uniform(-0.17, 0.17, len(v)), s=14, color=c, alpha=0.7, linewidths=0)
        ax.plot([np.median(v)] * 2, [i - 0.3, i + 0.3], color=INK, lw=2, solid_capstyle="round")
        ax.text(np.median(v), i + 0.36, f"{np.median(v):.0f}°", ha="center", fontsize=8.5, color=INK)
    ax.axvspan(135, 185, color=GRID, alpha=0.6, lw=0)
    ax.text(160, 2.62, "U-turn", ha="center", color=INK2, fontsize=8)
    ax.set_yticks([0, 1, 2], ["v3_2", "es2", "es3"])
    ax.set_ylim(-0.5, 2.8)
    ax.set_xlim(40, 185)
    ax.grid(axis="y", visible=False)
    ax.set_xlabel("per-run median saccade turn (deg)")
    ax.set_title("Why es3 covers less: it turns around")
    note(fig, "Gazebo cage20 (20 ft course cage, never used in training), GPS, 120 s patrols, seeds 0–31 per version. "
              "Contact counts after the fix: v3_2 vs es3 paired McNemar Holm p = 0.22.", y=-0.1)
    save(fig, "fig7_cage20_gazebo.png")


def fig_arms():
    arms = ["a1_intact", "a1_shuffle1", "a1_shuffle2", "a1_shuffle3", "a1_shuffle4", "a1_shuffle5",
            "a1_bypass", "a1_randread1"]
    lab = {"a1_intact": "intact connectome", "a1_bypass": "bypass (no LIF)", "a1_randread1": "random readout"}
    colr = {"a1_intact": BLUE, "a1_bypass": ORANGE, "a1_randread1": ORANGE}
    best = {a: json.load(open(RL / "twin" / a / "best_val.json")) for a in arms}
    base = json.load(open(RL / "twin" / "a1_intact" / "val_v3_2.json"))
    names = [lab.get(a, a.replace("a1_", "")) for a in arms] + ["v3_2, not optimised"]
    F = [best[a]["mean"]["F"] for a in arms] + [base["F"]]
    cov = [best[a]["mean"]["coverage"] for a in arms] + [base["coverage"]]
    pc = [best[a]["mean"]["p_contact"] for a in arms] + [base["p_contact"]]
    cs = [colr.get(a, AQUA) for a in arms] + [MUTED]
    y = np.arange(len(names))[::-1]
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.9), sharey=True, gridspec_kw={"wspace": 0.08})
    for ax, vals, t, fmt in zip(axs, (F, pc, cov), ("Hold-out fitness F", "Episodes with contact",
                                                     "Coverage"), ("{:+.2f}", "{:.0%}", "{:.2f}")):
        ax.barh(y, vals, 0.62, color=cs, edgecolor=SURF, linewidth=2)
        for yi, v in zip(y, vals):
            ax.text(v + (0.01 if v >= 0 else -0.01), yi, fmt.format(v), va="center",
                    ha="left" if v >= 0 else "right", fontsize=8, color=INK2)
        ax.axvline(0, color=INK2, lw=0.8)
        ax.set_title(t)
        ax.grid(axis="y", visible=False)
    axs[0].set_xlim(-0.75, 0.45)
    axs[1].set_xlim(0, 0.42)
    axs[2].set_xlim(0, 0.62)
    axs[0].set_yticks(y, names)
    h = [plt.Rectangle((0, 0), 1, 1, color=c, label=l) for c, l in
         ((BLUE, "intact MiniFly"), (AQUA, "degree-preserving shuffle"), (ORANGE, "degenerate controls"),
          (MUTED, "baseline"))]
    fig.legend(handles=h, loc="upper center", ncol=4, bbox_to_anchor=(0.5, 1.07))
    note(fig, "A1 control arms, fast twin: every arm re-optimised from v3_2 with the same budget "
              "(20 gens × 12 candidates × 12 episodes); best generation chosen on each arm's hold-out.")
    save(fig, "fig8_a1_control_arms.png")


if __name__ == "__main__":
    fig_ladder()
    fig_e1_forest()
    fig_e2()
    fig_e3()
    fig_s7e()
    fig_cmaes()
    fig_cage20()
    fig_arms()
