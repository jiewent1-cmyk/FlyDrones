"""Figures 9-16 (reports 5 and 6: the hardware-aligned setting) from result files committed in this branch.

    cd ecps295 && python reports/make_figures_hw.py      # writes reports/figures/fig9..16_*.png and reports/hw_stats.txt

Inputs (all in the repo):
  rl/results/gazebo/hw_runs.csv     one row per Gazebo run since 2026-10-05 (ROG and server), optical flow + ToF + route B
  rl/results/twin/hw/tests.csv.gz   fast-twin test episodes: rl.hw_arms (control arms), --conds (16 exported seeds),
                                    rl.ood (6 out-of-distribution modes)
  rl/results/twin/hw/<run>/         CMA-ES runs h2_s1..8 (intact), c4_s1..8 (bypass), c1_s1..3 / c1b_s1 (efference)
Group names in hw_runs.csv: fx_* ROG H0/H1 matrix, h0low_/h0cage_/h0wall_* server H0/H3/C0/C2, mc1_* ROG C1,
mc4_* / mc4r_* ROG C4 seeds (mc4r: brain seeds + 100), md_* ROG stage D (notex / ghost).
"""
import csv
import gzip
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import binomtest, mannwhitneyu, spearmanr

from make_figures import (AQUA, BLUE, GRID, INK, INK2, MUTED, NS, ORANGE, RED, SURF, VIOLET, YELLOW, note, save,
                          wilson)

HERE = Path(__file__).resolve().parent
RL = HERE.parent / "rl" / "results"
RNG = np.random.default_rng(2951)
OUT_TXT = []

BYP, INT = ORANGE, BLUE  # bypass vs intact everywhere in reports 5-6
SEEDS = range(1, 9)


def log(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    OUT_TXT.append(s)


# ---------------------------------------------------------------------------------------------------------- data
def gz_runs():
    D = {}
    for r in csv.DictReader(open(RL / "gazebo" / "hw_runs.csv")):
        for k in ("contact_episodes", "near_miss_episodes", "min_clearance_m", "path_m", "coverage", "over_box_s",
                  "ekf_err_xy_max_m"):
            r[k] = float(r[k]) if r[k] not in ("", "None") else np.nan
        r["c100"] = 100.0 * r["contact_episodes"] / max(1.0, r["path_m"])
        D.setdefault((r["group"], r["scenario"]), {})[(r["world"], int(r["seed"]))] = r
    return D


def twin_tests():
    T = {}
    for r in csv.DictReader(gzip.open(RL / "twin" / "hw" / "tests.csv.gz", "rt")):
        src = "arms" if r["file"].startswith("hw_arms") else "ood" if r["file"].startswith("ood") else "seeds"
        if src == "seeds" and r["tag"] == "v4" and r["file"] == "seeds_b.jsonl":
            continue  # v4 ran on both machines (bit-identical); keep one copy
        for k in ("contact_episodes", "min_clearance_m", "path_m", "coverage", "false_alarms_per_min"):
            r[k] = float(r[k]) if r[k] not in ("", "None") else np.nan
        T.setdefault((src, r["tag"], r["ood"], r["set"]), {})[(r["world"], r["key"])] = r
    return T


def runs_any(R):
    return sum(r["contact_episodes"] > 0 for r in R.values())


def paired(A, B, f):
    k = sorted(set(A) & set(B))
    return np.array([f(A[x]) for x in k]), np.array([f(B[x]) for x in k])


def mcnemar(A, B):
    a, b = paired(A, B, lambda r: r["contact_episodes"] > 0)
    saved, new = int((a & ~b).sum()), int((~a & b).sum())
    p = binomtest(saved, saved + new, 0.5).pvalue if saved + new else 1.0
    return len(a), int(a.sum()), int(b.sum()), saved, new, p


def boot_diff(A, B, key, n=10000):
    a, b = paired(A, B, lambda r: r[key])
    d = b - a
    d = d[~np.isnan(d)]
    m = d[RNG.integers(0, d.size, (n, d.size))].mean(axis=1)
    return d.mean(), np.percentile(m, 2.5), np.percentile(m, 97.5)


def boot_mean(x, n=10000):
    x = np.asarray(x, float)
    m = x[RNG.integers(0, x.size, (n, x.size))].mean(axis=1)
    return x.mean(), np.percentile(m, 2.5), np.percentile(m, 97.5)


def mw(a, b):
    return mannwhitneyu(a, b, alternative="two-sided", method="exact").pvalue


def ci_bar(ax, y, m, lo, hi, color, size=34):
    ax.plot([lo, hi], [y, y], color=color, lw=2, solid_capstyle="round", alpha=0.85)
    ax.scatter([m], [y], s=size, color=color, zorder=3, edgecolors=SURF, linewidths=1.5)


def strip(ax, x, vals, color, med=True, w=0.16, s=34):
    vals = np.asarray(vals, float)
    ax.scatter(x + RNG.uniform(-w, w, vals.size), vals, s=s, color=color, alpha=0.8, edgecolors=SURF, linewidths=1,
               zorder=3)
    if med:
        ax.plot([x - 0.28, x + 0.28], [np.median(vals)] * 2, color=INK, lw=2, solid_capstyle="round", zorder=4)


# ---------------------------------------------------------------------------------------------------------- fig 9
def fig_lowbox(G):
    """H1: lowbox contact runs are insensitive to the decoder; Gazebo cannot rank contacts reliably."""
    models = [("fx_full", "v4 (hand-built)"), ("fx_m_v4_oc5", "v4_oc5"), ("fx_m_v3_2", "v3_2"),
              ("fx_m_es2", "es2 (CMA, GPS cage)"), ("fx_m_es3", "es3 (CMA, GPS cage)"), ("fx_m_wcov2", "wcov2"),
              ("fx_m_wcov4", "wcov4"), ("fx_m_wcov8", "wcov8"), ("fx_h2_s1", "h2_s1 (CMA, v4 base)"),
              ("fx_h2_s2", "h2_s2"), ("fx_h2_s3", "h2_s3"), ("mc1_v4_eff", "v4_eff (C1)")]
    fig, axs = plt.subplots(1, 2, figsize=(14, 4.6), gridspec_kw={"wspace": 0.55, "width_ratios": [1.15, 1]})
    ax = axs[0]
    log("== fig9 lowbox contact runs (ROG, lowbox_s0-49)")
    for i, (g, name) in enumerate(models):
        R = G[(g, "lowbox")]
        k, n = runs_any(R), len(R)
        p = wilson(k, n)
        c = BLUE if not g.startswith("fx_h2") else AQUA
        ci_bar(ax, i, 100 * p[0], 100 * p[1], 100 * p[2], c)
        ax.text(61, i, f"{k}/{n}", va="center", fontsize=8.5, color=INK2)
        log(f"  {name:24s} {k}/{n}")
    ax.set_yticks(range(len(models)), [m[1] for m in models])
    ax.invert_yaxis()
    ax.set_xlim(0, 60)
    ax.set_xlabel("% of runs with ≥ 1 contact (95% Wilson CI)")
    ax.set_title("Lowbox: every decoder hits about 1 run in 5")
    ax.grid(axis="y", visible=False)

    # split-half reliability of each metric across the 8 fx models (the H1 calibration models)
    ax = axs[1]
    m8 = ["fx_full", "fx_m_v4_oc5", "fx_m_v3_2", "fx_m_es2", "fx_m_es3", "fx_m_wcov2", "fx_m_wcov4", "fx_m_wcov8"]
    worlds = sorted({w for g in m8 for (w, _s) in G[(g, "lowbox")]})
    mets = [("contact", lambda r: float(r["contact_episodes"] > 0)), ("contacts / 100 m", lambda r: r["c100"]),
            ("min clearance", lambda r: r["min_clearance_m"]), ("time over boxes", lambda r: r["over_box_s"]),
            ("EKF error", lambda r: r["ekf_err_xy_max_m"]), ("coverage", lambda r: r["coverage"]),
            ("path length", lambda r: r["path_m"])]
    tab = {}
    for name, f in mets:
        M = np.full((len(m8), len(worlds)), np.nan)
        for i, g in enumerate(m8):
            for (w, _s), r in G[(g, "lowbox")].items():
                M[i, worlds.index(w)] = f(r)
        rs = []
        for _ in range(2000):
            perm = RNG.permutation(len(worlds))
            a, b = perm[: len(worlds) // 2], perm[len(worlds) // 2:]
            rs.append(spearmanr(np.nanmean(M[:, a], 1), np.nanmean(M[:, b], 1))[0])
        tab[name] = (np.nanmedian(rs), np.nanpercentile(rs, 10), np.nanpercentile(rs, 90))
    names = sorted(tab, key=lambda k: tab[k][0])
    log("== fig9 split-half rho (8 models, 50 worlds, 2000 random halves): " +
        ", ".join(f"{k} {tab[k][0]:.2f}" for k in names))
    for i, k in enumerate(names):
        m, lo, hi = tab[k]
        ci_bar(ax, i, m, lo, hi, BLUE if m >= 0.6 else NS)
    ax.axvline(0.6, color=INK2, lw=0.8, ls=(0, (3, 3)))
    ax.text(0.61, len(names) - 0.6, "gate 0.6", fontsize=8, color=INK2)
    ax.set_yticks(range(len(names)), names)
    ax.set_xlim(-1, 1)
    ax.set_xlabel("Spearman ρ between two random halves of the worlds (median, 10–90%)")
    ax.set_title("Which Gazebo metrics can rank 8 models at all")
    ax.grid(axis="y", visible=False)
    note(fig, "ROG Gazebo 8.15, optical flow + ToF + route B, lowbox_s0–49, 120 s patrols, paired worlds and brain "
              "seeds. Blue: hand-built and GPS-cage CMA decoders; aqua: CMA on the v4 base (H2). Right: the 8 models "
              "of the H1 calibration; grey = below the 0.6 gate.")
    save(fig, "fig9_h1_lowbox_reliability.png")


# ---------------------------------------------------------------------------------------------------------- fig 10
def fig_h3(G):
    """H3 acceptance on the server: h2 candidates (and es3) vs v4, paired, three scenarios."""
    cands = [("h2_s1", BLUE), ("h2_s2", BLUE), ("h2_s3", AQUA), ("es3", VIOLET)]
    scen = [("h0low", "lowbox", "lowbox_s0–49"), ("h0wall", None, "wall approaches ×32"),
            ("h0cage", "cage20", "cage20 ×32")]
    fig, axs = plt.subplots(1, 3, figsize=(14, 3.6), gridspec_kw={"wspace": 0.12}, sharey=True)
    log("== fig10 H3 (server) candidate - v4")
    for j, key in enumerate(("min_clearance_m", "coverage", "c100")):
        ax = axs[j]
        for i, (pre, sc, lab) in enumerate(scen):
            for c, (cand, col) in enumerate(cands):
                ref = merge(G, f"{pre}_v4", sc)
                B = merge(G, f"{pre}_{cand}", sc)
                if not B:
                    continue
                m, lo, hi = boot_diff(ref, B, key)
                y = i * (len(cands) + 1.2) + c
                sig = lo > 0 or hi < 0
                ci_bar(ax, y, m, lo, hi, col if sig else NS)
                if j == 0:
                    n, a, b, s, nw, p = mcnemar(ref, B)
                    log(f"  {lab:20s} {cand:6s} contact runs {a}->{b}/{n} (saved {s} new {nw}, p={p:.2f}) "
                        f"dclr {boot_diff(ref, B, 'min_clearance_m')[0]:+.3f} "
                        f"dcov {boot_diff(ref, B, 'coverage')[0]:+.3f}")
        ax.axvline(0, color=INK2, lw=0.8)
        ax.set_title({"min_clearance_m": "Δ min clearance (m)", "coverage": "Δ coverage",
                      "c100": "Δ contacts / 100 m"}[key])
        ax.grid(axis="y", visible=False)
    yt, yl = [], []
    for i, (_p, _s, lab) in enumerate(scen):
        for c, (cand, _col) in enumerate(cands):
            yt.append(i * (len(cands) + 1.2) + c)
            yl.append(f"{lab} · {cand}" if c == 0 else cand)
    axs[0].set_yticks(yt, yl, fontsize=8.5)
    axs[0].invert_yaxis()
    note(fig, "Server Gazebo 8.10 (sim clock), optical flow + ToF + route B. Paired differences candidate − v4 "
              "(same worlds and brain seeds), mean and 95% bootstrap CI; grey = CI includes 0. h2_s3 (aqua) passed "
              "H3 as the conservative candidate; es3 (violet) is the GPS-cage decoder for reference.")
    save(fig, "fig10_h3_acceptance.png")


def merge(G, group, sc):
    """lowbox / cage20 groups are one scenario; wall groups span four wall worlds."""
    if sc is not None:
        return G.get((group, sc), {})
    out = {}
    for (g, s), R in G.items():
        if g == group and s.startswith("wall"):
            out.update(R)
    return out


# ---------------------------------------------------------------------------------------------------------- fig 11
def fig_c0c2(G, T):
    """C0 / C2 under the hardware-aligned setting: brain off and degree-preserving shuffles (server Gazebo)."""
    arms = [("v4", "v4 (real wiring)", BLUE)] + [(f"shuffle{k}", f"shuffle {k}", AQUA) for k in range(1, 6)] + \
           [("noloom", "no looming input", ORANGE), ("brainoff", "brain off", ORANGE)]
    fig, axs = plt.subplots(1, 3, figsize=(14, 3.9), gridspec_kw={"wspace": 0.5, "width_ratios": [1, 1, 1]})
    log("== fig11 C0/C2 (server Gazebo) contacts/100 m and contact runs")
    for j, (pre, sc, title) in enumerate((("h0low", "lowbox", "Gazebo lowbox_s0–49"),
                                          ("h0cage", "cage20", "Gazebo cage20 ×32"))):
        ax = axs[j]
        for i, (a, lab, col) in enumerate(arms):
            R = G[(f"{pre}_{a}", sc)]
            m, lo, hi = boot_mean([r["c100"] for r in R.values()])
            ci_bar(ax, i, max(m, 0.05), max(lo, 0.05), hi, col)
            ax.text(150, i, f"{runs_any(R)}/{len(R)}", va="center", fontsize=8, color=INK2)
            log(f"  {sc:7s} {a:9s} c100 {m:.2f} [{lo:.2f},{hi:.2f}]  runs {runs_any(R)}/{len(R)}")
        ax.set_xscale("log")
        ax.set_xlim(0.3, 120)
        ax.set_yticks(range(len(arms)), [a[1] for a in arms] if j == 0 else [])
        ax.invert_yaxis()
        ax.set_title(title)
        ax.set_xlabel("contacts / 100 m (log; mean, 95% CI)")
        ax.grid(axis="y", visible=False)
        ax.text(150, -0.9, "runs hit", fontsize=8, color=MUTED, ha="left")

    ax = axs[2]  # fast-twin prescreen: same arms, lowbox 150 / cage20 32
    log("== fig11 twin prescreen (rl.hw_arms) contact runs")
    for i, (a, lab, col) in enumerate(arms):
        for sset, off, mk in (("lowbox", -0.15, "o"), ("cage20", 0.15, "s")):
            R = T[("arms", a, "none", sset)]
            k, n = runs_any(R), len(R)
            p = wilson(k, n)
            ax.plot([100 * p[1], 100 * p[2]], [i + off] * 2, color=col, lw=2, alpha=0.85)
            ax.scatter([100 * p[0]], [i + off], s=30, marker=mk, color=col, edgecolors=SURF, linewidths=1.2,
                       zorder=3)
            log(f"  twin {sset:7s} {a:9s} {k}/{n}")
    ax.set_yticks(range(len(arms)), [])
    ax.invert_yaxis()
    ax.set_xlim(0, 100)
    ax.set_xlabel("% runs with contact\n● lowbox ×150   ■ cage20 ×32")
    ax.set_title("Fast twin prescreen")
    ax.grid(axis="y", visible=False)
    note(fig, "All arms on the v4 base, optical flow + ToF + route B. Gazebo: server, paired with v4 on the same "
              "worlds and seeds. Twin: per-episode randomised flow-error model, 3 brain seeds per lowbox world.")
    save(fig, "fig11_c0_c2_hw.png")


# ---------------------------------------------------------------------------------------------------------- fig 12
def seed_table(G, T):
    rows = []
    for arm, pre in (("bypass", "c4"), ("intact", "h2")):
        for s in SEEDS:
            name = f"{pre}_s{s}"
            bv = json.load(open(RL / "twin" / "hw" / name / "best_val.json"))
            tw = T[("seeds", name, "none", "lowbox")]
            twc = T[("seeds", name, "none", "cage20")]
            twe = T[("seeds", name, "none", "empty")]
            g = G.get((f"mc4_{name}", "lowbox")) or G.get((f"fx_{name}", "lowbox"))
            rows.append(dict(arm=arm, name=name, F=bv["mean"]["F"], gen=bv["gen"], tw=runs_any(tw), twn=len(tw),
                             twc=runs_any(twc), tw100=np.mean([r["c100"] if "c100" in r else
                                                              100 * r["contact_episodes"] / max(1, r["path_m"])
                                                              for r in tw.values()]),
                             fa=np.mean([r["false_alarms_per_min"] for r in twe.values()]),
                             gz=runs_any(g), gzn=len(g), G=g, T=tw))
    return rows


def fig_seeds(G, T, rows):
    v4g = G[("mc1_v4", "lowbox")]
    v4t = T[("seeds", "v4", "none", "lowbox")]
    log("== fig12 seed table (bypass c4 vs intact h2, same protocol)")
    for r in rows:
        n, a, b, s, nw, p = mcnemar(v4g, r["G"])
        n2, a2, b2, s2, nw2, p2 = mcnemar(v4t, r["T"])
        log(f"  {r['name']}  val F {r['F']:+.2f} (gen {r['gen']})  twin lowbox {r['tw']}/{r['twn']} (v4 {a2}, p={p2:.3f})"
            f"  cage20 {r['twc']}/32  FA/min {r['fa']:.2f}  Gazebo lowbox {r['gz']}/{r['gzn']} (v4 {a}, p={p:.3f})")
    by = {a: [r for r in rows if r["arm"] == a] for a in ("bypass", "intact")}
    panels = [("F", "Twin: best hold-out fitness F", "fitness (higher is better)", None),
              ("tw", "Twin independent test: lowbox", "runs with contact / 150", runs_any(v4t)),
              ("gz", "Gazebo (ROG): lowbox_s0–49", "runs with contact / 50", runs_any(v4g))]
    fig, axs = plt.subplots(1, 3, figsize=(13, 3.9), gridspec_kw={"wspace": 0.35})
    for ax, (k, title, yl, ref) in zip(axs, panels):
        a, b = [r[k] for r in by["bypass"]], [r[k] for r in by["intact"]]
        strip(ax, 0, a, BYP)
        strip(ax, 1, b, INT)
        p = mw(a, b)
        log(f"  {k}: bypass median {np.median(a):.2f}  intact median {np.median(b):.2f}  exact MW p={p:.5f}")
        if ref is not None:
            ax.axhline(ref, color=INK2, lw=0.9, ls=(0, (3, 3)))
            ax.text(1.68, ref, f"v4 {ref}", fontsize=8, color=INK2, va="bottom", ha="right")
        ax.set_xticks([0, 1], ["bypass\n(no LIF network)", "intact\n(MiniFly)"])
        ax.set_xlim(-0.6, 1.7)
        ax.set_title(title + "\n" + (f"8 vs 8 seeds, exact Mann-Whitney p = {p:.3f}" if p >= 0.001 else
                                      f"8 vs 8 seeds, exact Mann-Whitney p = {p:.1e}"), linespacing=1.6)
        ax.set_ylabel(yl)
        ax.grid(axis="x", visible=False)
    note(fig, "Both arms optimised with the identical protocol (v4hw space, fitness v3, randomised flow model, half "
              "lowbox worlds, 20 gens × 12 × 12, CMA seeds 1–8). Dots: seeds; bar: median. The independent twin test "
              "uses worlds and brain seeds outside the selection set (no selection bias).")
    save(fig, "fig12_bypass_vs_intact.png")


# ---------------------------------------------------------------------------------------------------------- fig 13
def fig_transfer(rows):
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.1), gridspec_kw={"wspace": 0.35})
    ax = axs[0]
    x = np.array([r["tw"] for r in rows])
    y = np.array([r["gz"] for r in rows])
    rho, p = spearmanr(x, y)
    log(f"== fig13 twin vs Gazebo per config: Spearman rho {rho:.2f} p={p:.4f}")
    for r in rows:
        ax.scatter(r["tw"], r["gz"], s=46, color=BYP if r["arm"] == "bypass" else INT, edgecolors=SURF,
                   linewidths=1.5, zorder=3)
        ax.annotate(r["name"], (r["tw"], r["gz"]), xytext=(4, 3), textcoords="offset points", fontsize=7,
                    color=MUTED)
    ax.set_xlabel("fast twin: runs with contact / 150")
    ax.set_ylabel("Gazebo: runs with contact / 50")
    ax.set_title(f"Twin ranking transfers to Gazebo (ρ = {rho:.2f})")
    ax = axs[1]
    for r in rows:
        ax.scatter(r["fa"], r["tw"], s=46, color=BYP if r["arm"] == "bypass" else INT, edgecolors=SURF,
                   linewidths=1.5, zorder=3)
        ax.annotate(r["name"], (r["fa"], r["tw"]), xytext=(4, 3), textcoords="offset points", fontsize=7,
                    color=MUTED)
    a = [r["fa"] for r in rows if r["arm"] == "bypass"]
    b = [r["fa"] for r in rows if r["arm"] == "intact"]
    log(f"  empty-field false turns/min: bypass median {np.median(a):.2f} intact {np.median(b):.2f} p={mw(a, b):.3f}")
    ax.set_xlabel("empty field: false turns per minute (twin)")
    ax.set_ylabel("lowbox runs with contact / 150 (twin)")
    ax.set_title("The price: more spontaneous turns")
    h = [plt.Line2D([], [], marker="o", ls="", color=c, label=l) for c, l in ((BYP, "bypass c4_s*"),
                                                                             (INT, "intact h2_s*"))]
    fig.legend(handles=h, loc="upper center", ncol=2, bbox_to_anchor=(0.5, 1.06))
    note(fig, "16 exported configurations. Twin: independent test (lowbox 150 episodes, empty field 32). Gazebo: ROG, "
              "lowbox_s0–49 paired with v4.")
    save(fig, "fig13_twin_gazebo_transfer.png")


# ---------------------------------------------------------------------------------------------------------- fig 14
def fig_ood(G, T):
    modes = ["lowlight", "notex", "halfcam", "wind", "tof", "ghost"]
    fig, axs = plt.subplots(1, 3, figsize=(14, 4.0), gridspec_kw={"wspace": 0.35, "width_ratios": [1.6, 1.6, 1]})
    log("== fig14 OOD (twin, seed level, change in runs with contact vs in-distribution k_0 subset)")
    for j, sset in enumerate(("lowbox", "cage20")):
        ax = axs[j]
        for i, m in enumerate(modes):
            d = {}
            for arm, pre in (("bypass", "c4"), ("intact", "h2")):
                d[arm] = []
                for s in SEEDS:
                    name = f"{pre}_s{s}"
                    O = T[("ood", name, m, sset)]
                    base = T[("seeds", name, "none", sset)]
                    base = {k: base[k] for k in O if k in base}
                    d[arm].append(runs_any(O) - runs_any(base))
            strip(ax, i - 0.2, d["bypass"], BYP, w=0.08, s=22)
            strip(ax, i + 0.2, d["intact"], INT, w=0.08, s=22)
            p = mw(d["bypass"], d["intact"])
            n = len(T[("ood", "c4_s1", m, sset)])
            if p < 0.05:
                ax.text(i, ax.get_ylim()[1] if False else max(d["bypass"] + d["intact"]) + 1.5, "*" if p >= 0.01 else
                        "**" if p >= 0.001 else "***", ha="center", fontsize=11, color=INK)
            log(f"  {sset:7s} {m:9s} n={n}  Δ bypass median {np.median(d['bypass']):+.1f}  "
                f"intact {np.median(d['intact']):+.1f}  p={p:.3f}")
        ax.axhline(0, color=INK2, lw=0.8)
        ax.set_xticks(range(len(modes)), modes)
        ax.set_title(f"Twin {sset}: change vs in-distribution")
        ax.set_ylabel(f"Δ runs with contact (of {len(T[('ood', 'c4_s1', 'notex', sset)])})")
        ax.grid(axis="x", visible=False)

    ax = axs[2]  # Gazebo notex: lowbox_s0-49 notex vs the same worlds and seeds in distribution
    log("== fig14 Gazebo notex (ROG lowbox_s0-49)")
    d = {"bypass": [], "intact": []}
    for arm, pre in (("bypass", "c4"), ("intact", "h2")):
        for s in SEEDS:
            name = f"{pre}_s{s}"
            O = G[(f"md_{name}", "lowbox_notex")]
            O = {(w.replace("_notex", ""), k): r for (w, k), r in O.items()}
            base = G.get((f"mc4_{name}", "lowbox")) or G.get((f"fx_{name}", "lowbox"))
            ks = sorted(set(O) & set(base))
            assert len(ks) >= 49, (name, len(ks))  # c4_s1 in distribution: 49 (one failed take-off)
            a, b = sum(base[k]["contact_episodes"] > 0 for k in ks), sum(O[k]["contact_episodes"] > 0 for k in ks)
            d[arm].append((a, b))
            log(f"  {name}: in-dist {a} -> notex {b} (n={len(ks)})")
    for x, arm, col in ((0, "bypass", BYP), (1, "intact", INT)):
        for a, b in d[arm]:
            ax.plot([x - 0.18, x + 0.18], [a, b], color=col, alpha=0.55, lw=1.5)
            ax.scatter([x - 0.18, x + 0.18], [a, b], s=20, color=col, edgecolors=SURF, linewidths=1, zorder=3)
    da = [b - a for a, b in d["bypass"]]
    di = [b - a for a, b in d["intact"]]
    p = mw(da, di)
    log(f"  Δ bypass median {np.median(da):+.1f} intact {np.median(di):+.1f} p={p:.3f}; in-dist medians "
        f"{np.median([a for a, _ in d['bypass']])} vs {np.median([a for a, _ in d['intact']])} "
        f"p={mw([a for a, _ in d['bypass']], [a for a, _ in d['intact']]):.3f}; notex medians "
        f"{np.median([b for _, b in d['bypass']])} vs {np.median([b for _, b in d['intact']])} "
        f"p={mw([b for _, b in d['bypass']], [b for _, b in d['intact']]):.3f}")
    ax.set_xticks([-0.18, 0.18, 0.82, 1.18], ["in-dist", "notex", "in-dist", "notex"], fontsize=8)
    ax.text(0, -0.13, "bypass", transform=ax.get_xaxis_transform(), ha="center", color=BYP, fontsize=9)
    ax.text(1, -0.13, "intact", transform=ax.get_xaxis_transform(), ha="center", color=INT, fontsize=9)
    ax.set_ylabel("runs with contact / 50")
    ax.set_title(f"Gazebo notex (Δ p = {p:.3f})")
    ax.grid(axis="x", visible=False)
    h = [plt.Line2D([], [], marker="o", ls="", color=c, label=l) for c, l in ((BYP, "bypass seeds"),
                                                                             (INT, "intact seeds"))]
    fig.legend(handles=h, loc="upper center", ncol=2, bbox_to_anchor=(0.5, 1.06))
    note(fig, "Each dot is one CMA seed (8 per arm). Twin: 50 lowbox / 32 cage20 episodes per mode, paired with the "
              "same worlds and brain seeds in distribution. Stars: exact Mann-Whitney on the change, bypass vs intact "
              "(* p < .05, ** p < .01, *** p < .001). Gazebo: ROG, textureless boxes and mat (gazebo/ood_world.py).", y=-0.1)
    save(fig, "fig14_ood.png")


# ---------------------------------------------------------------------------------------------------------- fig 15
def fig_curves():
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    for arm, pre, col in (("bypass", "c4", BYP), ("intact", "h2", INT)):
        for s in SEEDS:
            v = [json.loads(line) for line in open(RL / "twin" / "hw" / f"{pre}_s{s}" / "val.jsonl")]
            ax.plot([r["gen"] for r in v], [r["mean"]["F"] for r in v], color=col, lw=1.5, alpha=0.75,
                    marker="o", ms=3.5, label=arm if s == 1 else None)
        ref = [json.loads(line) for line in open(RL / "twin" / "hw" / f"{pre}_s1" / "val.jsonl")][0]
    ax.axhline(-1.83, color=INK2, lw=0.9, ls=(0, (3, 3)))
    ax.text(19.3, -1.83, "v4", fontsize=8, color=INK2, va="bottom", ha="right")
    ax.set_xlabel("generation")
    ax.set_ylabel("hold-out fitness F")
    ax.set_title("CMA-ES hold-out fitness, 8 seeds per arm")
    ax.legend(loc="lower right")
    note(fig, "Hold-out set: 24 fixed episodes (half lowbox), evaluated every 5 generations and at the last one. "
              "v4 line: hand-built v4 on the same hold-out (F ≈ −1.83).")
    save(fig, "fig15_c4_h2_curves.png")


# ---------------------------------------------------------------------------------------------------------- fig 16
def fig_c1(G, T):
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.4), gridspec_kw={"wspace": 0.55})
    ax = axs[0]
    log("== fig16 C1 efference copy")
    tags = [("v4", "v4"), ("v4_eff", "v4_eff"), ("h2_s3", "h2_s3"), ("h2_s3_eff", "h2_s3_eff")]
    for i, (t, lab) in enumerate(tags):
        R = T[("arms", t, "none", "empty")]
        m, lo, hi = boot_mean([r["false_alarms_per_min"] for r in R.values()])
        ci_bar(ax, i, m, lo, hi, AQUA if t.endswith("eff") else BLUE)
        log(f"  twin empty field {t}: {m:.2f}/min [{lo:.2f},{hi:.2f}]")
    ax.set_yticks(range(len(tags)), [t[1] for t in tags])
    ax.invert_yaxis()
    ax.set_xlim(0, 4)
    ax.set_xlabel("false turns per minute (mean, 95% CI)")
    ax.set_title("Twin: empty-field false alarms")
    ax.grid(axis="y", visible=False)
    ax = axs[1]
    pairs = [("ROG lowbox", ("mc1_v4", "lowbox"), ("mc1_v4_eff", "lowbox")),
             ("server lowbox", ("h0low_v4", "lowbox"), ("h0low_v4_eff", "lowbox")),
             ("server cage20", ("h0cage_v4", "cage20"), ("h0cage_v4_eff", "cage20"))]
    for i, (lab, a, b) in enumerate(pairs):
        n, ka, kb, s, nw, p = mcnemar(G[a], G[b])
        for off, k, col in ((-0.15, ka, BLUE), (0.15, kb, AQUA)):
            pw = wilson(k, n)
            ax.plot([100 * pw[1], 100 * pw[2]], [i + off] * 2, color=col, lw=2)
            ax.scatter([100 * pw[0]], [i + off], s=30, color=col, edgecolors=SURF, linewidths=1.2, zorder=3)
        ax.text(62, i, f"{ka} → {kb} / {n}  (p = {p:.2f})", va="center", fontsize=8, color=INK2)
        log(f"  Gazebo {lab}: v4 {ka} -> v4_eff {kb} / {n}, saved {s} new {nw}, p={p:.2f}")
    ax.set_yticks(range(len(pairs)), [p[0] for p in pairs])
    ax.invert_yaxis()
    ax.set_xlim(0, 60)
    ax.set_xlabel("% runs with contact (95% CI)")
    ax.set_title("Gazebo: v4 (blue) vs v4_eff (aqua)")
    ax.grid(axis="y", visible=False)
    note(fig, "v4_eff = v4 + blank-cue efference term (LCb threshold + 0.4 × turn, τ = 0.8 s). Twin: 32 empty-field "
              "episodes of 120 s. Gazebo has no empty field, so the false-alarm effect can only be checked in the twin.")
    save(fig, "fig16_c1_efference.png")


if __name__ == "__main__":
    G, T = gz_runs(), twin_tests()
    fig_lowbox(G)
    fig_h3(G)
    fig_c0c2(G, T)
    rows = seed_table(G, T)
    fig_seeds(G, T, rows)
    fig_transfer(rows)
    fig_ood(G, T)
    fig_curves()
    fig_c1(G, T)
    (HERE / "hw_stats.txt").write_text("\n".join(OUT_TXT) + "\n")
