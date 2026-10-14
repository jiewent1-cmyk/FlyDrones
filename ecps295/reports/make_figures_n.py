"""Figures 17-21 (reports 7 and 8: range sensing / safety shield, mushroom-body plasticity) from files in this branch.

    cd ecps295 && python reports/make_figures_n.py      # writes reports/figures/fig17..21_*.png and reports/n_stats.txt

Inputs (all in the repo):
  rl/results/twin/n/tests.csv.gz    fast-twin episodes: s4_screen (rl.shield_screen S4 conds), s5_screen (S5 conds),
                                    diag (rl.diag_contacts v4 / s3_rng), ood_struct (S2 / S3 under rl.ood modes)
  rl/results/twin/n/diag_events.csv one row per contact onset in the diag run (phase, bearing, seen in camera, ...)
  rl/results/twin/mb/               stage E: tests.csv.gz (frozen-g tests e1-e6), lives.jsonl (training lifetimes)
  rl/results/twin/hw/n3s5_s1/       N3 CMA-ES run (shield in the loop, fitness v5)
  rl/results/gazebo/hw_runs.csv     Gazebo groups msh_* (planar ToF), msh3_* (27 deg cone), ms5_* (S5), mb_v4 (v4,
                                    second brain seed), st_* (desktop S2 re-check); v4 pairs mc1_v4 (lowbox), md_v4 (cage20)
"""
import csv
import gzip
import json
from collections import Counter
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import binomtest, wilcoxon

from make_figures import AQUA, BLUE, INK, INK2, MUTED, ORANGE, SURF, VIOLET, YELLOW, note, save, wilson
from make_figures_hw import boot_diff, ci_bar, gz_runs, mcnemar, runs_any

HERE = Path(__file__).resolve().parent
RL = HERE.parent / "rl" / "results"
RNG = np.random.default_rng(2952)
OUT_TXT = []


def log(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    OUT_TXT.append(s)


def num(r, keys):
    for k in keys:
        r[k] = float(r[k]) if r.get(k) not in ("", "None", None) else np.nan
    return r


def twin(path):
    T = {}
    for r in csv.DictReader(gzip.open(path, "rt")):
        num(r, ("contact_episodes", "min_clearance_m", "path_m", "coverage", "false_alarms_per_min",
                "shield_clamp_ticks", "ticks"))
        r["c100"] = 100.0 * r["contact_episodes"] / max(1.0, r["path_m"])
        T.setdefault((r["file"], r["tag"], r["ood"], r["set"]), {})[(r["world"], r["key"])] = r
    return T


# ---------------------------------------------------------------------------------------------------------- fig 17
def fig_diag():
    """Where the remaining contacts come from: v4 vs the side/rear-ranger rule (s3_rng), 150 lowbox episodes each."""
    E = list(csv.DictReader(open(RL / "twin" / "n" / "diag_events.csv")))
    fig, axs = plt.subplots(1, 2, figsize=(12, 2.9), gridspec_kw={"wspace": 0.35})
    log("== fig17 contact onsets by phase / bearing (twin lowbox, 150 episodes per model)")
    for ax, key, cats, title in ((axs[0], "phase", ["forward", "turn", "back", "hover"], "Phase at contact"),
                                 (axs[1], "seen", ["seen", "not seen"], "Obstacle in the camera view 2 s before")):
        cols = [BLUE, AQUA, ORANGE, VIOLET]
        for i, tag in enumerate(("v4", "s3_rng")):
            ev = [e for e in E if e["tag"] == tag and e["set"] == "lowbox"]
            if key == "seen":
                lab = ["seen" if e["seen_2s"] == "True" else "not seen" for e in ev]
            else:
                lab = [e["phase"] for e in ev]
            c = Counter(lab)
            left = 0.0
            for j, cat in enumerate(cats):
                w = 100.0 * c[cat] / len(lab)
                ax.barh(i, w, left=left, height=0.55, color=cols[j], edgecolor=SURF, linewidth=2)
                if w >= 13:
                    ax.text(left + w / 2, i, f"{cat}\n{w:.0f}%", ha="center", va="center", fontsize=7.5, color=SURF)
                left += w
            seen = np.mean([e["seen_2s"] == "True" for e in ev])
            log(f"  {tag:7s} {key:6s} n={len(lab)} " + " ".join(f"{k} {100 * c[k] / len(lab):.0f}%" for k in cats) +
                f" | seen in camera 2 s before {100 * seen:.0f}%")
        ax.set_yticks([0, 1], ["v4", "v4 + side/rear\nrangers (S3)"])
        ax.invert_yaxis()
        ax.set_xlim(0, 100)
        ax.set_xlabel("% of contact onsets")
        ax.set_title(title)
        ax.grid(False)
    note(fig, "Fast twin, lowbox_s0–49 × 3 brain seeds. v4: 84 contact onsets in 34 episodes; S3 (side/rear rangers): "
              "28 onsets in 21 episodes. Seen = the obstacle was in the camera field of view at some point in the 2 s "
              "before contact.", y=-0.12)
    save(fig, "fig17_contact_diagnosis.png")


# ---------------------------------------------------------------------------------------------------------- fig 18
CONDS = [("v4", "v4", "s4_screen", BLUE), ("v4_c035", "v4, cruise 0.35", "s4_screen", MUTED),
         ("v4_c065", "v4, cruise 0.65", "s4_screen", MUTED), ("s3_rng", "S3 rule: side/rear rangers", "s4_screen", AQUA),
         ("sh3", "shield: side/rear", "s4_screen", ORANGE), ("sh4", "shield: + 1 front", "s4_screen", ORANGE),
         ("sh4u", "shield: + 1 front, unstick", "s4_screen", ORANGE),
         ("sh5u", "shield: ±25° pair + 3 (sh5u)", "s4_screen", ORANGE),
         ("shfu", "shield: 8×8 front + 3 (shfu)", "s4_screen", ORANGE),
         ("shou", "shield: 32-beam oracle", "s4_screen", ORANGE),
         ("shfu_c065", "shfu, cruise 0.65", "s4_screen", ORANGE),
         ("s5n", "S5: ToF → brain only", "s5_screen", VIOLET), ("s5", "S5: ToF → brain + sh5u", "s5_screen", VIOLET),
         ("s5f", "S5: ToF → brain + shfu", "s5_screen", VIOLET), ("s5f_g2", "s5f, near gain 2", "s5_screen", VIOLET),
         ("s5f_c065", "s5f, cruise 0.65", "s5_screen", VIOLET)]


def fig_shield(T):
    fig, axs = plt.subplots(1, 3, figsize=(15, 6.0), gridspec_kw={"wspace": 0.12, "width_ratios": [1.2, 1, 1]})
    base = {s: T[("s4_screen", "v4", "none", s)] for s in ("lowbox", "cage20")}
    log("== fig18 twin S4 / S5 screen (lowbox 150, cage20 32; paired with v4)")
    for i, (tag, lab, f, col) in enumerate(CONDS):
        L, C = T[(f, tag, "none", "lowbox")], T[(f, tag, "none", "cage20")]
        n, ka, kb, sv, nw, p = mcnemar(base["lowbox"], L)
        pw = wilson(kb, n)
        ci_bar(axs[0], i, 100 * pw[0], 100 * pw[1], 100 * pw[2], col)
        axs[0].text(36, i, f"{kb}/{n}", va="center", fontsize=8, color=INK2)
        m, lo, hi = boot_diff(base["lowbox"], L, "coverage")
        ci_bar(axs[1], i, m, lo, hi, col)
        sh = [r["shield_clamp_ticks"] / r["ticks"] for r in L.values() if not np.isnan(r["shield_clamp_ticks"])]
        if tag.startswith("sh") or tag in ("s5", "s5f", "s5f_g2", "s5f_c065"):
            ci_bar(axs[2], i, *(100 * np.array(_boot(sh))), col)
        log(f"  {tag:10s} lowbox {ka}->{kb}/{n} (McNemar p={p:.4f}) cage20 {runs_any(C)}/{len(C)} | Δcov {m:+.3f} "
            f"[{lo:+.3f},{hi:+.3f}] | shield-clamped ticks {100 * np.mean(sh) if sh else 0:.1f}%")
    for tag_a, tag_b, fa, fb in (("sh5u", "s5", "s4_screen", "s5_screen"), ("shfu", "s5f", "s4_screen", "s5_screen")):
        A, B = T[(fa, tag_a, "none", "lowbox")], T[(fb, tag_b, "none", "lowbox")]
        k = sorted(set(A) & set(B))
        a = np.array([A[x]["shield_clamp_ticks"] for x in k])
        b = np.array([B[x]["shield_clamp_ticks"] for x in k])
        log(f"  shield clamp ticks {tag_a} -> {tag_b}: mean {a.mean():.0f} -> {b.mean():.0f} "
            f"({100 * (b.mean() / a.mean() - 1):+.0f}%), Wilcoxon p={wilcoxon(b - a).pvalue:.5f}")
    axs[0].set_yticks(range(len(CONDS)), [c[1] for c in CONDS])
    for ax in axs[1:]:
        ax.set_yticks(range(len(CONDS)), [])
    for ax in axs:
        ax.set_ylim(len(CONDS) - 0.5, -0.5)
        ax.grid(axis="y", visible=False)
    axs[0].set_xlim(0, 44)
    axs[0].set_xlabel("% lowbox runs with contact (95% CI)")
    axs[0].set_title("Safety")
    axs[1].axvline(0, color=INK2, lw=0.8)
    axs[1].set_xlabel("Δ coverage vs v4 (paired, 95% CI)")
    axs[1].set_title("Cost in coverage")
    axs[2].set_xlabel("% of ticks the shield clamped the command")
    axs[2].set_title("Shield intervention")
    axs[2].set_xlim(0, None)
    h = [plt.Line2D([], [], marker="o", ls="", color=c, label=l) for c, l in
         ((BLUE, "v4"), (MUTED, "slower v4"), (AQUA, "rule-based"), (ORANGE, "range shield (S4)"),
          (VIOLET, "ToF into the brain (S5)"))]
    fig.legend(handles=h, loc="upper center", ncol=5, bbox_to_anchor=(0.55, 1.0))
    note(fig, "Fast twin, optical-flow navigation with per-episode randomised errors; lowbox_s0–49 × 3 brain seeds, "
              "paired with v4 on the same worlds and seeds. Shield = velocity-level CBF filter on single-zone ToF "
              "(27°, 10 Hz, 1 cm noise, d_safe 0.25 m).", y=-0.03)
    save(fig, "fig18_shield_screen.png")


def _boot(x, n=10000):
    x = np.asarray(x, float)
    m = x[RNG.integers(0, x.size, (n, x.size))].mean(axis=1)
    return x.mean(), np.percentile(m, 2.5), np.percentile(m, 97.5)


# ---------------------------------------------------------------------------------------------------------- fig 19
def fig_gazebo(G):
    arms = [("msh_sh5u", "sh5u, planar ToF", ORANGE), ("msh_shfu", "shfu, planar ToF", ORANGE),
            ("msh3_sh5u", "sh5u, 27° cone ToF", ORANGE), ("ms5_s5", "s5, 27° cone ToF", VIOLET)]
    fig, axs = plt.subplots(1, 3, figsize=(14, 3.3), gridspec_kw={"wspace": 0.15, "width_ratios": [1.45, 1, 1]})
    log("== fig19 Gazebo (ROG) acceptance, paired with v4 (mc1_v4 lowbox_s0-49, md_v4 cage20 k0-7)")
    v4l, v4c = G[("mc1_v4", "lowbox")], G[("md_v4", "cage20")]
    v4c = {k: r for k, r in v4c.items() if k[1] < 8}
    rows = [("v4", "v4", BLUE)] + arms
    for i, (g, lab, col) in enumerate(rows):
        L = v4l if g == "v4" else G[(g, "lowbox")]
        C = v4c if g == "v4" else G[(g, "cage20")]
        n, ka, kb, sv, nw, p = mcnemar(v4l, L)
        pw = wilson(kb, n)
        ci_bar(axs[0], i, 100 * pw[0], 100 * pw[1], 100 * pw[2], col)
        txt = f"{kb}/{n}" if g == "v4" else f"{kb}/{n}  p = {p:.3f}"
        axs[0].text(37, i, txt, va="center", fontsize=8, color=INK2)
        if g != "v4":
            m, lo, hi = boot_diff(v4l, L, "coverage")
            ci_bar(axs[1], i, m, lo, hi, col)
            cl = [float(r["shield_clamp_ticks"]) for r in L.values()]
            ci_bar(axs[2], i, *_boot(cl), col)
            c100 = 100 * sum(r["contact_episodes"] for r in L.values()) / sum(r["path_m"] for r in L.values())
            log(f"  {g:10s} lowbox {ka}->{kb}/{n} (saved {sv}, new {nw}, McNemar p={p:.4f}) contacts/100 m {c100:.2f} | "
                f"cage20 {runs_any(C)}/{len(C)} | Δcov {m:+.3f} [{lo:+.3f},{hi:+.3f}] | clamp ticks {np.mean(cl):.0f}")
        else:
            c100 = 100 * sum(r["contact_episodes"] for r in L.values()) / sum(r["path_m"] for r in L.values())
            log(f"  v4         lowbox {kb}/{n} contacts/100 m {c100:.2f} | cage20 {runs_any(C)}/{len(C)}")
    A, B = G[("msh3_sh5u", "lowbox")], G[("ms5_s5", "lowbox")]
    k = sorted(set(A) & set(B))
    a = np.array([float(A[x]["shield_clamp_ticks"]) for x in k])
    b = np.array([float(B[x]["shield_clamp_ticks"]) for x in k])
    log(f"  clamp ticks sh5u(cone) -> s5: {a.mean():.0f} -> {b.mean():.0f} ({100 * (b.mean() / a.mean() - 1):+.0f}%), "
        f"Wilcoxon p={wilcoxon(b - a).pvalue:.4f}")
    allc = sum(runs_any(G[(g, s)]) for g in ("msh3_sh5u",) for s in ("lowbox", "cage20"))
    log(f"  msh3_sh5u pooled {allc}/58, one-sided 95% upper bound {binomtest(allc, 58).proportion_ci(0.90, 'exact').high:.3f}")
    mb = G.get(("mb_v4", "lowbox"), {})
    if mb:
        log(f"  v4 second brain seed (mb_v4, k+100): lowbox {runs_any(mb)}/{len(mb)}, cage20 "
            f"{runs_any(G[('mb_v4', 'cage20')])}/{len(G[('mb_v4', 'cage20')])} (s5 on the same set: in progress)")
    for ax in axs:
        ax.set_ylim(len(rows) - 0.5, -0.5)
        ax.grid(axis="y", visible=False)
    axs[0].set_yticks(range(len(rows)), [r[1] for r in rows])
    for ax in axs[1:]:
        ax.set_yticks(range(len(rows)), [])
    axs[0].set_xlim(0, 58)
    axs[0].set_xlabel("% lowbox runs with contact (95% CI)")
    axs[0].set_title("Gazebo lowbox_s0–49 (McNemar vs v4)")
    axs[1].axvline(0, color=INK2, lw=0.8)
    axs[1].set_xlabel("Δ coverage vs v4 (95% CI)")
    axs[1].set_title("Coverage")
    axs[2].set_xlabel("shield-clamped ticks per run (of 2,396)")
    axs[2].set_title("Shield intervention")
    note(fig, "ROG, Gazebo 8.15 + ArduPilot SITL, optical flow + ToF + route B, same worlds and brain seeds as v4. "
              "ToF emulated from SITL ground truth and the world layout: planar fan, or the full 27° cone "
              "(axis + 8 rim rays, attitude, floor).", y=-0.08)
    save(fig, "fig19_gazebo_shield.png")


# ---------------------------------------------------------------------------------------------------------- fig 20
E_SET = [("e1", "e1: slow-down only, all teaching signals"), ("e2", "e2: + turn drive (k_sac 30)"),
         ("e3", "e3: turn drive, contacts / grazes only"), ("e4", "e4: slow-down, contacts / grazes only"),
         ("e5", "e5: + side/rear rangers as KC input, contacts only"), ("e6", "e6: + rangers, all teaching signals")]


def fig_mb(T0):
    T = {}
    for r in csv.DictReader(gzip.open(RL / "twin" / "mb" / "tests.csv.gz", "rt")):
        num(r, ("contact_episodes", "coverage", "path_m"))
        T.setdefault((r["file"], r["mbarm"], r["set"], int(r["life"])), []).append(r)
    v4 = {s: runs_any({k: r for k, r in T0[("seeds", "v4", "none", s)].items() if s == "cage20" or k[1].endswith("_0")})
          for s in ("lowbox", "cage20")}  # the k_0 test subset (brain seed k), as in rl.mb_life --test
    fig, axs = plt.subplots(1, 3, figsize=(15, 3.6), gridspec_kw={"wspace": 0.3})
    log("== fig20 stage E: frozen-g tests, learned vs KC-shuffled g (8 lifetimes per setting)")
    for j, (sset, n) in enumerate((("lowbox", 50), ("cage20", 32))):
        ax = axs[j]
        for i, (e, lab) in enumerate(E_SET):
            a = [sum(r["contact_episodes"] > 0 for r in T[(e, "learned", sset, L)]) for L in range(8)]
            b = [sum(r["contact_episodes"] > 0 for r in T[(e, "shuffled", sset, L)]) for L in range(8)]
            for x, y in zip(a, b):
                ax.plot([i - 0.15, i + 0.15], [x, y], color=MUTED, lw=0.8, alpha=0.6)
            ax.scatter(i - 0.15 + RNG.uniform(-0.03, 0.03, 8), a, s=20, color=VIOLET, edgecolors=SURF, linewidths=1,
                       zorder=3)
            ax.scatter(i + 0.15 + RNG.uniform(-0.03, 0.03, 8), b, s=20, color=MUTED, edgecolors=SURF, linewidths=1,
                       zorder=3)
            d = np.array(b) - np.array(a)
            p = wilcoxon(d).pvalue if np.any(d != 0) else 1.0
            cov = np.mean([r["coverage"] for L in range(8) for r in T[(e, "learned", sset, L)]])
            log(f"  {sset:6s} {e}: learned median {np.median(a):.1f} shuffled {np.median(b):.1f} (Wilcoxon p={p:.2f}) "
                f"v4 {v4[sset]} | learned coverage {cov:.3f}")
        ax.axhline(v4[sset], color=BLUE, lw=1.2, ls=(0, (4, 3)))
        ax.text(len(E_SET) - 0.55, v4[sset], "v4", color=BLUE, fontsize=8, va="bottom")
        ax.set_xticks(range(len(E_SET)), [x[0] for x in E_SET])
        ax.set_ylabel(f"runs with contact / {n}")
        ax.set_title(f"Twin {sset}: learned vs shuffled g")
        ax.grid(axis="x", visible=False)
    ax = axs[2]
    L = [json.loads(line) for line in open(RL / "twin" / "mb" / "lives.jsonl")]
    log("== fig20 contacts in the first vs second half of each training lifetime")
    for i, (e, lab) in enumerate(E_SET):
        fh = [sum(x["contact_episodes"] > 0 for x in d["log"][:15]) for d in L if d["tag"] == e]
        sh = [sum(x["contact_episodes"] > 0 for x in d["log"][15:]) for d in L if d["tag"] == e]
        for x, y in zip(fh, sh):
            ax.plot([i - 0.15, i + 0.15], [x, y], color=MUTED, lw=0.8, alpha=0.6)
        ax.scatter([i - 0.15] * len(fh), fh, s=18, color=YELLOW, edgecolors=SURF, linewidths=1, zorder=3)
        ax.scatter([i + 0.15] * len(sh), sh, s=18, color=ORANGE, edgecolors=SURF, linewidths=1, zorder=3)
        d = np.array(sh) - np.array(fh)
        p = wilcoxon(d).pvalue if np.any(d != 0) else 1.0
        log(f"  {e}: episodes with contact, first 15 {np.mean(fh):.2f} -> last 15 {np.mean(sh):.2f} (p={p:.2f}), "
            f"final g mean {np.mean([np.mean(d['g']) for d in L if d['tag'] == e]):.2f}")
    ax.set_xticks(range(len(E_SET)), [x[0] for x in E_SET])
    ax.set_ylabel("training episodes with contact (of 15)")
    ax.set_title("Training: first vs second half")
    ax.grid(axis="x", visible=False)
    note(fig, "Each pair of dots is one lifetime (30 procedural episodes of 90 s, g carried over). Tests freeze g and "
              "run the k_0 test set (lowbox_s0–49, cage20 ×32);\nthe shuffled control permutes g across Kenyon cells, keeping "
              "the same mean slow-down.\nLeft dots: learned g (violet) / first 15 episodes (yellow); right dots: "
              "shuffled g (grey) / last 15 episodes (orange).\n" + " · ".join(x[1] for x in E_SET) + ".", y=-0.2)
    save(fig, "fig20_mb_stage_e.png")


# ---------------------------------------------------------------------------------------------------------- fig 21
def fig_n3():
    d = RL / "twin" / "hw" / "n3s5_s1"
    V = [json.loads(line) for line in open(d / "val.jsonl")]
    Gn = [json.loads(line) for line in open(d / "gens.jsonl")]
    ref = V[0]["v3_2"]  # the log calls the reference "v3_2"; here it is the v4 base with the s5 shield (x0)
    fig, axs = plt.subplots(1, 3, figsize=(13, 3.2), gridspec_kw={"wspace": 0.35})
    log("== fig21 N3 (Mac mini, n3s5_s1: s5 shield in the loop, fitness v5, space v4hw_sh)")
    for ax, key, lab in ((axs[0], "F", "hold-out fitness F"), (axs[1], "coverage", "hold-out coverage"),
                         (axs[2], "interv", "intervention share")):
        ax.plot([v["gen"] for v in V], [v["mean"][key] for v in V], color=VIOLET, marker="o", ms=4, lw=1.8,
                label="CMA mean")
        ax.axhline(ref[key], color=BLUE, lw=1.2, ls=(0, (4, 3)), label="start (v4 + s5)")
        ax.set_xlabel("generation")
        ax.set_title(lab)
    axs[0].legend(loc="center right", fontsize=8)
    for v in V:
        m = v["mean"]
        log(f"  gen {v['gen']:2d}: F {m['F']:+.3f} contacts {m['contacts']} coverage {m['coverage']:.3f} path {m['path']:.1f} m "
            f"interv {m['interv']:.3f} | start F {ref['F']:+.3f} coverage {ref['coverage']:.3f} path {ref['path']:.1f} m "
            f"interv {ref['interv']:.3f}")
    log(f"  {len(Gn)} generations")
    note(fig, "Hold-out: 24 fixed episodes (half lowbox) every 5 generations. Fitness v5 = v4 fitness − 0.3 × share of "
              "shield-clamped ticks; search space v4hw + cruise speed + shield d_safe / α.", y=-0.1)
    save(fig, "fig21_n3_cma.png")


def ood_struct(T):
    log("== S2 / S3 under the stage-D OOD modes (twin, runs with contact; lowbox /50, cage20 /32)")
    for sset in ("lowbox", "cage20"):
        for m in ("lowlight", "notex", "wind", "ghost"):
            row = []
            for tag in ("v4", "s2_mem", "s3_rng", "s23"):
                R = T.get(("ood_struct", tag, m, sset))
                row.append(f"{tag} {runs_any(R)}" if R else f"{tag} -")
            log(f"  {sset:6s} {m:8s} " + "  ".join(row))


def gazebo_s2(G):
    log("== desktop Gazebo S2 re-check (lowbox, paired with st_v4)")
    for g in ("st_s2_mem", "st_s2_memw"):
        n, ka, kb, sv, nw, p = mcnemar(G[("st_v4", "lowbox")], G[(g, "lowbox")])
        m, lo, hi = boot_diff(G[("st_v4", "lowbox")], G[(g, "lowbox")], "coverage")
        log(f"  {g}: {ka} -> {kb}/{n} (McNemar p={p:.2f}), Δcov {m:+.3f} [{lo:+.3f},{hi:+.3f}]")


if __name__ == "__main__":
    from make_figures_hw import twin_tests
    T = twin(RL / "twin" / "n" / "tests.csv.gz")
    G = gz_runs()
    fig_diag()
    fig_shield(T)
    fig_gazebo(G)
    fig_mb(twin_tests())
    fig_n3()
    ood_struct(T)
    gazebo_s2(G)
    (HERE / "n_stats.txt").write_text("\n".join(OUT_TXT) + "\n")
