#!/usr/bin/env python3
"""make_figures.py -- figures for the V8 directory (real HeightGuard validation).

Data sources (read next to this script):
  jobs_v8.txt         one planned SITL run per line: tag|parm|speedup|yaw|overlay|args
  mtf01_sitl_rng.parm base parameter file (FENCE_ALT_MAX is cross-checked against the job overlay)
  results.csv         one row per run (collect.py): metrics.json scalars + guard summary
  guard_status.csv    every ~1 Hz status line the guard printed (log clock, see below)

fig1 is the plan (job list only).  fig2..fig6 are measured results.  The per-condition
table printed by `python make_figures.py` holds every number the README quotes.

Clocks.  results.csv "s0", "s1" and "f_t0" are on the flight controller's simulated
clock: s0 = arm time of the AUTO flight, s1 = disarm time, f_t0 = first change of
SIM_SONAR_POS_Z minus s0.  The guard log (guard_first_trip_log_t, guard_disarm_log_t,
guard_status.csv "t") is seconds since hg_loop.py started (wall clock; speedup is 1 in
all 65 jobs).  Both clocks see the same disarm event, so guard_disarm_log_t - s1 is the
offset between them.  Runs whose two disarm times agree to 1 s are used to estimate it
(the others have no guard disarm line, or a different one); the median offset is applied
to all runs.  Guard delay = guard_first_trip_log_t - OFFSET - (s0 + f_t0).  The guard
prints a status line about once per second, so the delay is an upper bound with up to
1 s granularity; it counts from the first parameter change of the fault ladder, not
from the moment the reading was wrong by more than the guard threshold.

Run:  python make_figures.py      (writes figures/fig*.png)
"""
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

OKABE = ["#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9", "#F0E442", "#000000"]
plt.rcParams.update({"figure.dpi": 150, "savefig.dpi": 150, "savefig.bbox": "tight",
    "font.size": 10, "axes.titlesize": 11, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.alpha": 0.3, "axes.prop_cycle": matplotlib.cycler(color=OKABE),
    "figure.facecolor": "white"})

HERE = Path(__file__).resolve().parent
FIG = HERE / "figures"
FIG.mkdir(exist_ok=True)

# display name of every condition found in the job list, in the order used in the figures
LABELS = {
    "clean": "clean (no fault)",
    "a": "a", "c": "c", "g5": "g5", "gN": "gN", "h": "h", "i": "i", "j2": "j2", "k": "k",
    "d3_gN": "D3 gN", "d3_h": "D3 h", "d3_k": "D3 k",
}
SQ_ORDER = ["clean", "a", "c", "g5", "gN", "h", "i", "j2", "k"]
D3_ORDER = ["d3_gN", "d3_h", "d3_k"]


def parse_jobs(path):
    """Return {condition: [run dicts]} from a job list file."""
    runs = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        tag, parm, spd, yaw, ovl, args = line.split("|", 5)
        faults = [(float(t), float(v)) for t, v in
                  re.findall(r"--fault ([0-9.]+):SIM_SONAR_POS_Z=([0-9.\-]+)", args)]
        wind = re.search(r"SIM_WIND_DIR=(\d+)", ovl)
        runs.append({
            "tag": tag,
            "cond": re.sub(r"_hg$", "", re.sub(r"^v8_", "", re.sub(r"_r\d+$", "", tag))),
            "mission": args.split()[0],
            "yaw": int(yaw),
            "wind": int(wind.group(1)) if wind else None,
            "faults": faults,
        })
    out = {}
    for r in runs:
        out.setdefault(r["cond"], []).append(r)
    return out


def ladder(cond_runs):
    """Fault schedule of a condition: [(t, z)] (identical in every run of the condition)."""
    return cond_runs[0]["faults"]


def fig2_fault_schedule(conds):
    """Fig 2: injected rangefinder reading (SIM_SONAR_POS_Z) against mission time."""
    order = [c for c in SQ_ORDER if c in conds] + [c for c in D3_ORDER if c in conds]
    fig, ax = plt.subplots(figsize=(8.6, 5.0))
    color_of = {}
    for i, c in enumerate(order):
        f = ladder(conds[c])
        if not f:
            continue
        sq = c in SQ_ORDER
        key = tuple(f)
        if key not in color_of:
            color_of[key] = OKABE[len(color_of) % len(OKABE)]
        color = color_of[key]
        t = [p[0] for p in f] + [f[-1][0] + 3.0]
        z = [p[1] for p in f] + [f[-1][1]]
        ax.step(t, z, where="post", color=color, lw=0.9 if len(f) > 20 else 1.8,
                ls="-" if sq else "--",
                alpha=0.45 if len(f) > 20 else (1.0 if sq else 0.8),
                label=LABELS.get(c, c) if sq else None,
                zorder=1 if len(f) > 20 else (3 if sq else 2))
    ax.axhline(0.0, color="0.55", lw=1.0, ls="--", zorder=1)
    ax.set_xlim(0, 64)
    ax.set_xlabel("Mission time of the injection (s)")
    ax.set_ylabel("SIM_SONAR_POS_Z set by --fault (m)")
    ax.set_title("SIM_SONAR_POS_Z schedule per condition (from the job list, no measurements)")
    handles, labels = ax.get_legend_handles_labels()
    handles.append(plt.Line2D([], [], color="0.4", lw=1.8, ls="--"))
    labels.append("D3 conditions (same schedules as SQ)")
    ax.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=4,
              frameon=False, fontsize=8)
    fig.savefig(FIG / "fig1_fault_schedule.png")
    plt.close(fig)


# ---------------------------------------------------------------- measured results
GREEN, ORANGE = "#009E73", "#D55E00"
ORDER = SQ_ORDER + D3_ORDER
PLAN = {}


def load_results():
    """results.csv + guard_status.csv + plan constants; adds ok flag, clock offset, guard delay."""
    res = pd.read_csv(HERE / "results.csv")
    st = pd.read_csv(HERE / "guard_status.csv")
    jobs = (HERE / "jobs_v8.txt").read_text()
    alts = set(re.findall(r"--alt ([0-9.]+)", jobs))
    fences = set(re.findall(r"FENCE_ALT_MAX=([0-9.]+)", jobs))
    parm = dict(ln.split()[:2] for ln in (HERE / "mtf01_sitl_rng.parm").read_text().splitlines()
                if ln.strip() and not ln.startswith("#"))
    assert len(alts) == 1 and len(fences) == 1 and float(parm["FENCE_ALT_MAX"]) == float(next(iter(fences)))
    plan = {"alt": float(next(iter(alts))), "fence": float(next(iter(fences)))}
    assert res.guard_cfg.nunique() == 1
    cfg = json.loads(res.guard_cfg.iloc[0])
    plan["e_m"], plan["t_s"] = cfg["e_m"], cfg["t_s"]
    res["fault"] = res.cond != "clean"
    # green: clean run that completed the mission, or fault run where the guard tripped
    res["ok"] = np.where(res.fault, res.guard_tripped == 1, res.mission_complete)
    off = res.guard_disarm_log_t - res.s1
    good = off[off.abs() < 1.0]
    plan.update(offset=float(good.median()), offset_n=len(good), offset_min=float(good.min()),
                offset_max=float(good.max()))
    res["trip_delay"] = res.guard_first_trip_log_t - plan["offset"] - (res.s0 + res.f_t0)
    st = st.merge(res[["tag", "s0", "f_t0"]], on="tag")
    st["t_fault"] = st.t - plan["offset"] - (st.s0 + st.f_t0.fillna(0))  # clean runs: time since arm
    return res, st, plan


def per_condition(res):
    rows = []
    for c in ORDER:
        d = res[res.cond == c]
        rows.append({"cond": c, "runs": len(d), "trips": int(d.guard_tripped.sum()),
                     "completed": int(d.mission_complete.sum()),
                     "max_med": d.truth_max_all.median(), "max_worst": d.truth_max_all.max(),
                     "over_fence": int((d.truth_max_all > PLAN["fence"]).sum()),
                     "td_med": d.touchdown_mps.median(), "td_max": d.touchdown_mps.max(),
                     "delay_med": d.trip_delay.median(), "delay_max": d.trip_delay.max(),
                     "disarmed": int(d.disarmed.sum())})
    return pd.DataFrame(rows).set_index("cond")


def strip(ax, res, col, ylabel, hlines=()):
    """One column of single runs per condition (green / orange by outcome) and the median bar."""
    rng = np.random.default_rng(3)
    for i, c in enumerate(ORDER):
        d = res[res.cond == c]
        x = i + rng.uniform(-0.22, 0.22, len(d))
        for ok, mk, co in ((True, "o", GREEN), (False, "D", ORANGE)):
            m = (d.ok == ok).values
            ax.scatter(x[m], d[col][m], marker=mk, s=26 if ok else 40, color=co, edgecolor="white",
                       lw=0.5, zorder=3)
        v = d[col].dropna()
        if len(v):
            ax.hlines(v.median(), i - 0.35, i + 0.35, color="black", lw=1.6, zorder=4)
    for y, ls in hlines:
        ax.axhline(y, color="0.35", lw=1.0, ls=ls, zorder=2)
    ax.set_xticks(range(len(ORDER)))
    ax.set_xticklabels([f"{LABELS[c]}\n(n={int((res.cond == c).sum())})" for c in ORDER], fontsize=8)
    ax.set_ylabel(ylabel)
    ax.set_xlim(-0.6, len(ORDER) - 0.4)
    ax.axvline(len(SQ_ORDER) - 0.5, color="0.7", lw=0.8)


def legend_runs(ax, extra=(), **kw):
    h = [plt.Line2D([], [], marker="o", ls="", color=GREEN, ms=6,
                    label="clean: mission completed / fault: guard tripped"),
         plt.Line2D([], [], marker="D", ls="", color=ORANGE, ms=6, label="fault run, guard did not trip"),
         plt.Line2D([], [], color="black", lw=1.6, label="median of the condition")]
    ax.legend(handles=h + list(extra), frameon=False, fontsize=8, **kw)


def fig2_trips(tab):
    """Fig 2: guard trips per condition (counts)."""
    fig, ax = plt.subplots(figsize=(8.6, 4.8))
    y = np.arange(len(ORDER))[::-1]
    for yi, c in zip(y, ORDER):
        r = tab.loc[c]
        if c == "clean":  # no trip is the wanted outcome here
            ax.barh(yi, r.runs - r.trips, color=GREEN, height=0.6, hatch="//", edgecolor="white",
                    label="clean run, guard did not trip (wanted)")
        ax.barh(yi, r.trips, color=GREEN, height=0.6, label="fault run, guard tripped" if c == "a" else None)
        if c != "clean":
            ax.barh(yi, r.runs - r.trips, left=r.trips, color=ORANGE, height=0.6,
                    label="fault run, guard did not trip" if c == "a" else None)
        ax.text(r.runs + 0.15, yi, f"{int(r.trips)} / {int(r.runs)}", va="center", fontsize=9)
    ax.set_yticks(y)
    ax.set_yticklabels([LABELS[c] for c in ORDER])
    ax.set_xlabel("Runs (count; label = tripped / runs)")
    ax.set_xlim(0, 11.5)
    ax.set_title("Guard trips per condition")
    ax.grid(axis="y", alpha=0)
    ax.legend(loc="lower right", frameon=False, fontsize=8)
    fig.savefig(FIG / "fig2_guard_trips.png")
    plt.close(fig)


def fig3_height(res):
    """Fig 3: true maximum height of the whole flight per run."""
    fig, ax = plt.subplots(figsize=(9.2, 5.0))
    strip(ax, res, "truth_max_all", "Maximum true height above ground, whole flight (m)",
          hlines=[(PLAN["alt"], "--"), (PLAN["fence"], ":")])
    ex = [plt.Line2D([], [], color="0.35", ls="--", label=f"mission altitude {PLAN['alt']:g} m"),
          plt.Line2D([], [], color="0.35", ls=":", label=f"fence FENCE_ALT_MAX {PLAN['fence']:g} m")]
    ax.set_ylim(1.7, 4.5)
    legend_runs(ax, ex, loc="upper right")
    ax.set_title("True maximum height per run (truth_max_all)")
    fig.savefig(FIG / "fig3_true_max_height.png")
    plt.close(fig)


def fig4_delay(res):
    """Fig 4: guard trip delay (upper bound, 1 s granularity) after the first fault change."""
    fig, ax = plt.subplots(figsize=(9.2, 4.8))
    strip(ax, res, "trip_delay", "Guard trip line minus first fault change (s)")
    ax.set_xlim(0.5, len(ORDER) - 0.4)
    ax.set_ylim(0, 16)
    n_no = int(((res.guard_tripped == 0) & res.fault).sum())
    ex = [plt.Line2D([], [], ls="", label=f"{n_no} fault runs without trip: no point (no delay)")]
    legend_runs(ax, ex, loc="upper left")
    ax.set_title("Guard delay after the first SIM_SONAR_POS_Z change (upper bound, 1 s status lines)")
    fig.savefig(FIG / "fig4_guard_delay.png")
    plt.close(fig)


def fig5_touchdown(res):
    fig, ax = plt.subplots(figsize=(9.2, 4.8))
    strip(ax, res, "touchdown_mps", "Touchdown speed, max 'SIM Hit ground' (m/s)")
    ax.set_ylim(0, 1.3)
    legend_runs(ax, loc="upper left", ncol=1)
    ax.set_title("Touchdown speed per run (touchdown_mps)")
    fig.savefig(FIG / "fig5_touchdown_speed.png")
    plt.close(fig)


def fig6_e_traces(res, st):
    """Fig 6: guard level difference e over time, one example run (rep 1) per condition.
    The fault runs where the guard did not trip are drawn too (orange dashed)."""
    fig, axes = plt.subplots(4, 3, figsize=(10.5, 10.0), sharey=True)
    for ax, c in zip(axes.flat, ORDER):
        d = st[st.cond == c]
        ex_tag = res[(res.cond == c) & (res.rep == 1)].tag.iloc[0]
        for tag in res[(res.cond == c) & (res.guard_tripped == 0) & res.fault].tag:
            if tag != ex_tag:
                x = d[d.tag == tag]
                ax.plot(x.t_fault, x.e, color=ORANGE, lw=1.0, ls="--",
                        label=tag.replace("v8_", "").replace("_hg", "") + ": not tripped")
        x = d[d.tag == ex_tag]
        r = res[res.tag == ex_tag].iloc[0]
        co = GREEN if r.ok else ORANGE
        ax.plot(x.t_fault, x.e, color=co, lw=1.4,
                label=ex_tag.replace("v8_", "").replace("_hg", "") + (" (example)" if r.ok else " (example, not tripped)"))
        if r.guard_tripped:
            ax.axvline(r.trip_delay, color=co, lw=1.0, ls=":", label="first trip line of the example")
        for s in (1, -1):
            ax.axhline(s * PLAN["e_m"], color="0.35", lw=0.9, ls="--",
                       label=f"±e_m = {PLAN['e_m']:g} m" if s == 1 else None)
        ax.axvline(0, color="0.6", lw=0.8)
        ax.set_title(LABELS[c] + ("  (t = 0: arm)" if c == "clean" else ""), fontsize=10)
        ax.set_xlim(-10, 60)
        ax.set_ylim(-1.4, 2.9)
        ax.legend(frameon=False, fontsize=6.5, loc="upper right")
    for ax in axes[-1]:
        ax.set_xlabel("Time since first fault change (s)")
    for ax in axes[:, 0]:
        ax.set_ylabel("Guard level diff e (m)")
    fig.suptitle("Guard level difference e, one example run (rep 1) per condition", y=0.995)
    fig.tight_layout()
    fig.savefig(FIG / "fig6_guard_e_traces.png")
    plt.close(fig)


def print_table(res, tab, st):
    pd.set_option("display.width", 250)
    pd.set_option("display.max_columns", 30)
    print("plan:", PLAN)
    print(tab.round(3).to_string())
    print("README table rows (cond | runs | trips | completed | median max m | worst max m | median touchdown m/s):")
    for c, r in tab.iterrows():
        print(f"| `{c}` | {int(r.runs)} | {int(r.trips)} | {int(r.completed)} | {r.max_med:.2f} | {r.max_worst:.2f} "
              f"| {r.td_med:.2f} |")
    print("a and i, every run: tripped, max e in AUTO mode before any trip line (1 Hz lines), "
          "t_fault of that max, true max, mode at LAND start")
    for tag in res[res.cond.isin(["a", "i"])].tag:
        x = st[(st.tag == tag) & (st["mode"] == "AUTO") & (st.tripped == 0)]
        r = res[res.tag == tag].iloc[0]
        print("  %-12s tripped %d  max e %.3f at %5.1f s  truth_max %.2f  f_dur %.1f  last AUTO line %.1f s" %
              (tag, r.guard_tripped, x.e.max(), x.t_fault[x.e.idxmax()], r.truth_max_all, r.f_dur,
               x.t_fault.max()))
    f = res[res.fault]
    print("fault runs", len(f), "tripped", int(f.guard_tripped.sum()), "| clean runs", int((~res.fault).sum()),
          "tripped", int(res[~res.fault].guard_tripped.sum()))
    print("guard reasons of tripped runs:", res[res.guard_tripped == 1].guard_reason.value_counts().to_dict())
    print("clean: max |e| over runs (status lines) %.3f, mission_complete %d" %
          (res[~res.fault].guard_e_abs_max.max(), res[~res.fault].mission_complete.sum()))
    cols = ["tag", "mission_complete", "final_mode", "disarmed", "left_auto_at", "flight_s", "f_t0", "truth_max_all",
            "f_true_max", "touchdown_mps", "guard_e_abs_max"]
    nt = res[res.fault & (res.guard_tripped == 0)]
    print("fault runs without trip:\n", nt[cols].to_string())
    for tag in nt.tag:
        x = st[st.tag == tag]
        print(" ", tag, "status lines", len(x), "| max e %.3f at %.1f s after fault" % (x.e.max(), x.t_fault[x.e.idxmax()]),
              "| min e %.3f" % x.e.min())
    tr = res[res.fault & (res.guard_tripped == 1)]
    print("tripped runs: median left_auto_at %.1f, median flight_s %.1f" % (tr.left_auto_at.median(), tr.flight_s.median()))
    print("runs never disarmed:", res[~res.disarmed.astype(bool)].tag.tolist())
    print("clock offset guard_disarm_log_t - s1: n=%d min %.3f max %.3f median %.3f" %
          (PLAN["offset_n"], PLAN["offset_min"], PLAN["offset_max"], PLAN["offset"]))
    print("trip delay (s) over tripped runs: min %.2f median %.2f max %.2f" %
          (tr.trip_delay.min(), tr.trip_delay.median(), tr.trip_delay.max()))
    print("fault runs above fence:", int((f.truth_max_all > PLAN["fence"]).sum()),
          "| tripped runs above fence:", int((tr.truth_max_all > PLAN["fence"]).sum()))


def main():
    global PLAN
    conds = parse_jobs(HERE / "jobs_v8.txt")
    fig2_fault_schedule(conds)
    res, st, PLAN = load_results()
    tab = per_condition(res)
    fig2_trips(tab)
    fig3_height(res)
    fig4_delay(res)
    fig5_touchdown(res)
    fig6_e_traces(res, st)
    print_table(res, tab, st)
    print("figures written to", FIG)
    for p in sorted(FIG.glob("*.png")):
        print(" ", p.name, p.stat().st_size // 1024, "kB")


if __name__ == "__main__":
    main()
