#!/usr/bin/env python3
"""Tables + figures for the Orange Pi profiling report from /opt/ecps295/logs/*_bench_P* (copied to the Mac).

    python analyze_profile.py <logs_dir> <out_dir>
Writes profile_table.md (per-run summary), stage_table.md (per-stage p50/p95/p99 for P1 / P5), and figures:
fig_stage_breakdown.png, fig_tick_cdf.png, fig_thermal.png.
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

L, O = Path(sys.argv[1]), Path(sys.argv[2])
O.mkdir(parents=True, exist_ok=True)
runs = {}
for d in sorted(L.glob("*_bench_P*")):
    tag = d.name.split("_bench_", 1)[1]
    if not (d / "run.json").exists():
        continue
    runs[tag] = {"dir": d, "sum": json.load(open(d / "run.json"))["summary"],
                 "ticks": list(csv.DictReader(open(d / "ticks.csv"))),
                 "sys": list(csv.DictReader(open(d / "system.csv")))}

DESC = {
    "P1_a76": "A76 ×2 (cpu6-7), schedutil",
    "P2_a55": "A55 ×6 (cpu0-5), schedutil",
    "P3_any": "no pinning, schedutil",
    "P4_a55one": "one A55 (cpu0)",
    "P5_a76_perf": "A76 ×2, performance governor",
    "P6_a76_bgload": "A76 ×2, 6 A55 cores 100 % busy",
    "P7_any_bgload": "no pinning, 6 A55 cores busy",
    "P8_a76_thermal15": "A76 ×2, 15 min thermal soak",
}
rows = ["| Run | Setting | Ticks | Deadline miss (>50 ms) | compute p50 | p95 | p99 | max | period p99 | max T_cpub °C | A76 MHz p50 |",
        "|---|---|---|---|---|---|---|---|---|---|---|"]
for tag, r in runs.items():
    s = r["sum"]
    c, p = s["compute_ms"], s["period_ms"]
    tmax = max(float(x.get("T_cpub", 0) or 0) for x in r["sys"]) if r["sys"] else float("nan")
    mhz = np.median([float(x.get("MHz_cpu6", 0) or 0) for x in r["sys"]]) if r["sys"] else float("nan")
    rows.append(f"| {tag} | {DESC.get(tag, '')} | {s['ticks']} | {s['deadline_miss']} ({100 * s['deadline_miss'] / max(1, s['ticks']):.1f} %) |"
                f" {c['p50']:.1f} | {c['p95']:.1f} | {c['p99']:.1f} | {c['p100']:.1f} | {p.get('p99', float('nan')):.1f} | {tmax:.1f} | {mhz:.0f} |")
(O / "profile_table.md").write_text("\n".join(rows) + "\n")

ST = ["retina", "encoder", "brain", "decoder", "governor", "map", "struct", "frame", "telemetry", "send"]
lines = ["| Stage | " + " | ".join(f"{t} p50 / p95 / p99" for t in ("P1_a76", "P5_a76_perf", "P2_a55") if t in runs) + " |",
         "|---|" + "---|" * len([t for t in ("P1_a76", "P5_a76_perf", "P2_a55") if t in runs])]
for k in ST:
    cells = []
    for t in ("P1_a76", "P5_a76_perf", "P2_a55"):
        if t not in runs:
            continue
        v = runs[t]["sum"]["stage_ms"].get(k)
        cells.append("—" if not v else f"{v['p50']:.2f} / {v['p95']:.2f} / {v['p99']:.2f}")
    lines.append(f"| {k} | " + " | ".join(cells) + " |")
extra = []
for t in ("P1_a76", "P5_a76_perf"):
    if t in runs:
        s = runs[t]["sum"]
        extra.append(f"- {t}: camera decode+gray+resize (own thread) p50 {s['camera_decode_ms']['p50']:.2f} ms, p95 {s['camera_decode_ms']['p95']:.2f} ms;"
                     f" map frames {s['struct'].get('shield_map_frames')}, RSS max {max(float(x.get('rss_mb', 0)) for x in runs[t]['sys']):.0f} MB")
(O / "stage_table.md").write_text("\n".join(lines) + "\n\n" + "\n".join(extra) + "\n")

# figures
if "P1_a76" in runs:
    fig, ax = plt.subplots(figsize=(7, 3.2))
    order = [t for t in ("P5_a76_perf", "P1_a76", "P6_a76_bgload") if t in runs]
    left = np.zeros(len(order))
    for k, col in zip(["retina", "encoder", "brain", "decoder", "map", "other"],
                      ["#4a6fa5", "#9db4d6", "#d9763b", "#f2c09b", "#4c9a45", "#bbbbbb"]):
        vals = []
        for t in order:
            sm = runs[t]["sum"]["stage_ms"]
            if k == "other":
                tot = runs[t]["sum"]["compute_ms"]["mean"]
                vals.append(max(0.0, tot - sum(sm[x]["mean"] for x in ("retina", "encoder", "brain", "decoder", "map") if x in sm)))
            else:
                vals.append(sm.get(k, {"mean": 0.0})["mean"])
        ax.barh(order, vals, left=left, color=col, label=k)
        left += np.array(vals)
    ax.axvline(50, color="k", ls="--", lw=1)
    ax.text(50.5, -0.45, "50 ms (20 Hz)", fontsize=8)
    ax.set_xlabel("mean time per control tick [ms]")
    ax.legend(ncol=6, fontsize=7, loc="upper center", bbox_to_anchor=(0.5, 1.22), frameon=False)
    fig.tight_layout()
    fig.savefig(O / "fig_stage_breakdown.png", dpi=150)

    fig, ax = plt.subplots(figsize=(6, 3.2))
    for t in runs:
        v = np.sort([float(x["compute_ms"]) for x in runs[t]["ticks"]])
        if len(v):
            ax.plot(v, np.linspace(0, 1, len(v)), label=t)
    ax.axvline(50, color="k", ls="--", lw=1)
    ax.set_xscale("log")
    ax.set_xlabel("compute per tick [ms] (log)")
    ax.set_ylabel("CDF")
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(O / "fig_tick_cdf.png", dpi=150)

if "P8_a76_thermal15" in runs:
    r = runs["P8_a76_thermal15"]
    t = [float(x["t"]) / 60 for x in r["sys"]]
    fig, ax = plt.subplots(figsize=(6, 3.2))
    for k in ("T_cpub", "T_cpul", "T_ddr"):
        ax.plot(t, [float(x.get(k, "nan") or "nan") for x in r["sys"]], label=k)
    ax2 = ax.twinx()
    tk = r["ticks"]
    tt = np.array([float(x["t"]) / 60 for x in tk])
    cm = np.array([float(x["compute_ms"]) for x in tk])
    nb = max(1, len(tt) // 60)
    ax2.plot(tt[: len(tt) // nb * nb].reshape(-1, nb).mean(1), np.percentile(cm[: len(cm) // nb * nb].reshape(-1, nb), 99, axis=1),
             color="k", lw=1, label="compute p99 (per bin)")
    ax.set_xlabel("minutes")
    ax.set_ylabel("°C")
    ax2.set_ylabel("ms")
    ax.legend(loc="upper left", fontsize=7)
    ax2.legend(loc="lower right", fontsize=7)
    fig.tight_layout()
    fig.savefig(O / "fig_thermal.png", dpi=150)
print((O / "profile_table.md").read_text())
print((O / "stage_table.md").read_text())
