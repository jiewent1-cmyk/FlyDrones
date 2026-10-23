"""Landmark fix in Gazebo (2026-10-22, lm1_matrix.txt): true drift while the tracker holds in front of the marker,
landmark fix on vs off, paired by world and seed.

    python lm_stats.py ~/sim/runs 'lm1__'

Per run (run.csv): the rows in ARRIVED from settle_s after the first one; reference = the true position (SIM_STATE) at
that moment; drift = distance of the true position from it. Reported: max, rms and end drift, and the EKF's own error
at the end (|EKF - truth|, both from the take-off point)."""
import csv
import math
import statistics as st
import sys
from pathlib import Path

root, prefix = Path(sys.argv[1]), sys.argv[2]
settle_s = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0


def f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def drift(run: Path):
    p = run / "run.csv"
    if not p.exists():
        return None
    rows = [r for r in csv.DictReader(open(p)) if r.get("trk") == "ARRIVED"]
    if not rows:
        return None
    t0 = f(rows[0]["t"]) + settle_s
    rows = [r for r in rows if f(r["t"]) >= t0]
    if len(rows) < 20:
        return None
    n0, e0 = f(rows[0]["north"]), f(rows[0]["east"])
    d = [math.hypot(f(r["north"]) - n0, f(r["east"]) - e0) for r in rows]
    last = rows[-1]
    ekf_err = math.hypot(f(last["ekf_north"]) - f(last["north"]), f(last["ekf_east"]) - f(last["east"]))
    return {"max": max(d), "rms": math.sqrt(sum(x * x for x in d) / len(d)), "end": d[-1], "ekf_err": ekf_err,
            "secs": f(last["t"]) - f(rows[0]["t"])}


pairs = {}
for run in sorted(root.glob(prefix + "*")):
    tag = run.name[len(prefix):]
    key, _, mode = tag.rpartition("_")
    r = drift(run)
    if r is not None:
        pairs.setdefault(key, {})[mode] = r
both = {k: v for k, v in pairs.items() if "on" in v and "off" in v}
print(f"{len(both)} pairs (of {len(pairs)} keys)")
for m in ("max", "rms", "end", "ekf_err"):
    off = [v["off"][m] for v in both.values()]
    on = [v["on"][m] for v in both.values()]
    better = sum(b < a for a, b in zip(off, on))
    line = f"{m:8s} off median {st.median(off):.3f} m  on median {st.median(on):.3f} m  on better in {better}/{len(both)}"
    try:
        from scipy.stats import wilcoxon

        line += f"  p={wilcoxon(off, on).pvalue:.2g}"
    except Exception:  # noqa: BLE001 - no scipy: no p value
        pass
    print(line)
for k, v in sorted(both.items()):
    print(f"  {k:18s} off max {v['off']['max']:.3f} end {v['off']['end']:.3f} | on max {v['on']['max']:.3f} end {v['on']['end']:.3f}"
          f" ({v['on']['secs']:.0f} s)")
