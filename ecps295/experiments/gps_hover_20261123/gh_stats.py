#!/usr/bin/env python3
"""Tables for the GPS hover study. usage: gh_stats.py [tag prefix] [metric ...]
Per flow noise x GPS quality x arm: n, complete, median [min-max] of the metrics, and the paired comparison with arm F
(same flow noise and repeat; Wilcoxon signed-rank, exact for small n)."""
import glob, json, os, statistics as st, sys
from scipy.stats import wilcoxon
D = os.path.dirname(os.path.abspath(__file__))
pre = sys.argv[1] if len(sys.argv) > 1 else ""
MET = sys.argv[2:] or ["exc_max_300", "exc_rms", "exc_end", "ekf_err_end"]
R, rows = {}, []
for f in sorted(glob.glob(D + "/runs/" + pre + "*/metrics.json")):
    try:
        d = json.load(open(f))
    except Exception:
        d = {"error": "bad json"}
    tag = os.path.basename(os.path.dirname(f))
    arm, cond, rep = tag[len(pre):].split("__")
    fl, gq = cond.split("_")
    R.setdefault((fl, gq, arm), {})[rep] = d
    rows.append(dict(tag=tag, arm=arm, flow=fl, gps=gq, rep=rep, **{k: v for k, v in d.items() if k not in ("script", "events", "tag")},
                     events=";".join(s for _, s in d.get("events", []) if "Hit ground" not in s)))
keys = sorted({k for r in rows for k in r})
open(D + "/results_%s.csv" % (pre or "all"), "w").write(",".join(keys) + "\n" + "\n".join(",".join(str(r.get(k, "")).replace(",", " ") for k in keys) for r in rows) + "\n")
flows, gqs = sorted({k[0] for k in R}), sorted({k[1] for k in R})
arms = sorted({k[2] for k in R}, key=lambda a: (a[0] not in "F", a))
for fl in flows:
    for gq in gqs:
        print("\n### flow %s, GPS %s\n" % (fl, gq))
        print("| arm | n | complete | " + " | ".join(MET) + " | %s < F | p |" % MET[0])
        print("|-|-|-|" + "-|" * (len(MET) + 2))
        for arm in arms:
            v = R.get((fl, gq, arm)) or (R.get((fl, "Gm", arm)) if arm in ("F", "F0") else None)
            if not v:
                continue
            ok = {r: d for r, d in v.items() if MET[0] in d}
            cells = []
            for k in MET:
                x = [d[k] for d in ok.values() if d.get(k) is not None]
                cells.append("%.2f [%.2f–%.2f]" % (st.median(x), min(x), max(x)) if x else "–")
            ref = R.get((fl, "Gm", "F"), {})
            pr = [(ok[r][MET[0]], ref[r][MET[0]]) for r in ok if r in ref and MET[0] in ref[r]]
            cmp_, p = "–", "–"
            if arm != "F" and len(pr) >= 5:
                cmp_ = "%d/%d" % (sum(a < b for a, b in pr), len(pr))
                try:
                    p = "%.3g" % wilcoxon([a for a, _ in pr], [b for _, b in pr]).pvalue
                except Exception:
                    p = "–"
            ev = sum(1 for d in v.values() if any("Hit ground" not in s for _, s in d.get("events", [])))
            print("| %s | %d | %d | %s | %s | %s |%s" % (arm, len(v), sum(bool(d.get("complete")) for d in v.values()), " | ".join(cells), cmp_, p,
                                                        " events in %d" % ev if ev else ""))
