"""Round 5 (bias limits against slow rangefinder drift): per arm x fault table from all_runs.csv + hg logs."""
import collections, csv, glob, json, os, statistics as st
D = os.path.dirname(os.path.abspath(__file__))
R = [r for r in csv.DictReader(open(D + "/all_runs.csv")) if r["arm"] in ("PORT", "PORT_T05", "X_SYN_FIX", "BR03")]
ARMS = ["X_SYN_FIX", "BR03", "PORT", "PORT_T05"]
FA = ["m_blip", "m_oo03", "m_oo10", "m_ooN", "r_gN", "r_h", "r_k", "r_a", "r_c", "r_i", "m_dr005", "m_dr020", "x_gN_noise", "none", "wind3", "b_dr12p", "b_dr12n", "b_st2p", "b_st2n", "b_noise", "f_scale", "f_noise"]
def info(r):
    tag = "ab_%s__%s__r%02d" % (r["arm"], r["fault"], int(r["rep"]))
    m = json.load(open(f"{D}/runs/{tag}/metrics.json"))
    tx = (m.get("script") or {}).get("texts", [])
    trip = any("ECPS: RANGEFINDER BAD" in t for t in tx)
    hg = open(f"{D}/runs/{tag}/hg.log", errors="ignore").read() if os.path.exists(f"{D}/runs/{tag}/hg.log") else ""
    reason = "bias" if "bias" in hg else ""
    return trip, m.get("left_auto_at"), m.get("mission_complete"), "Traceback" in hg, reason
out = ["# round 6", "", "runs: %d, rtf min %.2f" % (len(R), min(float(r["rtf_min"]) for r in R if r.get("rtf_min")) if "rtf_min" in R[0] else -1), ""]
out += ["| fault | " + " | ".join(ARMS) + " |", "|-" * (len(ARMS) + 1) + "|"]
tot = collections.defaultdict(collections.Counter)
for f in FA:
    line = []
    for a in ARMS:
        rs = [r for r in R if r["arm"] == a and r["fault"] == f]
        if not rs:
            line.append("-"); continue
        I = [info(r) for r in rs]
        c = collections.Counter(r["cls2"] for r in rs)
        tr = [i[1] for i in I if i[0] and i[1]]
        alt = [float(r["alt_max"]) for r in rs if r["alt_max"]]
        tot[a]["crash"] += sum(i[3] for i in I)
        for k, v in c.items(): tot[a][k] += v
        tot[a]["n"] += len(rs)
        if f in FA[13:]: tot[a]["false"] += sum(i[0] for i in I); tot[a]["nf"] += len(rs)
        line.append("%d trip %d%s; U%d D%d; alt %.2f/%.2f" % (len(rs), sum(i[0] for i in I), (" @%.0f s" % st.median(tr)) if tr else "",
                                                           c["UNSAFE"], c["DEGRADED"], st.median(alt), max(alt)))
    out.append("| %s | " % f + " | ".join(line) + " |")
out += ["", "| arm | n | OK | ABORT | ARMED_GND | DEGRADED | UNSAFE | false trips (no range fault) | guard crashes |", "|-|-|-|-|-|-|-|-|-|"]
for a in ARMS:
    t = tot[a]
    out.append("| %s | %d | %d | %d | %d | %d | %d | %d/%d | %d |" % (a, t["n"], t["OK"], t["ABORT"], t["ARMED_GND"], t["DEGRADED"], t["UNSAFE"], t["false"], t["nf"], t["crash"]))
open(D + "/r6_tables.md", "w").write("\n".join(out) + "\n"); print("\n".join(out))
