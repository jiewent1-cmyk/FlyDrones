#!/usr/bin/env python3
"""Round 3 tables from all_runs.csv: E1 false trips of X_OOR, E3 dose-response, E2 threshold x hold time, E5 20-rep arms."""
import csv, collections, math, statistics as st
R = list(csv.DictReader(open("all_runs.csv")))
def f(x):
    try: return float(x)
    except Exception: return None
def wil(k, n, z=1.96):
    if n == 0: return (0, 0)
    p = k / n; d = 1 + z * z / n; c = p + z * z / (2 * n); h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (max(0, (c - h) / d), min(1, (c + h) / d))
def cell(arm, fault): return [r for r in R if r["arm"] == arm and r["fault"] == fault]
def summ(rs):
    c = collections.Counter(r["cls2"] for r in rs); n = len(rs)
    tr = sorted(f(r["guard_trip_after_arm_s"]) for r in rs if r["guard"] == "tripped" and f(r["guard_trip_after_arm_s"]) is not None)
    al = [f(r["alt_max"]) for r in rs if f(r["alt_max"]) is not None]; cl = [f(r["clr"]) for r in rs if f(r["clr"]) is not None]
    return dict(n=n, ok=c["OK"], ab=c["ABORT"], dg=c["DEGRADED"], un=c["UNSAFE"], ag=c["ARMED_GND"], nf=c["NOFLY"] + c["?"], trips=len(tr),
                tmed=st.median(tr) if tr else None, tmax=max(tr) if tr else None, alt=st.median(al) if al else None, altmax=max(al) if al else None, clr=min(cl) if cl else None)
def row(name, s):
    g = lambda v, p="%.2f": "-" if v is None else p % v
    return "| %s | %d | %d | %d | %d | %d | %d | %d | %s | %s | %s | %s | %s |" % (name, s["n"], s["ok"], s["ab"], s["dg"], s["un"], s["ag"], s["trips"], g(s["tmed"], "%.1f"), g(s["tmax"], "%.1f"), g(s["alt"]), g(s["altmax"]), g(s["clr"]))
HD = "| %s | n | OK | abort | degr | unsafe | armed-gnd | trips | trip med s | trip max s | alt med | alt max | clr min |\n|-|-|-|-|-|-|-|-|-|-|-|-|-|"
NORNG = ["none", "wind3", "b_dr12p", "b_dr12n", "b_st1p", "b_st2p", "b_st2n", "b_noise", "f_scale", "f_noise"]
RNG = ["r_a", "r_c", "r_g5", "r_gN", "r_h", "r_k", "r_i", "r_j2", "r_oor3", "x_gN_noise", "x_h_drn", "x_h_drp", "x_k_st2n"]
print("## E1 X_OOR: flights without a rangefinder fault\n"); print(HD % "fault")
tot = dict(n=0, k=0)
for fa in NORNG:
    s = summ(cell("X_OOR", fa)); print(row(fa, s)); tot["n"] += s["n"]; tot["k"] += s["trips"]
print("\nfalse trips %d / %d, 95%% upper %.2f%%" % (tot["k"], tot["n"], 100 * wil(tot["k"], tot["n"])[1]))
for a in ("FULL",):
    n = sum(len(cell(a, fa)) for fa in NORNG); k = sum(summ(cell(a, fa))["trips"] for fa in NORNG); print("%s false trips %d / %d" % (a, k, n))
print("\n## E1 X_OOR vs FULL: rangefinder faults (20 reps)\n"); print(HD % "arm / fault")
for fa in RNG + ["m_oo03", "m_oo10", "m_ooN", "m_blip", "d3_none", "d3_b_st2n", "d3_r_h", "d3_r_k", "d3_r_gN"]:
    for a in ("FULL", "X_OOR", "NOGUARD"):
        rs = cell(a, fa)
        if rs: print(row(a + " " + fa, summ(rs)))
print("\n## E3 dose-response\n"); print(HD % "arm / fault")
for fa in sorted({r["fault"] for r in R if r["fault"].startswith(("m_lo", "m_hi", "m_dr", "m_du"))}):
    for a in ("NOGUARD", "FULL"):
        rs = cell(a, fa)
        if rs: print(row(a + " " + fa, summ(rs)))
print("\n## E2 threshold x hold time\n")
SW = sorted({r["arm"] for r in R if r["arm"].startswith("S_E")}) + ["G_E05", "FULL", "G_E10"]
SF = ["r_h", "r_gN", "r_k", "r_i", "r_a", "r_c"]; SN = ["none", "wind3", "b_st2n", "b_st2p", "b_noise", "b_dr12p"]
print("| arm | false trips | " + " | ".join("%s trips / tmed / altmax" % x for x in SF) + " | unsafe+degr (faults) |"); print("|-" * (len(SF) + 3) + "|")
for a in SW:
    k = n = 0
    for fa in SN:
        s = summ(cell(a, fa)); k += s["trips"]; n += s["n"]
    out = []; bad = nb = 0
    for fa in SF:
        s = summ(cell(a, fa)); bad += s["un"] + s["dg"] + s["ag"]; nb += s["n"]
        out.append("%d/%d / %s / %s" % (s["trips"], s["n"], "-" if s["tmed"] is None else "%.1f" % s["tmed"], "-" if s["altmax"] is None else "%.2f" % s["altmax"]))
    print("| %s | %d/%d | " % (a, k, n) + " | ".join(out) + " | %d/%d |" % (bad, nb))
print("\n## E5 arms (all 23 faults), counts and 95% interval of unsafe\n"); print("| arm | n | OK | abort | degr | unsafe | armed-gnd | unsafe % (95%) | rng-fault unsafe | false trips |"); print("|-|-|-|-|-|-|-|-|-|-|")
F23 = NORNG + RNG
for a in ["NOGUARD", "BARE", "FULL", "STOCKGATE", "SURF1", "FULL_NOALTF", "BARE_NOALTF", "G_LANDONLY", "G_RATE", "G_E05", "G_E10", "BAROPRI", "X_OOR"]:
    rs = [r for r in R if r["arm"] == a and r["fault"] in F23]
    if not rs: continue
    s = summ(rs); lo, hi = wil(s["un"], s["n"])
    ru = summ([r for r in rs if r["fault"] in RNG]); ft = summ([r for r in rs if r["fault"] in NORNG])
    print("| %s | %d | %d | %d | %d | %d | %d | %.1f (%.1f-%.1f) | %d/%d | %d/%d |" % (a, s["n"], s["ok"], s["ab"], s["dg"], s["un"], s["ag"], 100 * s["un"] / s["n"], 100 * lo, 100 * hi, ru["un"], ru["n"], ft["trips"], ft["n"]))
cc = collections.Counter((r["arm"], r["fault"]) for r in R); print(collections.Counter(cc.values()))
print("rtf<0.9:", sum(1 for r in R if (f(r.get("rtf_min")) or 1) < 0.9), "no link:", sum(r["guard"] == "no link" for r in R), "blind:", sum(r.get("guard_blind") == "True" for r in R), "total", len(R))
