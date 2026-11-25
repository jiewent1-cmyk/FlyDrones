# EXPERIMENT round 4b: invalid range samples are kept out of the h_r low-pass (cfg "skip_inv"); only the companion_r4 copy
p = "companion_r4/baro_range_monitor.py"
s = open(p).read()
a = "        if distance is not None and math.isfinite(distance):\n            h_r_raw ="
b = ("        if distance is not None and math.isfinite(distance) \\\n"
     "                and (not getattr(self, \"skip_invalid\", False) or self.range_valid(distance)):\n            h_r_raw =")
assert s.count(a) == 1
open(p, "w").write(s.replace(a, b))
p = "companion_r4/height_guard.py"
s = open(p).read()
a = """        return BaroRangeMonitor(fc=c["fc_hz"], tau=c["tau_s"], E=c["e_m"], T=c["t_s"], R=c["rate_m"],
                                rng_min=c["rng_min_m"], rng_max=c["rng_max_m"])"""
b = """        m = BaroRangeMonitor(fc=c["fc_hz"], tau=c["tau_s"], E=c["e_m"], T=c["t_s"], R=c["rate_m"],
                             rng_min=c["rng_min_m"], rng_max=c["rng_max_m"])
        m.skip_invalid = bool(c.get("skip_inv"))      # EXPERIMENT round 4b
        return m"""
assert s.count(a) == 1
open(p, "w").write(s.replace(a, b))
p = "batch.py"
s = open(p).read()
a = 'free = list(range(30, 30 + int(os.environ.get("NINST", "4"))))'
assert s.count(a) == 1
open(p, "w").write(s.replace(a, 'B0 = int(os.environ.get("BASE", "30"))\nfree = list(range(B0, B0 + int(os.environ.get("NINST", "4"))))'))
g = open("gen_r4.py").read()
a = "jobs, seen = [], set()\nfor rep in range(1, 11):\n    for arm in ARMS4:\n        for fault in RNG + [\"m_ooN\", \"m_blip\"] + NORNG:"
assert g.count(a) == 1
g = g.replace(a, """ARMS["X_SYN_FIX"] = (SYNP, {**R4, "oor_s": 0.3, "skip_inv": 1}, 4)
CELLS = [("X_SYN_FIX", f) for f in ("m_blip", "m_oo03", "m_oo10", "m_ooN", "r_oor3", "none", "wind3", "b_st2n", "r_gN", "r_h", "r_k", "r_i")] \\
    + [("X_SYN_OOR", f) for f in ("m_oo03", "m_oo10")]
jobs, seen = [], set()
for rep in range(1, 11):
    for arm, fault in CELLS:
        if 1:""")
g = g.replace('"/jobs_r4.txt"', '"/jobs_r4b.txt"').replace('"/jobs_r4_smoke.txt"', '"/jobs_r4b_smoke.txt"')
open("gen_r4b.py", "w").write(g)
