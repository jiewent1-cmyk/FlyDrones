# EXPERIMENT round 5: bias limits against slow rangefinder drift (only the companion_r4 copy)
#   b_cap  : trip when the tracked bias has moved more than b_cap [m] from its value at the first usable airborne frame
#   b_rate : trip when the tracked bias has moved more than b_rate [m] within the last 10 s
p = "companion_r4/baro_range_monitor.py"
s = open(p).read()
a = "        if arm_now:\n            self.armed = True\n"
assert s.count(a) == 1
s = s.replace(a, a + "            self._b0 = None\n")
a = "        # --- 触发汇总 ---\n"
b = '''        # --- EXPERIMENT round 5: bias limits ---
        cap, rate = getattr(self, "b_cap", 0.0), getattr(self, "b_rate", 0.0)
        if cap or rate:
            if getattr(self, "_b0", None) is None:
                self._b0 = self.b
                self._bh = deque()
            self._bh.append((t, self.b))
            while t - self._bh[0][0] > 10.0:
                self._bh.popleft()
            hit = (cap and abs(self.b - self._b0) > cap) or \\
                  (rate and t - self._bh[0][0] >= 9.5 and abs(self.b - self._bh[0][1]) > rate)
            if hit and self.t_trig is None:
                self.trig_A = True
                self.t_trig_A = self.t_trig = t
                self.reason = "bias"
'''
assert s.count(a) == 1
open(p, "w").write(s.replace(a, b + a))
p = "companion_r4/height_guard.py"
s = open(p).read()
a = '        m.skip_invalid = bool(c.get("skip_inv"))      # EXPERIMENT round 4b\n'
assert s.count(a) == 1
open(p, "w").write(s.replace(a, a + '        m.b_cap, m.b_rate = float(c.get("b_cap", 0.0)), float(c.get("b_rate", 0.0))      # EXPERIMENT round 5\n'))
g = open("gen_r4b.py").read()
i, j = g.index('ARMS["X_SYN_FIX"]'), g.index("jobs, seen = [], set()")
new = '''FIX = {**R4, "oor_s": 0.3, "skip_inv": 1}
ARMS["X_SYN_FIX"] = (SYNP, FIX, 4)
for k, v in (("BC06", {"b_cap": 0.6}), ("BC09", {"b_cap": 0.9}), ("BC12", {"b_cap": 1.2}), ("BC15", {"b_cap": 1.5}),
             ("BR03", {"b_rate": 0.3}), ("BR04", {"b_rate": 0.4})):
    ARMS[k] = (SYNP, {**FIX, **v}, 4)
FAULTS["m_dr002"] = ("", drift(0.02, tmax=100.0, dt=1.0))
F5 = ["m_dr002", "m_dr005", "r_i", "m_dr020", "r_gN", "none", "wind3", "b_dr12p", "b_dr12n", "b_st2p", "b_st2n", "b_noise"]
CELLS = [(a, f) for f in F5 for a in ("BC06", "BC09", "BC12", "BC15", "BR03", "BR04")] \\
    + [("X_SYN_FIX", f) for f in ("m_dr002", "m_dr005", "m_dr020", "b_dr12p", "b_dr12n", "b_st2p", "b_noise")]
'''
g = g[:i] + new + g[j:]
g = g.replace("range(1, 11)", "range(1, 9)").replace('"/jobs_r4b.txt"', '"/jobs_r5.txt"').replace('"/jobs_r4b_smoke.txt"', '"/jobs_r5_smoke.txt"')
k = g.index("pick = [")
g = g[:k] + 'pick = ["ab_BC09__m_dr005__r01", "ab_BR03__r_i__r01", "ab_BR03__b_dr12p__r01", "ab_BC09__none__r01"]\n' + g[g.index("open(D", k):]
open("gen_r5.py", "w").write(g)
