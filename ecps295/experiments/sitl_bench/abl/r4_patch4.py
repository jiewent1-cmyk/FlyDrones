# round 4b fix: with skip_invalid the first sample can be invalid, so h_r may still be None when a valid one arrives
p = "companion_r4/baro_range_monitor.py"
s = open(p).read()
a = "            if h_r_raw is not None:\n                self.h_r += self._alpha(dt, self._tau_f) * (h_r_raw - self.h_r)"
b = ("            if h_r_raw is not None and self.h_r is None:\n                self.h_r = h_r_raw\n"
     "            elif h_r_raw is not None:\n                self.h_r += self._alpha(dt, self._tau_f) * (h_r_raw - self.h_r)")
assert s.count(a) == 1
open(p, "w").write(s.replace(a, b))
