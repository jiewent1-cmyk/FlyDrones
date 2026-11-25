"""Round 4, second step: the guard watches the real sensor (DISTANCE_SENSOR id 0), and the synthetic range uses the bias from 5 s before the trip."""
import os
p = "companion_r4/height_guard.py"
s = open(p).read()

def rep(a, b):
    global s
    assert s.count(a) == 1, a
    s = s.replace(a, b)

rep("""        if self.cfg.get("synth"):
            self._synth(drone, rng, att, prs, armed, now)
""", """        if self.cfg.get("synth"):
            # RANGEFINDER is the FC's pick of the two sensors, so it would show the synthetic one: watch sensor 0 itself
            d0 = msgs.get("DISTANCE_SENSOR[0]")
            if d0 is not None:
                rng = _Rng(d0.current_distance * 0.01, d0._timestamp)
            self._synth(drone, rng, att, prs, armed, now)
""")
rep("""class HeightGuard:
""", """class _Rng:
    def __init__(self, distance, stamp):
        self.distance, self._timestamp = distance, stamp


class HeightGuard:
""")
rep("""        if armed and not self._synth_on:
            return
        h = 0.05
        if armed and self._sh is not None:
            h = max(0.03, self._sh - self.mon.b)
""", """        if armed and not self._synth_on:
            if self.mon.b_set:                # bias history: the last seconds before a trip already follow the fault
                self._b_hist.append((now, self.mon.b))
                while now - self._b_hist[0][0] > 5.0:
                    self._b_hist.popleft()
            return
        if not armed:
            self._b_hist.clear()
        h = 0.05
        if armed and self._sh is not None:
            h = max(0.03, self._sh - (self._b_hist[0][1] if self._b_hist else self.mon.b))
""")
rep("""        self._sp_hist = deque()
""", """        self._sp_hist = deque()
        self._b_hist = deque()
""")
open(p + ".new", "w").write(s)
os.replace(p + ".new", p)
h = open("hg_loop.py").read()
a = "    for mid in (mav.MAVLINK_MSG_ID_RANGEFINDER, mav.MAVLINK_MSG_ID_ATTITUDE, mav.MAVLINK_MSG_ID_SCALED_PRESSURE):\n"
assert a in h
h = h.replace(a, "    for mid in (mav.MAVLINK_MSG_ID_RANGEFINDER, mav.MAVLINK_MSG_ID_ATTITUDE, mav.MAVLINK_MSG_ID_SCALED_PRESSURE) + \\\n"
                 "            ((mav.MAVLINK_MSG_ID_DISTANCE_SENSOR,) if _cfg.get(\"synth\") else ()):\n")
open("hg_loop.py.new", "w").write(h)
os.replace("hg_loop.py.new", "hg_loop.py")
