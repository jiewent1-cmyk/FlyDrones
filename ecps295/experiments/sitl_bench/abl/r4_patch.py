"""Round 4: synthetic rangefinder. Makes companion_r4/ (experimental copy of the guard) and lets hg_loop pick it by cfg "_dir"."""
import os, shutil
os.makedirs("companion_r4", exist_ok=True)
shutil.copy("companion/baro_range_monitor.py", "companion_r4/baro_range_monitor.py")
s = open("companion/height_guard.py").read()

def rep(a, b):
    global s
    assert s.count(a) == 1, a
    s = s.replace(a, b)

rep("from __future__ import annotations\n\nimport time\n", "from __future__ import annotations\n\nimport math\nimport time\nfrom collections import deque\n")
rep("""        self.active = False                   # the monitor is judging (armed and off the ground)
""", """        self.active = False                   # the monitor is judging (armed and off the ground)
        # EXPERIMENT (round 4): synthetic rangefinder from the barometer, see _synth()
        self._sp_hist = deque()
        self._sp_stamp = self._sp0 = self._sh = None
        self._s_armed = self._synth_on = False
""")
rep("""        rng, att, prs = msgs.get("RANGEFINDER"), msgs.get("ATTITUDE"), msgs.get("SCALED_PRESSURE")
""", """        rng, att, prs = msgs.get("RANGEFINDER"), msgs.get("ATTITUDE"), msgs.get("SCALED_PRESSURE")
        if self.cfg.get("synth"):
            self._synth(drone, rng, att, prs, armed, now)
""")
rep("""    def _act(self, drone, now: float) -> None:
""", """    def _synth(self, drone, rng, att, prs, armed, now) -> None:
        \"\"\"EXPERIMENT (round 4). Second downward rangefinder (RNGFND2_TYPE 10, MAVLink) made from the barometer.

        Why: after a trip the EKF still scales the optical flow with the height above ground it gets from the faulty
        rangefinder (HAGL 0.05 m at a true 2 m: velocity estimate ~0, the position loop has no damping), and with the
        range out of limits the FC reports "no relative position" and LAND levels the attitude. Both need a believable
        range. Sent on the ground (the arming check wants data from every rangefinder), silent in normal flight (the EKF
        would mix it with the real one), sent again once the guard has tripped or the real range is invalid.\"\"\"
        if prs is not None and prs._timestamp != self._sp_stamp:
            self._sp_stamp = prs._timestamp
            if not armed:
                self._sp_hist.append((now, prs.press_abs))
                while now - self._sp_hist[0][0] > 3.0:
                    self._sp_hist.popleft()
            elif self._sp0:
                hb = 44330.0 * (1.0 - (prs.press_abs / self._sp0) ** 0.1903)
                self._sh = hb if self._sh is None else self._sh + 0.25 * (hb - self._sh)
        if armed and not self._s_armed:
            p = [v for t, v in self._sp_hist if now - t <= 2.0]
            self._sp0, self._sh = (sum(p) / len(p) if p else None), None
        if not armed:
            self._sp0, self._synth_on = None, False
        self._s_armed = armed
        if armed and rng is not None and self.active and (self.mon.h_b or 0.0) > 0.5 \\
                and not (float(self.cfg["rng_min_m"]) <= rng.distance <= float(self.cfg["rng_max_m"])):
            self._synth_on = True             # the real range is invalid in the air
        if self.tripped_t is not None:
            self._synth_on = True
        if armed and not self._synth_on:
            return
        h = 0.05
        if armed and self._sh is not None:
            h = max(0.03, self._sh - self.mon.b)
        c = math.cos(att.roll) * math.cos(att.pitch) if att is not None else 1.0
        d = min(7.5, h / max(0.5, c))
        drone.m.mav.distance_sensor_send(int(now * 1000) & 0xFFFFFFFF, 1, 800, int(round(d * 100)), 0, 2, 25, 255)

    def _act(self, drone, now: float) -> None:
""")
rep("""        m = drone.m
        m.mav.command_long_send(m.target_system, m.target_component, MAV_CMD_SET_EKF_SOURCE_SET, 0,
""", """        m = drone.m
        if self.cfg.get("synth"):             # the FC drops the faulty rangefinder: every reading is below its minimum
            m.mav.param_set_send(m.target_system, m.target_component, b"RNGFND1_MIN", 50.0, 9)
        m.mav.command_long_send(m.target_system, m.target_component, MAV_CMD_SET_EKF_SOURCE_SET, 0,
""")
open("companion_r4/height_guard.py", "w").write(s)
h = open("hg_loop.py").read()
a = 'sys.path.insert(0, sys.argv[0].rsplit("/", 1)[0] + "/companion")\n'
assert a in h
h = h.replace(a, '_cfg = json.loads(sys.argv[sys.argv.index("--cfg") + 1]) if "--cfg" in sys.argv else {}\n'
                 'sys.path.insert(0, sys.argv[0].rsplit("/", 1)[0] + "/" + _cfg.get("_dir", "companion"))  # "_dir": experimental copy of the guard\n')
open("hg_loop.py.new", "w").write(h)
os.replace("hg_loop.py.new", "hg_loop.py")
g = open("gen_r3.py").read()
a = 'ARMS = {**ARMS1, "X_OOR": (T0, OOR, 3)}\n'
assert a in g
g = g.split('jobs, seen = [], set()')[0].replace(a, a + '''SYNP = T0 + ",RNGFND2_TYPE=10,RNGFND2_ORIENT=25,RNGFND2_MAX=8,RNGFND2_MIN=0.01,RNGFND2_GNDCLR=0.03"
R4 = {"_dir": "companion_r4", "synth": 1}
ARMS4 = {"X_SYN": (SYNP, R4, 4), "X_SYN_OOR": (SYNP, {**R4, "oor_s": 0.3}, 4), "X_SYN_L1": (SYNP, {**R4, "oor_s": 0.3, "source_set": 1}, 4)}
ARMS.update(ARMS4)
''')
g = g.replace('"""Round 3 of the safety ablation', '"""Round 4 (synthetic rangefinder after a trip; generated from gen_r3.py by r4_patch.py). Original header: Round 3 of the safety ablation')
g += '''jobs, seen = [], set()
for rep in range(1, 11):
    for arm in ARMS4:
        for fault in RNG + ["m_ooN", "m_blip"] + NORNG:
            ovl, hg, _ = ARMS[arm]
            fo, fa = FAULTS[fault]
            yaw = YAWS[rep - 1]
            wdir = (yaw * 7 + 40 * rep) % 360
            tag = f"ab_{arm}__{fault}__r{rep:02d}"
            o = ",".join(x for x in (HARSH, ovl, fo, f"SIM_WIND_DIR={wdir}") if x)
            jobs.append("|".join([tag, "mtf01_sitl_rng.parm", "1", str(yaw), o, (SQ + " " + fa).strip()]))
            json.dump(hg, open(f"{D}/hg/{tag}.json", "w"))
open(D + "/jobs_r4.txt", "w").write("\\n".join(jobs) + "\\n")
pick = [f"ab_{a}__{f}__r01" for a, f in (("X_SYN", "none"), ("X_SYN", "r_gN"), ("X_SYN", "r_h"), ("X_SYN_OOR", "r_oor3"), ("X_SYN_OOR", "m_ooN"), ("X_SYN_L1", "r_gN"), ("X_SYN", "r_g5"), ("X_SYN_OOR", "r_k"))]
open(D + "/jobs_r4_smoke.txt", "w").write("\\n".join(j for j in jobs if j.split("|")[0] in pick) + "\\n")
print(len(jobs), "jobs")
'''
open("gen_r4.py", "w").write(g)
