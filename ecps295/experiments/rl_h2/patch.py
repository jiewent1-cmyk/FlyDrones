"""N1b (2026-10-13): optional full-cone ToF emulation in Gazebo (ECPS_TOF_CONE3D=1); default path unchanged."""
import os, shutil, sys
E = os.path.expanduser("~/sim/rl_h2/ecps295")
for f in ("mavlink_twin.py", "run_g4.py"):
    if not os.path.exists(f"{E}/{f}.pre_cone3d"):
        shutil.copy2(f"{E}/{f}", f"{E}/{f}.pre_cone3d")
src = open(f"{E}/mavlink_twin.py").read()
old_head = '''    def ranges(self, beams, fov_deg: float = 0.0, rate_hz: float = 10.0, noise_m: float = 0.01) -> list[float]:'''
new_methods = '''    def _ray3(self, p: tuple, d: tuple) -> float:
        """3D slab test from p = (north, east, up) along the unit vector d (north, east, up) against the layout boxes
        and, in flight, the floor; max 2 m."""
        best = min(2.0, p[2] / -d[2]) if d[2] < -1e-9 and p[2] > 0.3 else 2.0
        for o in getattr(getattr(self, "room", None), "boxes", None) or []:
            t0, t1 = 0.0, best
            for i in range(3):
                if abs(d[i]) < 1e-9:
                    if not o.lo[i] <= p[i] <= o.hi[i]:
                        t0 = math.inf
                        break
                else:
                    u, v = (o.lo[i] - p[i]) / d[i], (o.hi[i] - p[i]) / d[i]
                    t0, t1 = max(t0, min(u, v)), min(t1, max(u, v))
            if t0 <= t1:
                best = min(best, t0)
        return best

    def _cone3(self, n: float, e: float, a: float, s, b: float, h: float) -> float:
        """Shortest slant range over the whole ToF cone (axis + 8 rim rays at half-angle h) for body bearing b (rad),
        rotated by the SIM_STATE attitude. Unlike the planar fan it also sees box tops just below the sensor."""
        cr, sr, cp, sp = math.cos(s.roll), math.sin(s.roll), math.cos(s.pitch), math.sin(s.pitch)
        cy, sy = math.cos(s.yaw), math.sin(s.yaw)
        best = 2.0
        for k in range(9):
            r, phi = (0.0, 0.0) if k == 0 else (h, (k - 1) * math.pi / 4)
            az, el = b + r * math.cos(phi), r * math.sin(phi)
            x, y, z = math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), -math.sin(el)  # body FRD
            dn = cp * cy * x + (sr * sp * cy - cr * sy) * y + (cr * sp * cy + sr * sy) * z
            de = cp * sy * x + (sr * sp * sy + cr * cy) * y + (cr * sp * sy - sr * cy) * z
            dd = -sp * x + sr * cp * y + cr * cp * z
            best = min(best, self._ray3((n, e, a), (dn, de, -dd)))
        return best

'''
assert src.count(old_head) == 1
src = src.replace(old_head, new_methods + old_head)
old_body = '''                h = math.radians(fov_deg) / 2
                offs = (-h, 0.0, h) if h > 0 else (0.0,)
                st[1] = [max(0.0, min(2.0, min(self._ray(n, e, a, s.yaw + math.radians(b) + o) for o in offs) - 0.06
                                      + (st[2].gauss(0.0, noise_m) if noise_m > 0 else 0.0))) for b in beams]'''
new_body = '''                h = math.radians(fov_deg) / 2
                if h > 0 and os.environ.get("ECPS_TOF_CONE3D") == "1":  # N1b: full cone + attitude (box tops, floor)
                    raw = [self._cone3(n, e, a, s, math.radians(b), h) for b in beams]
                else:
                    offs = (-h, 0.0, h) if h > 0 else (0.0,)
                    raw = [min(self._ray(n, e, a, s.yaw + math.radians(b) + o) for o in offs) for b in beams]
                st[1] = [max(0.0, min(2.0, r - 0.06 + (st[2].gauss(0.0, noise_m) if noise_m > 0 else 0.0))) for r in raw]'''
assert src.count(old_body) == 1
src = src.replace(old_body, new_body)
old_imp = '''        import random

        if not hasattr(self, "_beams"):'''
assert src.count(old_imp) == 1
src = src.replace(old_imp, '''        import os
        import random

        if not hasattr(self, "_beams"):''')
open(f"{E}/mavlink_twin.py", "w").write(src)
g = open(f"{E}/run_g4.py").read()
old = '''    summary.update(pilot.struct.summary())
json.dump(summary, open(f"{a.out}.json", "w"), indent=1)'''
assert g.count(old) == 1
g = g.replace(old, '''    summary.update(pilot.struct.summary())
summary["tof_cone3d"] = os.environ.get("ECPS_TOF_CONE3D") == "1"  # N1b: Gazebo ToF emulation = full cone (else planar fan)
json.dump(summary, open(f"{a.out}.json", "w"), indent=1)''')
open(f"{E}/run_g4.py", "w").write(g)
print("patched")
