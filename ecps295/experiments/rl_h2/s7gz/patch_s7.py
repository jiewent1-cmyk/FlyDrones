"""S7 (2026-10-14) Gazebo side of the map shield: MavlinkTwin.zones() (n x n multi-zone ToF from SIM_STATE truth incl.
attitude, layout boxes + floor), vectorised _rays3np(); scalar multi-zone beams (0 < fov < 10 deg) now use the 8-row
column (_zones3) under ECPS_TOF_CONE3D=1 like the per-beam-list path; installs ecps_struct.py (map hook) + ecps_map.py
and the new minifly yamls. Single-zone (27 deg) and planar paths unchanged."""
import shutil
from pathlib import Path
E = Path.home() / "sim/rl_h2/ecps295"
SRC = Path.home() / "sim/rl_h2/s7gz"
for f in ("mavlink_twin.py", "ecps_struct.py"):
    if not (E / f"{f}.pre_s7").exists():
        shutil.copy2(E / f, E / f"{f}.pre_s7")
for f in ("ecps_struct.py", "ecps_map.py"):
    shutil.copy2(SRC / f, E / f)
for y in sorted((SRC / "yamls").glob("*.yaml")):
    if not (E / "minifly" / y.name).exists():
        shutil.copy2(y, E / "minifly" / y.name)
        print("yaml", y.name)
src = (E / "mavlink_twin.py").read_text()
old = '''                    if h > 0 and os.environ.get("ECPS_TOF_CONE3D") == "1":  # N1b: full cone + attitude (box tops, floor)
                        raw = [self._cone3(n, e, a, s, math.radians(b), h) for b in beams]'''
new = '''                    if h > 0 and os.environ.get("ECPS_TOF_CONE3D") == "1":  # N1b: full cone + attitude (box tops, floor);
                        # S7: a multi-zone column (fov < 10 deg = zone pitch) is its 8 zone rays, as in _ranges_model
                        raw = [self._zones3(n, e, a, s, math.radians(b), math.radians(fov_deg)) if fov_deg < 10.0
                               else self._cone3(n, e, a, s, math.radians(b), h) for b in beams]'''
assert src.count(old) == 1, "old path"
src = src.replace(old, new)
anchor = "    def side_ranges(self) -> dict:\n"
assert src.count(anchor) == 1
zones = '''    def zones(self, bearing: float, hfov: float, vfov: float, n: int = 8, rate_hz: float = 15.0, noise_m: float = 0.01,
              max_m: float = 3.5):
        """S7 (2026-10-14): one multi-zone ToF (n x n zones, level mount) facing body bearing `bearing` (deg), emulated
        from SIM_STATE truth (position + attitude) and the layout boxes + floor, as rl.twin.TwinDrone.zones(): slant range
        per zone-centre ray (row 0 = top), nan beyond max_m; None until truth arrives."""
        import numpy as np

        from ecps_map import zone_dirs

        if not hasattr(self, "_zones"):
            self._zones = {}
        key = (bearing, hfov, vfov, n, rate_hz, noise_m, max_m)
        st = self._zones.get(key)
        if st is None:
            st = self._zones[key] = [-1.0, None, np.random.default_rng(87_000 + len(self._zones)), zone_dirs(bearing, hfov, vfov, n)]
        now = time.monotonic()
        if now - st[0] >= 1.0 / rate_hz - 1e-6:
            tru = self.truth_ned()
            s = self.m.messages.get("SIM_STATE") if self.m is not None else None
            if tru is not None and s is not None:
                st[0] = now
                tn, te, ta = tru
                cr, sr, cp, sp = math.cos(s.roll), math.sin(s.roll), math.cos(s.pitch), math.sin(s.pitch)
                cy, sy = math.cos(s.yaw), math.sin(s.yaw)
                B = st[3]
                x, y, z = B[:, 0], B[:, 1], -B[:, 2]  # body FRD
                dn = cp * cy * x + (sr * sp * cy - cr * sy) * y + (cr * sp * cy + sr * sy) * z
                de = cp * sy * x + (sr * sp * sy + cr * cy) * y + (cr * sp * sy - sr * cy) * z
                dd = -sp * x + sr * cp * y + cr * cp * z
                D = np.stack([dn, de, -dd], axis=1)
                b = s.yaw + math.radians(bearing)
                o = np.array([tn + 0.06 * math.cos(b), te + 0.06 * math.sin(b), ta])
                r = self._rays3np(o, D)
                if noise_m > 0:
                    r = r + st[2].normal(0.0, noise_m, len(r))
                st[1] = np.where(r > max_m, np.nan, np.maximum(r, 0.0))
        return st[1]

    def _rays3np(self, o, D):
        """Vectorised _ray3: distance along each unit ray D (N, 3) (north, east, up) from o to the first layout box or,
        in flight, the floor; inf if none."""
        import numpy as np

        if not hasattr(self, "_box_lo"):
            boxes = getattr(getattr(self, "room", None), "boxes", None) or []
            self._box_lo = np.array([b.lo for b in boxes], float).reshape(-1, 3)
            self._box_hi = np.array([b.hi for b in boxes], float).reshape(-1, 3)
        t = np.full(len(D), np.inf)
        if len(self._box_lo):
            Dz = np.where(np.abs(D) < 1e-12, 1e-12, D)
            t1 = (self._box_lo[None, :, :] - o[None, None, :]) / Dz[:, None, :]
            t2 = (self._box_hi[None, :, :] - o[None, None, :]) / Dz[:, None, :]
            tmin = np.maximum(np.minimum(t1, t2).max(axis=2), 0.0)
            tmax = np.maximum(t1, t2).min(axis=2)
            t = np.where(tmax >= tmin, tmin, np.inf).min(axis=1)
        if o[2] > 0.3:
            t = np.minimum(t, np.where(D[:, 2] < -1e-9, o[2] / np.maximum(-D[:, 2], 1e-9), np.inf))
        return t

'''
src = src.replace(anchor, zones + anchor)
(E / "mavlink_twin.py").write_text(src)
print("patched")
