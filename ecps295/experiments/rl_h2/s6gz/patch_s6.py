"""S6 (2026-10-14) Gazebo side of the sensor-layout screen: per-beam FoV lists, the camera-style range model and
multi-zone (8x8) columns in MavlinkTwin.ranges(); installs the S6 ecps_struct / ecps_retina / ecps_pilot (range memory,
model passthrough; bit-identical for the old configs) and writes minifly/s6_*.yaml from yamls/. Old call path unchanged."""
import os, shutil, sys
from pathlib import Path
E = Path.home() / "sim/rl_h2/ecps295"
SRC = Path.home() / "sim/rl_h2/s6gz"
for f in ("mavlink_twin.py", "ecps_struct.py", "ecps_retina.py", "ecps_pilot.py"):
    if not (E / f"{f}.pre_s6").exists():
        shutil.copy2(E / f, E / f"{f}.pre_s6")
for f in ("ecps_struct.py", "ecps_retina.py", "ecps_pilot.py"):
    shutil.copy2(SRC / f, E / f)
for y in sorted((SRC / "yamls").glob("s6_*.yaml")):
    shutil.copy2(y, E / "minifly" / y.name)
src = (E / "mavlink_twin.py").read_text()
old_ray = '''    def _ray(self, n: float, e: float, a: float, ang: float) -> float:
        """Horizontal distance from (n, e) at height a to the first layout box (nets included) along bearing ang, max
        2 m; the same slab test as rl.twin.TwinDrone._ray."""
        best = 2.0
        dn, de = math.cos(ang), math.sin(ang)
        for o in getattr(getattr(self, "room", None), "boxes", None) or []:
            if not o.lo[2] <= a <= o.hi[2]:
                continue
'''
new_ray = '''    def _ray(self, n: float, e: float, a: float, ang: float, skip: frozenset = frozenset()) -> float:
        """Horizontal distance from (n, e) at height a to the first layout box (nets included) along bearing ang, max
        2 m; the same slab test as rl.twin.TwinDrone._ray. `skip`: box names this ray does not see (S6 camera model)."""
        best = 2.0
        dn, de = math.cos(ang), math.sin(ang)
        for o in getattr(getattr(self, "room", None), "boxes", None) or []:
            if not o.lo[2] <= a <= o.hi[2] or (skip and _box_name(o) in skip):
                continue
'''
assert src.count(old_ray) == 1, "ray"
src = src.replace(old_ray, new_ray)
i0 = src.index("    def ranges(self, beams, fov_deg: float = 0.0, rate_hz: float = 10.0, noise_m: float = 0.01) -> list[float]:")
i1 = src.index("    def side_ranges(self) -> dict:")
new_ranges = '''    def ranges(self, beams, fov_deg=0.0, rate_hz: float = 10.0, noise_m: float = 0.01, **model) -> list[float]:
        """Horizontal ToF beams (body bearings in deg, + = right) emulated from SIM_STATE truth and the world layout
        (run_g4 --layout sets self.room): same geometry, cone, noise and sample-and-hold as rl.twin.TwinDrone.ranges().
        SITL only; reads 2.0 (no echo) until truth arrives.
        S6 (2026-10-14): fov_deg may be a per-beam list and `model` adds the camera-style range model, as in the twin;
        see _ranges_model() for the geometry per sensor kind. Without either the old path runs unchanged."""
        import os
        import random

        if not hasattr(self, "_beams"):
            self._beams = {}
        fk = tuple(float(f) for f in fov_deg) if isinstance(fov_deg, (list, tuple)) else fov_deg
        key = (tuple(beams), fk, rate_hz, noise_m, tuple(sorted(model.items())))
        st = self._beams.get(key)
        if st is None:
            st = self._beams[key] = [-1.0, [2.0] * len(beams), random.Random(86_000 + len(self._beams))]
        now = time.monotonic()
        if now - st[0] >= 1.0 / rate_hz - 1e-6:
            tru = self.truth_ned()
            s = self.m.messages.get("SIM_STATE") if self.m is not None else None
            if tru is not None and s is not None:
                st[0] = now
                n, e, a = tru
                if model or isinstance(fk, tuple):
                    st[1] = self._ranges_model(n, e, a, s, beams, fk, noise_m, st, **model)
                else:
                    h = math.radians(fov_deg) / 2
                    if h > 0 and os.environ.get("ECPS_TOF_CONE3D") == "1":  # N1b: full cone + attitude (box tops, floor)
                        raw = [self._cone3(n, e, a, s, math.radians(b), h) for b in beams]
                    else:
                        offs = (-h, 0.0, h) if h > 0 else (0.0,)
                        raw = [min(self._ray(n, e, a, s.yaw + math.radians(b) + o) for o in offs) for b in beams]
                    st[1] = [max(0.0, min(2.0, r - 0.06 + (st[2].gauss(0.0, noise_m) if noise_m > 0 else 0.0))) for r in raw]
        return st[1]

    def _ranges_model(self, n, e, a, s, beams, fov, noise_m, st, noise_k=0.0, max_m=2.0, p_miss=0.0, p_net=1.0):
        """S6 per-beam geometry (with ECPS_TOF_CONE3D=1; planar otherwise): single-zone ToF (fov >= 10 deg) = the full
        cone (_cone3); multi-zone column (0 < fov < 10 deg, zone pitch = fov) = the 8 zone rays of that 8x8 column
        (_zones3), nearest of the column; 2D lidar (fov 0) and the camera model (noise_k > 0, ground-contact-line
        ranging at flight height) stay planar. Camera model as in the twin: sigma = noise_m + noise_k d^2, nothing
        beyond max_m, p_miss lost readings, each net wall seen with p_net for the whole run."""
        import os

        rng = st[2]
        if len(st) < 4:
            boxes = getattr(getattr(self, "room", None), "boxes", None) or []
            st.append(frozenset(_box_name(o) for o in boxes if _box_name(o).startswith("net_") and rng.random() >= p_net))
        skip = st[3]
        cone = os.environ.get("ECPS_TOF_CONE3D") == "1"
        fovs = fov if isinstance(fov, tuple) else (float(fov),) * len(beams)
        out = []
        for b, fv in zip(beams, fovs):
            br, h = math.radians(b), math.radians(fv) / 2
            if cone and noise_k == 0 and fv >= 10.0:
                d = self._cone3(n, e, a, s, br, h)
            elif cone and noise_k == 0 and 0 < fv < 10.0:
                d = self._zones3(n, e, a, s, br, math.radians(fv))
            else:
                offs = (-h, 0.0, h) if h > 0 else (0.0,)
                d = min(self._ray(n, e, a, s.yaw + br + o, skip) for o in offs)
            d -= 0.06
            sd = noise_m + noise_k * d * d
            if sd > 0:
                d += rng.gauss(0.0, sd)
            if (p_miss > 0 and rng.random() < p_miss) or d > max_m:
                d = 2.0
            out.append(max(0.0, min(2.0, d)))
        return out

    def _zones3(self, n: float, e: float, a: float, s, b: float, pitch: float) -> float:
        """S6 multi-zone ToF column (VL53L5CX / L7CX 8x8): the 8 zone-centre rays of the column at body bearing b,
        elevations (k - 3.5) * pitch, rotated by the SIM_STATE attitude like _cone3; nearest of the column (box tops
        and, low enough, the floor included)."""
        cr, sr, cp, sp = math.cos(s.roll), math.sin(s.roll), math.cos(s.pitch), math.sin(s.pitch)
        cy, sy = math.cos(s.yaw), math.sin(s.yaw)
        best = 2.0
        for k in range(8):
            el = (k - 3.5) * pitch
            x, y, z = math.cos(el) * math.cos(b), math.cos(el) * math.sin(b), -math.sin(el)  # body FRD
            dn = cp * cy * x + (sr * sp * cy - cr * sy) * y + (cr * sp * cy + sr * sy) * z
            de = cp * sy * x + (sr * sp * sy + cr * cy) * y + (cr * sp * sy - sr * cy) * z
            dd = -sp * x + sr * cp * y + cr * cp * z
            best = min(best, self._ray3((n, e, a), (dn, de, -dd)))
        return best

'''
src = src[:i0] + new_ranges + src[i1:]
helper = '''

def _box_name(o) -> str:
    """Layout box name (world_layout.room_from_layout passes the layout name to flydrones Box), '' if none."""
    for k in ("name", "label", "id"):
        v = getattr(o, k, None)
        if isinstance(v, str):
            return v
    return ""
'''
src = src.rstrip("\n") + "\n" + helper
(E / "mavlink_twin.py").write_text(src)
print("patched")
