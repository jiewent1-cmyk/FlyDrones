"""N4 (2026-10-15): optional band below the drone in the S7 local map (ecps_map.LocalMap), so low box tops bound motion too.
Why: Gazebo w2map runs (116, flow-EKF): the EKF xy error grows 4.4x faster while the drone is over a low box (0.035 vs
0.008 m/s; the EKF terrain state lags the box top, so the flow is scaled with the floor distance), and in Gazebo the
velocity loop acts on that estimate, so the drone really drifts; the one c065 contact followed 4 s over a box (EKF error
0.44 -> 1.48 m). EK3_TERR_GRAD / EK3_RNG_I_GATE tuning did not help (S7d, 10-01).
Config ecps_pilot.struct.shield.map: band_lo_m (default band_m = old behaviour, bit-identical), over_r_m (default 0.15 m):
when an occupied voxel of the lower band is already within over_r_m of the drone (it is over the obstacle) the lower band is
skipped for that query, so the drone can fly off. run from the tree root: python3 patch_bandlo.py (backup ecps_map.py.pre_bandlo)"""
import shutil
from pathlib import Path

p = Path("ecps_map.py")
s = p.read_text()
assert "band_lo" not in s, "already patched"
pairs = [
    ("""        self.band = float(cfg.get("band_m", 0.10))
""", """        self.band = float(cfg.get("band_m", 0.10))
        # N4 (2026-10-15): depth of the band below the drone; deeper than band_m -> low box tops are obstacles (flying over
        # them corrupts the flow-EKF), except when the drone is already over one (within over_r_m), so it can leave
        self.band_lo = float(cfg.get("band_lo_m", self.band))
        self.over_r = float(cfg.get("over_r_m", 0.15))
        self.over_n = 0  # queries made while over a lower-band obstacle
"""),
    ("""        G = self.L.reshape(self.n, self.n, self.nz)[i0:i1, j0:j1, k0:k1].max(axis=2) > self.l_occ
""", """        W = self.L.reshape(self.n, self.n, self.nz)[i0:i1, j0:j1]
        G = W[:, :, k0:k1].max(axis=2) > self.l_occ
        kl = max(0, int((z - self.band_lo) / self.res))
        if kl < k0:  # N4 lower band
            Gl = W[:, :, kl:k0].max(axis=2) > self.l_occ
            il, jl = np.nonzero(Gl)
            dl = np.hypot((il + i0 + 0.5) * self.res - self.half - x, (jl + j0 + 0.5) * self.res - self.half - y)
            if len(dl) and dl.min() <= self.over_r:
                self.over_n += 1
            else:
                G = G | Gl
"""),
]
for old, new in pairs:
    assert s.count(old) == 1, old[:60]
    s = s.replace(old, new)
shutil.copy2(p, str(p) + ".pre_bandlo")
p.write_text(s)
print("patched ecps_map.py")
