"""ROG Gazebo tree (2026-10-15, N4 drift stress), default off. Run from ~/sim/rl_h2/ecps295 after patch_drift.py.
  mavlink_twin.py  ECPS_DRIFT_MPS > 0: every setpoint sent through send() gets a steady world-frame velocity the companion
                   did not command (uncompensated wind / GUIDED steady-state error); the shield never sees it
  run_g4.py        drift direction from the brain seed (paired runs drift the same way); run.json drift_mps / drift_dir_deg
backups *.pre_driftgz"""
import shutil
from pathlib import Path


def patch(path, pairs):
    p = Path(path)
    s = p.read_text()
    assert "drift_mps" not in s, f"{path}: already patched"
    for old, new in pairs:
        assert s.count(old) == 1, (path, old[:70])
        s = s.replace(old, new)
    shutil.copy2(p, str(p) + ".pre_driftgz")
    p.write_text(s)


patch("mavlink_twin.py", [
    ("""        self._ext_sent = 0.0  # monotonic time of the last height actually sent (EcpsPilot height-source watchdog)
        self.mute = False
""", """        self._ext_sent = 0.0  # monotonic time of the last height actually sent (EcpsPilot height-source watchdog)
        self.mute = False
        # N4 stress (2026-10-15): ECPS_DRIFT_MPS > 0 adds a steady world-frame velocity (direction drift_dir, rad from north,
        # set by run_g4 from the brain seed) to every setpoint sent through send(): an uncompensated drift the companion did
        # not command, so the range shield never sees it (rl.twin ood="wind" without the gusts)
        import os

        self.drift_mps = float(os.environ.get("ECPS_DRIFT_MPS", "0") or 0.0)
        self.drift_dir = 0.0
"""),
    ("""        self._heartbeat()
        self.send_ext_height()
        super().send(cmd)
""", """        self._heartbeat()
        self.send_ext_height()
        if self.drift_mps > 0 and self.m is not None:
            att = self.m.messages.get("ATTITUDE")
            yw = att.yaw if att is not None else 0.0
            dn, de = self.drift_mps * math.cos(self.drift_dir), self.drift_mps * math.sin(self.drift_dir)
            self._send_velocity(cmd.forward * self.v_max + math.cos(yw) * dn + math.sin(yw) * de,
                                cmd.lateral * self.v_max - math.sin(yw) * dn + math.cos(yw) * de,
                                -cmd.throttle * self.vz_max, cmd.yaw * self.yr_max)
            return
        super().send(cmd)
"""),
])
patch("run_g4.py", [
    ("""_arms.install(ARM, brain, pilot, drone, cfg, a.seed)
""", """_arms.install(ARM, brain, pilot, drone, cfg, a.seed)
if getattr(drone, "drift_mps", 0.0) > 0:  # N4 drift stress: paired runs (same brain seed) drift the same way
    import random as _random

    drone.drift_dir = _random.Random(77_000 + a.seed).uniform(0.0, 2 * math.pi)
"""),
    ("""summary["tof_cone3d"] = os.environ.get("ECPS_TOF_CONE3D") == "1\"""",
     """summary["drift_mps"] = getattr(drone, "drift_mps", 0.0)  # N4 drift stress (ECPS_DRIFT_MPS), 0 = off
summary["drift_dir_deg"] = round(math.degrees(getattr(drone, "drift_dir", 0.0)), 1)
summary["shield_drift_ticks"] = getattr(getattr(getattr(pilot, "struct", None), "shield", None), "drift_ticks", 0)
summary["tof_cone3d"] = os.environ.get("ECPS_TOF_CONE3D") == "1\""""),
])
print("patched mavlink_twin.py, run_g4.py")
