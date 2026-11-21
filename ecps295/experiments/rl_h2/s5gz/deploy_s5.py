"""N1c 2026-10-13: S5 in Gazebo = ToF beams into the retina near grids + the S4 shield (same as rl.shield_screen cond s5).
Installs the S5 ecps_pilot.py / ecps_retina.py (identical to ROG's current files except the S5 hooks; off by default),
writes minifly/s5.yaml and makes run_g4.py log the retina ToF input ("tof" column, "tof_retina" in run.json)."""
import shutil
from pathlib import Path
import yaml
E = Path.home() / "sim/rl_h2/ecps295"
SRC = Path.home() / "sim/rl_h2/s5gz"
for f in ("ecps_pilot.py", "ecps_retina.py", "run_g4.py"):
    if not (E / f"{f}.pre_s5").exists():
        shutil.copy2(E / f, E / f"{f}.pre_s5")
for f in ("ecps_pilot.py", "ecps_retina.py"):
    shutil.copy2(SRC / f, E / f)
B5 = [-25.0, 25.0, 90.0, 180.0, -90.0]
cfg = yaml.safe_load((E / "minifly/v4.yaml").read_text())
cfg.setdefault("vision", {})["ecps_tof"] = {"enabled": True, "beams": B5}
cfg.setdefault("ecps_pilot", {}).setdefault("struct", {})["shield"] = {"enabled": True, "beams": B5, "unstick_s": 1.0}
(E / "minifly/s5.yaml").write_text("# s5 (arm intact): v4 + ToF beams into the retina near grids (vision.ecps_tof) + S4 shield,"
                                   " same overrides as rl.shield_screen s5\n" + yaml.safe_dump(cfg, sort_keys=False))
g = (E / "run_g4.py").read_text()
old = '            "box_under": int(box_under(tn, te, ta)),\n'
if '"tof": round(' not in g:
    assert g.count(old) == 1
    g = g.replace(old, old + '            "tof": round(max(getattr(pilot.retina, "last_tof", {"L": 0.0}).values()), 2),  # S5 retina input\n')
old2 = 'summary["tof_cone3d"] = os.environ.get("ECPS_TOF_CONE3D") == "1"  # N1b: Gazebo ToF emulation = full cone (else planar fan)\n'
if "tof_retina" not in g:
    assert g.count(old2) == 1
    g = g.replace(old2, old2 + 'summary["tof_retina"] = bool(getattr(getattr(pilot, "retina", None), "tof_on", False))  # N1c: S5 active\n')
(E / "run_g4.py").write_text(g)
print("deployed")
