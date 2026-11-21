"""route review 2026-10-13: S4 shield variants for Gazebo (v4 + ecps_pilot.struct.shield), same configs as rl.shield_screen;
patch run_g4.py so run.json carries the struct / shield counters"""
import copy, sys
from pathlib import Path
import yaml
E = Path(sys.argv[1])
FRONT8 = [-19.69, -14.06, -8.44, -2.81, 2.81, 8.44, 14.06, 19.69]
CONDS = {
    "sh5u": {"beams": [-25.0, 25.0, 90.0, 180.0, -90.0], "unstick_s": 1.0},
    "shfu": {"beams": FRONT8 + [90.0, 180.0, -90.0], "fov_deg": 5.6, "rate_hz": 15.0, "unstick_s": 1.0},
}
v4 = yaml.safe_load((E / "minifly" / "v4.yaml").read_text())
for name, sh in CONDS.items():
    cfg = copy.deepcopy(v4)
    over = {"ecps_pilot": {"struct": {"shield": {"enabled": True, **sh}}}}
    cfg.setdefault("ecps_pilot", {}).setdefault("struct", {})["shield"] = over["ecps_pilot"]["struct"]["shield"]
    (E / "minifly" / f"{name}.yaml").write_text(f"# {name} (arm intact, structural fix on v4: {over})\n" + yaml.safe_dump(cfg, sort_keys=False))
    print("wrote", name)
p = E / "run_g4.py"
s = p.read_text()
old = 'json.dump(summary, open(f"{a.out}.json", "w"), indent=1)'
if "pilot.struct.summary()" not in s:
    assert s.count(old) == 1
    s = s.replace(old, 'if getattr(pilot, "struct", None) is not None:  # 2026-10-13 ecps_struct counters (S2 vetoes, S3 pushes, S4 shield)\n'
                       '    summary.update(pilot.struct.summary())\n' + old)
    p.write_text(s)
    print("patched run_g4.py")
