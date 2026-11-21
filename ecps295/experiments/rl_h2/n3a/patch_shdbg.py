"""ROG Gazebo tree only (2026-10-15): shield debug columns in run.csv, behaviour unchanged.
After the S7b graze (g7_w2map__lowbox_s32_k132: 3 s of slow side drift during an in-place turn, then a retreat into the box)
the log lacked the lateral command and the shield's view. Adds:
  ecps_struct.RangeShield.dbg = (nearest constraint reading m, its body bearing deg, constraints the request violates)
  run.csv: cmd_lateral, sh_near, sh_brg, sh_bind
run from ~/sim/rl_h2/ecps295: python3 patch_shdbg.py  (backups *.pre_shdbg)"""
import shutil
from pathlib import Path


def patch(path, pairs):
    p = Path(path)
    s = p.read_text()
    assert "sh_near" not in s and "self.dbg" not in s, f"{path}: already patched"
    for old, new in pairs:
        assert s.count(old) == 1, (path, old[:70])
        s = s.replace(old, new)
    shutil.copy2(p, str(p) + ".pre_shdbg")
    p.write_text(s)


patch("ecps_struct.py", [
    ("""        self.mem_ticks = self.path_ticks = 0
""", """        self.mem_ticks = self.path_ticks = 0
        self.dbg = (None, None, 0)  # debug only: nearest constraint reading, its bearing, constraints the request violates
"""),
    ("""        retreat = False
        for _ in range(3):  # cyclic projection""", """        if cons:
            k = min(range(len(cons)), key=lambda i: cons[i][1])
            (ux, uy), dk = cons[k]
            self.dbg = (round(dk, 3), round(math.degrees(math.atan2(uy, ux)), 1),
                        sum(vx * u0 + vy * u1 > self.alpha * (di - self.d_safe) + 1e-9 for (u0, u1), di in cons))
        else:
            self.dbg = (None, None, 0)
        retreat = False
        for _ in range(3):  # cyclic projection"""),
])

patch("run_g4.py", [
    ("""            "cmd_forward": info.cmd.forward,
            "escape": int(info.cmd.escape),""", """            "cmd_forward": info.cmd.forward,
            "cmd_lateral": getattr(info.cmd, "lateral", 0.0),
            **_shield_dbg(pilot),
            "escape": int(info.cmd.escape),"""),
    ("""clock = {"t": 0.0}  # flight time across phases
""", """clock = {"t": 0.0}  # flight time across phases


def _shield_dbg(pilot) -> dict:
    \"\"\"2026-10-15 debug columns: what the range shield saw this tick (nearest constraint reading / body bearing, number of
    constraints the requested command violated); empty without a shield.\"\"\"
    sh = getattr(getattr(pilot, "struct", None), "shield", None)
    if sh is None:
        return {}
    d, b, n = getattr(sh, "dbg", (None, None, 0))
    return {"sh_near": d, "sh_brg": b, "sh_bind": n}
"""),
])
print("patched ecps_struct.py, run_g4.py")
