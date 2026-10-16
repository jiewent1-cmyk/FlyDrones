"""N4 twin-Gazebo gap (2026-10-15): per-tick trace of reference twin episodes (s7_w2map, exactly the rl.n3m_eval REF job)
on the Gazebo lowbox layouts, to see what the drone does when it reaches a low box (flies over / brain turns / shield).
python -m rl.box_trace <out.npz> ['{"ventral": "rays", ...}'] <k> [<k> ...]   (brain seed k, flow model hw_arms.flow_dr(k)
+ the optional TwinDrone extras, 120 s)"""
import json
import sys

import numpy as np

from rl import episode as E
from rl import hw_arms
from rl import shield_screen as ss
from rl.batch import world_path
from rl.twin import World

TR = []
_orig = E.SimClockPilot.tick


def _tick(self, t, dt):
    info = _orig(self, t, dt)
    d = self.drone
    est = d.est if d.nav == "flow" else (np.nan, np.nan)
    TR.append((t, *d.pos, est[0], est[1], info.raw.forward, info.raw.yaw, info.cmd.forward, info.cmd.yaw,
               float(bool(info.cmd.escape)), float(getattr(info.cmd, "lateral", 0.0) or 0.0)))
    return info


E.SimClockPilot.tick = _tick

if __name__ == "__main__":
    dst, rest = sys.argv[1], sys.argv[2:]
    extra = json.loads(rest.pop(0)) if rest and rest[0].startswith("{") else {}
    ks = [int(x) for x in rest]
    out = {}
    for k in ks:
        TR.clear()
        r = E.run_episode(ss.V4, World.from_layout(world_path(f"lowbox_s{k}")), seed=k, seconds=120.0,
                          overrides=ss.CONDS["s7_w2map"][2], dyn_over={**hw_arms.flow_dr(k), **extra})
        out[f"k{k}"] = np.array(TR, dtype=float)
        out[f"k{k}_meta"] = np.array([r["over_box_s"], r["coverage"], r["path_m"], r["contact_episodes"], r["escapes"]], dtype=float)
        print(k, "over_box_s", r["over_box_s"], "cov", r["coverage"], "path", r["path_m"], "contact", r["contact_episodes"], flush=True)
        np.savez_compressed(dst, **out)
