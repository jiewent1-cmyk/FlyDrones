"""Matrix files for the post-slew-fix re-runs (SafetyGovernor slew sign bug, commit 8b8eef9). Every run here uses the
fixed code; tags carry the prefix "fx_" so they never overwrite the S8 runs made with the bug.

    python gen_slewcheck.py OUT_DIR

Phase A (does the bug change the S8 ablation?): the S8 key conditions again, E1 on lowbox_s0..19 and E2 on the 32
wall approaches, same flags as gen_ablation.py. ablation_stats.py then compares fx_<cond> with <cond> (bug) and the
fixed effects with the S8 ones.
Phase B (hand-built models vs the RL decoders, all fixed):
  lowbox  E1 setting (flow + route B), lowbox_s0..29          v4, v4_oc5, v3_2, es2, es3, b2_wcov2/4/8
  cage20  the RL setting (GPS patrol in the 20' cage), 32 brain seeds
  wall    E2 wall approaches (flow + route B)
The RL decoders (rl/configs/*.yaml) are v3_2 brains with CMA-ES decoder parameters; their variant is given as a path
relative to ecps295/minifly (run_matrix.sh loads minifly/<variant>.yaml).
"""

import sys
from pathlib import Path

out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)

PATROL = "--mode patrol --seconds 120 --cruise 0.5 --max-forward 0.5"
APPROACH = "--mode approach --seconds 20 --cruise 0.5 --max-forward 0.5"
FENCE, EXT = "--fence-turn", "--ext-height"


def s(*kv: str) -> str:
    return " ".join(f"--set {x}" for x in kv)


# S8 key conditions (gen_ablation.py), condition -> (variant, extra)
A_E1 = {
    "full": ("v4", ""),
    "no_lcb": ("v4", s("vision.ecps_blank.gain=0")),
    "no_lcv": ("v4", s("vision.ecps_ventral.enabled=false")),
    "no_retreat": ("v4", s("decoder.ecps.retreat.enabled=false")),
    "no_new_cells": (
        "v4",
        s("vision.ecps_blank.gain=0", "vision.ecps_near.enabled=false", "vision.ecps_ventral.enabled=false"),
    ),
    "no_saccade": ("v4", s("decoder.ecps.saccade.enabled=false", "decoder.escape.mode=climb")),
    "no_efference": ("v4", s("decoder.ecps.throttle_per_forward=0")),
    "lad_v3": ("v3", ""),
}
A_E2 = ["full", "no_lcb", "no_new_cells", "no_saccade", "no_efference"]

RL = "../rl/configs/"
MODELS = {  # name -> variant
    "v4": "v4",
    "v4_oc5": RL + "v4_oc5",
    "v3_2": "v3_2",
    "es2": RL + "v3_2_es2",
    "es3": RL + "v3_2_es3",
    "wcov2": RL + "b2_wcov2",
    "wcov4": RL + "b2_wcov4",
    "wcov8": RL + "b2_wcov8",
}

LOWBOX20 = [(f"lowbox_s{k}", k) for k in range(20)]
LOWBOX30 = [(f"lowbox_s{k}", k) for k in range(30)]
WALLS = [(w, k) for w in ("wall_tape", "wall_plain", "wall_offset", "wall_offset_plain") for k in range(8)]
CAGE = [("cage20", k) for k in range(32)]


def lines(cond: str, variant: str, extra: str, episodes, base: str) -> list[str]:
    return [f"fx_{cond}__{w}_k{k} {w} {variant} {k} {base} {extra}".rstrip() for w, k in episodes]


A_e1, A_e1_noB, A_e2, B_low, B_cage, B_wall = [], [], [], [], [], []
for cond, (variant, extra) in A_E1.items():
    A_e1 += lines(cond, variant, f"{FENCE} {EXT} {extra}".strip(), LOWBOX20, PATROL)
A_e1_noB += lines("no_routeB", "v4", FENCE, LOWBOX20, PATROL)
for cond in A_E2:
    variant, extra = A_E1[cond]
    A_e2 += lines(cond, variant, f"{EXT} {extra}".strip(), WALLS, APPROACH)
for name, variant in MODELS.items():
    # v4 on lowbox_s0..19 and on the walls is Phase A's fx_full; only the extra worlds are added here
    low = [e for e in LOWBOX30 if e[1] >= 20] if name == "v4" else LOWBOX30
    B_low += lines("full" if name == "v4" else "m_" + name, variant, f"{FENCE} {EXT}", low, PATROL)
    B_cage += lines("m_" + name, variant, FENCE, CAGE, PATROL)
    if name != "v4":
        B_wall += lines("m_" + name, variant, EXT, WALLS, APPROACH)

G, FS = "$V/G_extnav_height.parm", "$V/companion_fs.parm"
BATCHES = (
    ("A_e1", A_e1, "flow", f"{G},{FS}"),
    ("A_e1_noB", A_e1_noB, "flow", FS),
    ("A_e2", A_e2, "flow", f"{G},{FS}"),
    ("B_low", B_low, "flow", f"{G},{FS}"),
    ("B_cage", B_cage, "gps", FS),
    ("B_wall", B_wall, "flow", f"{G},{FS}"),
)
tsv = []
for name, ls, nav, extra in BATCHES:
    (out / f"{name}.txt").write_text("\n".join(ls) + "\n")
    tsv.append(f"{name}\t{nav}\t{extra}")
    print(f"{name}: {len(ls)} runs ({nav}, {extra})")
(out / "batches.tsv").write_text("\n".join(tsv) + "\n")
print("total", sum(len(b[1]) for b in BATCHES))
