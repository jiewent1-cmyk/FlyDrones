"""Matrix files for the S8 ablation (TechRoute §10 S8, README "S8 ablation"). Every condition flies the same episodes.

    python gen_ablation.py OUT_DIR

Writes one matrix per parameter batch and batches.tsv (batch, NAV, EXTRA_PARM) for run_ablation.sh:
  e1_B      E1 patrol, companion height (route B)          flow, G_extnav_height + companion_fs
  e1_noB    E1 patrol, flow EKF height                       flow, companion_fs
  e1_gps    E1 patrol, GPS reference                         gps, companion_fs
  e2_B      E2 wall approaches, route B                      flow, G_extnav_height + companion_fs
  e2_noB    E2 wall approaches, upstream baseline            flow, companion_fs
  e3_B      E3 faults (camera freeze, height source, hang)   flow, G_extnav_height + companion_fs
  e3_noFS   E3 companion hang without the FC failsafe        flow, G_extnav_height
Tags are <condition>__<episode> (ablation_stats.py pairs on the episode). Variant "up_v0" = upstream MiniFly with the
upstream decoder, pilot and retina (run_matrix.sh).
"""

import sys
from pathlib import Path

out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)

PATROL = "--mode patrol --seconds 120 --cruise 0.5 --max-forward 0.5"
APPROACH = "--mode approach --seconds 20 --cruise 0.5 --max-forward 0.5"
FENCE = "--fence-turn"
EXT = "--ext-height"


def s(*kv: str) -> str:
    return " ".join(f"--set {x}" for x in kv)


# condition -> (variant, extra args); route B / fence-turn flags are added per batch below
ABLATIONS = {
    "full": ("v4", ""),
    # model: new cell types and their sensory features
    "no_lcb": ("v4", s("vision.ecps_blank.gain=0")),
    "no_lcb_side": ("v4", s("vision.ecps_blank.side_gain=0")),
    "no_lcn": ("v4", s("vision.ecps_near.enabled=false")),
    "no_lcv": ("v4", s("vision.ecps_ventral.enabled=false")),
    "no_retreat": ("v4", s("decoder.ecps.retreat.enabled=false")),
    "no_retreat_cap": ("v4", s("decoder.ecps.retreat.max_s=1000000000")),
    "no_baro_rise": ("v4", s("vision.ecps_ventral.rise_m=99")),
    "no_new_cells": (
        "v4",
        s("vision.ecps_blank.gain=0", "vision.ecps_near.enabled=false", "vision.ecps_ventral.enabled=false"),
    ),
    # read-out (EcpsDecoder)
    "no_saccade": ("v4", s("decoder.ecps.saccade.enabled=false", "decoder.escape.mode=climb")),
    "no_caution": ("v4", s("decoder.ecps.caution.hold_s=0", "decoder.ecps.caution.ramp_s=0", "decoder.ecps.caution.on_brake=0")),
    "no_brake_mem": ("v4", s("decoder.ecps.brake_decay=0.93")),
    "no_efference": ("v4", s("decoder.ecps.throttle_per_forward=0")),
    "no_yaw_decouple": ("v4", s("decoder.ecps.throttle_yaw_decouple=0")),
    # cumulative ladder (each step's own config)
    "lad_v1": ("v1_3", ""),
    "lad_v2": ("v2_1", ""),
    "lad_v3": ("v3", ""),
    # upstream FlyDrones
    "up": ("up_v0", ""),
}
SYSTEM_ONLY = {"no_fence_turn", "no_routeB", "gps_ref"}  # E1 only

E1_EPISODES = [(f"lowbox_s{k}", k) for k in range(20)]
E2_EPISODES = [(w, k) for w in ("wall_tape", "wall_plain", "wall_offset", "wall_offset_plain") for k in range(8)]


def lines(cond: str, variant: str, extra: str, episodes, base: str) -> list[str]:
    return [f"{cond}__{world}_k{k} {world} {variant} {k} {base} {extra}".rstrip() for world, k in episodes]


e1_B, e1_noB, e1_gps, e2_B, e2_noB, e3_B, e3_noFS = [], [], [], [], [], [], []
for cond, (variant, extra) in ABLATIONS.items():
    upstream = variant.startswith("up_")
    fence = "" if upstream else FENCE  # the turn-back is an EcpsPilot feature
    e1_B += lines(cond if not upstream else "up_B", variant, f"{fence} {EXT} {extra}".strip(), E1_EPISODES, PATROL)
    if cond not in ("no_retreat", "no_retreat_cap", "no_lcv", "no_baro_rise"):  # nothing under the drone on the walls
        e2_B += lines(cond if not upstream else "up_B", variant, f"{EXT} {extra}".strip(), E2_EPISODES, APPROACH)
e1_B += lines("no_fence_turn", "v4", EXT, E1_EPISODES, PATROL)
e1_noB += lines("no_routeB", "v4", FENCE, E1_EPISODES, PATROL)
e1_noB += lines("up", "up_v0", "", E1_EPISODES, PATROL)
e1_gps += lines("gps_ref", "v4", FENCE, E1_EPISODES, PATROL)
e2_noB += lines("up", "up_v0", "", E2_EPISODES, APPROACH)
# E3: faults 30 s into the patrol, each with and without the safety feature that handles it
e3_eps = E1_EPISODES[:10]
F = f"{FENCE} {EXT}"
e3_B += lines("cam_freeze", "v4", f"{F} --freeze-cam-at 30", e3_eps, PATROL)
e3_B += lines(
    "cam_freeze_no_wd",
    "v4",
    f"{F} --freeze-cam-at 30 {s('ecps_pilot.stale_s=999', 'ecps_pilot.land_after_s=999')}",
    e3_eps,
    PATROL,
)
e3_B += lines("hdrop", "v4", f"{F} --ext-drop-at 30", e3_eps, PATROL)  # height module stops, brain alive
e3_B += lines("hdrop_no_wd", "v4", f"{F} --ext-drop-at 30 {s('ecps_pilot.height_source.enabled=false')}", e3_eps, PATROL)
e3_B += lines("hdrop10", "v4", f"{F} --ext-drop-at 30 --ext-drop-for 10", e3_eps, PATROL)
e3_B += lines("hang", "v4", f"{F} --mute-at 30", e3_eps, PATROL)  # whole companion hangs
e3_noFS += lines("hang_no_fs", "v4", f"{F} --mute-at 30", e3_eps, PATROL)

G, FS = "$V/G_extnav_height.parm", "$V/companion_fs.parm"
BATCHES = (  # E3 first: short, and it checks the S7b safety fixes before the long E1/E2 batches
    ("e3_B", e3_B, "flow", f"{G},{FS}"),
    ("e3_noFS", e3_noFS, "flow", G),
    ("e1_B", e1_B, "flow", f"{G},{FS}"),
    ("e1_noB", e1_noB, "flow", FS),
    ("e1_gps", e1_gps, "gps", FS),
    ("e2_B", e2_B, "flow", f"{G},{FS}"),
    ("e2_noB", e2_noB, "flow", FS),
)
tsv = []
for name, ls, nav, extra in BATCHES:
    (out / f"{name}.txt").write_text("\n".join(ls) + "\n")
    tsv.append(f"{name}\t{nav}\t{extra}")
    print(f"{name}: {len(ls)} runs ({nav}, {extra})")
(out / "batches.tsv").write_text("\n".join(tsv) + "\n")
print("total", sum(len(b[1]) for b in BATCHES))
