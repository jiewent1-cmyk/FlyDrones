"""D stage Gazebo worlds: out-of-distribution copies of an existing world, matching rl.twin OOD_MODES.

  notex  boxes one flat colour (emissive, no sun shading), no tape bands, plain mid-grey mat, no floor strips
  ghost  boxes keep their collision but lose every visual (garyb9 ghost check)
Nets, cage frame, ground and scenery are untouched. Writes ecps295_<world>_<mode>.sdf / .layout.json next to the source.

python gazebo/ood_world.py --mode notex ghost --worlds lowbox_s0 ... cage20
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

W = Path(__file__).resolve().parent / "worlds"
KEEP = re.compile(r"(scenery|net_\w+|ground_plane|floor_tape|eva_mat|cage_frame)$")
MODEL = re.compile(r'<model name="(\w+)">.*?</model>', re.S)
VISUAL = re.compile(r'\s*<visual name="(\w+)">.*?</visual>', re.S)
MAT = re.compile(r"<material>.*?</material>", re.S)
MAT_GREY = (0.30 + 0.55) / 2  # rl.twin notex: mat at the mean of the two tile shades


def _box(m: str, mode: str) -> str:
    if mode == "ghost":
        return VISUAL.sub("", m)

    def vis(v: re.Match) -> str:
        if v.group(1).startswith("tape"):
            return ""
        c = re.search(r"<diffuse>([\d.]+ [\d.]+ [\d.]+)", v.group(0)).group(1)
        # emissive only: the same radiance on every face whatever the sun direction
        return MAT.sub(f"<material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><emissive>{c} 1</emissive></material>", v.group(0))

    return VISUAL.sub(vis, m)


def convert(world: str, mode: str) -> str:
    src = W / f"ecps295_{world}.sdf"
    name = f"ecps295_{world}_{mode}"
    s = src.read_text()

    def model(m: re.Match) -> str:
        n, body = m.group(1), m.group(0)
        if not KEEP.match(n):
            return _box(body, mode)
        if mode == "notex" and n == "floor_tape":
            return VISUAL.sub("", body)
        if mode == "notex" and n == "eva_mat":
            g = f"{MAT_GREY:.3f} {MAT_GREY:.3f} {MAT_GREY:.3f} 1"
            return MAT.sub(f"<material><ambient>{g}</ambient><diffuse>{g}</diffuse></material>", body)
        return body

    out, n = MODEL.subn(model, s)
    assert n == s.count("<model name="), (world, n)
    out, k = re.subn(r'<world name="[^"]+">', f'<world name="{name}">', out)
    assert k == 1
    (W / f"{name}.sdf").write_text(out)
    lay = json.loads((W / f"ecps295_{world}.layout.json").read_text())
    lay["world"], lay["ood"] = name, mode
    (W / f"{name}.layout.json").write_text(json.dumps(lay, indent=1))
    return name


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", nargs="+", choices=["notex", "ghost"], required=True)
    ap.add_argument("--worlds", nargs="+", required=True)
    a = ap.parse_args()
    for m in a.mode:
        for w in a.worlds:
            print(convert(w, m))
