"""Offline check of EcpsRetina's `blank` feature on recorded runs (run_g4.py --record-frames + its CSV).

Prints, per run, the blank level / textureless share per eye against obstacle clearance, so thresholds can be tuned
on real Gazebo frames before any closed-loop flight:

    python blank_eval.py --config minifly/v2.yaml runs/rec_plain:plain.csv runs/rec_tape:tape.csv ...
"""

import argparse
import csv
import glob
import os

import cv2
import numpy as np
from ecps_retina import EcpsRetina

from flydrones.config import load_config

ap = argparse.ArgumentParser()
ap.add_argument("runs", nargs="+", help="dir:csv pairs (frames in dir/frames)")
ap.add_argument("--config", default="")
ap.add_argument("--every", type=float, default=0.5, help="print interval [s]")
a = ap.parse_args()
cfg = load_config(a.config or None)

for spec in a.runs:
    d, c = spec.split(":")
    rows = list(csv.DictReader(open(os.path.join(d, c))))
    files = sorted(glob.glob(os.path.join(d, "frames", "*.jpg")))
    ret = EcpsRetina.from_config(cfg)
    print(f"== {d}  ({len(files)} frames)")
    levels, last = [], -1e9
    for i, f in enumerate(files[: len(rows)]):
        frame = cv2.resize(cv2.imread(f, cv2.IMREAD_REDUCED_GRAYSCALE_2), (192, 144), interpolation=cv2.INTER_AREA)
        ret.encode(frame)
        b = ret.last_blank
        r = rows[i]
        levels.append((float(r["clearance"]), b["L"], b["R"], b["frac_L"], b["frac_R"]))
        t = float(r["t"])
        if t - last >= a.every:
            last = t
            print(
                f"t={t:5.1f} clr={float(r['clearance']):6.2f} alt={float(r['alt']):.2f} fwd={float(r['cmd_forward']):+.2f} "
                f"blank L/R {b['L']:.2f}/{b['R']:.2f}  frac L/R {b['frac_L']:.2f}/{b['frac_R']:.2f}  side_L {b['side_L']:.2f}"
            )
    arr = np.array(levels)
    far = arr[arr[:, 0] > 1.2]
    near = arr[(arr[:, 0] < 0.8) & (arr[:, 0] > 0)]
    if far.size:
        print(f"   clearance > 1.2 m: max blank {far[:, 1:3].max():.2f}, mean frac {far[:, 3:5].mean():.2f}")
    if near.size:
        print(f"   0 < clearance < 0.8 m: max blank {near[:, 1:3].max():.2f}, mean frac {near[:, 3:5].mean():.2f}")
