"""Print a run_g4.py CSV around an obstacle approach: clearance, speed command, looming neurons, escape."""

import csv
import sys

path, below = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 1.3
step = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
last = None
for r in csv.DictReader(open(path)):
    c, t = float(r["clearance"]), float(r["t"])
    if c < below and (last is None or t - last >= step):
        last = t
        print(
            f"t={t:5.1f} clr={c:6.3f} N={float(r['north']):5.2f} E={float(r['east']):+5.2f} alt={float(r['alt']):4.2f} "
            f"fwd={float(r['cmd_forward']):+.2f} thr={float(r['cmd_throttle']):+.2f} yaw={float(r['cmd_yaw']):+.2f} "
            f"DNp03 {float(r['DNp03_L']):3.0f}/{float(r['DNp03_R']):3.0f} DNp01 {float(r['DNp01_L']):3.0f}/{float(r['DNp01_R']):3.0f} "
            f"esc={r['escape']}"
        )
