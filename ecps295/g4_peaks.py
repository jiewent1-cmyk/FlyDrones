"""Peak looming-DN rates and closest approach in a run_g4.py probe CSV."""

import csv
import sys

BOX_FACE_N, CAM_X = 1.25, 0.087  # camtest box_a near face (north, m); camera ahead of body origin
for path in sys.argv[1:]:
    rows = list(csv.DictReader(open(path)))
    seg = [r for r in rows if r["phase"] == "approach"]
    i0 = rows.index(seg[0])
    win = rows[i0 : i0 + len(seg) + 20]  # approach + 1 s after

    def pk(k, win=win):
        return max(float(r[k]) for r in win)

    closest = BOX_FACE_N - CAM_X - max(float(r["north"]) for r in win)
    print(
        f"{path}: peak DNp03 L/R {pk('DNp03_L'):.0f}/{pk('DNp03_R'):.0f} Hz, DNp01 L/R {pk('DNp01_L'):.0f}/{pk('DNp01_R'):.0f} Hz, "
        f"closest camera-to-box {closest:.2f} m, escape ticks {sum(int(r['escape']) for r in win)}"
    )
    for k in ("DNp03_R", "DNp01_R"):
        first = next((r for r in win if float(r[k]) >= 20), None)
        if first:
            print(f"   {k} first >= 20 Hz at camera-to-box {BOX_FACE_N - CAM_X - float(first['north']):.2f} m (t={first['t']} s)")
    esc = next((r for r in win if r["escape"] == "1"), None)
    if esc:
        print(f"   escape onset at camera-to-box {BOX_FACE_N - CAM_X - float(esc['north']):.2f} m (t={esc['t']} s)")
