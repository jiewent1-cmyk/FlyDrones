#!/usr/bin/env python3
"""summ.py [group-prefix ...]: one line per run with the key metrics."""
import glob
import json
import os
import re
import sys

pats = sys.argv[1:] or ['a_', 'b_', 'c', 'd', 'e']
cols = ['mission', 'mission_complete', 'final_mode', 'alt_max', 'track_rms', 'alt_leg_rms',
        'alt_err_max', 'hov_alt_mean', 'hov_alt_min', 'hov_alt_max', 'hov_drift_max',
        'hov_rad_max', 'left_auto_at', 'touchdown_mps', 'land_offset', 'f_true_max',
        'f_true_min', 'f_dev_max', 'f_t0', 'f_dur', 'climb_overshoot', 'pos_est_err_max']
for d in sorted(glob.glob('/home/jacky/sim/val1/runs/*/metrics.json')):
    tag = os.path.basename(os.path.dirname(d))
    if not any(tag.startswith(p) for p in pats):
        continue
    try:
        r = json.load(open(d))
    except Exception as e:
        print(tag, 'BADJSON', e)
        continue
    if 'error' in r:
        print('%-20s ERROR %s' % (tag, r['error']))
        continue
    out = []
    for c in cols:
        v = r.get(c)
        if v is None:
            continue
        out.append('%s=%s' % (c, v))
    print('%-20s %s' % (tag, ' '.join(out)))
