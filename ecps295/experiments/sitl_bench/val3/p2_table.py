#!/usr/bin/env python3
"""Collect the part-2 matrix metrics into a markdown table."""
import json
import os

BASE = '/home/jacky/sim/val3/runs'
rows = []
for tag in sorted(os.listdir(BASE)):
    parts = tag.split('_')
    if len(parts) != 3 or parts[0] != 'p2' or parts[1] not in ('HOV', 'SQ') or parts[2] not in ('mav', 'nat'):
        continue
    mj = os.path.join(BASE, tag, 'metrics.json')
    if not os.path.exists(mj):
        continue
    j = json.load(open(mj))
    s = j.get('script') or {}
    a = s.get('auto') or {}
    rows.append((tag, j, s, a))

hdr = ['run', 'mission_complete', 'armed', 'flew', 'items', 'alt_max', 'radius_max', 'hgt_est_rms',
       'hgt_est_maxabs', 'pos_est_err_max', 'land_offset', 'auto_seconds', 'track_rms', 'alt_rms']
print('| ' + ' | '.join(hdr) + ' |')
print('|' + '---|' * len(hdr))
for tag, j, s, a in rows:
    v = [tag, j.get('mission_complete'), s.get('auto_armed'), a.get('flew'),
         a.get('items_reached'), j.get('alt_max'), j.get('radius_max'), j.get('hgt_est_rms'),
         j.get('hgt_est_maxabs'), j.get('pos_est_err_max'), j.get('land_offset'), j.get('auto_seconds'),
         j.get('track_rms'), j.get('alt_rms')]
    print('| ' + ' | '.join(str(x) for x in v) + ' |')
