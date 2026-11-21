#!/usr/bin/env python3
"""report_tables.py: markdown tables from runs/*/metrics.json (group = tag without _rN)."""
import glob
import json
import os
import re
import statistics as stt

D = '/home/jacky/sim/val1'
rows = {}
for f in sorted(glob.glob(D + '/runs/*/metrics.json')):
    tag = os.path.basename(os.path.dirname(f))
    grp = re.sub(r'_r\d+$', '', tag)
    try:
        r = json.load(open(f))
    except Exception:
        r = {'error': 'bad json'}
    r['tag'] = tag
    rows.setdefault(grp, []).append(r)

ORDER = ['a_d3', 'a_hov2', 'a_sq', 'a_hov1', 'b_d3', 'b_hov2', 'b_sq', 'b_hov1',
         'c1_viso0_novis', 'c2_viso1_novis', 'c3_viso1_vis', 'c4_viso0_noorigin',
         'd1_hb', 'd2_rcfail', 'd3_batt', 'd4_son_dn', 'd5_son_up',
         'e1_fence', 'e2_r35', 'e3_alt23']
COLS = {'d3': ['track_rms', 'alt_leg_rms', 'alt_err_max', 'alt_max'],
        'sq': ['track_rms', 'alt_leg_rms', 'alt_err_max', 'land_offset'],
        'hov2': ['hov_alt_mean', 'hov_alt_min', 'hov_alt_max', 'hov_drift_max'],
        'hov1': ['hov_alt_mean', 'hov_alt_min', 'hov_alt_max', 'hov_drift_max'],
        'hb': ['final_mode', 'left_auto_at', 'touchdown_mps', 'land_offset'],
        'rcfail': ['final_mode', 'left_auto_at', 'touchdown_mps', 'land_offset'],
        'batt': ['final_mode', 'left_auto_at', 'touchdown_mps', 'land_offset'],
        'son_dn': ['f_true_min', 'f_true_max', 'f_dev_max', 'mission_complete'],
        'son_up': ['f_true_min', 'f_true_max', 'f_dev_max', 'mission_complete'],
        'fence': ['rad_max', 'alt_max', 'mission_complete', 'track_rms'],
        'r35': ['rad_max', 'alt_max', 'mission_complete', 'track_rms'],
        'alt23': ['rad_max', 'alt_max', 'mission_complete', 'track_rms'],
        'novis': ['final_mode', 'mission_complete', 'alt_max', 'track_rms'],
        'vis': ['final_mode', 'mission_complete', 'alt_max', 'track_rms'],
        'noorigin': ['final_mode', 'mission_complete', 'alt_max', 'track_rms']}


def med(rs, c):
    v = [r[c] for r in rs if isinstance(r.get(c), (int, float)) and not isinstance(r.get(c), bool)]
    if not v:
        return '-'
    if len(v) == 1:
        return '%.3f' % v[0]
    return '%.3f [%.3f-%.3f]' % (stt.median(v), min(v), max(v))


def ok(rs):
    return sum(1 for r in rs if r.get('mission_complete'))


def cols_for(g):
    for k, v in COLS.items():
        if k in g:
            return v
    return ['final_mode', 'mission_complete', 'alt_max', 'track_rms']


print('| group | n | complete | m1 | m2 | m3 | m4 | bad |')
print('|---|---|---|---|---|---|---|---|')
for g in ORDER:
    rs = rows.get(g)
    if not rs:
        continue
    cols = cols_for(g)
    bad = ','.join(r['tag'] for r in rs if 'error' in r)
    print('| %s | %d | %d/%d | %s | %s |' % (g, len(rs), ok(rs), len(rs),
          ' | '.join(med(rs, c) for c in cols), bad if bad else '-'))
