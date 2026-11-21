#!/usr/bin/env python3
"""failsafe_report.py: per-run arm result + STATUSTEXT lines of interest (C and D groups)."""
import glob
import json
import os
import re

D = '/home/jacky/sim/val1'
KEY = re.compile(r'arm|fail|Fence|fence|VisOdom|EKF|Land|LAND|land|RTL|RTL|GCS|Radio|battery|Batt|position|not healthy', re.I)
for d in sorted(glob.glob(D + '/runs/*/')):
    tag = os.path.basename(d.rstrip('/'))
    if not re.match(r'^[cde]\d?_', tag):
        continue
    p = os.path.join(d, 'run.log')
    if not os.path.exists(p):
        print('%s: no run.log' % tag)
        continue
    txt = open(p, errors='replace').read()
    m = re.search(r'RESULT_JSON (.*)', txt)
    sc = json.loads(m.group(1)) if m else {}
    lo = sc.get('loiter_armed')
    au = sc.get('auto_armed')
    fm = (sc.get('auto') or {}).get('final_mode') or (sc.get('auto') or {}).get('modes')
    lines = []
    for line in txt.splitlines():
        if 'FC text:' in line and KEY.search(line):
            lines.append(line.split('FC text:')[1].strip())
    seen = []
    for x in lines:
        if x not in seen:
            seen.append(x)
    print('%-20s loiter_armed=%s auto_armed=%s final=%s' % (tag, lo, au, fm))
    for x in seen[:14]:
        print('      %s' % x)
