#!/usr/bin/env python3
"""For each correction event in comp.csv: true distance from the hover target (field origin) just before and 5 s after."""
import csv, glob, math, sys
from pymavlink import mavutil
rd = sys.argv[1]
m = mavutil.mavlink_connection(sorted(glob.glob(rd + '/logs/*.BIN'))[-1])
sim, att = [], []
while True:
    x = m.recv_match(type=['SIM2', 'SIM', 'ARM'])
    if x is None:
        break
    if x.get_type() == 'SIM2':
        sim.append((x.TimeUS / 1e6, x.PN, x.PE))
    elif x.get_type() == 'SIM':
        att.append((x.TimeUS / 1e6, x.Yaw))
rows = list(csv.DictReader(open(rd + '/comp.csv')))
t0 = next(float(r['t']) for r in rows if r['ev'].startswith('obs'))
o = min(sim, key=lambda p: abs(p[0] - 30))  # on the ground before take-off
def dist(t):
    p = min(sim, key=lambda q: abs(q[0] - t))
    return math.hypot(p[1] - o[1], p[2] - o[2])
last = -1e9
for r in rows:
    t = float(r['t'])
    if r['ev'] == 'reset':
        print('reset %.1f  |corr| %.2f  dist before %.2f  after 5 s %.2f' % (t, math.hypot(float(r['mx']), float(r['my'])), dist(t - 0.1), dist(t + 5)))
    if r['ev'] == 'sent':
        if t - last > 2:
            print('extnav reacq %.1f  dist before %.2f  after 5 s %.2f' % (t, dist(t - 0.1), dist(t + 5)))
        last = t
