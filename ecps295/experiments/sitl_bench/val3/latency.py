#!/usr/bin/env python3
"""Measured message latency of one part-2 run.

The feeder logs every DISTANCE_SENSOR it sends (truth.csv, column dist_m) and the flight
controller streams its own DISTANCE_SENSOR report back (run.json -> fc_distance_sensor).
Pairing a report with the send that produced the same value gives
    delay = t_report - t_send
The FC streams at 5 Hz, so the observed delay contains up to one streaming period
(0.2 s); the minimum over the run is the tightest observed value.

usage: latency.py <run_dir>
"""
import csv
import json
import os
import sys

rd = sys.argv[1]
rows = list(csv.DictReader(open(os.path.join(rd, 'truth.csv'))))
run = json.load(open(os.path.join(rd, 'run.json')))
rep = run.get('fc_distance_sensor', [])
sends = [(float(r['t']), int(round(float(r['dist_m']) * 100))) for r in rows]
print('run', os.path.basename(rd), 'sends', len(sends), 'fc reports', len(rep))
delays = []
for t_rep, cm, orient in rep:
    cand = [(t_rep - t_s, t_s) for t_s, c in sends if c == cm and t_s <= t_rep]
    if cand:
        delays.append(min(cand)[0])
if not delays:
    print('no pairable report')
    sys.exit(0)
delays.sort()
n = len(delays)
print('paired %d reports' % n)
print('delay min %.3f s  p10 %.3f s  median %.3f s  max %.3f s' % (
    delays[0], delays[int(0.1 * n)], delays[n // 2], delays[-1]))
