#!/usr/bin/env python3
"""Compare the truth-computed optical flow (truth.csv of val3_fly.py) with the flow the
flight controller logged (dataflash OF) in the same run.

usage: cmp_flow.py <run_dir>

The two timelines differ by a constant offset (dataflash TimeUS is the FC clock, truth.csv
is the script wall clock); the offset is found by minimising the RMS of the difference.
"""
import csv
import math
import os
import sys

from pymavlink import mavutil

rd = sys.argv[1]
truth = []
with open(os.path.join(rd, 'truth.csv')) as f:
    for row in csv.DictReader(f):
        truth.append((float(row['t']), float(row['flow_x']), float(row['flow_y']),
                      float(row['height_agl']), float(row['dist_m'])))
of = []
ld = os.path.join(rd, 'logs')
for fn in sorted(os.listdir(ld)):
    if not fn.endswith('.BIN'):
        continue
    m = mavutil.mavlink_connection(os.path.join(ld, fn))
    while True:
        x = m.recv_match(type=['OF'])
        if x is None:
            break
        of.append((x.TimeUS / 1e6, x.flowX, x.flowY, x.Qual))
print('truth samples', len(truth), 'of samples', len(of))


def rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else None


def align(off):
    # truth samples that fall inside the OF window
    t0, t1 = of[0][0] + off, of[-1][0] + off
    sel = [r for r in truth if t0 <= r[0] <= t1]
    # pair by nearest OF sample
    dx, dy, n = [], [], 0
    for r in sel:
        # nearest OF sample
        best = min(of, key=lambda o: abs(o[0] + off - r[0]))
        if abs(best[0] + off - r[0]) > 0.05:
            continue
        dx.append(best[1] - r[1])
        dy.append(best[2] - r[2])
        n += 1
    return n, rms(dx), rms(dy)


best = None
for i in range(-400, 401):
    off = i * 0.05
    n, rx, ry = align(off)
    if n < 10:
        continue
    score = (rx or 9e9) + (ry or 9e9)
    if best is None or score < best[0]:
        best = (score, off, n, rx, ry)
print('best offset %.2f s  n=%d  rms_flowX=%.5f  rms_flowY=%.5f' % (best[1], best[2], best[3], best[4]))
n, rx, ry = align(best[1])
print('paired samples', n, 'rms_dx', rx, 'rms_dy', ry)
sel = [r for r in truth if of[0][0] + best[1] <= r[0] <= of[-1][0] + best[1]]
print('truth flow_x range %.4f..%.4f  flow_y range %.4f..%.4f' % (
    min(r[1] for r in sel), max(r[1] for r in sel), min(r[2] for r in sel), max(r[2] for r in sel)))
print('of     flowX range %.4f..%.4f  flowY range %.4f..%.4f  qual set %d' % (
    min(o[1] for o in of), max(o[1] for o in of), min(o[2] for o in of), max(o[2] for o in of),
    sum(1 for o in of if o[3])))
print('truth height_agl range %.3f..%.3f  dist_m range %.3f..%.3f' % (
    min(r[3] for r in sel), max(r[3] for r in sel), min(r[4] for r in sel), max(r[4] for r in sel)))
for r in sel[::max(1, len(sel) // 12)]:
    print('  t=%.2f truth fx=%+.4f fy=%+.4f agl=%.2f dist=%.2f' % (r[0], r[1], r[2], r[3], r[4]))
