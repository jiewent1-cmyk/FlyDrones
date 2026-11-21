#!/usr/bin/env python3
"""Compare the truth-computed optical flow (truth.csv of val3_fly.py) with the flow the
flight controller logged (dataflash OF) in the same run.

usage: cmp_flow2.py <run_dir>

Fast version of cmp_flow.py: nearest-neighbour lookup via bisect.
The two timelines differ by a constant offset (dataflash TimeUS is the FC clock, truth.csv
is the script wall clock); the offset is found by minimising the RMS of the difference.
"""
import bisect
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
of.sort()
of_t = [o[0] for o in of]
print('truth samples', len(truth), 'of samples', len(of))
if not of or not truth:
    sys.exit(0)


def nearest(t):
    """index of nearest OF sample to time t"""
    i = bisect.bisect_left(of_t, t)
    if i <= 0:
        return 0
    if i >= len(of_t):
        return len(of_t) - 1
    return i if of_t[i] - t < t - of_t[i - 1] else i - 1


def rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else None


def align(off):
    t0, t1 = of_t[0] + off, of_t[-1] + off
    dx, dy = [], []
    for r in truth:
        if not (t0 <= r[0] <= t1):
            continue
        j = nearest(r[0] + off)
        if abs(of_t[j] + off - r[0]) > 0.05:
            continue
        dx.append(of[j][1] - r[1])
        dy.append(of[j][2] - r[2])
    return len(dx), rms(dx), rms(dy)


best = None
for i in range(-400, 401):
    off = i * 0.05
    n, rx, ry = align(off)
    if n < 10:
        continue
    score = (rx or 9e9) + (ry or 9e9)
    if best is None or score < best[0]:
        best = (score, off, n, rx, ry)
if best is None:
    print('no alignment found')
    sys.exit(0)
print('best offset %.2f s  n=%d  rms_flowX=%.5f  rms_flowY=%.5f' % (best[1], best[2], best[3], best[4]))
off = best[1]
t0, t1 = of_t[0] + off, of_t[-1] + off
sel = [r for r in truth if t0 <= r[0] <= t1]
print('truth flow_x range %.4f..%.4f  flow_y range %.4f..%.4f' % (
    min(r[1] for r in sel), max(r[1] for r in sel), min(r[2] for r in sel), max(r[2] for r in sel)))
print('of     flowX range %.4f..%.4f  flowY range %.4f..%.4f  qual>0 count %d' % (
    min(o[1] for o in of), max(o[1] for o in of), min(o[2] for o in of), max(o[2] for o in of),
    sum(1 for o in of if o[3])))
print('truth height_agl range %.3f..%.3f  dist_m range %.3f..%.3f' % (
    min(r[3] for r in sel), max(r[3] for r in sel), min(r[4] for r in sel), max(r[4] for r in sel)))
step = max(1, len(sel) // 12)
for r in sel[::step]:
    j = nearest(r[0] + off)
    print('  t=%.2f truth fx=%+.4f fy=%+.4f | of fx=%+.4f fy=%+.4f | agl=%.2f dist=%.2f' % (
        r[0], r[1], r[2], of[j][1], of[j][2], r[3], r[4]))
