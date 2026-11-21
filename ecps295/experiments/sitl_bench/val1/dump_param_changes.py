#!/usr/bin/env python3
"""dump_param_changes.py <run dir> [NAME ...]: PARM records with time (sim seconds from log start)."""
import os
import sys

from pymavlink import mavutil

rd = sys.argv[1]
names = sys.argv[2:]
ld = os.path.join(rd, 'logs')
for f in sorted(os.listdir(ld)):
    if not f.endswith('.BIN'):
        continue
    m = mavutil.mavlink_connection(os.path.join(ld, f))
    t0 = None
    while True:
        x = m.recv_match(type=['PARM'])
        if x is None:
            break
        if t0 is None:
            t0 = x.TimeUS
        if names and x.Name not in names:
            continue
        print('%8.2f %-24s %s' % ((x.TimeUS - t0) / 1e6, x.Name, x.Value))
