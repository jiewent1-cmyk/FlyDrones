#!/usr/bin/env python3
"""dump_params.py <run dir>: print all PARM records from the dataflash log."""
import os
import sys

from pymavlink import mavutil

rd = sys.argv[1]
ld = os.path.join(rd, 'logs')
for f in sorted(os.listdir(ld)):
    if not f.endswith('.BIN'):
        continue
    m = mavutil.mavlink_connection(os.path.join(ld, f))
    while True:
        x = m.recv_match(type=['PARM'])
        if x is None:
            break
        print('%-24s %s' % (x.Name, x.Value))
